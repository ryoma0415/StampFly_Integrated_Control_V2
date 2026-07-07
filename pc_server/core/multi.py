"""マルチ機体同時制御(MODE_MULTI)。

DroneSlot(機体1機ぶんの実行状態)と MultiControlManager(2〜4機の選択・
機体別目標・一斉開始/停止・フェイルセーフ監視)を提供する。SessionManager が
唯一のインスタンスを保持し、UI コマンド(multi_select / multi_target /
multi_start / stop)と 20Hz supervisor から呼ばれる。

設計方針:
- 機体ファーム無改修。宛先分けはリレーの RLY_SET_PEERS + RLY_MUX_UP/DOWN
  (serial.send_to / register_node_handler)で行い、ESP-NOW 区間のバイト列は
  単機時と同一。node_id = 選択順 index = RLY_SET_PEERS のエントリ index。
- 各スロットは独立した PositionController(自前の 50Hz 送信スレッド)を持ち、
  CMD_SETPOINT はノード宛で送る(ハートビート兼用 — 機体側 200ms/500ms の
  自律フェイルセーフが最終防衛線)。
- MoCap は1つの NatNet クライアントを共有し、機体プロファイルの
  rigid_body_id で MocapSource.subscribe() する。
- PC 側フェイルセーフは単機セッションと同じ規範をスロットごとに適用する:
  STOP 再送(600ms×3、LANDING/WAIT イベントで解除)/ MoCap 途絶
  (>300ms 警告・水平固定は PositionController 内蔵、>2s で当該機へ CMD_STOP)
  / START 猶予(離陸しないまま grace 経過で armed 解除)
  / XY 誤差発散(飛行中の閉ループで divergence_error_m 超過が
  divergence_hold_s 継続したら当該機へ CMD_STOP — rigid_body_id 取り違えで
  他機の位置とループを閉じる交差結合の検知)。
- v1 スコープ: 静的目標のみ(円軌道なし)・ヨー角制御 OFF・CSV ログなし。
"""

from __future__ import annotations

import threading
from typing import Callable, Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由

from . import config as cfg
from .mocap import DEG_TO_RAD, RAD_TO_DEG, MocapSource
from .position import PositionController
from .serial_link import SerialLink, SerialLinkError


def _state_name(state: int) -> str:
    try:
        return proto.FlightState(state).name
    except ValueError:
        return f"UNKNOWN({state})"


def _reason_name(reason: int) -> str:
    try:
        return proto.Reason(reason).name
    except ValueError:
        return f"UNKNOWN({reason})"

# フェーズ判定用の状態集合(session.py の _IN_FLIGHT_STATES / _ON_GROUND_STATES
# と同一定義。循環 import を避けるためここで proto から導出する)
_IN_FLIGHT_STATES = frozenset({
    proto.FlightState.TAKEOFF, proto.FlightState.HOVER,
    proto.FlightState.LANDING,
})
_ON_GROUND_STATES = frozenset({
    proto.FlightState.INIT, proto.FlightState.CALIBRATION,
    proto.FlightState.WAIT, proto.FlightState.COMPLETE,
})
_START_REJECT_REASONS = frozenset({
    proto.Reason.START_REJECTED_LOW_VOLTAGE,
    proto.Reason.START_REJECTED_NOT_READY,
})

# スロットフェーズ(単機セッションの armed/flying と同じ意味論。idle は
# 「開始前/着陸済み」で、セッション全体の phase とは独立に機体ごとに持つ)
SLOT_IDLE = "idle"
SLOT_ARMED = "armed"
SLOT_FLYING = "flying"


class DroneSlot:
    """機体1機ぶんの実行状態(MultiControlManager._lock で保護)。"""

    def __init__(self, node_id: int, profile: dict,
                 controller: PositionController) -> None:
        self.node_id = node_id
        self.profile = profile
        self.name: str = profile["name"]
        self.mac: str = profile["mac"]
        self.rigid_body_id: int = int(profile["rigid_body_id"])
        self.bias_roll_rad: float = profile["roll_bias_deg"] * DEG_TO_RAD
        self.bias_pitch_rad: float = profile["pitch_bias_deg"] * DEG_TO_RAD
        self.controller = controller

        self.phase = SLOT_IDLE
        self.armed_since: Optional[float] = None
        # STOP 再送管理: None または {"deadline": t, "resends": n}
        self.stop_pending: Optional[dict] = None
        self.mocap_warned = False
        self.mocap_stop_sent = False
        # XY 誤差が divergence_error_m を超え続けている開始時刻(発散検知)
        self.error_high_since: Optional[float] = None
        self.target_set = False
        # 最新テレメトリ(ノード帰属済み)
        self.tlm: Optional[proto.TlmState] = None
        self.tlm_t: Optional[float] = None


class MultiControlManager:
    """2〜4機の同時位置制御(選択・目標・一斉開始/停止・監視)。"""

    def __init__(self, server_config: dict, control_config: dict,
                 serial: SerialLink, mocap: MocapSource,
                 notify_info: Callable[[str], None],
                 notify_warn: Callable[[str], None],
                 events_put: Callable[[dict], None],
                 clock: Callable[[], float]) -> None:
        self._server_config = server_config
        self._control_config = control_config
        self._serial = serial
        self._mocap = mocap
        self._info = notify_info
        self._warn = notify_warn
        self._events_put = events_put
        self._clock = clock

        multi_cfg = server_config["multi"]
        self._min_drones: int = multi_cfg["min_drones"]
        self._max_drones: int = min(int(multi_cfg["max_drones"]),
                                    proto.RLY_MAX_PEERS)
        self._tlm_state_div: int = multi_cfg["tlm_state_div"]
        self._target_xy_abs_max_m: float = multi_cfg["target_xy_abs_max_m"]
        self._min_separation_m: float = multi_cfg["min_target_separation_m"]

        failsafe = server_config["failsafe"]
        self._stop_ack_timeout_s: float = failsafe["stop_ack_timeout_s"]
        self._stop_max_retries: int = failsafe["stop_max_retries"]
        self._start_grace_s: float = failsafe["start_grace_s"]
        self._mocap_dropout_level_s: float = failsafe["mocap_dropout_level_s"]
        self._mocap_dropout_stop_s: float = failsafe["mocap_dropout_stop_s"]
        self._telemetry_fresh_s: float = \
            server_config["freshness"]["telemetry_fresh_s"]
        # armed/flying スロットの TLM 途絶ハードタイムアウト(リレー再起動・
        # ピア表喪失の検出。機体側は 500ms 途絶で自律着陸済みのはずなので、
        # PC 側はスロットを安全側(idle)へ戻して操作可能な状態を回復する)
        self._tlm_timeout_s: float = multi_cfg["tlm_timeout_s"]
        # 一斉開始時の「地上にいること」の検証(RB ID 取り違え対策の一部)
        self._start_ground_z_max_m: float = multi_cfg["start_ground_z_max_m"]
        # 飛行中の XY 誤差の持続的発散の検知(rigid_body_id 取り違えで他機の
        # 位置とループを閉じる交差結合の検出。地上高チェックをすり抜けた
        # 取り違えの最終防衛線)
        self._divergence_error_m: float = multi_cfg["divergence_error_m"]
        self._divergence_hold_s: float = multi_cfg["divergence_hold_s"]

        # スロット表(_lock 保護。RX スレッドの TLM ハンドラと UI コマンド、
        # supervisor が競合する)
        self._lock = threading.Lock()
        self._slots: list[DroneSlot] = []
        self._active = False

        # ノード付き受信ハンドラ(RLY_MUX_DOWN の内側フレーム)
        serial.register_node_handler(proto.MsgType.TLM_STATE,
                                     self._on_tlm_state)
        serial.register_node_handler(proto.MsgType.TLM_EVENT,
                                     self._on_tlm_event)

    # ==================================================================
    # 選択 / 解除
    # ==================================================================

    def select(self, names: list[str], airframes: list[dict]
               ) -> tuple[bool, str]:
        """機体プロファイル名の並びを検証し、リレーへ SET_PEERS して
        スロット(node_id = 並び順)を構築する。

        接続中のみ可。既に選択済みなら選び直し(全スロット再構築)。
        """
        if not self._serial.is_connected:
            return False, "未接続のため機体を選択できません"
        if self.any_armed_or_flying():
            return False, "飛行中は機体選択を変更できません"
        if not (self._min_drones <= len(names) <= self._max_drones):
            return False, (f"機体は {self._min_drones}〜{self._max_drones} 機を"
                           f"選択してください({len(names)} 機指定)")
        if len(set(names)) != len(names):
            return False, "同じ機体が重複して選択されています"

        profiles: list[dict] = []
        for name in names:
            profile = next((p for p in airframes if p["name"] == name), None)
            if profile is None:
                return False, f"機体プロファイルが見つかりません: {name}"
            if not cfg.mac_is_set(profile.get("mac")):
                return False, f"機体「{name}」は MAC が未設定です"
            if not profile.get("rigid_body_id"):
                return False, (f"機体「{name}」は rigid_body_id が未設定です。"
                               "機体プロファイル編集で設定してください"
                               "(「RB確認」で ID を照合できます)")
            profiles.append(profile)

        macs = [p["mac"] for p in profiles]
        if len(set(macs)) != len(macs):
            return False, "MAC が重複しています(同じ機体を指す別プロファイル)"
        rb_ids = [int(p["rigid_body_id"]) for p in profiles]
        if len(set(rb_ids)) != len(rb_ids):
            return False, "rigid_body_id が重複しています"
        channels = {p["wifi_channel"] for p in profiles}
        if len(channels) != 1:
            # リレーの無線は1チャネル(RLY_SET_PEERS 契約)
            return False, ("選択した機体の wifi_channel が一致していません: "
                           + ", ".join(f"{p['name']}=ch{p['wifi_channel']}"
                                       for p in profiles))
        channel = channels.pop()

        # 選び直しに備え、既存スロットを畳んでから設定する
        self.deactivate(clear_peers=False)

        peers = [(cfg.parse_mac(p["mac"]), self._tlm_state_div)
                 for p in profiles]
        try:
            ok, ack = self._serial.set_relay_peers(peers, channel)
        except SerialLinkError as exc:
            return False, f"RLY_SET_PEERS 送信失敗: {exc}"
        if not ok:
            status = ack.status if ack is not None else "no_ack"
            return False, (f"リレーのピア設定に失敗しました(status={status})。"
                           "リレーファームが複数機対応版か確認してください")

        # MoCap: 1つの NatNet クライアントを共有(未起動ならパッシブ起動。
        # primary コールバックは単機 Position モード専用のため触らない)
        if not self._mocap.connected():
            if not self._mocap.start():
                self._warn("NatNet 接続に失敗しました(複数機モード)")

        slots: list[DroneSlot] = []
        for node_id, profile in enumerate(profiles):
            controller = PositionController(
                self._server_config, self._control_config,
                self._make_emit(node_id, profile), clock=self._clock)
            slot = DroneSlot(node_id, profile, controller)
            slots.append(slot)
        with self._lock:
            self._slots = slots
            self._active = True
        for slot in slots:
            self._mocap.subscribe(slot.rigid_body_id,
                                  slot.controller.on_mocap_pose)
            slot.controller.start()   # 50Hz 送信開始(機体の relay 学習も兼ねる)

        self._info("複数機選択: " + ", ".join(
            f"[{s.node_id}] {s.name}(RB{s.rigid_body_id})" for s in slots)
            + f" ch{channel}")
        return True, "選択しました"

    def deactivate(self, clear_peers: bool = True) -> None:
        """全スロットを停止・解放する(モード離脱・切断・選び直し時)。"""
        with self._lock:
            slots = self._slots
            self._slots = []
            self._active = False
        for slot in slots:
            slot.controller.set_control_active(False)
            slot.controller.stop()
            self._mocap.unsubscribe(slot.rigid_body_id)
        if clear_peers and slots and self._serial.is_connected:
            try:
                self._serial.set_relay_peers([], 0)
            except SerialLinkError:
                pass   # 切断済みなら畳むだけでよい

    # ==================================================================
    # 目標 / 開始 / 停止
    # ==================================================================

    def set_target(self, name: str, x: float, y: float, z: float
                   ) -> tuple[bool, str]:
        """機体別の目標位置を設定する(XY は設定クランプ内のみ受理)。

        いずれかのスロットが armed/flying の間は、他機の設定済み目標との
        XY 最小間隔も検証する(開始前は start_all が一括検証するため、
        設定順に依存しないよう緩めておく)。
        """
        limit = self._target_xy_abs_max_m
        if abs(x) > limit or abs(y) > limit:
            return False, (f"目標 XY は ±{limit:.1f}m 以内で指定してください"
                           f"(x={x:.2f}, y={y:.2f})")
        slot = self._slot_by_name(name)
        if slot is None:
            return False, f"機体「{name}」は選択されていません"
        if self.any_armed_or_flying():
            with self._lock:
                others = [s for s in self._slots
                          if s is not slot and s.target_set]
            for other in others:
                ox, oy, _ = other.controller.get_target()
                dist = ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5
                if dist < self._min_separation_m:
                    return False, (
                        f"目標が近すぎます: 「{other.name}」との XY 距離 "
                        f"{dist:.2f}m < {self._min_separation_m:.2f}m")
        slot.controller.set_target(x, y, z)
        with self._lock:
            slot.target_set = True
        return True, "目標を設定しました"

    def start_all(self) -> tuple[bool, str]:
        """選択済み全機の一斉離陸(機体ごとに CMD_START をノード宛送信)。"""
        with self._lock:
            slots = list(self._slots)
            active = self._active
        if not (active and slots):
            return False, "機体が選択されていません"
        if not self._serial.is_connected:
            return False, "未接続です"
        if any(s.phase != SLOT_IDLE for s in slots):
            return False, "既に開始済みの機体があります(停止してから再開)"

        now = self._clock()
        for slot in slots:
            if not slot.target_set:
                return False, f"機体「{slot.name}」の目標位置が未設定です"
            age = slot.controller.mocap_age_s(now)
            if age is None or age > self._mocap_dropout_level_s:
                return False, (f"機体「{slot.name}」の MoCap"
                               f"(RB{slot.rigid_body_id})が新鮮ではありません。"
                               "リジッドボディの追跡状態を確認してください")
            # 離陸前は地上にいるはず(宙にある RB は ID 取り違えや吊り上げの兆候)
            snap = slot.controller.mocap_snapshot(now)
            if snap is not None and abs(snap.get("z") or 0.0) \
                    > self._start_ground_z_max_m:
                return False, (
                    f"機体「{slot.name}」(RB{slot.rigid_body_id})の高度 "
                    f"{snap['z']:.2f}m が地上とみなせません"
                    f"(> {self._start_ground_z_max_m:.2f}m)。"
                    "リジッドボディ ID の対応を「RB確認」で確認してください")

        # 目標同士の最小間隔(XY 平面)を検証する(空間共有の安全策)
        targets = [(s, s.controller.get_target()) for s in slots]
        for i in range(len(targets)):
            for j in range(i + 1, len(targets)):
                si, ti = targets[i]
                sj, tj = targets[j]
                dist = ((ti[0] - tj[0]) ** 2 + (ti[1] - tj[1]) ** 2) ** 0.5
                if dist < self._min_separation_m:
                    return False, (
                        f"目標が近すぎます: 「{si.name}」と「{sj.name}」の"
                        f"XY 距離 {dist:.2f}m < {self._min_separation_m:.2f}m")

        started: list[str] = []
        for slot in slots:
            try:
                self._serial.send_to(slot.node_id, proto.MsgType.CMD_START)
            except SerialLinkError as exc:
                # 途中失敗: 送信済みの機体は止める(片肺離陸を防ぐ)
                self._warn(f"CMD_START 送信失敗({slot.name}): {exc}")
                self.stop_all()
                return False, f"CMD_START 送信失敗: {exc}"
            with self._lock:
                slot.phase = SLOT_ARMED
                slot.armed_since = now
            slot.controller.set_control_active(True)
            started.append(slot.name)
        self._info("一斉離陸開始: " + ", ".join(started))
        return True, "開始しました"

    def stop_all(self) -> bool:
        """全機へ CMD_STOP(再送監視つき)。フェーズを問わず受け付ける。"""
        with self._lock:
            slots = list(self._slots)
        if not slots or not self._serial.is_connected:
            return False
        now = self._clock()
        ok_any = False
        for slot in slots:
            slot.controller.set_control_active(False)
            # 再送監視は送信「前」に仕掛ける(単機 stop() と同じ後勝ち規則)
            with self._lock:
                slot.stop_pending = {
                    "deadline": now + self._stop_ack_timeout_s,
                    "resends": 0,
                }
            if self._send_stop_to(slot):
                ok_any = True
            else:
                with self._lock:
                    slot.stop_pending = None
        return ok_any

    def emergency_stop_all(self) -> None:
        """SPACE 緊急停止の優先経路(状態管理なしで CMD_STOP を先行送出)。

        session.emergency_stop() から呼ばれる。ロックは serial の TX ロック
        のみで完結し、再送監視などの状態管理は続く stop() → stop_all() が行う。
        """
        with self._lock:
            slots = list(self._slots)
        for slot in slots:
            slot.controller.set_control_active(False)
        if not self._serial.is_connected:
            return
        for slot in slots:
            try:
                self._serial.send_to(slot.node_id, proto.MsgType.CMD_STOP)
            except SerialLinkError:
                return   # リンク断は supervisor が畳む

    def _send_stop_to(self, slot: DroneSlot) -> bool:
        try:
            self._serial.send_to(slot.node_id, proto.MsgType.CMD_STOP)
        except SerialLinkError as exc:
            self._warn(f"CMD_STOP 送信失敗({slot.name}): {exc}")
            return False
        self._info(f"CMD_STOP 送信({slot.name})")
        return True

    # ==================================================================
    # 受信ハンドラ(RX スレッド上: 状態更新とキュー投入のみ)
    # ==================================================================

    def _slot_by_node(self, node_id: int) -> Optional[DroneSlot]:
        with self._lock:
            for slot in self._slots:
                if slot.node_id == node_id:
                    return slot
        return None

    def _slot_by_name(self, name: str) -> Optional[DroneSlot]:
        with self._lock:
            for slot in self._slots:
                if slot.name == name:
                    return slot
        return None

    def _on_tlm_state(self, node_id: int, frame: proto.Frame) -> None:
        slot = self._slot_by_node(node_id)
        if slot is None:
            return
        try:
            tlm = proto.TlmState.from_payload(frame.payload)
        except ValueError:
            return
        now = self._clock()
        with self._lock:
            slot.tlm = tlm
            slot.tlm_t = now
        self._update_slot_phase(slot, tlm.state, tlm.flags, now)

    def _on_tlm_event(self, node_id: int, frame: proto.Frame) -> None:
        slot = self._slot_by_node(node_id)
        if slot is None:
            return
        try:
            event = proto.TlmEvent.from_payload(frame.payload)
        except ValueError:
            return
        # STOP 再送待ちの解除(LANDING / WAIT イベント)
        if event.state in (proto.FlightState.LANDING, proto.FlightState.WAIT):
            with self._lock:
                slot.stop_pending = None
        # START 拒否 → 当該スロットの armed を解除
        if event.reason in _START_REJECT_REASONS:
            with self._lock:
                if slot.phase == SLOT_ARMED:
                    slot.phase = SLOT_IDLE
                    slot.armed_since = None
            slot.controller.set_control_active(False)
            self._warn(f"離陸拒否({slot.name}): {_reason_name(event.reason)}")
        self._update_slot_phase(slot, event.state, 0, self._clock())
        self._events_put({
            "type": "event",
            "data": {
                "drone": slot.name,
                "node_id": slot.node_id,
                "state": int(event.state),
                "state_name": _state_name(event.state),
                "prev_state": int(event.prev_state),
                "prev_state_name": _state_name(event.prev_state),
                "reason": int(event.reason),
                "reason_name": _reason_name(event.reason),
                "flags": int(event.flags),
                "voltage": float(event.voltage),
            },
        })

    def _update_slot_phase(self, slot: DroneSlot, state: int, flags: int,
                           now: float) -> None:
        """機体の報告状態からスロットフェーズを更新する(単機と同じ規則)。"""
        flying_flag = bool(flags & proto.TlmState.FLAG_FLYING)
        in_flight = flying_flag or state in _IN_FLIGHT_STATES
        deactivate = False
        with self._lock:
            if slot.phase == SLOT_ARMED:
                if in_flight:
                    slot.phase = SLOT_FLYING
                    slot.armed_since = None
                elif (state in _ON_GROUND_STATES
                      and slot.armed_since is not None
                      and now - slot.armed_since > self._start_grace_s):
                    slot.phase = SLOT_IDLE
                    slot.armed_since = None
                    deactivate = True
            elif slot.phase == SLOT_FLYING and not in_flight \
                    and state in _ON_GROUND_STATES:
                slot.phase = SLOT_IDLE
                deactivate = True
            elif slot.phase == SLOT_IDLE and in_flight:
                # 既に飛行中の機体に接続した場合の昇格(単機と同じ)
                slot.phase = SLOT_FLYING
        if deactivate:
            slot.controller.set_control_active(False)

    # ==================================================================
    # 監視(session.supervise から 20Hz で呼ばれる)
    # ==================================================================

    def supervise(self, now: float) -> None:
        with self._lock:
            if not self._active:
                return
            slots = list(self._slots)

        for slot in slots:
            # --- STOP 再送(600ms 以内に LANDING/WAIT イベントなし) ---
            resend = False
            with self._lock:
                pending = slot.stop_pending
                if pending is not None and now >= pending["deadline"]:
                    if pending["resends"] < self._stop_max_retries:
                        pending["resends"] += 1
                        pending["deadline"] = now + self._stop_ack_timeout_s
                        resend = True
                        resend_count = pending["resends"]
                    else:
                        slot.stop_pending = None
                        self._warn(f"CMD_STOP への応答がありません"
                                   f"({slot.name}、再送上限到達)")
            if resend:
                self._warn(f"CMD_STOP 応答なし({slot.name})→ 再送 "
                           f"({resend_count}/{self._stop_max_retries})")
                self._send_stop_to(slot)

            # --- TLM 途絶(リレー再起動・ピア表喪失・機体電源断の検出) ---
            # 機体は 500ms のセットポイント途絶で自律着陸するため、PC 側は
            # スロットを安全側(idle)へ戻し、オペレータへ状況を通知する。
            # ベースラインは armed_since(CMD_START 後 TLM が一度も来ない
            # 機体もこの経路で解放され、セッションが詰まらない)。
            demote = False
            with self._lock:
                if slot.phase in (SLOT_ARMED, SLOT_FLYING):
                    baseline = slot.tlm_t if slot.tlm_t is not None \
                        else slot.armed_since
                    if baseline is not None \
                            and now - baseline > self._tlm_timeout_s:
                        slot.phase = SLOT_IDLE
                        slot.armed_since = None
                        demote = True
            if demote:
                slot.controller.set_control_active(False)
                self._warn(f"テレメトリ途絶 >{self._tlm_timeout_s:.0f}s"
                           f"({slot.name}): 機体は自律着陸しているはずです。"
                           "リレー/機体の状態を確認してください"
                           "(リレー再起動時は 選択適用 で再設定)")
                self._send_stop_to(slot)   # ベストエフォート(届けば冗長停止)

            # --- MoCap 途絶(armed/flying のスロットのみ) ---
            with self._lock:
                phase = slot.phase
            if phase not in (SLOT_ARMED, SLOT_FLYING):
                continue
            age = slot.controller.mocap_age_s(now)
            dropped = age is None or age > self._mocap_dropout_level_s
            warn_dropout = False
            send_stop = False
            with self._lock:
                if dropped:
                    if not slot.mocap_warned:
                        slot.mocap_warned = True
                        warn_dropout = True
                    if (age is None or age > self._mocap_dropout_stop_s) \
                            and not slot.mocap_stop_sent:
                        slot.mocap_stop_sent = True
                        send_stop = True
                else:
                    slot.mocap_warned = False
                    slot.mocap_stop_sent = False
            if warn_dropout:
                self._warn(f"MoCap 途絶 >300ms({slot.name}): "
                           "セットポイントを水平に固定します")
            if send_stop:
                self._warn(f"MoCap 途絶 >2s({slot.name}): "
                           "CMD_STOP を送信します(自動着陸)")
                slot.controller.set_control_active(False)
                with self._lock:
                    slot.stop_pending = {
                        "deadline": now + self._stop_ack_timeout_s,
                        "resends": 0,
                    }
                self._send_stop_to(slot)

            # --- XY 誤差の持続的発散(flying かつ閉ループ中のみ) ---
            # rigid_body_id を取り違えると各機の PID が他機の位置でループを
            # 閉じる交差結合の不安定系になる。MoCap が新鮮なのに XY 誤差が
            # 閾値を超え続けたら発散とみなし当該機を止める(途絶中は誤差が
            # 古いため判定せず、上の途絶フェイルセーフに委ねる)。
            error_m = slot.controller.xy_error_m()
            control_active = slot.controller.control_active
            diverged = False
            with self._lock:
                if (phase == SLOT_FLYING and control_active and not dropped
                        and error_m is not None
                        and error_m > self._divergence_error_m):
                    if slot.error_high_since is None:
                        slot.error_high_since = now
                    elif now - slot.error_high_since \
                            > self._divergence_hold_s:
                        slot.error_high_since = None
                        diverged = True
                else:
                    slot.error_high_since = None
            if diverged:
                self._warn(
                    f"XY 位置誤差 >{self._divergence_error_m:.1f}m が "
                    f"{self._divergence_hold_s:.1f}s 継続({slot.name}): "
                    "発散とみなし CMD_STOP を送信します(rigid_body_id の"
                    "対応を「RB確認」で確認してください)")
                slot.controller.set_control_active(False)
                with self._lock:
                    slot.stop_pending = {
                        "deadline": now + self._stop_ack_timeout_s,
                        "resends": 0,
                    }
                self._send_stop_to(slot)

    # ==================================================================
    # スナップショット
    # ==================================================================

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def any_armed_or_flying(self) -> bool:
        with self._lock:
            return any(s.phase in (SLOT_ARMED, SLOT_FLYING)
                       for s in self._slots)

    def selected_names(self) -> list[str]:
        with self._lock:
            return [s.name for s in self._slots]

    def flying_profiles(self) -> dict[str, dict]:
        """armed/flying スロットの選択時プロファイル(name → profile)。

        update_airframes の飛行ガード(飛行中の機体のプロファイル変更拒否)が
        参照する。
        """
        with self._lock:
            return {s.name: s.profile for s in self._slots
                    if s.phase in (SLOT_ARMED, SLOT_FLYING)}

    def snapshot(self, now: float) -> dict:
        """WebSocket 20Hz 配信用(session.multi ノード。UI 単位系 deg/m)。"""
        with self._lock:
            active = self._active
            slots = list(self._slots)
        drones = []
        for slot in slots:
            with self._lock:
                tlm = slot.tlm
                tlm_t = slot.tlm_t
                phase = slot.phase
                target_set = slot.target_set
                stop_pending = slot.stop_pending is not None
            if tlm is None or tlm_t is None:
                tlm_node = None
            else:
                tlm_node = {
                    "state": int(tlm.state),
                    "state_name": _state_name(tlm.state),
                    "flying": bool(tlm.flags & proto.TlmState.FLAG_FLYING),
                    "low_voltage": bool(tlm.flags
                                        & proto.TlmState.FLAG_LOW_VOLTAGE),
                    "voltage": float(tlm.voltage),
                    "altitude_est": float(tlm.altitude_est),
                    "yaw": float(tlm.yaw) * RAD_TO_DEG,
                    "fresh": (now - tlm_t) <= self._telemetry_fresh_s,
                }
            tx, ty, tz = slot.controller.get_target()
            drones.append({
                "node_id": slot.node_id,
                "name": slot.name,
                "mac": slot.mac,
                "rigid_body_id": slot.rigid_body_id,
                "phase": phase,
                "target": ({"x": tx, "y": ty, "z": tz}
                           if target_set else None),
                "tlm": tlm_node,
                "mocap": slot.controller.mocap_snapshot(now),
                "latency_ms": self._serial.node_latency_ms(slot.node_id),
                "stop_pending": stop_pending,
            })
        return {"active": active, "drones": drones}

    # ==================================================================
    # 内部
    # ==================================================================

    def _make_emit(self, node_id: int, profile: dict):
        """スロット専用の setpoint 送信クロージャ(50Hz スレッドから)。"""
        bias_roll = profile["roll_bias_deg"] * DEG_TO_RAD
        bias_pitch = profile["pitch_bias_deg"] * DEG_TO_RAD

        def emit(roll_rad: float, pitch_rad: float, alt_m: float,
                 meta: dict) -> None:
            # v1: ヨー角制御は OFF 固定(flags bit1=0 → 機体はレートダンピング)
            setpoint = proto.CmdSetpoint(
                roll_ref=roll_rad + bias_roll,
                pitch_ref=pitch_rad + bias_pitch,
                alt_ref=alt_m,
                yaw_ref=0.0,
                flags=proto.CmdSetpoint.FLAG_ALT_REF_VALID,
            )
            try:
                self._serial.send_setpoint_to(node_id, setpoint)
            except SerialLinkError:
                pass   # 切断検知は serial の on_disconnect → supervisor が処理

        return emit
