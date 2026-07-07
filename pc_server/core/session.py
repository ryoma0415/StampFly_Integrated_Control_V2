"""SessionManager: pc_server の状態と全コンポーネントを一元管理する。

責務(ARCHITECTURE.md):
- 接続/切断、モード切替(posture/position)、Start/Stop/Reset、ログ ON/OFF
- 機体プロファイル適用: RLY_SET_TARGET(MAC/チャネル)送信+バイアス(deg)を
  rad に変換して送信 roll/pitch 指令へ「加算」
- 単位変換の境界: UI/WebSocket は deg/m、protocol と core 内部は rad/m。
  変換はこの層でのみ行う。
- フェイルセーフ(PROTOCOL.md 規範):
  - MoCap 途絶 >300ms(Position)→ 水平固定+UI警告(固定は position.py)
  - MoCap 途絶 >2s(Position)→ CMD_STOP 送信(自動着陸)
  - STOP 送信後 600ms 以内に LANDING/WAIT イベントなし → 再送(最大3回)+UI警告
  - シリアル切断 → UI 赤色警告(serial_connected=false)

公開メソッドはブロッキングし得る(set_relay_target 最大4秒)。asyncio から
呼ぶ場合は to_thread 等で executor に逃がすこと(app.py 参照)。
UI コマンドは _command_lock で直列化される(phase チェックとそれに続く
動作を原子化し、複数クライアントからの同時コマンドの交錯を防ぐ)。
"""

from __future__ import annotations

import functools
import math
import queue
import threading
import time
from typing import Callable, Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由

from . import config as cfg
from .calibration import CalibrationManager, ack_detail, ack_ok
from .experiment import ExperimentHub
from .ffprofile import FfProfileManager
from .logger import FlightLogger
from .mocap import DEG_TO_RAD, RAD_TO_DEG, MocapSource
from .multi import MultiControlManager
from .position import PositionController
from .posture import PostureController, run_paced_loop
from .serial_link import SerialLink, SerialLinkError

MODE_POSTURE = "posture"
MODE_POSITION = "position"
MODE_EXPERIMENT = "experiment"   # v2: モーターテスト/スイープ/キャリブ実験
MODE_MULTI = "multi"             # 複数機同時位置制御(2〜4機、MoCap)

_ALL_MODES = (MODE_POSTURE, MODE_POSITION, MODE_EXPERIMENT, MODE_MULTI)

PHASE_IDLE = "idle"
PHASE_CONNECTED = "connected"
PHASE_ARMED = "armed"
PHASE_FLYING = "flying"

# 飛行中とみなす FlightState(LANDING 中も「flying」フェーズとして扱う)
_IN_FLIGHT_STATES = frozenset({
    proto.FlightState.TAKEOFF, proto.FlightState.HOVER, proto.FlightState.LANDING,
})
_ON_GROUND_STATES = frozenset({
    proto.FlightState.INIT, proto.FlightState.CALIBRATION,
    proto.FlightState.WAIT, proto.FlightState.COMPLETE,
})
_START_REJECT_REASONS = frozenset({
    proto.Reason.START_REJECTED_LOW_VOLTAGE,
    proto.Reason.START_REJECTED_NOT_READY,
})

MS_PER_S = 1000.0

# WebSocket の "log" メッセージの origin 値(LOG_TEXT 由来以外はサーバ発)
LOG_ORIGIN_NAMES = {proto.LogText.ORIGIN_RELAY: "relay",
                    proto.LogText.ORIGIN_DRONE: "drone"}
LOG_ORIGIN_SERVER = "server"


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


def _json_safe(value):
    """非有限 float(NaN/Inf)を再帰的に None へ落とす。

    json.dumps は NaN/Infinity を不正な JSON トークンとして出力し
    (allow_nan=True 既定)、ブラウザ側 JSON.parse がフレームごと黙って
    捨てて UI が固まるため、WS へ渡す dict(スナップショット・イベント)は
    すべてこれを通す。TLM 由来の float は struct.unpack が NaN を素通し
    するので、個別フィールドではなくこの一括変換で漏れなく守る。"""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def _ui_command(method):
    """UI コマンドを self._command_lock で直列化するデコレータ。

    phase チェック(self._lock 内)とそれに基づく動作(送信スレッドの
    起動/停止・CMD 送信など)の間に別コマンドが割り込む TOCTOU を防ぐ。
    RLock のため supervisor のフェイルセーフ(stop 呼び出し)とも安全。
    """
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._command_lock:
            return method(self, *args, **kwargs)
    return wrapper


class SessionManager:
    """サーバ全体のセッション状態(唯一のインスタンスを app.py が保持)。"""

    def __init__(self,
                 server_config: Optional[dict] = None,
                 control_config: Optional[dict] = None,
                 airframes: Optional[list[dict]] = None,
                 transport_factory: Optional[Callable] = None,
                 natnet_client_factory: Optional[Callable] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.server_config = server_config or cfg.load_server_config()
        self.control_config = control_config or cfg.load_control_config()
        self.airframes = airframes if airframes is not None else cfg.load_airframes()
        # 旧形式(rigid_body_id なし)のプロファイルをメモリ上で正規化する
        # (update_airframes の同一性比較・複数機モードの参照を一貫させる)
        for profile in self.airframes:
            profile.setdefault("rigid_body_id", None)

        failsafe = self.server_config["failsafe"]
        self._stop_ack_timeout_s: float = failsafe["stop_ack_timeout_s"]
        self._stop_max_retries: int = failsafe["stop_max_retries"]
        self._start_grace_s: float = failsafe["start_grace_s"]
        self._mocap_dropout_level_s: float = failsafe["mocap_dropout_level_s"]
        self._mocap_dropout_stop_s: float = failsafe["mocap_dropout_stop_s"]
        self._telemetry_fresh_s: float = self.server_config["freshness"]["telemetry_fresh_s"]
        # RLY_STATS(1Hz)の鮮度閾値。受信「時刻」で判定する(counter の内容変化では
        # 判定しない — ターゲット未設定で上りを拒否中は全counterが静止し得るため)
        self._relay_stats_fresh_s: float = self.server_config["freshness"]["relay_stats_fresh_s"]
        self._supervisor_period_s: float = 1.0 / self.server_config["rates"]["supervisor_hz"]

        self._clock = clock

        # UI への即時メッセージ(event / log)。app.py が drain する。
        self.events: queue.Queue = queue.Queue()

        # 構成要素
        self.serial = SerialLink(self.server_config,
                                 transport_factory=transport_factory,
                                 on_disconnect=self._on_serial_disconnect)
        self.mocap = MocapSource(self.control_config["natnet"],
                                 self.control_config["coordinate_transform"],
                                 client_factory=natnet_client_factory)
        self.posture = PostureController(self.server_config, self._emit_setpoint,
                                         clock=clock)
        self.position = PositionController(self.server_config, self.control_config,
                                           self._emit_setpoint, clock=clock)
        self.logger = FlightLogger(
            flush_every_rows=self.server_config["logging"]["flush_every_rows"])

        # v2: 実験モード(モーターテスト/スイープ)+キャリブ/FF プロファイル
        self.experiment = ExperimentHub(self.server_config, self.serial,
                                        notify=self.warn)
        self.calibration = CalibrationManager(
            self.server_config, self.serial, self.experiment, notify=self.info,
            tlm_state_provider=self._tlm_state_snapshot)
        self.ffprofile = FfProfileManager(self.serial, self.experiment,
                                          self.calibration, notify=self.warn)
        # 複数機同時制御(MODE_MULTI)。ノード付き受信ハンドラは
        # MultiControlManager 自身が serial に登録する。
        self.multi = MultiControlManager(
            self.server_config, self.control_config, self.serial, self.mocap,
            notify_info=self.info, notify_warn=self.warn,
            # WS 行きの dict は非有限 float を必ず None 化する(規約)
            events_put=lambda event: self.events.put(_json_safe(event)),
            clock=clock)

        # 受信ディスパッチ登録(ハンドラは RX スレッド上: ブロッキング禁止)
        self.serial.register_handler(proto.MsgType.TLM_STATE, self._on_tlm_state)
        self.serial.register_handler(proto.MsgType.TLM_EVENT, self._on_tlm_event)
        self.serial.register_handler(proto.MsgType.LOG_TEXT, self._on_log_text)
        self.serial.register_handler(proto.MsgType.RLY_STATS, self._on_rly_stats)
        self.serial.register_handler(proto.MsgType.TLM_EXP, self._on_tlm_exp)
        self.serial.register_handler(proto.MsgType.TLM_CAL_DATA,
                                     self._on_tlm_cal_data)

        # UI コマンドの直列化(_ui_command デコレータが使用)。supervisor の
        # stop()/teardown とも共有する。self._lock より外側で取得すること
        # (逆順で取るとデッドロックする)。
        self._command_lock = threading.RLock()

        # セッション状態(self._lock で保護)
        self._lock = threading.Lock()
        self._mode = MODE_POSTURE
        self._phase = PHASE_IDLE
        self._airframe: Optional[dict] = None
        self._bias_roll_rad = 0.0
        self._bias_pitch_rad = 0.0
        self._relay_target_ok = False
        self._logging_enabled = False
        self._armed_since: Optional[float] = None
        self._link_lost = False
        # 実験モード: 機体が MOTOR_TEST 状態であることを ACK で確認済みか
        self._experiment_active = False

        # STOP 再送管理: None または {"deadline": t, "resends": n}
        self._stop_pending: Optional[dict] = None

        # MoCap 途絶警告のエピソード管理
        self._mocap_warned = False
        self._mocap_stop_sent = False

        # 最新テレメトリ
        self._tlm_state: Optional[proto.TlmState] = None
        self._tlm_state_t: Optional[float] = None
        self._rly_stats: Optional[proto.RlyStats] = None
        self._rly_stats_t: Optional[float] = None   # 受信時刻(鮮度判定用)

        # 監視スレッド
        self._supervisor_thread: Optional[threading.Thread] = None
        self._supervisor_stop = threading.Event()

        # 既定の機体プロファイル(UI が select_airframe するまでの初期値)。
        # MAC 未設定のプロファイルは選択できないため、「MAC が設定済みの最初の
        # プロファイル」を既定とする。1件もなければ未選択のまま起動する
        # (connect 時に「機体プロファイルが選択されていません」と案内される)。
        default_profile = next(
            (p for p in self.airframes if cfg.mac_is_set(p.get("mac"))), None)
        if default_profile is not None:
            self._apply_airframe(default_profile)

    # ==================================================================
    # UI コマンド(app.py から executor 経由で呼ばれる。ブロッキング可)
    # ==================================================================

    @_ui_command
    def connect(self, port: str) -> bool:
        """シリアル接続し、リレーに機体ターゲットを設定、送信を開始する。"""
        with self._lock:
            if self._phase != PHASE_IDLE:
                self._warn_locked("既に接続済みです")
                return False
            airframe = self._airframe
        if airframe is None:
            self.warn("機体プロファイルが選択されていません")
            return False

        try:
            self.serial.connect(port)
        except SerialLinkError as exc:
            self.warn(f"シリアル接続失敗: {exc}")
            return False

        with self._lock:
            self._phase = PHASE_CONNECTED
            self._link_lost = False
            self._stop_pending = None
            self._tlm_state = None
            self._tlm_state_t = None
            self._rly_stats = None
            self._rly_stats_t = None

        # リレーへ ESP-NOW ピア設定(各1.0s 待ち、初回+最大3回再送=最大4回。
        # 完了まで上り転送はリレー側で拒否されるため、送信スレッド起動より先に行う)
        self._configure_relay_target(airframe)

        with self._lock:
            mode = self._mode
        if mode == MODE_EXPERIMENT:
            # 実験モードで接続: 50Hz 送信は開始せず CMD_MODE(1)+ACK
            self._enter_experiment()
        elif mode == MODE_MULTI:
            pass   # 複数機モード: multi_select(RLY_SET_PEERS)まで送信なし
        else:
            self._start_active_sender()
        if self._logging_enabled:
            self._start_log_file()
        self._supervisor_stop.clear()
        self._supervisor_thread = threading.Thread(
            target=self._supervisor_loop, name="session-supervisor", daemon=True)
        self._supervisor_thread.start()
        self.info(f"接続しました: {port}")
        return True

    @_ui_command
    def disconnect(self) -> None:
        """送信停止・mocap停止・ログ停止のうえシリアルを閉じる。"""
        with self._lock:
            was_active = self._phase != PHASE_IDLE
        self._supervisor_stop.set()
        thread = self._supervisor_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._supervisor_thread = None
        if not was_active and not self.serial.is_connected:
            return   # もともと未接続
        self._teardown_session("切断しました")

    @_ui_command
    def select_airframe(self, name: str) -> bool:
        """機体プロファイルを選択し、接続中なら RLY_SET_TARGET を再送する。"""
        profile = next((p for p in self.airframes if p["name"] == name), None)
        if profile is None:
            self.warn(f"機体プロファイルが見つかりません: {name}")
            return False
        if not cfg.mac_is_set(profile.get("mac")):
            # MAC 未設定プロファイルは接続中・未接続を問わず選択不可
            # (リレーへ設定すべきターゲットが存在しないため)
            self.warn(f"機体「{name}」は MAC が未設定のため選択できません。"
                      "機体を USB 接続し起動ログ『ESP-NOW ready: MAC=...』で MAC を"
                      "確認して、機体プロファイル編集画面で設定してください")
            return False
        with self._lock:
            if self._phase in (PHASE_ARMED, PHASE_FLYING):
                self._warn_locked("飛行中は機体プロファイルを変更できません")
                return False
        if self.multi.active:
            # RLY_SET_TARGET はリレーのマルチピア表をクリアするため、選択中は
            # 地上でも拒否する(飛行中に許すと全機への上り経路が切断される)
            self.warn("複数機モードの機体選択中は機体プロファイルを変更できません"
                      "(先に複数機の選択を解除してください)")
            return False
        self._apply_airframe(profile)
        if self.serial.is_connected:
            self._configure_relay_target(profile)
        self.info(f"機体プロファイル: {name}")
        return True

    @_ui_command
    def update_airframes(self, new_airframes) -> tuple[bool, Optional[str]]:
        """機体プロファイル一覧を検証・保存し、セッションへ反映する。

        戻り値は (ok, error)。error は UI 表示用の日本語メッセージ。

        ポリシー(本実装の規範):
        - 検証: 名前は非空かつ一意 / mac は空(未設定)または 6 オクテット16進 /
          wifi_channel・バイアス・default_alt_m は config の制限内 / notes は文字列。
          件数と name/notes の文字数は airframe_limits の上限内
          (max_profiles / name_max_chars / notes_max_chars)。
        - 飛行ガード: phase が armed/flying のとき、選択中プロファイルが
          変更・削除される更新は拒否する(飛行中の機体の挙動を変えないため)。
          選択中プロファイルが同一内容のままなら、他プロファイルの編集は許可。
        - 受理時: airframes.json へ原子的に保存し self.airframes を更新する。
        - 選択中プロファイルが残存し bias / default_alt_m が変わった場合
          (飛行中でないときのみ到達し得る)は即座に再適用する。
        - 選択中プロファイルが削除 / MAC 未設定化された場合は選択を解除し
          relay_target_ok を false にする(未選択のままの start は拒否される)。
        - MAC / wifi_channel の変更は次の select_airframe / connect で反映する。
          RLY_SET_TARGET の自動再送はしない(接続中はオペレータへ選び直しを促す
          ログを出す)。
        """
        ok, error, normalized = self._validate_airframes(new_airframes)
        if not ok:
            return False, error

        with self._lock:
            phase = self._phase
            current = self._airframe

        # --- 飛行ガード: 選択中プロファイルの変更/削除を拒否 ---
        if phase in (PHASE_ARMED, PHASE_FLYING) and current is not None:
            replacement = next(
                (p for p in normalized if p["name"] == current["name"]), None)
            if replacement != current:
                return False, (f"飛行中のため、選択中の機体プロファイル"
                               f"「{current['name']}」の変更・削除はできません。"
                               "着陸後にやり直してください")

        # --- 飛行ガード(複数機): 飛行中スロットのプロファイル変更/削除を拒否 ---
        for name, profile in self.multi.flying_profiles().items():
            replacement = next(
                (p for p in normalized if p["name"] == name), None)
            if replacement != profile:
                return False, (f"複数機制御で飛行中のため、機体プロファイル"
                               f"「{name}」の変更・削除はできません。"
                               "着陸後にやり直してください")

        # --- 永続化(原子的書き込み)→ メモリ反映 ---
        try:
            cfg.save_airframes(normalized)
        except OSError as exc:
            return False, f"airframes.json の保存に失敗しました: {exc}"
        self.airframes = normalized

        self._refresh_selected_airframe(current)
        self.info(f"機体プロファイルを更新しました({len(normalized)}件)")
        return True, None

    def _refresh_selected_airframe(self, current: Optional[dict]) -> None:
        """update_airframes 受理後、選択中プロファイルを新リストへ追従させる。"""
        if current is None:
            return
        replacement = next(
            (p for p in self.airframes if p["name"] == current["name"]), None)

        if replacement is None or not cfg.mac_is_set(replacement["mac"]):
            # 削除された / MAC が未設定になった → 選択解除(飛行ガード通過済み =
            # 非飛行時のみ到達)。バイアスもゼロへ戻す。リレーのピア設定自体は
            # 残る(STOP は届く)が「選択中プロファイルに対応する設定」ではなく
            # なったため relay_target_ok を落とし、UI のリレー表示で選び直しを促す。
            with self._lock:
                self._airframe = None
                self._bias_roll_rad = 0.0
                self._bias_pitch_rad = 0.0
                self._relay_target_ok = False
            reason = ("削除された" if replacement is None else "MAC が未設定になった")
            self.warn(f"選択中の機体プロファイル「{current['name']}」が{reason}ため"
                      "選択を解除しました。機体を選び直してください")
            return

        bias_changed = (
            replacement["roll_bias_deg"] != current["roll_bias_deg"]
            or replacement["pitch_bias_deg"] != current["pitch_bias_deg"]
            or replacement["default_alt_m"] != current["default_alt_m"])
        target_changed = (replacement["mac"] != current["mac"]
                          or replacement["wifi_channel"] != current["wifi_channel"])

        with self._lock:
            self._airframe = replacement
            if bias_changed:
                # 飛行ガード通過済みのため、ここに来るのは非飛行時のみ。
                # 次の 50Hz 送信から新バイアスが加算される。
                self._bias_roll_rad = replacement["roll_bias_deg"] * DEG_TO_RAD
                self._bias_pitch_rad = replacement["pitch_bias_deg"] * DEG_TO_RAD
        if bias_changed:
            self.posture.set_default_alt(replacement["default_alt_m"])
            self.info(f"機体「{replacement['name']}」のバイアス/初期高度を"
                      "再適用しました")
        if target_changed and self.serial.is_connected:
            # RLY_SET_TARGET は黙って再送しない(契約)。選び直しで反映させる。
            self.warn(f"機体「{replacement['name']}」の MAC/チャネル変更は"
                      "まだリレーに反映されていません。機体プロファイルを"
                      "選び直してください")

    def _validate_airframes(self, new_airframes) \
            -> tuple[bool, Optional[str], list[dict]]:
        """プロファイル配列を検証し、正規化済みリストを返す。

        正規化: キーを正準順(name, mac, wifi_channel, roll_bias_deg,
        pitch_bias_deg, default_alt_m, rigid_body_id, notes)に揃え、
        数値型・MAC 表記("AA:BB:..." 大文字)を統一する。制限値は
        server.json(clamps.max_roll_pitch_deg / airframe_limits)から取る。
        rigid_body_id は任意(複数機モードで必須。null = 未設定)。
        """
        limits = self.server_config["airframe_limits"]
        ch_min, ch_max = limits["wifi_channel_min"], limits["wifi_channel_max"]
        alt_min, alt_max = limits["default_alt_min_m"], limits["default_alt_max_m"]
        max_profiles = limits["max_profiles"]
        name_max = limits["name_max_chars"]
        notes_max = limits["notes_max_chars"]
        bias_limit = self.server_config["clamps"]["max_roll_pitch_deg"]
        required_keys = ("name", "mac", "wifi_channel", "roll_bias_deg",
                         "pitch_bias_deg", "default_alt_m", "notes")
        optional_keys = ("rigid_body_id",)   # 複数機モード用(null = 未設定)

        if not isinstance(new_airframes, list) or not new_airframes:
            return False, "機体プロファイルは1件以上の配列で指定してください", []
        if len(new_airframes) > max_profiles:
            # 設定ファイル・UI プルダウンの肥大化を防ぐ上限
            return False, (f"機体プロファイルは最大 {max_profiles} 件までです"
                           f"({len(new_airframes)} 件指定されました)"), []

        normalized: list[dict] = []
        names: set[str] = set()
        for index, entry in enumerate(new_airframes):
            label = f"{index + 1}件目"
            if not isinstance(entry, dict):
                return False, f"{label}: プロファイルはオブジェクトで指定してください", []
            missing = [k for k in required_keys if k not in entry]
            if missing:
                return False, f"{label}: キーが不足しています: {', '.join(missing)}", []
            unknown = [k for k in entry
                       if k not in required_keys and k not in optional_keys]
            if unknown:
                return False, f"{label}: 不明なキーがあります: {', '.join(unknown)}", []

            name = entry["name"]
            if not isinstance(name, str) or not name.strip():
                return False, f"{label}: 機体名が空です", []
            name = name.strip()
            if len(name) > name_max:
                return False, (f"{label}: 機体名は {name_max} 文字以内で"
                               "指定してください"), []
            label = f"「{name}」"
            if name in names:
                return False, f"機体名が重複しています: {label}", []
            names.add(name)

            mac_raw = entry["mac"]
            if not isinstance(mac_raw, str):
                return False, f"{label}: MAC は文字列で指定してください", []
            if cfg.mac_is_set(mac_raw):
                try:
                    mac = cfg.format_mac(cfg.parse_mac(mac_raw.strip()))
                except ValueError:
                    return False, (f"{label}: MAC の形式が不正です: {mac_raw!r}"
                                   "(AA:BB:CC:DD:EE:FF 形式、未設定なら空欄)"), []
            else:
                mac = cfg.MAC_UNSET

            channel = entry["wifi_channel"]
            if isinstance(channel, bool) or not isinstance(channel, int) \
                    or not (ch_min <= channel <= ch_max):
                return False, (f"{label}: wifi_channel は {ch_min}–{ch_max} の"
                               "整数で指定してください"), []

            numbers: dict[str, float] = {}
            for key, low, high, jp in (
                    ("roll_bias_deg", -bias_limit, bias_limit, "Roll バイアス"),
                    ("pitch_bias_deg", -bias_limit, bias_limit, "Pitch バイアス"),
                    ("default_alt_m", alt_min, alt_max, "初期高度")):
                value = entry[key]
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    return False, f"{label}: {jp}({key})は数値で指定してください", []
                if not (low <= value <= high):
                    return False, (f"{label}: {jp}({key})は {low}–{high} の"
                                   "範囲で指定してください"), []
                numbers[key] = float(value)

            notes = entry["notes"]
            if not isinstance(notes, str):
                return False, f"{label}: notes は文字列で指定してください", []
            if len(notes) > notes_max:
                return False, (f"{label}: notes は {notes_max} 文字以内で"
                               "指定してください"), []

            rigid_body_id = entry.get("rigid_body_id")
            if rigid_body_id is not None:
                if isinstance(rigid_body_id, bool) \
                        or not isinstance(rigid_body_id, int) \
                        or rigid_body_id < 1:
                    return False, (f"{label}: rigid_body_id は 1 以上の整数"
                                   "(未設定なら null)で指定してください"), []

            normalized.append({
                "name": name,
                "mac": mac,
                "wifi_channel": channel,
                "roll_bias_deg": numbers["roll_bias_deg"],
                "pitch_bias_deg": numbers["pitch_bias_deg"],
                "default_alt_m": numbers["default_alt_m"],
                "rigid_body_id": rigid_body_id,
                "notes": notes,
            })
        return True, None, normalized

    @_ui_command
    def set_mode(self, mode: str) -> bool:
        """posture/position/experiment モードを切り替える(飛行中は不可)。

        experiment 開始(契約 §3.1): 飛行中は拒否 → 50Hz セットポイント送信
        停止 → CMD_MODE(1) 送信+ACK 確認。experiment 終了: モーター停止確認 →
        CMD_MODE(0)+ACK → posture/position に復帰(50Hz 送信再開)。
        未接続時はモードのみ切り替え、CMD_MODE は次の connect で送る。
        """
        if mode not in _ALL_MODES:
            self.warn(f"不明なモード: {mode}")
            return False
        with self._lock:
            if self._phase in (PHASE_ARMED, PHASE_FLYING):
                self._warn_locked("飛行中はモードを変更できません")
                return False
            if self._mode == mode:
                return True
            previous = self._mode
        if self.multi.any_armed_or_flying():
            self.warn("複数機制御中はモードを変更できません")
            return False
        if previous == MODE_EXPERIMENT:
            # 実験モードからの離脱: モーター停止確認 → CMD_MODE(0)+ACK
            self._exit_experiment()
        if previous == MODE_MULTI:
            # 複数機モードからの離脱: スロット解放+単機ターゲットへ復帰
            self._exit_multi()
        self._stop_active_sender()
        with self._lock:
            self._mode = mode
        if self.serial.is_connected:
            if mode == MODE_EXPERIMENT:
                if not self._enter_experiment():
                    # 機体が MOTOR_TEST に入れなかった → 元のモードへ戻す
                    with self._lock:
                        self._mode = previous
                    if previous not in (MODE_EXPERIMENT, MODE_MULTI):
                        self._start_active_sender()
                    return False
            elif mode == MODE_MULTI:
                pass   # multi_select(RLY_SET_PEERS)まで送信なし
            else:
                self._start_active_sender()
            if self._logging_enabled:
                self._start_log_file()   # ファイル名にモードを含むため切替
        self.info(f"モード: {mode}")
        return True

    @_ui_command
    def activate_experiment(self) -> bool:
        """実験モードの再有効化(CMD_STOP 等で機体が WAIT に戻った後の再開)。"""
        with self._lock:
            mode = self._mode
        if mode != MODE_EXPERIMENT:
            self.warn("実験モードではありません")
            return False
        if not self.serial.is_connected:
            self.warn("未接続のため実験モードを有効化できません")
            return False
        return self._enter_experiment()

    def _enter_experiment(self) -> bool:
        """CMD_MODE(1) を送信し ACK を確認して実験機能を有効化する。"""
        payload = proto.CmdMode(mode=proto.CmdMode.MODE_MOTOR_TEST).to_payload()
        try:
            ack = self.serial.send_with_ack(proto.MsgType.CMD_MODE, payload)
        except SerialLinkError as exc:
            self.warn(f"CMD_MODE(MOTOR_TEST) 送信失敗: {exc}")
            return False
        if not ack_ok(ack):
            self.warn(f"実験モードに入れませんでした"
                      f"(CMD_MODE ACK: {ack_detail(ack)})。"
                      "機体が WAIT 状態か確認してください")
            return False
        with self._lock:
            self._experiment_active = True
        self.experiment.activate()
        self.info("実験モード開始(機体: MOTOR_TEST)")
        return True

    def _exit_multi(self) -> None:
        """複数機モードの離脱: スロット解放+リレーを単機ターゲットへ戻す。"""
        with self._lock:
            airframe = self._airframe
        if self.serial.is_connected and airframe is not None \
                and cfg.mac_is_set(airframe.get("mac")):
            # ピア表は RLY_SET_TARGET が上書きクリアするため個別クリア不要
            self.multi.deactivate(clear_peers=False)
            self._configure_relay_target(airframe)
        else:
            self.multi.deactivate(clear_peers=True)

    def _exit_experiment(self) -> None:
        """実験機能を停止し、接続中なら CMD_MODE(0) で WAIT に戻す。"""
        self.experiment.deactivate()   # スイープ中断+モーター停止を含む
        with self._lock:
            was_active = self._experiment_active
            self._experiment_active = False
        if not (was_active and self.serial.is_connected):
            return
        payload = proto.CmdMode(mode=proto.CmdMode.MODE_FLIGHT).to_payload()
        try:
            ack = self.serial.send_with_ack(proto.MsgType.CMD_MODE, payload)
        except SerialLinkError as exc:
            self.warn(f"CMD_MODE(FLIGHT) 送信失敗: {exc}")
            return
        if not ack_ok(ack):
            # 機体側フェイルセーフ(モーター1.5s途絶停止)に任せ、警告のみ
            self.warn(f"実験モード終了の ACK が確認できません"
                      f"({ack_detail(ack)})。機体状態を確認してください")
        else:
            self.info("実験モード終了(機体: WAIT)")

    @_ui_command
    def start(self) -> bool:
        """離陸開始(CMD_START)。connected フェーズかつ機体選択中のみ受け付ける。"""
        with self._lock:
            phase = self._phase
            mode = self._mode
            airframe = self._airframe
        if phase != PHASE_CONNECTED:
            self.warn(f"開始できません(phase={phase})")
            return False
        if mode == MODE_EXPERIMENT:
            # 実験モード中は START 不可(契約 §3.1。機体側も MOTOR_TEST 中の
            # CMD_START を reason=10 で拒否する)
            self.warn("実験モード中は離陸できません")
            return False
        if mode == MODE_MULTI:
            # 複数機モードは専用の一斉開始(multi_start)を使う
            self.warn("複数機モードでは「一斉スタート」を使用してください")
            return False
        if airframe is None:
            # 接続後でも PUT /api/airframes で選択中プロファイルが削除/MAC 未設定化
            # されると選択は解除される(_refresh_selected_airframe)。プロファイル
            # 未選択のまま ARM しない(connect() と同じガード)。
            self.warn("機体プロファイルが選択されていません。"
                      "機体プロファイルを選び直してください")
            return False
        if mode == MODE_POSITION:
            age = self.position.mocap_age_s(self._clock())
            if age is None or age > self._mocap_dropout_level_s:
                self.warn("MoCap データが新鮮でないため開始できません")
                return False
        try:
            self.serial.send(proto.MsgType.CMD_START)
        except SerialLinkError as exc:
            self.warn(f"CMD_START 送信失敗: {exc}")
            return False
        with self._lock:
            self._phase = PHASE_ARMED
            self._armed_since = self._clock()
        if mode == MODE_POSITION:
            self.position.set_control_active(True)
        self.info("CMD_START 送信")
        return True

    @_ui_command
    def stop(self) -> bool:
        """即時着陸(CMD_STOP)。全飛行状態で受け付け、応答なしなら再送する。"""
        if not self.serial.is_connected:
            self.warn("未接続のため停止コマンドを送れません")
            return False
        self.position.set_control_active(False)
        with self._lock:
            mode = self._mode
        if mode == MODE_MULTI:
            # 複数機モード: 全機へ CMD_STOP(スロットごとの再送監視つき)
            return self.multi.stop_all()
        if mode == MODE_EXPERIMENT:
            # 緊急停止: スイープ/シーケンス中断+モーター停止(キープアライブ
            # が回し続けないよう PC 側状態も落とす)。CMD_STOP で機体は
            # MOTOR_TEST→WAIT に遷移するため、実験の再開には再有効化が必要。
            self.experiment.sequence.abort_if_running()
            self.experiment.sweep.abort_if_running()
            self.experiment.motor_stop()
            with self._lock:
                self._experiment_active = False
        # 再送監視は送信「前」に仕掛ける: 送信直後に届く LANDING イベント
        # (RXスレッド)が _stop_pending を消す方が常に後勝ちになるようにし、
        # 着陸成功後の偽の再送・「応答なし」警告を防ぐ。
        with self._lock:
            self._stop_pending = {
                "deadline": self._clock() + self._stop_ack_timeout_s,
                "resends": 0,
            }
        ok = self._send_stop()
        if not ok:
            # 送信できていないなら応答を待つ意味がない(再送は次の stop で)
            with self._lock:
                self._stop_pending = None
        return ok

    def emergency_stop(self) -> None:
        """SPACE 緊急停止の優先経路(_command_lock を**経由しない**)。

        低速コマンド(select_airframe の RLY_SET_TARGET ACK 待ち最大約4秒、
        connect 等)が _command_lock を保持していても、モーターキープアライブの
        停止と CMD_STOP の送出を先行させる。放置するとキープアライブが
        CMD_MOTOR_RUN を 0.4s 周期で送り続け、機体側 1.5s 途絶フェイルセーフも
        発動しないまま緊急停止が最大約4秒遅れる。

        ここで触るのは hub 内部ロック・serial の TX ロックのみで完結する
        操作に限る(フェーズ遷移や STOP 再送監視などの状態管理は、続けて
        呼ばれる通常の stop() が _command_lock 下で行う)。
        """
        self.position.set_control_active(False)
        # 複数機モード: 全機へ CMD_STOP を先行送出(状態管理は後続の stop())
        self.multi.emergency_stop_all()
        # スイープ/シーケンス中断+モーター停止+キープアライブ停止
        # (CMD_MOTOR_STOP は飛行状態では機体側が bad_state で無害に破棄する)
        self.experiment.sequence.abort_if_running()
        self.experiment.sweep.abort_if_running()
        self.experiment.motor_stop()
        if self.serial.is_connected and not self.multi.active:
            # 単機経路の CMD_STOP(マルチ中は非エンベロープ上りをリレーが
            # 拒否するため送らない — 全機分は emergency_stop_all が送出済み)
            self._send_stop()

    @_ui_command
    def reset(self) -> bool:
        """CMD_RESET(COMPLETE からの復帰)。受理条件はファームが検証する。"""
        if not self.serial.is_connected:
            self.warn("未接続のためリセットできません")
            return False
        try:
            self.serial.send(proto.MsgType.CMD_RESET)
        except SerialLinkError as exc:
            self.warn(f"CMD_RESET 送信失敗: {exc}")
            return False
        self.info("CMD_RESET 送信")
        return True

    @_ui_command
    def set_logging(self, enabled: bool) -> None:
        """CSV ログの ON/OFF。接続中なら即座にファイルを開閉する。"""
        with self._lock:
            self._logging_enabled = bool(enabled)
        if enabled and self.serial.is_connected:
            self._start_log_file()
        elif not enabled:
            self.logger.stop()
            self.info("ログ停止")

    def set_setpoint_deg(self, roll_deg: float, pitch_deg: float, alt_m: float,
                         yaw_deg: Optional[float] = None) -> None:
        """Posture モードの UI setpoint(deg/m)。rad へ変換して渡す。"""
        self.posture.set_setpoint(
            roll_deg * DEG_TO_RAD, pitch_deg * DEG_TO_RAD, alt_m,
            yaw_rad=None if yaw_deg is None else yaw_deg * DEG_TO_RAD)

    def set_target(self, x: float, y: float, z: float) -> None:
        """Position モードの目標位置(制御座標系 m)。"""
        self.position.set_target(x, y, z)

    def set_yaw_setpoint_deg(self, yaw_deg: float) -> None:
        """UI ヨー角スライダ(±180°)。両モードのコントローラへ反映する。"""
        yaw_rad = yaw_deg * DEG_TO_RAD
        self.posture.set_setpoint_yaw_only(yaw_rad)
        self.position.set_yaw_setpoint(yaw_rad)

    @_ui_command
    def set_yaw_control(self, enabled: bool) -> None:
        """ヨー角制御 ON/OFF(CMD_SETPOINT flags bit1)。"""
        self.posture.set_yaw_control(enabled)
        self.position.set_yaw_control(enabled)
        self.info(f"ヨー角制御: {'ON' if enabled else 'OFF'}")

    # ------------------------------------------------------------------
    # v2: 円軌道モード(Position タブ)
    # ------------------------------------------------------------------

    @_ui_command
    def circle_start(self, center_x: float, center_y: float, radius_m: float,
                     period_s: float, clockwise: bool, alt_m: float,
                     face_tangent: bool) -> bool:
        with self._lock:
            mode = self._mode
        if mode != MODE_POSITION:
            self.warn("円軌道は Position モードでのみ使用できます")
            return False
        ok, error = self.position.start_circle(
            center_x, center_y, radius_m, period_s, clockwise, alt_m,
            face_tangent, now=self._clock())
        if not ok:
            self.warn(f"円軌道を開始できません: {error}")
            return False
        direction = "CW" if clockwise else "CCW"
        self.info(f"円軌道開始: 中心=({center_x:.2f}, {center_y:.2f}) "
                  f"r={radius_m:.2f}m 周期={period_s:.1f}s {direction} "
                  f"高度={alt_m:.2f}m"
                  + ("(進行方向を向く)" if face_tangent else ""))
        return True

    @_ui_command
    def circle_stop(self) -> None:
        self.position.stop_circle()
        self.info("円軌道停止: 現在目標でホバリングに復帰します")

    # ------------------------------------------------------------------
    # 複数機同時制御(Multi タブ)
    # ------------------------------------------------------------------

    @_ui_command
    def multi_select(self, names: list[str]) -> bool:
        """複数機モードの機体選択(RLY_SET_PEERS + スロット構築)。"""
        with self._lock:
            mode = self._mode
        if mode != MODE_MULTI:
            self.warn("複数機モードではありません")
            return False
        ok, message = self.multi.select(names, self.airframes)
        if not ok:
            self.warn(f"機体選択不可: {message}")
        return ok

    @_ui_command
    def multi_target(self, name: str, x: float, y: float, z: float) -> bool:
        """機体別の目標位置(制御座標系 m)。

        _command_lock で multi_start と直列化する(一斉開始の目標間隔検証と
        目標変更の TOCTOU を防ぐ。UI はボタン押下時のみ送るため低頻度)。
        """
        ok, message = self.multi.set_target(name, x, y, z)
        if not ok:
            self.warn(f"目標設定不可: {message}")
        return ok

    @_ui_command
    def multi_start(self) -> bool:
        """選択済み全機の一斉離陸。"""
        with self._lock:
            mode = self._mode
        if mode != MODE_MULTI:
            self.warn("複数機モードではありません")
            return False
        ok, message = self.multi.start_all()
        if not ok:
            self.warn(f"一斉開始不可: {message}")
        return ok

    def mocap_bodies(self) -> dict:
        """現在観測中の全リジッドボディ一覧(紐付け確認 UI 用)。

        NatNet 未接続なら接続を試みる(primary コールバックは単機 Position
        モード専用のため no-op で起動する。既に起動済みなら何もしない)。
        """
        connected = self.mocap.connected()
        if not connected:
            connected = self.mocap.start()   # パッシブ起動(primary は触らない)
        return {
            "connected": bool(connected),
            "bodies": _json_safe(self.mocap.bodies_snapshot()),
        }

    # ------------------------------------------------------------------
    # v2: モーターテスト(Experiment タブ)
    # ------------------------------------------------------------------

    def _experiment_ready(self) -> Optional[str]:
        """モーター操作の前提チェック。問題があれば理由を返す。"""
        with self._lock:
            mode = self._mode
            active = self._experiment_active
        if mode != MODE_EXPERIMENT:
            return "実験モードではありません"
        if not self.serial.is_connected:
            return "未接続です"
        if not active:
            return "実験モードが有効化されていません(再有効化してください)"
        return None

    @_ui_command
    def motor_start(self, duty: float, mask: int) -> dict:
        reason = self._experiment_ready()
        if reason is not None:
            self.warn(f"モーター開始不可: {reason}")
            return {"ok": False, "message": reason}
        result = self.experiment.motor_start(duty, mask)
        if not result.get("ok") and result.get("message"):
            # hub 側の拒否(スイープ/シーケンス実行中など)も UI へ通知する
            self.warn(f"モーター開始不可: {result['message']}")
        return result

    @_ui_command
    def motor_apply(self, duty: float) -> dict:
        reason = self._experiment_ready()
        if reason is not None:
            self.warn(f"duty 変更不可: {reason}")
            return {"ok": False, "message": reason}
        result = self.experiment.motor_apply(duty)
        if not result.get("ok") and result.get("message"):
            self.warn(f"duty 変更不可: {result['message']}")
        return result

    def motor_stop(self) -> dict:
        # 停止は前提チェックなしで常に受け付ける(SPACE 緊急停止経路)。
        # 意図的に _ui_command(_command_lock)を経由しない: 低速コマンドの
        # 背後で緊急停止が待たされないよう、hub 内部ロックのみで完結させる
        # (キープアライブ停止 → CMD_MOTOR_STOP 冗長送出)。
        if not self.serial.is_connected:
            return {"ok": False, "message": "未接続です"}
        return self.experiment.motor_stop()

    def sweep_start(self, mask, pattern, notes) -> dict:
        reason = self._experiment_ready()
        if reason is not None:
            return {"ok": False, "message": reason,
                    **self.experiment.sweep.status()}
        return self.experiment.sweep.start(mask, pattern=pattern, notes=notes)

    def sequence_start(self, masks, pattern, notes, min_start_vbat) -> dict:
        reason = self._experiment_ready()
        if reason is not None:
            return {"ok": False, "message": reason,
                    **self.experiment.sequence.status()}
        return self.experiment.sequence.start(
            masks, pattern=pattern, notes=notes, min_start_vbat=min_start_vbat)

    def shutdown(self) -> None:
        """サーバ終了時の後始末。"""
        self.disconnect()
        self.experiment.shutdown()

    # ==================================================================
    # 内部: 接続まわり
    # ==================================================================

    def _apply_airframe(self, profile: dict) -> None:
        with self._lock:
            self._airframe = profile
            # バイアス(deg)→ rad。送信時に roll/pitch 指令へ加算される。
            self._bias_roll_rad = profile["roll_bias_deg"] * DEG_TO_RAD
            self._bias_pitch_rad = profile["pitch_bias_deg"] * DEG_TO_RAD
            self._relay_target_ok = False
        self.posture.set_default_alt(profile["default_alt_m"])

    def _configure_relay_target(self, profile: dict) -> None:
        try:
            mac = cfg.parse_mac(profile["mac"])
            ok, ack = self.serial.set_relay_target(mac, profile["wifi_channel"])
        except (SerialLinkError, ValueError) as exc:
            self.warn(f"RLY_SET_TARGET 失敗: {exc}")
            ok, ack = False, None
        with self._lock:
            self._relay_target_ok = ok
        if ok:
            self.info(f"リレーターゲット設定完了: {profile['mac']} "
                      f"ch{profile['wifi_channel']}")
        else:
            status = ack.status if ack is not None else "no_ack"
            self.warn(f"リレーターゲット設定失敗(status={status})。"
                      "ドローン宛コマンドは転送されません")

    def _start_active_sender(self) -> None:
        with self._lock:
            mode = self._mode
        if mode == MODE_POSITION:
            if not self.mocap.connected():
                if not self.mocap.start(self.position.on_mocap_pose):
                    self.warn("NatNet 接続に失敗しました")
            self.position.start()
        else:
            self.posture.start()

    def _stop_active_sender(self) -> None:
        self.posture.stop()
        self.position.stop()
        self.mocap.shutdown()

    def _start_log_file(self) -> None:
        with self._lock:
            mode = self._mode
        path = self.logger.start(mode)
        self.info(f"ログ開始: {path.name}")

    def _teardown_session(self, message: str) -> None:
        self._stop_active_sender()
        # 複数機スロットを解放(シリアルはこの後閉じるためピア解除は送らない。
        # 各機体はセットポイント 500ms 途絶で自動着陸する)
        self.multi.deactivate(clear_peers=False)
        # 実験機能を停止(シリアルはこの後閉じるため CMD_MODE は送らない。
        # 機体側はモーターコマンド 1.5s 途絶で自動停止する)
        self.experiment.deactivate()
        self.logger.stop()
        self.serial.disconnect()
        with self._lock:
            self._phase = PHASE_IDLE
            self._stop_pending = None
            self._relay_target_ok = False
            self._armed_since = None
            self._mocap_warned = False
            self._mocap_stop_sent = False
            self._experiment_active = False
        self.position.set_control_active(False)
        self.info(message)

    def _on_serial_disconnect(self, reason: str) -> None:
        """SerialLink からの切断通知(RX/TXスレッド上: フラグのみ立てる)。"""
        with self._lock:
            self._link_lost = True
        self.warn(f"シリアル切断: {reason}")

    # ==================================================================
    # 内部: 送信(50Hz スレッドから。バイアス加算・送信・ログ)
    # ==================================================================

    def _emit_setpoint(self, roll_rad: float, pitch_rad: float, alt_m: float,
                       meta: dict) -> None:
        """整形済みセットポイントにバイアスを加算し、送信してログする。

        v2: ヨー目標は meta("yaw_ref_rad" / "yaw_ctrl_on")で受け取り、
        ON のとき flags bit1(FLAG_YAW_REF_VALID)を立てて 17B で送信する。
        OFF のときは yaw_ref=0 / bit1=0(機体は V1 と同一のレートダンピング)。
        """
        with self._lock:
            bias_roll = self._bias_roll_rad
            bias_pitch = self._bias_pitch_rad
            phase = self._phase
        yaw_ctrl_on = bool(meta.get("yaw_ctrl_on"))
        yaw_ref = float(meta.get("yaw_ref_rad") or 0.0) if yaw_ctrl_on else 0.0
        flags = proto.CmdSetpoint.FLAG_ALT_REF_VALID
        if yaw_ctrl_on:
            flags |= proto.CmdSetpoint.FLAG_YAW_REF_VALID
        setpoint = proto.CmdSetpoint(
            roll_ref=roll_rad + bias_roll,
            pitch_ref=pitch_rad + bias_pitch,
            alt_ref=alt_m,
            yaw_ref=yaw_ref,
            flags=flags,
        )
        seq: Optional[int] = None
        send_success = False
        try:
            seq = self.serial.send_setpoint(setpoint)
            send_success = True
        except SerialLinkError:
            pass   # 切断検知は _on_serial_disconnect → supervisor が処理

        if self.logger.active:
            self.logger.log_row(self._build_log_row(
                setpoint, seq, send_success, phase, meta))

    def _build_log_row(self, setpoint: proto.CmdSetpoint, seq: Optional[int],
                       send_success: bool, phase: str, meta: dict) -> dict:
        row = {
            "mode": meta.get("mode"),
            "phase": phase,
            "command_sequence": seq,
            "send_success": send_success,
            "feedback_latency_ms": self.serial.latency_ms,
            "roll_ref_rad": setpoint.roll_ref,
            "pitch_ref_rad": setpoint.pitch_ref,
            "roll_ref_deg": setpoint.roll_ref * RAD_TO_DEG,
            "pitch_ref_deg": setpoint.pitch_ref * RAD_TO_DEG,
            "alt_ref_m": setpoint.alt_ref,
            "roll_bias_deg": self._bias_roll_rad * RAD_TO_DEG,
            "pitch_bias_deg": self._bias_pitch_rad * RAD_TO_DEG,
            # v2: ヨー指令(送信した CMD_SETPOINT のヨー目標)
            "cmd_yaw_ref_rad": setpoint.yaw_ref,
            "cmd_yaw_ref_deg": setpoint.yaw_ref * RAD_TO_DEG,
            "yaw_ctrl_on": bool(setpoint.flags
                                & proto.CmdSetpoint.FLAG_YAW_REF_VALID),
        }

        # Position モードの診断列
        filtered = meta.get("filtered_pos")
        if filtered is not None:
            row["pos_x"], row["pos_y"], row["pos_z"] = filtered
        raw = meta.get("raw_pos")
        if raw is not None:
            row["raw_pos_x"], row["raw_pos_y"], row["raw_pos_z"] = raw
        pid_components = meta.get("pid_components")
        if pid_components is not None:
            row["pid_x_p"] = pid_components["x"]["p"]
            row["pid_x_i"] = pid_components["x"]["i"]
            row["pid_x_d"] = pid_components["x"]["d"]
            row["pid_y_p"] = pid_components["y"]["p"]
            row["pid_y_i"] = pid_components["y"]["i"]
            row["pid_y_d"] = pid_components["y"]["d"]
        for key in ("error_x", "error_y", "target_x", "target_y", "target_z",
                    "data_valid", "control_active", "mocap_dropout",
                    "is_outlier", "used_prediction", "confidence",
                    "consecutive_outliers", "data_source", "filter_threshold",
                    "tracking_valid", "rb_error", "frame_number", "marker_count",
                    "frame_dt_ms", "mocap_age_ms",
                    "mocap_yaw_deg", "traj_mode", "traj_phase_rad"):
            if key in meta:
                row[key] = meta[key]
        if "marker_count" in meta:
            row["rb_marker_count"] = meta["marker_count"]

        # 最新テレメトリのスナップショット
        with self._lock:
            tlm = self._tlm_state
            tlm_t = self._tlm_state_t
        if tlm is not None and tlm_t is not None:
            row.update({
                "tlm_age_ms": (self._clock() - tlm_t) * MS_PER_S,
                "tlm_seq_echo": tlm.seq_echo,
                "tlm_elapsed_ms": tlm.elapsed_ms,
                "tlm_state": tlm.state,
                "tlm_state_name": _state_name(tlm.state),
                "tlm_flags": tlm.flags,
                "tlm_reason": tlm.reason,
                "tlm_reason_name": _reason_name(tlm.reason),
                "tlm_roll_rad": tlm.roll,
                "tlm_pitch_rad": tlm.pitch,
                "tlm_yaw_rad": tlm.yaw,
                "tlm_p_rad_s": tlm.p,
                "tlm_q_rad_s": tlm.q,
                "tlm_r_rad_s": tlm.r,
                "tlm_roll_ref_rad": tlm.roll_ref,
                "tlm_pitch_ref_rad": tlm.pitch_ref,
                "tlm_alt_ref_m": tlm.alt_ref,
                "tlm_altitude_tof_m": tlm.altitude_tof,
                "tlm_altitude_est_m": tlm.altitude_est,
                "tlm_alt_velocity_m_s": tlm.alt_velocity,
                "tlm_z_dot_ref_m_s": tlm.z_dot_ref,
                "tlm_voltage_v": tlm.voltage,
                "tlm_duty_fr": tlm.duty_fr,
                "tlm_duty_fl": tlm.duty_fl,
                "tlm_duty_rr": tlm.duty_rr,
                "tlm_duty_rl": tlm.duty_rl,
                "tlm_ax_g": tlm.ax,
                "tlm_ay_g": tlm.ay,
                "tlm_az_g": tlm.az,
                "tlm_loop_dt_us": tlm.loop_dt_us,
                # v2: TLM_STATE 末尾拡張(ヨー推定/FF 診断)
                "tlm_yaw_est_rad": tlm.yaw_est_rad,
                "tlm_yaw_gyro_int_rad": tlm.yaw_gyro_int_rad,
                "tlm_yaw_ref_rad": tlm.yaw_ref_rad,
                "tlm_current_a": tlm.current_a,
                "tlm_db_hat_x_ut": tlm.db_hat_x_ut,
                "tlm_db_hat_y_ut": tlm.db_hat_y_ut,
                "tlm_bm_x_ut": tlm.bm_x_ut,
                "tlm_bm_y_ut": tlm.bm_y_ut,
                "tlm_nis": tlm.nis,
                "tlm_ffg": tlm.ffg,
                "tlm_ff_status": tlm.ff_status,
            })
        return row

    # ==================================================================
    # 内部: 受信ハンドラ(RXスレッド上: 状態更新とキュー投入のみ)
    # ==================================================================

    def _tlm_state_snapshot(self) -> tuple[Optional[proto.TlmState],
                                           Optional[float]]:
        """最新 TLM_STATE と受信からの経過秒(CalibrationManager が参照)。"""
        with self._lock:
            tlm = self._tlm_state
            tlm_t = self._tlm_state_t
        age = None if tlm_t is None else (self._clock() - tlm_t)
        return tlm, age

    def _on_tlm_state(self, frame: proto.Frame) -> None:
        try:
            tlm = proto.TlmState.from_payload(frame.payload)
        except ValueError:
            return
        with self._lock:
            self._tlm_state = tlm
            self._tlm_state_t = self._clock()
        self._update_phase_from_drone(tlm.state, tlm.flags)

    def _on_tlm_event(self, frame: proto.Frame) -> None:
        try:
            event = proto.TlmEvent.from_payload(frame.payload)
        except ValueError:
            return
        # STOP 再送待ちの解除(LANDING / WAIT イベント)
        if event.state in (proto.FlightState.LANDING, proto.FlightState.WAIT):
            with self._lock:
                self._stop_pending = None
        # START 拒否 → armed を解除
        if event.reason in _START_REJECT_REASONS:
            with self._lock:
                if self._phase == PHASE_ARMED:
                    self._phase = PHASE_CONNECTED
                    self._armed_since = None
            self.warn(f"離陸拒否: {_reason_name(event.reason)}")
        self._update_phase_from_drone(event.state, 0)
        self.events.put(_json_safe({
            "type": "event",
            "data": {
                "state": event.state,
                "state_name": _state_name(event.state),
                "prev_state": event.prev_state,
                "prev_state_name": _state_name(event.prev_state),
                "reason": event.reason,
                "reason_name": _reason_name(event.reason),
                "flags": event.flags,
                "voltage": event.voltage,
            },
        }))

    def _on_log_text(self, frame: proto.Frame) -> None:
        try:
            log = proto.LogText.from_payload(frame.payload)
        except ValueError:
            return
        origin = LOG_ORIGIN_NAMES.get(log.origin, f"origin{log.origin}")
        self.events.put({"type": "log", "origin": origin, "line": log.text})

    def _on_rly_stats(self, frame: proto.Frame) -> None:
        try:
            stats = proto.RlyStats.from_payload(frame.payload)
        except ValueError:
            return
        with self._lock:
            self._rly_stats = stats
            self._rly_stats_t = self._clock()

    def _on_tlm_exp(self, frame: proto.Frame) -> None:
        """TLM_EXP(実験テレメトリ)→ ExperimentHub へ(RXスレッド上)。"""
        try:
            tlm = proto.TlmExp.from_payload(frame.payload)
        except ValueError:
            return
        self.experiment.on_tlm_exp(tlm, frame.seq)

    def _on_tlm_cal_data(self, frame: proto.Frame) -> None:
        """TLM_CAL_DATA → CalibrationManager へ(RXスレッド上)。"""
        try:
            cal = proto.TlmCalData.from_payload(frame.payload)
        except ValueError:
            return
        self.calibration.on_cal_data(cal)

    def _update_phase_from_drone(self, state: int, flags: int) -> None:
        """ドローンの報告状態から armed/flying/connected フェーズを更新する。"""
        flying_flag = bool(flags & proto.TlmState.FLAG_FLYING)
        in_flight = flying_flag or state in _IN_FLIGHT_STATES
        deactivate_control = False
        with self._lock:
            if self._phase == PHASE_ARMED:
                if in_flight:
                    self._phase = PHASE_FLYING
                    self._armed_since = None
                elif (state in _ON_GROUND_STATES
                      and self._armed_since is not None
                      and self._clock() - self._armed_since > self._start_grace_s):
                    # START 後も離陸へ遷移しない → 受理されなかったとみなす
                    self._phase = PHASE_CONNECTED
                    self._armed_since = None
                    deactivate_control = True
            elif self._phase == PHASE_FLYING and not in_flight \
                    and state in _ON_GROUND_STATES:
                self._phase = PHASE_CONNECTED
                deactivate_control = True
            elif self._phase == PHASE_CONNECTED and in_flight:
                # 接続先の機体が既に飛行中(例: ホバリング中に PC サーバを再起動
                # して再接続)。connected のままだと飛行ガード(モード/プロファイル
                # 変更・start の拒否)が素通りするため flying へ昇格する。
                # START 猶予(_armed_since)の判定は armed フェーズ限定なので
                # この昇格とは干渉しない。
                self._phase = PHASE_FLYING
        if deactivate_control:
            self.position.set_control_active(False)

    # ==================================================================
    # 内部: 監視スレッド(STOP再送 / MoCap途絶 / リンク断)
    # ==================================================================

    def _supervisor_loop(self) -> None:
        run_paced_loop(self._supervisor_stop, self._clock,
                       self._supervisor_period_s, self.supervise)

    def supervise(self, now: float) -> None:
        """フェイルセーフ監視の1周期(テストから直接呼べる)。"""
        # --- シリアル切断: セッションを安全に畳む ---
        with self._lock:
            link_lost = self._link_lost
        if link_lost:
            with self._lock:
                self._link_lost = False
            self._supervisor_stop.set()
            # UI コマンドと teardown が交錯しないよう直列化する
            with self._command_lock:
                self._teardown_session("シリアル切断によりセッションを終了しました")
            return

        # --- STOP 再送(600ms 以内に LANDING/WAIT イベントなし) ---
        resend_stop = False
        with self._lock:
            pending = self._stop_pending
            if pending is not None and now >= pending["deadline"]:
                if pending["resends"] < self._stop_max_retries:
                    pending["resends"] += 1
                    pending["deadline"] = now + self._stop_ack_timeout_s
                    resend_stop = True
                    resend_count = pending["resends"]
                else:
                    self._stop_pending = None
                    self._warn_locked("CMD_STOP への応答がありません(再送上限到達)")
        if resend_stop:
            self.warn(f"CMD_STOP 応答なし → 再送 "
                      f"({resend_count}/{self._stop_max_retries})")
            self._send_stop()

        # --- MoCap 途絶ポリシー(Position モード) ---
        with self._lock:
            mode = self._mode
            phase = self._phase
        if mode == MODE_POSITION and phase in (PHASE_ARMED, PHASE_FLYING):
            age = self.position.mocap_age_s(now)
            dropped = age is None or age > self._mocap_dropout_level_s
            warn_dropout = False
            send_stop = False
            with self._lock:
                if dropped:
                    if not self._mocap_warned:
                        self._mocap_warned = True
                        warn_dropout = True
                    if (age is None or age > self._mocap_dropout_stop_s) \
                            and not self._mocap_stop_sent:
                        self._mocap_stop_sent = True
                        send_stop = True
                else:
                    self._mocap_warned = False
                    self._mocap_stop_sent = False
            if warn_dropout:
                self.warn("MoCap 途絶 >300ms: セットポイントを水平に固定します")
            if send_stop:
                self.warn("MoCap 途絶 >2s: CMD_STOP を送信します(自動着陸)")
                self.stop()

        # --- 複数機モード(スロットごとの STOP 再送 / MoCap 途絶 / 猶予) ---
        self.multi.supervise(now)

    def _send_stop(self) -> bool:
        try:
            self.serial.send(proto.MsgType.CMD_STOP)
        except SerialLinkError as exc:
            self.warn(f"CMD_STOP 送信失敗: {exc}")
            return False
        self.info("CMD_STOP 送信")
        return True

    # ==================================================================
    # UI への通知 / スナップショット
    # ==================================================================

    def info(self, message: str) -> None:
        self.events.put({"type": "log", "origin": LOG_ORIGIN_SERVER,
                         "line": message})

    def warn(self, message: str) -> None:
        self.events.put({"type": "log", "origin": LOG_ORIGIN_SERVER,
                         "line": f"[警告] {message}"})

    def _warn_locked(self, message: str) -> None:
        """self._lock 保持中でも安全な警告(queue.put はブロックしない)。"""
        self.events.put({"type": "log", "origin": LOG_ORIGIN_SERVER,
                         "line": f"[警告] {message}"})

    def get_state_snapshot(self) -> dict:
        """WebSocket 20Hz 配信用の状態スナップショット(UI 単位系: deg/m)。"""
        now = self._clock()
        with self._lock:
            tlm = self._tlm_state
            tlm_t = self._tlm_state_t
            mode = self._mode
            phase = self._phase
            airframe = self._airframe
            logging_enabled = self._logging_enabled
            relay_target_ok = self._relay_target_ok
            rly_stats = self._rly_stats
            rly_stats_t = self._rly_stats_t
            mocap_warned = self._mocap_warned
            experiment_active = self._experiment_active

        # drone: TLM_STATE 全フィールド(角度・角速度は deg 換算)+ fresh。
        # 一度も受信していない間は null — ゼロ値を実測値として配ると UI 側で
        # 「INIT・0.00V(危険表示)」と本物の異常が区別できなくなるため。
        if tlm is None or tlm_t is None:
            drone = None
        else:
            drone = {
                "seq_echo": tlm.seq_echo,
                "elapsed_ms": tlm.elapsed_ms,
                "state": tlm.state,
                "state_name": _state_name(tlm.state),
                "flags": tlm.flags,
                "low_voltage": bool(tlm.flags & proto.TlmState.FLAG_LOW_VOLTAGE),
                "setpoint_fresh": bool(tlm.flags & proto.TlmState.FLAG_SETPOINT_FRESH),
                "flying": bool(tlm.flags & proto.TlmState.FLAG_FLYING),
                "reason": tlm.reason,
                "reason_name": _reason_name(tlm.reason),
                "roll": tlm.roll * RAD_TO_DEG,
                "pitch": tlm.pitch * RAD_TO_DEG,
                "yaw": tlm.yaw * RAD_TO_DEG,
                "p": tlm.p * RAD_TO_DEG,
                "q": tlm.q * RAD_TO_DEG,
                "r": tlm.r * RAD_TO_DEG,
                "roll_ref": tlm.roll_ref * RAD_TO_DEG,
                "pitch_ref": tlm.pitch_ref * RAD_TO_DEG,
                "alt_ref": tlm.alt_ref,
                "altitude_tof": tlm.altitude_tof,
                "altitude_est": tlm.altitude_est,
                "alt_velocity": tlm.alt_velocity,
                "z_dot_ref": tlm.z_dot_ref,
                "voltage": tlm.voltage,
                "duty_fr": tlm.duty_fr,
                "duty_fl": tlm.duty_fl,
                "duty_rr": tlm.duty_rr,
                "duty_rl": tlm.duty_rl,
                "ax": tlm.ax,
                "ay": tlm.ay,
                "az": tlm.az,
                "loop_dt_us": tlm.loop_dt_us,
                "fresh": (now - tlm_t) <= self._telemetry_fresh_s,
                # v2: ヨー推定/FF 診断(角度は deg 換算、UI 単位規約)
                "yaw_est": tlm.yaw_est_rad * RAD_TO_DEG,
                "yaw_gyro_int": tlm.yaw_gyro_int_rad * RAD_TO_DEG,
                "yaw_ref": tlm.yaw_ref_rad * RAD_TO_DEG,
                "current_a": tlm.current_a,
                "db_hat_x_ut": tlm.db_hat_x_ut,
                "db_hat_y_ut": tlm.db_hat_y_ut,
                "bm_x_ut": tlm.bm_x_ut,
                "bm_y_ut": tlm.bm_y_ut,
                "nis": tlm.nis,
                "ffg": tlm.ffg,
                "ff_status": tlm.ff_status,
                "ff_mode": tlm.ff_status & proto.TlmState.FF_STATUS_FF_MODE_MASK,
                "est_mode_ekf": bool(tlm.ff_status
                                     & proto.TlmState.FF_STATUS_EST_EKF),
                "anchor_valid": bool(tlm.ff_status
                                     & proto.TlmState.FF_STATUS_ANCHOR_VALID),
                "ffcal_loaded": bool(tlm.ff_status
                                     & proto.TlmState.FF_STATUS_FFCAL_LOADED),
                "yaw_ctrl_active": bool(
                    tlm.ff_status & proto.TlmState.FF_STATUS_YAW_CTRL_ACTIVE),
                "mag_fresh": bool(tlm.ff_status
                                  & proto.TlmState.FF_STATUS_MAG_FRESH),
            }

        # mocap: Position モード時のみ(リジッドボディ未検出なら null)
        mocap = self.position.mocap_snapshot(now) if mode == MODE_POSITION else None

        # setpoint: 現在の整形済みセットポイント(バイアス加算前、UI単位)
        trajectory = None
        if mode == MODE_POSITION:
            roll_rad, pitch_rad, alt_m = self.position.current_setpoint()
            yaw_rad, yaw_ctrl_on = self.position.yaw_setpoint()
            tx, ty, tz = self.position.get_target()
            target = {"x": tx, "y": ty, "z": tz}
            trajectory = self.position.trajectory_snapshot()
        else:
            roll_rad, pitch_rad, alt_m = self.posture.current_setpoint()
            yaw_rad, yaw_ctrl_on = self.posture.yaw_setpoint()
            target = None

        relay_stats = None
        if rly_stats is not None:
            relay_stats = {
                "up_frames": rly_stats.up_frames,
                "down_frames": rly_stats.down_frames,
                "crc_errors": rly_stats.crc_errors,
                "cobs_errors": rly_stats.cobs_errors,
                "espnow_send_fail": rly_stats.espnow_send_fail,
                "overflow_drops": rly_stats.overflow_drops,
            }

        # logger.file_path はプロパティを2回読むと(各読みが別個にロックを
        # 取るため)stop() と競合して None になり得る。一度だけ読んで使う
        # (20Hz 配信タスクを AttributeError で殺さないため)。
        log_path = self.logger.file_path

        session = {
            "mode": mode,
            "phase": phase,
            "serial_connected": self.serial.is_connected,
            "airframe": airframe["name"] if airframe else None,
            "relay_target_ok": relay_target_ok,
            "logging": logging_enabled,
            "log_file": log_path.name if log_path else None,
            "target": target,
            "setpoint": {
                "roll_deg": roll_rad * RAD_TO_DEG,
                "pitch_deg": pitch_rad * RAD_TO_DEG,
                "alt_m": alt_m,
                "yaw_deg": yaw_rad * RAD_TO_DEG,
            },
            "yaw_ctrl_on": yaw_ctrl_on,
            "trajectory": trajectory,
            "latency_ms": self.serial.latency_ms,
            "relay_stats": relay_stats,
            # リレー鮮度: RLY_STATS(1Hz)の受信時刻ベース。counter が静止していても
            # フレームが届いている限り true(UI のリレーリンク表示が使用)
            "relay_fresh": (rly_stats_t is not None
                            and (now - rly_stats_t) <= self._relay_stats_fresh_s),
            "link_stats": self.serial.stats(),
            "mocap_dropout": mocap_warned,
        }

        # 実験モードの状態(UI の Experiment タブが 20Hz で参照)
        experiment = None
        if mode == MODE_EXPERIMENT:
            exp_sample, exp_age = self.experiment.latest_sample()
            experiment = {
                "active": experiment_active,
                "motor": self.experiment.motor_status(),
                "sweep": self.experiment.sweep.status(),
                "sequence": self.experiment.sequence.status(),
                "cal3d": self.experiment.cal3d_status(),
                "exp_age_s": exp_age,
                "exp": None if exp_sample is None else {
                    "current_a": exp_sample["current_a"],
                    "vbat_v": exp_sample["vbat_v"],
                    "cv": exp_sample["cv"],
                    "b_raw": exp_sample["b_raw"],
                    "b_cal": exp_sample["b_cal"],
                    "imu_temp_c": exp_sample["imu_temp_c"],
                    # 加速度 [g](フィルタ後・較正前 = 6面キャリブの入力そのもの)
                    "ax": exp_sample["ax"],
                    "ay": exp_sample["ay"],
                    "az": exp_sample["az"],
                    "roll_deg": exp_sample["roll_rad"] * RAD_TO_DEG,
                    "pitch_deg": exp_sample["pitch_rad"] * RAD_TO_DEG,
                    "yaw_deg": exp_sample["yaw_rad"] * RAD_TO_DEG,
                    "duty_cmd": exp_sample["duty_cmd_fw"],
                    "motors_mask": exp_sample["motors_mask_fw"],
                    "mag_fresh": exp_sample["mag_fresh"],
                    "motors_running": exp_sample["motors_running"],
                },
            }
        session["experiment"] = experiment

        # 複数機モードの状態(UI の複数機タブが 20Hz で参照)
        session["multi"] = (self.multi.snapshot(now)
                            if mode == MODE_MULTI else None)

        # TLM 由来の非有限 float を一括で None 化(WS の JSON を壊さない)
        return _json_safe({
            "type": "state",
            "data": {"drone": drone, "mocap": mocap, "session": session}})
