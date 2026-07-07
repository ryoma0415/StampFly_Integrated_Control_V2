"""複数機同時制御(MODE_MULTI): 選択・目標・一斉開始/停止・フェイルセーフ。"""

from __future__ import annotations

import pytest

import stampfly_protocol as proto

from core.session import MODE_MULTI, MODE_POSTURE

from conftest import halt_supervisor
from fakes import make_ack_responder, make_pose, wait_until

# 複数機テスト用プロファイル(全機 ch1、rigid_body_id 設定済み)
MULTI_AIRFRAMES = [
    {"name": "drone 1", "mac": "48:CA:43:3A:51:30", "wifi_channel": 1,
     "roll_bias_deg": 1.0, "pitch_bias_deg": 0.0, "default_alt_m": 0.3,
     "rigid_body_id": 1, "notes": ""},
    {"name": "drone 2", "mac": "48:CA:43:38:A1:CC", "wifi_channel": 1,
     "roll_bias_deg": 0.0, "pitch_bias_deg": 0.0, "default_alt_m": 0.3,
     "rigid_body_id": 2, "notes": ""},
    {"name": "drone 3", "mac": "48:CA:43:38:F0:60", "wifi_channel": 1,
     "roll_bias_deg": 0.0, "pitch_bias_deg": 0.0, "default_alt_m": 0.3,
     "rigid_body_id": 3, "notes": ""},
    {"name": "no-rb", "mac": "48:CA:43:38:9C:88", "wifi_channel": 1,
     "roll_bias_deg": 0.0, "pitch_bias_deg": 0.0, "default_alt_m": 0.3,
     "rigid_body_id": None, "notes": ""},
    {"name": "other-ch", "mac": "34:B7:DA:5D:27:68", "wifi_channel": 6,
     "roll_bias_deg": 0.0, "pitch_bias_deg": 0.0, "default_alt_m": 0.3,
     "rigid_body_id": 5, "notes": ""},
]


class FakeMultiRelay:
    """複数機対応リレーのフェイク(RLY_SET_PEERS/MUX_UP を処理)。"""

    def __init__(self) -> None:
        self._single = make_ack_responder()
        self.peers: list[tuple[bytes, int]] = []
        self.mux_up: list[tuple[int, proto.Frame]] = []

    def __call__(self, frame: proto.Frame):
        if frame.type == proto.MsgType.RLY_SET_TARGET:
            return self._single(frame)
        if frame.type == proto.MsgType.RLY_SET_PEERS:
            req = proto.RlySetPeers.from_payload(frame.payload)
            self.peers = [(bytes(p.mac), p.tlm_state_div) for p in req.peers]
            ack = proto.RlyPeersAck(
                status=proto.RlyPeersAck.STATUS_OK, count=len(req.peers),
                wifi_channel=req.wifi_channel)
            return [(proto.MsgType.RLY_PEERS_ACK, ack.to_payload())]
        if frame.type == proto.MsgType.RLY_MUX_UP:
            node_id, inner_bytes = proto.mux_unwrap(frame.payload)
            status, inner = proto.parse_frame(inner_bytes)
            assert status is proto.ParseStatus.OK   # 不正な内側フレームは契約違反
            self.mux_up.append((node_id, inner))
        return None

    def uplinks(self, node_id: int, msg_type: int) -> list[proto.Frame]:
        return [f for n, f in self.mux_up
                if n == node_id and f.type == msg_type]


@pytest.fixture
def multi_session(session_factory):
    """接続済み+MODE_MULTI の SessionManager(フェイクマルチリレーつき)。"""
    relay = FakeMultiRelay()
    session, transport, clock = session_factory(
        airframes=[dict(p) for p in MULTI_AIRFRAMES], responder=relay)
    session.connect("FAKE")
    halt_supervisor(session)
    assert session.set_mode(MODE_MULTI)
    return session, transport, clock, relay


def select_two(session):
    assert session.multi_select(["drone 1", "drone 2"]) is True
    return session.multi._slots


def feed_fresh_mocap(session, clock, positions=None):
    """全スロットへ新鮮な pose を直接注入する(NatNet 実時間刻印を回避)。"""
    positions = positions or {}
    for slot in session.multi._slots:
        x, y, z = positions.get(slot.name, (0.0, 0.0, 0.0))
        for _ in range(3):
            slot.controller.on_mocap_pose(
                make_pose(x=x, y=y, z=z, t=clock()))


class TestMultiSelect:
    def test_select_sets_peers_and_builds_slots(self, multi_session):
        session, transport, clock, relay = multi_session
        select_two(session)

        # リレーへ SET_PEERS(index = node_id、MAC は選択順)
        assert [m.hex() for m, _ in relay.peers] == \
            ["48ca433a5130", "48ca4338a1cc"]
        # スナップショットに2機ぶんのスロット
        snap = session.get_state_snapshot()["data"]["session"]["multi"]
        assert snap["active"] is True
        assert [d["name"] for d in snap["drones"]] == ["drone 1", "drone 2"]
        assert [d["node_id"] for d in snap["drones"]] == [0, 1]
        assert all(d["phase"] == "idle" for d in snap["drones"])
        # 選択直後から各機宛の CMD_SETPOINT(ハートビート)が MUX で流れる
        assert wait_until(
            lambda: relay.uplinks(0, proto.MsgType.CMD_SETPOINT)
            and relay.uplinks(1, proto.MsgType.CMD_SETPOINT))

    def test_select_rejects_bad_inputs(self, multi_session):
        session, transport, clock, relay = multi_session
        # 1機は不可(min_drones=2)
        assert session.multi_select(["drone 1"]) is False
        # rigid_body_id 未設定
        assert session.multi_select(["drone 1", "no-rb"]) is False
        # チャネル不一致
        assert session.multi_select(["drone 1", "other-ch"]) is False
        # 重複
        assert session.multi_select(["drone 1", "drone 1"]) is False
        assert session.multi.active is False

    def test_select_requires_multi_mode(self, session_factory):
        relay = FakeMultiRelay()
        session, transport, clock = session_factory(
            airframes=[dict(p) for p in MULTI_AIRFRAMES], responder=relay)
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.multi_select(["drone 1", "drone 2"]) is False

    def test_leaving_multi_restores_single_target(self, multi_session):
        session, transport, clock, relay = multi_session
        select_two(session)
        transport.clear_sent()
        assert session.set_mode(MODE_POSTURE)
        # 単機ターゲット(選択中プロファイル)への復帰 = RLY_SET_TARGET 送信
        assert len(transport.frames_of_type(proto.MsgType.RLY_SET_TARGET)) >= 1
        assert session.multi.active is False


class TestMultiStart:
    def test_start_requires_targets_and_fresh_mocap(self, multi_session):
        session, transport, clock, relay = multi_session
        select_two(session)
        # 目標未設定
        assert session.multi_start() is False
        assert session.multi_target("drone 1", 0.5, 0.5, 0.4) is True
        assert session.multi_target("drone 2", -0.5, -0.5, 0.4) is True
        # MoCap 未受信
        assert session.multi_start() is False
        feed_fresh_mocap(session, clock)
        assert session.multi_start() is True
        # 各機へ CMD_START がノード宛で届く
        assert len(relay.uplinks(0, proto.MsgType.CMD_START)) == 1
        assert len(relay.uplinks(1, proto.MsgType.CMD_START)) == 1
        snap = session.get_state_snapshot()["data"]["session"]["multi"]
        assert all(d["phase"] == "armed" for d in snap["drones"])

    def test_start_rejects_close_targets(self, multi_session):
        session, transport, clock, relay = multi_session
        select_two(session)
        # min_target_separation_m=0.5 未満の XY 距離は拒否
        assert session.multi_target("drone 1", 0.0, 0.0, 0.4) is True
        assert session.multi_target("drone 2", 0.3, 0.0, 0.4) is True
        feed_fresh_mocap(session, clock)
        assert session.multi_start() is False
        assert relay.uplinks(0, proto.MsgType.CMD_START) == []

    def test_target_xy_clamp(self, multi_session):
        session, transport, clock, relay = multi_session
        select_two(session)
        # target_xy_abs_max_m=2.0 の外は拒否
        assert session.multi_target("drone 1", 2.5, 0.0, 0.4) is False
        assert session.multi_target("drone 1", 1.5, -1.5, 0.4) is True

    def test_start_is_rejected_for_single_mode_command(self, multi_session):
        session, transport, clock, relay = multi_session
        select_two(session)
        # 単機用の start は複数機モードでは使えない
        assert session.start() is False


class TestMultiStopAndFailsafe:
    def _armed_session(self, multi_session):
        session, transport, clock, relay = multi_session
        select_two(session)
        session.multi_target("drone 1", 0.5, 0.5, 0.4)
        session.multi_target("drone 2", -0.5, -0.5, 0.4)
        feed_fresh_mocap(session, clock)
        assert session.multi_start() is True
        return session, transport, clock, relay

    def test_stop_sends_to_all_nodes(self, multi_session):
        session, transport, clock, relay = self._armed_session(multi_session)
        assert session.stop() is True
        assert len(relay.uplinks(0, proto.MsgType.CMD_STOP)) == 1
        assert len(relay.uplinks(1, proto.MsgType.CMD_STOP)) == 1

    def test_stop_resend_until_landing_event(self, multi_session):
        session, transport, clock, relay = self._armed_session(multi_session)
        assert session.stop() is True
        # node 0 だけ LANDING イベントを返す(MUX_DOWN)
        event = proto.TlmEvent(state=proto.FlightState.LANDING,
                               prev_state=proto.FlightState.HOVER,
                               reason=proto.Reason.STOP_CMD, flags=0,
                               voltage=3.8)
        inner = proto.pack_frame(proto.MsgType.TLM_EVENT, 1,
                                 event.to_payload())
        transport.push(proto.MsgType.RLY_MUX_DOWN, proto.mux_wrap(0, inner))
        assert wait_until(
            lambda: session.multi._slots[0].stop_pending is None)

        # 期限超過で node 1 のみ再送される
        clock.advance(0.7)   # stop_ack_timeout_s=0.6
        session.multi.supervise(clock())
        assert len(relay.uplinks(0, proto.MsgType.CMD_STOP)) == 1
        assert len(relay.uplinks(1, proto.MsgType.CMD_STOP)) == 2

    def test_mocap_dropout_stops_only_affected_drone(self, multi_session):
        session, transport, clock, relay = self._armed_session(multi_session)
        # 両機 flying へ(TLM_STATE を MUX_DOWN で注入)
        for node in (0, 1):
            tlm = proto.TlmState(state=proto.FlightState.HOVER,
                                 flags=proto.TlmState.FLAG_FLYING)
            inner = proto.pack_frame(proto.MsgType.TLM_STATE, 1,
                                     tlm.to_payload())
            transport.push(proto.MsgType.RLY_MUX_DOWN,
                           proto.mux_wrap(node, inner))
        assert wait_until(lambda: all(
            s.phase == "flying" for s in session.multi._slots))

        # drone 1 のみ MoCap を更新し続け、drone 2 は途絶させる
        clock.advance(2.5)
        slot1 = session.multi._slots[0]
        for _ in range(3):
            slot1.controller.on_mocap_pose(make_pose(x=0.5, y=0.5, t=clock()))
        session.multi.supervise(clock())
        # 途絶した drone 2(node 1)にだけ CMD_STOP が飛ぶ
        assert relay.uplinks(0, proto.MsgType.CMD_STOP) == []
        assert len(relay.uplinks(1, proto.MsgType.CMD_STOP)) == 1

    def test_flying_guard_blocks_mode_change(self, multi_session):
        session, transport, clock, relay = self._armed_session(multi_session)
        assert session.set_mode(MODE_POSTURE) is False

    def test_teardown_releases_slots(self, multi_session):
        session, transport, clock, relay = self._armed_session(multi_session)
        session.disconnect()
        assert session.multi.active is False
        assert session.multi._slots == []


class TestMultiDivergence:
    """XY 誤差の持続的発散の検知(rigid_body_id 取り違えの最終防衛線)。"""

    def _flying_session(self, multi_session, positions):
        """目標設定 → 一斉開始 → 両機 flying まで進めた session を返す。"""
        session, transport, clock, relay = multi_session
        select_two(session)
        session.multi_target("drone 1", 0.5, 0.5, 0.4)
        session.multi_target("drone 2", -0.5, -0.5, 0.4)
        feed_fresh_mocap(session, clock, positions=positions)
        assert session.multi_start() is True
        for node in (0, 1):
            tlm = proto.TlmState(state=proto.FlightState.HOVER,
                                 flags=proto.TlmState.FLAG_FLYING)
            inner = proto.pack_frame(proto.MsgType.TLM_STATE, 1,
                                     tlm.to_payload())
            transport.push(proto.MsgType.RLY_MUX_DOWN,
                           proto.mux_wrap(node, inner))
        assert wait_until(lambda: all(
            s.phase == "flying" for s in session.multi._slots))
        return session, transport, clock, relay

    def _run_supervise(self, session, clock, positions, steps, dt=0.2):
        """MoCap を新鮮に保ちながら(dt < 300ms)supervise を回す。"""
        for _ in range(steps):
            clock.advance(dt)
            feed_fresh_mocap(session, clock, positions=positions)
            session.multi.supervise(clock())

    def test_divergence_stops_only_affected_drone(self, multi_session):
        # drone 1 の RB が目標から遠い位置(誤差 ≈2.1m > 1.0m)を報告し続ける
        positions = {"drone 1": (2.0, 2.0, 0.0), "drone 2": (-0.5, -0.5, 0.0)}
        session, transport, clock, relay = self._flying_session(
            multi_session, positions)
        # divergence_hold_s=1.0 超過まで継続(0.2s × 7 = 1.2s 経過で発火)
        self._run_supervise(session, clock, positions, steps=7)
        assert len(relay.uplinks(0, proto.MsgType.CMD_STOP)) == 1
        assert relay.uplinks(1, proto.MsgType.CMD_STOP) == []
        # 閉ループは切られ、STOP 再送監視が仕掛かる
        slot = session.multi._slots[0]
        assert slot.controller.control_active is False
        assert slot.stop_pending is not None
        # LANDING イベントで再送監視を解除(機体が STOP に応答)
        event = proto.TlmEvent(state=proto.FlightState.LANDING,
                               prev_state=proto.FlightState.HOVER,
                               reason=proto.Reason.STOP_CMD, flags=0,
                               voltage=3.8)
        inner = proto.pack_frame(proto.MsgType.TLM_EVENT, 1,
                                 event.to_payload())
        transport.push(proto.MsgType.RLY_MUX_DOWN, proto.mux_wrap(0, inner))
        assert wait_until(lambda: session.multi._slots[0].stop_pending is None)
        # 制御停止後は発散判定から除外され、再発火しない
        self._run_supervise(session, clock, positions, steps=2)
        assert len(relay.uplinks(0, proto.MsgType.CMD_STOP)) == 1

    def test_no_stop_when_error_normal(self, multi_session):
        # 両機とも目標位置に静止(誤差 0)
        positions = {"drone 1": (0.5, 0.5, 0.0), "drone 2": (-0.5, -0.5, 0.0)}
        session, transport, clock, relay = self._flying_session(
            multi_session, positions)
        self._run_supervise(session, clock, positions, steps=10)
        assert relay.uplinks(0, proto.MsgType.CMD_STOP) == []
        assert relay.uplinks(1, proto.MsgType.CMD_STOP) == []

    def test_error_recovery_resets_hold_timer(self, multi_session):
        """hold 未満の一時的な誤差超過は STOP しない(タイマはリセット)。"""
        positions = {"drone 1": (0.5, 0.5, 0.0), "drone 2": (-0.5, -0.5, 0.0)}
        session, transport, clock, relay = self._flying_session(
            multi_session, positions)
        # 目標を動かして誤差 ≈1.8m を 0.6s(< hold 1.0s)だけ作る
        assert session.multi_target("drone 1", 1.8, 1.8, 0.4) is True
        self._run_supervise(session, clock, positions, steps=4)
        # 誤差解消(目標を現在位置へ戻す)→ タイマリセット
        assert session.multi_target("drone 1", 0.5, 0.5, 0.4) is True
        self._run_supervise(session, clock, positions, steps=1)
        # 再度 0.6s の誤差超過 — 通算 1.2s だが連続 1.0s 未満なので STOP しない
        assert session.multi_target("drone 1", 1.8, 1.8, 0.4) is True
        self._run_supervise(session, clock, positions, steps=4)
        assert relay.uplinks(0, proto.MsgType.CMD_STOP) == []


class TestMultiGuards:
    """レビュー指摘由来のガード(飛行中のピア表破壊・目標間隔 TOCTOU 等)。"""

    def _armed_session(self, multi_session):
        session, transport, clock, relay = multi_session
        select_two(session)
        session.multi_target("drone 1", 0.5, 0.5, 0.4)
        session.multi_target("drone 2", -0.5, -0.5, 0.4)
        feed_fresh_mocap(session, clock)
        assert session.multi_start() is True
        return session, transport, clock, relay

    def test_select_airframe_refused_while_multi_selected(self, multi_session):
        """RLY_SET_TARGET はマルチピア表をクリアするため選択中は常に拒否。"""
        session, transport, clock, relay = multi_session
        select_two(session)
        transport.clear_sent()
        assert session.select_airframe("drone 3") is False
        assert transport.frames_of_type(proto.MsgType.RLY_SET_TARGET) == []

    def test_update_airframes_refused_for_flying_multi_profile(
            self, multi_session, tmp_path, monkeypatch):
        from core import config as cfg
        monkeypatch.setattr(cfg, "AIRFRAMES_CONFIG_PATH",
                            tmp_path / "airframes.json")
        session, transport, clock, relay = self._armed_session(multi_session)
        edited = [dict(p) for p in MULTI_AIRFRAMES]
        edited[0]["roll_bias_deg"] = 5.0   # 飛行中の drone 1 を変更
        ok, error = session.update_airframes(edited)
        assert ok is False and "複数機" in error
        # 飛行中でない機体のみの変更は許可される
        edited = [dict(p) for p in MULTI_AIRFRAMES]
        edited[2]["roll_bias_deg"] = 5.0   # drone 3(非選択)
        ok, error = session.update_airframes(edited)
        assert ok is True, error

    def test_target_separation_enforced_in_flight(self, multi_session):
        session, transport, clock, relay = self._armed_session(multi_session)
        # 飛行中に drone 1 の目標を drone 2 の目標へ近づける → 拒否
        assert session.multi_target("drone 1", -0.4, -0.4, 0.4) is False
        # 十分離れた目標変更は許可
        assert session.multi_target("drone 1", 0.8, 0.8, 0.4) is True

    def test_tlm_timeout_demotes_silent_armed_slot(self, multi_session):
        """CMD_START 後にテレメトリが一度も来ない機体はタイムアウトで解放。"""
        session, transport, clock, relay = self._armed_session(multi_session)
        clock.advance(3.5)   # multi.tlm_timeout_s=3.0
        session.multi.supervise(clock())
        assert all(s.phase == "idle" for s in session.multi._slots)
        # ベストエフォートの CMD_STOP が各機へ送られる
        assert len(relay.uplinks(0, proto.MsgType.CMD_STOP)) == 1
        assert len(relay.uplinks(1, proto.MsgType.CMD_STOP)) == 1

    def test_start_rejects_airborne_rigid_body(self, multi_session):
        """離陸前に z が地上とみなせない機体がいれば開始拒否(ID 取り違え対策)。"""
        session, transport, clock, relay = multi_session
        select_two(session)
        session.multi_target("drone 1", 0.5, 0.5, 0.4)
        session.multi_target("drone 2", -0.5, -0.5, 0.4)
        feed_fresh_mocap(session, clock,
                         positions={"drone 2": (-0.5, -0.5, 0.6)})
        assert session.multi_start() is False
        assert relay.uplinks(0, proto.MsgType.CMD_START) == []
