"""フェイルセーフ(PROTOCOL.md 規範): MoCap途絶 / STOP再送 / シリアル切断。"""

from __future__ import annotations

import pytest

import stampfly_protocol as proto
from core.serial_link import SerialLinkError
from core.session import (
    MODE_POSITION, PHASE_ARMED, PHASE_CONNECTED, PHASE_FLYING, PHASE_IDLE,
)

from conftest import halt_supervisor
from fakes import make_pose, wait_until


def connected_position_session(session_factory):
    """Position モードで armed まで進めたセッションを返す。

    監視スレッドと送信スレッドを止め、supervise()/step() を手動で
    決定的に呼べる状態にする。
    """
    session, transport, clock = session_factory()
    assert session.connect("FAKE")
    halt_supervisor(session)
    assert session.set_mode(MODE_POSITION)
    session.position.on_mocap_pose(make_pose(t=clock()))
    assert session.start()
    assert session._phase == PHASE_ARMED
    session.position.stop()   # 50Hz スレッドを止めて step() を手動駆動
    transport.clear_sent()
    return session, transport, clock


class TestMocapDropout:
    def test_sender_drops_xy_valid_after_300ms(self, server_config,
                                               control_config,
                                               session_factory):
        """途絶 >300ms: CMD_POS_ERR bit2=0(機体側が水平固定)、alt_ref 維持。

        送信は継続し(ハートビート)、実誤差もそのまま送り続ける
        (0 すり替えは復帰1サンプル目の D 項スパイクの原因)。
        """
        session, transport, clock = connected_position_session(session_factory)
        # 有効な mocap で非ゼロ誤差を作る
        session.position.set_target(0.5, 0.0, 0.4)
        for _ in range(5):
            session.position.on_mocap_pose(make_pose(t=clock.advance(0.01)))
        session.position.step(clock())
        session.position.step(clock.advance(0.02))
        sent = [proto.CmdPosErr.from_payload(f.payload)
                for f in transport.frames_of_type(proto.MsgType.CMD_POS_ERR)]
        assert sent, "CMD_POS_ERR が送信されていない"
        assert sent[-1].flags & proto.CmdPosErr.FLAG_XY_ERR_VALID
        assert sent[-1].err_x == pytest.approx(0.5, abs=0.05)

        # 途絶: 350ms 経過後の送信は bit2=0 になる(送信自体は継続)
        clock.advance(0.35)
        before = len(transport.frames_of_type(proto.MsgType.CMD_POS_ERR))
        for _ in range(30):   # alt スルー制限(0.3m/s)が 0.4m に到達するまで
            session.position.step(clock.advance(0.02))
        frames = transport.frames_of_type(proto.MsgType.CMD_POS_ERR)
        assert len(frames) > before        # ハートビートは継続
        last = proto.CmdPosErr.from_payload(frames[-1].payload)
        assert not (last.flags & proto.CmdPosErr.FLAG_XY_ERR_VALID)
        assert last.err_x == pytest.approx(0.5, abs=0.05)   # 実誤差は保持
        assert last.alt_ref == pytest.approx(0.4)           # alt_ref は維持

    def test_supervisor_warns_after_300ms(self, session_factory):
        session, transport, clock = connected_position_session(session_factory)
        clock.advance(0.5)
        session.supervise(clock())
        assert session._mocap_warned is True
        assert session._mocap_stop_sent is False
        assert transport.frames_of_type(proto.MsgType.CMD_STOP) == []

    def test_supervisor_sends_stop_after_2s(self, session_factory):
        session, transport, clock = connected_position_session(session_factory)
        clock.advance(2.1)
        session.supervise(clock())
        stops = transport.frames_of_type(proto.MsgType.CMD_STOP)
        assert len(stops) == 1
        assert session._mocap_stop_sent is True
        # STOP 再送監視も同時に始まる
        assert session._stop_pending is not None

    def test_stop_sent_only_once_per_dropout(self, session_factory):
        session, transport, clock = connected_position_session(session_factory)
        clock.advance(2.1)
        session.supervise(clock())
        clock.advance(0.1)
        session.supervise(clock())
        assert len(transport.frames_of_type(proto.MsgType.CMD_STOP)) == 1

    def test_recovery_clears_dropout_flags(self, session_factory):
        session, transport, clock = connected_position_session(session_factory)
        clock.advance(0.5)
        session.supervise(clock())
        assert session._mocap_warned is True
        # mocap 復帰
        session.position.on_mocap_pose(make_pose(t=clock()))
        session.supervise(clock())
        assert session._mocap_warned is False

    def test_dropout_policy_inactive_when_not_flying(self, session_factory):
        """connected フェーズ(未離陸)では途絶しても STOP しない。"""
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        session.set_mode(MODE_POSITION)
        transport.clear_sent()
        clock.advance(3.0)
        session.supervise(clock())
        assert transport.frames_of_type(proto.MsgType.CMD_STOP) == []


class TestStopRetry:
    def test_stop_resent_up_to_3_times_without_event(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        transport.clear_sent()

        assert session.stop()
        assert len(transport.frames_of_type(proto.MsgType.CMD_STOP)) == 1

        # 600ms ごとに再送(最大3回)
        for expected in (2, 3, 4):
            clock.advance(0.7)
            session.supervise(clock())
            assert len(transport.frames_of_type(proto.MsgType.CMD_STOP)) == expected

        # 上限到達後は再送しない(警告のみ)
        clock.advance(0.7)
        session.supervise(clock())
        clock.advance(0.7)
        session.supervise(clock())
        assert len(transport.frames_of_type(proto.MsgType.CMD_STOP)) == 4
        assert session._stop_pending is None

    def test_landing_event_cancels_retry(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        transport.clear_sent()

        assert session.stop()
        event = proto.TlmEvent(state=proto.FlightState.LANDING,
                               prev_state=proto.FlightState.HOVER,
                               reason=proto.Reason.STOP_CMD, voltage=3.7)
        transport.push(proto.MsgType.TLM_EVENT, event.to_payload())
        assert wait_until(lambda: session._stop_pending is None)

        clock.advance(0.7)
        session.supervise(clock())
        assert len(transport.frames_of_type(proto.MsgType.CMD_STOP)) == 1

    def test_wait_event_also_cancels_retry(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        transport.clear_sent()
        assert session.stop()
        event = proto.TlmEvent(state=proto.FlightState.WAIT,
                               prev_state=proto.FlightState.LANDING,
                               reason=proto.Reason.LANDED, voltage=3.7)
        transport.push(proto.MsgType.TLM_EVENT, event.to_payload())
        assert wait_until(lambda: session._stop_pending is None)


class TestSerialDisconnect:
    def test_write_failure_tears_down_session(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session._phase == PHASE_CONNECTED

        transport.fail_writes = True
        with pytest.raises(SerialLinkError):
            session.serial.send(proto.MsgType.CMD_STOP)
        # 切断フラグが立ち、supervise がセッションを安全に畳む
        session.supervise(clock())
        assert session._phase == PHASE_IDLE
        assert session.serial.is_connected is False

        snapshot = session.get_state_snapshot()
        assert snapshot["data"]["session"]["serial_connected"] is False
        assert snapshot["data"]["session"]["phase"] == "idle"


class TestStartGuards:
    def test_start_refused_without_fresh_mocap(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        session.set_mode(MODE_POSITION)
        transport.clear_sent()
        # mocap 未受信のまま start → 拒否
        assert session.start() is False
        assert transport.frames_of_type(proto.MsgType.CMD_START) == []

    def test_start_refused_when_not_connected(self, session_factory):
        session, transport, clock = session_factory()
        assert session.start() is False

    def test_armed_to_flying_to_connected(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.start()   # posture モードは mocap 不要
        assert session._phase == PHASE_ARMED

        flying = proto.TlmState(state=proto.FlightState.HOVER,
                                flags=proto.TlmState.FLAG_FLYING)
        transport.push(proto.MsgType.TLM_STATE, flying.to_payload())
        assert wait_until(lambda: session._phase == PHASE_FLYING)

        landed = proto.TlmState(state=proto.FlightState.WAIT, flags=0)
        transport.push(proto.MsgType.TLM_STATE, landed.to_payload())
        assert wait_until(lambda: session._phase == PHASE_CONNECTED)

    def test_connected_promotes_to_flying_when_drone_already_airborne(
            self, session_factory):
        """接続先の機体が既に飛行中(例: サーバ再起動)→ flying へ昇格する。

        connected のままだと飛行ガード(モード/プロファイル変更・start の拒否)
        が素通りするため、TLM の飛行報告で connected→flying を昇格させる。
        """
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session._phase == PHASE_CONNECTED

        airborne = proto.TlmState(state=proto.FlightState.HOVER,
                                  flags=proto.TlmState.FLAG_FLYING)
        transport.push(proto.MsgType.TLM_STATE, airborne.to_payload())
        assert wait_until(lambda: session._phase == PHASE_FLYING)

        # 飛行ガード一式が有効になる
        assert session.set_mode(MODE_POSITION) is False
        assert session.start() is False
        assert session.select_airframe("zero-bias") is False

        # 着陸報告で connected へ戻る(既存の FLYING→CONNECTED 遷移)
        landed = proto.TlmState(state=proto.FlightState.WAIT, flags=0)
        transport.push(proto.MsgType.TLM_STATE, landed.to_payload())
        assert wait_until(lambda: session._phase == PHASE_CONNECTED)

    def test_armed_grace_tolerates_stale_wait_state(self, session_factory):
        """START 直後の WAIT 報告(猶予内)では armed を維持する。"""
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.start()

        baseline = session.serial.stats()["rx_frames_ok"]
        stale = proto.TlmState(state=proto.FlightState.WAIT, flags=0)
        transport.push(proto.MsgType.TLM_STATE, stale.to_payload())
        assert wait_until(
            lambda: session.serial.stats()["rx_frames_ok"] > baseline)
        assert session._phase == PHASE_ARMED

        # 猶予(1.5s)を超えても WAIT のまま → connected へ戻す
        clock.advance(2.0)
        transport.push(proto.MsgType.TLM_STATE, stale.to_payload())
        assert wait_until(lambda: session._phase == PHASE_CONNECTED)
