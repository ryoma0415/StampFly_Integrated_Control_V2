"""v2 ヨー指令: シェイパーの wrap/スルーレート、CMD_SETPOINT 17B 送信。"""

from __future__ import annotations

import math

import pytest

import stampfly_protocol as proto

from core.posture import PostureController, SetpointShaper, wrap_pi

from conftest import halt_supervisor
from fakes import FakeClock

STEP_DT = 0.02   # 50Hz


@pytest.fixture
def shaper(server_config):
    return SetpointShaper(server_config["clamps"])


class TestWrapPi:
    def test_basic_range(self):
        assert wrap_pi(0.0) == 0.0
        assert wrap_pi(math.pi) == pytest.approx(math.pi)
        assert wrap_pi(-math.pi) == pytest.approx(math.pi)   # 境界は +π 側
        assert wrap_pi(3 * math.pi) == pytest.approx(math.pi)
        assert wrap_pi(math.radians(190.0)) == pytest.approx(math.radians(-170.0))
        assert wrap_pi(math.radians(-190.0)) == pytest.approx(math.radians(170.0))


class TestYawShaping:
    def test_slew_rate_45_deg_per_s(self, shaper):
        # 1ステップ(20ms)あたり最大 0.9°
        shaper.shape_yaw(0.0, 0.0)   # 初期化(dt=0)
        yaw = shaper.shape_yaw(math.radians(90.0), STEP_DT)
        assert yaw == pytest.approx(math.radians(45.0 * STEP_DT))

    def test_converges_to_target(self, shaper):
        t = 0.0
        shaper.shape_yaw(0.0, t)
        target = math.radians(30.0)
        for _ in range(200):   # 4秒 > 30°/45°/s
            t += STEP_DT
            yaw = shaper.shape_yaw(target, t)
        assert yaw == pytest.approx(target)

    def test_shortest_path_across_180(self, shaper):
        """+170° 付近から -170° への目標は ±180° 跨ぎ(遠回りしない)。"""
        t = 0.0
        shaper.shape_yaw(0.0, t)
        # まず +170° まで回す
        target = math.radians(170.0)
        for _ in range(400):
            t += STEP_DT
            shaper.shape_yaw(target, t)
        # -170° へ: 最短経路は +20°(+170 → +180/-180 → -170)
        target2 = math.radians(-170.0)
        t += STEP_DT
        yaw = shaper.shape_yaw(target2, t)
        # 1ステップでは +側にさらに進む(wrap 経由の最短経路)
        assert yaw > math.radians(170.0) or yaw < math.radians(-170.0)
        for _ in range(100):   # 20° / 45°/s ≈ 0.44s
            t += STEP_DT
            yaw = shaper.shape_yaw(target2, t)
        assert wrap_pi(yaw - target2) == pytest.approx(0.0, abs=1e-9)

    def test_reset_returns_to_zero(self, shaper):
        shaper.shape_yaw(0.0, 0.0)
        shaper.shape_yaw(math.radians(90.0), 1.0)
        shaper.reset()
        assert shaper.shape_yaw(0.0, 2.0) == 0.0


class TestPostureYawEmit:
    def _make(self, server_config):
        emitted = []
        controller = PostureController(
            server_config,
            emit=lambda r, p, a, meta: emitted.append((r, p, a, meta)),
            clock=FakeClock(),
        )
        return controller, emitted

    def test_meta_carries_yaw(self, server_config):
        controller, emitted = self._make(server_config)
        controller.set_setpoint(0.0, 0.0, 0.3, yaw_rad=math.radians(10.0))
        controller.set_yaw_control(True)
        t = 100.0
        for _ in range(100):
            t += STEP_DT
            controller.step(t)
        _, _, _, meta = emitted[-1]
        assert meta["yaw_ctrl_on"] is True
        assert meta["yaw_ref_rad"] == pytest.approx(math.radians(10.0))
        yaw, on = controller.yaw_setpoint()
        assert on is True
        assert yaw == pytest.approx(math.radians(10.0))

    def test_yaw_ctrl_off_by_default(self, server_config):
        controller, emitted = self._make(server_config)
        controller.step(100.02)
        _, _, _, meta = emitted[-1]
        assert meta["yaw_ctrl_on"] is False


class TestSessionSetpointWire:
    def test_yaw_ref_and_flags_bit1(self, session_factory):
        """ヨー制御 ON で 17B CMD_SETPOINT の flags bit1 と yaw_ref が立つ。"""
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()   # 手動 step で決定的に
        session.set_yaw_control(True)
        session.set_setpoint_deg(0.0, 0.0, 0.3, yaw_deg=5.0)
        transport.clear_sent()
        for _ in range(60):   # 1.2秒 > 5°/45°/s
            session.posture.step(clock.advance(STEP_DT))
        frame = transport.frames_of_type(proto.MsgType.CMD_SETPOINT)[-1]
        assert len(frame.payload) == proto.CmdSetpoint.PAYLOAD_SIZE   # 17B
        sent = proto.CmdSetpoint.from_payload(frame.payload)
        assert sent.flags & proto.CmdSetpoint.FLAG_ALT_REF_VALID
        assert sent.flags & proto.CmdSetpoint.FLAG_YAW_REF_VALID
        assert sent.yaw_ref == pytest.approx(math.radians(5.0), abs=1e-6)

    def test_yaw_off_sends_zero(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()
        session.set_setpoint_deg(0.0, 0.0, 0.3, yaw_deg=90.0)   # 制御 OFF のまま
        transport.clear_sent()
        for _ in range(10):
            session.posture.step(clock.advance(STEP_DT))
        sent = proto.CmdSetpoint.from_payload(
            transport.frames_of_type(proto.MsgType.CMD_SETPOINT)[-1].payload)
        assert not (sent.flags & proto.CmdSetpoint.FLAG_YAW_REF_VALID)
        assert sent.yaw_ref == 0.0
