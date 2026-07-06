"""v2 円軌道モード: 合流位相・回転方向・接線追従・停止復帰・ログ meta。"""

from __future__ import annotations

import math

import pytest

from core.position import (
    TRAJ_MODE_CIRCLE, TRAJ_MODE_HOVER, PositionController,
)
from core.posture import wrap_pi

from fakes import FakeClock, make_pose

STEP_DT = 0.02


def make_controller(server_config, control_config):
    emitted = []
    clock = FakeClock()
    controller = PositionController(
        server_config, control_config,
        emit=lambda r, p, a, meta: emitted.append((r, p, a, meta)),
        clock=clock)
    return controller, emitted, clock


def feed_pose(controller, clock, x=0.0, y=0.0, z=0.3, yaw_rad=0.0):
    controller.on_mocap_pose(make_pose(x=x, y=y, z=z, t=clock(),
                                       yaw_rad=yaw_rad))


class TestCircleStart:
    def test_requires_position(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        ok, error = controller.start_circle(0.0, 0.0, 0.3, 30.0, True, 0.5,
                                            False, now=clock())
        assert not ok
        assert "MoCap" in error

    def test_param_validation(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.5, y=0.0)
        # 半径・周期・中心・高度の範囲外は拒否(control.json trajectory 節)
        assert not controller.start_circle(0.0, 0.0, 99.0, 30.0, True, 0.5,
                                           False, now=clock())[0]
        assert not controller.start_circle(0.0, 0.0, 0.3, 0.5, True, 0.5,
                                           False, now=clock())[0]
        assert not controller.start_circle(9.0, 0.0, 0.3, 30.0, True, 0.5,
                                           False, now=clock())[0]
        assert not controller.start_circle(0.0, 0.0, 0.3, 30.0, True, 99.0,
                                           False, now=clock())[0]

    def test_merges_at_nearest_point(self, server_config, control_config):
        """現在位置から円周最近傍点(同じ方位角)に位相を合わせる。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.5, y=0.5)   # 中心(0,0) から 45° 方向
        ok, error = controller.start_circle(0.0, 0.0, 0.3, 30.0, False, 0.5,
                                            False, now=clock())
        assert ok, error
        controller.step(clock())   # 軌道目標を確定
        tx, ty, tz = controller.get_target()
        # 目標は 45° 方向の円周上の点
        assert tx == pytest.approx(0.3 * math.cos(math.radians(45.0)), abs=1e-6)
        assert ty == pytest.approx(0.3 * math.sin(math.radians(45.0)), abs=1e-6)
        assert tz == pytest.approx(0.5)


class TestCircleMotion:
    def _start(self, server_config, control_config, clockwise,
               face_tangent=False, period_s=8.0):
        controller, emitted, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.3, y=0.0)   # 位相 0 から開始
        ok, error = controller.start_circle(0.0, 0.0, 0.3, period_s, clockwise,
                                            0.5, face_tangent, now=clock())
        assert ok, error
        return controller, emitted, clock

    def test_ccw_phase_advances(self, server_config, control_config):
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=False)
        controller.step(clock())
        # 2秒 = 周期8秒の 1/4 → 位相 +90°
        for _ in range(100):
            controller.step(clock.advance(STEP_DT))
        tx, ty, _ = controller.get_target()
        assert tx == pytest.approx(0.0, abs=1e-6)
        assert ty == pytest.approx(0.3, abs=1e-6)
        meta = emitted[-1][3]
        assert meta["traj_mode"] == TRAJ_MODE_CIRCLE
        assert meta["traj_phase_rad"] == pytest.approx(math.pi / 2, abs=1e-6)

    def test_cw_phase_recedes(self, server_config, control_config):
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=True)
        controller.step(clock())
        for _ in range(100):   # 2秒 → 位相 -90°
            controller.step(clock.advance(STEP_DT))
        tx, ty, _ = controller.get_target()
        assert tx == pytest.approx(0.0, abs=1e-6)
        assert ty == pytest.approx(-0.3, abs=1e-6)

    def test_face_tangent_follows_heading(self, server_config, control_config):
        """「進行方向を向く」ON かつヨー制御 ON → yaw_ref が接線方向へ追従。"""
        # 周期 32s(接線方位の角速度 11.25°/s < ヨースルー 45°/s)なら追従できる
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=False,
                                                 face_tangent=True,
                                                 period_s=32.0)
        controller.set_yaw_control(True)
        # 位相 0 のとき CCW の接線方向は +90°。初期ギャップ 90° は
        # (45−11.25)°/s で詰まり、約 2.7 秒で追いつく
        for _ in range(150):   # 3秒
            controller.step(clock.advance(STEP_DT))
        meta = emitted[-1][3]
        phase = meta["traj_phase_rad"]
        expected_heading = wrap_pi(phase + math.pi / 2)
        assert abs(wrap_pi(meta["yaw_ref_rad"] - expected_heading)) \
            < math.radians(2.0)
        assert meta["yaw_ctrl_on"] is True

    def test_face_tangent_ignored_when_yaw_off(self, server_config,
                                               control_config):
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=False,
                                                 face_tangent=True)
        # ヨー制御 OFF のままなら yaw_ref は動かない(flags bit1 も立たない)
        for _ in range(50):
            controller.step(clock.advance(STEP_DT))
        meta = emitted[-1][3]
        assert meta["yaw_ctrl_on"] is False
        assert meta["yaw_ref_rad"] == pytest.approx(0.0)

    def test_stop_returns_to_hover_at_current_target(self, server_config,
                                                     control_config):
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=False)
        for _ in range(50):
            controller.step(clock.advance(STEP_DT))
        frozen = controller.get_target()
        controller.stop_circle()
        for _ in range(10):
            controller.step(clock.advance(STEP_DT))
        assert controller.get_target() == pytest.approx(frozen)
        meta = emitted[-1][3]
        assert meta["traj_mode"] == TRAJ_MODE_HOVER
        assert meta["traj_phase_rad"] is None
        assert controller.trajectory_snapshot() == {"mode": "hover"}


class TestMocapYawMeta:
    def test_mocap_yaw_deg_in_meta(self, server_config, control_config):
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock, yaw_rad=math.radians(25.0))
        controller.step(clock.advance(STEP_DT))
        meta = emitted[-1][3]
        assert meta["mocap_yaw_deg"] == pytest.approx(25.0)
        assert meta["traj_mode"] == TRAJ_MODE_HOVER
