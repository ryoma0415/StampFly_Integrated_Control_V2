"""position: 合成 mocap → フィルタ → XY PID → セットポイントのパイプライン。"""

from __future__ import annotations

import pytest

from core.mocap import CoordinateTransformer, MocapSource
from core.position import PositionController

from fakes import FakeClock, FakeNatNetClient, make_mocap_frame, make_pose

STEP_DT = 0.02
FRAME_DT = 0.01   # mocap 100Hz 相当


def make_controller(server_config, control_config, clock=None):
    emitted = []
    clock = clock or FakeClock()
    controller = PositionController(
        server_config, control_config,
        emit=lambda r, p, a, meta: emitted.append((r, p, a, meta)),
        clock=clock)
    return controller, emitted, clock


def feed_poses(controller, clock, positions, dt=FRAME_DT, **pose_kwargs):
    """合成 mocap pose を順に与える(時刻は FakeClock と同期)。"""
    for pos in positions:
        t = clock.advance(dt)
        controller.on_mocap_pose(make_pose(*pos, t=t, **pose_kwargs))


def settle_steps(controller, clock, n=2):
    """step を複数回呼ぶ(初回はシェイパ初期化で dt=0 のため出力 0)。"""
    controller.step(clock())
    for _ in range(n - 1):
        controller.step(clock.advance(STEP_DT))


class TestPidPipeline:
    def test_positive_x_error_gives_positive_roll(self, server_config, control_config):
        """目標が +x 側 → roll 指令は正(正ゲイン規約)。"""
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_control_active(True)
        controller.set_target(0.5, 0.0, 0.3)
        feed_poses(controller, clock, [(0.0, 0.0, 0.3)] * 10)

        settle_steps(controller, clock)
        roll, pitch, alt, meta = emitted[-1]
        assert meta["data_valid"] is True
        assert meta["error_x"] == pytest.approx(0.5, abs=0.01)
        assert roll > 0.0
        assert pitch == pytest.approx(0.0, abs=1e-6)

    def test_positive_y_error_gives_positive_pitch(self, server_config, control_config):
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_control_active(True)
        controller.set_target(0.0, 0.5, 0.3)
        feed_poses(controller, clock, [(0.0, 0.0, 0.3)] * 10)

        settle_steps(controller, clock)
        roll, pitch, _, meta = emitted[-1]
        assert pitch > 0.0
        assert roll == pytest.approx(0.0, abs=1e-6)

    def test_pid_output_respects_output_limit(self, server_config, control_config):
        """大誤差でも PID 出力は ±0.087 rad(config)に制限される。"""
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_control_active(True)
        controller.set_target(10.0, -10.0, 0.3)
        # スルー制限の影響を除くため十分長く回す
        for _ in range(100):
            feed_poses(controller, clock, [(0.0, 0.0, 0.3)])
            controller.step(clock())
        limit = control_config["pid"]["output_limit"][1]
        roll, pitch, _, _ = emitted[-1]
        assert roll == pytest.approx(limit, abs=1e-9)
        assert pitch == pytest.approx(-limit, abs=1e-9)

    def test_yaw_not_used_in_control(self, server_config, control_config):
        """pose の yaw を変えても指令は変わらない(ヨーは無制御)。"""
        outputs = []
        for yaw in (0.0, 1.0):
            controller, emitted, clock = make_controller(server_config, control_config)
            controller.set_control_active(True)
            controller.set_target(0.5, 0.0, 0.3)
            feed_poses(controller, clock, [(0.0, 0.0, 0.3)] * 10, yaw_rad=yaw)
            settle_steps(controller, clock)
            outputs.append(emitted[-1][:3])
        assert outputs[0] == outputs[1]

    def test_control_inactive_keeps_level_command(self, server_config, control_config):
        """Start 前(control_active=False)は誤差があっても水平を維持する。"""
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_target(0.5, 0.5, 0.3)
        feed_poses(controller, clock, [(0.0, 0.0, 0.3)] * 10)
        controller.step(clock())
        roll, pitch, _, meta = emitted[-1]
        assert roll == 0.0
        assert pitch == 0.0
        assert meta["control_active"] is False

    def test_invalid_tracking_zeroes_command(self, server_config, control_config):
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_control_active(True)
        controller.set_target(0.5, 0.0, 0.3)
        feed_poses(controller, clock, [(0.0, 0.0, 0.3)] * 10, tracking_valid=False)
        controller.step(clock())
        roll, pitch, _, meta = emitted[-1]
        assert meta["data_valid"] is False
        assert roll == 0.0
        assert pitch == 0.0
        assert controller.pid.pid_x.anomaly_detected

    def test_alt_follows_target_z(self, server_config, control_config):
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_target(0.0, 0.0, 0.8)
        feed_poses(controller, clock, [(0.0, 0.0, 0.3)] * 3)
        # 十分な回数ステップして alt スルー制限を通過させる
        t = clock()
        for _ in range(200):
            t += STEP_DT
            feed_poses(controller, clock, [(0.0, 0.0, 0.3)])
            controller.step(t)
        assert emitted[-1][2] == pytest.approx(0.8)

    def test_meta_contains_log_vocabulary(self, server_config, control_config):
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_control_active(True)
        feed_poses(controller, clock, [(0.1, -0.1, 0.3)] * 3)
        controller.step(clock())
        meta = emitted[-1][3]
        for key in ("mode", "data_valid", "control_active", "mocap_dropout",
                    "error_x", "error_y", "target_x", "target_y", "target_z",
                    "pid_components", "data_source", "filtered_pos", "raw_pos",
                    "confidence", "is_outlier", "frame_number"):
            assert key in meta, key
        assert meta["data_source"] == "rigid_body"


class TestMocapSource:
    def test_coordinate_transform_motive_to_control(self, control_config):
        """既定変換: 制御x←Motive z, 制御y←−Motive x, 制御z←Motive y。"""
        transformer = CoordinateTransformer(control_config["coordinate_transform"])
        assert transformer.motive_to_control((1.0, 2.0, 3.0)) == (3.0, -1.0, 2.0)

    def test_rigid_body_pose_extraction(self, control_config):
        FakeNatNetClient.instances.clear()
        source = MocapSource(control_config["natnet"],
                             control_config["coordinate_transform"],
                             client_factory=FakeNatNetClient)
        poses = []
        assert source.start(poses.append) is True
        client = FakeNatNetClient.instances[-1]

        client.new_frame_with_data_listener(
            make_mocap_frame(pos=(0.1, 0.3, 0.2), frame_number=42))
        assert len(poses) == 1
        pose = poses[0]
        assert pose["x"] == pytest.approx(0.2)    # ← Motive z
        assert pose["y"] == pytest.approx(-0.1)   # ← −Motive x
        assert pose["z"] == pytest.approx(0.3)    # ← Motive y(高度)
        assert pose["frame_number"] == 42
        assert pose["marker_count"] == 4
        assert source.latest_pose()["x"] == pytest.approx(0.2)
        source.shutdown()
        assert client.shutdown_called

    def test_no_centroid_fallback_for_missing_rigid_body(self, control_config):
        """対象リジッドボディ不在のフレームは破棄(重心フォールバック禁止)。"""
        FakeNatNetClient.instances.clear()
        source = MocapSource(control_config["natnet"],
                             control_config["coordinate_transform"],
                             client_factory=FakeNatNetClient)
        poses = []
        source.start(poses.append)
        client = FakeNatNetClient.instances[-1]

        client.new_frame_with_data_listener(
            make_mocap_frame(rigid_body_id=99))   # 対象IDではない
        assert poses == []
        assert source.latest_pose() is None
        assert source.stats()["frames_without_rigid_body"] == 1
        source.shutdown()
