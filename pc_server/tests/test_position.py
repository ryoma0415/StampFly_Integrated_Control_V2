"""position: 合成 mocap → フィルタ → 位置誤差(機上XY制御)のパイプライン。

XY PID は機体側 flight_control が実行するため、この層は誤差計算・有効性
判定・軌道・整形済み alt/yaw の meta 供給までを検証する(roll/pitch の
角度指令は常に 0 で emit される)。
"""

from __future__ import annotations

import pytest

from core.mocap import CoordinateTransformer, MocapSource
from core.position import PositionController

from fakes import (FakeClock, FakeNatNetClient, make_mocap_frame,
                   make_mocap_frame_multi, make_pose)

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


class TestErrorPipeline:
    def test_error_is_target_minus_filtered_position(self, server_config,
                                                     control_config):
        """誤差 = 目標 − フィルタ済み位置(機体側 XY PID の入力になる)。"""
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_control_active(True)
        controller.set_target(0.5, -0.2, 0.3)
        feed_poses(controller, clock, [(0.0, 0.0, 0.3)] * 10)

        settle_steps(controller, clock)
        roll, pitch, alt, meta = emitted[-1]
        assert meta["data_valid"] is True
        assert meta["control_active"] is True
        assert meta["error_x"] == pytest.approx(0.5, abs=0.01)
        assert meta["error_y"] == pytest.approx(-0.2, abs=0.01)
        # roll/pitch の角度指令は機体側で計算されるため常に 0 で emit する
        assert roll == 0.0
        assert pitch == 0.0

    def test_control_inactive_reported_in_meta(self, server_config,
                                               control_config):
        """Start 前(control_active=False)は meta に反映される(bit2=0 の根拠)。"""
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_target(0.5, 0.5, 0.3)
        feed_poses(controller, clock, [(0.0, 0.0, 0.3)] * 10)
        controller.step(clock())
        _, _, _, meta = emitted[-1]
        assert meta["control_active"] is False
        # 誤差自体は計算され続ける(閉ループ ON/OFF と独立)
        assert meta["error_x"] == pytest.approx(0.5, abs=0.01)

    def test_invalid_tracking_clears_data_valid(self, server_config,
                                                control_config):
        controller, emitted, clock = make_controller(server_config, control_config)
        controller.set_control_active(True)
        controller.set_target(0.5, 0.0, 0.3)
        feed_poses(controller, clock, [(0.0, 0.0, 0.3)] * 10, tracking_valid=False)
        controller.step(clock())
        _, _, _, meta = emitted[-1]
        assert meta["data_valid"] is False
        assert meta["control_active"] is True

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
                    "data_source", "filtered_pos", "raw_pos",
                    "confidence", "is_outlier", "frame_number"):
            assert key in meta, key
        assert meta["data_source"] == "rigid_body"
        # 旧 PC 側 XY PID の語彙は v4 で削除済み
        assert "pid_components" not in meta


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
        # インベントリ(紐付け確認)には対象外IDも記録される
        bodies = source.bodies_snapshot()
        assert [b["rigid_body_id"] for b in bodies] == [99]
        source.shutdown()

    def test_subscribe_dispatches_per_rigid_body(self, control_config):
        """subscribe した ID には個別コールバック、primary は従来どおり。"""
        FakeNatNetClient.instances.clear()
        source = MocapSource(control_config["natnet"],
                             control_config["coordinate_transform"],
                             client_factory=FakeNatNetClient)
        primary_poses: list[dict] = []
        sub_poses: list[dict] = []
        source.start(primary_poses.append)
        source.subscribe(2, sub_poses.append)
        client = FakeNatNetClient.instances[-1]

        client.new_frame_with_data_listener(make_mocap_frame_multi([
            {"rigid_body_id": 1, "pos": (0.1, 0.3, 0.2)},
            {"rigid_body_id": 2, "pos": (0.5, 0.4, -0.2)},
        ], frame_number=7))

        # primary(control.json natnet.rigid_body_id=1)は従来経路
        assert len(primary_poses) == 1
        assert primary_poses[0]["rigid_body_id"] == 1
        assert primary_poses[0]["x"] == pytest.approx(0.2)
        # subscribe(2) には ID 2 の pose のみが届く
        assert len(sub_poses) == 1
        assert sub_poses[0]["rigid_body_id"] == 2
        assert sub_poses[0]["x"] == pytest.approx(-0.2)   # ← Motive z
        assert sub_poses[0]["y"] == pytest.approx(-0.5)   # ← −Motive x
        assert sub_poses[0]["z"] == pytest.approx(0.4)    # ← Motive y

        # インベントリには両方が ID 昇順で載り、age_s を持つ
        bodies = source.bodies_snapshot()
        assert [b["rigid_body_id"] for b in bodies] == [1, 2]
        assert all(b["age_s"] >= 0.0 for b in bodies)

        # unsubscribe 後は届かない
        source.unsubscribe(2)
        client.new_frame_with_data_listener(make_mocap_frame_multi([
            {"rigid_body_id": 2, "pos": (0.5, 0.4, -0.2)},
        ]))
        assert len(sub_poses) == 1
        source.shutdown()
