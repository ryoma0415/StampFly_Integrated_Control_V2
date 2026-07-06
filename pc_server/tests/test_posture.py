"""posture: クランプ・スルーレート制限・50Hz送信ステップ。"""

from __future__ import annotations

import math

import pytest

from core.mocap import DEG_TO_RAD
from core.posture import PostureController, SetpointShaper

from fakes import FakeClock

STEP_DT = 0.02   # 50Hz


@pytest.fixture
def shaper(server_config):
    return SetpointShaper(server_config["clamps"])


def test_roll_pitch_clamped_to_max_angle(shaper):
    # ±10°(config)を超える目標はクランプされる
    t = 0.0
    shaper.shape(0.0, 0.0, 0.3, t)   # 初期化(dt=0)
    for _ in range(200):             # 4秒ぶん回してスルー制限の影響を除く
        t += STEP_DT
        roll, pitch, _ = shaper.shape(math.radians(45.0), math.radians(-45.0), 0.3, t)
    assert roll == pytest.approx(10.0 * DEG_TO_RAD)
    assert pitch == pytest.approx(-10.0 * DEG_TO_RAD)


def test_slew_rate_limit_30_deg_per_s(shaper):
    shaper.shape(0.0, 0.0, 0.3, 0.0)
    # 1ステップ(20ms)では 30°/s × 0.02s = 0.6° しか動けない
    roll, pitch, _ = shaper.shape(math.radians(10.0), math.radians(-10.0), 0.3, STEP_DT)
    assert roll == pytest.approx(0.6 * DEG_TO_RAD)
    assert pitch == pytest.approx(-0.6 * DEG_TO_RAD)
    # 10°/(30°/s) = 1/3 秒後に到達する
    t = STEP_DT
    for _ in range(int(1.0 / STEP_DT)):
        t += STEP_DT
        roll, pitch, _ = shaper.shape(math.radians(10.0), math.radians(-10.0), 0.3, t)
    assert roll == pytest.approx(10.0 * DEG_TO_RAD)


def test_alt_clamped_to_configured_range(shaper):
    _, _, alt_high = shaper.shape(0.0, 0.0, 5.0, 0.0)
    assert alt_high == pytest.approx(1.2)   # 上限 1.2m
    shaper.reset()
    _, _, alt_low = shaper.shape(0.0, 0.0, 0.0, 0.0)
    assert alt_low == pytest.approx(0.1)    # 下限 0.1m


def test_alt_rate_limited(shaper):
    shaper.shape(0.0, 0.0, 0.3, 0.0)
    # 0.3m/s × 0.02s = 6mm/step
    _, _, alt = shaper.shape(0.0, 0.0, 1.0, STEP_DT)
    assert alt == pytest.approx(0.306)


def test_large_dt_is_capped(shaper):
    # スレッド停止などで dt が伸びても 1 ステップの変化は MAX_STEP_DT_S ぶんまで
    shaper.shape(0.0, 0.0, 0.3, 0.0)
    roll, _, _ = shaper.shape(math.radians(10.0), 0.0, 0.3, 10.0)
    assert roll == pytest.approx(30.0 * 0.1 * DEG_TO_RAD)   # 30°/s × 0.1s


def test_controller_step_emits_shaped_setpoint(server_config):
    emitted = []
    clock = FakeClock()
    controller = PostureController(
        server_config,
        emit=lambda r, p, a, meta: emitted.append((r, p, a, meta)),
        clock=clock)

    controller.set_setpoint(math.radians(5.0), math.radians(-3.0), 0.5)
    t = clock()
    controller.step(t)               # 初期化ステップ(dt=0 → 角度はまだ0)
    roll0, pitch0, alt0, meta0 = emitted[0]
    assert roll0 == pytest.approx(0.0)
    assert pitch0 == pytest.approx(0.0)
    assert meta0["mode"] == "posture"

    for i in range(50):              # 1秒ぶん → 5°/(30°/s) = 0.17s で到達済み
        t += STEP_DT
        controller.step(t)
    roll, pitch, alt, _ = emitted[-1]
    assert roll == pytest.approx(5.0 * DEG_TO_RAD)
    assert pitch == pytest.approx(-3.0 * DEG_TO_RAD)
    assert alt == pytest.approx(0.5)
    assert controller.current_setpoint() == (roll, pitch, alt)


def test_controller_sender_thread_runs_at_50hz(server_config):
    """実クロックで短時間スレッドを回し、送信が継続することを確認する。"""
    emitted = []
    controller = PostureController(
        server_config, emit=lambda r, p, a, meta: emitted.append((r, p, a)))
    controller.start()
    try:
        import time
        time.sleep(0.25)
    finally:
        controller.stop()
    # 0.25s で名目 12〜13 回(スケジューリング余裕を見て 5 回以上)
    assert len(emitted) >= 5
    assert not controller.running
