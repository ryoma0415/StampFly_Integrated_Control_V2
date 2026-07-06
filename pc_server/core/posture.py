"""Posture モード: UI setpoint → クランプ/スルーレート制限 → 50Hz 送信。

安全クランプ(ARCHITECTURE.md「安全クランプ(多層)」の pc_server 層):
roll/pitch ±10°(config) + スルーレート 30°/s、alt 0.1–1.2m + 0.3m/s。
v2: ヨー角目標 ±180°(config) + 最短経路 wrap + スルーレート 45°/s。
この層の整形(SetpointShaper)は Position モードの送信経路でも共用する。

単位はすべて core 内部規約(rad / m)。deg⇔rad 変換は session 層が行う。
"""

from __future__ import annotations

import threading
import time
from math import pi
from typing import Callable, Optional

from .mocap import DEG_TO_RAD

TWO_PI = 2.0 * pi


def wrap_pi(angle_rad: float) -> float:
    """角度を (-π, π] に正規化する(ヨー最短経路計算用)。"""
    wrapped = (angle_rad + pi) % TWO_PI - pi
    # Python の % は正の剰余を返すため wrapped ∈ [-π, π)。
    # 境界 -π は +π 側に寄せて (-π, π] とする(±180° 指令の符号安定化)。
    if wrapped == -pi:
        return pi
    return wrapped

# スレッド一時停止などで dt が異常に伸びた場合の上限(スルー制限の暴走防止)
MAX_STEP_DT_S = 0.1

# 送信スレッド停止時の join 待ち上限。超過=ループがブロックしている
# (構造定数。チューニング値ではないため config には置かない)
SENDER_JOIN_TIMEOUT_S = 1.0


def run_paced_loop(stop_event: threading.Event, clock: Callable[[], float],
                   period_s: float, step: Callable[[float], None]) -> None:
    """stop_event が立つまで step(now) を period_s 周期で呼ぶ送信ループ。

    Posture/Position 両コントローラの 50Hz 送信スレッドが共用する。
    大幅な遅延が起きた場合は周期を再同期する(バースト送信しない)。
    """
    next_t = clock()
    while not stop_event.is_set():
        now = clock()
        if now >= next_t:
            step(now)
            next_t += period_s
            if now - next_t > period_s:
                next_t = now + period_s   # 大幅遅延時は再同期
        else:
            stop_event.wait(min(period_s, next_t - now))


class SetpointShaper:
    """セットポイントのクランプ+スルーレート制限(pc_server 安全層)。

    roll/pitch はゼロ(水平)から、alt は最初の目標値から開始する。
    スレッド安全ではない。所有スレッド(50Hz送信スレッド)からのみ使用する。
    """

    def __init__(self, clamps_config: dict) -> None:
        self._max_angle_rad = clamps_config["max_roll_pitch_deg"] * DEG_TO_RAD
        self._slew_rad_per_s = clamps_config["slew_rate_deg_per_s"] * DEG_TO_RAD
        self._alt_min_m = clamps_config["alt_min_m"]
        self._alt_max_m = clamps_config["alt_max_m"]
        self._alt_rate_m_per_s = clamps_config["alt_rate_m_per_s"]
        self._max_yaw_rad = clamps_config["max_yaw_deg"] * DEG_TO_RAD
        self._yaw_slew_rad_per_s = clamps_config["yaw_slew_rate_deg_per_s"] * DEG_TO_RAD

        self._roll = 0.0
        self._pitch = 0.0
        self._alt: Optional[float] = None
        self._last_t: Optional[float] = None
        # ヨーは独立した整形状態を持つ(shape() と別呼び出しでも dt が
        # 二重計上されないよう、時刻も別管理する)
        self._yaw = 0.0
        self._yaw_last_t: Optional[float] = None

    @property
    def alt_limits(self) -> tuple[float, float]:
        return (self._alt_min_m, self._alt_max_m)

    def reset(self) -> None:
        """整形状態を初期化する(roll/pitch は水平へ、alt は次回目標値へ)。"""
        self._roll = 0.0
        self._pitch = 0.0
        self._alt = None
        self._last_t = None
        self._yaw = 0.0
        self._yaw_last_t = None

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _slew(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def shape(self, roll_target: float, pitch_target: float, alt_target: float,
              now: float) -> tuple[float, float, float]:
        """目標値をクランプし、前回出力からのスルーレート制限を適用する。"""
        roll_target = self._clamp(roll_target, -self._max_angle_rad, self._max_angle_rad)
        pitch_target = self._clamp(pitch_target, -self._max_angle_rad, self._max_angle_rad)
        alt_target = self._clamp(alt_target, self._alt_min_m, self._alt_max_m)

        if self._last_t is None:
            dt = 0.0
        else:
            dt = self._clamp(now - self._last_t, 0.0, MAX_STEP_DT_S)
        self._last_t = now

        if self._alt is None:
            # 初回の alt は目標値から開始(地上での基準ジャンプを避ける意図はなく、
            # alt_ref は飛行開始時点の目標として扱われるため)
            self._alt = alt_target

        max_angle_delta = self._slew_rad_per_s * dt
        max_alt_delta = self._alt_rate_m_per_s * dt
        self._roll = self._slew(self._roll, roll_target, max_angle_delta)
        self._pitch = self._slew(self._pitch, pitch_target, max_angle_delta)
        self._alt = self._slew(self._alt, alt_target, max_alt_delta)
        return (self._roll, self._pitch, self._alt)

    def shape_yaw(self, yaw_target: float, now: float) -> float:
        """ヨー角目標の整形: 最短経路(±π wrap)+スルーレート制限。

        roll/pitch と違い目標は円環上にあるため、誤差を wrap してから
        スルー制限する(±180° 跨ぎで遠回りしない)。出力は (-π, π]。
        """
        yaw_target = wrap_pi(yaw_target)
        yaw_target = self._clamp(yaw_target, -self._max_yaw_rad, self._max_yaw_rad)

        if self._yaw_last_t is None:
            dt = 0.0
        else:
            dt = self._clamp(now - self._yaw_last_t, 0.0, MAX_STEP_DT_S)
        self._yaw_last_t = now

        delta = wrap_pi(yaw_target - self._yaw)
        max_delta = self._yaw_slew_rad_per_s * dt
        delta = self._clamp(delta, -max_delta, max_delta)
        self._yaw = wrap_pi(self._yaw + delta)
        return self._yaw


class PostureController:
    """UI スライダ由来のセットポイントを 50Hz で送信するコントローラ。

    emit(roll_rad, pitch_rad, alt_m, meta) は session 層が供給し、
    バイアス加算・CMD_SETPOINT 送信・CSV ログを担う。
    """

    MODE_NAME = "posture"

    def __init__(self, server_config: dict,
                 emit: Callable[[float, float, float, dict], None],
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._period_s = 1.0 / server_config["rates"]["setpoint_hz"]
        self._shaper = SetpointShaper(server_config["clamps"])
        self._emit = emit
        self._clock = clock

        self._lock = threading.Lock()
        alt_min, _ = self._shaper.alt_limits
        self._target_roll = 0.0
        self._target_pitch = 0.0
        self._target_alt = alt_min
        self._target_yaw = 0.0
        self._yaw_ctrl_on = False
        self._last_output = (0.0, 0.0, alt_min)
        self._last_yaw_output = 0.0

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # UI からの入力(session 層で deg→rad 変換済み)
    # ------------------------------------------------------------------

    def set_setpoint(self, roll_rad: float, pitch_rad: float, alt_m: float,
                     yaw_rad: Optional[float] = None) -> None:
        with self._lock:
            self._target_roll = roll_rad
            self._target_pitch = pitch_rad
            self._target_alt = alt_m
            if yaw_rad is not None:
                self._target_yaw = yaw_rad

    def set_setpoint_yaw_only(self, yaw_rad: float) -> None:
        """ヨー角目標のみ更新する(共通ヨースライダ用)。"""
        with self._lock:
            self._target_yaw = yaw_rad

    def set_yaw_control(self, enabled: bool) -> None:
        """ヨー角制御トグル(ON: CMD_SETPOINT flags bit1 を立て yaw_ref を送る)。"""
        with self._lock:
            self._yaw_ctrl_on = bool(enabled)

    def yaw_setpoint(self) -> tuple[float, bool]:
        """直近に送信した整形済みヨー目標と制御 ON/OFF(スナップショット用)。"""
        with self._lock:
            return (self._last_yaw_output, self._yaw_ctrl_on)

    def set_default_alt(self, alt_m: float) -> None:
        """機体プロファイルの default_alt_m を初期目標高度として反映する。"""
        with self._lock:
            self._target_alt = alt_m

    def current_setpoint(self) -> tuple[float, float, float]:
        """直近に送信した整形済みセットポイント(バイアス加算前)。"""
        with self._lock:
            return self._last_output

    # ------------------------------------------------------------------
    # 50Hz 送信
    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            if self._thread.is_alive():
                # 動作中、または stop() で止めきれなかった旧スレッドが残存。
                # SetpointShaper は単一所有者前提のため再起動を拒否する。
                return
            self._thread = None
        # 停止イベントはスレッドごとに新規生成し、ループへ明示的に渡す。
        # join がタイムアウトした旧ループが後から復帰しても、自分専用の
        # (set 済み)イベントを見て必ず終了するため、二重送信は起きない。
        self._stop_event = threading.Event()
        self._shaper.reset()
        self._thread = threading.Thread(
            target=self._sender_loop, args=(self._stop_event,),
            name="posture-sender", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=SENDER_JOIN_TIMEOUT_S)
            if thread.is_alive():
                # ループがブロックしたまま(例: シリアル write 停滞)。
                # 参照を保持して start() の再起動を拒否し、ブロック解除後に
                # 旧ループが新ループと並走する事態を構造的に防ぐ。
                return
        self._thread = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _sender_loop(self, stop_event: threading.Event) -> None:
        run_paced_loop(stop_event, self._clock, self._period_s, self.step)

    def step(self, now: float) -> None:
        """1周期ぶんの整形+送信(テストから直接呼べる)。"""
        with self._lock:
            targets = (self._target_roll, self._target_pitch, self._target_alt)
            yaw_target = self._target_yaw
            yaw_ctrl_on = self._yaw_ctrl_on
        roll, pitch, alt = self._shaper.shape(*targets, now)
        yaw = self._shaper.shape_yaw(yaw_target, now)
        with self._lock:
            self._last_output = (roll, pitch, alt)
            self._last_yaw_output = yaw
        # ヨーは meta 経由で session 層に渡す(emit の引数互換を保つ)
        self._emit(roll, pitch, alt, {
            "mode": self.MODE_NAME,
            "yaw_ref_rad": yaw,
            "yaw_ctrl_on": yaw_ctrl_on,
        })
