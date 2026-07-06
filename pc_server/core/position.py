"""Position モード: mocap → フィルタ → XY PID → セットポイント → 50Hz 送信。

データフロー(legacy hovering_controller の構造を踏襲):
- NatNet コールバック(on_mocap_pose, NatNetスレッド)で PositionFilter →
  有効性判定 → XY PID を更新し、最新指令(roll/pitch)をキャッシュする。
- 50Hz 送信スレッドはキャッシュ指令を SetpointShaper(クランプ+スルーレート
  制限)に通して emit する。目標位置は UI から随時更新。
- yaw は制御に使用しない(UI 表示専用、mocap.py が算出)。

フェイルセーフ(PROTOCOL.md):
- MoCap 途絶 > mocap_dropout_level_s(300ms)→ roll/pitch を水平(0)に固定し
  て送信を継続(alt_ref は維持)。>2s の CMD_STOP は session 層の監視が行う。

単位は core 内部規約(rad / m)。座標は制御座標系(mocap.py で変換済み)。
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from .filter import PositionFilter
from .mocap import RAD_TO_DEG
from .pid import XYPIDController
from .posture import SENDER_JOIN_TIMEOUT_S, SetpointShaper, run_paced_loop

# 異常解除に要する信頼度(legacy hovering_controller と同値)
ANOMALY_CLEAR_CONFIDENCE = 0.5
MS_PER_S = 1000.0


class PositionController:
    """OptiTrack 位置フィードバックで XY を閉ループ制御するコントローラ。

    emit(roll_rad, pitch_rad, alt_m, meta) は session 層が供給し、
    バイアス加算・CMD_SETPOINT 送信・CSV ログを担う。
    """

    MODE_NAME = "position"

    def __init__(self, server_config: dict, control_config: dict,
                 emit: Callable[[float, float, float, dict], None],
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._period_s = 1.0 / server_config["rates"]["setpoint_hz"]
        self._dropout_level_s: float = server_config["failsafe"]["mocap_dropout_level_s"]
        self._mocap_fresh_s: float = server_config["freshness"]["mocap_fresh_s"]
        self._shaper = SetpointShaper(server_config["clamps"])
        self._emit = emit
        self._clock = clock

        self.position_filter = PositionFilter.from_config(control_config["filter"])
        self.pid = XYPIDController.from_config(control_config["pid"])
        self._confidence_zero_threshold: float = (
            control_config["control"]["confidence_zero_threshold"])
        self._frame_hold_s: float = control_config["control"]["frame_hold_ms"] / MS_PER_S

        target_default = control_config["target_default"]
        self._lock = threading.Lock()
        # pid / position_filter は NatNet スレッド・50Hz 送信スレッド・
        # UI(executor)/supervisor スレッドから触られる共有状態のため、
        # 専用ロックで保護する(規約: スレッド共有状態は lock で保護)。
        # ロック順序は self._lock → self._pid_lock の一方向のみ(逆順禁止)。
        self._pid_lock = threading.Lock()
        self._filter_lock = threading.Lock()
        self._target = (target_default["x"], target_default["y"], target_default["z"])

        # NatNet コールバックが更新する最新状態
        self._last_pose: Optional[dict] = None         # mocap.py の pose dict
        self._last_pose_t: Optional[float] = None
        self._last_frame_dt: Optional[float] = None
        self._last_filter_result: Optional[dict] = None
        self._last_cmd = (0.0, 0.0)                    # PID 出力 (roll, pitch) [rad]
        self._last_errors = (0.0, 0.0)
        self._last_data_valid = False

        # 直近の送信値(バイアス加算前)
        self._last_output = (0.0, 0.0, self._target[2])

        # XY 閉ループの有効フラグ(legacy の control_active に相当)。
        # Start 受理後のみ session 層が True にする。False の間もフィルタは
        # 回し続けるが、PID は更新せず指令は水平(0)を維持する。
        self._control_active = False

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # UI からの入力(座標は制御座標系の m)
    # ------------------------------------------------------------------

    def set_target(self, x: float, y: float, z: float) -> None:
        with self._lock:
            self._target = (x, y, z)

    def get_target(self) -> tuple[float, float, float]:
        with self._lock:
            return self._target

    def current_setpoint(self) -> tuple[float, float, float]:
        """直近に送信した整形済みセットポイント(バイアス加算前)。"""
        with self._lock:
            return self._last_output

    def set_control_active(self, active: bool) -> None:
        """XY 閉ループの有効/無効を切り替える(有効化時に PID をリセット)。"""
        with self._lock:
            if active and not self._control_active:
                with self._pid_lock:
                    self.pid.reset()
            self._control_active = active
            if not active:
                self._last_cmd = (0.0, 0.0)

    @property
    def control_active(self) -> bool:
        with self._lock:
            return self._control_active

    # ------------------------------------------------------------------
    # NatNet コールバック(NatNetスレッド上: ブロッキング・print 禁止)
    # ------------------------------------------------------------------

    def on_mocap_pose(self, pose: dict) -> None:
        """新規 mocap フレームでフィルタと PID を更新し、指令をキャッシュする。"""
        t = pose["t_mono"]
        position = (pose["x"], pose["y"], pose["z"])

        with self._lock:
            prev_t = self._last_pose_t
            frame_dt = None if prev_t is None else (t - prev_t)
            self._last_pose = pose
            self._last_pose_t = t
            self._last_frame_dt = frame_dt
            target_x, target_y, _ = self._target
            control_active = self._control_active

        # フィルタは NatNet スレッドと reset_control(UI/executor)が共有する
        with self._filter_lock:
            filter_result = self.position_filter.process_position(
                position,
                marker_count=pose["marker_count"],
                current_time=t,
                tracking_valid=pose["tracking_valid"],
                quality_weight=pose["quality"],
                rigid_body_error=pose["error"],
                source="rigid_body",
            )

        confidence = filter_result["confidence"]
        consecutive_outliers = filter_result["consecutive_outliers"]
        is_data_valid = (
            confidence >= self._confidence_zero_threshold
            and not filter_result["is_outlier"]
            and filter_result["tracking_valid"]
        )
        # フレーム間隔が保持時間を超えた場合も無効扱い(legacy と同じ)
        if frame_dt is not None and frame_dt > self._frame_hold_s:
            is_data_valid = False

        fx, fy, _ = filter_result["filtered_position"]
        error_x = target_x - fx
        error_y = target_y - fy

        if control_active:
            # set_anomaly_state → calculate を1ロック区間で行い、50Hz 送信
            # スレッド(step の途絶処理)との交互実行で異常フラグと I 項が
            # 食い違わないようにする
            with self._pid_lock:
                if not is_data_valid:
                    self.pid.set_anomaly_state(True)
                elif (consecutive_outliers == 0
                        and confidence > ANOMALY_CLEAR_CONFIDENCE):
                    self.pid.set_anomaly_state(False)
                roll_cmd, pitch_cmd = self.pid.calculate(
                    error_x, error_y, t, is_data_valid)
            if not is_data_valid:
                roll_cmd = 0.0
                pitch_cmd = 0.0
        else:
            # 閉ループ無効時はフィルタのみ更新し、指令は水平を維持
            roll_cmd = 0.0
            pitch_cmd = 0.0

        with self._lock:
            self._last_filter_result = filter_result
            self._last_cmd = (roll_cmd, pitch_cmd)
            self._last_errors = (error_x, error_y)
            self._last_data_valid = is_data_valid

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
        self.reset_control()
        # 停止イベントはスレッドごとに新規生成し、ループへ明示的に渡す。
        # join がタイムアウトした旧ループが後から復帰しても、自分専用の
        # (set 済み)イベントを見て必ず終了するため、二重送信は起きない。
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._sender_loop, args=(self._stop_event,),
            name="position-sender", daemon=True)
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

    def reset_control(self) -> None:
        """フィルタ・PID・整形状態をリセットする(セッション開始時)。"""
        with self._filter_lock:
            self.position_filter.reset()
        with self._pid_lock:
            self.pid.reset()
        self._shaper.reset()
        with self._lock:
            self._last_pose = None
            self._last_pose_t = None
            self._last_frame_dt = None
            self._last_filter_result = None
            self._last_cmd = (0.0, 0.0)
            self._last_errors = (0.0, 0.0)
            self._last_data_valid = False
            self._control_active = False

    def _sender_loop(self, stop_event: threading.Event) -> None:
        run_paced_loop(stop_event, self._clock, self._period_s, self.step)

    def step(self, now: float) -> None:
        """1周期ぶんの途絶判定+整形+送信(テストから直接呼べる)。"""
        with self._lock:
            pose_t = self._last_pose_t
            roll_cmd, pitch_cmd = self._last_cmd
            error_x, error_y = self._last_errors
            data_valid = self._last_data_valid
            filter_result = self._last_filter_result
            pose = self._last_pose
            frame_dt = self._last_frame_dt
            target = self._target
            control_active = self._control_active

        age = None if pose_t is None else (now - pose_t)
        dropped = age is None or age > self._dropout_level_s
        if dropped:
            # MoCap 途絶 >300ms: roll/pitch を水平へ固定(alt_ref は維持)
            roll_cmd = 0.0
            pitch_cmd = 0.0
            data_valid = False
            if control_active:
                with self._pid_lock:
                    self.pid.set_anomaly_state(True)

        roll, pitch, alt = self._shaper.shape(roll_cmd, pitch_cmd, target[2], now)
        with self._lock:
            self._last_output = (roll, pitch, alt)

        with self._pid_lock:
            pid_components = self.pid.get_all_components()
        meta = {
            "mode": self.MODE_NAME,
            "data_valid": data_valid,
            "control_active": control_active,
            "mocap_dropout": dropped,
            "mocap_age_ms": None if age is None else age * MS_PER_S,
            "error_x": error_x,
            "error_y": error_y,
            "target_x": target[0],
            "target_y": target[1],
            "target_z": target[2],
            "pid_components": pid_components,
            "frame_dt_ms": None if frame_dt is None else frame_dt * MS_PER_S,
        }
        meta["data_source"] = "rigid_body" if pose is not None else "none"
        if pose is not None:
            meta["frame_number"] = pose["frame_number"]
            meta["marker_count"] = pose["marker_count"]
            meta["rb_error"] = pose["error"]
            meta["tracking_valid"] = pose["tracking_valid"]
            meta["raw_pos"] = (pose["x"], pose["y"], pose["z"])
        if filter_result is not None:
            meta["filtered_pos"] = tuple(filter_result["filtered_position"])
            meta["is_outlier"] = filter_result["is_outlier"]
            meta["used_prediction"] = filter_result["used_prediction"]
            meta["confidence"] = filter_result["confidence"]
            meta["consecutive_outliers"] = filter_result["consecutive_outliers"]
            meta["filter_threshold"] = filter_result["threshold"]
        self._emit(roll, pitch, alt, meta)

    # ------------------------------------------------------------------
    # session 層向けスナップショット
    # ------------------------------------------------------------------

    def mocap_age_s(self, now: Optional[float] = None) -> Optional[float]:
        """最後に有効な mocap pose を受けてからの経過秒(未受信なら None)。"""
        if now is None:
            now = self._clock()
        with self._lock:
            pose_t = self._last_pose_t
        return None if pose_t is None else (now - pose_t)

    def mocap_snapshot(self, now: Optional[float] = None) -> Optional[dict]:
        """WebSocket の "mocap" フィールド用スナップショット(deg/m)。"""
        if now is None:
            now = self._clock()
        with self._lock:
            pose = self._last_pose
            pose_t = self._last_pose_t
            filter_result = self._last_filter_result
        if pose is None or pose_t is None:
            return None
        position = (filter_result["filtered_position"]
                    if filter_result is not None else (pose["x"], pose["y"], pose["z"]))
        confidence = (filter_result["confidence"] if filter_result is not None
                      else pose["quality"])
        return {
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
            "yaw_deg": pose["yaw_rad"] * RAD_TO_DEG,
            "confidence": float(confidence),
            "fresh": (now - pose_t) <= self._mocap_fresh_s,
        }
