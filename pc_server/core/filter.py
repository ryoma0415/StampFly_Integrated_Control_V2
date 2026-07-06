"""位置データフィルタ(legacy NatNet_PID_Controller/position_filter.py の移植・整理)。

整理内容:
- 時刻は time.monotonic() のみ(time.time() 禁止)。
- パラメータは config/control.json の "filter" セクションから供給する。
- アルゴリズム(動的閾値・予測補間・重み付き移動平均)は飛行実績のある
  旧実装と同一に保つ。
"""

from __future__ import annotations

import time
from collections import deque
from typing import Optional

import numpy as np

# --- アルゴリズム固有の構造定数(旧実装と同値) ---
MIN_THRESHOLD_FLOOR_M = 0.02         # 動的閾値の絶対下限
MOTION_ALLOWANCE_GAIN = 2.5          # 直近移動量に対する閾値余裕
SPEED_ALLOWANCE_GAIN = 1.5           # 推定速度に対する閾値余裕
THRESHOLD_EMA_KEEP = 0.6             # 動的閾値の EMA(現値保持率)
VELOCITY_EMA_KEEP = 0.7              # 速度ノルム EMA
STEP_EMA_KEEP = 0.7                  # ステップ距離 EMA
DT_EMA_KEEP = 0.8                    # フレーム間隔 EMA
OUTLIER_BASE_CONFIDENCE = 0.35       # 外れ値時の基礎信頼度
CONFIDENCE_FLOOR = 0.05
MARKER_FULL_COUNT = 4                # 信頼度正規化に使うマーカー数


class PositionFilter:
    """位置データの異常検出とフィルタリング。

    - 位置の急変(外れ値)を動的閾値で検出
    - 重み付き移動平均による平滑化
    - 外れ値時は速度推定に基づく予測位置で補間
    """

    def __init__(self, window_size: int = 5, outlier_threshold: float = 0.1,
                 velocity_window: int = 3, enable_prediction: bool = True,
                 max_outlier_threshold: float = 0.4) -> None:
        self.window_size = window_size
        self.base_threshold = outlier_threshold
        self.max_threshold = max(outlier_threshold, max_outlier_threshold)
        self.min_threshold = max(MIN_THRESHOLD_FLOOR_M, outlier_threshold * 0.5)
        self.dynamic_threshold = outlier_threshold
        self.velocity_window = velocity_window
        self.enable_prediction = enable_prediction

        self.position_history: deque = deque(maxlen=window_size)
        self.velocity_history: deque = deque(maxlen=velocity_window)
        self.time_history: deque = deque(maxlen=velocity_window)

        self.last_valid_position: Optional[tuple] = None
        self.last_valid_time: Optional[float] = None
        self.prev_valid_position: Optional[tuple] = None

        self.estimated_velocity = np.zeros(3)
        self.velocity_magnitude_ema = 0.0
        self.step_distance_ema = 0.0
        self.dt_ema: Optional[float] = None

        self.outlier_detected = False
        self.outlier_count = 0
        self.consecutive_outliers = 0

        self.total_samples = 0
        self.outlier_samples = 0
        self.last_source: Optional[str] = None
        self.last_confidence = 1.0

    @classmethod
    def from_config(cls, filter_config: dict) -> "PositionFilter":
        """control.json の "filter" セクションから生成する。"""
        return cls(
            window_size=filter_config["window_size"],
            outlier_threshold=filter_config["outlier_threshold"],
            velocity_window=filter_config["velocity_window"],
            enable_prediction=filter_config["enable_prediction"],
            max_outlier_threshold=filter_config["max_outlier_threshold"],
        )

    def reset(self) -> None:
        """内部状態をリセットする。"""
        self.position_history.clear()
        self.velocity_history.clear()
        self.time_history.clear()
        self.last_valid_position = None
        self.last_valid_time = None
        self.prev_valid_position = None
        self.estimated_velocity = np.zeros(3)
        self.velocity_magnitude_ema = 0.0
        self.step_distance_ema = 0.0
        self.dynamic_threshold = self.base_threshold
        self.dt_ema = None
        self.outlier_detected = False
        self.outlier_count = 0
        self.consecutive_outliers = 0
        self.total_samples = 0
        self.outlier_samples = 0
        self.last_source = None
        self.last_confidence = 1.0

    def _compute_dynamic_threshold(self, time_diff, marker_count,
                                   tracking_valid, quality_weight) -> float:
        """直近の動きとトラッキング品質から外れ値判定閾値を更新する。"""
        motion_allowance = self.step_distance_ema * MOTION_ALLOWANCE_GAIN

        if time_diff is None and self.dt_ema is not None:
            time_diff = self.dt_ema

        speed_allowance = 0.0
        if time_diff is not None:
            speed_allowance = (self.velocity_magnitude_ema
                               * max(time_diff, 0.0) * SPEED_ALLOWANCE_GAIN)

        adaptive = self.base_threshold + motion_allowance + speed_allowance

        if marker_count is not None:
            marker_ratio = min(max(marker_count, 0), MARKER_FULL_COUNT) / MARKER_FULL_COUNT
            adaptive *= (0.9 + 0.2 * marker_ratio)

        if not tracking_valid:
            adaptive *= 0.8

        if quality_weight is not None:
            adaptive *= (0.8 + 0.4 * max(0.0, min(quality_weight, 1.2)))

        adaptive = max(self.min_threshold, min(self.max_threshold, adaptive))

        if self.dynamic_threshold is None:
            self.dynamic_threshold = adaptive
        else:
            self.dynamic_threshold = (THRESHOLD_EMA_KEEP * self.dynamic_threshold
                                      + (1.0 - THRESHOLD_EMA_KEEP) * adaptive)
        return self.dynamic_threshold

    def is_outlier(self, position, current_time=None, tracking_valid=True,
                   marker_count=None, quality_weight=None) -> tuple[bool, float]:
        """位置データが外れ値かを判定する。Returns: (is_outlier, threshold)。"""
        if self.last_valid_position is None:
            return False, self.base_threshold

        distance = float(np.linalg.norm(
            np.array(position) - np.array(self.last_valid_position)))

        time_diff = None
        if self.last_valid_time is not None:
            if current_time is None:
                current_time = time.monotonic()
            time_diff = current_time - self.last_valid_time

        threshold = self._compute_dynamic_threshold(
            time_diff, marker_count, tracking_valid, quality_weight)
        return distance > threshold, threshold

    def estimate_velocity(self) -> np.ndarray:
        """位置履歴から速度ベクトルを推定する(始点-終点の差分)。"""
        if len(self.velocity_history) < 2:
            return np.zeros(3)

        positions = np.array(list(self.velocity_history))
        times = np.array(list(self.time_history))
        rel_times = times - times[0]

        velocity = np.zeros(3)
        if rel_times[-1] > 0:
            velocity = (positions[-1] - positions[0]) / rel_times[-1]
        return velocity

    def predict_position(self, time_delta: float):
        """推定速度から time_delta 秒後の位置を予測する。"""
        if self.last_valid_position is None:
            return None
        predicted = np.array(self.last_valid_position) + self.estimated_velocity * time_delta
        return tuple(predicted)

    def apply_moving_average(self):
        """新しいデータほど重い指数重み付き移動平均を返す。"""
        if len(self.position_history) == 0:
            return None
        positions = np.array(list(self.position_history))
        weights = np.exp(np.linspace(-1, 0, len(positions)))
        weights /= weights.sum()
        return tuple(np.average(positions, weights=weights, axis=0))

    def process_position(self, position, marker_count=None, current_time=None,
                         tracking_valid=True, quality_weight=None,
                         rigid_body_error=None, source="rigid_body") -> dict:
        """位置データを処理する(メインのフィルタリング関数)。

        Returns:
            dict: filtered_position / raw_position / is_outlier / used_prediction /
                  confidence / marker_count / consecutive_outliers / source /
                  threshold / tracking_valid / rigid_body_error
        """
        if current_time is None:
            current_time = time.monotonic()

        self.total_samples += 1
        self.last_source = source

        # 初回サンプルはそのまま受理
        if self.last_valid_position is None:
            self.last_valid_position = position
            self.last_valid_time = current_time
            self.position_history.append(position)
            self.velocity_history.append(position)
            self.time_history.append(current_time)
            self.dynamic_threshold = self.base_threshold
            self.last_confidence = 1.0
            return {
                "filtered_position": position,
                "raw_position": position,
                "is_outlier": False,
                "used_prediction": False,
                "confidence": 1.0,
                "marker_count": marker_count,
                "consecutive_outliers": 0,
                "source": source,
                "threshold": self.dynamic_threshold,
                "tracking_valid": tracking_valid,
                "rigid_body_error": rigid_body_error,
            }

        is_outlier, threshold = self.is_outlier(
            position, current_time=current_time, tracking_valid=tracking_valid,
            marker_count=marker_count, quality_weight=quality_weight)

        if is_outlier:
            self.outlier_detected = True
            self.outlier_count += 1
            self.outlier_samples += 1
            self.consecutive_outliers += 1

            used_prediction = False
            filtered_position = self.last_valid_position
            if self.enable_prediction and self.last_valid_time is not None:
                predicted = self.predict_position(current_time - self.last_valid_time)
                if predicted is not None:
                    filtered_position = predicted
                    used_prediction = True

            confidence = OUTLIER_BASE_CONFIDENCE
            if marker_count is not None:
                confidence *= max(0.25, marker_count / MARKER_FULL_COUNT)
            if not tracking_valid:
                confidence *= 0.4
            if quality_weight is not None:
                confidence *= 0.5 + 0.5 * max(0.0, min(quality_weight, 1.0))
            if rigid_body_error is not None:
                confidence *= 1.0 / (1.0 + max(0.0, rigid_body_error))
        else:
            self.outlier_detected = False
            self.consecutive_outliers = 0

            self.position_history.append(position)
            self.velocity_history.append(position)
            self.time_history.append(current_time)

            self.estimated_velocity = self.estimate_velocity()
            velocity_mag = float(np.linalg.norm(self.estimated_velocity))
            self.velocity_magnitude_ema = (VELOCITY_EMA_KEEP * self.velocity_magnitude_ema
                                           + (1.0 - VELOCITY_EMA_KEEP) * velocity_mag)

            filtered_position = self.apply_moving_average()
            if filtered_position is None:
                filtered_position = position

            step_distance = float(np.linalg.norm(
                np.array(filtered_position) - np.array(self.last_valid_position)))
            self.step_distance_ema = (STEP_EMA_KEEP * self.step_distance_ema
                                      + (1.0 - STEP_EMA_KEEP) * step_distance)
            self.prev_valid_position = self.last_valid_position
            self.last_valid_position = filtered_position
            self.last_valid_time = current_time

            if len(self.time_history) >= 2:
                dt = self.time_history[-1] - self.time_history[-2]
                if dt > 0:
                    self.dt_ema = (DT_EMA_KEEP * self.dt_ema + (1.0 - DT_EMA_KEEP) * dt
                                   if self.dt_ema is not None else dt)

            used_prediction = False
            confidence = 1.0
            if marker_count is not None:
                marker_ratio = min(max(marker_count, 0), MARKER_FULL_COUNT) / MARKER_FULL_COUNT
                confidence *= 0.7 + 0.3 * marker_ratio
            if not tracking_valid:
                confidence *= 0.7
            if quality_weight is not None:
                confidence *= max(0.4, min(1.0, quality_weight))
            if rigid_body_error is not None:
                confidence *= 1.0 / (1.0 + max(0.0, rigid_body_error))

        confidence = max(CONFIDENCE_FLOOR, min(1.0, confidence))
        self.last_confidence = confidence

        return {
            "filtered_position": filtered_position,
            "raw_position": position,
            "is_outlier": is_outlier,
            "used_prediction": used_prediction,
            "confidence": confidence,
            "marker_count": marker_count,
            "consecutive_outliers": self.consecutive_outliers,
            "source": source,
            "threshold": threshold,
            "tracking_valid": tracking_valid,
            "rigid_body_error": rigid_body_error,
        }

    def get_statistics(self) -> dict:
        """フィルタの統計情報を返す。"""
        outlier_rate = (self.outlier_samples / self.total_samples
                        if self.total_samples > 0 else 0.0)
        return {
            "total_samples": self.total_samples,
            "outlier_samples": self.outlier_samples,
            "outlier_rate": outlier_rate,
            "current_threshold": self.dynamic_threshold,
            "consecutive_outliers": self.consecutive_outliers,
            "estimated_velocity": self.estimated_velocity.tolist(),
            "last_confidence": self.last_confidence,
            "last_source": self.last_source,
        }
