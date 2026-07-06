"""拡張PIDコントローラ(legacy NatNet_PID_Controller/pid_controller.py の移植・整理)。

整理内容(ARCHITECTURE.md の指示):
- 負ゲイン規約は廃止。ゲインはすべて正値で与え、軸の符号は座標変換
  (control.json の coordinate_transform)側で扱う。
- 時刻は time.monotonic() のみ(time.time() 禁止)。
- ゲイン等の数値は config/control.json から供給する(本モジュールに
  チューニング値のマジックナンバーを置かない)。

アルゴリズム自体(D項LPF / I項条件付き更新 / 異常時減衰)は飛行実績のある
旧実装と同一に保つ。
"""

from __future__ import annotations

import math
import time
from typing import Optional

# --- アルゴリズム固有の定数(チューニング値ではなく旧実装の構造定数) ---
INITIAL_DT_S = 0.01            # 初回呼び出し時の仮 dt
MIN_DT_S = 0.001               # これ未満の dt はスキップ(ゼロ除算防止)
ANOMALY_RECOVERY_TICKS = 10    # 異常回復後に I 項を減衰させ続ける回数
INVALID_DATA_I_DECAY = 0.995   # データ無効時の I 項微減衰率
INVALID_DATA_D_SCALE = 0.3     # データ無効時の D 項抑制係数
INTEGRAL_ERROR_FACTOR_GAIN = 2.0  # 動的アンチワインドアップの誤差感度


class PIDController:
    """1軸の拡張PIDコントローラ。

    拡張機能:
    - D項の低域通過フィルタ
    - I項の条件付き更新(大誤差時は更新しない)+異常時の減衰
    - 出力制限と動的アンチワインドアップ
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_limit: Optional[tuple[float, float]] = None,
                 d_filter_alpha: float = 0.7, i_decay_rate: float = 0.98,
                 i_update_threshold: float = 0.5, enable_i_control: bool = True) -> None:
        if kp < 0 or ki < 0 or kd < 0:
            # 負ゲイン規約の継承を構造的に防ぐ(符号は座標変換側で扱う)
            raise ValueError("PID gains must be non-negative; "
                             "handle axis sign in coordinate_transform")
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.d_filter_alpha = d_filter_alpha
        self.i_decay_rate = i_decay_rate
        self.i_update_threshold = i_update_threshold
        self.enable_i_control = enable_i_control

        # 内部状態
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time: Optional[float] = None
        self.filtered_derivative = 0.0

        # デバッグ/ログ用の各項
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0

        # 異常状態管理
        self.i_update_suspended = False
        self.anomaly_detected = False
        self.anomaly_recovery_count = 0

    def reset(self) -> None:
        """内部状態をすべてリセットする。"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        self.filtered_derivative = 0.0
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0
        self.i_update_suspended = False
        self.anomaly_detected = False
        self.anomaly_recovery_count = 0

    def set_anomaly_state(self, is_anomaly: bool) -> None:
        """異常状態を設定する(異常中は I 項更新を停止、回復後は減衰期間)。"""
        if is_anomaly and not self.anomaly_detected:
            self.anomaly_detected = True
            self.i_update_suspended = True
            self.anomaly_recovery_count = 0
        elif not is_anomaly and self.anomaly_detected:
            self.anomaly_detected = False
            self.anomaly_recovery_count = ANOMALY_RECOVERY_TICKS

    def calculate(self, error: float, current_time: Optional[float] = None,
                  is_data_valid: bool = True) -> float:
        """PID出力を計算する。

        Args:
            error: 目標値 - 現在値
            current_time: time.monotonic() 基準の時刻(None なら内部で取得)
            is_data_valid: データ有効性(False で I 項減衰・D 項抑制)
        """
        if current_time is None:
            current_time = time.monotonic()

        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            dt = INITIAL_DT_S
        else:
            dt = current_time - self.prev_time

        if dt < MIN_DT_S:
            # 呼び出し間隔が短すぎる場合は前回値を維持
            return self.last_p_term + self.last_i_term + self.last_d_term

        # P項
        p_term = self.kp * error
        self.last_p_term = p_term

        # I項(条件付き更新)
        if self.enable_i_control:
            if self.anomaly_recovery_count > 0:
                # 異常回復期間中は I 項を徐々に減衰
                self.integral *= self.i_decay_rate
                self.anomaly_recovery_count -= 1
                if self.anomaly_recovery_count == 0:
                    self.i_update_suspended = False

            should_update_i = (
                not self.i_update_suspended
                and is_data_valid
                and abs(error) < self.i_update_threshold
            )
            if should_update_i:
                self.integral += error * dt
                if self.output_limit is not None and self.ki != 0:
                    # 動的アンチワインドアップ(誤差が大きいほど上限を絞る)
                    error_factor = math.exp(-abs(error) * INTEGRAL_ERROR_FACTOR_GAIN)
                    max_integral = abs(self.output_limit[1] / self.ki) * error_factor
                    self.integral = max(-max_integral, min(max_integral, self.integral))
            elif not is_data_valid:
                self.integral *= INVALID_DATA_I_DECAY
        else:
            self.integral += error * dt
            if self.output_limit is not None and self.ki != 0:
                max_integral = abs(self.output_limit[1] / self.ki)
                self.integral = max(-max_integral, min(max_integral, self.integral))

        i_term = self.ki * self.integral
        self.last_i_term = i_term

        # D項(低域通過フィルタ付き)
        raw_derivative = (error - self.prev_error) / dt
        self.filtered_derivative = (
            self.d_filter_alpha * raw_derivative
            + (1.0 - self.d_filter_alpha) * self.filtered_derivative
        )
        derivative = self.filtered_derivative
        if not is_data_valid:
            derivative *= INVALID_DATA_D_SCALE
        d_term = self.kd * derivative
        self.last_d_term = d_term

        output = p_term + i_term + d_term
        if self.output_limit is not None:
            output = max(self.output_limit[0], min(self.output_limit[1], output))

        self.prev_error = error
        self.prev_time = current_time
        return output

    def get_components(self) -> tuple[float, float, float]:
        """直近の (P項, I項, D項) を返す(ログ用)。"""
        return (self.last_p_term, self.last_i_term, self.last_d_term)


class XYPIDController:
    """XY平面用の2軸独立PID。x軸出力→roll_ref、y軸出力→pitch_ref に対応する。"""

    def __init__(self, kp_x: float, ki_x: float, kd_x: float,
                 kp_y: float, ki_y: float, kd_y: float,
                 output_limit: Optional[tuple[float, float]] = None,
                 d_filter_alpha: float = 0.7, i_decay_rate: float = 0.98,
                 i_update_threshold: float = 0.5, enable_i_control: bool = True) -> None:
        self.pid_x = PIDController(kp_x, ki_x, kd_x, output_limit,
                                   d_filter_alpha, i_decay_rate,
                                   i_update_threshold, enable_i_control)
        self.pid_y = PIDController(kp_y, ki_y, kd_y, output_limit,
                                   d_filter_alpha, i_decay_rate,
                                   i_update_threshold, enable_i_control)

    @classmethod
    def from_config(cls, pid_config: dict) -> "XYPIDController":
        """control.json の "pid" セクションから生成する。"""
        limit = pid_config.get("output_limit")
        return cls(
            kp_x=pid_config["x"]["kp"], ki_x=pid_config["x"]["ki"], kd_x=pid_config["x"]["kd"],
            kp_y=pid_config["y"]["kp"], ki_y=pid_config["y"]["ki"], kd_y=pid_config["y"]["kd"],
            output_limit=tuple(limit) if limit is not None else None,
            d_filter_alpha=pid_config["d_filter_alpha"],
            i_decay_rate=pid_config["i_decay_rate"],
            i_update_threshold=pid_config["i_update_threshold"],
            enable_i_control=pid_config["enable_i_control"],
        )

    def reset(self) -> None:
        self.pid_x.reset()
        self.pid_y.reset()

    def set_anomaly_state(self, is_anomaly: bool) -> None:
        self.pid_x.set_anomaly_state(is_anomaly)
        self.pid_y.set_anomaly_state(is_anomaly)

    def calculate(self, error_x: float, error_y: float,
                  current_time: Optional[float] = None,
                  is_data_valid: bool = True) -> tuple[float, float]:
        output_x = self.pid_x.calculate(error_x, current_time, is_data_valid)
        output_y = self.pid_y.calculate(error_y, current_time, is_data_valid)
        return (output_x, output_y)

    def get_all_components(self) -> dict:
        x_comp = self.pid_x.get_components()
        y_comp = self.pid_y.get_components()
        return {
            "x": {"p": x_comp[0], "i": x_comp[1], "d": x_comp[2]},
            "y": {"p": y_comp[0], "i": y_comp[1], "d": y_comp[2]},
        }
