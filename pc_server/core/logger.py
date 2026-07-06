"""CSV フライトログ(ログ ON 時のみ)。

- 出力先: リポジトリ直下 logs/YYYYMMDD_HHMMSS_<mode>.csv
- 50Hz の制御行(CMD_SETPOINT 送信ごとに1行)+最新テレメトリ/mocap
  スナップショットの結合。列定義は docs/LOG_STRUCTURE.md に文書化
  (旧 NatNet_PID_Controller の57列の語彙を継承し、TLM_STATE 列で拡張)。
- 値が未取得の列は空文字。0/1 フラグは文字列 "0"/"1"。
- 経過時間は time.monotonic() 基準。timestamp 列のみ壁時計(ISO8601)。
"""

from __future__ import annotations

import csv
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from .config import LOGS_DIR

# 列定義(docs/LOG_STRUCTURE.md と1対1で対応させること)
COLUMNS: tuple[str, ...] = (
    # --- セッション / タイミング ---
    "timestamp", "elapsed_time", "mode", "phase",
    "command_sequence", "send_success", "feedback_latency_ms",
    # --- 指令(送信した CMD_SETPOINT、バイアス加算後) ---
    "roll_ref_rad", "pitch_ref_rad", "roll_ref_deg", "pitch_ref_deg", "alt_ref_m",
    "roll_bias_deg", "pitch_bias_deg",
    # --- 位置と誤差(Position モードのみ。制御座標系 m) ---
    "pos_x", "pos_y", "pos_z",
    "raw_pos_x", "raw_pos_y", "raw_pos_z",
    "error_x", "error_y",
    "target_x", "target_y", "target_z",
    # --- PID 成分(Position モードのみ) ---
    "pid_x_p", "pid_x_i", "pid_x_d",
    "pid_y_p", "pid_y_i", "pid_y_d",
    # --- フィルタ状態とデータ由来(Position モードのみ) ---
    "data_valid", "control_active", "mocap_dropout", "is_outlier", "used_prediction",
    "confidence", "consecutive_outliers", "data_source",
    "filter_threshold", "tracking_valid",
    # --- リジッドボディ / フレーム診断 ---
    "rb_error", "rb_marker_count",
    "frame_number", "marker_count", "frame_dt_ms", "mocap_age_ms",
    # --- 機体テレメトリ(最新 TLM_STATE のスナップショット) ---
    "tlm_age_ms", "tlm_seq_echo", "tlm_elapsed_ms",
    "tlm_state", "tlm_state_name", "tlm_flags", "tlm_reason", "tlm_reason_name",
    "tlm_roll_rad", "tlm_pitch_rad", "tlm_yaw_rad",
    "tlm_p_rad_s", "tlm_q_rad_s", "tlm_r_rad_s",
    "tlm_roll_ref_rad", "tlm_pitch_ref_rad", "tlm_alt_ref_m",
    "tlm_altitude_tof_m", "tlm_altitude_est_m",
    "tlm_alt_velocity_m_s", "tlm_z_dot_ref_m_s",
    "tlm_voltage_v",
    "tlm_duty_fr", "tlm_duty_fl", "tlm_duty_rr", "tlm_duty_rl",
    "tlm_ax_g", "tlm_ay_g", "tlm_az_g", "tlm_loop_dt_us",
)

FLOAT_DECIMALS = 6   # CSV 上の float 桁数


def _format_cell(value) -> str:
    """セル値を CSV 文字列に変換する(None → 空、bool → 0/1)。"""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{value:.{FLOAT_DECIMALS}f}"
    return str(value)


class FlightLogger:
    """CSV フライトロガー。log_row() はスレッド安全(50Hz送信スレッドから呼ぶ)。"""

    def __init__(self, logs_dir: Path = LOGS_DIR, flush_every_rows: int = 50) -> None:
        self._logs_dir = Path(logs_dir)
        self._flush_every_rows = flush_every_rows
        self._lock = threading.Lock()
        self._file = None
        self._writer: Optional[csv.writer] = None
        self._file_path: Optional[Path] = None
        self._t0: Optional[float] = None
        self._rows_since_flush = 0

    @property
    def active(self) -> bool:
        with self._lock:
            return self._file is not None

    @property
    def file_path(self) -> Optional[Path]:
        with self._lock:
            return self._file_path

    def start(self, mode: str) -> Path:
        """ログファイルを作成しヘッダを書く。既に開いていれば閉じてから開く。"""
        self.stop()
        self._logs_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self._logs_dir / f"{stamp}_{mode}.csv"
        with self._lock:
            self._file = open(path, "w", newline="", encoding="utf-8")
            self._writer = csv.writer(self._file)
            self._writer.writerow(COLUMNS)
            self._file.flush()
            self._file_path = path
            self._t0 = time.monotonic()
            self._rows_since_flush = 0
        return path

    def stop(self) -> None:
        with self._lock:
            file = self._file
            self._file = None
            self._writer = None
            self._file_path = None
            self._t0 = None
        if file is not None:
            try:
                file.close()
            except OSError:
                pass

    def log_row(self, row: dict) -> None:
        """1行を書き込む。row は COLUMNS のキーを持つ dict(欠損キーは空欄)。

        timestamp / elapsed_time は本メソッドが自動付与する。
        """
        with self._lock:
            if self._writer is None or self._t0 is None:
                return
            values = dict(row)
            values.setdefault("timestamp", datetime.now().isoformat(timespec="milliseconds"))
            values.setdefault("elapsed_time", time.monotonic() - self._t0)
            self._writer.writerow([_format_cell(values.get(col)) for col in COLUMNS])
            self._rows_since_flush += 1
            if self._rows_since_flush >= self._flush_every_rows:
                self._file.flush()
                self._rows_since_flush = 0
