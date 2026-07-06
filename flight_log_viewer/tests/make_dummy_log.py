#!/usr/bin/env python3
"""合成ダミーフライトログ(94列・50Hz)の生成スクリプト。

flight_log_viewer の全機能(静止画・ヨー解析・アニメーション・レポート)を
実ログなしで検証するための CSV を生成する。列は viewer.constants.V2_COLUMNS
(= docs/LOG_STRUCTURE.md v2 契約の 94 列)と同一順序で出力する。

シナリオ(position モード):
  接続 → 離陸 → ホバリング → 円軌道(ヨー指令ステップあり) → 着陸 → 完了
  - MoCap 真値ヨーに対し Madgwick=+1.5°/min、ジャイロ積算=−6°/min の
    ドリフトを与え、EKF はほぼ真値に追従させる(ヨー解析の検証用)。
  - NIS スパイクと ffg ゲート発火、b_m の緩慢ドリフトも合成する。

posture モードでは位置・MoCap・軌道列を空欄にする(実機と同じ挙動)。

使い方:
  python tests/make_dummy_log.py                     # ../../logs/dummy_position.csv
  python tests/make_dummy_log.py --mode posture      # ../../logs/dummy_posture.csv
  python tests/make_dummy_log.py --duration 20 --out /tmp/short.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

# flight_log_viewer/ を import パスに追加
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

from viewer.constants import (  # noqa: E402
    FF_STATUS_ANCHOR_VALID,
    FF_STATUS_EST_EKF,
    FF_STATUS_FFCAL_LOADED,
    FF_STATUS_MAG_FRESH,
    FF_STATUS_YAW_CTRL_ACTIVE,
    LOG_RATE_HZ,
    N_COLUMNS,
    TLM_FLAG_FLYING,
    TLM_FLAG_SETPOINT_FRESH,
    V2_COLUMNS,
)

DEFAULT_LOGS_DIR = _HERE.parent.parent / "logs"
FLOAT_DECIMALS = 6  # logger.py と同じ桁数

# FlightState(プロトコル準拠)
STATE_WAIT = 2
STATE_TAKEOFF = 3
STATE_HOVER = 4
STATE_LANDING = 5
STATE_COMPLETE = 6
STATE_NAMES = {
    STATE_WAIT: "WAIT", STATE_TAKEOFF: "TAKEOFF", STATE_HOVER: "HOVER",
    STATE_LANDING: "LANDING", STATE_COMPLETE: "COMPLETE",
}

# ffg ゲートビット(yaw 側定義)
FFG_R_INFLATED = 1 << 0
FFG_NIS_REJECT = 1 << 1
FFG_TILT_SKIP = 1 << 4

# シナリオの時間配分(duration=40s 基準の比率)
PHASE_CONNECT_END = 0.05     # 接続(地上)
PHASE_TAKEOFF_END = 0.125    # 離陸
PHASE_HOVER_END = 0.30       # ホバリング
PHASE_CIRCLE_END = 0.80      # 円軌道
PHASE_LANDING_END = 0.875    # 着陸(以降 COMPLETE)

# 飛行パラメータ
HOVER_ALT_M = 0.35
CIRCLE_RADIUS_M = 0.5
CIRCLE_PERIOD_S = 10.0
YAW_CMD_STEP1_DEG = 45.0     # 1 回目のヨー指令
YAW_CMD_STEP2_DEG = -30.0    # 2 回目のヨー指令
YAW_RESPONSE_TAU_S = 0.8     # ヨー応答の時定数

# 推定器ドリフト(ヨー解析の検証値)
MADGWICK_DRIFT_DEG_MIN = 1.5
GYRO_INT_DRIFT_DEG_MIN = -6.0
EKF_BIAS_DEG = 0.5

RANDOM_SEED = 20260706


def _fmt(value) -> str:
    """logger.py の _format_cell と同じ書式(None → 空、bool → 0/1)。"""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{value:.{FLOAT_DECIMALS}f}"
    return str(value)


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def generate_rows(mode: str, duration_s: float) -> list[dict]:
    """1 行 = COLUMNS キーの dict のリストを生成する。"""
    rng = np.random.default_rng(RANDOM_SEED)
    n_rows = int(duration_s * LOG_RATE_HZ)
    t0_wall = datetime.now()

    # 位相の絶対時刻
    t_connect = duration_s * PHASE_CONNECT_END
    t_takeoff = duration_s * PHASE_TAKEOFF_END
    t_hover = duration_s * PHASE_HOVER_END
    t_circle = duration_s * PHASE_CIRCLE_END
    t_landing = duration_s * PHASE_LANDING_END
    t_yaw_on = t_hover * 0.9                      # ヨー制御 ON
    t_yaw_step1 = t_hover + (t_circle - t_hover) * 0.1
    t_yaw_step2 = t_hover + (t_circle - t_hover) * 0.55

    rows: list[dict] = []
    yaw_true_deg = 0.0        # MoCap 真値ヨー(1 次遅れ応答の状態)
    circle_phase = 0.0        # 円軌道位相 [rad]
    dt = 1.0 / LOG_RATE_HZ

    for i in range(n_rows):
        t = i * dt
        row: dict = {col: None for col in V2_COLUMNS}

        # ---- フェーズ判定 ----
        if t < t_connect:
            state, phase, flying = STATE_WAIT, "connected", False
        elif t < t_takeoff:
            state, phase, flying = STATE_TAKEOFF, "flying", True
        elif t < t_hover:
            state, phase, flying = STATE_HOVER, "flying", True
        elif t < t_circle:
            state, phase, flying = STATE_HOVER, "flying", True
        elif t < t_landing:
            state, phase, flying = STATE_LANDING, "flying", True
        else:
            state, phase, flying = STATE_COMPLETE, "connected", False

        in_circle = t_hover <= t < t_circle
        yaw_ctrl_on = flying and t >= t_yaw_on

        # ---- セッション / タイミング ----
        row["timestamp"] = (t0_wall + timedelta(seconds=t)).isoformat(
            timespec="milliseconds")
        row["elapsed_time"] = t
        row["mode"] = mode
        row["phase"] = phase
        row["command_sequence"] = i + 1
        row["send_success"] = 1
        row["feedback_latency_ms"] = 7.0 + float(rng.normal(0.0, 1.5))

        # ---- 目標位置(円軌道)と実位置 ----
        if in_circle:
            circle_phase = (circle_phase + 2.0 * math.pi * dt / CIRCLE_PERIOD_S) \
                % (2.0 * math.pi)
            target_x = CIRCLE_RADIUS_M * math.cos(circle_phase)
            target_y = CIRCLE_RADIUS_M * math.sin(circle_phase)
            traj_mode = 1
        else:
            target_x, target_y = 0.0, 0.0
            traj_mode = 0

        # 実位置: 目標に少し遅れて追従+ノイズ
        lag_phase = circle_phase - 0.25 if in_circle else 0.0
        if in_circle:
            pos_x = CIRCLE_RADIUS_M * math.cos(lag_phase) + float(rng.normal(0, 0.012))
            pos_y = CIRCLE_RADIUS_M * math.sin(lag_phase) + float(rng.normal(0, 0.012))
        else:
            pos_x = float(rng.normal(0, 0.02))
            pos_y = float(rng.normal(0, 0.02))

        # ---- 高度 ----
        if t < t_connect:
            alt = 0.0
        elif t < t_takeoff:
            alt = HOVER_ALT_M * (t - t_connect) / max(t_takeoff - t_connect, 1e-6)
        elif t < t_circle:
            alt = HOVER_ALT_M
        elif t < t_landing:
            # 着陸フェーズ: 一定レートで降下
            alt = HOVER_ALT_M * (1.0 - (t - t_circle) / max(t_landing - t_circle, 1e-6))
        else:
            alt = 0.0
        alt = max(alt, 0.0)

        # ---- ヨー(真値と各推定系統) ----
        if yaw_ctrl_on:
            if t >= t_yaw_step2:
                cmd_yaw_deg = YAW_CMD_STEP2_DEG
            elif t >= t_yaw_step1:
                cmd_yaw_deg = YAW_CMD_STEP1_DEG
            else:
                cmd_yaw_deg = 0.0
        else:
            cmd_yaw_deg = 0.0
        # 真値ヨーは指令に 1 次遅れで追従(制御 OFF 時は 0 付近を維持)
        target_yaw = cmd_yaw_deg if yaw_ctrl_on else 0.0
        yaw_true_deg += (target_yaw - yaw_true_deg) * (dt / YAW_RESPONSE_TAU_S)
        yaw_true_noisy = yaw_true_deg + float(rng.normal(0, 0.15))

        madgwick_deg = (yaw_true_deg + MADGWICK_DRIFT_DEG_MIN * (t / 60.0)
                        + float(rng.normal(0, 0.6)))
        ekf_deg = yaw_true_deg + EKF_BIAS_DEG + float(rng.normal(0, 0.25))
        gyro_int_deg = (yaw_true_deg + GYRO_INT_DRIFT_DEG_MIN * (t / 60.0)
                        + float(rng.normal(0, 0.3)))

        # ---- 姿勢指令 / 実測 ----
        roll_ref_deg = 2.0 * math.sin(2.0 * math.pi * t / 6.0) if flying else 0.0
        pitch_ref_deg = 1.5 * math.cos(2.0 * math.pi * t / 7.0) if flying else 0.0
        roll_meas_deg = roll_ref_deg + float(rng.normal(0, 0.4))
        pitch_meas_deg = pitch_ref_deg + float(rng.normal(0, 0.4))

        # ---- EKF 診断(NIS スパイク・ゲート発火・b_m ドリフト) ----
        nis = abs(float(rng.normal(2.0, 0.8)))
        ffg = 0
        t_spike1 = t_hover + (t_circle - t_hover) * 0.3
        t_spike2 = t_hover + (t_circle - t_hover) * 0.7
        if flying and t_spike1 <= t < t_spike1 + 0.6:
            nis = float(rng.uniform(6.5, 11.0))      # R 膨張域
            ffg |= FFG_R_INFLATED
        if flying and t_spike2 <= t < t_spike2 + 0.15:
            nis = float(rng.uniform(14.0, 18.0))     # 棄却域
            ffg |= FFG_NIS_REJECT
        if flying and abs(roll_meas_deg) > 1.9:      # まれに tilt スキップ
            if rng.random() < 0.05:
                ffg |= FFG_TILT_SKIP

        bm_x = 2.5 * min(t / duration_s, 1.0) + float(rng.normal(0, 0.05))
        bm_y = -1.2 * min(t / duration_s, 1.0) + float(rng.normal(0, 0.05))

        # ---- duty / 電源 ----
        duty_base = 0.45
        duty_offsets = {"fl": 0.010, "fr": -0.005, "rl": 0.003, "rr": -0.008}
        if flying:
            duty = {
                key: duty_base + offset + float(rng.normal(0, 0.006))
                for key, offset in duty_offsets.items()
            }
        else:
            duty = {key: 0.0 for key in duty_offsets}
        voltage = 4.15 - 0.40 * (t / duration_s) - (0.08 if flying else 0.0) \
            + float(rng.normal(0, 0.005))
        current = (2.6 + 0.4 * math.sin(2.0 * math.pi * t / 5.0)
                   + float(rng.normal(0, 0.05))) if flying else 0.05
        db_hat_x = 3.0 * duty["fl"] * math.sin(2.0 * math.pi * t / 4.0) if flying else 0.0
        db_hat_y = -2.0 * duty["fr"] * math.cos(2.0 * math.pi * t / 4.0) if flying else 0.0

        # ---- 指令列 ----
        row["roll_ref_rad"] = math.radians(roll_ref_deg)
        row["pitch_ref_rad"] = math.radians(pitch_ref_deg)
        row["roll_ref_deg"] = roll_ref_deg
        row["pitch_ref_deg"] = pitch_ref_deg
        row["alt_ref_m"] = alt if flying else 0.0
        row["roll_bias_deg"] = 0.3
        row["pitch_bias_deg"] = -0.2

        # ---- v2 ヨー指令列 ----
        row["cmd_yaw_ref_rad"] = math.radians(cmd_yaw_deg)
        row["cmd_yaw_ref_deg"] = cmd_yaw_deg
        row["yaw_ctrl_on"] = 1 if yaw_ctrl_on else 0

        # ---- 位置系(position モードのみ) ----
        if mode == "position":
            row["pos_x"], row["pos_y"], row["pos_z"] = pos_x, pos_y, alt
            row["raw_pos_x"] = pos_x + float(rng.normal(0, 0.004))
            row["raw_pos_y"] = pos_y + float(rng.normal(0, 0.004))
            row["raw_pos_z"] = alt + float(rng.normal(0, 0.004))
            row["error_x"] = target_x - pos_x
            row["error_y"] = target_y - pos_y
            row["target_x"], row["target_y"], row["target_z"] = target_x, target_y, HOVER_ALT_M
            row["pid_x_p"] = 0.02 * (target_x - pos_x)
            row["pid_x_i"] = 0.004 * math.sin(2.0 * math.pi * t / 9.0)
            row["pid_x_d"] = float(rng.normal(0, 0.002))
            row["pid_y_p"] = 0.02 * (target_y - pos_y)
            row["pid_y_i"] = 0.004 * math.cos(2.0 * math.pi * t / 9.0)
            row["pid_y_d"] = float(rng.normal(0, 0.002))
            row["data_valid"] = 1
            row["control_active"] = 1 if flying else 0
            row["mocap_dropout"] = 0
            row["is_outlier"] = 1 if rng.random() < 0.01 else 0
            row["used_prediction"] = 0
            row["confidence"] = min(1.0, 0.95 + float(rng.normal(0, 0.02)))
            row["consecutive_outliers"] = 0
            row["data_source"] = "rigid_body"
            row["filter_threshold"] = 0.35
            row["tracking_valid"] = 1
            row["rb_error"] = 0.0004 + float(rng.normal(0, 0.00005))
            row["rb_marker_count"] = 5 if rng.random() > 0.03 else 4
            row["frame_number"] = int(t * 100.0)
            row["marker_count"] = row["rb_marker_count"]
            row["frame_dt_ms"] = 10.0 + float(rng.normal(0, 0.5))
            row["mocap_age_ms"] = float(rng.uniform(1.0, 9.0))
            row["mocap_yaw_deg"] = yaw_true_noisy
            row["traj_mode"] = traj_mode
            row["traj_phase_rad"] = circle_phase if traj_mode == 1 else None

        # ---- テレメトリ(TLM_STATE スナップショット) ----
        row["tlm_age_ms"] = 20.0 if i % 2 else 0.5   # 25Hz TLM を 50Hz 行に結合
        row["tlm_seq_echo"] = max(0, i - 2)
        row["tlm_elapsed_ms"] = int(t * 1000.0) + 5000
        row["tlm_state"] = state
        row["tlm_state_name"] = STATE_NAMES[state]
        flags = TLM_FLAG_SETPOINT_FRESH
        if flying:
            flags |= TLM_FLAG_FLYING
        row["tlm_flags"] = flags
        row["tlm_reason"] = 1 if flying else 0
        row["tlm_reason_name"] = "START_CMD" if flying else "NONE"
        row["tlm_roll_rad"] = math.radians(roll_meas_deg)
        row["tlm_pitch_rad"] = math.radians(pitch_meas_deg)
        row["tlm_yaw_rad"] = _wrap_pi(math.radians(madgwick_deg))
        row["tlm_p_rad_s"] = float(rng.normal(0, 0.05))
        row["tlm_q_rad_s"] = float(rng.normal(0, 0.05))
        row["tlm_r_rad_s"] = float(rng.normal(0, 0.08))
        row["tlm_roll_ref_rad"] = math.radians(roll_ref_deg)
        row["tlm_pitch_ref_rad"] = math.radians(pitch_ref_deg)
        row["tlm_alt_ref_m"] = row["alt_ref_m"]
        row["tlm_altitude_tof_m"] = alt + float(rng.normal(0, 0.008))
        row["tlm_altitude_est_m"] = alt + float(rng.normal(0, 0.003))
        row["tlm_alt_velocity_m_s"] = float(rng.normal(0, 0.03))
        row["tlm_z_dot_ref_m_s"] = 0.0
        row["tlm_voltage_v"] = voltage
        row["tlm_duty_fr"] = duty["fr"]
        row["tlm_duty_fl"] = duty["fl"]
        row["tlm_duty_rr"] = duty["rr"]
        row["tlm_duty_rl"] = duty["rl"]
        row["tlm_ax_g"] = float(rng.normal(0, 0.02))
        row["tlm_ay_g"] = float(rng.normal(0, 0.02))
        row["tlm_az_g"] = 1.0 + float(rng.normal(0, 0.02))
        row["tlm_loop_dt_us"] = int(2500 + rng.normal(0, 30))

        # ---- v2 テレメトリ列 ----
        row["tlm_yaw_est_rad"] = _wrap_pi(math.radians(ekf_deg))
        row["tlm_yaw_gyro_int_rad"] = math.radians(gyro_int_deg)  # 積算(ラップなし)
        row["tlm_yaw_ref_rad"] = math.radians(cmd_yaw_deg) if yaw_ctrl_on else 0.0
        row["tlm_current_a"] = current
        row["tlm_db_hat_x_ut"] = db_hat_x
        row["tlm_db_hat_y_ut"] = db_hat_y
        row["tlm_bm_x_ut"] = bm_x
        row["tlm_bm_y_ut"] = bm_y
        row["tlm_nis"] = nis
        row["tlm_ffg"] = ffg
        ff_status = 2  # ff_mode=B
        ff_status |= FF_STATUS_EST_EKF | FF_STATUS_ANCHOR_VALID | FF_STATUS_FFCAL_LOADED
        if yaw_ctrl_on:
            ff_status |= FF_STATUS_YAW_CTRL_ACTIVE
        if i % 5 != 0:
            ff_status |= FF_STATUS_MAG_FRESH
        row["tlm_ff_status"] = ff_status

        rows.append(row)
    return rows


def write_csv(rows: list[dict], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(V2_COLUMNS)
        for row in rows:
            writer.writerow([_fmt(row.get(col)) for col in V2_COLUMNS])


def main() -> int:
    parser = argparse.ArgumentParser(description="ダミーフライトログ(94列)の生成")
    parser.add_argument("--mode", choices=("position", "posture"), default="position",
                        help="ログモード(posture では位置/MoCap 列が空欄)")
    parser.add_argument("--duration", type=float, default=40.0,
                        help="ログ長 [s](既定 40)")
    parser.add_argument("--out", type=Path, default=None,
                        help=f"出力パス(既定: {DEFAULT_LOGS_DIR}/dummy_<mode>.csv)")
    args = parser.parse_args()

    out_path = args.out or (DEFAULT_LOGS_DIR / f"dummy_{args.mode}.csv")
    rows = generate_rows(args.mode, args.duration)
    write_csv(rows, out_path)
    print(f"生成完了: {out_path} ({len(rows)}行 × {N_COLUMNS}列, mode={args.mode})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
