"""静止画グラフ一式(PNG)の生成。

旧 Drone_Log_Viewer(For_Research)の FlightLogFigureGenerator を
V2 の 94 列構成で再構築したもの。列が存在しない/全欠損のグラフは
自動でスキップする(Posture ログでは位置系グラフが出ない等)。
"""

from __future__ import annotations

from pathlib import Path

from . import jp_font

jp_font.setup_japanese_font()

import matplotlib.colors as mcolors  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .constants import (  # noqa: E402
    AX_BG,
    COLORS,
    FIG_BG,
    FIG_DPI,
    GRID_ALPHA,
    GRID_COLOR,
    LOOP_DT_NOMINAL_US,
    LOW_VOLTAGE_THRESHOLD_V,
    TEXT_COLOR,
)
from .loader import FlightLog  # noqa: E402
from .style import styled_legend, new_fig, save_fig  # noqa: E402


def _style_time_colorbar(fig, ax, norm, cmap, **kwargs):
    """plasma 時間カラーバーを白背景ライトテーマで付ける。"""
    cbar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, **kwargs)
    cbar.set_label("時間 [s]", color=TEXT_COLOR, fontsize=10)
    cbar.ax.yaxis.set_tick_params(color=TEXT_COLOR)
    for label in cbar.ax.yaxis.get_ticklabels():
        label.set_color(TEXT_COLOR)
    cbar.outline.set_edgecolor(GRID_COLOR)
    return cbar


def _fig_xy_trajectory(log: FlightLog, out_dir: Path) -> Path | None:
    """01: XY 軌跡+目標(円軌道の目標軌道も重畳)。"""
    if not (log.has("pos_x") and log.has("pos_y")):
        return None
    df = log.df
    fig, ax = new_fig(figsize=(8.0, 8.0))

    if log.has("raw_pos_x") and log.has("raw_pos_y"):
        ax.plot(df["raw_pos_x"], df["raw_pos_y"], color=COLORS["raw_trajectory"],
                linewidth=0.6, alpha=0.5, label="生位置")
    ax.plot(df["pos_x"], df["pos_y"], color=COLORS["trajectory"],
            linewidth=1.0, alpha=0.85, label="飛行軌跡(フィルタ後)")

    # 目標(ホバ時は点、円軌道時は目標軌道の線が現れる)
    if log.has("target_x") and log.has("target_y"):
        ax.plot(df["target_x"], df["target_y"], color=COLORS["target"],
                linewidth=1.2, linestyle="--", alpha=0.9, label="目標軌道")

    valid = df["pos_x"].notna() & df["pos_y"].notna()
    if valid.any():
        first = df.index[valid][0]
        last = df.index[valid][-1]
        ax.scatter(df.at[first, "pos_x"], df.at[first, "pos_y"], color=COLORS["start"],
                   s=110, zorder=5, edgecolors="#111111", linewidth=1.5, label="開始位置")
        ax.scatter(df.at[last, "pos_x"], df.at[last, "pos_y"], color=COLORS["end"],
                   s=110, zorder=5, edgecolors="#111111", linewidth=1.5, label="終了位置")

    ax.set_title("XY 飛行軌跡と目標", fontsize=14)
    ax.set_xlabel("X [m]", fontsize=11)
    ax.set_ylabel("Y [m]", fontsize=11)
    ax.set_aspect("equal")
    styled_legend(ax)
    return save_fig(fig, out_dir, "01_xy_trajectory.png")


def _fig_attitude(log: FlightLog, out_dir: Path) -> Path | None:
    """02: 姿勢(Roll/Pitch 指令 vs 実測)。"""
    if not (log.has("roll_ref_deg") or log.has("tlm_roll_deg")):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(2, 1, figsize=(12.0, 8.0), sharex=True)

    for ax, axis_label, ref_col, meas_col, meas_color in (
        (axes[0], "Roll", "roll_ref_deg", "tlm_roll_deg", COLORS["meas_roll"]),
        (axes[1], "Pitch", "pitch_ref_deg", "tlm_pitch_deg", COLORS["meas_pitch"]),
    ):
        if log.has(ref_col):
            ax.plot(t, df[ref_col], color="#666666", linewidth=1.0, alpha=0.8,
                    label=f"{axis_label} 指令")
        if log.has(meas_col):
            ax.plot(t, df[meas_col], color=meas_color, linewidth=1.0, alpha=0.9,
                    label=f"{axis_label} 実測(AHRS)")
        ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
        ax.set_ylabel("角度 [deg]", fontsize=10)
        styled_legend(ax)

    axes[0].set_title("姿勢: 指令 vs 実測", fontsize=14)
    axes[-1].set_xlabel("時間 [s]", fontsize=11)
    return save_fig(fig, out_dir, "02_attitude.png")


def _fig_altitude(log: FlightLog, out_dir: Path) -> Path | None:
    """03: 高度(目標/ToF/推定)と昇降速度。"""
    if not (log.has("alt_ref_m") or log.has("tlm_altitude_est_m")):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(2, 1, figsize=(12.0, 8.0), sharex=True,
                        height_ratios=[2.0, 1.0])

    ax = axes[0]
    if log.has("alt_ref_m"):
        ax.plot(t, df["alt_ref_m"], color=COLORS["alt_ref"], linewidth=1.0,
                linestyle="--", alpha=0.9, label="目標高度")
    if log.has("tlm_altitude_tof_m"):
        ax.plot(t, df["tlm_altitude_tof_m"], color=COLORS["alt_tof"], linewidth=0.8,
                alpha=0.7, label="ToF 生値")
    if log.has("tlm_altitude_est_m"):
        ax.plot(t, df["tlm_altitude_est_m"], color=COLORS["alt_est"], linewidth=1.2,
                alpha=0.9, label="推定高度")
    ax.set_title("高度", fontsize=14)
    ax.set_ylabel("高度 [m]", fontsize=10)
    styled_legend(ax)

    ax = axes[1]
    if log.has("tlm_alt_velocity_m_s"):
        ax.plot(t, df["tlm_alt_velocity_m_s"], color=COLORS["alt_est"], linewidth=1.0,
                alpha=0.9, label="昇降速度")
    if log.has("tlm_z_dot_ref_m_s"):
        ax.plot(t, df["tlm_z_dot_ref_m_s"], color=COLORS["alt_ref"], linewidth=1.0,
                linestyle="--", alpha=0.8, label="昇降速度指令")
    ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
    ax.set_ylabel("速度 [m/s]", fontsize=10)
    ax.set_xlabel("時間 [s]", fontsize=11)
    styled_legend(ax)
    return save_fig(fig, out_dir, "03_altitude.png")


def _fig_position_tracking(log: FlightLog, out_dir: Path) -> Path | None:
    """04: 位置追従(X/Y の目標 vs 実測+誤差)。Position モードのみ。"""
    if not (log.has("pos_x") and log.has("target_x")):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(3, 1, figsize=(12.0, 10.0), sharex=True)

    for ax, axis_label, pos_col, target_col in (
        (axes[0], "X", "pos_x", "target_x"),
        (axes[1], "Y", "pos_y", "target_y"),
    ):
        ax.plot(t, df[target_col], color=COLORS["target"], linewidth=1.0,
                linestyle="--", alpha=0.9, label=f"目標 {axis_label}")
        ax.plot(t, df[pos_col], color=COLORS["trajectory"], linewidth=1.0,
                alpha=0.9, label=f"実測 {axis_label}")
        ax.set_ylabel(f"{axis_label} [m]", fontsize=10)
        styled_legend(ax)

    ax = axes[2]
    if log.has("error_x"):
        ax.plot(t, df["error_x"], color="#dc2626", linewidth=1.0, alpha=0.9, label="誤差 X")
    if log.has("error_y"):
        ax.plot(t, df["error_y"], color="#0d9488", linewidth=1.0, alpha=0.9, label="誤差 Y")
    ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
    ax.set_ylabel("誤差 [m]", fontsize=10)
    ax.set_xlabel("時間 [s]", fontsize=11)
    styled_legend(ax)

    axes[0].set_title("位置追従(目標 vs 実測)", fontsize=14)
    return save_fig(fig, out_dir, "04_position_tracking.png")


def _fig_pid(log: FlightLog, out_dir: Path) -> Path | None:
    """05: XY PID 成分。Position モードのみ。"""
    if not (log.has("pid_x_p") or log.has("pid_y_p")):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(2, 1, figsize=(12.0, 8.0), sharex=True)

    for ax, axis_label, prefix in ((axes[0], "X軸", "pid_x"), (axes[1], "Y軸", "pid_y")):
        for comp, label in (("p", "P"), ("i", "I"), ("d", "D")):
            col = f"{prefix}_{comp}"
            if log.has(col):
                ax.plot(t, df[col], color=COLORS[comp], linewidth=1.0, alpha=0.9,
                        label=label)
        ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
        ax.set_ylabel(f"{axis_label} 出力", fontsize=10)
        styled_legend(ax)

    axes[0].set_title("XY PID 成分", fontsize=14)
    axes[-1].set_xlabel("時間 [s]", fontsize=11)
    return save_fig(fig, out_dir, "05_pid_components.png")


def _fig_duty(log: FlightLog, out_dir: Path) -> Path | None:
    """06: モーター duty(FL/FR/RL/RR)。"""
    duty_cols = (
        ("tlm_duty_fl", "FL", COLORS["duty_fl"]),
        ("tlm_duty_fr", "FR", COLORS["duty_fr"]),
        ("tlm_duty_rl", "RL", COLORS["duty_rl"]),
        ("tlm_duty_rr", "RR", COLORS["duty_rr"]),
    )
    if not any(log.has(col) for col, _, _ in duty_cols):
        return None
    t = log.t
    df = log.df
    fig, ax = new_fig(figsize=(12.0, 5.0))
    for col, label, color in duty_cols:
        if log.has(col):
            ax.plot(t, df[col], color=color, linewidth=0.8, alpha=0.9, label=label)
    ax.set_title("モーター duty", fontsize=14)
    ax.set_xlabel("時間 [s]", fontsize=11)
    ax.set_ylabel("duty (0-1)", fontsize=11)
    ax.set_ylim(bottom=0)
    styled_legend(ax, ncol=4)
    return save_fig(fig, out_dir, "06_duty.png")


def _fig_power(log: FlightLog, out_dir: Path) -> Path | None:
    """07: バッテリ電圧と総電流(v2 の tlm_current_a)。"""
    if not (log.has("tlm_voltage_v") or log.has("tlm_current_a")):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(2, 1, figsize=(12.0, 8.0), sharex=True)

    ax = axes[0]
    if log.has("tlm_voltage_v"):
        ax.plot(t, df["tlm_voltage_v"], color=COLORS["voltage"], linewidth=1.0,
                alpha=0.9, label="バッテリ電圧")
        ax.axhline(y=LOW_VOLTAGE_THRESHOLD_V, color="#ef4444", linestyle="--",
                   alpha=0.8, label=f"低電圧しきい値 {LOW_VOLTAGE_THRESHOLD_V}V")
    ax.set_title("電源(電圧 / 電流)", fontsize=14)
    ax.set_ylabel("電圧 [V]", fontsize=10)
    styled_legend(ax)

    ax = axes[1]
    if log.has("tlm_current_a"):
        ax.plot(t, df["tlm_current_a"], color=COLORS["current"], linewidth=1.0,
                alpha=0.9, label="総電流(INA3221, 20Hz)")
    ax.set_ylabel("電流 [A]", fontsize=10)
    ax.set_xlabel("時間 [s]", fontsize=11)
    styled_legend(ax)
    return save_fig(fig, out_dir, "07_power.png")


def _fig_latency(log: FlightLog, out_dir: Path) -> Path | None:
    """08: フィードバックレイテンシと機体ループ周期。"""
    if not (log.has("feedback_latency_ms") or log.has("tlm_loop_dt_us")):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(2, 1, figsize=(12.0, 8.0), sharex=True)

    ax = axes[0]
    if log.has("feedback_latency_ms"):
        ax.plot(t, df["feedback_latency_ms"], color=COLORS["latency"], linewidth=0.8,
                alpha=0.9, label="往復レイテンシ(seq_echo)")
    ax.set_title("通信レイテンシ / 制御周期", fontsize=14)
    ax.set_ylabel("レイテンシ [ms]", fontsize=10)
    ax.set_ylim(bottom=0)
    styled_legend(ax)

    ax = axes[1]
    if log.has("tlm_loop_dt_us"):
        ax.plot(t, df["tlm_loop_dt_us"], color=COLORS["loop_dt"], linewidth=0.8,
                alpha=0.9, label="機体 loop_dt")
        ax.axhline(y=LOOP_DT_NOMINAL_US, color="gray", linestyle="--", alpha=0.7,
                   label=f"公称 {LOOP_DT_NOMINAL_US:.0f}µs (400Hz)")
    ax.set_ylabel("loop_dt [µs]", fontsize=10)
    ax.set_xlabel("時間 [s]", fontsize=11)
    styled_legend(ax)
    return save_fig(fig, out_dir, "08_latency_loop_dt.png")


def _fig_mocap_diag(log: FlightLog, out_dir: Path) -> Path | None:
    """09: MoCap 診断(マーカー数・フレーム間隔・鮮度)。Position モードのみ。"""
    if not (log.has("marker_count") or log.has("mocap_age_ms")):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(2, 1, figsize=(12.0, 8.0), sharex=True)

    ax = axes[0]
    if log.has("marker_count"):
        ax.fill_between(t, 0, df["marker_count"], color=COLORS["marker"], alpha=0.4)
        ax.plot(t, df["marker_count"], color=COLORS["marker"], linewidth=0.8,
                label="有効マーカー数")
    ax.set_title("MoCap 診断", fontsize=14)
    ax.set_ylabel("マーカー数", fontsize=10)
    ax.set_ylim(bottom=0)
    styled_legend(ax)

    ax = axes[1]
    if log.has("frame_dt_ms"):
        ax.plot(t, df["frame_dt_ms"], color="#64748b", linewidth=0.7, alpha=0.8,
                label="フレーム間隔")
    if log.has("mocap_age_ms"):
        ax.plot(t, df["mocap_age_ms"], color="#db2777", linewidth=0.7, alpha=0.8,
                label="pose 鮮度")
    ax.set_ylabel("[ms]", fontsize=10)
    ax.set_xlabel("時間 [s]", fontsize=11)
    ax.set_ylim(bottom=0)
    styled_legend(ax)
    return save_fig(fig, out_dir, "09_mocap_diagnostics.png")


def _fig_xyz_3d(log: FlightLog, out_dir: Path) -> Path | None:
    """15: 3D 飛行軌跡(plasma 時間カラー散布)。Position/Multi のみ。

    旧 Drone_Log_Viewer(For_Presentation/generate_static_image.py の
    _save_3d_position_png)の見た目を踏襲: plasma カラー散布+カラーバー、
    view_init(25, -60)、始点緑/終点赤、等スケール。
    """
    if not (log.has("pos_x") and log.has("pos_y") and log.has("pos_z")):
        return None
    df = log.df
    mask = (df["pos_x"].notna() & df["pos_y"].notna()
            & df["pos_z"].notna()).to_numpy()
    if mask.sum() < 2:
        return None
    t = log.t[mask]
    x = df["pos_x"].to_numpy(dtype=float)[mask]
    y = df["pos_y"].to_numpy(dtype=float)[mask]
    z = df["pos_z"].to_numpy(dtype=float)[mask]

    fig = plt.figure(figsize=(10.0, 8.0), dpi=FIG_DPI)
    fig.patch.set_facecolor(FIG_BG)
    ax = fig.add_subplot(projection="3d")
    ax.set_facecolor(FIG_BG)
    pane_rgba = mcolors.to_rgba(AX_BG)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color(pane_rgba)
    ax.tick_params(colors=TEXT_COLOR, labelsize=9)
    ax.grid(True, alpha=GRID_ALPHA, color=GRID_COLOR)

    cmap = plt.get_cmap("plasma")
    tmax = float(t.max())
    tmin = float(t.min())
    norm = plt.Normalize(tmin, tmax if tmax > tmin else tmin + 1.0)
    ax.plot(x, y, z, color=COLORS["trajectory"], linewidth=1.2, alpha=0.7)
    ax.scatter(x, y, z, c=t, cmap=cmap, norm=norm, s=8, alpha=0.9,
               depthshade=False)
    ax.scatter(x[:1], y[:1], z[:1], color=COLORS["start"], s=80,
               edgecolors="#111111", linewidth=1.2, label="開始位置")
    ax.scatter(x[-1:], y[-1:], z[-1:], color=COLORS["end"], s=80,
               edgecolors="#111111", linewidth=1.2, label="終了位置")

    # 等スケール(最大レンジの立方体に合わせる)
    half = max(float(np.ptp(x)), float(np.ptp(y)), float(np.ptp(z)), 0.2) / 2.0
    ax.set_xlim((x.max() + x.min()) / 2 - half, (x.max() + x.min()) / 2 + half)
    ax.set_ylim((y.max() + y.min()) / 2 - half, (y.max() + y.min()) / 2 + half)
    ax.set_zlim((z.max() + z.min()) / 2 - half, (z.max() + z.min()) / 2 + half)
    ax.view_init(elev=25, azim=-60)

    ax.set_title("3D 飛行軌跡(時間カラー)", color=TEXT_COLOR, fontsize=14, pad=12)
    ax.set_xlabel("X [m]", color=TEXT_COLOR, fontsize=10)
    ax.set_ylabel("Y [m]", color=TEXT_COLOR, fontsize=10)
    ax.set_zlabel("Z [m]", color=TEXT_COLOR, fontsize=10)
    _style_time_colorbar(fig, ax, norm, cmap, pad=0.12, shrink=0.7)
    styled_legend(ax)
    return save_fig(fig, out_dir, "15_xyz_3d.png")


def _fig_xy_time(log: FlightLog, out_dir: Path) -> Path | None:
    """16: XY 軌跡の plasma 時間カラー散布版(旧 v3_2d 相当)。Position のみ。"""
    if not (log.has("pos_x") and log.has("pos_y")):
        return None
    df = log.df
    mask = (df["pos_x"].notna() & df["pos_y"].notna()).to_numpy()
    if mask.sum() < 2:
        return None
    t = log.t[mask]
    x = df["pos_x"].to_numpy(dtype=float)[mask]
    y = df["pos_y"].to_numpy(dtype=float)[mask]

    fig, ax = new_fig(figsize=(9.0, 8.0))
    cmap = plt.get_cmap("plasma")
    tmax = float(t.max())
    tmin = float(t.min())
    norm = plt.Normalize(tmin, tmax if tmax > tmin else tmin + 1.0)
    ax.plot(x, y, color=COLORS["trajectory"], linewidth=1.0, alpha=0.6)
    ax.scatter(x, y, c=t, cmap=cmap, norm=norm, s=10, alpha=0.9)
    if log.has("target_x") and log.has("target_y"):
        ax.plot(df["target_x"], df["target_y"], color=COLORS["target"],
                linewidth=1.0, linestyle="--", alpha=0.7, label="目標軌道")
    ax.scatter(x[:1], y[:1], color=COLORS["start"], s=110, zorder=5,
               edgecolors="#111111", linewidth=1.5, label="開始位置")
    ax.scatter(x[-1:], y[-1:], color=COLORS["end"], s=110, zorder=5,
               edgecolors="#111111", linewidth=1.5, label="終了位置")

    ax.set_title("XY 軌跡(時間カラー)", fontsize=14)
    ax.set_xlabel("X [m]", fontsize=11)
    ax.set_ylabel("Y [m]", fontsize=11)
    ax.set_aspect("equal")
    _style_time_colorbar(fig, ax, norm, cmap, pad=0.02)
    styled_legend(ax)
    return save_fig(fig, out_dir, "16_xy_time.png")


def _fig_cmd_echo(log: FlightLog, out_dir: Path) -> Path | None:
    """17: 送信指令 vs 機体適用エコー(旧 06_feedback_vs_command 相当)。全モード。

    PC が送った roll_ref_deg/pitch_ref_deg と、機体が実際に適用した
    tlm_roll_ref_rad/tlm_pitch_ref_rad(deg 変換済み派生列)を重畳し、
    指令伝達・遅延を確認する。
    """
    pairs = (
        ("Roll", "roll_ref_deg", "tlm_roll_ref_deg", COLORS["meas_roll"]),
        ("Pitch", "pitch_ref_deg", "tlm_pitch_ref_deg", COLORS["meas_pitch"]),
    )
    if not any(log.has(col)
               for _, ref_col, echo_col, _c in pairs
               for col in (ref_col, echo_col)):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(2, 1, figsize=(12.0, 8.0), sharex=True)

    for ax, (axis_label, ref_col, echo_col, echo_color) in zip(axes, pairs):
        if log.has(ref_col):
            ax.plot(t, df[ref_col], color="#666666", linewidth=1.0, alpha=0.85,
                    label=f"{axis_label} 送信指令(PC)")
        if log.has(echo_col):
            ax.plot(t, df[echo_col], color=echo_color, linewidth=1.0, alpha=0.9,
                    label=f"{axis_label} 機体適用エコー")
        ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
        ax.set_ylabel(f"{axis_label} [deg]", fontsize=10)
        styled_legend(ax)

    axes[0].set_title("指令エコー: 送信指令 vs 機体適用", fontsize=14)
    axes[-1].set_xlabel("時間 [s]", fontsize=11)
    return save_fig(fig, out_dir, "17_cmd_echo.png")


def generate_static_figures(log: FlightLog, out_dir: str | Path) -> list[Path]:
    """静止画グラフ一式を out_dir に生成し、生成できたパスの一覧を返す。"""
    out_dir = Path(out_dir)
    print("\n静止画グラフを生成します...")
    generators = (
        _fig_xy_trajectory,
        _fig_attitude,
        _fig_altitude,
        _fig_position_tracking,
        _fig_pid,
        _fig_duty,
        _fig_power,
        _fig_latency,
        _fig_mocap_diag,
        _fig_xyz_3d,
        _fig_xy_time,
        _fig_cmd_echo,
    )
    saved: list[Path] = []
    for gen in generators:
        path = gen(log, out_dir)
        if path is not None:
            saved.append(path)
    print(f"静止画グラフ生成完了: {len(saved)} 枚")
    return saved
