"""複数機同時制御(multi)ログの共有図と機体別図一式の生成。

- M01_multi_xy.png: 全機の XY 軌跡+目標を機体別色(MULTI_DRONE_COLORS)で
  1 枚に重畳した共有プロット。
- make_multi_figures: 機体ごとのサブフォルダに通常の静止画一式
  (01-09, 15-17)+ヨー解析図(10-14)を出力する。

図の自動スキップ規約(必要列のデータ有無)は単機と同じ。
"""

from __future__ import annotations

from pathlib import Path

from . import jp_font

jp_font.setup_japanese_font()

import matplotlib.pyplot as plt  # noqa: E402

from .constants import MULTI_DRONE_COLORS  # noqa: E402
from .loader import FlightLog  # noqa: E402
from .plots import generate_static_figures  # noqa: E402
from .style import styled_legend, new_fig, save_fig  # noqa: E402
from .yaw_analysis import compute_yaw_stats, generate_yaw_figures  # noqa: E402

MULTI_XY_FILENAME = "M01_multi_xy.png"


def drone_label(log: FlightLog) -> str:
    """機体別サブフォルダ名・表示名(機体名があれば機体名、無ければ stem)。"""
    return log.drone_name or log.name


def drone_color(index: int) -> str:
    """機体 index(0 始まり)に対応する機体別カラー。"""
    return MULTI_DRONE_COLORS[index % len(MULTI_DRONE_COLORS)]


def fig_multi_xy(logs: list[FlightLog], out_dir: str | Path) -> Path | None:
    """M01: 全機の XY 軌跡+目標を機体別色で共有プロットする。"""
    out_dir = Path(out_dir)
    fig, ax = new_fig(figsize=(9.0, 9.0))
    has_any = False

    for i, log in enumerate(logs):
        if not (log.has("pos_x") and log.has("pos_y")):
            continue
        df = log.df
        color = drone_color(i)
        name = drone_label(log)
        ax.plot(df["pos_x"], df["pos_y"], color=color, linewidth=1.1,
                alpha=0.9, label=f"{name} 軌跡")
        if log.has("target_x") and log.has("target_y"):
            ax.plot(df["target_x"], df["target_y"], color=color, linewidth=1.0,
                    linestyle="--", alpha=0.55, label=f"{name} 目標")
        valid = df["pos_x"].notna() & df["pos_y"].notna()
        if valid.any():
            first = df.index[valid][0]
            last = df.index[valid][-1]
            ax.scatter(df.at[first, "pos_x"], df.at[first, "pos_y"],
                       color=color, marker="o", s=90, zorder=5,
                       edgecolors="#111111", linewidth=1.2)
            ax.scatter(df.at[last, "pos_x"], df.at[last, "pos_y"],
                       color=color, marker="X", s=110, zorder=5,
                       edgecolors="#111111", linewidth=1.2)
        has_any = True

    if not has_any:
        plt.close(fig)
        return None

    ax.set_title("複数機 XY 軌跡(○=開始 / ×=終了)", fontsize=14)
    ax.set_xlabel("X [m]", fontsize=11)
    ax.set_ylabel("Y [m]", fontsize=11)
    ax.set_aspect("equal")
    styled_legend(ax, ncol=2)
    return save_fig(fig, out_dir, MULTI_XY_FILENAME)


def make_multi_figures(logs: list[FlightLog],
                       out_dir: str | Path) -> dict[str, list[Path]]:
    """機体別サブフォルダに通常図一式+ヨー解析図を生成する。

    Returns:
        {機体名: 生成された図パスのリスト(番号順)} の辞書。
    """
    out_dir = Path(out_dir)
    results: dict[str, list[Path]] = {}
    for log in logs:
        name = drone_label(log)
        sub_dir = out_dir / name
        print(f"\n--- 機体 {name} の図を生成します ---")
        stats = compute_yaw_stats(log)
        paths = generate_static_figures(log, sub_dir)
        paths += generate_yaw_figures(log, sub_dir, stats)
        results[name] = sorted(paths, key=lambda p: p.name)
    return results
