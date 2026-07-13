"""白背景ライトテーマの描画スタイル共通ヘルパー。"""

from __future__ import annotations

from pathlib import Path

from . import jp_font

jp_font.setup_japanese_font()

import matplotlib.pyplot as plt  # noqa: E402

from .constants import (  # noqa: E402
    AX_BG,
    FIG_BG,
    FIG_DPI,
    GRID_ALPHA,
    GRID_COLOR,
    TEXT_COLOR,
)


def style_ax(ax) -> None:
    """Axes を白背景ライトテーマに整える。"""
    ax.set_facecolor(AX_BG)
    ax.tick_params(colors=TEXT_COLOR, labelsize=9)
    ax.title.set_color(TEXT_COLOR)
    ax.xaxis.label.set_color(TEXT_COLOR)
    ax.yaxis.label.set_color(TEXT_COLOR)
    ax.grid(True, alpha=GRID_ALPHA, color=GRID_COLOR)
    for spine in ax.spines.values():
        spine.set_edgecolor(TEXT_COLOR)


def new_fig(nrows: int = 1, ncols: int = 1, figsize: tuple[float, float] = (12.0, 5.0),
            sharex: bool = False, height_ratios: list[float] | None = None):
    """白背景の Figure と Axes(常に 2 次元でなく flatten 済み配列)を返す。"""
    gridspec_kw = {"height_ratios": height_ratios} if height_ratios else None
    fig, axes = plt.subplots(
        nrows, ncols, figsize=figsize, dpi=FIG_DPI, sharex=sharex,
        gridspec_kw=gridspec_kw,
    )
    fig.patch.set_facecolor(FIG_BG)
    if nrows * ncols == 1:
        style_ax(axes)
        return fig, axes
    axes_flat = axes.ravel() if hasattr(axes, "ravel") else axes
    for ax in axes_flat:
        style_ax(ax)
    return fig, axes


def styled_legend(ax, **kwargs) -> None:
    """白背景ライトテーマ向けの凡例(白地に濃文字)。"""
    kwargs.setdefault("loc", "upper right")
    kwargs.setdefault("fontsize", 8)
    kwargs.setdefault("framealpha", 0.8)
    leg = ax.legend(**kwargs)
    if leg is not None:
        leg.get_frame().set_facecolor(AX_BG)
        leg.get_frame().set_edgecolor("#cccccc")
        for text in leg.get_texts():
            text.set_color(TEXT_COLOR)


def save_fig(fig, out_dir: Path, filename: str) -> Path:
    """Figure を PNG 保存してパスを返す(close まで行う)。"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / filename
    fig.tight_layout()
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  保存: {path}")
    return path
