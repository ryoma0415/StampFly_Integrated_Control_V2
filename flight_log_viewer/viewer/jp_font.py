"""日本語フォントの自動選択(旧 Drone_Log_Viewer の OS 別選択を踏襲)。

matplotlib の backend は必ず Agg(ヘッドレス)に固定する。pyplot を import
する前に本モジュールを import すること。
"""

from __future__ import annotations

import platform

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402  (backend 固定後に import する)
from matplotlib import font_manager  # noqa: E402

# OS 別の優先候補(旧実装: macOS=Hiragino Sans / Windows=MS Gothic / 他=DejaVu)
_FONT_CANDIDATES: dict[str, tuple[str, ...]] = {
    "Darwin": ("Hiragino Sans", "Hiragino Maru Gothic Pro", "Apple SD Gothic Neo"),
    "Windows": ("MS Gothic", "Yu Gothic", "Meiryo"),
}
# どの OS でも最後に試す共通候補
_COMMON_CANDIDATES: tuple[str, ...] = (
    "Noto Sans CJK JP", "IPAexGothic", "Arial Unicode MS", "DejaVu Sans",
)

_configured_font: str | None = None


def setup_japanese_font() -> str:
    """インストール済みフォントから日本語フォントを選んで rcParams に設定する。

    複数回呼んでも安全(2回目以降は初回の選択を返すだけ)。
    """
    global _configured_font
    if _configured_font is not None:
        return _configured_font

    installed = {f.name for f in font_manager.fontManager.ttflist}
    candidates = _FONT_CANDIDATES.get(platform.system(), ()) + _COMMON_CANDIDATES
    chosen = "DejaVu Sans"
    for name in candidates:
        if name in installed:
            chosen = name
            break

    plt.rcParams["font.family"] = chosen
    plt.rcParams["axes.unicode_minus"] = False
    _configured_font = chosen
    return chosen
