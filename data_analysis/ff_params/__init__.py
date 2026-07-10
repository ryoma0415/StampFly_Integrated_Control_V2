"""電流FF較正パラメーター抽出ライブラリ (stampfly_ff_profile v1).

旧 analyze_feasibility_20260612.py (削除済み) の集計・フィット関数を忠実移植し、
8スイープラン (全機×4姿勢 + 単機×4) から FFプロファイルdict を生成する。
仕様: docs/ff_pipeline_design.md §2-3。
"""
from .core import (
    MOTORS,
    CANON_ORIENTATIONS,
    load_run,
    aggregate,
    fit_affine,
    fit_prop,
    classify_runs,
    extract_profile,
    mag3d_hash,
    normalize_orientation,
)

__all__ = [
    "MOTORS",
    "CANON_ORIENTATIONS",
    "load_run",
    "aggregate",
    "fit_affine",
    "fit_prop",
    "classify_runs",
    "extract_profile",
    "mag3d_hash",
    "normalize_orientation",
]
