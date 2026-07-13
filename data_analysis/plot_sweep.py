#!/usr/bin/env python3
"""StampFly スイープ結果のグラフ化（校正解析 + 加算性検証の統合スクリプト）.

旧 analyze_calibration.py（スイープ1本 → 9〜12枚の図）と
旧 analyze_additivity.py（加算性シーケンス → 比較図 + 判定）を1本に統合したもの。

[1] スイープ1本のグラフ化（sweep_<stamp>_samples.csv）
    入力は pc_server が出力する samples-only CSV（bracket-baseline 方式）。
    測定行(phase=measure)にはドリフト除去後の電流ノイズ dB_cor_{x,y,z} が入る。
    新フォーマット（step_idx / leg 列つき、往復スイープ）では
    図⑩ ヒステリシス・図⑪ サンプル単位回帰・図⑫ 残差 vs IMU温度 を追加し
    計12枚。旧フォーマットでは9枚。同じ stamp の meta JSON があれば
    測定条件サマリを表示し、各図の下部にも条件を記載する。

[2] 加算性シーケンスの検証グラフ（sequence_<runid>_meta.json）
    モーター別スイープ（FL/FR/RL/RR 単機）の ΔB_m の和が全機同時スイープの
    ΔB_all と一致するか（線形重ね合わせ）を検証する。対角ペア run があれば
    該当2機の和とも比較。比較図（軸別 上段: 測定/予測, 下段: 残差±3σ）に加え、
    判定サマリを PNG（00_verdict_summary.png）としても出力する。
    シーケンスが単機4本のみ（多機 run なし）の場合は、csv_dir 内の
    別取得の全機/対角ペアスイープ（sweep_*_meta.json, aborted=false）を
    候補一覧から対話選択して比較する（Enter=同姿勢で時刻最近傍の1本）。
    --target で対象を明示すれば対話質問なしで実行できる（バッチ向け）。

使い方:
    python plot_sweep.py                        # 対話: メニュー → ファイル番号選択
    python plot_sweep.py <path> [-o 出力先] [--results-dir dir] [--target stem]...
        <path> は sweep_*_samples.csv（→スイープ解析）または
        sequence_*_meta.json（→加算性検証）。拡張子/名前で自動判別する。
    --results-dir はファイル一覧の探索先 / シーケンスの samples CSV の
        ディレクトリ（既定: ../pc_server/data/sweep_results/）。
    --target は加算性検証の比較対象（多機スイープの stem または
        samples.csv パス。複数指定可）。

出力先の既定:
    data_analysis/graphs/sweep_<stamp>/          （スイープ解析）
    data_analysis/graphs/additivity_<stem>/      （加算性検証）

依存: numpy, matplotlib のみ。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d projection)

# ライトテーマ（白背景）。matplotlib 既定の白背景をベースに、文字・軸を濃色に統一する。
plt.rcParams.update({
    "figure.facecolor": "#ffffff",
    "savefig.facecolor": "#ffffff",
    "axes.facecolor": "#ffffff",
    "axes.edgecolor": "#444444",
    "axes.labelcolor": "#222222",
    "axes.titlecolor": "#222222",
    "text.color": "#222222",
    "xtick.color": "#444444",
    "ytick.color": "#444444",
    "grid.color": "#999999",
    "legend.facecolor": "#ffffff",
    "legend.edgecolor": "#cccccc",
    "legend.labelcolor": "#222222",
})

# 日本語フォント（macOS優先）。見つからなければ既定フォントのまま（英字は問題なし）。
_INSTALLED = {f.name for f in fm.fontManager.ttflist}
for _jp in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic", "Yu Gothic",
            "Noto Sans CJK JP", "IPAexGothic", "Apple SD Gothic Neo", "Arial Unicode MS"):
    if _jp in _INSTALLED:
        plt.rcParams["font.family"] = _jp
        break
plt.rcParams["axes.unicode_minus"] = False

AXES = ("x", "y", "z")
AXIS_LABEL = {"x": "ΔB_x", "y": "ΔB_y", "z": "ΔB_z"}
FLIGHT_DUTY_LO = 0.5
FLIGHT_DUTY_HI = 0.8

SINGLES = ("FL", "FR", "RL", "RR")          # 単機 run のモーター名
NOISE_SIGMA = 4.0                            # 判定: 最大|残差| ≤ max(4σ, 下限)
NOISE_FLOOR_UT = 0.5                         # 判定しきい値の下限 [µT]（分解能スケール）

# 加算性の比較対象になれる多機構成（全機 / 対角ペア）
ALLOWED_MULTI = (frozenset(SINGLES), frozenset({"FL", "RR"}), frozenset({"FR", "RL"}))

SWEEP_GLOB = "sweep_*_samples.csv"
SEQ_GLOB = "sequence_*_meta.json"

# 各図の下部に入れる測定条件サマリ（run_sweep / run_additivity が設定）
_COND_FOOTER: str | None = None


# ------------------------------------------------------------ common ----------
def _f(value: str | None) -> float:
    """空欄/欠損は NaN。"""
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


STR_COLS = ("phase", "motors", "leg")


def load_samples(path: Path) -> dict:
    """samples CSV を 列名→配列 の dict に読む（phase/motors/leg は文字列のまま）。"""
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    if not rows:
        sys.exit(f"空のCSVです: {path}")
    out = {}
    for key in rows[0].keys():
        if key in STR_COLS:
            out[key] = np.array([r.get(key) or "" for r in rows], dtype=object)
        else:
            out[key] = np.array([_f(r.get(key, "")) for r in rows], dtype=float)
    return out


def col(data: dict, name: str) -> np.ndarray:
    return data.get(name, np.full(len(data["phase"]), np.nan))


def default_results_dir() -> Path:
    """pc_server が結果を書き出す sweep_results フォルダ（既定）。"""
    return Path(__file__).resolve().parent.parent / "pc_server" / "data" / "sweep_results"


def graphs_dir() -> Path:
    """グラフ出力のルート（data_analysis/graphs/）。"""
    return Path(__file__).resolve().parent / "graphs"


def _fmt_size(n: int) -> str:
    return f"{n / 1024:.0f} KB" if n < 1024 * 1024 else f"{n / 1024 / 1024:.1f} MB"


def _select_from_list(cands: list[Path], what: str, base_dir: Path) -> Path | None:
    """候補ファイルを一覧表示し、番号で1つ選ばせる（会話的選択）。

    Enter は最新（[1]）を選択、q で中止して None を返す。
    パイプ等で対話入力が無い（EOF）場合は最新を自動選択する。
    """
    print(f"\n{base_dir} の {what} から、対象を選んでください:\n")
    for i, p in enumerate(cands, 1):
        st = p.stat()
        mtime = time.strftime("%Y-%m-%d %H:%M", time.localtime(st.st_mtime))
        mark = "  ← 最新" if i == 1 else ""
        print(f"  [{i}] {p.name}   {mtime}   {_fmt_size(st.st_size)}{mark}")
    print()
    while True:
        try:
            raw = input(f"番号を入力 [1-{len(cands)}]（Enter=最新 / q=中止）: ").strip()
        except EOFError:
            print("（対話入力なし → 最新を自動選択）")
            return cands[0]
        if raw == "":
            return cands[0]
        if raw.lower() in ("q", "quit", "exit"):
            return None
        if raw.isdigit() and 1 <= int(raw) <= len(cands):
            return cands[int(raw) - 1]
        print(f"  '{raw}' は無効です。1〜{len(cands)} の番号を入力してください。")


def _list_by_mtime(d: Path, pattern: str) -> list[Path]:
    """d 内の pattern に一致するファイルを新しい順（mtime 降順）に返す。"""
    if not d.is_dir():
        return []
    return sorted(d.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)


def _strip_suffix(stem: str, suffix: str) -> str:
    return stem[: -len(suffix)] if stem.endswith(suffix) else stem


def save_fig(fig, out: Path, name: str) -> None:
    """全図共通の保存処理（条件フッター → レイアウト調整 → PNG 出力 → クローズ）。"""
    fig.tight_layout()
    if _COND_FOOTER:
        fig.text(0.5, -0.005, _COND_FOOTER, ha="center", va="top",
                 fontsize=8, color="#555555")
    fig.savefig(out / name, dpi=130, bbox_inches="tight")
    plt.close(fig)


# ==============================================================================
# [1] スイープ1本のグラフ化（旧 analyze_calibration.py）
# ==============================================================================

# ----------------------------------------------------------- aggregation ------
def measure_mask(data: dict) -> np.ndarray:
    return data["phase"] == "measure"


def baseline_rows_mask(data: dict) -> np.ndarray:
    # 初期base(duty=0) と 各duty後の baseline 行
    return (data["phase"] == "baseline") | (data["phase"] == "base")


def measure_arrays(data: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """measure 行の (電流, duty, ΔB[N,3]) を取り出す共通処理。"""
    m = measure_mask(data)
    cur = col(data, "current_a")[m]
    duty = col(data, "duty_cmd")[m]
    dB = np.column_stack([col(data, f"dB_cor_{a}")[m] for a in AXES])
    return cur, duty, dB


def per_duty_measure(data: dict) -> dict:
    """duty -> {I, dB[3] 平均, dB_std[3], vbat}（measure行の duty 別集計）.

    std はブラケット減算後の dB_cor で取る。生の b_cor だと、往復スイープで
    同じ duty を up/down の2回踏んだ際に2訪問間の基準ドリフト分が std に
    混入して過大評価になる（dB_cor はドリフト除去済みなので訪問をまたいでも
    純粋な短時間ばらつきを表す）。
    """
    m = measure_mask(data)
    duty = col(data, "duty_cmd")[m]
    cur = col(data, "current_a")[m]
    vbat = col(data, "vbat_v")[m]
    dB = np.column_stack([col(data, f"dB_cor_{a}")[m] for a in AXES])
    res = {}
    for d in sorted(set(np.round(duty, 3))):
        sel = np.isclose(duty, d)
        res[float(d)] = {
            "I": float(np.nanmean(cur[sel])),
            "dB": np.nanmean(dB[sel], axis=0),
            "dB_std": np.nanstd(dB[sel], axis=0),
            "vbat": float(np.nanmean(vbat[sel])),
        }
    return res


def duty0_point(data: dict) -> tuple[float, np.ndarray]:
    """duty=0 の (アイドル電流, ΔB=0)。直線フィットの低電流アンカー。"""
    b = data["phase"] == "base"
    cur = col(data, "current_a")[b]
    idle = float(np.nanmean(cur)) if cur.size else math.nan
    return idle, np.zeros(3)


def per_duty_baseline(data: dict) -> dict:
    """duty -> 平均 motor-off 基準ベクトル b_cor（baseline/base 行）。

    注意: 往復スイープでは同じ duty の基準窓が up/down で2回あり、duty キーの
    平均では2訪問が混ざって軌跡が潰れる。往復データの図⑦⑧には
    baseline_sequence()（時系列順・1窓=1点）を使うこと。
    """
    bm = baseline_rows_mask(data)
    duty = col(data, "duty_cmd")[bm]
    Bcor = np.column_stack([col(data, f"b{a}_cor")[bm] for a in AXES])
    res = {}
    for d in sorted(set(np.round(duty, 3))):
        sel = np.isclose(duty, d)
        res[float(d)] = np.nanmean(Bcor[sel], axis=0)
    return res


def baseline_sequence(data: dict) -> list[dict]:
    """時系列順の motor-off 基準点リスト [{order, duty, leg, t, vec[3]}]。

    step_idx で1基準窓=1点に集約する（初期 base は order=-1）。step_idx 列の
    無い旧フォーマットでは空に近いリストになるので、呼び出し側は
    per_duty_baseline にフォールバックする。
    """
    bm = baseline_rows_mask(data)
    if not bm.any() or "step_idx" not in data:
        return []
    t = col(data, "t_s")[bm]
    duty = col(data, "duty_cmd")[bm]
    Bcor = np.column_stack([col(data, f"b{a}_cor")[bm] for a in AXES])
    phase = data["phase"][bm]
    leg = data["leg"][bm] if has_leg_column(data) else np.array([""] * int(bm.sum()), dtype=object)
    step = col(data, "step_idx")[bm]
    keys = np.where(phase == "base", -1.0, step)
    seq = []
    for k in sorted(set(keys[np.isfinite(keys)])):
        sel = keys == k
        seq.append({
            "order": int(k),
            "duty": float(np.nanmean(duty[sel])),
            "leg": next((l for l in leg[sel] if l in ("up", "down")), ""),
            "t": float(np.nanmean(t[sel])),
            "vec": np.nanmean(Bcor[sel], axis=0),
        })
    seq.sort(key=lambda e: e["t"])
    return seq


def _baseline_seq_labels(seq: list[dict]) -> list[str]:
    """基準点の短い表示ラベル（base / duty値＋up↑down↓）。"""
    return ["base" if e["order"] < 0 else f"{e['duty']:.1f}" + ("↓" if e["leg"] == "down" else "↑")
            for e in seq]


def heading_deg(bx: np.ndarray, by: np.ndarray) -> np.ndarray:
    return np.degrees(np.arctan2(by, bx))


def wrap_deg(d: np.ndarray) -> np.ndarray:
    return (d + 180.0) % 360.0 - 180.0


def fit_line(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    """y = a*x + b の最小二乗。返り値 (a, b, R^2)。"""
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if x.size < 2:
        return math.nan, math.nan, math.nan
    a, b = np.polyfit(x, y, 1)
    yhat = a * x + b
    ss_res = float(np.sum((y - yhat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return float(a), float(b), float(r2)


def fit_line_se(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float, float, np.ndarray]:
    """y = a*x + b の最小二乗（傾きの標準誤差つき）。

    返り値 (a, b, R^2, SE_a, 残差)。残差は入力と同じ長さで無効点は NaN。
    """
    ok = np.isfinite(x) & np.isfinite(y)
    resid = np.full(x.shape, np.nan)
    n = int(ok.sum())
    if n < 3:
        return math.nan, math.nan, math.nan, math.nan, resid
    a, b = np.polyfit(x[ok], y[ok], 1)
    r = y[ok] - (a * x[ok] + b)
    resid[ok] = r
    ss_res = float(np.sum(r ** 2))
    ss_tot = float(np.sum((y[ok] - np.mean(y[ok])) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    sxx = float(np.sum((x[ok] - np.mean(x[ok])) ** 2))
    se = math.sqrt(ss_res / (n - 2) / sxx) if sxx > 0 else math.nan
    return float(a), float(b), float(r2), float(se), resid


# ------------------------------------------------------------ leg / meta ------
def has_leg_column(data: dict) -> bool:
    """新フォーマット（leg 列あり）か。"""
    return "leg" in data


def measure_legs(data: dict) -> set:
    """measure 行に現れる leg の集合（{"up","down"} なら往復スイープ）。"""
    if not has_leg_column(data):
        return set()
    m = measure_mask(data)
    return {l for l in data["leg"][m] if l in ("up", "down")}


def per_duty_leg_measure(data: dict) -> dict:
    """leg("up"/"down") -> duty -> {I, dB[3] 平均}（measure行の leg×duty 集計）."""
    m = measure_mask(data)
    leg = data["leg"][m]
    duty = col(data, "duty_cmd")[m]
    cur = col(data, "current_a")[m]
    dB = np.column_stack([col(data, f"dB_cor_{a}")[m] for a in AXES])
    res = {"up": {}, "down": {}}
    for lg in ("up", "down"):
        s0 = leg == lg
        for d in sorted(set(np.round(duty[s0], 3))):
            sel = s0 & np.isclose(duty, d)
            res[lg][float(d)] = {
                "I": float(np.nanmean(cur[sel])),
                "dB": np.nanmean(dB[sel], axis=0),
            }
    return res


def meta_path_for(samples: Path) -> Path:
    """samples CSV パスから同じ stamp の meta JSON パスを導出する。

    `..._samples.csv` → `..._meta.json`。それ以外の名前（合成CSV等）は
    `<stem>_meta.json` を隣に探す。
    """
    stem = _strip_suffix(samples.stem, "_samples")
    return samples.with_name(stem + "_meta.json")


def load_meta(samples: Path) -> dict | None:
    """meta JSON を読み込む。無い/壊れている場合は None（旧フォーマット互換）。"""
    mp = meta_path_for(samples)
    if not mp.is_file():
        return None
    try:
        return json.loads(mp.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        print(f"meta JSON の読み込みに失敗（無視して続行）: {mp} ({e})")
        return None


def _num(x) -> str:
    return f"{x:.3f}" if isinstance(x, (int, float)) else "-"


def meta_condition_line(meta: dict) -> str:
    """図の下部に入れる1行の測定条件サマリ（motors / pattern / notes）。"""
    notes = meta.get("notes") or {}
    parts = [f"motors={meta.get('motors', '-')}", f"pattern={meta.get('pattern', '-')}"]
    for key in ("location", "orientation", "memo"):
        v = notes.get(key)
        if v:
            parts.append(f"{key}={v}")
    return "測定条件: " + " / ".join(parts)


def print_meta_summary(meta: dict) -> None:
    """meta JSON の測定条件サマリを冒頭に表示する。"""
    notes = meta.get("notes") or {}
    bat = meta.get("battery") or {}
    temp = meta.get("imu_temp_c") or {}
    flags = meta.get("baseline_flags") or []
    print("\n=== 測定条件サマリ（meta JSON）===")
    print(f"  pattern: {meta.get('pattern', '-')} / motors: {meta.get('motors', '-')} / "
          f"method: {meta.get('method', '-')}")
    print(f"  notes: location={notes.get('location', '-')} / "
          f"orientation={notes.get('orientation', '-')} / memo={notes.get('memo', '-')}")
    print(f"  battery: {_num(bat.get('vbat_start_v'))} → {_num(bat.get('vbat_end_v'))} V "
          f"(min {_num(bat.get('vbat_min_v'))} V) / アイドル電流 {_num(meta.get('idle_current_a'))} A")
    print(f"  IMU温度: {_num(temp.get('start'))} → {_num(temp.get('end'))} °C "
          f"(min {_num(temp.get('min'))} / max {_num(temp.get('max'))})")
    warn = meta.get("baseline_jump_warn_uT")
    suffix = f"（隣接基準ジャンプ > {warn} µT のステップ）" if warn is not None else ""
    print(f"  baseline_flags: {len(flags)} 件{suffix}")
    for fl in flags:
        print(f"    - step {fl.get('step_idx')} duty={fl.get('duty')} leg={fl.get('leg')} "
              f"jump={fl.get('jump_uT')} µT")


# -------------------------------------------------------------- figures -------
def fig1_vector_4d(data: dict, out: Path) -> None:
    cur, duty, dB = measure_arrays(data)
    fig = plt.figure(figsize=(13, 5.5))
    for j, (cvar, label) in enumerate([(cur, "電流 [A]"), (duty, "duty")]):
        ax = fig.add_subplot(1, 2, j + 1, projection="3d")
        sc = ax.scatter(dB[:, 0], dB[:, 1], dB[:, 2], c=cvar, cmap="viridis", s=10, alpha=0.7)
        ax.set_xlabel("ΔB_x"); ax.set_ylabel("ΔB_y"); ax.set_zlabel("ΔB_z")
        ax.set_title(f"ノイズ磁束ベクトル（色 = {label}）")
        fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.1, label=label)
    fig.suptitle("① 電流・duty 対 ノイズ磁束ベクトル ΔB（4次元: 3D + 色）", fontsize=13)
    save_fig(fig, out, "01_noise_vector_4d.png")


def _scatter_dB_vs_I(axarr, data, pdm, idle, dB0, band=None):
    """fig2/3/4 共通: 各軸サブプロットに散布 + duty平均(+duty0)を描く。"""
    cur, duty, dB = measure_arrays(data)
    if band is not None:
        sel = (duty >= band[0] - 1e-6) & (duty <= band[1] + 1e-6)
        cur, duty, dB = cur[sel], duty[sel], dB[sel]
    duties = sorted(pdm)
    if band is not None:
        duties = [d for d in duties if band[0] - 1e-6 <= d <= band[1] + 1e-6]
    mean_I = np.array([pdm[d]["I"] for d in duties])
    for i, a in enumerate(AXES):
        ax = axarr[i]
        ax.scatter(cur, dB[:, i], s=6, alpha=0.3, color="gray", label="サンプル")
        mean_dB = np.array([pdm[d]["dB"][i] for d in duties])
        ax.scatter(mean_I, mean_dB, s=55, color="tab:red", zorder=5, label="duty平均")
        if band is None and np.isfinite(idle):
            ax.scatter([idle], [dB0[i]], s=70, marker="D", color="tab:cyan", zorder=6, label="duty=0")
        ax.set_xlabel("電流 I [A]"); ax.set_ylabel(f"{AXIS_LABEL[a]} [µT]")
        ax.grid(alpha=0.3)
    return duties, mean_I


def _fit_points(pdm, idle, dB0, axis_i, duties, include_duty0):
    xs = [pdm[d]["I"] for d in duties]
    ys = [pdm[d]["dB"][axis_i] for d in duties]
    if include_duty0 and np.isfinite(idle):
        xs.append(idle); ys.append(dB0[axis_i])
    return np.array(xs), np.array(ys)


def fig2_dB_vs_I(data, pdm, idle, dB0, out: Path) -> None:
    fig, axarr = plt.subplots(1, 3, figsize=(15, 4.6))
    _scatter_dB_vs_I(axarr, data, pdm, idle, dB0)
    axarr[0].legend(loc="best", fontsize=8)
    fig.suptitle("② 電流 対 ΔB_x / ΔB_y / ΔB_z（duty平均・duty=0 を含む）", fontsize=13)
    save_fig(fig, out, "02_dB_vs_current.png")


def _mark_flagged_steps(axarr, data, flags) -> None:
    """meta の baseline_flags（基準ジャンプ>閾値）のステップを各軸に赤×で強調する。

    新フォーマットなら step_idx で、無ければ duty(+leg) で該当 measure 行を特定し、
    その平均電流・平均 ΔB の位置にマーカーを置く（フラグ付き = 品質注意）。
    """
    if not flags:
        return
    m = measure_mask(data)
    cur = col(data, "current_a")[m]
    duty = col(data, "duty_cmd")[m]
    step = col(data, "step_idx")[m]
    dB = np.column_stack([col(data, f"dB_cor_{a}")[m] for a in AXES])
    leg = data["leg"][m] if has_leg_column(data) else None
    labeled = False
    for fl in flags:
        sel = None
        si = fl.get("step_idx")
        if si is not None and np.isfinite(step).any():
            sel = step == float(si)
        if sel is None or not sel.any():
            sel = np.isclose(duty, float(fl.get("duty", math.nan)))
            if leg is not None and fl.get("leg") in ("up", "down"):
                sel = sel & (leg == fl["leg"])
        if not sel.any():
            continue
        If = float(np.nanmean(cur[sel]))
        for i in range(len(AXES)):
            axarr[i].scatter([If], [float(np.nanmean(dB[sel, i]))],
                             marker="x", s=120, color="red", linewidths=2.5, zorder=7,
                             label="baseline flag（品質注意）" if (i == 0 and not labeled) else None)
        labeled = True


def fig3_dB_vs_I_fit(data, pdm, idle, dB0, out: Path, flags=None) -> None:
    fig, axarr = plt.subplots(1, 3, figsize=(15, 4.6))
    duties, _ = _scatter_dB_vs_I(axarr, data, pdm, idle, dB0)
    for i in range(len(AXES)):
        xs, ys = _fit_points(pdm, idle, dB0, i, duties, include_duty0=True)
        slope, intercept, r2 = fit_line(xs, ys)
        xline = np.linspace(min(xs), max(xs), 50)
        axarr[i].plot(xline, slope * xline + intercept, "g-", lw=2,
                      label=f"ΔB={slope:.3f}·I{intercept:+.2f}\nR²={r2:.3f}")
    _mark_flagged_steps(axarr, data, flags)
    for i in range(len(AXES)):
        axarr[i].legend(loc="best", fontsize=8)
    fig.suptitle("③ 全域フィット  ΔB = a·I + b（duty=0 含む）", fontsize=13)
    save_fig(fig, out, "03_dB_vs_current_fit.png")


def fig4_dB_vs_I_flightband(data, pdm, idle, dB0, out: Path) -> None:
    fig, axarr = plt.subplots(1, 3, figsize=(15, 4.6))
    duties, _ = _scatter_dB_vs_I(axarr, data, pdm, idle, dB0, band=(FLIGHT_DUTY_LO, FLIGHT_DUTY_HI))
    for i in range(len(AXES)):
        xs, ys = _fit_points(pdm, idle, dB0, i, duties, include_duty0=False)
        slope, intercept, r2 = fit_line(xs, ys)
        if xs.size >= 2:
            xline = np.linspace(min(xs), max(xs), 50)
            axarr[i].plot(xline, slope * xline + intercept, "m-", lw=2,
                          label=f"ΔB={slope:.3f}·I{intercept:+.2f}\nR²={r2:.3f}")
        axarr[i].legend(loc="best", fontsize=8)
    fig.suptitle(f"④ 飛行帯フィット duty {FLIGHT_DUTY_LO}–{FLIGHT_DUTY_HI}  ΔB = a·I + b", fontsize=13)
    save_fig(fig, out, "04_dB_vs_current_fit_flightband.png")


def fig5_dB_magnitude(data, pdm, out: Path) -> None:
    cur, duty, dB = measure_arrays(data)
    mag = np.linalg.norm(dB, axis=1)
    duties = sorted(pdm)
    mean_I = np.array([pdm[d]["I"] for d in duties])
    mean_mag = np.array([np.linalg.norm(pdm[d]["dB"]) for d in duties])
    fig, axarr = plt.subplots(1, 2, figsize=(13, 4.8))
    sc = axarr[0].scatter(cur, mag, c=duty, cmap="viridis", s=10, alpha=0.7)
    axarr[0].plot(mean_I, mean_mag, "r-o", lw=1.5, label="duty平均")
    axarr[0].set_xlabel("電流 I [A]"); axarr[0].set_ylabel("|ΔB| [µT]")
    axarr[0].set_title("|ΔB| 対 電流（色=duty）"); axarr[0].grid(alpha=0.3); axarr[0].legend(fontsize=8)
    fig.colorbar(sc, ax=axarr[0], label="duty")
    sc2 = axarr[1].scatter(duty, mag, c=cur, cmap="plasma", s=10, alpha=0.7)
    axarr[1].plot(duties, mean_mag, "r-o", lw=1.5, label="duty平均")
    axarr[1].set_xlabel("duty"); axarr[1].set_ylabel("|ΔB| [µT]")
    axarr[1].set_title("|ΔB| 対 duty（色=電流）"); axarr[1].grid(alpha=0.3); axarr[1].legend(fontsize=8)
    fig.colorbar(sc2, ax=axarr[1], label="電流 [A]")
    fig.suptitle("⑤ 電流・duty 対 ΔB の大きさ |ΔB|", fontsize=13)
    save_fig(fig, out, "05_dB_magnitude.png")


def fig6_B_std(pdm, out: Path) -> None:
    duties = sorted(pdm)
    mean_I = np.array([pdm[d]["I"] for d in duties])
    std = {a: np.array([pdm[d]["dB_std"][i] for d in duties]) for i, a in enumerate(AXES)}
    fig, axarr = plt.subplots(1, 2, figsize=(13, 4.8))
    for a in AXES:
        axarr[0].plot(duties, std[a], "-o", label=f"ΔB{a} std")
        axarr[1].plot(mean_I, std[a], "-o", label=f"ΔB{a} std")
    axarr[0].set_xlabel("duty"); axarr[0].set_ylabel("std [µT]")
    axarr[0].set_title("ブラケット減算後 ΔB 成分 std 対 duty"); axarr[0].grid(alpha=0.3); axarr[0].legend(fontsize=8)
    axarr[1].set_xlabel("電流 I [A]"); axarr[1].set_ylabel("std [µT]")
    axarr[1].set_title("ブラケット減算後 ΔB 成分 std 対 電流"); axarr[1].grid(alpha=0.3); axarr[1].legend(fontsize=8)
    fig.suptitle("⑥ 電流・duty 対 ΔBx std / ΔBy std / ΔBz std（ドリフト除去後の短時間ばらつき）", fontsize=13)
    save_fig(fig, out, "06_B_std.png")


def fig7_baseline_drift_3d(pdb: dict, out: Path, seq: list[dict] | None = None) -> None:
    if seq and len(seq) >= 3:
        # 時系列順（往復対応）: 1基準窓=1点。duty キー平均だと up/down の
        # 2訪問が混ざり軌跡が潰れる。
        base0 = seq[0]["vec"]
        diffs = np.array([e["vec"] - base0 for e in seq])
        order = np.arange(len(seq))
        color, clabel = order, "基準点の時系列順"
        title = "⑦ 基準磁場ベクトルの初期baseからの差分（色=時系列順）\n＝ドリフトの軌跡（往復対応）"
    else:
        duties = sorted(pdb)
        base0 = pdb.get(0.0)
        if base0 is None:
            base0 = pdb[duties[0]]
        diffs = np.array([pdb[d] - base0 for d in duties])
        color, clabel = duties, "duty"
        title = "⑦ duty 対 基準磁場ベクトルの duty=0 からの差分（色=duty）\n＝ドリフトの軌跡"
    fig = plt.figure(figsize=(7.5, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(diffs[:, 0], diffs[:, 1], diffs[:, 2], c=color, cmap="viridis", s=60)
    ax.plot(diffs[:, 0], diffs[:, 1], diffs[:, 2], color="gray", lw=0.8, alpha=0.6)
    ax.scatter([0], [0], [0], color="#111111", marker="x", s=80, label="初期 base")
    ax.set_xlabel("Δx [µT]"); ax.set_ylabel("Δy [µT]"); ax.set_zlabel("Δz [µT]")
    ax.set_title(title)
    fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.1, label=clabel); ax.legend(fontsize=8)
    save_fig(fig, out, "07_baseline_drift_3d.png")


def fig8_baseline_components(pdb: dict, out: Path, seq: list[dict] | None = None) -> None:
    if seq and len(seq) >= 3:
        # 時系列順（往復対応）: x 軸は基準窓の経過順、ラベルは duty＋↑/↓。
        x = np.arange(len(seq))
        labels = _baseline_seq_labels(seq)
        base0 = seq[0]["vec"]
        vals = {a: np.array([e["vec"][i] for e in seq]) for i, a in enumerate(AXES)}
        diffs = {a: vals[a] - base0[i] for i, a in enumerate(AXES)}
        xlabel = "基準点（時系列順, ↑=up ↓=down）"
        fig, axarr = plt.subplots(2, 3, figsize=(15, 8))
        for i, a in enumerate(AXES):
            for row, (series, c, ttl) in enumerate([
                    (vals[a], "tab:blue", f"基準磁場 B{a}"),
                    (diffs[a], "tab:orange", f"差分 B{a} − 初期base")]):
                axc = axarr[row, i]
                axc.plot(x, series, "-o", color=c)
                axc.set_title(ttl); axc.set_xlabel(xlabel)
                axc.set_ylabel(f"B{a} [µT]"); axc.grid(alpha=0.3)
                axc.set_xticks(x); axc.set_xticklabels(labels, rotation=60, fontsize=6)
            axarr[1, i].axhline(0, color="gray", lw=0.8)
        fig.suptitle("⑧ 基準磁場ベクトル各成分 と 初期baseからの差分（時系列順・往復対応）", fontsize=13)
        save_fig(fig, out, "08_baseline_components.png")
        return
    duties = sorted(pdb)
    base0 = pdb.get(0.0, pdb[duties[0]])
    vals = {a: np.array([pdb[d][i] for d in duties]) for i, a in enumerate(AXES)}
    diffs = {a: np.array([pdb[d][i] - base0[i] for d in duties]) for i, a in enumerate(AXES)}
    fig, axarr = plt.subplots(2, 3, figsize=(15, 8))
    for i, a in enumerate(AXES):
        axarr[0, i].plot(duties, vals[a], "-o", color="tab:blue")
        axarr[0, i].set_title(f"基準磁場 B{a}"); axarr[0, i].set_xlabel("duty")
        axarr[0, i].set_ylabel(f"B{a} [µT]"); axarr[0, i].grid(alpha=0.3)
        axarr[1, i].plot(duties, diffs[a], "-o", color="tab:orange")
        axarr[1, i].set_title(f"差分 ΔB{a}(duty) − B{a}(duty=0)"); axarr[1, i].set_xlabel("duty")
        axarr[1, i].set_ylabel(f"ΔB{a} [µT]"); axarr[1, i].grid(alpha=0.3)
        axarr[1, i].axhline(0, color="gray", lw=0.8)
    fig.suptitle("⑧ duty 対 基準磁場ベクトル各成分 と duty=0 からの差分（計6枚）", fontsize=13)
    save_fig(fig, out, "08_baseline_components.png")


def fig9_quality(data, pdm, out: Path) -> None:
    m = measure_mask(data)
    cur = col(data, "current_a")[m]
    duty = col(data, "duty_cmd")[m]
    bx_raw = col(data, "bx_raw")[m]
    by_raw = col(data, "by_raw")[m]
    # base heading (raw 水平) を基準にした Δheading
    b = data["phase"] == "base"
    base_bx = float(np.nanmean(col(data, "bx_raw")[b]))
    base_by = float(np.nanmean(col(data, "by_raw")[b]))
    base_head = heading_deg(np.array([base_bx]), np.array([base_by]))[0]
    dhead = wrap_deg(heading_deg(bx_raw, by_raw) - base_head)

    duties = sorted(pdm)
    roll_std = []; pitch_std = []; yaw_std = []
    for d in duties:
        sel = np.isclose(col(data, "duty_cmd")[m], d)
        roll_std.append(float(np.nanstd(col(data, "roll_deg")[m][sel])))
        pitch_std.append(float(np.nanstd(col(data, "pitch_deg")[m][sel])))
        yaw_std.append(float(np.nanstd(col(data, "yaw_deg")[m][sel])))
    mean_I = [pdm[d]["I"] for d in duties]
    mean_vbat = [pdm[d]["vbat"] for d in duties]

    fig, axarr = plt.subplots(2, 2, figsize=(13, 9))
    sc = axarr[0, 0].scatter(cur, dhead, c=duty, cmap="viridis", s=10, alpha=0.7)
    axarr[0, 0].set_xlabel("電流 I [A]"); axarr[0, 0].set_ylabel("Δheading [deg]")
    axarr[0, 0].set_title("電流 対 Δheading（色=duty）"); axarr[0, 0].grid(alpha=0.3)
    fig.colorbar(sc, ax=axarr[0, 0], label="duty")
    axarr[0, 1].plot(duties, roll_std, "-o", label="Roll std")
    axarr[0, 1].plot(duties, pitch_std, "-o", label="Pitch std")
    axarr[0, 1].plot(duties, yaw_std, "-o", label="Yaw std")
    axarr[0, 1].set_xlabel("duty"); axarr[0, 1].set_ylabel("姿勢 std [deg]")
    axarr[0, 1].set_title("duty 対 姿勢 std"); axarr[0, 1].grid(alpha=0.3); axarr[0, 1].legend(fontsize=8)
    axarr[1, 0].plot(duties, mean_vbat, "-o", color="tab:purple")
    axarr[1, 0].set_xlabel("duty"); axarr[1, 0].set_ylabel("バッテリ電圧 [V]")
    axarr[1, 0].set_title("duty 対 バッテリ電圧"); axarr[1, 0].grid(alpha=0.3)
    axarr[1, 1].plot(duties, mean_I, "-o", color="tab:green")
    axarr[1, 1].set_xlabel("duty"); axarr[1, 1].set_ylabel("電流 [A]")
    axarr[1, 1].set_title("duty 対 電流"); axarr[1, 1].grid(alpha=0.3)
    fig.suptitle("⑨ Δheading / 姿勢std / バッテリ電圧 / 電流", fontsize=13)
    save_fig(fig, out, "09_quality_checks.png")


def print_fits(pdm, idle, dB0) -> None:
    duties = sorted(pdm)
    band = [d for d in duties if FLIGHT_DUTY_LO - 1e-6 <= d <= FLIGHT_DUTY_HI + 1e-6]
    print("\n=== ΔB = a·I + b フィット結果 [µT/A, µT] ===")
    for i, a in enumerate(AXES):
        xs, ys = _fit_points(pdm, idle, dB0, i, duties, include_duty0=True)
        sa, sb, sr = fit_line(xs, ys)
        xf, yf = _fit_points(pdm, idle, dB0, i, band, include_duty0=False)
        fa, fb, fr = fit_line(xf, yf)
        print(f"  {AXIS_LABEL[a]}: 全域 a={sa:.3f} b={sb:.3f} R²={sr:.3f} | "
              f"飛行帯({FLIGHT_DUTY_LO}-{FLIGHT_DUTY_HI}) a={fa:.3f} b={fb:.3f} R²={fr:.3f}")


# ------------------------------------------- 新フォーマット拡張（図⑩〜⑫）----
def fig10_hysteresis(data, out: Path) -> None:
    """図⑩ up/down レグ分離のヒステリシス確認。

    上段: 各軸の duty平均 ΔB vs 電流 を up / down 別系列で表示。
    下段: 同一 duty での down − up 差（棒グラフ）。この差が
    ヒステリシス＋残存系統誤差の直接推定量になる。
    """
    pdl = per_duty_leg_measure(data)
    common = sorted(set(pdl["up"]) & set(pdl["down"]))
    fig, axarr = plt.subplots(2, 3, figsize=(15, 8.5))
    diffs = {}
    for i, a in enumerate(AXES):
        ax = axarr[0, i]
        for lg, mk, c in (("up", "-o", "tab:blue"), ("down", "-s", "tab:red")):
            ds = sorted(pdl[lg])
            I = [pdl[lg][d]["I"] for d in ds]
            y = [pdl[lg][d]["dB"][i] for d in ds]
            ax.plot(I, y, mk, color=c, ms=5, label=lg)
        ax.set_xlabel("電流 I [A]"); ax.set_ylabel(f"{AXIS_LABEL[a]} [µT]")
        ax.set_title(f"{AXIS_LABEL[a]}: up / down 別 duty平均")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
        d_arr = np.array([pdl["down"][d]["dB"][i] - pdl["up"][d]["dB"][i] for d in common])
        diffs[a] = d_arr
        ax2 = axarr[1, i]
        ax2.bar(common, d_arr, width=0.06, color="tab:purple", alpha=0.85)
        ax2.axhline(0, color="gray", lw=0.8)
        ax2.set_xlabel("duty"); ax2.set_ylabel("down − up [µT]")
        ax2.set_title(f"{AXIS_LABEL[a]}: down − up 差"); ax2.grid(alpha=0.3)
    fig.suptitle("⑩ ヒステリシス確認: up / down レグ別 ΔB と down − up 差", fontsize=13)
    save_fig(fig, out, "10_hysteresis_updown.png")

    print(f"\n=== 図⑩ up/down ヒステリシス（down − up, duty平均, 共通duty {len(common)} 点）[µT] ===")
    for a in AXES:
        d_arr = diffs[a]
        if d_arr.size:
            print(f"  {AXIS_LABEL[a]}: 平均 {np.nanmean(d_arr):+.3f} / 最大|差| {np.nanmax(np.abs(d_arr)):.3f}")
        else:
            print(f"  {AXIS_LABEL[a]}: 共通 duty なし")


def sample_level_fit(data: dict) -> dict:
    """measure 全サンプルで ΔB = a·I + b を軸別フィット（duty平均10点ではなく全点）。

    返り値: {"cur","t","temp", 軸: {"a","b","r2","se","resid","y"}}。
    """
    m = measure_mask(data)
    cur = col(data, "current_a")[m]
    res = {"cur": cur, "t": col(data, "t_s")[m], "temp": col(data, "imu_temp_c")[m]}
    dB = np.column_stack([col(data, f"dB_cor_{a}")[m] for a in AXES])
    for i, a in enumerate(AXES):
        fa, fb, r2, se, resid = fit_line_se(cur, dB[:, i])
        res[a] = {"a": fa, "b": fb, "r2": r2, "se": se, "resid": resid, "y": dB[:, i]}
    return res


def fig11_sample_level_fit(sf: dict, pdm, idle, dB0, out: Path) -> None:
    """図⑪ サンプル単位回帰: 全サンプルの直線フィットと duty平均フィットの比較。

    上段: 散布 + 両フィット直線（傾き±標準誤差を表示）。
    下段: 残差 vs 時間（系統残差＝ドリフト/温度等の確認）。
    """
    cur = sf["cur"]
    duties = sorted(pdm)
    fig, axarr = plt.subplots(2, 3, figsize=(15, 8.5))
    print("\n=== 図⑪ サンプル単位回帰 ΔB = a·I + b（measure 全サンプル）[µT/A, µT] ===")
    for i, a in enumerate(AXES):
        f = sf[a]
        # duty平均フィット（図③と同じ全域・duty=0込み）との比較
        xs, ys = _fit_points(pdm, idle, dB0, i, duties, include_duty0=True)
        da, db_, dr2 = fit_line(xs, ys)
        ax = axarr[0, i]
        ax.scatter(cur, f["y"], s=6, alpha=0.3, color="gray", label="サンプル")
        xline = np.linspace(np.nanmin(cur), np.nanmax(cur), 50)
        ax.plot(xline, f["a"] * xline + f["b"], "g-", lw=2,
                label=f"全サンプル a={f['a']:.3f}±{f['se']:.3f}\nR²={f['r2']:.3f}")
        ax.plot(xline, da * xline + db_, "r--", lw=1.5, label=f"duty平均 a={da:.3f}")
        ax.set_xlabel("電流 I [A]"); ax.set_ylabel(f"{AXIS_LABEL[a]} [µT]")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
        ax2 = axarr[1, i]
        ax2.scatter(sf["t"], f["resid"], s=6, alpha=0.35, color="tab:cyan")
        ax2.axhline(0, color="gray", lw=0.8)
        ax2.set_xlabel("時間 t [s]"); ax2.set_ylabel("残差 [µT]")
        ax2.set_title(f"{AXIS_LABEL[a]}: 残差 vs 時間"); ax2.grid(alpha=0.3)
        print(f"  {AXIS_LABEL[a]}: a={f['a']:.3f}±{f['se']:.3f} b={f['b']:.3f} R²={f['r2']:.3f} | "
              f"duty平均 a={da:.3f} → 差 {f['a'] - da:+.3f} µT/A")
    fig.suptitle("⑪ サンプル単位回帰（全点フィット±SE と duty平均フィットの比較 / 残差 vs 時間）",
                 fontsize=13)
    save_fig(fig, out, "11_sample_level_fit.png")


def _temp_regression_panel(ax, temp, y, ylabel, title, color) -> tuple[float, float]:
    """温度に対する回帰の共通サブプロット。返り値 (傾き µT/°C, 相関 r)。"""
    slope, b, _ = fit_line(temp, y)
    ok = np.isfinite(temp) & np.isfinite(y)
    r = float(np.corrcoef(temp[ok], y[ok])[0, 1]) if ok.sum() >= 2 else math.nan
    ax.scatter(temp, y, s=6, alpha=0.3, color=color)
    if np.isfinite(slope):
        xline = np.linspace(np.nanmin(temp), np.nanmax(temp), 50)
        ax.plot(xline, slope * xline + b, "-", color="#111111", lw=2,
                label=f"傾き {slope:+.3f} µT/°C\nr={r:.3f}")
        ax.legend(fontsize=8)
    ax.axhline(0, color="gray", lw=0.8)
    ax.set_xlabel("IMU温度 [°C]"); ax.set_ylabel(ylabel)
    ax.set_title(title); ax.grid(alpha=0.3)
    return slope, r


def fig12_residual_vs_temp(data: dict, sf: dict, out: Path) -> None:
    """図⑫ 温度ドリフトの確認（軸別回帰）。

    上段: サンプル単位フィットの残差 vs IMU温度。ブラケット基準補間後も
          温度相関が残っていれば、補正しきれない温度ドリフトがある。
    下段: motor-off 基準磁場（平均からの差）vs IMU温度。掃引中の生の
          温度ドリフト量 [µT/°C] が直接見える（ブラケット減算前）。
    """
    temp = sf["temp"]
    bm = baseline_rows_mask(data)
    btemp = col(data, "imu_temp_c")[bm]
    fig, axarr = plt.subplots(2, 3, figsize=(15, 8.5))
    print("\n=== 図⑫ 温度ドリフト（軸別回帰, 傾き [µT/°C]）===")
    for i, a in enumerate(AXES):
        slope, r = _temp_regression_panel(
            axarr[0, i], temp, sf[a]["resid"], "残差 [µT]",
            f"{AXIS_LABEL[a]}: フィット残差 vs 温度", "tab:orange")
        bvals = col(data, f"b{a}_cor")[bm]
        bvals = bvals - np.nanmean(bvals)
        bslope, br = _temp_regression_panel(
            axarr[1, i], btemp, bvals, f"B{a} − 平均 [µT]",
            f"B{a}: motor-off 基準 vs 温度", "tab:green")
        print(f"  {AXIS_LABEL[a]}: 残差 {slope:+.3f} µT/°C (r={r:.3f}) | "
              f"motor-off基準 {bslope:+.3f} µT/°C (r={br:.3f})")
    fig.suptitle("⑫ 残差 vs IMU温度（上: ブラケット補正後の残差 / 下: motor-off 基準の生ドリフト）",
                 fontsize=13)
    save_fig(fig, out, "12_residual_vs_temp.png")


# ------------------------------------------------------------ sweep main ------
def run_sweep(path: Path, out: Path) -> None:
    """スイープ1本の samples CSV から9〜12枚の図を出力する。"""
    global _COND_FOOTER
    out.mkdir(parents=True, exist_ok=True)

    data = load_samples(path)
    pdm = per_duty_measure(data)
    pdb = per_duty_baseline(data)
    idle, dB0 = duty0_point(data)
    if not pdm:
        sys.exit("measure 行がありません。CSVを確認してください。")

    print(f"入力: {path}\n出力: {out}\nmeasure duty 段数: {len(pdm)} / 基準 duty 段数: {len(pdb)} / アイドル電流: {idle:.3f} A")

    # meta JSON（同 stamp の …_meta.json）。無くても完全動作（旧フォーマット互換）。
    meta = load_meta(path)
    if meta is not None:
        print_meta_summary(meta)
        _COND_FOOTER = f"{path.name}  |  {meta_condition_line(meta)}"
    else:
        print("meta JSON なし（旧フォーマット）→ 条件サマリは省略します。")
        _COND_FOOTER = path.name
    flags = (meta or {}).get("baseline_flags") or []

    fig1_vector_4d(data, out)
    fig2_dB_vs_I(data, pdm, idle, dB0, out)
    fig3_dB_vs_I_fit(data, pdm, idle, dB0, out, flags=flags)
    fig4_dB_vs_I_flightband(data, pdm, idle, dB0, out)
    fig5_dB_magnitude(data, pdm, out)
    fig6_B_std(pdm, out)
    # 往復スイープでは duty キー平均だと up/down の2訪問が混ざり軌跡が潰れる
    # ため、step_idx ベースの時系列順基準点列を図⑦⑧に渡す（旧CSVは空→従来動作）。
    bseq = baseline_sequence(data)
    fig7_baseline_drift_3d(pdb, out, seq=bseq)
    fig8_baseline_components(pdb, out, seq=bseq)
    fig9_quality(data, pdm, out)
    print_fits(pdm, idle, dB0)
    n_figs = 9

    # --- 新フォーマット拡張（図⑩〜⑫）。旧CSV（step_idx/leg 列なし）はスキップ。 ---
    if has_leg_column(data):
        if measure_legs(data) >= {"up", "down"}:
            fig10_hysteresis(data, out)
            n_figs += 1
        else:
            print("\nleg 列はありますが up/down 両レグが揃っていないため 図⑩（ヒステリシス）はスキップしました。")
        sf = sample_level_fit(data)
        fig11_sample_level_fit(sf, pdm, idle, dB0, out)
        n_figs += 1
        if np.isfinite(sf["temp"]).any():
            fig12_residual_vs_temp(data, sf, out)
            n_figs += 1
        else:
            print("\nimu_temp_c が全て NaN のため 図⑫（残差 vs IMU温度）はスキップしました。")
    else:
        print("\nleg 列が無い旧フォーマットCSVのため 図⑩〜⑫（ヒステリシス / サンプル単位回帰 / 残差vs温度）はスキップしました。")

    print(f"\n{n_figs}枚の図を {out} に出力しました。")


# ==============================================================================
# [2] 加算性シーケンスの検証グラフ（旧 analyze_additivity.py）
# ==============================================================================

# ----------------------------------------------------------- aggregation ------
def aggregate_run(data: dict, idle: float, label: str, motors: str) -> dict:
    """measure 行を duty 別に集計して run 辞書を作る。

    各 duty の I と ΔB は up/down 両 leg の平均（leg ごとに平均 → 等重み平均）。
    noise は統計誤差 std/√n と leg 間差/2（経験的再現性）の大きい方を軸別に持つ。
    """
    m = data["phase"] == "measure"
    duty = data["duty_cmd"][m]
    cur = data["current_a"][m]
    leg = data["leg"][m] if "leg" in data else np.array([""] * int(m.sum()), dtype=object)
    dB = np.column_stack([data[f"dB_cor_{a}"][m] for a in AXES])
    pd_ = {}
    for d in sorted(set(np.round(duty, 3))):
        sel = np.isclose(duty, d)
        legs = sorted({lg for lg in leg[sel] if lg})
        if len(legs) >= 2:
            leg_I = []
            leg_dB = []
            for lg in legs:
                s2 = sel & (leg == lg)
                leg_I.append(float(np.nanmean(cur[s2])))
                leg_dB.append(np.nanmean(dB[s2], axis=0))
            leg_dB = np.array(leg_dB)
            I = float(np.mean(leg_I))
            v = np.mean(leg_dB, axis=0)
            leg_half = 0.5 * (np.max(leg_dB, axis=0) - np.min(leg_dB, axis=0))
        else:
            I = float(np.nanmean(cur[sel]))
            v = np.nanmean(dB[sel], axis=0)
            leg_half = np.zeros(3)
        n = int(np.sum(sel))
        sem = np.nanstd(dB[sel], axis=0) / max(1.0, math.sqrt(n))
        pd_[float(d)] = {"I": I, "dB": v, "noise": np.maximum(sem, leg_half)}
    return {"label": label, "motors": motors, "idle": float(idle), "pd": pd_}


def _idle_from(meta: dict | None, data: dict) -> float:
    """アイドル電流。meta の idle_current_a → 無ければ base 行（duty=0）の平均電流。"""
    idle = math.nan
    if meta is not None:
        idle = _f(str(meta.get("idle_current_a", "")))
    if not math.isfinite(idle):
        b = data["phase"] == "base"
        idle = float(np.nanmean(data["current_a"][b])) if b.any() else math.nan
    return idle


def load_sequence_runs(meta_path: Path, csv_dir: Path) -> tuple[list[dict], dict]:
    """sequence meta JSON から完了済み run を全て読み込んで集計する。

    返り値: (runs, sequence meta 辞書)。
    """
    seq = json.loads(meta_path.read_text(encoding="utf-8"))
    if seq.get("schema") != "stampfly_sweep_sequence_meta":
        print(f"警告: schema が想定外です: {seq.get('schema')!r}（続行します）")
    runs = []
    for r in seq.get("runs", []):
        motors = r.get("motors", "?")
        if r.get("phase") != "done" or r.get("aborted"):
            print(f"  スキップ: {motors}（phase={r.get('phase')}, aborted={r.get('aborted')}）")
            continue
        spath = csv_dir / r["samples"]
        if not spath.is_file():
            sys.exit(f"samples CSV が見つかりません: {spath}（--results-dir でディレクトリを指定できます）")
        data = load_samples(spath)
        meta = None
        if r.get("meta"):
            mpath = csv_dir / r["meta"]
            if mpath.is_file():
                meta = json.loads(mpath.read_text(encoding="utf-8"))
        runs.append(aggregate_run(data, _idle_from(meta, data),
                                  label=spath.stem, motors=motors))
    return runs, seq


# ------------------------------------------- 外部比較対象（全機スイープ）------
def _meta_epoch(meta: dict | None, path: Path) -> float:
    """meta の取得時刻を epoch 秒で返す（started_at_epoch → created_at → mtime）。"""
    if meta:
        ep = meta.get("started_at_epoch")
        if isinstance(ep, (int, float)) and math.isfinite(ep):
            return float(ep)
        ca = meta.get("created_at")
        if isinstance(ca, str):
            try:
                return datetime.strptime(ca, "%Y-%m-%dT%H:%M:%S%z").timestamp()
            except ValueError:
                pass
    return path.stat().st_mtime


def _sequence_epoch(seq: dict, meta_path: Path) -> float:
    """シーケンスの開始時刻（run_id → created_at → mtime）。"""
    rid = seq.get("run_id")
    if isinstance(rid, str):
        try:
            return time.mktime(time.strptime(rid, "%Y%m%d_%H%M%S"))
        except ValueError:
            pass
    return _meta_epoch(seq, meta_path)


def find_multi_candidates(csv_dir: Path, exclude_samples: set[str],
                          singles_avail: set[str]) -> list[dict]:
    """csv_dir から比較対象になれる多機スイープ候補を探す（新しい順）。

    条件: motors が全機（FL+FR+RL+RR）か対角ペア、aborted=false、
    samples CSV が実在、構成モーターの単機 run がシーケンス側に揃っている。
    """
    cands = []
    for mp in csv_dir.glob("sweep_*_meta.json"):
        try:
            meta = json.loads(mp.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if meta.get("aborted"):
            continue
        tokens = frozenset(t for t in str(meta.get("motors", "")).split("+") if t)
        if tokens not in ALLOWED_MULTI or not tokens <= singles_avail:
            continue
        stem = _strip_suffix(mp.stem, "_meta")
        spath = csv_dir / f"{stem}_samples.csv"
        if not spath.is_file() or spath.name in exclude_samples:
            continue
        cands.append({
            "stem": stem,
            "samples": spath,
            "motors": meta.get("motors", "?"),
            "orientation": (meta.get("notes") or {}).get("orientation") or "-",
            "epoch": _meta_epoch(meta, mp),
        })
    cands.sort(key=lambda c: c["epoch"], reverse=True)
    return cands


def _select_external_targets(cands: list[dict], seq_orient: str | None,
                             seq_epoch: float) -> list[dict] | None:
    """候補一覧から比較対象を対話選択する。返り値: 選択候補リスト / None（中止）。

    Enter はシーケンス（単機）と同姿勢で時刻が最も近い1本を自動選択。
    対話入力が無い（EOF）場合も同じ自動選択を行う。
    """
    print("\nシーケンスは単機のみです。比較対象の全機スイープを選択してください")
    print("（複数可、カンマ区切り、Enter=単機と同姿勢で時刻が最も近い1本、q=中止）:\n")
    for i, c in enumerate(cands, 1):
        ts = time.strftime("%Y-%m-%d %H:%M", time.localtime(c["epoch"]))
        same = "  ← 同姿勢" if c["orientation"] == seq_orient else ""
        print(f"  [{i}] {c['stem']}   motors={c['motors']}   "
              f"orientation={c['orientation']}   {ts}{same}")
    print()
    same_or = [c for c in cands if c["orientation"] == seq_orient]
    default = min(same_or, key=lambda c: abs(c["epoch"] - seq_epoch)) if same_or else None
    while True:
        try:
            raw = input(f"番号を入力 [1-{len(cands)}]（カンマ区切り可 / Enter=自動 / q=中止）: ").strip()
        except EOFError:
            if default is not None:
                print(f"（対話入力なし → 同姿勢・時刻最近傍 {default['stem']} を自動選択）")
                return [default]
            print("（対話入力なし・同姿勢の候補なし → 中止）")
            return None
        if raw == "":
            if default is not None:
                print(f"（同姿勢・時刻最近傍 {default['stem']} を自動選択）")
                return [default]
            print("  同じ姿勢（orientation）の候補がありません。番号で明示選択するか q で中止してください。")
            continue
        if raw.lower() in ("q", "quit", "exit"):
            return None
        parts = [p.strip() for p in raw.split(",") if p.strip()]
        if parts and all(p.isdigit() and 1 <= int(p) <= len(cands) for p in parts):
            seen: set[int] = set()
            sel = []
            for p in parts:
                idx = int(p)
                if idx not in seen:
                    seen.add(idx)
                    sel.append(cands[idx - 1])
            return sel
        print(f"  '{raw}' は無効です。1〜{len(cands)} の番号（カンマ区切り）を入力してください。")


def _resolve_target_samples(spec: str, csv_dir: Path) -> Path:
    """--target の値（stem または samples.csv パス）を samples CSV パスに解決する。"""
    p = Path(spec)
    if p.is_file():
        if p.name.endswith("_meta.json"):
            p = p.with_name(_strip_suffix(p.stem, "_meta") + "_samples.csv")
        return p
    stem = spec
    for suf in (".csv", ".json"):
        if stem.endswith(suf):
            stem = stem[: -len(suf)]
    stem = _strip_suffix(_strip_suffix(stem, "_samples"), "_meta")
    return csv_dir / f"{stem}_samples.csv"


def load_external_run(samples: Path, motors: str | None = None) -> dict:
    """外部指定の多機スイープを run 辞書に読み込む（source='external'）。"""
    if not samples.is_file():
        sys.exit(f"比較対象の samples CSV が見つかりません: {samples}")
    meta = load_meta(samples)
    data = load_samples(samples)
    if motors is None:
        motors = (meta or {}).get("motors")
    if not motors and "motors" in data:
        motors = next((m for m in data["motors"] if m), None)
    if not motors:
        sys.exit(f"比較対象のモーター構成（motors）が不明です: {samples}")
    if meta is not None and meta.get("aborted"):
        print(f"  警告: {samples.name} は aborted=true のスイープです（続行します）。")
    run = aggregate_run(data, _idle_from(meta, data),
                        label=_strip_suffix(samples.stem, "_samples"), motors=str(motors))
    run["source"] = "external"
    run["orientation"] = ((meta or {}).get("notes") or {}).get("orientation") or "-"
    return run


# ------------------------------------------------------------- analysis -------
def fit_slope(run: dict) -> np.ndarray:
    """ΔB = a·(I−idle) の原点通過最小二乗フィット（軸別）。a = Σxy/Σx²。"""
    duties = sorted(run["pd"])
    x = np.array([run["pd"][d]["I"] for d in duties]) - run["idle"]
    Y = np.array([run["pd"][d]["dB"] for d in duties])
    a = np.full(3, math.nan)
    for i in range(3):
        ok = np.isfinite(x) & np.isfinite(Y[:, i])
        denom = float(np.sum(x[ok] ** 2))
        if denom > 0:
            a[i] = float(np.sum(x[ok] * Y[ok, i])) / denom
    return a


def compare_additivity(target: dict, singles: list[dict]):
    """共通 duty での Σ単機予測 と 同時測定 を比較する。

    返り値: (keys, meas[N,3], pred[N,3], resid[N,3], noise[N,3])
    resid = pred − meas。noise は両者の合成（√和の二乗）。
    """
    common = set(target["pd"])
    for s in singles:
        common &= set(s["pd"])
    keys = sorted(common)
    if not keys:
        sys.exit(f"共通 duty がありません: {target['motors']} と単機 run の duty 段を確認してください。")
    meas = np.array([target["pd"][k]["dB"] for k in keys])
    pred = np.array([np.sum([s["pd"][k]["dB"] for s in singles], axis=0) for k in keys])
    noise = np.array([
        np.sqrt(target["pd"][k]["noise"] ** 2
                + np.sum([s["pd"][k]["noise"] ** 2 for s in singles], axis=0))
        for k in keys
    ])
    return keys, meas, pred, pred - meas, noise


def print_main_table(name: str, keys, meas, pred, resid) -> None:
    print(f"\n=== 主検証（duty一致比較）: {name} ===")
    print("    残差 = Σ単機予測 − 同時測定 [µT]")
    print("  duty  | " + "  ".join(f"測定{a}{'':>4}予測{a}{'':>4}残差{a}{'':>2}" for a in AXES))
    for j, k in enumerate(keys):
        cells = "  ".join(f"{meas[j, i]:7.2f} {pred[j, i]:7.2f} {resid[j, i]:+6.2f}  "
                          for i in range(3))
        print(f"  {k:5.2f} | {cells}")
    mean_abs = np.nanmean(np.abs(resid), axis=0)
    max_abs = np.nanmax(np.abs(resid), axis=0)
    print("  " + "-" * 70)
    print("  平均|残差| [µT]: " + "  ".join(f"{a}={mean_abs[i]:.3f}" for i, a in enumerate(AXES)))
    print("  最大|残差| [µT]: " + "  ".join(f"{a}={max_abs[i]:.3f}" for i, a in enumerate(AXES)))


def print_slope_check(name: str, target: dict, singles: list[dict]) -> None:
    """副検証: 電流空間の傾き a [µT/A] の整合チェック。

    同時運転では各モーターが総電流のほぼ 1/機数 を分担するため、加算性が
    成り立てば slope_同時 ≈ (Σ_m slope_m)/機数 になる（4機なら /4、ペアなら /2）。
    """
    tokens = [t for t in target["motors"].split("+") if t]
    divisor = len(tokens) if len(tokens) > 1 else len(singles)
    a_t = fit_slope(target)
    a_sum = np.sum([fit_slope(s) for s in singles], axis=0) / divisor
    print(f"\n=== 副検証（電流空間の傾き）: {name}  ΔB = a·(I−idle) [µT/A] ===")
    print(f"    仮定: 同時運転時、各モーターは総電流のほぼ 1/{divisor} を分担")
    print(f"  軸 | slope_同時   Σ単機slope/{divisor}      差")
    for i, a in enumerate(AXES):
        print(f"  {a}  | {a_t[i]:10.3f}   {a_sum[i]:13.3f}   {a_t[i] - a_sum[i]:+7.3f}")


def print_current_check(name: str, target: dict, singles: list[dict], keys) -> None:
    """電圧垂下の注意の定量化: 同 duty の電流 I_同時 と Σ(I_m−idle_m)+idle_同時 の比較。"""
    print(f"\n=== 電圧垂下チェック（同 duty の電流比較）: {name} ===")
    print("  duty  | I_同時 [A]  Σ(I_m−idle_m)+idle_同時 [A]    差 [A]")
    diffs = []
    for k in keys:
        i_t = target["pd"][k]["I"]
        i_pred = sum(s["pd"][k]["I"] - s["idle"] for s in singles) + target["idle"]
        diffs.append(i_t - i_pred)
        print(f"  {k:5.2f} | {i_t:9.3f}   {i_pred:21.3f}   {i_t - i_pred:+8.3f}")
    diffs = np.array(diffs)
    print(f"  差の平均 {np.nanmean(diffs):+.3f} A / 最大|差| {np.nanmax(np.abs(diffs)):.3f} A")
    print("  注意: 同時運転は電圧垂下でモーター個別の電流が単機時より下がるため、")
    print("        duty一致比較には系統差が乗り得る（差が大きいほど ΔB 比較にも影響）。")


def verdict_info(name: str, resid: np.ndarray, noise: np.ndarray) -> dict:
    """判定サマリ: 最大残差をノイズ由来のしきい値と比較して 成立/不成立 を返す。

    しきい値 = max(NOISE_SIGMA × RMS(合成ノイズ), NOISE_FLOOR_UT)。
    """
    max_resid = float(np.nanmax(np.abs(resid)))
    finite = noise[np.isfinite(noise)]
    if finite.size:
        noise_typ = float(np.sqrt(np.mean(finite ** 2)))  # 合成ノイズの RMS
        thr = max(NOISE_SIGMA * noise_typ, NOISE_FLOOR_UT)
        ok = max_resid <= thr
        line = (f"加算性[{name}]: 最大残差 {max_resid:.2f} µT"
                f"（測定ノイズ~{noise_typ:.2f} µT・しきい値 {thr:.2f} µT に対し"
                f"{'成立' if ok else '不成立'}）")
    else:
        noise_typ = math.nan
        thr = NOISE_FLOOR_UT
        ok = max_resid <= thr
        line = (f"加算性[{name}]: 最大残差 {max_resid:.2f} µT"
                f"（測定ノイズ不明・しきい値 {NOISE_FLOOR_UT:.2f} µT に対し"
                f"{'成立' if ok else '不成立'}）")
    return {"name": name, "max_resid": max_resid, "noise_typ": noise_typ,
            "thr": thr, "ok": ok, "line": line}


# -------------------------------------------------------------- figures -------
def fig_additivity(name: str, fname: str, keys, meas, pred, resid, noise,
                   out: Path) -> None:
    """軸別: 上段 duty vs 測定/予測の重ねプロット、下段 残差（±3σ ノイズ帯付き）。"""
    fig, axarr = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    for i, a in enumerate(AXES):
        ax = axarr[0, i]
        ax.plot(keys, meas[:, i], "o-", color="tab:blue", label="同時測定 ΔB")
        ax.plot(keys, pred[:, i], "s--", color="tab:red", label="Σ単機 予測")
        ax.set_title(AXIS_LABEL[a])
        ax.set_ylabel("ΔB [µT]")
        ax.grid(alpha=0.3)
        rx = axarr[1, i]
        if np.isfinite(noise[:, i]).all():
            rx.fill_between(keys, -3 * noise[:, i], 3 * noise[:, i],
                            color="gray", alpha=0.35, label="±3σ ノイズ")
        rx.plot(keys, resid[:, i], "o-", color="tab:green", label="残差（予測−測定）")
        rx.axhline(0, color="gray", lw=0.8)
        rx.set_xlabel("duty")
        rx.set_ylabel("残差 [µT]")
        rx.grid(alpha=0.3)
    axarr[0, 0].legend(fontsize=8)
    axarr[1, 0].legend(fontsize=8)
    fig.suptitle(f"加算性検証 {name}: Σ単機 ΔB と同時スイープ ΔB の比較", fontsize=13)
    save_fig(fig, out, fname)


def fig_verdict_summary(verdicts: list[dict], out: Path) -> None:
    """判定サマリを PNG として残す（00_verdict_summary.png）。"""
    n = len(verdicts)
    fig, ax = plt.subplots(figsize=(11, 1.6 + 1.0 * n))
    ax.axis("off")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.text(0.02, 0.96, "加算性 判定サマリ", fontsize=15, fontweight="bold", va="top")
    ax.text(0.02, 0.96 - 0.10, f"判定基準: 最大|残差| <= max({NOISE_SIGMA:.0f}σ×RMS(合成ノイズ), "
            f"{NOISE_FLOOR_UT:.1f} µT)", fontsize=9, color="#555555", va="top")
    y = 0.96 - 0.24
    dy = 0.72 / max(1, n)
    for v in verdicts:
        mark = "成立" if v["ok"] else "不成立"
        color = "tab:green" if v["ok"] else "tab:red"
        ax.text(0.02, y, f"[{mark}]", fontsize=12, fontweight="bold", color=color, va="top")
        noise_s = f"{v['noise_typ']:.2f} µT" if math.isfinite(v["noise_typ"]) else "不明"
        ax.text(0.13, y, f"{v['name']}\n最大残差 {v['max_resid']:.2f} µT / "
                f"測定ノイズ~{noise_s} / しきい値 {v['thr']:.2f} µT",
                fontsize=10, va="top")
        y -= dy
    save_fig(fig, out, "00_verdict_summary.png")


# ------------------------------------------------------------- comparison -----
def build_comparisons(runs: list[dict]) -> list[tuple[dict, list[dict]]]:
    """run 一覧から (同時run, [構成する単機run...]) の比較ペアを組む。

    4機同時（FL+FR+RL+RR 相当）→ 単機4本の和、対角ペア → 該当2機の和。
    """
    singles = {r["motors"]: r for r in runs if r["motors"] in SINGLES}
    comps = []
    for r in runs:
        tokens = [t for t in r["motors"].split("+") if t]
        if len(tokens) < 2:
            continue
        if all(t in singles for t in tokens):
            comps.append((r, [singles[t] for t in tokens]))
        else:
            missing = [t for t in tokens if t not in singles]
            print(f"  比較スキップ: {r['motors']}（単機 run が不足: {'+'.join(missing)}）")
    return comps


def _target_note(target: dict, seq_orient: str | None) -> str:
    """比較対象の由来（sequence 内 / 外部選択）と姿勢の注記（フッター・表用）。"""
    orient = target.get("orientation") or "-"
    src = "外部選択" if target.get("source") == "external" else "シーケンス内"
    note = f"比較対象={src} {target['label']} (orientation={orient})"
    if seq_orient and orient not in ("-", seq_orient):
        note += " ※姿勢が異なる比較（参考）"
    return note


def analyze_comparisons(comps: list[tuple[dict, list[dict]]], out: Path,
                        seq_orient: str | None = None) -> None:
    """全比較ペアについて 主検証・副検証・電流チェック・図 を実行し判定サマリを出す。"""
    global _COND_FOOTER
    base_footer = _COND_FOOTER
    verdicts = []
    for idx, (target, singles) in enumerate(comps, 1):
        name = f"{' + '.join(s['motors'] for s in singles)} → {target['motors']}"
        note = _target_note(target, seq_orient)
        print(f"\n--- {note} ---")
        _COND_FOOTER = f"{base_footer}  |  {note}" if base_footer else note
        keys, meas, pred, resid, noise = compare_additivity(target, singles)
        print_main_table(name, keys, meas, pred, resid)
        print_slope_check(name, target, singles)
        print_current_check(name, target, singles, keys)
        fname = f"{idx:02d}_additivity_{target['motors'].replace('+', '_')}_{target['label']}.png"
        fig_additivity(name, fname, keys, meas, pred, resid, noise, out)
        verdicts.append(verdict_info(name, resid, noise))
    _COND_FOOTER = base_footer
    if any(t.get("source") == "external" for t, _ in comps):
        ext = ", ".join(_target_note(t, seq_orient) for t, _ in comps
                        if t.get("source") == "external")
        _COND_FOOTER = f"{base_footer}  |  {ext}" if base_footer else ext
    print("\n=== 判定サマリ ===")
    for v in verdicts:
        print(f"  {v['line']}")
    fig_verdict_summary(verdicts, out)
    _COND_FOOTER = base_footer
    print(f"\n図 {len(comps) + 1} 枚（比較 {len(comps)} + 判定サマリ 1）を {out} に出力しました。")


# ------------------------------------------------------- additivity main ------
def run_additivity(meta_path: Path, csv_dir: Path, out: Path,
                   targets: list[str] | None = None) -> None:
    """sequence meta JSON から加算性検証の図と判定を出力する。

    シーケンスが単機のみの場合は csv_dir から多機スイープ候補を探し、
    対話選択（または --target 指定）で外部の比較対象を組み合わせる。
    """
    global _COND_FOOTER
    out.mkdir(parents=True, exist_ok=True)
    print(f"入力: {meta_path}\nCSVディレクトリ: {csv_dir}\n出力: {out}")
    runs, seq = load_sequence_runs(meta_path, csv_dir)
    if not runs:
        sys.exit("完了済み（phase=='done'）の run がありません。")
    seq_orient = (seq.get("notes") or {}).get("orientation")
    for r in runs:
        r.setdefault("source", "sequence")
        r.setdefault("orientation", seq_orient or "-")
        print(f"  {r['label']}: motors={r['motors']} idle={r['idle']:.3f} A "
              f"duty段数={len(r['pd'])}")
    cond = [meta_path.name, f"pattern={seq.get('pattern', '-')}"]
    if seq.get("notes"):
        cond.append(f"notes={seq['notes']}")
    _COND_FOOTER = "  |  ".join(cond)

    # --target 明示指定（対話質問なし）
    external: list[dict] = []
    if targets:
        for spec in targets:
            external.append(load_external_run(_resolve_target_samples(spec, csv_dir)))

    # シーケンスが単機のみ → csv_dir から多機スイープ候補を探して対話選択
    has_multi = any(len([t for t in r["motors"].split("+") if t]) >= 2 for r in runs)
    if not has_multi and not external:
        singles_avail = {r["motors"] for r in runs if r["motors"] in SINGLES}
        exclude = {Path(str(r2.get("samples", ""))).name for r2 in seq.get("runs", [])}
        cands = find_multi_candidates(csv_dir, exclude, singles_avail)
        if cands:
            sel = _select_external_targets(cands, seq_orient,
                                           _sequence_epoch(seq, meta_path))
            if sel is None:
                sys.exit("中止しました（比較対象が選択されていません）。")
            for c in sel:
                external.append(load_external_run(c["samples"], motors=c["motors"]))

    for r in external:
        print(f"  外部比較対象 {r['label']}: motors={r['motors']} idle={r['idle']:.3f} A "
              f"duty段数={len(r['pd'])} orientation={r.get('orientation', '-')}")
    comps = build_comparisons(runs + external)
    if not comps:
        sys.exit("加算性検証には全機同時スイープが必要です。\n"
                 "Experiment タブの電流×磁場スイープで FL+FR+RL+RR を1本取得してください。")
    analyze_comparisons(comps, out, seq_orient=seq_orient)


# ==============================================================================
# エントリポイント（自動判別 + 対話メニュー）
# ==============================================================================
def detect_mode(path: Path) -> str:
    """入力パスから解析モードを判別する（"sweep" / "sequence"）。"""
    name = path.name.lower()
    if name.endswith(".json") or "sequence" in name:
        return "sequence"
    if name.endswith(".csv"):
        return "sweep"
    sys.exit(f"入力の種類を判別できません: {path}\n"
             f"sweep_*_samples.csv または sequence_*_meta.json を指定してください。")


def interactive_select(results_dir: Path) -> tuple[str, Path] | None:
    """メニュー → ファイル番号選択の対話フロー。返り値 (mode, path) または None。"""
    print("\n=== plot_sweep: スイープ結果のグラフ化 ===")
    print("  [1] スイープ1本のグラフ化        （sweep_*_samples.csv → 図 9〜12枚）")
    print("  [2] 加算性シーケンスの検証グラフ（sequence_*_meta.json → 比較図＋判定）")
    print()
    while True:
        try:
            raw = input("メニュー番号を入力 [1-2]（q=中止）: ").strip()
        except EOFError:
            print("（対話入力なし → [1] スイープ1本のグラフ化 を自動選択）")
            raw = "1"
        if raw.lower() in ("q", "quit", "exit"):
            return None
        if raw in ("1", "2"):
            break
        print(f"  '{raw}' は無効です。1 か 2 を入力してください。")
    if raw == "1":
        cands = _list_by_mtime(results_dir, SWEEP_GLOB)
        if not cands:
            sys.exit(f"samples CSV が見つかりません: {results_dir} に {SWEEP_GLOB} がありません。")
        path = _select_from_list(cands, "samples CSV", results_dir)
        return ("sweep", path) if path else None
    cands = _list_by_mtime(results_dir, SEQ_GLOB)
    if not cands:
        sys.exit(f"sequence meta が見つかりません: {results_dir} に {SEQ_GLOB} がありません。")
    path = _select_from_list(cands, "sequence meta JSON", results_dir)
    return ("sequence", path) if path else None


def main() -> None:
    ap = argparse.ArgumentParser(
        description="StampFly スイープ結果のグラフ化（校正解析 + 加算性検証）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="引数を省略すると対話メニュー（[1]スイープ1本 / [2]加算性シーケンス）で\n"
               "対象ファイルを番号選択できます。")
    ap.add_argument("path", nargs="?",
                    help="sweep_*_samples.csv または sequence_*_meta.json"
                         "（省略時は対話メニューで選択）")
    ap.add_argument("-o", "--out",
                    help="出力ディレクトリ（省略時は data_analysis/graphs/sweep_<stamp>/ "
                         "または graphs/additivity_<stem>/）")
    ap.add_argument("--results-dir",
                    help="スイープ結果ディレクトリ（一覧の探索先 / シーケンスの samples CSV の場所。"
                         "既定: ../pc_server/data/sweep_results/）")
    ap.add_argument("--target", action="append", metavar="STEM_OR_CSV",
                    help="加算性検証の比較対象（多機スイープの stem または samples.csv パス。"
                         "複数指定可）。指定時は対話質問しない。sequence 入力時のみ有効。")
    args = ap.parse_args()

    results_dir = Path(args.results_dir) if args.results_dir else default_results_dir()

    if args.path:
        path = Path(args.path)
        if not path.is_file():
            sys.exit(f"ファイルが見つかりません: {path}")
        mode = detect_mode(path)
    else:
        sel = interactive_select(results_dir)
        if sel is None:
            sys.exit("中止しました（対象が選択されていません）。")
        mode, path = sel

    if mode == "sweep":
        if args.target:
            print("警告: --target はスイープ解析では使いません（無視します）。")
        stem = _strip_suffix(path.stem, "_samples")
        out = Path(args.out) if args.out else graphs_dir() / stem
        run_sweep(path, out)
    else:
        # samples CSV の場所: --results-dir 指定があればそこ、無ければ meta と同じ場所
        csv_dir = Path(args.results_dir) if args.results_dir else path.parent
        stem = _strip_suffix(path.stem, "_meta")
        out = Path(args.out) if args.out else graphs_dir() / f"additivity_{stem}"
        run_additivity(path, csv_dir, out, targets=args.target)


if __name__ == "__main__":
    main()
