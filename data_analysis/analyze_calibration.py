#!/usr/bin/env python3
"""StampFly Yaw 校正データ解析（bracket-baseline サンプルCSV → 9〜12枚の図）.

入力は pc_server が出力する samples-only CSV（サマリ/メタ無し）。
測定行(phase=measure)には pc_server が前後ブラケット基準を時間補間して引いた
ドリフト除去後の電流ノイズ dB_cor_{x,y,z} と、引いた基準 bx_base_cor.. が入っている。
基準行(phase=baseline / 初期 base)には各duty の motor-off 基準磁場 b_cor が入る。

新フォーマット（step_idx / leg 列つき、往復スイープ 0.1→1.0→0.1）の場合は
さらに 図⑩ ヒステリシス（up/down分離）・図⑪ サンプル単位回帰・
図⑫ 残差 vs IMU温度 を追加出力する（計12枚）。同じ stamp の
sweep_<stamp>_meta.json があれば測定条件サマリを冒頭に表示し、
baseline_flags（基準ジャンプ>閾値のステップ）を図③に赤×で重ねる。
旧フォーマット CSV では従来どおり9枚を出力し、新機能は安全にスキップする。

使い方:
    python analyze_calibration.py                       # 一覧から対話選択
    python analyze_calibration.py samples.csv [-o 出力ディレクトリ]
    引数を省略すると ../pc_server/sweep_results/ 内の samples CSV を新しい順に
    一覧表示し、番号を入力して1つ選ぶ（会話的選択）。図の出力先は既定で
    data_analysis/graphs/analysis_<stem>/（この解析フォルダ配下）。

依存: numpy, matplotlib のみ。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d projection)

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


# ---------------------------------------------------------------- load --------
def _f(value: str) -> float:
    """空欄/欠損は NaN。"""
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def load_samples(path: Path) -> dict:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    if not rows:
        sys.exit(f"空のCSVです: {path}")
    cols = rows[0].keys()
    out = {"phase": np.array([r.get("phase", "") for r in rows], dtype=object)}
    for key in cols:
        if key in ("phase", "motors"):
            continue
        if key == "leg":  # 新フォーマット: "up"/"down" の文字列列（数値化しない）
            out[key] = np.array([r.get(key) or "" for r in rows], dtype=object)
            continue
        out[key] = np.array([_f(r.get(key, "")) for r in rows], dtype=float)
    return out


def col(data: dict, name: str) -> np.ndarray:
    return data.get(name, np.full(len(data["phase"]), np.nan))


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
    stem = samples.stem
    if stem.endswith("_samples"):
        stem = stem[: -len("_samples")]
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
def save_fig(fig, out: Path, name: str) -> None:
    """全図共通の保存処理（レイアウト調整 → PNG 出力 → クローズ）。"""
    fig.tight_layout()
    fig.savefig(out / name, dpi=130, bbox_inches="tight")
    plt.close(fig)


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
        ax.scatter(cur, dB[:, i], s=6, alpha=0.25, color="tab:gray", label="サンプル")
        mean_dB = np.array([pdm[d]["dB"][i] for d in duties])
        ax.scatter(mean_I, mean_dB, s=55, color="tab:red", zorder=5, label="duty平均")
        if band is None and np.isfinite(idle):
            ax.scatter([idle], [dB0[i]], s=70, marker="D", color="tab:blue", zorder=6, label="duty=0")
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
    ax.scatter([0], [0], [0], color="black", marker="x", s=80, label="初期 base")
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
        ax.scatter(cur, f["y"], s=6, alpha=0.25, color="tab:gray", label="サンプル")
        xline = np.linspace(np.nanmin(cur), np.nanmax(cur), 50)
        ax.plot(xline, f["a"] * xline + f["b"], "g-", lw=2,
                label=f"全サンプル a={f['a']:.3f}±{f['se']:.3f}\nR²={f['r2']:.3f}")
        ax.plot(xline, da * xline + db_, "r--", lw=1.5, label=f"duty平均 a={da:.3f}")
        ax.set_xlabel("電流 I [A]"); ax.set_ylabel(f"{AXIS_LABEL[a]} [µT]")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
        ax2 = axarr[1, i]
        ax2.scatter(sf["t"], f["resid"], s=6, alpha=0.3, color="tab:blue")
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
        ax.plot(xline, slope * xline + b, "k-", lw=2,
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


# ---------------------------------------------------------------- main --------
SWEEP_GLOB = "sweep_*_samples.csv"


def sweep_dir() -> Path:
    """pc_server が CSV を書き出す sweep_results フォルダ。"""
    return Path(__file__).resolve().parent.parent / "pc_server" / "sweep_results"


def list_samples() -> list[Path]:
    """sweep_results 内の samples CSV を新しい順（mtime 降順）に返す。"""
    d = sweep_dir()
    if not d.is_dir():
        return []
    return sorted(d.glob(SWEEP_GLOB), key=lambda p: p.stat().st_mtime, reverse=True)


def _fmt_size(n: int) -> str:
    return f"{n / 1024:.0f} KB" if n < 1024 * 1024 else f"{n / 1024 / 1024:.1f} MB"


def choose_samples(cands: list[Path]) -> Path | None:
    """利用可能な samples CSV を一覧表示し、番号で1つ選ばせる（会話的選択）。

    Enter は最新（[1]）を選択、q で中止して None を返す。
    パイプ等で対話入力が無い（EOF）場合は最新を自動選択する。
    """
    print(f"\n{sweep_dir()} の samples CSV から、グラフ化するものを選んでください:\n")
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


def default_out_dir(path: Path) -> Path:
    """既定の出力先: data_analysis/graphs/analysis_<stem>/（この解析フォルダ配下）。

    CSV ごとにサブフォルダを分けるので、別の CSV を選んでも上書きされない。
    """
    return Path(__file__).resolve().parent / "graphs" / f"analysis_{path.stem}"


def main() -> None:
    ap = argparse.ArgumentParser(description="StampFly Yaw 校正データ解析（9〜12枚の図）")
    ap.add_argument("samples", nargs="?",
                    help="samples CSV パス（省略時は sweep_results の一覧から対話選択）")
    ap.add_argument("-o", "--out",
                    help="出力ディレクトリ（省略時は data_analysis/graphs/analysis_<stem>/）")
    args = ap.parse_args()

    if args.samples:
        path = Path(args.samples)
    else:
        cands = list_samples()
        if not cands:
            sys.exit(f"samples CSV が見つかりません: {sweep_dir()} に {SWEEP_GLOB} がありません。")
        path = choose_samples(cands)
        if path is None:
            sys.exit("中止しました（CSV が選択されていません）。")
    if not path.is_file():
        sys.exit(f"samples CSV が見つかりません: {path}")

    out = Path(args.out) if args.out else default_out_dir(path)
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
    else:
        print("meta JSON なし（旧フォーマット）→ 条件サマリは省略します。")
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


if __name__ == "__main__":
    main()
