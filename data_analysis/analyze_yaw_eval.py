#!/usr/bin/env python3
"""StampFly Yaw補正実験ログ（yaw_eval_results）の可視化。

`pc_server/yaw_eval_results/yawlog_*_log.csv` を選択し、FF電流補正＋EKF が
どれだけヨー誤差を減らせたかを確認するための図を出力する。

機体をテープで固定した静置ベンチ記録が前提。真のヨーは一定なので、
**ヨーが停止時（モーターOFF）の基準値から動いた分 = そのまま誤差** と読める。
補正前（yu: リファレンス相補フィルタ）と補正後（y: FF+EKF）の「基準からのずれ」を
比べることで、電流ノイズをどれだけ打ち消せているかが分かる。

出力（analyze_calibration.py と同じ連番PNGスタイル）:
  ① Yaw時系列オーバーレイ（mt / yu / y + モーター状態）
  ①' 補正後Yaw y の詳細（y のみ自動スケール拡大）
  ② Yaw誤差 vs 電流（静置ベンチの核心指標）
  ③ 補正前後の誤差分布（duty帯別・箱ひげ）
  ④ FF補正量 ΔB̂ vs 電流（参考affineと重ね描き）
  ⑤ 生磁気ベクトルの外乱 vs 電流（mag3D前座標）
  ⑥ EKF健全性: NIS ＋ ゲート発火のラスタ
  ⑦ 磁気バイアス b_m ＋ 背景（温度・電圧）
  ⓪ 数値サマリ（コンソール＋パネル）

使い方:
  python analyze_yaw_eval.py                     # yaw_eval_results から対話選択
  python analyze_yaw_eval.py path/to_log.csv     # CSV を直接指定
  python analyze_yaw_eval.py -o out_dir          # 出力先を指定
"""

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

# 日本語フォント（macOS優先）。見つからなければ既定フォントのまま（英字は問題なし）。
_INSTALLED = {f.name for f in fm.fontManager.ttflist}
for _jp in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic", "Yu Gothic",
            "Noto Sans CJK JP", "IPAexGothic", "Apple SD Gothic Neo", "Arial Unicode MS"):
    if _jp in _INSTALLED:
        plt.rcParams["font.family"] = _jp
        break
plt.rcParams["axes.unicode_minus"] = False

LOG_GLOB = "yawlog_*_log.csv"

# ffg（EKFゲート状態ビット, yaw_estimator_kf.hpp と一致）
GATE_BITS = [
    ("R膨張(soft)", "NIS>5.99 / norm 8-20µT → R膨張して採用", "#fbbf24"),
    ("NIS棄却", "NIS>13.8 → 磁気更新スキップ→ジャイロ滑走", "#ef4444"),
    ("norm棄却", "|‖B_corr‖−‖B0‖|>20µT → スキップ", "#f97316"),
    ("z棄却", "|B_corr.z−B0.z|>12µT → スキップ", "#a855f7"),
    ("tilt>25°", "傾き過大 → スキップ", "#64748b"),
    ("b_m凍結", "‖b_m‖>20µT → 磁気更新凍結(要再アンカー)", "#dc2626"),
    ("ドリフト警告", "|db_m/dt|>0.3µT/s 10s継続", "#0ea5e9"),
]


# ---------------------------------------------------------------- load --------
def _f(value) -> float:
    """空欄/欠損は NaN。"""
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def load_log(path: Path) -> dict:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    if not rows:
        sys.exit(f"空のCSVです: {path}")
    out = {}
    for key in rows[0].keys():
        out[key] = np.array([_f(r.get(key, "")) for r in rows], dtype=float)
    return out


def col(data: dict, name: str, n: int) -> np.ndarray:
    return data.get(name, np.full(n, np.nan))


def meta_path_for(log: Path) -> Path:
    name = log.name
    if name.endswith("_log.csv"):
        name = name[: -len("_log.csv")] + "_meta.json"
    return log.with_name(name)


def load_meta(log: Path) -> dict | None:
    mp = meta_path_for(log)
    if not mp.is_file():
        return None
    try:
        return json.loads(mp.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        print(f"meta JSON の読み込みに失敗（無視して続行）: {mp} ({e})")
        return None


# --------------------------------------------------------------- angles -------
def wrap180(deg: np.ndarray) -> np.ndarray:
    """角度差を [-180, 180) に畳む。"""
    return (deg + 180.0) % 360.0 - 180.0


def circ_mean_deg(a: np.ndarray) -> float:
    """有限値のみで円周平均（deg）。空なら NaN。"""
    a = a[np.isfinite(a)]
    if a.size == 0:
        return math.nan
    r = np.deg2rad(a)
    return math.degrees(math.atan2(np.mean(np.sin(r)), np.mean(np.cos(r))))


def unwrap_deg(a: np.ndarray) -> np.ndarray:
    """連続系列（欠損なし想定）の deg を unwrap して跳びを消す。"""
    return np.rad2deg(np.unwrap(np.deg2rad(a)))


# --------------------------------------------------------------- derive -------
def derive(data: dict) -> dict:
    """図・サマリで共通に使う派生量をまとめて計算する。"""
    n = len(next(iter(data.values())))
    t = col(data, "t_s", n)
    y = col(data, "y", n)        # 補正後（アクティブ推定器 = FF+EKF）
    yu = col(data, "yu", n)      # 補正前（リファレンス相補フィルタ）
    mt = col(data, "mt", n)      # 生の磁気方位（融合前）
    ml = col(data, "ml", n)      # 水平化磁気方位（filtered）
    cur = col(data, "cur", n)
    md = col(data, "md", n)
    mr = col(data, "mr", n)
    ffg = np.nan_to_num(col(data, "ffg", n)).astype(int)
    ffm = np.nan_to_num(col(data, "ffm", n)).astype(int)
    fes = np.nan_to_num(col(data, "fes", n)).astype(int)

    motor_off = mr < 0.5                       # 停止区間（基準の母集団）
    motor_on = ~motor_off
    base_y = circ_mean_deg(y[motor_off])
    base_yu = circ_mean_deg(yu[motor_off])

    # 各推定器の「自分の停止時基準からのずれ」= 誤差
    err_y = np.abs(wrap180(y - base_y))
    err_yu = np.abs(wrap180(yu - base_yu))

    # 生磁気ベクトルの外乱（停止時平均からの差分, mag3D前座標）
    bm = {a: col(data, "m" + a, n) for a in ("x", "y", "z")}
    base_bm = {a: float(np.nanmean(bm[a][motor_off])) for a in ("x", "y", "z")}
    dmag = {a: bm[a] - base_bm[a] for a in ("x", "y", "z")}
    dmag_norm = np.sqrt(sum(dmag[a] ** 2 for a in ("x", "y", "z")))

    return dict(
        n=n, t=t, y=y, yu=yu, mt=mt, ml=ml, cur=cur, md=md, mr=mr,
        ffg=ffg, ffm=ffm, fes=fes, motor_off=motor_off, motor_on=motor_on,
        base_y=base_y, base_yu=base_yu, err_y=err_y, err_yu=err_yu,
        fdx=col(data, "fdx", n), fdy=col(data, "fdy", n), fdz=col(data, "fdz", n),
        fbx=col(data, "fbx", n), fby=col(data, "fby", n), fns=col(data, "fns", n),
        tmp=col(data, "tmp", n), vb=col(data, "vb", n),
        dmag=dmag, dmag_norm=dmag_norm,
    )


def duty_band(md: np.ndarray) -> list:
    """duty を idle/低/中/高 に分ける（マスクとラベル）。"""
    return [
        (md < 0.05, "idle\n(OFF)"),
        ((md >= 0.05) & (md < 0.35), "低\n0.1–0.3"),
        ((md >= 0.35) & (md < 0.65), "中\n0.4–0.6"),
        (md >= 0.65, "高\n0.7–1.0"),
    ]


# -------------------------------------------------------------- figures -------
def save_fig(fig, out: Path, name: str) -> None:
    fig.tight_layout()
    fig.savefig(out / name, dpi=130, bbox_inches="tight")
    plt.close(fig)


def _motor_spans(ax, t, motor_on):
    """モーターON区間を薄い赤帯で示す（立ち上がり/立ち下がり検出）。"""
    on = motor_on.astype(int)
    edges = np.diff(np.concatenate([[0], on, [0]]))
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0] - 1
    label_done = False
    for s, e in zip(starts, ends):
        if s >= len(t):
            continue
        e = min(e, len(t) - 1)
        ax.axvspan(t[s], t[e], color="#ef4444", alpha=0.08,
                   label=None if label_done else "モーター回転中")
        label_done = True


def fig1_timeseries(d: dict, out: Path) -> None:
    t = d["t"]
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [3, 1]})

    _motor_spans(ax, t, d["motor_on"])
    # 生磁気方位（融合前・最も暴れる）: 欠損点があるので散布で
    mfin = np.isfinite(d["mt"])
    ax.plot(t[mfin], d["mt"][mfin], ".", ms=2.5, color="#fbbf24", alpha=0.5,
            label="mt 生磁気方位（融合前）")
    # 補正前/補正後の融合結果: 連続なので unwrap して線に
    ax.plot(t, unwrap_deg(d["yu"]), "-", lw=1.6, color="#f472b6",
            label="yu 補正前（相補フィルタ）")
    ax.plot(t, unwrap_deg(d["y"]), "-", lw=1.8, color="#2563eb",
            label="y 補正後（FF+EKF）")
    ax.axhline(d["base_y"], ls=":", lw=1, color="#2563eb", alpha=0.6)
    ax.set_ylabel("Yaw [deg]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8, ncol=2)
    ax.set_title("① Yaw時系列: 補正前(yu) は電流で漂い、補正後(y) は基準に張り付く", fontsize=12)

    ax2.plot(t, d["cur"], "-", lw=1.2, color="#16a34a", label="総電流 [A]")
    ax2.set_ylabel("電流 [A]", color="#16a34a")
    ax2.tick_params(axis="y", labelcolor="#16a34a")
    ax2.grid(alpha=0.3)
    axd = ax2.twinx()
    axd.plot(t, d["md"], "-", lw=1.0, color="#6b7280", alpha=0.7, label="duty")
    axd.set_ylabel("duty", color="#6b7280")
    axd.tick_params(axis="y", labelcolor="#6b7280")
    ax2.set_xlabel("時間 [s]")
    save_fig(fig, out, "01_yaw_timeseries.png")


def fig1b_yaw_corrected_detail(d: dict, out: Path) -> None:
    """補正後Yaw y だけを自動スケールで拡大。①では yu/mt に潰されて
    見えない y の細かい推移（duty ステップごとの微小オフセット・ノイズ）を見る。"""
    t = d["t"]
    y = unwrap_deg(d["y"])
    base = float(np.nanmean(y[d["motor_off"]])) if d["motor_off"].any() else float(np.nanmean(y))

    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [3, 1]})

    _motor_spans(ax, t, d["motor_on"])
    ax.axhline(base, ls="--", lw=1, color="#6b7280", label=f"停止時基準 {base:.1f}°")
    ax.plot(t, y, "-", lw=1.3, color="#1d4ed8", label="y 補正後")

    # 自動スケール（外れ値に強い 0.5–99.5%tile に少し余白）
    fin = y[np.isfinite(y)]
    if fin.size:
        lo, hi = np.percentile(fin, 0.5), np.percentile(fin, 99.5)
        pad = max(1.0, (hi - lo) * 0.12)
        ax.set_ylim(lo - pad, hi + pad)
    ax.set_ylabel("補正後Yaw y [deg]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9, ncol=2)

    on = d["motor_on"]
    if on.any():
        dev = np.abs(y[on] - base)
        ax.set_title(f"①' 補正後Yaw y の詳細（y のみ拡大）— 回転中の基準からのずれ "
                     f"平均{np.nanmean(dev):.1f}° / 最大{np.nanmax(dev):.1f}°", fontsize=12)
    else:
        ax.set_title("①' 補正後Yaw y の詳細（y のみ拡大）", fontsize=12)

    ax2.plot(t, d["cur"], "-", lw=1.2, color="#16a34a", label="総電流 [A]")
    ax2.set_ylabel("電流 [A]", color="#16a34a")
    ax2.tick_params(axis="y", labelcolor="#16a34a")
    ax2.grid(alpha=0.3)
    axd = ax2.twinx()
    axd.plot(t, d["md"], "-", lw=1.0, color="#6b7280", alpha=0.7, label="duty")
    axd.set_ylabel("duty", color="#6b7280")
    axd.tick_params(axis="y", labelcolor="#6b7280")
    ax2.set_xlabel("時間 [s]")
    save_fig(fig, out, "01b_yaw_corrected_detail.png")


def fig2_error_vs_current(d: dict, out: Path) -> None:
    cur = d["cur"]
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.scatter(cur, d["err_yu"], s=8, alpha=0.25, color="#f472b6", label="補正前 yu")
    ax.scatter(cur, d["err_y"], s=8, alpha=0.35, color="#2563eb", label="補正後 y")

    # 電流ビンごとの中央値トレンド線
    finite = np.isfinite(cur)
    if finite.any():
        lo, hi = np.nanmin(cur[finite]), np.nanmax(cur[finite])
        edges = np.linspace(lo, hi, 13)
        centers = 0.5 * (edges[:-1] + edges[1:])
        for err, color, lab in ((d["err_yu"], "#db2777", "補正前 中央値"),
                                (d["err_y"], "#1d4ed8", "補正後 中央値")):
            med = []
            for a, b in zip(edges[:-1], edges[1:]):
                sel = finite & (cur >= a) & (cur < b) & np.isfinite(err)
                med.append(np.median(err[sel]) if sel.any() else np.nan)
            ax.plot(centers, med, "-o", ms=4, lw=2, color=color, label=lab)

    ax.set_xlabel("総電流 [A]")
    ax.set_ylabel("|Yaw − 停止時基準|  [deg]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    ax.set_title("② Yaw誤差 vs 電流: 補正前は電流に比例して増大、補正後はほぼ水平", fontsize=12)
    save_fig(fig, out, "02_error_vs_current.png")


def fig3_error_distribution(d: dict, out: Path) -> None:
    bands = duty_band(d["md"])
    fig, ax = plt.subplots(figsize=(11, 6))
    positions, labels = [], []
    for i, (mask, label) in enumerate(bands):
        for j, (err, color) in enumerate(((d["err_yu"], "#f472b6"), (d["err_y"], "#2563eb"))):
            vals = err[mask & np.isfinite(err)]
            if vals.size == 0:
                continue
            pos = i * 3 + j
            bp = ax.boxplot([vals], positions=[pos], widths=0.7, patch_artist=True,
                            showfliers=False)
            for box in bp["boxes"]:
                box.set(facecolor=color, alpha=0.5)
            for med in bp["medians"]:
                med.set(color="black")
        positions.append(i * 3 + 0.5)
        labels.append(label)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels)
    ax.set_ylabel("|Yaw − 停止時基準|  [deg]")
    ax.grid(alpha=0.3, axis="y")
    # 凡例（ダミーパッチ）
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(facecolor="#f472b6", alpha=0.5, label="補正前 yu"),
                       Patch(facecolor="#2563eb", alpha=0.5, label="補正後 y")],
              loc="best", fontsize=9)
    ax.set_title("③ 補正前後のヨー誤差分布（duty帯別・箱ひげ, 外れ値非表示）", fontsize=12)
    save_fig(fig, out, "03_error_distribution.png")


def fig4_deltaB_vs_current(d: dict, meta: dict | None, out: Path) -> None:
    cur = d["cur"]
    comps = [("fdx", "ΔB̂_x", "#2563eb"), ("fdy", "ΔB̂_y", "#16a34a"),
             ("fdz", "ΔB̂_z", "#dc2626")]
    fig, axarr = plt.subplots(1, 2, figsize=(13, 5.5))

    ax = axarr[0]
    affine = (((meta or {}).get("applied_profile") or {}).get("method_a") or {}).get("affine_ref")
    for k, (key, lab, color) in enumerate(comps):
        ax.scatter(cur, d[key], s=8, alpha=0.3, color=color, label=lab)
        if affine and np.isfinite(cur).any():
            xs = np.linspace(np.nanmin(cur), np.nanmax(cur), 50)
            a, b = affine["a"][k], affine["b"][k]
            ax.plot(xs, a * xs + b, "--", lw=1.2, color=color, alpha=0.8)
    ax.set_xlabel("総電流 [A]")
    ax.set_ylabel("適用した ΔB̂ [µT]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    ax.set_title("④a 適用FF補正量 ΔB̂ vs 電流（破線=参考affine a·I+b）", fontsize=11)

    ax2 = axarr[1]
    mag = np.sqrt(d["fdx"] ** 2 + d["fdy"] ** 2 + d["fdz"] ** 2)
    sc = ax2.scatter(cur, mag, s=10, c=d["md"], cmap="viridis", alpha=0.7)
    ax2.set_xlabel("総電流 [A]")
    ax2.set_ylabel("|ΔB̂| [µT]")
    ax2.grid(alpha=0.3)
    fig.colorbar(sc, ax=ax2, shrink=0.7, pad=0.02, label="duty")
    ax2.set_title("④b 補正量の大きさ |ΔB̂|（色=duty）", fontsize=11)
    save_fig(fig, out, "04_deltaB_vs_current.png")


def fig5_raw_disturbance(d: dict, out: Path) -> None:
    cur = d["cur"]
    fig, axarr = plt.subplots(1, 2, figsize=(13, 5.5))
    ax = axarr[0]
    for a, color in (("x", "#2563eb"), ("y", "#16a34a"), ("z", "#dc2626")):
        ax.scatter(cur, d["dmag"][a], s=8, alpha=0.3, color=color, label=f"Δm{a}")
    ax.set_xlabel("総電流 [A]")
    ax.set_ylabel("生磁気の停止時基準からの差分 [µT]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    ax.set_title("⑤a 生磁気ベクトルの外乱 vs 電流（mag3D前座標）", fontsize=11)

    ax2 = axarr[1]
    ax2.scatter(cur, d["dmag_norm"], s=10, color="#7c3aed", alpha=0.5)
    ax2.set_xlabel("総電流 [A]")
    ax2.set_ylabel("|Δ生磁気| [µT]")
    ax2.grid(alpha=0.3)
    ax2.set_title("⑤b 補正が打ち消している外乱の実サイズ（生座標での目安）", fontsize=11)
    save_fig(fig, out, "05_raw_disturbance.png")


def fig6_nis_gates(d: dict, out: Path) -> None:
    t = d["t"]
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [2, 2]})

    fns = d["fns"]
    ax.plot(t, fns, "-", lw=0.9, color="#0ea5e9", label="NIS")
    ax.axhline(5.99, ls="--", color="#f59e0b", lw=1, label="χ²(2) 95%=5.99 (R膨張)")
    ax.axhline(13.8, ls="--", color="#ef4444", lw=1, label="χ²(2) 99.9%=13.8 (棄却)")
    ax.axhline(2.0, ls=":", color="#6b7280", lw=1, label="期待値≈2")
    ax.set_ylabel("NIS")
    top = np.nanpercentile(fns[np.isfinite(fns)], 99) if np.isfinite(fns).any() else 15
    ax.set_ylim(0, max(15, float(top) * 1.2))
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, ncol=2)
    ax.set_title("⑥ EKF健全性: NIS（観測整合度）と ゲート発火", fontsize=12)

    # ゲートラスタ: 各ビットが立っている区間を横帯で
    ffg = d["ffg"]
    for b, (name, _desc, color) in enumerate(GATE_BITS):
        active = ((ffg >> b) & 1).astype(bool)
        ax2.fill_between(t, b, b + 0.8, where=active, color=color, step="mid")
    ax2.set_yticks([b + 0.4 for b in range(len(GATE_BITS))])
    ax2.set_yticklabels([g[0] for g in GATE_BITS], fontsize=8)
    ax2.set_ylim(0, len(GATE_BITS))
    ax2.set_xlabel("時間 [s]")
    ax2.grid(alpha=0.3, axis="x")
    ax2.set_title("ゲート発火（帯 = そのゲートが立っている区間。棄却系はジャイロ滑走）",
                  fontsize=10)
    save_fig(fig, out, "06_nis_gates.png")


def fig7_bm_background(d: dict, out: Path) -> None:
    t = d["t"]
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})

    bm_norm = np.sqrt(d["fbx"] ** 2 + d["fby"] ** 2)
    ax.plot(t, d["fbx"], "-", lw=1.2, color="#2563eb", label="b_mx")
    ax.plot(t, d["fby"], "-", lw=1.2, color="#16a34a", label="b_my")
    ax.plot(t, bm_norm, "-", lw=1.5, color="#7c3aed", label="‖b_m‖")
    ax.axhline(20, ls="--", color="#ef4444", lw=1, label="凍結閾値 20µT")
    ax.set_ylabel("磁気バイアス b_m [µT]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8, ncol=2)
    ax.set_title("⑦ EKF磁気バイアス b_m（ドリフト追従）と背景要因", fontsize=12)

    ax2.plot(t, d["tmp"], "-", lw=1.2, color="#f97316", label="IMU温度 [°C]")
    ax2.set_ylabel("温度 [°C]", color="#f97316")
    ax2.tick_params(axis="y", labelcolor="#f97316")
    ax2.grid(alpha=0.3)
    axv = ax2.twinx()
    axv.plot(t, d["vb"], "-", lw=1.2, color="#0ea5e9", label="電圧 [V]")
    axv.set_ylabel("電圧 [V]", color="#0ea5e9")
    axv.tick_params(axis="y", labelcolor="#0ea5e9")
    ax2.set_xlabel("時間 [s]")
    save_fig(fig, out, "07_bm_background.png")


# -------------------------------------------------------------- summary -------
def compute_summary(d: dict, meta: dict | None) -> dict:
    on = d["motor_on"]
    err_y_on = d["err_y"][on & np.isfinite(d["err_y"])]
    err_yu_on = d["err_yu"][on & np.isfinite(d["err_yu"])]
    cur = d["cur"][np.isfinite(d["cur"])]

    def _rms(a):
        return float(np.sqrt(np.mean(a ** 2))) if a.size else math.nan

    peak_yu = float(np.max(err_yu_on)) if err_yu_on.size else math.nan
    peak_y = float(np.max(err_y_on)) if err_y_on.size else math.nan
    reduction = (1 - peak_y / peak_yu) * 100 if peak_yu and np.isfinite(peak_yu) and peak_yu > 0 else math.nan
    rms_yu, rms_y = _rms(err_yu_on), _rms(err_y_on)
    reduction_rms = (1 - rms_y / rms_yu) * 100 if rms_yu and np.isfinite(rms_yu) and rms_yu > 0 else math.nan

    # ゲート発火率（EKF更新が走ったサンプル母数 = fresh磁気に近いが、ここでは全行比）
    ffg = d["ffg"]
    gate_rate = {}
    for b, (name, _desc, _c) in enumerate(GATE_BITS):
        cnt = int((((ffg >> b) & 1)).sum())
        gate_rate[name] = (cnt, 100.0 * cnt / d["n"] if d["n"] else 0.0)

    bm_norm = np.sqrt(d["fbx"] ** 2 + d["fby"] ** 2)
    ffm = int(np.bincount(d["ffm"][d["ffm"] >= 0]).argmax()) if d["n"] else 0
    fes = int(np.bincount(d["fes"][d["fes"] >= 0]).argmax()) if d["n"] else 0

    return dict(
        duration=float(d["t"][-1]) if d["n"] else 0.0,
        samples=d["n"],
        max_current=float(np.max(cur)) if cur.size else math.nan,
        peak_err_uncorr=peak_yu, peak_err_corr=peak_y, reduction_pct=reduction,
        rms_err_uncorr=rms_yu, rms_err_corr=rms_y, reduction_rms_pct=reduction_rms,
        bm_max=float(np.nanmax(bm_norm)) if np.isfinite(bm_norm).any() else math.nan,
        gate_rate=gate_rate, ffm=ffm, fes=fes,
        profile=(((meta or {}).get("applied_profile") or {}).get("name")
                 or (meta or {}).get("applied_state", {}).get("name") or "(不明)"),
        profile_memo=(((meta or {}).get("applied_profile") or {}).get("memo")
                      or (meta or {}).get("memo") or ""),
    )


def print_summary(s: dict) -> None:
    mode = {0: "off", 1: "A(総電流LUT)", 2: "B(LUT+差動)"}.get(s["ffm"], str(s["ffm"]))
    est = {0: "相補フィルタ", 1: "EKF"}.get(s["fes"], str(s["fes"]))
    print("\n" + "=" * 60)
    print("  Yaw補正実験サマリ")
    print("=" * 60)
    print(f"  適用プロファイル : {s['profile']}  [{s['profile_memo']}]")
    print(f"  記録長 / サンプル: {s['duration']:.1f} s / {s['samples']} 行")
    print(f"  FF方式 / 推定器  : {mode} / {est}")
    print(f"  最大電流         : {s['max_current']:.2f} A")
    print("  ---- ヨー誤差（停止時基準からのずれ, モーター回転中）----")
    print(f"    補正前 yu  ピーク {s['peak_err_uncorr']:.1f}°  RMS {s['rms_err_uncorr']:.1f}°")
    print(f"    補正後 y   ピーク {s['peak_err_corr']:.1f}°  RMS {s['rms_err_corr']:.1f}°")
    if np.isfinite(s["reduction_rms_pct"]):
        print(f"    RMS誤差 削減率  : {s['reduction_rms_pct']:.1f} %  "
              f"(ピーク基準 {s['reduction_pct']:.1f} %)")
    print(f"  b_m 最大         : {s['bm_max']:.1f} µT")
    print("  ---- ゲート発火（全行比）----")
    for name, (cnt, pct) in s["gate_rate"].items():
        if cnt:
            print(f"    {name:12s}: {cnt} 行 ({pct:.1f}%)")
    print("=" * 60 + "\n")


def fig0_summary_panel(s: dict, out: Path) -> None:
    mode = {0: "off", 1: "A (総電流LUT)", 2: "B (LUT+差動)"}.get(s["ffm"], str(s["ffm"]))
    est = {0: "相補フィルタ", 1: "EKF"}.get(s["fes"], str(s["fes"]))
    lines = [
        ("適用プロファイル", f"{s['profile']}"),
        ("メモ", f"{s['profile_memo']}"),
        ("記録長 / サンプル", f"{s['duration']:.1f} s  /  {s['samples']} 行"),
        ("FF方式 / 推定器", f"{mode}  /  {est}"),
        ("最大電流", f"{s['max_current']:.2f} A"),
        ("補正前 yu 誤差", f"RMS {s['rms_err_uncorr']:.1f}°   (ピーク {s['peak_err_uncorr']:.1f}°)"),
        ("補正後 y 誤差", f"RMS {s['rms_err_corr']:.1f}°   (ピーク {s['peak_err_corr']:.1f}°)"),
        ("誤差 削減率",
         (f"RMS {s['reduction_rms_pct']:.1f} %   (ピーク基準 {s['reduction_pct']:.1f} %)"
          if np.isfinite(s["reduction_rms_pct"]) else "—")),
        ("b_m 最大", f"{s['bm_max']:.1f} µT"),
    ]
    gate_lines = [f"{name}: {cnt}行 ({pct:.1f}%)"
                  for name, (cnt, pct) in s["gate_rate"].items() if cnt] or ["（発火なし）"]

    fig, ax = plt.subplots(figsize=(10, 6.5))
    ax.axis("off")
    y = 0.96
    ax.text(0.02, y, "⓪ Yaw補正実験 サマリ", fontsize=15, fontweight="bold",
            transform=ax.transAxes)
    y -= 0.08
    for k, v in lines:
        ax.text(0.04, y, k, fontsize=11, transform=ax.transAxes, color="#374151")
        ax.text(0.42, y, v, fontsize=11, transform=ax.transAxes, fontweight="bold")
        y -= 0.072
    y -= 0.02
    ax.text(0.04, y, "ゲート発火（全行比）", fontsize=11, transform=ax.transAxes,
            color="#374151", fontweight="bold")
    y -= 0.06
    for gl in gate_lines:
        ax.text(0.06, y, gl, fontsize=10, transform=ax.transAxes)
        y -= 0.05
    # 効果の一言
    if np.isfinite(s["reduction_rms_pct"]):
        ax.text(0.02, 0.02,
                f"→ 電流ノイズ由来のヨー誤差を RMS {s['rms_err_uncorr']:.0f}° から "
                f"{s['rms_err_corr']:.1f}° へ（{s['reduction_rms_pct']:.0f}% 削減）",
                fontsize=12, transform=ax.transAxes, color="#1d4ed8", fontweight="bold")
    save_fig(fig, out, "00_summary.png")


# ---------------------------------------------------------------- select ------
def yaweval_dir() -> Path:
    return Path(__file__).resolve().parent.parent / "pc_server" / "yaw_eval_results"


def list_logs() -> list[Path]:
    d = yaweval_dir()
    if not d.is_dir():
        return []
    return sorted(d.glob(LOG_GLOB), key=lambda p: p.stat().st_mtime, reverse=True)


def _fmt_size(n: int) -> str:
    return f"{n / 1024:.0f} KB" if n < 1024 * 1024 else f"{n / 1024 / 1024:.1f} MB"


def choose_log(cands: list[Path]) -> Path | None:
    print(f"\n{yaweval_dir()} の yawログから、グラフ化するものを選んでください:\n")
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
    stem = path.stem
    if stem.endswith("_log"):
        stem = stem[: -len("_log")]
    return Path(__file__).resolve().parent / "graphs" / f"yaweval_{stem}"


def main() -> None:
    ap = argparse.ArgumentParser(
        description="StampFly Yaw補正実験ログ（yaw_eval_results）の可視化（7図＋サマリ）")
    ap.add_argument("log", nargs="?",
                    help="yawlog_*_log.csv パス（省略時は yaw_eval_results から対話選択）")
    ap.add_argument("-o", "--out",
                    help="出力ディレクトリ（省略時は data_analysis/graphs/yaweval_<stem>/）")
    args = ap.parse_args()

    if args.log:
        path = Path(args.log)
    else:
        cands = list_logs()
        if not cands:
            sys.exit(f"yawログが見つかりません: {yaweval_dir()} に {LOG_GLOB} がありません。")
        path = choose_log(cands)
        if path is None:
            sys.exit("中止しました（CSV が選択されていません）。")
    if not path.is_file():
        sys.exit(f"yawログが見つかりません: {path}")

    out = Path(args.out) if args.out else default_out_dir(path)
    out.mkdir(parents=True, exist_ok=True)

    data = load_log(path)
    d = derive(data)
    meta = load_meta(path)

    print(f"入力: {path}\n出力: {out}")
    if not np.isfinite(d["base_y"]) or not np.isfinite(d["base_yu"]):
        print("警告: モーター停止(OFF)区間が見つからず基準ヨーを決められません。"
              "誤差評価（②③）が不正確になります。")
    if (d["fes"] == 1).sum() == 0:
        print("注意: このログは EKF(est=1) 区間がありません。NIS/b_m/ゲート図（⑥⑦）は"
              "リファレンス相補フィルタ動作のため値が入りません。")

    fig1_timeseries(d, out)
    fig1b_yaw_corrected_detail(d, out)
    fig2_error_vs_current(d, out)
    fig3_error_distribution(d, out)
    fig4_deltaB_vs_current(d, meta, out)
    fig5_raw_disturbance(d, out)
    fig6_nis_gates(d, out)
    fig7_bm_background(d, out)

    s = compute_summary(d, meta)
    fig0_summary_panel(s, out)
    print_summary(s)
    print(f"図9枚（①・①'・②〜⑦＋サマリ）を出力しました → {out}")


if __name__ == "__main__":
    main()
