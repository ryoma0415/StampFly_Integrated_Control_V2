#!/usr/bin/env python3
"""StampFly Experiment 計測ログ（explog CSV）のグラフ化.

pc_server の Experiment モードが出力する
pc_server/data/exp_logs/explog_<YYYYMMDD_HHMMSS>.csv（≈25Hz）を読み、
ヨー推定3系統の比較・FF補正の説明力・EKF健全性などを PNG 6枚 + summary.txt に
まとめる。必要な列が無い / 全欠損の図は自動でスキップする。

出力図:
    01_yaw_comparison.png   ヨー3系統（±180°ラップ絶対値）+ 磁気方位(FF補正後) + duty
    02_yaw_error.png        基準に対する各系統の誤差時系列（RMS/最大を凡例に）
    03_current_duty.png     電流 / バッテリ電圧 / duty
    04_mag_ff.png           校正済み磁場変化 Δb_cal と FF推定外乱 db_hat の重畳
    05_ekf_diagnostics.png  NIS（閾値 2.0 / 5.99 / 13.8）+ ffg 8ゲートのタイムライン
    06_ff_status.png        ff_status（ff_mode + フラグビット）のタイムライン

使い方:
    python plot_explog.py                    # ../pc_server/data/exp_logs/ から対話選択
    python plot_explog.py explog_xxx.csv [-o 出力ディレクトリ]
    引数を省略すると explog CSV を新しい順に一覧表示し、番号で1つ選ぶ
    （Enter=最新 / q=中止）。出力先は既定で data_analysis/graphs/explog_<stamp>/。

同じ stamp の explog_<stamp>_meta.json があれば、ff_state
（プロファイル名 / ff_mode / est_mode）を図タイトルとサマリに表示する。

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

# 日本語フォント（macOS優先）。見つからなければ既定フォントのまま（英字は問題なし）。
_INSTALLED = {f.name for f in fm.fontManager.ttflist}
for _jp in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic", "Yu Gothic",
            "Noto Sans CJK JP", "IPAexGothic", "Apple SD Gothic Neo", "Arial Unicode MS"):
    if _jp in _INSTALLED:
        plt.rcParams["font.family"] = _jp
        break
plt.rcParams["axes.unicode_minus"] = False

# ダークテーマ
plt.rcParams.update({
    "figure.facecolor": "#0f1117",
    "savefig.facecolor": "#0f1117",
    "axes.facecolor": "#151a23",
    "axes.edgecolor": "#3a4152",
    "axes.labelcolor": "#e6e9ef",
    "axes.titlecolor": "#e6e9ef",
    "text.color": "#e6e9ef",
    "xtick.color": "#aab2c0",
    "ytick.color": "#aab2c0",
    "grid.color": "#3a4152",
    "legend.facecolor": "#1c2230",
    "legend.edgecolor": "#3a4152",
    "legend.labelcolor": "#e6e9ef",
})

EXPLOG_GLOB = "explog_*.csv"

# ヨー3系統（列名, 表示ラベル, 色）
YAW_STREAMS = [
    ("yaw_gyro_int_deg", "ジャイロ積分", "#f59e0b"),
    ("yaw_est_deg", "EKF推定 (FF+EKF)", "#38bdf8"),
    ("yaw_madgwick_deg", "Madgwick", "#f472b6"),
]

# ffg（EKFゲート状態ビット, ファーム側 yaw_estimator_kf と一致）
GATE_BITS = [
    ("R膨張(soft)", "#fbbf24"),
    ("NIS棄却", "#ef4444"),
    ("norm棄却", "#f97316"),
    ("z棄却", "#a855f7"),
    ("tilt>25°", "#94a3b8"),
    ("b_m凍結", "#dc2626"),
    ("ドリフト警告", "#0ea5e9"),
    ("再捕捉中", "#4ade80"),
]

# ff_status: 下位2bit = ff_mode(0=off,1=A,2=B)、bit2-6 = フラグ
FF_MODE_MASK = 0x03
FF_MODE_NAMES = {0: "off", 1: "A", 2: "B", 3: "?(3)"}
FF_STATUS_FLAGS = [  # (bit, ラベル, 色)
    (2, "est=EKF", "#38bdf8"),
    (3, "アンカー有効", "#4ade80"),
    (4, "FF係数ロード済", "#a78bfa"),
    (5, "Yaw制御アクティブ", "#fb923c"),
    (6, "磁気fresh", "#f472b6"),
]

NIS_EXPECT = 2.0
NIS_SOFT = 5.99
NIS_REJECT = 13.8


# ---------------------------------------------------------------- load --------
def _f(value) -> float:
    """空欄/欠損は NaN。"""
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def load_explog(path: Path) -> dict:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    if not rows:
        sys.exit(f"空のCSVです: {path}")
    out: dict = {}
    for key in rows[0].keys():
        if key is None:
            continue
        if key == "motors":  # モーター名の文字列列（"FL+FR+RL+RR" 等）
            out[key] = np.array([r.get(key) or "" for r in rows], dtype=object)
            continue
        out[key] = np.array([_f(r.get(key, "")) for r in rows], dtype=float)
    out["_n"] = len(rows)
    return out


def col(data: dict, name: str) -> np.ndarray:
    """列を取得。無ければ全 NaN（自動スキップ用）。"""
    v = data.get(name)
    if v is None or v.dtype == object:
        return np.full(data["_n"], np.nan)
    return v


def usable(data: dict, *names: str) -> bool:
    """全列が存在し、かつ有限値を1つ以上持つか。"""
    return all(np.isfinite(col(data, n)).any() for n in names)


def meta_path_for(log: Path) -> Path:
    return log.with_name(log.stem + "_meta.json")


def load_meta(log: Path) -> dict | None:
    mp = meta_path_for(log)
    if not mp.is_file():
        return None
    try:
        return json.loads(mp.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        print(f"meta JSON の読み込みに失敗（無視して続行）: {mp} ({e})")
        return None


def ff_state_line(meta: dict | None) -> str:
    """meta の ff_state を1行サマリに（無ければ空文字）。"""
    ff = (meta or {}).get("ff_state") or {}
    if not ff:
        return ""
    parts = []
    if ff.get("name"):
        parts.append(f"プロファイル: {ff['name']}")
    if ff.get("ff_mode") is not None:
        m = ff["ff_mode"]
        parts.append(f"ff_mode={FF_MODE_NAMES.get(m, m) if isinstance(m, int) else m}")
    if ff.get("est_mode") is not None:
        parts.append(f"est_mode={ff['est_mode']}")
    return " / ".join(parts)


# --------------------------------------------------------------- angles -------
def wrap180(deg: np.ndarray) -> np.ndarray:
    """角度差を [-180, 180) に畳む。"""
    return (deg + 180.0) % 360.0 - 180.0


def unwrap_deg(a: np.ndarray) -> np.ndarray:
    """NaN を保ったまま有限値だけを unwrap した deg 系列を返す。"""
    out = np.full(a.shape, np.nan)
    fin = np.isfinite(a)
    if fin.sum() >= 2:
        out[fin] = np.rad2deg(np.unwrap(np.deg2rad(a[fin])))
    elif fin.any():
        out[fin] = a[fin]
    return out


def start_offset(a: np.ndarray, n: int = 10) -> float:
    """開始 n サンプル（有限値のみ）の平均。相対表示の基準。"""
    fin = a[np.isfinite(a)]
    if fin.size == 0:
        return math.nan
    return float(np.mean(fin[:n]))


def relative(a: np.ndarray) -> tuple[np.ndarray, float]:
    """unwrap して開始時点オフセットを引いた相対ヨー系列と、そのオフセット。"""
    u = unwrap_deg(a)
    off = start_offset(u)
    return u - off, off


def circ_mean_deg(a: np.ndarray) -> float:
    """円周平均 [deg]。±180 付近でも破綻しない平均角。"""
    fin = a[np.isfinite(a)]
    if fin.size == 0:
        return math.nan
    r = np.deg2rad(fin)
    return math.degrees(math.atan2(float(np.mean(np.sin(r))),
                                   float(np.mean(np.cos(r)))))


def wrapped_plot_series(t: np.ndarray, deg: np.ndarray,
                        jump_deg: float = 180.0) -> tuple[np.ndarray, np.ndarray]:
    """±180° ラップ表示用の (t, y)。

    wrap180 した系列の隣接差が jump_deg を超える箇所（ラップ跨ぎ）に
    NaN 点を挿入し、プロット時に縦線が入らないよう線を切る。
    """
    w = wrap180(deg)
    if len(w) < 2:
        return t, w
    d = np.abs(np.diff(w))
    jumps = np.where(np.isfinite(d) & (d > jump_deg))[0]
    if jumps.size == 0:
        return t, w
    t2 = np.insert(t.astype(float), jumps + 1, (t[jumps] + t[jumps + 1]) / 2.0)
    y2 = np.insert(w, jumps + 1, np.nan)
    return t2, y2


def tilt_deg_series(data: dict) -> np.ndarray:
    """チルト角 [deg] = acos(cos(roll)·cos(pitch))。roll/pitch 欠損は NaN。"""
    r = np.deg2rad(col(data, "roll_deg"))
    p = np.deg2rad(col(data, "pitch_deg"))
    c = np.clip(np.cos(r) * np.cos(p), -1.0, 1.0)
    return np.degrees(np.arccos(c))


def mag_yaw_deg(data: dict, subtract_ff: bool, tilt_mask_deg: float = 15.0) -> np.ndarray:
    """磁気方位 [deg]（ファーム yaw_estimation と同一規約）。

    ファーム実装（firmware_stampfly/src/yaw_estimation/）:
      - FF補正は水平2軸のみ: b_corr = (bx_cal − db_hat_x, by_cal − db_hat_y, bz_cal)
      - levelMagVectorBody(angle_utils.hpp): 独自符号のチルト補償
            lx = mx·cp + mz·sp
            ly = mx·sr·sp + my·cr + mz·sr·cp
      - yaw_mag = atan2(ly, lx)  （yaw_estimator.cpp:133。ψ とともに増える CCW 規約）
    tilt > tilt_mask_deg の区間は NaN でマスク（レベル化の信頼性低下）。
    """
    bx = col(data, "bx_cal").copy()
    by = col(data, "by_cal").copy()
    bz = col(data, "bz_cal")
    if subtract_ff:
        bx = bx - np.nan_to_num(col(data, "db_hat_x_ut"))
        by = by - np.nan_to_num(col(data, "db_hat_y_ut"))
    r = np.deg2rad(col(data, "roll_deg"))
    p = np.deg2rad(col(data, "pitch_deg"))
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    lx = bx * cp + bz * sp
    ly = bx * sr * sp + by * cr + bz * sr * cp
    yaw = np.degrees(np.arctan2(ly, lx))
    tilt = tilt_deg_series(data)
    yaw[np.isfinite(tilt) & (tilt > tilt_mask_deg)] = np.nan
    return yaw


# --------------------------------------------------------------- derive -------
def time_axis(data: dict) -> np.ndarray:
    """時間軸 [s]（開始=0）。exp_elapsed_ms 優先、無ければ t_s、最後は行番号/25Hz。"""
    el = col(data, "exp_elapsed_ms")
    if np.isfinite(el).any():
        return (el - np.nanmin(el)) / 1000.0
    ts = col(data, "t_s")
    if np.isfinite(ts).any():
        return ts - np.nanmin(ts)
    return np.arange(data["_n"]) / 25.0


def motor_on_mask(data: dict) -> np.ndarray:
    """モーターON区間（duty_cmd > 0、mask 列があれば併用）。"""
    duty = np.nan_to_num(col(data, "duty_cmd"))
    on = duty > 1e-4
    mm = col(data, "motors_mask")
    if np.isfinite(mm).any():
        on &= np.nan_to_num(mm) > 0
    return on


def sample_rate(t: np.ndarray) -> float:
    dt = np.diff(t[np.isfinite(t)])
    dt = dt[dt > 0]
    return float(1.0 / np.median(dt)) if dt.size else math.nan


# -------------------------------------------------------------- figures -------
def save_fig(fig, out: Path, name: str) -> None:
    fig.tight_layout()
    fig.savefig(out / name, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  出力: {name}")


def _motor_spans(ax, t: np.ndarray, on: np.ndarray) -> None:
    """モーターON区間を薄い赤帯で示す。"""
    onl = on.astype(int)
    edges = np.diff(np.concatenate([[0], onl, [0]]))
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0] - 1
    label_done = False
    for s, e in zip(starts, ends):
        if s >= len(t):
            continue
        e = min(e, len(t) - 1)
        ax.axvspan(t[s], t[e], color="#ef4444", alpha=0.10,
                   label=None if label_done else "モーターON")
        label_done = True


def _duty_panel(ax, t: np.ndarray, data: dict, on: np.ndarray) -> None:
    """下段共通: duty_cmd + ON区間帯。"""
    _motor_spans(ax, t, on)
    ax.plot(t, col(data, "duty_cmd"), "-", lw=1.2, color="#aab2c0", label="duty_cmd")
    ax.set_ylabel("duty")
    ax.set_xlabel("時間 [s]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)


def fig01_yaw_comparison(data: dict, t: np.ndarray, on: np.ndarray,
                         out: Path, tsuffix: str) -> bool:
    streams = [(k, lab, c) for k, lab, c in YAW_STREAMS if usable(data, k)]
    if not streams:
        print("  スキップ: 01_yaw_comparison（ヨー列が1つも無い/全欠損）")
        return False
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7.5), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    _motor_spans(ax, t, on)
    for key, lab, color in streams:
        tw, yw = wrapped_plot_series(t, col(data, key))
        ax.plot(tw, yw, "-", lw=1.5, color=color, label=f"{lab} ({key})")

    # 第4トレース: FF補正後の磁気方位（ファーム規約, 開始10サンプルで EKF に位置合わせ）
    if usable(data, "bx_cal", "by_cal", "bz_cal") and usable(data, "yaw_est_deg"):
        ym = mag_yaw_deg(data, subtract_ff=True, tilt_mask_deg=15.0)
        ye = col(data, "yaw_est_deg")
        both = np.isfinite(ym) & np.isfinite(ye)
        idx = np.where(both)[0][:10]
        if idx.size:
            off = circ_mean_deg(wrap180(ye[idx] - ym[idx]))
            tw, yw = wrapped_plot_series(t, ym + off)
            ax.plot(tw, yw, ":", lw=1.0, color="#4ade80", alpha=0.55,
                    label="磁気方位(FF補正後, 開始点合わせ) [tilt>15°マスク]")

    yr = col(data, "yaw_ref_deg")
    if np.isfinite(yr).any():
        tw, yw = wrapped_plot_series(t, yr)
        ax.plot(tw, yw, "--", lw=1.0, color="#e6e9ef", alpha=0.6, label="目標 yaw_ref")
    ax.axhline(0, ls=":", lw=0.8, color="#6b7280")
    ax.set_ylim(-190, 190)
    ax.set_yticks(np.arange(-180, 181, 90))
    ax.set_ylabel("Yaw [deg]（±180° ラップ・絶対値）")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    ax.set_title("① ヨー3系統の比較（±180° ラップ表示・UIと同じ絶対値）" + tsuffix,
                 fontsize=12)
    _duty_panel(ax2, t, data, on)
    save_fig(fig, out, "01_yaw_comparison.png")
    return True


def fig02_yaw_error(data: dict, t: np.ndarray, on: np.ndarray,
                    out: Path, tsuffix: str) -> bool:
    streams = [(k, lab, c) for k, lab, c in YAW_STREAMS if usable(data, k)]
    if not streams:
        print("  スキップ: 02_yaw_error（ヨー列が1つも無い/全欠損）")
        return False
    has_gyro = usable(data, "yaw_gyro_int_deg")
    nrows = 2 if has_gyro and len(streams) >= 2 else 1
    fig, axes = plt.subplots(nrows, 1, figsize=(13, 4.0 * nrows), sharex=True,
                             squeeze=False)
    axes = axes[:, 0]

    def _stats_label(lab: str, err: np.ndarray) -> str:
        fin = err[np.isfinite(err)]
        if fin.size == 0:
            return lab
        rms = float(np.sqrt(np.mean(fin ** 2)))
        return f"{lab}  RMS {rms:.2f}° / 最大 {np.max(np.abs(fin)):.2f}°"

    # 上段: 各系統の「自分の開始10サンプル平均」からのずれ（＝ドリフト）
    ax = axes[0]
    _motor_spans(ax, t, on)
    for key, lab, color in streams:
        rel, _ = relative(col(data, key))
        ax.plot(t, rel, "-", lw=1.3, color=color, label=_stats_label(lab, rel))
    ax.axhline(0, ls=":", lw=0.8, color="#6b7280")
    ax.set_ylabel("誤差 [deg]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    ax.set_title("② ヨー誤差: 上段 = 開始10サンプル平均基準（各系統のドリフト）" + tsuffix,
                 fontsize=12)

    # 下段: ジャイロ積分を基準にした差（磁気系統の相対誤差）
    if nrows == 2:
        ax2 = axes[1]
        _motor_spans(ax2, t, on)
        ref, _ = relative(col(data, "yaw_gyro_int_deg"))
        for key, lab, color in streams:
            if key == "yaw_gyro_int_deg":
                continue
            rel, _ = relative(col(data, key))
            err = wrap180(rel - ref)
            ax2.plot(t, err, "-", lw=1.3, color=color, label=_stats_label(lab, err))
        ax2.axhline(0, ls=":", lw=0.8, color="#6b7280")
        ax2.set_ylabel("誤差 [deg]")
        ax2.grid(alpha=0.3)
        ax2.legend(loc="best", fontsize=8)
        ax2.set_title("下段 = ジャイロ積分基準（短時間はジャイロ積分が真値に近い）",
                      fontsize=10)
    axes[-1].set_xlabel("時間 [s]")
    save_fig(fig, out, "02_yaw_error.png")
    return True


def fig03_current_duty(data: dict, t: np.ndarray, on: np.ndarray,
                       out: Path, tsuffix: str) -> bool:
    if not (usable(data, "current_a") or usable(data, "vbat_v")
            or usable(data, "duty_cmd")):
        print("  スキップ: 03_current_duty（current_a/vbat_v/duty_cmd が全て無い/全欠損）")
        return False
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    _motor_spans(ax, t, on)
    ax.plot(t, col(data, "current_a"), "-", lw=1.3, color="#4ade80", label="総電流 [A]")
    ax.set_ylabel("電流 [A]", color="#4ade80")
    ax.tick_params(axis="y", labelcolor="#4ade80")
    ax.grid(alpha=0.3)
    axv = ax.twinx()
    axv.plot(t, col(data, "vbat_v"), "-", lw=1.2, color="#38bdf8", label="vbat [V]")
    axv.set_ylabel("バッテリ電圧 [V]", color="#38bdf8")
    axv.tick_params(axis="y", labelcolor="#38bdf8")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = axv.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="best", fontsize=8)
    ax.set_title("③ 電流・バッテリ電圧・duty" + tsuffix, fontsize=12)
    _duty_panel(ax2, t, data, on)
    save_fig(fig, out, "03_current_duty.png")
    return True


def fig04_mag_ff(data: dict, t: np.ndarray, on: np.ndarray,
                 out: Path, tsuffix: str) -> bool:
    if not usable(data, "bx_cal", "by_cal", "bz_cal"):
        print("  スキップ: 04_mag_ff（b*_cal が無い/全欠損）")
        return False
    has_ff = usable(data, "db_hat_x_ut") or usable(data, "db_hat_y_ut")
    axes_spec = [
        ("x", col(data, "bx_cal"), col(data, "db_hat_x_ut"), "#60a5fa"),
        ("y", col(data, "by_cal"), col(data, "db_hat_y_ut"), "#4ade80"),
        ("z", col(data, "bz_cal"), None, "#f87171"),  # FF は水平2軸のみ
    ]
    fig, axarr = plt.subplots(3, 1, figsize=(13, 9.5), sharex=True)
    for ax, (a, bcal, dbh, color) in zip(axarr, axes_spec):
        _motor_spans(ax, t, on)
        base = start_offset(bcal)
        db = bcal - base
        ax.plot(t, db, "-", lw=1.3, color=color,
                label=f"Δb{a}_cal（開始基準 {base:.1f}µT からの変化）")
        if dbh is not None and np.isfinite(dbh).any():
            ax.plot(t, dbh, "--", lw=1.3, color="#e6e9ef", alpha=0.85,
                    label=f"db_hat_{a}（FF推定外乱）")
            resid = db - dbh
            ax.plot(t, resid, "-", lw=1.0, color="#a78bfa", alpha=0.8,
                    label=f"残差 Δb{a} − db_hat_{a}")
        elif a != "z":
            ax.text(0.01, 0.05, "db_hat 列なし/全欠損", transform=ax.transAxes,
                    fontsize=8, color="#aab2c0")
        ax.axhline(0, ls=":", lw=0.8, color="#6b7280")
        ax.set_ylabel(f"B_{a} [µT]")
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    axarr[-1].set_xlabel("時間 [s]")
    ttl = "④ 磁場変化 Δb_cal と FF補正 db_hat（一致するほど FF が外乱を説明）"
    if not has_ff:
        ttl = "④ 磁場変化 Δb_cal（db_hat 無し）"
    axarr[0].set_title(ttl + tsuffix, fontsize=12)
    save_fig(fig, out, "04_mag_ff.png")
    return True


def fig05_ekf_diagnostics(data: dict, t: np.ndarray, on: np.ndarray,
                          out: Path, tsuffix: str) -> bool:
    has_nis = usable(data, "nis")
    has_ffg = usable(data, "ffg")
    if not (has_nis or has_ffg):
        print("  スキップ: 05_ekf_diagnostics（nis/ffg が無い/全欠損）")
        return False
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [2, 2]})

    if has_nis:
        nis = col(data, "nis")
        _motor_spans(ax, t, on)
        ax.plot(t, nis, "-", lw=0.9, color="#0ea5e9", label="NIS")
        ax.axhline(NIS_SOFT, ls="--", color="#f59e0b", lw=1,
                   label=f"χ²(2) 95%={NIS_SOFT}（R膨張）")
        ax.axhline(NIS_REJECT, ls="--", color="#ef4444", lw=1,
                   label=f"χ²(2) 99.9%={NIS_REJECT}（棄却）")
        ax.axhline(NIS_EXPECT, ls=":", color="#aab2c0", lw=1,
                   label=f"期待値≈{NIS_EXPECT}")
        fin = nis[np.isfinite(nis)]
        top = np.percentile(fin, 99) if fin.size else 15.0
        ax.set_ylim(0, max(15.0, float(top) * 1.2))
        ax.set_ylabel("NIS")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8, ncol=2)
    else:
        ax.text(0.5, 0.5, "nis 列なし/全欠損", ha="center", va="center",
                transform=ax.transAxes, color="#aab2c0")
    ax.set_title("⑤ EKF健全性: NIS と ffg 8ゲート発火タイムライン" + tsuffix, fontsize=12)

    if has_ffg:
        ffg = np.nan_to_num(col(data, "ffg")).astype(int)
        for b, (name, color) in enumerate(GATE_BITS):
            active = ((ffg >> b) & 1).astype(bool)
            ax2.fill_between(t, b + 0.1, b + 0.9, where=active, color=color, step="mid")
        ax2.set_yticks([b + 0.5 for b in range(len(GATE_BITS))])
        ax2.set_yticklabels([g[0] for g in GATE_BITS], fontsize=8)
        ax2.set_ylim(0, len(GATE_BITS))
        ax2.grid(alpha=0.3, axis="x")
        ax2.set_title("ゲート発火（帯 = そのゲートが立っている区間）", fontsize=10)
    else:
        ax2.text(0.5, 0.5, "ffg 列なし/全欠損", ha="center", va="center",
                 transform=ax2.transAxes, color="#aab2c0")
    ax2.set_xlabel("時間 [s]")
    save_fig(fig, out, "05_ekf_diagnostics.png")
    return True


def fig06_ff_status(data: dict, t: np.ndarray, on: np.ndarray,
                    out: Path, tsuffix: str) -> bool:
    if not usable(data, "ff_status"):
        print("  スキップ: 06_ff_status（ff_status が無い/全欠損）")
        return False
    st = np.nan_to_num(col(data, "ff_status")).astype(int)
    mode = st & FF_MODE_MASK
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [1, 2]})

    _motor_spans(ax, t, on)
    ax.step(t, mode, where="post", lw=1.5, color="#38bdf8")
    ax.set_yticks(sorted(FF_MODE_NAMES.keys()))
    ax.set_yticklabels([f"{k}: {v}" for k, v in sorted(FF_MODE_NAMES.items())],
                       fontsize=8)
    ax.set_ylim(-0.4, 3.4)
    ax.set_ylabel("ff_mode")
    ax.grid(alpha=0.3)
    ax.set_title("⑥ ff_status: ff_mode（下位2bit）と フラグビット（bit2-6）" + tsuffix,
                 fontsize=12)

    for i, (bit, name, color) in enumerate(FF_STATUS_FLAGS):
        active = ((st >> bit) & 1).astype(bool)
        ax2.fill_between(t, i + 0.1, i + 0.9, where=active, color=color, step="mid")
    ax2.set_yticks([i + 0.5 for i in range(len(FF_STATUS_FLAGS))])
    ax2.set_yticklabels([f"bit{b}: {n}" for b, n, _ in FF_STATUS_FLAGS], fontsize=8)
    ax2.set_ylim(0, len(FF_STATUS_FLAGS))
    ax2.grid(alpha=0.3, axis="x")
    ax2.set_xlabel("時間 [s]")
    save_fig(fig, out, "06_ff_status.png")
    return True


# -------------------------------------------------------------- summary -------
STILL_GYRO_THRESH_RAD_S = 0.05   # |r| < 0.05 rad/s (≈2.9°/s) を静止とみなす
STILL_MIN_SEG_S = 3.0            # ドリフトフィットに使う静止区間の最短長
STILL_ENOUGH_S = 10.0            # これ未満なら「静止区間不足のため参考値」


def stationary_mask(data: dict, on: np.ndarray) -> np.ndarray:
    """静止区間: モーターOFF かつ |r_rad_s| < 閾値（r 欠損時は OFF のみ）。"""
    m = ~on
    r = col(data, "r_rad_s")
    if np.isfinite(r).any():
        m = m & np.isfinite(r) & (np.abs(r) < STILL_GYRO_THRESH_RAD_S)
    return m


def contiguous_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """True 連続区間の (開始, 終了) index（両端含む）リスト。"""
    m = mask.astype(int)
    edges = np.diff(np.concatenate([[0], m, [0]]))
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0] - 1
    return list(zip(starts.tolist(), ends.tolist()))


def stationary_drift_deg_per_min(t: np.ndarray, rel: np.ndarray,
                                 still: np.ndarray) -> tuple[float, float]:
    """静止区間のみのドリフト率 [deg/min] と、使用した静止時間合計 [s]。

    STILL_MIN_SEG_S 以上の静止区間ごとに線形フィットし、区間長で加重平均する
    （意図的回転を含むログで全区間フィットが無意味な値になるのを避ける）。
    """
    slopes: list[float] = []
    weights: list[float] = []
    for s, e in contiguous_runs(still):
        seg_t = t[s:e + 1]
        seg_y = rel[s:e + 1]
        fin = np.isfinite(seg_t) & np.isfinite(seg_y)
        if fin.sum() < 5:
            continue
        dur = float(np.ptp(seg_t[fin]))
        if dur < STILL_MIN_SEG_S:
            continue
        slopes.append(float(np.polyfit(seg_t[fin], seg_y[fin], 1)[0]) * 60.0)
        weights.append(dur)
    if not slopes:
        return math.nan, 0.0
    return float(np.average(slopes, weights=weights)), float(sum(weights))


def write_summary(data: dict, t: np.ndarray, on: np.ndarray, path: Path,
                  meta: dict | None, out: Path) -> None:
    lines: list[str] = []
    add = lines.append
    n = data["_n"]
    dur = float(np.nanmax(t) - np.nanmin(t)) if np.isfinite(t).any() else math.nan
    add("StampFly Experiment 計測ログ サマリ")
    add("=" * 60)
    add(f"入力      : {path}")
    add(f"サンプル数: {n}  /  記録時間: {dur:.1f} s  /  実効レート: {sample_rate(t):.1f} Hz")
    on_ratio = 100.0 * on.mean() if n else math.nan
    add(f"モーターON: {int(on.sum())} サンプル ({on_ratio:.1f}%)")

    ffl = ff_state_line(meta)
    if ffl:
        add(f"FF状態    : {ffl}")
    if meta:
        for k in ("started_at", "ended_at", "aborted"):
            if k in meta:
                add(f"meta.{k}: {meta[k]}")

    add("")
    still = stationary_mask(data, on)
    still_runs = [(s, e) for s, e in contiguous_runs(still)
                  if float(t[e] - t[s]) >= STILL_MIN_SEG_S]
    still_total = float(sum(t[e] - t[s] for s, e in still_runs))
    add(f"--- ヨー各系統のドリフト（静止区間のみ: モーターOFF かつ "
        f"|r|<{math.degrees(STILL_GYRO_THRESH_RAD_S):.1f}°/s・区間毎線形フィット加重平均） ---")
    add(f"静止区間  : {len(still_runs)} 区間 / 合計 {still_total:.1f} s"
        f"（{STILL_MIN_SEG_S:.0f} s 未満の区間は除外）")
    for key, lab, _ in YAW_STREAMS:
        if not usable(data, key):
            add(f"{lab:24s}: 列なし/全欠損")
            continue
        rel, _ = relative(col(data, key))
        fin = rel[np.isfinite(rel)]
        if not fin.size:
            add(f"{lab:24s}: 有効値なし")
            continue
        rate, used_s = stationary_drift_deg_per_min(t, rel, still)
        if math.isnan(rate):
            add(f"{lab:24s}: 静止区間不足のため算出不可 "
                f"(最終値 {fin[-1]:+.2f}° / 最大|ずれ| {np.max(np.abs(fin)):.2f}°)")
            continue
        note = "（静止区間不足のため参考値）" if used_s < STILL_ENOUGH_S else ""
        add(f"{lab:24s}: ドリフト率 {rate:+.3f} °/min{note}  "
            f"(静止 {used_s:.1f} s 使用 / 最終値 {fin[-1]:+.2f}° / "
            f"最大|ずれ| {np.max(np.abs(fin)):.2f}°)")

    # 開始静止 vs 終了静止の生磁気方位差（機体が物理的に元の向きへ戻ったかの指標）
    if usable(data, "bx_cal", "by_cal", "bz_cal"):
        ym_raw = mag_yaw_deg(data, subtract_ff=False, tilt_mask_deg=15.0)
        if len(still_runs) >= 2:
            s0, e0 = still_runs[0]
            s1, e1 = still_runs[-1]
            h0 = circ_mean_deg(ym_raw[s0:e0 + 1])
            h1 = circ_mean_deg(ym_raw[s1:e1 + 1])
            src = (f"開始静止 t={t[s0]:.1f}–{t[e0]:.1f}s vs "
                   f"終了静止 t={t[s1]:.1f}–{t[e1]:.1f}s")
        else:
            h0 = circ_mean_deg(ym_raw[:10])
            h1 = circ_mean_deg(ym_raw[-10:])
            src = "静止区間が2つ未満のため先頭/末尾10サンプルで代用（参考値）"
        if math.isnan(h0) or math.isnan(h1):
            add("生磁気方位差: tilt>15° 等で有効サンプルなし")
        else:
            dh = wrap180(np.array([h1 - h0]))[0]
            add(f"生磁気方位差（{src}）: {dh:+.1f}°"
                f"  ※FF補正なし・ファーム規約 atan2 準拠。0°に近いほど物理的に元の向き")

    add("")
    add("--- 電流 / 電圧 ---")
    cur = col(data, "current_a")
    if np.isfinite(cur).any():
        add(f"電流 [A]  : min {np.nanmin(cur):.3f} / 平均 {np.nanmean(cur):.3f} / "
            f"max {np.nanmax(cur):.3f}")
    else:
        add("電流 [A]  : 列なし/全欠損")
    vb = col(data, "vbat_v")
    if np.isfinite(vb).any():
        add(f"電圧 [V]  : min {np.nanmin(vb):.3f} / 平均 {np.nanmean(vb):.3f} / "
            f"max {np.nanmax(vb):.3f}")
    else:
        add("電圧 [V]  : 列なし/全欠損")

    add("")
    add("--- NIS 統計 ---")
    nis = col(data, "nis")
    fin = nis[np.isfinite(nis)]
    if fin.size:
        add(f"有効 {fin.size} / 平均 {np.mean(fin):.2f}（期待値≈{NIS_EXPECT}） / "
            f"中央値 {np.median(fin):.2f} / p95 {np.percentile(fin, 95):.2f} / "
            f"最大 {np.max(fin):.2f}")
        add(f"NIS > {NIS_SOFT}（R膨張域）: {int((fin > NIS_SOFT).sum())} "
            f"({100.0 * (fin > NIS_SOFT).mean():.1f}%)  /  "
            f"NIS > {NIS_REJECT}（棄却域）: {int((fin > NIS_REJECT).sum())} "
            f"({100.0 * (fin > NIS_REJECT).mean():.1f}%)")
    else:
        add("nis 列なし/全欠損")

    add("")
    add("--- ffg ゲート発火数 ---")
    ffgc = col(data, "ffg")
    if np.isfinite(ffgc).any():
        ffg = np.nan_to_num(ffgc).astype(int)
        for b, (name, _) in enumerate(GATE_BITS):
            cnt = int(((ffg >> b) & 1).sum())
            add(f"bit{b} {name:12s}: {cnt:6d} サンプル ({100.0 * cnt / n:.1f}%)")
    else:
        add("ffg 列なし/全欠損")

    add("")
    add("--- ff_status ---")
    stc = col(data, "ff_status")
    if np.isfinite(stc).any():
        st = np.nan_to_num(stc).astype(int)
        modes = st & FF_MODE_MASK
        seen = ", ".join(f"{FF_MODE_NAMES.get(m, m)}×{int((modes == m).sum())}"
                         for m in sorted(set(modes.tolist())))
        add(f"ff_mode 内訳: {seen}")
        for bit, name, _ in FF_STATUS_FLAGS:
            cnt = int(((st >> bit) & 1).sum())
            add(f"bit{bit} {name:16s}: {cnt:6d} サンプル ({100.0 * cnt / n:.1f}%)")
    else:
        add("ff_status 列なし/全欠損")

    text = "\n".join(lines) + "\n"
    (out / "summary.txt").write_text(text, encoding="utf-8")
    print("  出力: summary.txt")
    print("\n" + text)


# ---------------------------------------------------------------- main --------
def explog_dir() -> Path:
    """pc_server が Experiment ログを書き出す exp_logs フォルダ。"""
    return Path(__file__).resolve().parent.parent / "pc_server" / "data" / "exp_logs"


def list_explogs() -> list[Path]:
    """exp_logs 内の explog CSV を新しい順（mtime 降順）に返す。"""
    d = explog_dir()
    if not d.is_dir():
        return []
    return sorted(d.glob(EXPLOG_GLOB), key=lambda p: p.stat().st_mtime, reverse=True)


def _fmt_size(n: int) -> str:
    return f"{n / 1024:.0f} KB" if n < 1024 * 1024 else f"{n / 1024 / 1024:.1f} MB"


def choose_explog(cands: list[Path]) -> Path | None:
    """explog CSV を一覧表示し、番号で1つ選ばせる（会話的選択）。

    Enter は最新（[1]）を選択、q で中止して None を返す。
    パイプ等で対話入力が無い（EOF）場合は最新を自動選択する。
    """
    print(f"\n{explog_dir()} の explog CSV から、グラフ化するものを選んでください:\n")
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
    """既定の出力先: data_analysis/graphs/explog_<stamp>/。"""
    stem = path.stem
    if stem.startswith("explog_"):
        stem = stem[len("explog_"):]
    return Path(__file__).resolve().parent / "graphs" / f"explog_{stem}"


def main() -> None:
    ap = argparse.ArgumentParser(
        description="StampFly Experiment 計測ログ（explog CSV）のグラフ化（PNG 6枚 + summary.txt）")
    ap.add_argument("explog", nargs="?",
                    help="explog CSV パス（省略時は exp_logs の一覧から対話選択）")
    ap.add_argument("-o", "--out",
                    help="出力ディレクトリ（省略時は data_analysis/graphs/explog_<stamp>/）")
    args = ap.parse_args()

    if args.explog:
        path = Path(args.explog)
    else:
        cands = list_explogs()
        if not cands:
            sys.exit(f"explog CSV が見つかりません: {explog_dir()} に {EXPLOG_GLOB} がありません。")
        path = choose_explog(cands)
        if path is None:
            sys.exit("中止しました（CSV が選択されていません）。")
    if not path.is_file():
        sys.exit(f"explog CSV が見つかりません: {path}")

    out = Path(args.out) if args.out else default_out_dir(path)
    out.mkdir(parents=True, exist_ok=True)

    data = load_explog(path)
    t = time_axis(data)
    on = motor_on_mask(data)
    meta = load_meta(path)
    ffl = ff_state_line(meta)
    tsuffix = f"\n{ffl}" if ffl else ""

    print(f"入力: {path}")
    print(f"出力: {out}")
    if meta is None:
        print("meta JSON なし → ff_state 表示は省略します。")
    elif ffl:
        print(f"FF状態: {ffl}")

    n_figs = 0
    n_figs += fig01_yaw_comparison(data, t, on, out, tsuffix)
    n_figs += fig02_yaw_error(data, t, on, out, tsuffix)
    n_figs += fig03_current_duty(data, t, on, out, tsuffix)
    n_figs += fig04_mag_ff(data, t, on, out, tsuffix)
    n_figs += fig05_ekf_diagnostics(data, t, on, out, tsuffix)
    n_figs += fig06_ff_status(data, t, on, out, tsuffix)
    write_summary(data, t, on, path, meta, out)

    print(f"{n_figs}枚の図 + summary.txt を {out} に出力しました。")


if __name__ == "__main__":
    main()
