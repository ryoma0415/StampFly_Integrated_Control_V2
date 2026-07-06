#!/usr/bin/env python3
"""StampFly モーター磁気ノイズの加算性（重ね合わせ）検証.

モーター別スイープ（FL/FR/RL/RR 単機運転）で得た電流ノイズ ΔB_m を足し合わせた
ものが、全機同時スイープの ΔB_all と一致するか（線形重ね合わせ＝加算性）を
定量化する。対角ペア（FL+RR / FR+RL）の run があれば、該当2機の和とも比較する。

入力は pc_server の SequenceRunner が出力する sequence_<runid>_meta.json と、
そこから参照される各 run の samples CSV（bracket-baseline 方式。measure 行に
ドリフト除去済みの dB_cor_{x,y,z} 入り）・個別 meta JSON（idle_current_a）。

使い方:
    python analyze_additivity.py                      # 一覧から対話選択
    python analyze_additivity.py sequence_..._meta.json [-o 出力先] [-d CSVディレクトリ]
    python analyze_additivity.py --legacy 単機1.csv 単機2.csv ... --legacy-target 同時.csv

検証内容:
  1) 主検証（duty一致比較）: 各 duty で Σ_単機 ΔB_m(d) と同時測定 ΔB(d) を軸別比較
  2) 副検証（電流空間の傾き）: ΔB = a·(I−idle) の a を slope_同時 と Σslope_m/機数 で比較
     （各モーターは同時運転時に総電流のほぼ 1/機数 を分担するという仮定の整合チェック）
  3) 電圧垂下チェック: 同 duty の I_同時 と [Σ_m (I_m−idle_m) + idle_同時] の差

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

AXES = ("x", "y", "z")
AXIS_LABEL = {"x": "ΔB_x", "y": "ΔB_y", "z": "ΔB_z"}
SINGLES = ("FL", "FR", "RL", "RR")          # 単機 run のモーター名
DIAG_PAIRS = ("FL+RR", "FR+RL")             # 対角ペア run のモーター名
NOISE_SIGMA = 4.0                            # 判定: 最大|残差| ≤ max(4σ, 下限)
NOISE_FLOOR_UT = 0.5                         # 判定しきい値の下限 [µT]（分解能スケール）

LEGACY_HELP = """\
レガシーモード（--legacy / --legacy-target）の旧データの場所:
  Measure_current_vs_b_field/StampFly_WiFi_Telemetry_System/pc_server/sweep_results/
    単機:  sweep_20260605_234246 (FL), sweep_20260605_235122 (FR),
           sweep_20260606_000011 (RL), sweep_20260606_000852 (RR)
    同時:  sweep_20260605_232458 (FR+RL ペア), sweep_20260604_160705 (4機) 等
  summary CSV（sweep_*_summary.csv）の phase=="measure" 行を使う。
  行の対応付けは duty_cmd 列があれば duty 一致、無ければ duty 順位（行の出現順）。
例:
  python analyze_additivity.py --legacy FR_summary.csv RL_summary.csv \\
                               --legacy-target FR+RL_summary.csv
"""


# ---------------------------------------------------------------- load --------
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


def load_sequence_runs(meta_path: Path, csv_dir: Path) -> list[dict]:
    """sequence meta JSON から完了済み run を全て読み込んで集計する。"""
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
            sys.exit(f"samples CSV が見つかりません: {spath}（-d/--dir でディレクトリを指定できます）")
        data = load_samples(spath)
        idle = math.nan
        if r.get("meta"):
            mpath = csv_dir / r["meta"]
            if mpath.is_file():
                idle = _f(str(json.loads(mpath.read_text(encoding="utf-8")).get("idle_current_a", "")))
        if not math.isfinite(idle):  # フォールバック: base 行（duty=0）の平均電流
            b = data["phase"] == "base"
            idle = float(np.nanmean(data["current_a"][b])) if b.any() else math.nan
        runs.append(aggregate_run(data, idle, label=spath.stem, motors=motors))
    return runs


def load_legacy_summary(path: Path) -> tuple[dict, bool]:
    """旧 Measure 版 summary CSV を run 辞書に読む。返り値 (run, duty列の有無)。

    使用列: phase / duty_cmd(任意) / current_a_mean / dB_cor_{x,y,z}。
    ノイズは b{x,y,z}_cor_std / √n_samples（列があれば）。
    """
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    if not rows:
        sys.exit(f"空のCSVです: {path}")
    meas = [r for r in rows if r.get("phase") == "measure"]
    if not meas:
        sys.exit(f"phase=='measure' 行がありません: {path}")
    base = [r for r in rows if r.get("phase") == "base"]
    idle = _f(base[0].get("current_a_mean")) if base else math.nan
    motors = meas[0].get("motors") or "?"
    has_duty = "duty_cmd" in rows[0]
    pd_ = {}
    for rank, r in enumerate(meas, 1):
        key = round(_f(r.get("duty_cmd")), 3) if has_duty else float(rank)
        dB = np.array([_f(r.get(f"dB_cor_{a}")) for a in AXES])
        n = _f(r.get("n_samples"))
        std = np.array([_f(r.get(f"b{a}_cor_std")) for a in AXES])
        if math.isfinite(n) and n > 0 and np.isfinite(std).all():
            noise = std / math.sqrt(n)
        else:
            noise = np.full(3, math.nan)
        pd_[float(key)] = {"I": _f(r.get("current_a_mean")), "dB": dB, "noise": noise}
    return {"label": path.stem, "motors": motors, "idle": idle, "pd": pd_}, has_duty


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


def _fmt_key(k: float, duty_is_rank: bool) -> str:
    return f"#{int(k):>4d}" if duty_is_rank else f"{k:5.2f}"


def print_main_table(name: str, keys, meas, pred, resid, duty_is_rank: bool) -> None:
    hdr = "順位" if duty_is_rank else "duty"
    print(f"\n=== 主検証（duty一致比較）: {name} ===")
    print("    残差 = Σ単機予測 − 同時測定 [µT]")
    print(f"  {hdr:>5} | " + "  ".join(f"測定{a}{'':>4}予測{a}{'':>4}残差{a}{'':>2}" for a in AXES))
    for j, k in enumerate(keys):
        cells = "  ".join(f"{meas[j, i]:7.2f} {pred[j, i]:7.2f} {resid[j, i]:+6.2f}  "
                          for i in range(3))
        print(f"  {_fmt_key(k, duty_is_rank)} | {cells}")
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


def print_current_check(name: str, target: dict, singles: list[dict],
                        keys, duty_is_rank: bool) -> None:
    """電圧垂下の注意の定量化: 同 duty の電流 I_同時 と Σ(I_m−idle_m)+idle_同時 の比較。"""
    print(f"\n=== 電圧垂下チェック（同 duty の電流比較）: {name} ===")
    hdr = "順位" if duty_is_rank else "duty"
    print(f"  {hdr:>5} | I_同時 [A]  Σ(I_m−idle_m)+idle_同時 [A]    差 [A]")
    diffs = []
    for k in keys:
        i_t = target["pd"][k]["I"]
        i_pred = sum(s["pd"][k]["I"] - s["idle"] for s in singles) + target["idle"]
        diffs.append(i_t - i_pred)
        print(f"  {_fmt_key(k, duty_is_rank)} | {i_t:9.3f}   {i_pred:21.3f}   {i_t - i_pred:+8.3f}")
    diffs = np.array(diffs)
    print(f"  差の平均 {np.nanmean(diffs):+.3f} A / 最大|差| {np.nanmax(np.abs(diffs)):.3f} A")
    print("  注意: 同時運転は電圧垂下でモーター個別の電流が単機時より下がるため、")
    print("        duty一致比較には系統差が乗り得る（差が大きいほど ΔB 比較にも影響）。")


def verdict_line(name: str, resid: np.ndarray, noise: np.ndarray) -> str:
    """判定サマリ1行: 最大残差をノイズ由来のしきい値と比較して 成立/不成立 を返す。"""
    max_resid = float(np.nanmax(np.abs(resid)))
    finite = noise[np.isfinite(noise)]
    if finite.size:
        noise_typ = float(np.sqrt(np.mean(finite ** 2)))  # 合成ノイズの RMS
        thr = max(NOISE_SIGMA * noise_typ, NOISE_FLOOR_UT)
        ok = max_resid <= thr
        return (f"加算性[{name}]: 最大残差 {max_resid:.2f} µT"
                f"（測定ノイズ~{noise_typ:.2f} µT・しきい値 {thr:.2f} µT に対し"
                f"{'成立' if ok else '不成立'}）")
    ok = max_resid <= NOISE_FLOOR_UT
    return (f"加算性[{name}]: 最大残差 {max_resid:.2f} µT"
            f"（測定ノイズ不明・しきい値 {NOISE_FLOOR_UT:.2f} µT に対し"
            f"{'成立' if ok else '不成立'}）")


# -------------------------------------------------------------- figures -------
def save_fig(fig, out: Path, name: str) -> None:
    """全図共通の保存処理（レイアウト調整 → PNG 出力 → クローズ）。"""
    fig.tight_layout()
    fig.savefig(out / name, dpi=130, bbox_inches="tight")
    plt.close(fig)


def fig_additivity(name: str, fname: str, keys, meas, pred, resid, noise,
                   out: Path, duty_is_rank: bool) -> None:
    """軸別: 上段 duty vs 測定/予測の重ねプロット、下段 残差（±3σ ノイズ帯付き）。"""
    xlabel = "duty 順位" if duty_is_rank else "duty"
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
                            color="gray", alpha=0.25, label="±3σ ノイズ")
        rx.plot(keys, resid[:, i], "o-", color="tab:green", label="残差（予測−測定）")
        rx.axhline(0, color="gray", lw=0.8)
        rx.set_xlabel(xlabel)
        rx.set_ylabel("残差 [µT]")
        rx.grid(alpha=0.3)
    axarr[0, 0].legend(fontsize=8)
    axarr[1, 0].legend(fontsize=8)
    fig.suptitle(f"加算性検証 {name}: Σ単機 ΔB と同時スイープ ΔB の比較", fontsize=13)
    save_fig(fig, out, fname)


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


def analyze_comparisons(comps: list[tuple[dict, list[dict]]], out: Path,
                        duty_is_rank: bool = False) -> None:
    """全比較ペアについて 主検証・副検証・電流チェック・図 を実行し判定サマリを出す。"""
    verdicts = []
    for idx, (target, singles) in enumerate(comps, 1):
        name = f"{' + '.join(s['motors'] for s in singles)} → {target['motors']}"
        keys, meas, pred, resid, noise = compare_additivity(target, singles)
        print_main_table(name, keys, meas, pred, resid, duty_is_rank)
        print_slope_check(name, target, singles)
        print_current_check(name, target, singles, keys, duty_is_rank)
        fname = f"{idx:02d}_additivity_{target['motors'].replace('+', '_')}.png"
        fig_additivity(name, fname, keys, meas, pred, resid, noise, out, duty_is_rank)
        verdicts.append(verdict_line(name, resid, noise))
    print("\n=== 判定サマリ ===")
    for v in verdicts:
        print(f"  {v}")
    print(f"\n図 {len(comps)} 枚を {out} に出力しました。")


# ---------------------------------------------------------------- main --------
SEQ_GLOB = "sequence_*_meta.json"


def sweep_dir() -> Path:
    """pc_server が結果を書き出す sweep_results フォルダ。"""
    return Path(__file__).resolve().parent.parent / "pc_server" / "sweep_results"


def list_sequence_metas() -> list[Path]:
    """sweep_results 内の sequence meta JSON を新しい順（mtime 降順）に返す。"""
    d = sweep_dir()
    if not d.is_dir():
        return []
    return sorted(d.glob(SEQ_GLOB), key=lambda p: p.stat().st_mtime, reverse=True)


def _fmt_size(n: int) -> str:
    return f"{n / 1024:.0f} KB" if n < 1024 * 1024 else f"{n / 1024 / 1024:.1f} MB"


def choose_meta(cands: list[Path]) -> Path | None:
    """利用可能な sequence meta を一覧表示し、番号で1つ選ばせる（会話的選択）。

    Enter は最新（[1]）を選択、q で中止して None を返す。
    パイプ等で対話入力が無い（EOF）場合は最新を自動選択する。
    """
    print(f"\n{sweep_dir()} の sequence meta から、解析するものを選んでください:\n")
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


def default_out_dir(stem: str) -> Path:
    """既定の出力先: data_analysis/graphs/additivity_<stem>/（この解析フォルダ配下）。"""
    return Path(__file__).resolve().parent / "graphs" / f"additivity_{stem}"


def _strip_suffix(stem: str, suffix: str) -> str:
    return stem[: -len(suffix)] if stem.endswith(suffix) else stem


def main() -> None:
    ap = argparse.ArgumentParser(
        description="StampFly モーター磁気ノイズの加算性（重ね合わせ）検証",
        epilog=LEGACY_HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sequence_meta", nargs="?",
                    help="sequence meta JSON パス（省略時は sweep_results の一覧から対話選択）")
    ap.add_argument("-d", "--dir",
                    help="samples CSV のあるディレクトリ（省略時は meta と同じディレクトリ）")
    ap.add_argument("-o", "--out",
                    help="出力ディレクトリ（省略時は data_analysis/graphs/additivity_<meta名stem>/）")
    ap.add_argument("--legacy", nargs="+", metavar="SINGLE_CSV",
                    help="レガシーモード: 旧Measure版 summary CSV（単機）を複数指定")
    ap.add_argument("--legacy-target", metavar="TARGET_CSV",
                    help="レガシーモード: 比較対象の同時スイープ summary CSV（ペア/全機）")
    args = ap.parse_args()

    # ---------------- レガシーモード（旧 Measure 版 summary CSV） ----------------
    if args.legacy or args.legacy_target:
        if not (args.legacy and args.legacy_target):
            sys.exit("--legacy と --legacy-target は両方指定してください。")
        singles = []
        all_have_duty = True
        for p in args.legacy:
            run, has_duty = load_legacy_summary(Path(p))
            singles.append(run)
            all_have_duty &= has_duty
        target, has_duty = load_legacy_summary(Path(args.legacy_target))
        all_have_duty &= has_duty
        if not all_have_duty:
            # duty 列が無いファイルが混じる場合は全 run を duty 順位で対応付け直す
            print("duty_cmd 列が無いため、duty 順位（measure 行の出現順）で対応付けます。")
            for r in singles + [target]:
                r["pd"] = {float(i): v for i, v in enumerate(r["pd"].values(), 1)}
        stem = _strip_suffix(Path(args.legacy_target).stem, "_summary")
        out = Path(args.out) if args.out else default_out_dir(f"legacy_{stem}")
        out.mkdir(parents=True, exist_ok=True)
        print(f"レガシーモード 入力: 単機 {len(singles)} 本 "
              f"({', '.join(s['motors'] for s in singles)}) / 対象 {target['motors']}")
        print(f"出力: {out}")
        for r in singles + [target]:
            print(f"  {r['label']}: motors={r['motors']} idle={r['idle']:.3f} A "
                  f"duty段数={len(r['pd'])}")
        analyze_comparisons([(target, singles)], out, duty_is_rank=not all_have_duty)
        return

    # ---------------- 通常モード（SequenceRunner の sequence meta） ----------------
    if args.sequence_meta:
        meta_path = Path(args.sequence_meta)
    else:
        cands = list_sequence_metas()
        if not cands:
            sys.exit(f"sequence meta が見つかりません: {sweep_dir()} に {SEQ_GLOB} がありません。")
        meta_path = choose_meta(cands)
        if meta_path is None:
            sys.exit("中止しました（meta が選択されていません）。")
    if not meta_path.is_file():
        sys.exit(f"sequence meta が見つかりません: {meta_path}")

    csv_dir = Path(args.dir) if args.dir else meta_path.parent
    stem = _strip_suffix(meta_path.stem, "_meta")
    out = Path(args.out) if args.out else default_out_dir(stem)
    out.mkdir(parents=True, exist_ok=True)

    print(f"入力: {meta_path}\nCSVディレクトリ: {csv_dir}\n出力: {out}")
    runs = load_sequence_runs(meta_path, csv_dir)
    if not runs:
        sys.exit("完了済み（phase=='done'）の run がありません。")
    for r in runs:
        print(f"  {r['label']}: motors={r['motors']} idle={r['idle']:.3f} A "
              f"duty段数={len(r['pd'])}")
    comps = build_comparisons(runs)
    if not comps:
        sys.exit("比較できる組み合わせがありません（単機4本＋同時 run が必要です）。")
    analyze_comparisons(comps, out)


if __name__ == "__main__":
    main()
