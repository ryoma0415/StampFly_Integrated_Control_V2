#!/usr/bin/env python3
"""スイープ8ラン → FFプロファイルJSON (stampfly_ff_profile v1) 抽出CLI.

仕様: docs/ff_pipeline_design.md §3。

入力 (どちらか):
  --folder <path>   フォルダ内の sweep_*_{meta.json,samples.csv} ちょうど8ペア (主)
                    (パスが存在しない場合は --results-dir 直下のサブフォルダ名として解決)
  --stems S1 .. S8  stem 8個指定 (従)。--results-dir (既定 ../pc_server/data/sweep_results)

自動分類: meta.motors が FL+FR+RL+RR = 全機ラン(4本、notes.orientation 4種必須。
表記ゆれ Yaw=+-180°/±180° 許容)、FL/FR/RL/RR 単独 = 単機ラン(各1本必須)。

出力: -o/--out (既定 ../pc_server/data/ff_profiles/<name>.json)
  name 既定: フォルダ指定時=フォルダ名 / それ以外 ff_<最初の全機ランの日付YYYYMMDD>
  memo 既定: "<notes.locationの多数派> <acquired_span> 取得8本"

使用例:
  .venv/bin/python make_ff_profile.py --folder ../pc_server/data/sweep_results/DroneTest_20260629
  .venv/bin/python make_ff_profile.py --stems sweep_20260629_141809 ... (8個) \
      --name Drone-test_20260629 --memo "..."
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from ff_params import core  # noqa: E402

DEFAULT_RESULTS_DIR = HERE.parent / "pc_server" / "data" / "sweep_results"
DEFAULT_OUT_DIR = HERE.parent / "pc_server" / "data" / "ff_profiles"


def _find_pairs(folder: Path) -> list[str]:
    """フォルダ内の sweep_*_meta.json + samples.csv ペアの stem を列挙。"""
    stems = []
    for mp in sorted(folder.glob("sweep_*_meta.json")):
        stem = mp.name[: -len("_meta.json")]
        if (folder / f"{stem}_samples.csv").exists():
            stems.append(stem)
    return stems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="スイープ8ラン → FFプロファイルJSON 抽出")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--folder", help="8ペア(16ファイル)入りフォルダ (主)")
    src.add_argument("--stems", nargs="+", help="stem 8個 (従)")
    ap.add_argument("--results-dir", default=str(DEFAULT_RESULTS_DIR),
                    help="--stems 時の探索dir (既定 ../pc_server/data/sweep_results)")
    ap.add_argument("--name", help="プロファイル名 (既定: フォルダ名 or ff_<日付>)")
    ap.add_argument("--memo", help="1行メモ (既定: 自動生成)")
    ap.add_argument("-o", "--out",
                    help="出力先 (.json ファイル or dir。既定 ../pc_server/data/ff_profiles/<name>.json)")
    ap.add_argument("--plots", action="store_true", help="検証図PNGを出力 (既定off)")
    args = ap.parse_args(argv)

    # --- 入力列挙 ---
    if args.folder:
        folder = Path(args.folder)
        if not folder.is_dir():
            cand = Path(args.results_dir) / args.folder
            if cand.is_dir():
                folder = cand
            else:
                print(f"エラー: フォルダが見つからない: {args.folder}", file=sys.stderr)
                return 2
        stems = _find_pairs(folder)
        if len(stems) != 8:
            print(f"エラー: {folder} 内の sweep ペアが {len(stems)} 個 (ちょうど8必要): {stems}",
                  file=sys.stderr)
            return 2
        data_dir = folder
        default_name = folder.name
        source_dir = str(folder)
    else:
        stems = args.stems
        if len(stems) != 8:
            print(f"エラー: --stems は8個必要 (指定 {len(stems)} 個)", file=sys.stderr)
            return 2
        data_dir = Path(args.results_dir)
        default_name = None
        source_dir = str(data_dir)
        for stem in stems:
            for suf in ("_meta.json", "_samples.csv"):
                if not (data_dir / f"{stem}{suf}").exists():
                    print(f"エラー: {data_dir / (stem + suf)} が見つからない", file=sys.stderr)
                    return 2

    # --- 読み込み・分類 ---
    runs = [core.load_run(stem, data_dir) for stem in stems]
    try:
        all_runs, motor_runs = core.classify_runs(runs)
    except ValueError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 2

    # --- name / memo 既定値 ---
    name = args.name or default_name
    if not name:
        first_date = core._date_of(all_runs[0]["meta"]).replace("-", "")
        name = f"ff_{first_date}"
    memo = args.memo
    if memo is None:
        locs = Counter((r["meta"].get("notes") or {}).get("location", "")
                       for r in all_runs + [motor_runs[m] for m in core.MOTORS])
        loc = locs.most_common(1)[0][0]
        dates = sorted({core._date_of(r["meta"])
                        for r in all_runs + [motor_runs[m] for m in core.MOTORS]} - {""})
        span = dates[0] if len(dates) == 1 else f"{dates[0]}〜{dates[-1]}"
        memo = f"{loc} {span} 取得8本"

    # --- 抽出 ---
    try:
        profile, internals = core.extract_profile(all_runs, motor_runs,
                                                  name=name, memo=memo, source_dir=source_dir)
    except ValueError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 2

    # --- 出力 ---
    if args.out:
        out = Path(args.out)
        if out.suffix != ".json":
            out = out / f"{name}.json"
    else:
        out = DEFAULT_OUT_DIR / f"{name}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    # extract_profile 内で assert_finite 済みだが、万一の取りこぼしも
    # allow_nan=False で非標準JSONリテラル (NaN/Infinity) の書き出しを防ぐ
    try:
        text = json.dumps(profile, ensure_ascii=False, indent=2, allow_nan=False)
    except ValueError as e:
        print(f"エラー: プロファイルに非有限値 (NaN/Inf) が含まれる: {e}", file=sys.stderr)
        return 2
    out.write_text(text + "\n")

    lut = profile["method_a"]["lut"]
    print(f"プロファイル出力: {out}")
    print(f"  name={name}  memo={memo}")
    print(f"  LUT {len(lut['points'])}点 (I={lut['points'][0]['i_a']:.3f}–"
          f"{lut['points'][-1]['i_a']:.3f} A, i_idle={lut['i_idle_a']:.4f} A)")
    print(f"  affine_ref a={[round(v,3) for v in profile['method_a']['affine_ref']['a']]} "
          f"b={[round(v,3) for v in profile['method_a']['affine_ref']['b']]}")
    for m in core.MOTORS:
        am = profile["method_b"]["a_m"][m]
        q = profile["method_b"]["duty_to_current"][m]
        print(f"  {m}: a_m={[round(v,3) for v in am]} µT/A  "
              f"duty→I c=[{q['c2']:+.4f},{q['c1']:+.4f},{q['c0']:+.4f}] RMS={q['rms_a']:.4f}A")
    print(f"  pair_diff_xy={profile['method_b']['pair_diff_xy_uT_per_A']:.2f} µT/A")
    print(f"  additivity_closure={[round(v,3) for v in profile['stats']['additivity_closure']]}")
    if profile["quality"]["warnings"]:
        print("  警告:")
        for w in profile["quality"]["warnings"]:
            print(f"    - {w}")

    if args.plots:
        _make_plots(profile, internals, name)
    return 0


def _make_plots(profile: dict, internals: dict, name: str) -> None:
    """検証図PNG: LUT + affine (軸別) と 単機 a_m xy平面ベクトル。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    out_dir = HERE / "graphs" / f"ff_profile_{name}"
    out_dir.mkdir(parents=True, exist_ok=True)

    pts = profile["method_a"]["lut"]["points"]
    I = np.array([p["i_a"] for p in pts])
    dB = np.array([p["db"] for p in pts])
    a = np.array(profile["method_a"]["affine_ref"]["a"])
    b = np.array(profile["method_a"]["affine_ref"]["b"])
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.2))
    for k, ax_name in enumerate(core.AXES):
        ax = axs[k]
        ax.plot(I, dB[:, k], "o-", ms=4, label="LUT")
        xs = np.linspace(I.min(), I.max(), 20)
        ax.plot(xs, a[k] * xs + b[k], "k--", lw=1, label="affine_ref")
        ax.set_xlabel("I_total [A]")
        ax.set_ylabel(f"dB_{ax_name} [uT]")
        ax.grid(alpha=0.3)
        if k == 0:
            ax.legend(fontsize=8)
    fig.suptitle(f"{name}: method_a LUT")
    fig.tight_layout()
    fig.savefig(out_dir / "lut.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    for m in core.MOTORS:
        am = profile["method_b"]["a_m"][m]
        ax.annotate("", xy=(am[0], am[1]), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color="C0"))
        ax.text(am[0], am[1], f" {m}", fontsize=10)
    ab = profile["method_b"]["a_bar"]
    ax.annotate("", xy=(ab[0], ab[1]), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color="C3", lw=2))
    ax.text(ab[0], ab[1], " mean", color="C3", fontsize=10)
    ax.set_xlabel("a_x [uT/A]")
    ax.set_ylabel("a_y [uT/A]")
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(f"{name}: a_m (xy)")
    fig.tight_layout()
    fig.savefig(out_dir / "a_m_vectors.png", dpi=140)
    plt.close(fig)
    print(f"  検証図 → {out_dir}")


if __name__ == "__main__":
    sys.exit(main())
