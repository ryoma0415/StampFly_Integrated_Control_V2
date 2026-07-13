#!/usr/bin/env python3
"""スイープ8ラン → FFプロファイルJSON (stampfly_ff_profile v1) 抽出CLI.

仕様: docs/ff_pipeline_design.md §3。

入力 (いずれか):
  --folder <path>   フォルダ内の sweep_*_{meta.json,samples.csv} 8ペア、または
                    4ペア + sequence_*_meta.json (参照先の単機4本が同フォルダか
                    --results-dir に実在すること)。
                    (パスが存在しない場合は --results-dir 直下のサブフォルダ名として解決)
  --stems T1 .. Tn  stem 指定。sweep stem 8個 (従来)、または sweep stem 4個 +
                    sequence meta 1個 (stem / ファイル名 / パス) の5個など、
                    sequence 展開後にちょうど8本になる組み合わせ。
                    --results-dir (既定 ../pc_server/data/sweep_results)
  引数なし          対話モード: sweep_results のサブフォルダ一覧 / stems 手動選択
                    から番号で選び、name/memo を入力して生成 (q で中止)。

自動分類: meta.motors が FL+FR+RL+RR = 全機ラン(4本、notes.orientation 4種必須。
表記ゆれ Yaw=+-180°/±180° 許容)、FL/FR/RL/RR 単独 = 単機ラン(各1本必須)。
sequence meta は完了済み (phase=done, aborted でない) の単機ランのみ展開する。

出力: -o/--out (既定 ../pc_server/data/ff_profiles/<name>.json)
  name 既定: フォルダ指定時=フォルダ名 / それ以外 ff_<最初の全機ランの日付YYYYMMDD>
  memo 既定: "<notes.locationの多数派> <acquired_span> 取得8本"

使用例:
  .venv/bin/python make_ff_profile.py
  .venv/bin/python make_ff_profile.py --folder ../pc_server/data/sweep_results/DroneTest_20260629
  .venv/bin/python make_ff_profile.py --stems sweep_20260629_141809 ... (8個) \
      --name Drone-test_20260629 --memo "..."
  .venv/bin/python make_ff_profile.py --stems sweep_A sweep_B sweep_C sweep_D \
      sequence_20260701_120000   # 全機4本 + sequence meta (単機4本に展開)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
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


def _collect_from_folder(folder: Path, results_dir: Path) -> list[tuple[str, Path]]:
    """フォルダから (stem, dir) ペアを収集。

    フォルダ直下の sweep ペアに加え、sequence_*_meta.json があれば参照先の
    完了済み単機ランを展開して追加する (探索順: フォルダ → results_dir)。
    重複 stem は先勝ちで除去。ValueError は呼び出し側で報告する。
    """
    pairs: list[tuple[str, Path]] = [(s, folder) for s in _find_pairs(folder)]
    seen = {s for s, _ in pairs}
    for mp in sorted(folder.glob("sequence_*_meta.json")):
        for stem, d in core.expand_sequence_meta(mp, [folder, results_dir]):
            if stem not in seen:
                seen.add(stem)
                pairs.append((stem, d))
    return pairs


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="スイープ8ラン → FFプロファイルJSON 抽出")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--folder",
                     help="8ペア (または 4ペア + sequence meta) 入りフォルダ (主)")
    src.add_argument("--stems", nargs="+",
                     help="sweep stem 8個、または sequence meta 混在 (展開後8本)")
    ap.add_argument("--results-dir", default=str(DEFAULT_RESULTS_DIR),
                    help="--stems 時の探索dir (既定 ../pc_server/data/sweep_results)")
    ap.add_argument("--name", help="プロファイル名 (既定: フォルダ名 or ff_<日付>)")
    ap.add_argument("--memo", help="1行メモ (既定: 自動生成)")
    ap.add_argument("-o", "--out",
                    help="出力先 (.json ファイル or dir。既定 ../pc_server/data/ff_profiles/<name>.json)")
    ap.add_argument("--plots", action="store_true", help="検証図PNGを出力 (既定off)")
    args = ap.parse_args(argv)

    if not args.folder and not args.stems:
        return _interactive_main(args)
    return _run(args)


def _run(args) -> int:
    """--folder / --stems (対話モードからも到達) の共通生成パス。"""
    results_dir = Path(args.results_dir)

    # --- 入力列挙: (stem, dir) ペアに正規化 ---
    if args.folder:
        folder = Path(args.folder)
        if not folder.is_dir():
            cand = results_dir / args.folder
            if cand.is_dir():
                folder = cand
            else:
                print(f"エラー: フォルダが見つからない: {args.folder}", file=sys.stderr)
                return 2
        try:
            pairs = _collect_from_folder(folder, results_dir)
        except ValueError as e:
            print(f"エラー: {e}", file=sys.stderr)
            return 2
        if len(pairs) != 8:
            print(f"エラー: {folder} 内の sweep ペアが {len(pairs)} 本 "
                  f"(sequence 展開込み。ちょうど8必要): {[s for s, _ in pairs]}",
                  file=sys.stderr)
            return 2
        default_name = folder.name
        source_dir = str(folder)
    else:
        try:
            pairs = core.expand_stems(args.stems, results_dir)
        except ValueError as e:
            print(f"エラー: {e}", file=sys.stderr)
            return 2
        if len(pairs) != 8:
            print(f"エラー: --stems は展開後8本必要 "
                  f"(指定 {len(args.stems)} 個 → 展開後 {len(pairs)} 本)", file=sys.stderr)
            return 2
        default_name = None
        source_dir = str(results_dir)
        for stem, d in pairs:
            for suf in ("_meta.json", "_samples.csv"):
                if not (d / f"{stem}{suf}").exists():
                    print(f"エラー: {d / (stem + suf)} が見つからない", file=sys.stderr)
                    return 2

    # --- 読み込み・分類 ---
    runs = [core.load_run(stem, d) for stem, d in pairs]
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


# ------------------------------------------------------------ interactive -----
def _input(prompt: str) -> str | None:
    """input() ラッパ。q/quit/exit または EOF で None (=中止)。"""
    try:
        raw = input(prompt).strip()
    except EOFError:
        print()
        return None
    if raw.lower() in ("q", "quit", "exit"):
        return None
    return raw


def _stem_summary(stem: str, d: Path) -> str:
    """一覧表示用: motors / orientation / 日付 を meta から拾う (失敗時は空)。"""
    try:
        meta = json.loads((d / f"{stem}_meta.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return ""
    motors = meta.get("motors", "?")
    orient = core.normalize_orientation(
        (meta.get("notes") or {}).get("orientation", ""))
    parts = [f"motors={motors}"]
    if orient:
        parts.append(orient)
    if meta.get("aborted"):
        parts.append("中断")
    return "  ".join(parts)


def _choose_folder(results_dir: Path) -> tuple[str | None, bool]:
    """サブフォルダ一覧 + 'stems 手動選択' を番号で選ばせる。

    Returns: (フォルダパス or None, 手動選択フラグ)。中止は (None, False)。
    """
    subs = sorted((p for p in results_dir.iterdir() if p.is_dir()),
                  key=lambda p: p.stat().st_mtime, reverse=True)
    print(f"\n{results_dir} から入力を選んでください:\n")
    for i, p in enumerate(subs, 1):
        n_pairs = len(_find_pairs(p))
        n_seq = len(list(p.glob("sequence_*_meta.json")))
        info = f"sweepペア {n_pairs}本" + (f" + sequence meta {n_seq}個" if n_seq else "")
        mtime = time.strftime("%Y-%m-%d %H:%M", time.localtime(p.stat().st_mtime))
        mark = "  ← 最新" if i == 1 else ""
        print(f"  [{i}] {p.name}/   {mtime}   {info}{mark}")
    manual_idx = len(subs) + 1
    print(f"  [{manual_idx}] sweep_results 直下の stem を手動選択")
    print()
    while True:
        raw = _input(f"番号を入力 [1-{manual_idx}]"
                     f"（Enter={'最新' if subs else '手動選択'} / q=中止）: ")
        if raw is None:
            return None, False
        if raw == "":
            return (str(subs[0]), False) if subs else (None, True)
        if raw.isdigit() and 1 <= int(raw) <= manual_idx:
            idx = int(raw)
            if idx == manual_idx:
                return None, True
            return str(subs[idx - 1]), False
        print(f"  '{raw}' は無効です。1〜{manual_idx} の番号を入力してください。")


def _choose_stems(results_dir: Path) -> list[str] | None:
    """sweep_results 直下の sweep ペア + sequence meta から複数選択させる。

    番号をスペース/カンマ区切りで入力 (sequence 展開後にちょうど8本必要)。
    中止は None。
    """
    stems = _find_pairs(results_dir)
    seqs = sorted(results_dir.glob("sequence_*_meta.json"))
    items: list[tuple[str, str]] = []          # (token, 表示)
    for s in stems:
        items.append((s, f"{s}   {_stem_summary(s, results_dir)}"))
    for mp in seqs:
        try:
            n = len(json.loads(mp.read_text(encoding='utf-8')).get("runs", []))
        except (OSError, json.JSONDecodeError):
            n = 0
        items.append((mp.name, f"{mp.name}   sequence meta ({n} runs)"))
    if not items:
        print(f"エラー: {results_dir} に sweep ペア / sequence meta が無い",
              file=sys.stderr)
        return None
    items.sort(key=lambda t: t[0], reverse=True)   # stem 名 = 日時なので新しい順
    print(f"\n{results_dir} 直下から使用するものを選んでください "
          "(全機4 + 単機4、または 全機4 + sequence meta 1):\n")
    for i, (_tok, disp) in enumerate(items, 1):
        print(f"  [{i}] {disp}")
    print()
    while True:
        raw = _input("番号をスペース/カンマ区切りで入力（例: 1 2 3 4 5 / q=中止）: ")
        if raw is None:
            return None
        nums = [t for t in raw.replace(",", " ").split() if t]
        if not nums or not all(t.isdigit() and 1 <= int(t) <= len(items) for t in nums):
            print(f"  無効な入力です。1〜{len(items)} の番号を入力してください。")
            continue
        idxs = list(dict.fromkeys(int(t) for t in nums))   # 重複除去 (順序維持)
        tokens = [items[i - 1][0] for i in idxs]
        try:
            pairs = core.expand_stems(tokens, results_dir)
        except ValueError as e:
            print(f"  エラー: {e}")
            continue
        if len(pairs) != 8:
            print(f"  選択 {len(tokens)} 個 → sequence 展開後 {len(pairs)} 本 "
                  "(ちょうど8本必要)。選び直してください。")
            continue
        return tokens


def _interactive_main(args) -> int:
    """引数なし起動: フォルダ/stems を対話選択し name/memo を入力して生成。"""
    results_dir = Path(args.results_dir)
    if not results_dir.is_dir():
        print(f"エラー: sweep_results が見つからない: {results_dir}", file=sys.stderr)
        return 2
    folder, manual = _choose_folder(results_dir)
    if folder is None and not manual:
        print("中止しました。")
        return 0
    if manual:
        tokens = _choose_stems(results_dir)
        if tokens is None:
            print("中止しました。")
            return 0
        args.stems = tokens
    else:
        args.folder = folder

    default_name = Path(folder).name if folder else "(自動: ff_<日付>)"
    raw = _input(f"プロファイル名 [Enter={default_name} / q=中止]: ")
    if raw is None:
        print("中止しました。")
        return 0
    if raw:
        args.name = raw
    raw = _input("メモ [Enter=自動生成 / q=中止]: ")
    if raw is None:
        print("中止しました。")
        return 0
    if raw:
        args.memo = raw
    print()
    return _run(args)


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
    vec_pts = [(0.0, 0.0)]  # annotate/text は autoscale に反映されないため軸範囲を自前で決める
    for m in core.MOTORS:
        am = profile["method_b"]["a_m"][m]
        ax.annotate("", xy=(am[0], am[1]), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color="C0"))
        ax.text(am[0], am[1], f" {m}", fontsize=10)
        vec_pts.append((am[0], am[1]))
    ab = profile["method_b"]["a_bar"]
    ax.annotate("", xy=(ab[0], ab[1]), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color="C3", lw=2))
    ax.text(ab[0], ab[1], " mean", color="C3", fontsize=10)
    vec_pts.append((ab[0], ab[1]))
    vp = np.asarray(vec_pts, dtype=float)
    pad_x = 0.15 * max(float(np.ptp(vp[:, 0])), 1e-6)
    pad_y = 0.15 * max(float(np.ptp(vp[:, 1])), 1e-6)
    ax.set_xlim(vp[:, 0].min() - pad_x, vp[:, 0].max() + pad_x)
    ax.set_ylim(vp[:, 1].min() - pad_y, vp[:, 1].max() + pad_y)
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
