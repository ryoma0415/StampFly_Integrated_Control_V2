#!/usr/bin/env python3
"""StampFly V2 フライトログ可視化ツール(対話式 CLI)。

引数なしで実行すると対話式に「../logs のログ選択 → 出力内容選択」を行う。
引数を与えるとバッチ実行もできる(例は README.md を参照)。

出力先: flight_log_viewer/output/<ログ名>/
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# flight_log_viewer/ を import パスに追加(どこから実行しても動くように)
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from viewer.loader import FlightLog, load_log  # noqa: E402

# 既定パス
DEFAULT_LOGS_DIR = _HERE.parent / "logs"       # V2 リポジトリ直下 logs/
DEFAULT_OUTPUT_DIR = _HERE / "output"

# 対話メニュー項目
_MENU_ITEMS = (
    "静止画グラフ一式+ヨー解析+サマリレポート",
    "アニメーション MP4(動画なし)",
    "アニメーション MP4(スマホ動画と同期)",
    "すべて生成(静止画+レポート+アニメーション)",
    "2 ログのヨー安定性比較",
)


# ---------------------------------------------------------------------------
# 対話式ヘルパー(旧 Drone_Log_Viewer の選択 UI を踏襲)
# ---------------------------------------------------------------------------

def _select_from_list(items: list[str], label: str) -> int | None:
    """番号選択 UI。選択インデックス(0 始まり)を返す。q でキャンセル。"""
    print(f"\n=== {label} ===")
    for i, item in enumerate(items, start=1):
        print(f"  {i}. {item}")
    while True:
        choice = input(f"\n{label} を選択してください (1-{len(items)}, q で中止): ").strip()
        if choice.lower() == "q":
            return None
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(items):
                return idx
        except ValueError:
            pass
        print("無効な入力です。もう一度入力してください。")


def _select_csv(logs_dir: Path, label: str = "ログファイル") -> Path | None:
    """logs_dir の CSV を列挙して選択させる。パス直接入力にも対応。"""
    csv_files = sorted(logs_dir.glob("*.csv"))
    if not csv_files:
        print(f"\n{logs_dir} に CSV が見つかりません。パスを直接入力してください。")
        raw = input("CSV パス (空で中止): ").strip()
        if not raw:
            return None
        return Path(raw).expanduser()

    names = [p.name for p in csv_files] + ["(パスを直接入力する)"]
    idx = _select_from_list(names, label)
    if idx is None:
        return None
    if idx == len(csv_files):
        raw = input("CSV パス (空で中止): ").strip()
        return Path(raw).expanduser() if raw else None
    return csv_files[idx]


def _ask_path(prompt: str) -> Path | None:
    raw = input(prompt).strip()
    return Path(raw).expanduser() if raw else None


# ---------------------------------------------------------------------------
# 実行アクション
# ---------------------------------------------------------------------------

def _out_dir_for(log: FlightLog, output_root: Path) -> Path:
    return output_root / log.name


def run_figures_and_report(log: FlightLog, output_root: Path) -> None:
    """静止画一式+ヨー解析図+サマリレポート。"""
    from viewer import plots, report, yaw_analysis  # noqa: PLC0415

    out_dir = _out_dir_for(log, output_root)
    stats = yaw_analysis.compute_yaw_stats(log)
    figure_paths = plots.generate_static_figures(log, out_dir)
    figure_paths += yaw_analysis.generate_yaw_figures(log, out_dir, stats)
    report.generate_report(log, out_dir, figure_paths)


def run_animation(log: FlightLog, output_root: Path,
                  video_path: Path | None, track: bool,
                  fps: float | None, start_s: float | None,
                  end_s: float | None) -> None:
    """アニメーション MP4。"""
    from viewer import animation  # noqa: PLC0415

    out_dir = _out_dir_for(log, output_root)
    suffix = "_with_video" if video_path else ""
    out_path = out_dir / f"{log.name}_animation{suffix}.mp4"
    animation.generate_animation(
        log, out_path, video_path=video_path, fps=fps,
        start_s=start_s, end_s=end_s, track=track)


def run_comparison(log_a: FlightLog, log_b: FlightLog, output_root: Path) -> None:
    """2 ログ比較レポート。"""
    from viewer import report  # noqa: PLC0415

    out_dir = output_root / f"compare_{log_a.name}_vs_{log_b.name}"
    report.generate_comparison(log_a, log_b, out_dir)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="StampFly V2 フライトログ可視化ツール"
                    "(引数なしで対話モード)")
    parser.add_argument("csv", nargs="?", type=Path,
                        help="対象ログ CSV(省略時は対話選択)")
    parser.add_argument("--figures", action="store_true",
                        help="静止画グラフ+ヨー解析+レポートを生成")
    parser.add_argument("--animation", action="store_true",
                        help="アニメーション MP4 を生成")
    parser.add_argument("--all", action="store_true",
                        help="--figures と --animation の両方")
    parser.add_argument("--video", type=Path, default=None,
                        help="同期合成するスマホ動画(MP4)。--animation と併用")
    parser.add_argument("--track", action="store_true",
                        help="動画に ROI 追跡枠を合成(GUI 必要・opencv 必要)")
    parser.add_argument("--fps", type=float, default=None,
                        help="アニメーションの出力 fps(既定: 動画fps または 20)")
    parser.add_argument("--start", type=float, default=None,
                        help="アニメーション切り出し開始 [s]")
    parser.add_argument("--end", type=float, default=None,
                        help="アニメーション切り出し終了 [s]")
    parser.add_argument("--compare", type=Path, default=None,
                        help="比較対象の 2 本目のログ CSV(ヨー安定性比較)")
    parser.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS_DIR,
                        help=f"対話選択時のログ置き場(既定: {DEFAULT_LOGS_DIR})")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_DIR,
                        help=f"出力ルート(既定: {DEFAULT_OUTPUT_DIR})")
    return parser.parse_args()


def _run_batch(args: argparse.Namespace) -> int:
    """アクションフラグ指定時のバッチ実行。"""
    log = load_log(args.csv)
    print(f"読み込み完了: {log.path}({len(log.df)}行, モード={log.mode})")

    if args.compare is not None:
        log_b = load_log(args.compare)
        run_comparison(log, log_b, args.output)
        return 0

    did_something = False
    if args.figures or args.all:
        run_figures_and_report(log, args.output)
        did_something = True
    if args.animation or args.all:
        run_animation(log, args.output, args.video, args.track,
                      args.fps, args.start, args.end)
        did_something = True
    if not did_something:
        # CSV だけ指定された場合は静止画+レポートを既定動作にする
        run_figures_and_report(log, args.output)
    return 0


def _run_interactive(args: argparse.Namespace) -> int:
    """対話モード。"""
    print("=== StampFly V2 フライトログ可視化ツール ===")
    print(f"ログ置き場: {args.logs_dir}")

    menu_idx = _select_from_list(list(_MENU_ITEMS), "出力内容")
    if menu_idx is None:
        print("中止しました。")
        return 1

    csv_path = _select_csv(args.logs_dir)
    if csv_path is None:
        print("中止しました。")
        return 1
    log = load_log(csv_path)
    print(f"\n読み込み完了: {log.path}({len(log.df)}行, モード={log.mode})")

    if menu_idx == 4:  # 比較
        csv_b = _select_csv(args.logs_dir, "比較対象ログ")
        if csv_b is None:
            print("中止しました。")
            return 1
        log_b = load_log(csv_b)
        run_comparison(log, log_b, args.output)
        return 0

    video_path: Path | None = None
    track = False
    if menu_idx == 2:  # 動画同期アニメーション
        video_path = _ask_path("スマホ動画のパス (空で中止): ")
        if video_path is None:
            print("中止しました。")
            return 1
        if not video_path.is_file():
            print(f"エラー: 動画が見つかりません: {video_path}")
            return 1
        track = input("ROI 追跡枠を合成しますか? (y/N): ").strip().lower() == "y"

    if menu_idx in (0, 3):
        run_figures_and_report(log, args.output)
    if menu_idx in (1, 2, 3):
        run_animation(log, args.output, video_path, track,
                      args.fps, args.start, args.end)

    print(f"\n=== 処理完了 === 出力先: {_out_dir_for(log, args.output)}")
    return 0


def main() -> int:
    args = _parse_args()
    try:
        if args.csv is not None:
            return _run_batch(args)
        return _run_interactive(args)
    except KeyboardInterrupt:
        print("\n中止しました。")
        return 130
    except (FileNotFoundError, ValueError, RuntimeError) as e:
        print(f"エラー: {e}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
