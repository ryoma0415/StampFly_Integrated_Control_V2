#!/usr/bin/env python3
"""sweep.py — 推定器定数のオフライン掃引ラッパ(stampfly_ecosystem eskf_sweep.py の型)。

対象定数はファームヘッダの const/constexpr(alt_kalman.hpp の q1/q2/R/beta、
yaw_config.hpp の FF_EKF_*)で、マクロガードされていないため -D では上書き
できない。そこで「-D でビルドし直す」のと同じ意味になる方式として、掃引値
ごとに build/sweep/ 配下へシャドウソースツリー(全ファイル symlink+対象
ヘッダのみ正規表現置換した生成コピー)を作り、build.sh で再ビルド→実行する。
ファームツリー本体には一切触れない。

※ 契約(yaw_config.hpp「数式・符号・定数値変更禁止」)により実機値は不変。
   本掃引は「もし変えたら」のオフライン評価専用であり、結果をファームへ
   反映する場合はベンチ・飛行再検証が前提。

使い方:
  # 高度KF: R を掃引し、実ログの mocap 真値 RMS で比較(--fixed 系列)
  python3 sweep.py alt --log ../../logs/flight_logs/20260717_195243_position.csv \\
      --param R --values 1.6e-05,4e-04,1e-03,4e-03

  # ヨーEKF: yaw_config.hpp の定数を掃引し、合成自己試験の指標で比較
  python3 sweep.py yaw --param FF_EKF_R_BASE_UT2 --values 2.0,4.0,8.0
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW_SRC = os.path.normpath(os.path.join(HERE, "..", "..", "firmware_stampfly", "src"))
SWEEP_ROOT = os.path.join(HERE, "build", "sweep")

# 掃引可能な定数 → (対象ヘッダの相対パス, 置換パターン, 置換テンプレート)
# パターンは対象ヘッダ内でちょうど1回一致しなければエラー(定義行の変化検知)。
ALT_HPP = "alt_kalman.hpp"
YAW_HPP = os.path.join("yaw_estimation", "yaw_config.hpp")
ALT_PARAMS = {
    "q1": (ALT_HPP, r"(float q1 = )[^,]+(,)", r"\g<1>{v}f\g<2>"),
    "q2": (ALT_HPP, r"(q2 = )[^;]+(;)", r"\g<1>{v}f\g<2>"),
    "R": (ALT_HPP, r"(?m)^(\s*)float R = [^;]+;", r"\g<1>float R = {v}f;"),
    "beta": (ALT_HPP, r"(float beta = )[^;]+(;)", r"\g<1>{v}f\g<2>"),
}


def patch_rule(target: str, param: str):
    if target == "alt":
        if param not in ALT_PARAMS:
            sys.exit(f"alt の掃引対象は {sorted(ALT_PARAMS)} のみ(指定: {param})")
        return ALT_PARAMS[param]
    if not re.fullmatch(r"[A-Z][A-Z0-9_]+", param):
        sys.exit(f"yaw の掃引対象は yaw_config.hpp の定数名(例 FF_EKF_R_BASE_UT2): {param}")
    return (
        YAW_HPP,
        rf"(?m)^static const float {param} = [^;]+;",
        rf"static const float {param} = {{v}}f;",
    )


def make_shadow(tag: str, hpp_rel: str, pattern: str, template: str, value: float) -> str:
    """全ファイル symlink+対象ヘッダのみ置換生成のシャドウ src ツリーを作る。"""
    shadow = os.path.join(SWEEP_ROOT, tag, "src")
    shutil.rmtree(os.path.join(SWEEP_ROOT, tag), ignore_errors=True)
    os.makedirs(os.path.join(shadow, "yaw_estimation"))
    for sub in ("", "yaw_estimation"):
        src_dir = os.path.join(FW_SRC, sub)
        for name in os.listdir(src_dir):
            p = os.path.join(src_dir, name)
            if os.path.isfile(p):
                os.symlink(p, os.path.join(shadow, sub, name))
    # 対象ヘッダ: symlink を外し、置換した実コピーに差し替える
    target = os.path.join(shadow, hpp_rel)
    os.unlink(target)
    with open(os.path.join(FW_SRC, hpp_rel)) as f:
        text = f.read()
    lit = f"{value:.9g}"
    if not any(c in lit for c in ".eE"):  # "2" → "2.0"(f サフィックスを付けるため)
        lit += ".0"
    new_text, n = re.subn(pattern, template.replace("{v}", lit), text)
    if n != 1:
        sys.exit(f"置換が {n} 回一致({hpp_rel} の定義行が変わった可能性): {pattern}")
    with open(target, "w") as f:
        f.write(new_text)
    return shadow


def build(shadow_src: str, tag: str) -> str:
    out_dir = os.path.join(SWEEP_ROOT, tag)
    env = dict(os.environ, FW_SRC=shadow_src, OUT=out_dir)
    r = subprocess.run(["sh", os.path.join(HERE, "build.sh")], env=env,
                       capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"build failed (tag={tag}):\n{r.stderr}")
    return out_dir


def parse_kv_line(stdout: str, prefix: str) -> dict[str, str]:
    for line in reversed(stdout.splitlines()):
        if line.startswith(prefix + " "):
            return dict(tok.split("=", 1) for tok in line.split()[1:] if "=" in tok)
    sys.exit(f"{prefix} 行が出力に無い:\n{stdout}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", choices=["alt", "yaw"])
    ap.add_argument("--param", required=True, help="掃引する定数名")
    ap.add_argument("--values", required=True, help="カンマ区切りの掃引値")
    ap.add_argument("--log", help="alt: 入力 v4 ログ CSV(必須)")
    args = ap.parse_args()

    values = [float(v) for v in args.values.split(",")]
    if args.target == "alt" and not args.log:
        sys.exit("alt には --log <v4_log.csv> が必要")
    hpp_rel, pattern, template = patch_rule(args.target, args.param)

    if args.target == "alt":
        keys = ["id_asis_rms", "mocap_asis_rms", "mocap_fixed_rms", "mocap_fixed_max"]
        prefix = "RESULT"
    else:
        keys = ["rms_deg", "max_deg", "nis_mean", "reject_rate", "result"]
        prefix = "SELFTEST"

    print(f"# sweep {args.target} {args.param} = {values}")
    print("# 注意: 契約により実機定数は不変。オフライン評価専用。")
    w = max(14, len(args.param))
    header = f"{args.param:>{w}} | " + " | ".join(f"{k:>16}" for k in keys)
    print(header)
    print("-" * len(header))
    for v in values:
        tag = f"{args.target}_{args.param}_{v:.9g}".replace("-", "m").replace("+", "")
        shadow = make_shadow(tag, hpp_rel, pattern, template, v)
        out_dir = build(shadow, tag)
        if args.target == "alt":
            cmd = [os.path.join(out_dir, "replay_alt"), args.log]
        else:
            cmd = [os.path.join(out_dir, "replay_yaw"), "--selftest"]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0 and args.target == "alt":
            sys.exit(f"run failed:\n{r.stdout}\n{r.stderr}")
        kv = parse_kv_line(r.stdout, prefix)
        print(f"{v:>{w}.9g} | " + " | ".join(f"{kv.get(k, 'n/a'):>16}" for k in keys))


if __name__ == "__main__":
    main()
