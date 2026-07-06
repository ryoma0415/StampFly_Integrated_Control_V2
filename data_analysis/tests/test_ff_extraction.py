#!/usr/bin/env python3
"""FFプロファイル抽出の受入テスト (pytest不要、.venv/bin/python で直接実行).

  ① 6/12照合(厳密): 6/12 の 8 stem で抽出し、
     graphs/feasibility_20260612/results.json の pooled_model.a/b・
     additivity.per_motor_a_prop・duty_to_current_quadfit・lut_breakpoints.points
     と相対誤差 < 1e-6 で一致すること (同一コードパスの移植確認)。
  ② 付録A再現(新機体): 6/29+6/30 の 8本で抽出し、
     yaw_estimation_ff_two_methods.md 付録A の数値と仕様§3.3の許容内で一致。

実行: cd data_analysis && .venv/bin/python tests/test_ff_extraction.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA_ANALYSIS = HERE.parent
sys.path.insert(0, str(DATA_ANALYSIS))

import numpy as np  # noqa: E402

from ff_params import core  # noqa: E402

RESULTS_DIR = DATA_ANALYSIS.parent / "pc_server" / "sweep_results"
FEAS_JSON = DATA_ANALYSIS / "graphs" / "feasibility_20260612" / "results.json"

STEMS_0612 = [
    # 全機 x 4姿勢
    "sweep_20260612_202215", "sweep_20260612_203731",
    "sweep_20260612_204405", "sweep_20260612_205956",
    # 単機 FL / FR / RL / RR
    "sweep_20260612_211141", "sweep_20260612_211628",
    "sweep_20260612_212714", "sweep_20260612_213135",
]
STEMS_0629 = [
    "sweep_20260629_141809", "sweep_20260629_142817",
    "sweep_20260629_150309", "sweep_20260629_151655",
    "sweep_20260630_163738", "sweep_20260630_172536",
    "sweep_20260630_174549", "sweep_20260630_175440",
]

_fail_count = 0


def _check(cond: bool, msg: str) -> None:
    global _fail_count
    if not cond:
        _fail_count += 1
        print(f"  FAIL: {msg}")


def _relerr(got, ref) -> float:
    """相対誤差 (ref==0 かつ got==0 なら 0)。"""
    got, ref = float(got), float(ref)
    if ref == 0.0:
        return abs(got)
    return abs(got - ref) / abs(ref)


def _extract(stems: list[str]):
    runs = [core.load_run(s, RESULTS_DIR) for s in stems]
    all_runs, motor_runs = core.classify_runs(runs)
    return core.extract_profile(all_runs, motor_runs, name="test", memo="test",
                                source_dir=str(RESULTS_DIR))


# =========================================================== ① 6/12 照合 ======
def test_0612_exact_match() -> None:
    print("[受入①] 6/12 8 stem 抽出 vs results.json (相対誤差 < 1e-6)")
    ref = json.loads(FEAS_JSON.read_text())
    profile, internals = _extract(STEMS_0612)
    TOL = 1e-6
    max_err = 0.0

    # pooled_model.a/b vs method_a.affine_ref
    for key, got in (("a", profile["method_a"]["affine_ref"]["a"]),
                     ("b", profile["method_a"]["affine_ref"]["b"])):
        for k in range(3):
            e = _relerr(got[k], ref["pooled_model"][key][k])
            max_err = max(max_err, e)
            _check(e < TOL, f"pooled_model.{key}[{k}]: got={got[k]!r} ref={ref['pooled_model'][key][k]!r} relerr={e:.3g}")

    # additivity.per_motor_a_prop vs method_b.a_m
    for m in core.MOTORS:
        for k in range(3):
            got = profile["method_b"]["a_m"][m][k]
            r = ref["additivity"]["per_motor_a_prop"][m][k]
            e = _relerr(got, r)
            max_err = max(max_err, e)
            _check(e < TOL, f"a_m[{m}][{k}]: got={got!r} ref={r!r} relerr={e:.3g}")

    # duty_to_current_quadfit
    for m in core.MOTORS:
        got = profile["method_b"]["duty_to_current"][m]
        r = ref["duty_to_current_quadfit"][m]
        for gk, rk in (("c2", 0), ("c1", 1), ("c0", 2)):
            e = _relerr(got[gk], r["c2_c1_c0"][rk])
            max_err = max(max_err, e)
            _check(e < TOL, f"duty_to_current[{m}].{gk}: got={got[gk]!r} ref={r['c2_c1_c0'][rk]!r} relerr={e:.3g}")
        e = _relerr(got["rms_a"], r["rms_a"])
        max_err = max(max_err, e)
        _check(e < TOL, f"duty_to_current[{m}].rms_a relerr={e:.3g}")

    # lut_breakpoints.points vs method_a.lut.points
    ref_pts = ref["lut_breakpoints"]["points"]
    got_pts = profile["method_a"]["lut"]["points"]
    _check(len(got_pts) == len(ref_pts),
           f"LUT点数: got={len(got_pts)} ref={len(ref_pts)}")
    for i, (gp, rp) in enumerate(zip(got_pts, ref_pts)):
        e = _relerr(gp["i_a"], rp["I"])
        max_err = max(max_err, e)
        _check(e < TOL, f"lut[{i}].i_a: got={gp['i_a']!r} ref={rp['I']!r} relerr={e:.3g}")
        for k in range(3):
            e = _relerr(gp["db"][k], rp["dB"][k])
            max_err = max(max_err, e)
            _check(e < TOL, f"lut[{i}].db[{k}]: got={gp['db'][k]!r} ref={rp['dB'][k]!r} relerr={e:.3g}")
    e = _relerr(profile["method_a"]["lut"]["i_idle_a"], ref["lut_breakpoints"]["idle_mean_a"])
    max_err = max(max_err, e)
    _check(e < TOL, f"i_idle_a relerr={e:.3g}")

    print(f"  最大相対誤差 = {max_err:.3g}  (許容 {TOL})")


# =========================================================== ② 付録A再現 ======
# yaw_estimation_ff_two_methods.md 付録A (新機体 2026-06-29/30) の数値
APPENDIX_A_AFFINE_A = [24.41, 28.67, 8.40]
APPENDIX_A_AFFINE_B = [-6.50, -3.49, -1.56]
APPENDIX_A_AM = {
    "FL": [36.98, 38.26, 43.35],
    "FR": [3.04, -6.10, -11.10],
    "RL": [24.99, 38.96, 42.89],
    "RR": [27.07, 43.52, -51.59],
}
APPENDIX_A_ABAR = [23.02, 28.66, 5.89]
APPENDIX_A_D2C = {
    "FL": [0.457, 0.851, 0.053],
    "FR": [0.361, 0.768, 0.055],
    "RL": [0.383, 0.855, 0.052],
    "RR": [0.424, 0.781, 0.049],
}
APPENDIX_A_PAIR_DIFF_XY = 30.4


def _within(got, ref, rel, abs_tol=None) -> bool:
    """許容: 相対 rel または (指定時) 絶対 abs_tol のどちらか満たせばOK。"""
    got, ref = float(got), float(ref)
    if abs(got - ref) <= (abs_tol if abs_tol is not None else 0.0):
        return True
    return ref != 0.0 and abs(got - ref) / abs(ref) <= rel


def test_appendix_a() -> None:
    print("[受入②] 6/29+6/30 8本 抽出 vs ff_two_methods 付録A (§3.3許容)")
    profile, internals = _extract(STEMS_0629)

    # affine: 許容 2% or 0.15 abs
    a = profile["method_a"]["affine_ref"]["a"]
    b = profile["method_a"]["affine_ref"]["b"]
    for k in range(3):
        _check(_within(a[k], APPENDIX_A_AFFINE_A[k], 0.02, 0.15),
               f"affine a[{k}]: got={a[k]:.4f} ref={APPENDIX_A_AFFINE_A[k]}")
        _check(_within(b[k], APPENDIX_A_AFFINE_B[k], 0.02, 0.15),
               f"affine b[{k}]: got={b[k]:.4f} ref={APPENDIX_A_AFFINE_B[k]}")
    print(f"  affine a: got={[round(v,3) for v in a]} ref={APPENDIX_A_AFFINE_A}")
    print(f"  affine b: got={[round(v,3) for v in b]} ref={APPENDIX_A_AFFINE_B}")

    # a_m: 許容 2% or 0.5 µT/A abs (ā も同基準)
    for m in core.MOTORS:
        got = profile["method_b"]["a_m"][m]
        for k in range(3):
            _check(_within(got[k], APPENDIX_A_AM[m][k], 0.02, 0.5),
                   f"a_m[{m}][{k}]: got={got[k]:.4f} ref={APPENDIX_A_AM[m][k]}")
        print(f"  a_m {m}: got={[round(v,3) for v in got]} ref={APPENDIX_A_AM[m]}")
    abar = profile["method_b"]["a_bar"]
    for k in range(3):
        _check(_within(abar[k], APPENDIX_A_ABAR[k], 0.02, 0.5),
               f"a_bar[{k}]: got={abar[k]:.4f} ref={APPENDIX_A_ABAR[k]}")
    print(f"  a_bar: got={[round(v,3) for v in abar]} ref={APPENDIX_A_ABAR}")

    # duty→電流 c2/c1/c0: 許容 5%
    # (付録Aは小数3桁丸めのため c0≈0.05 では丸めだけで ~1% 動く。5%許容内)
    for m in core.MOTORS:
        got = profile["method_b"]["duty_to_current"][m]
        for gk, idx in (("c2", 0), ("c1", 1), ("c0", 2)):
            _check(_within(got[gk], APPENDIX_A_D2C[m][idx], 0.05),
                   f"duty_to_current[{m}].{gk}: got={got[gk]:.4f} ref={APPENDIX_A_D2C[m][idx]}")
        print(f"  duty→I {m}: got=[{got['c2']:.3f},{got['c1']:.3f},{got['c0']:.3f}] "
              f"ref={APPENDIX_A_D2C[m]}")

    # pair_diff_xy: 許容 2%
    pd = profile["method_b"]["pair_diff_xy_uT_per_A"]
    _check(_within(pd, APPENDIX_A_PAIR_DIFF_XY, 0.02),
           f"pair_diff_xy: got={pd:.3f} ref={APPENDIX_A_PAIR_DIFF_XY}")
    print(f"  pair_diff_xy: got={pd:.3f} ref={APPENDIX_A_PAIR_DIFF_XY}")


# ================================================= ③ 中断ラン拒否 (C11) ======
def test_reject_aborted_run() -> None:
    print("[C11] classify_runs: meta.aborted=true のランを stem 列挙付きで拒否")
    # meta.aborted=true の合成ラン (classify_runs は meta/stem のみ参照)
    runs = [{"stem": "sweep_fake_aborted",
             "meta": {"motors": "FL+FR+RL+RR", "aborted": True,
                      "notes": {"orientation": "Yaw=0°"}}}]
    try:
        core.classify_runs(runs)
        _check(False, "aborted ラン が ValueError にならなかった")
    except ValueError as e:
        msg = str(e)
        _check("aborted" in msg, f"エラーメッセージに 'aborted' が無い: {msg}")
        _check("sweep_fake_aborted" in msg, f"エラーメッセージに stem が無い: {msg}")
        print(f"  OK: ValueError = {msg[:60]}...")
    # aborted=false / 欠落 は拒否しない (実データ8本で classify が通ること)
    runs = [core.load_run(s, RESULTS_DIR) for s in STEMS_0612]
    try:
        core.classify_runs(runs)
        print("  OK: 完走8本 (aborted=false) は従来どおり分類成功")
    except ValueError as e:
        _check(False, f"完走ランが誤って拒否された: {e}")


# ================================================= ④ 非有限値検出 (C12) ======
def test_assert_finite() -> None:
    print("[C12] assert_finite: NaN/Inf 注入で JSONパス付き ValueError")
    ok = {"a": [1.0, 2.5], "b": {"c": -3.0, "d": None, "e": "text"}}
    try:
        core.assert_finite(ok)
        print("  OK: 有限値のみ (None含む) は通過")
    except ValueError as e:
        _check(False, f"有限値のみで誤検出: {e}")
    for bad_val, label in ((math.nan, "NaN"), (math.inf, "Inf")):
        bad = {"method_a": {"lut": {"points": [{"db": [0.0, bad_val, 1.0]}]}}}
        try:
            core.assert_finite(bad)
            _check(False, f"{label} 注入が ValueError にならなかった")
        except ValueError as e:
            msg = str(e)
            _check("method_a.lut.points[0].db[1]" in msg,
                   f"JSONパスがメッセージに無い: {msg}")
            print(f"  OK: {label} → ValueError = {msg}")


def main() -> int:
    test_0612_exact_match()
    print()
    test_appendix_a()
    print()
    test_reject_aborted_run()
    print()
    test_assert_finite()
    print()
    if _fail_count:
        print(f"NG: {_fail_count} 件の不一致")
        return 1
    print("OK: 全受入テスト合格")
    return 0


if __name__ == "__main__":
    sys.exit(main())
