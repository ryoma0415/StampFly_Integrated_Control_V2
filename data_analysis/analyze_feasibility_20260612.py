#!/usr/bin/env python3
"""2026-06-12 取得 8 スイープの横断解析: 電流ノイズ補正の実現可能性評価.

入力 (pc_server/sweep_results/):
  全機 (FL+FR+RL+RR) x 4姿勢: 202215(Yaw=0), 203731(Yaw=90), 204405(Yaw=±180), 205956(Yaw=-90)
  単機 x Yaw=0: 211141(FL), 211628(FR), 212714(RL), 213135(RR)

評価項目:
  A) 姿勢間の係数一貫性 (body座標の ΔB=a·I+b が4姿勢で一致するか)
  B) 補正残差 → Yaw角誤差換算 (per-orientationフィット / pooled / leave-one-out)
  C) 加算性: Σ単機 vs 全機 (電流空間 / duty空間)
  D) ①総電流モデル vs ③モーター別モデル: 差動推力シナリオの誤差
  E) 基準磁場ドリフト・ヒステリシス・ノイズ床 (KF設計パラメータ根拠)

出力: graphs/feasibility_20260612/ に図 + results.json + コンソールにレポート
依存: numpy, matplotlib
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

_INSTALLED = {f.name for f in fm.fontManager.ttflist}
for _jp in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic", "Yu Gothic",
            "Noto Sans CJK JP", "IPAexGothic", "Apple SD Gothic Neo", "Arial Unicode MS"):
    if _jp in _INSTALLED:
        plt.rcParams["font.family"] = _jp
        break
plt.rcParams["axes.unicode_minus"] = False

HERE = Path(__file__).resolve().parent
RESULTS_DIR = HERE.parent / "pc_server" / "sweep_results"
OUT_DIR = HERE / "graphs" / "feasibility_20260612"
OUT_DIR.mkdir(parents=True, exist_ok=True)

AXES = ("x", "y", "z")

ALL_RUNS = {  # stem -> orientation label
    "sweep_20260612_202215": "Yaw=0",
    "sweep_20260612_203731": "Yaw=90",
    "sweep_20260612_204405": "Yaw=180",
    "sweep_20260612_205956": "Yaw=-90",
}
MOTOR_RUNS = {  # stem -> motor
    "sweep_20260612_211141": "FL",
    "sweep_20260612_211628": "FR",
    "sweep_20260612_212714": "RL",
    "sweep_20260612_213135": "RR",
}
FLIGHT_BAND_A = (2.6, 3.5)   # 全機時の飛行帯電流 [A]


# ---------------------------------------------------------------- load --------
def _f(v):
    if v is None or v == "":
        return math.nan
    try:
        return float(v)
    except ValueError:
        return math.nan


def load_run(stem: str) -> dict:
    meta = json.loads((RESULTS_DIR / f"{stem}_meta.json").read_text())
    rows = list(csv.DictReader((RESULTS_DIR / f"{stem}_samples.csv").open()))
    cols = {}
    for key in rows[0].keys():
        if key in ("phase", "motors", "leg"):
            cols[key] = np.array([r.get(key) or "" for r in rows], dtype=object)
        else:
            cols[key] = np.array([_f(r.get(key, "")) for r in rows], dtype=float)
    return {"meta": meta, "cols": cols, "stem": stem}


def aggregate(run: dict) -> dict:
    """measure行を (duty, leg) 別に集計: 平均電流, 平均dB, 窓内std, n."""
    c = run["cols"]
    m = c["phase"] == "measure"
    duty, cur, leg = c["duty_cmd"][m], c["current_a"][m], c["leg"][m]
    temp = c["imu_temp_c"][m]
    dB = np.column_stack([c[f"dB_cor_{a}"][m] for a in AXES])
    steps = []
    for d in sorted(set(np.round(duty, 3))):
        for lg in ("up", "down"):
            sel = np.isclose(duty, d) & (leg == lg)
            if sel.sum() < 3:
                continue
            steps.append({
                "duty": d, "leg": lg, "n": int(sel.sum()),
                "I": float(cur[sel].mean()),
                "dB": dB[sel].mean(axis=0),
                "std": dB[sel].std(axis=0),
                "temp": float(temp[sel].mean()),
            })
    idle = float(run["meta"].get("idle_current_a", math.nan))
    return {"steps": steps, "idle": idle}


def fit_affine(I, Y, anchor=None):
    """軸別 affine LS: y = a*I + b。anchor=(I0, 0) を含められる。"""
    I = np.asarray(I, float)
    Y = np.asarray(Y, float)
    if anchor is not None:
        I = np.append(I, anchor[0])
        Y = np.vstack([Y, np.zeros(3)])
    A = np.column_stack([I, np.ones_like(I)])
    coef, *_ = np.linalg.lstsq(A, Y, rcond=None)
    return coef[0], coef[1]   # a[3], b[3]


def fit_prop(I_active, Y):
    """原点拘束 y = a*(I-idle)。"""
    I = np.asarray(I_active, float)
    Y = np.asarray(Y, float)
    denom = (I * I).sum()
    return (I[:, None] * Y).sum(axis=0) / denom


# ---------------------------------------------------------------- main --------
def main():
    results = {}
    print("=" * 88)
    print("A) 全機スイープ 4姿勢: body座標係数の一貫性")
    print("=" * 88)

    all_aggs = {}
    horiz_fields = {}
    for stem, label in ALL_RUNS.items():
        run = load_run(stem)
        agg = aggregate(run)
        all_aggs[label] = agg
        # 実測水平磁場 (baseline corベクトルの平均)
        bp = np.array([p["cor"] for p in run["meta"]["baseline_points"]])
        horiz = float(np.hypot(bp[:, 0], bp[:, 1]).mean())
        horiz_fields[label] = horiz
        # ドリフト指標
        drift_vec = bp[-1] - bp[0]
        t_span = run["meta"]["baseline_points"][-1]["t_s"] - run["meta"]["baseline_points"][0]["t_s"]
        jumps = np.linalg.norm(np.diff(bp, axis=0), axis=1)
        temps = run["meta"]["imu_temp_c"]
        agg["drift"] = {
            "total_uT": float(np.linalg.norm(drift_vec)),
            "vec": drift_vec.tolist(),
            "span_s": float(t_span),
            "rate_uT_per_min": float(np.linalg.norm(drift_vec) / t_span * 60),
            "max_jump_uT": float(jumps.max()),
            "temp_drop_c": temps["start"] - temps["end"],
        }
        agg["horiz_uT"] = horiz
        agg["vbat"] = run["meta"]["battery"]

    # 各姿勢の affine フィット (全域, idleアンカー込み) と飛行帯フィット
    fits = {}
    for label, agg in all_aggs.items():
        I = [s["I"] for s in agg["steps"]]
        Y = [s["dB"] for s in agg["steps"]]
        a, b = fit_affine(I, Y, anchor=(agg["idle"], 0.0))
        sel = [(i, y) for i, y in zip(I, Y) if FLIGHT_BAND_A[0] <= i <= FLIGHT_BAND_A[1]]
        a_fb, b_fb = fit_affine([s[0] for s in sel], [s[1] for s in sel]) if len(sel) >= 3 else (a, b)
        a_pr = fit_prop(np.array(I) - agg["idle"], Y)
        fits[label] = {"a": a, "b": b, "a_fb": a_fb, "b_fb": b_fb, "a_prop": a_pr}
        print(f"\n[{label}]  idle={agg['idle']:.3f}A  H_horiz={agg['horiz_uT']:.1f}µT  "
              f"vbat {agg['vbat']['vbat_start_v']:.2f}→{agg['vbat']['vbat_end_v']:.2f}V "
              f"(min {agg['vbat']['vbat_min_v']:.2f})")
        for k, ax in enumerate(AXES):
            print(f"  ΔB_{ax} = {a[k]:8.3f}·I {b[k]:+8.3f}   | 飛行帯: {a_fb[k]:8.3f}·I {b_fb[k]:+8.3f}"
                  f"   | 原点拘束: {a_pr[k]:8.3f}·(I−idle)")

    # 姿勢間ばらつき
    a_mat = np.array([fits[l]["a"] for l in ALL_RUNS.values()])        # (4,3)
    a_fb_mat = np.array([fits[l]["a_fb"] for l in ALL_RUNS.values()])
    print("\n--- 姿勢間の傾き a [µT/A] (全域フィット) ---")
    print(f"  平均: {a_mat.mean(axis=0).round(3).tolist()}")
    print(f"  std : {a_mat.std(axis=0).round(3).tolist()}")
    print(f"  rel : {(a_mat.std(axis=0)/np.abs(a_mat.mean(axis=0))*100).round(1).tolist()} %")
    print("--- 飛行帯フィット ---")
    print(f"  平均: {a_fb_mat.mean(axis=0).round(3).tolist()}")
    print(f"  std : {a_fb_mat.std(axis=0).round(3).tolist()}")
    results["orientation_consistency"] = {
        "a_mean": a_mat.mean(axis=0).tolist(), "a_std": a_mat.std(axis=0).tolist(),
        "a_fb_mean": a_fb_mat.mean(axis=0).tolist(), "a_fb_std": a_fb_mat.std(axis=0).tolist(),
        "per_orientation": {l: {k: np.asarray(v).tolist() for k, v in f.items()}
                            for l, f in fits.items()},
    }

    # ------------------------------------------------------------------
    print("\n" + "=" * 88)
    print("B) 補正残差 → Yaw角誤差 (実測水平磁場で換算, 残差⊥磁場の最悪ケース)")
    print("=" * 88)
    H_mean = float(np.mean(list(horiz_fields.values())))

    def yaw_err_deg(r_xy, H):
        return math.degrees(math.atan2(r_xy, H))

    # pooled モデル (4姿勢の duty-step 点を結合, idleアンカー4点込み)
    I_pool, Y_pool = [], []
    for label, agg in all_aggs.items():
        I_pool += [s["I"] for s in agg["steps"]] + [agg["idle"]]
        Y_pool += [s["dB"] for s in agg["steps"]] + [np.zeros(3)]
    a_pool, b_pool = fit_affine(I_pool, Y_pool)
    sel = [(i, y) for i, y in zip(I_pool, Y_pool) if FLIGHT_BAND_A[0] <= i <= FLIGHT_BAND_A[1]]
    a_pool_fb, b_pool_fb = fit_affine([s[0] for s in sel], [s[1] for s in sel])
    results["pooled_model"] = {"a": a_pool.tolist(), "b": b_pool.tolist(),
                               "a_fb": a_pool_fb.tolist(), "b_fb": b_pool_fb.tolist()}
    print(f"\npooledモデル(全域):   a={a_pool.round(3).tolist()} b={b_pool.round(3).tolist()}")
    print(f"pooledモデル(飛行帯): a={a_pool_fb.round(3).tolist()} b={b_pool_fb.round(3).tolist()}")

    scenarios = {}
    rows_table = []
    for label, agg in all_aggs.items():
        H = agg["horiz_uT"]
        steps = agg["steps"]
        I = np.array([s["I"] for s in steps])
        Y = np.array([s["dB"] for s in steps])
        fb = (I >= FLIGHT_BAND_A[0]) & (I <= FLIGHT_BAND_A[1])

        def stats(resid, mask):
            r = resid[mask]
            r_xy = np.hypot(r[:, 0], r[:, 1])
            return {
                "rms_axis": np.sqrt((r ** 2).mean(axis=0)).tolist(),
                "max_axis": np.abs(r).max(axis=0).tolist(),
                "rms_xy": float(np.sqrt((r_xy ** 2).mean())),
                "max_xy": float(r_xy.max()),
                "yaw_rms_deg": yaw_err_deg(float(np.sqrt((r_xy ** 2).mean())), H),
                "yaw_max_deg": yaw_err_deg(float(r_xy.max()), H),
            }

        uncorr = stats(Y, fb)                                # 補正なし
        own = Y - (I[:, None] * fits[label]["a"] + fits[label]["b"])      # 自姿勢fit
        own_fb = Y - (I[:, None] * fits[label]["a_fb"] + fits[label]["b_fb"])
        pool = Y - (I[:, None] * a_pool + b_pool)            # pooled
        pool_fb = Y - (I[:, None] * a_pool_fb + b_pool_fb)
        # leave-one-out
        I_loo, Y_loo = [], []
        for l2, agg2 in all_aggs.items():
            if l2 == label:
                continue
            I_loo += [s["I"] for s in agg2["steps"]] + [agg2["idle"]]
            Y_loo += [s["dB"] for s in agg2["steps"]] + [np.zeros(3)]
        a_loo, b_loo = fit_affine(I_loo, Y_loo)
        loo = Y - (I[:, None] * a_loo + b_loo)

        scen = {
            "uncorrected_fb": uncorr,
            "own_fit_fb": stats(own, fb), "own_fit_all": stats(own, np.ones(len(I), bool)),
            "own_fb_fit_fb": stats(own_fb, fb),
            "pooled_fb": stats(pool, fb), "pooled_fb_fit_fb": stats(pool_fb, fb),
            "loo_fb": stats(loo, fb),
        }
        scenarios[label] = scen
        rows_table.append((label, uncorr, scen["own_fb_fit_fb"], scen["pooled_fb_fit_fb"], scen["loo_fb"]))

    print(f"\n飛行帯 {FLIGHT_BAND_A[0]}–{FLIGHT_BAND_A[1]} A における水平残差 |r_xy| と最悪Yaw誤差:")
    print(f"{'姿勢':>8} | {'補正なし':^21} | {'自姿勢fit(飛行帯)':^21} | {'pooled(飛行帯)':^21} | {'leave-one-out':^21}")
    print(f"{'':>8} | {'RMS µT':>9} {'max°':>10} | {'RMS µT':>9} {'max°':>10} | {'RMS µT':>9} {'max°':>10} | {'RMS µT':>9} {'max°':>10}")
    for label, u, o, p, l in rows_table:
        print(f"{label:>8} | {u['rms_xy']:9.2f} {u['yaw_max_deg']:10.2f} | "
              f"{o['rms_xy']:9.3f} {o['yaw_max_deg']:10.3f} | "
              f"{p['rms_xy']:9.3f} {p['yaw_max_deg']:10.3f} | "
              f"{l['rms_xy']:9.3f} {l['yaw_max_deg']:10.3f}")
    results["correction_scenarios"] = scenarios
    results["H_horiz_mean_uT"] = H_mean
    results["H_horiz_per_orientation"] = horiz_fields

    # ------------------------------------------------------------------
    print("\n" + "=" * 88)
    print("C) 加算性: Σ単機 vs 全機 (Yaw=0)")
    print("=" * 88)
    motor_aggs = {}
    motor_fits = {}
    for stem, mname in MOTOR_RUNS.items():
        run = load_run(stem)
        agg = aggregate(run)
        motor_aggs[mname] = agg
        I = np.array([s["I"] for s in agg["steps"]])
        Y = np.array([s["dB"] for s in agg["steps"]])
        a_pr = fit_prop(I - agg["idle"], Y)
        a_af, b_af = fit_affine(I, Y, anchor=(agg["idle"], 0.0))
        motor_fits[mname] = {"a_prop": a_pr, "a": a_af, "b": b_af, "idle": agg["idle"]}
        print(f"  {mname}: a_prop={a_pr.round(3).tolist()} µT/A (原点拘束, I_active空間) "
              f" max I={I.max():.2f}A")

    a_props = np.array([motor_fits[m]["a_prop"] for m in ("FL", "FR", "RL", "RR")])
    a_sum = a_props.sum(axis=0)
    print(f"\n  Σa_m = {a_sum.round(3).tolist()}  → Σa_m/4 = {(a_sum/4).round(3).tolist()}")
    az0 = fits["Yaw=0"]["a_prop"]
    print(f"  全機(Yaw=0) 原点拘束 a = {np.asarray(az0).round(3).tolist()}  (総電流空間)")
    print(f"  比 (Σa_m/4) / a_all = {(a_sum/4/np.asarray(az0)).round(3).tolist()}")

    # duty別比較: 電流空間予測 Σ a_m·(I_active_all/4) vs 実測
    agg0 = all_aggs["Yaw=0"]
    add_rows = []
    for s in agg0["steps"]:
        Ia = s["I"] - agg0["idle"]
        pred_cur = a_sum / 4 * Ia
        # duty一致予測: 単機の同duty実測dBの和
        pred_duty = np.zeros(3)
        ok = True
        for mname, magg in motor_aggs.items():
            match = [t for t in magg["steps"] if np.isclose(t["duty"], s["duty"]) and t["leg"] == s["leg"]]
            if not match:
                ok = False
                break
            pred_duty += match[0]["dB"]
        add_rows.append({
            "duty": s["duty"], "leg": s["leg"], "I_all": s["I"],
            "meas": s["dB"].tolist(),
            "pred_current_space": pred_cur.tolist(),
            "resid_current_space": (s["dB"] - pred_cur).tolist(),
            "pred_duty_space": pred_duty.tolist() if ok else None,
            "resid_duty_space": (s["dB"] - pred_duty).tolist() if ok else None,
        })
    rc = np.array([r["resid_current_space"] for r in add_rows])
    rd = np.array([r["resid_duty_space"] for r in add_rows if r["resid_duty_space"]])
    meas_norm = np.linalg.norm([r["meas"] for r in add_rows], axis=1)
    print(f"\n  電流空間加算性 残差: RMS={np.sqrt((rc**2).mean(axis=0)).round(3).tolist()} µT, "
          f"max|.|={np.abs(rc).max(axis=0).round(3).tolist()} µT")
    print(f"    (|ΔB_meas| 中央値 {np.median(meas_norm):.1f} µT に対し "
          f"相対 {np.abs(rc).max()/np.median(meas_norm)*100:.1f}% max)")
    print(f"  duty空間加算性 残差: RMS={np.sqrt((rd**2).mean(axis=0)).round(3).tolist()} µT, "
          f"max|.|={np.abs(rd).max(axis=0).round(3).tolist()} µT")
    results["additivity"] = {
        "per_motor_a_prop": {m: motor_fits[m]["a_prop"].tolist() for m in motor_fits},
        "sum_over_4": (a_sum / 4).tolist(),
        "all_motor_a_prop": np.asarray(az0).tolist(),
        "ratio": (a_sum / 4 / np.asarray(az0)).tolist(),
        "rows": add_rows,
        "resid_current_rms": np.sqrt((rc ** 2).mean(axis=0)).tolist(),
        "resid_current_max": np.abs(rc).max(axis=0).tolist(),
        "resid_duty_rms": np.sqrt((rd ** 2).mean(axis=0)).tolist(),
        "resid_duty_max": np.abs(rd).max(axis=0).tolist(),
    }

    # ------------------------------------------------------------------
    print("\n" + "=" * 88)
    print("D) ①総電流モデル vs ③モーター別: 差動推力(ヨー操作)シナリオ")
    print("=" * 88)
    # 対角ペア差: FL+RR vs FR+RL (ccw/cw)
    pairA = a_props[0] + a_props[3]   # FL+RR
    pairB = a_props[1] + a_props[2]   # FR+RL
    dvec = (pairA - pairB) / 2        # δI/2 ずつ移すときの ΔB変化 [µT/A]
    dxy = float(np.hypot(dvec[0], dvec[1]))
    print(f"  per-motor a_prop 行列 [µT/A]:")
    for m, a in zip(("FL", "FR", "RL", "RR"), a_props):
        print(f"    {m}: {a.round(3).tolist()}  |a_xy|={np.hypot(a[0],a[1]):.2f}")
    print(f"  対角ペア差 (FL+RR − FR+RL)/2 = {dvec.round(3).tolist()} µT/A")
    print(f"  → 総電流一定のままペア間で δI [A] 再配分すると ①モデルは "
          f"|δB_xy| = {dxy:.2f}·δI µT を見逃す")
    for dI in (0.2, 0.5, 1.0):
        print(f"     δI={dI:.1f}A → 見逃し {dxy*dI:.2f} µT ≈ Yaw {math.degrees(math.atan2(dxy*dI, H_mean)):.2f}°")
    # 単機運転に①を当てたときの誤差 (極端な配分ズレの上限)
    print("\n  単機スイープに pooled①モデルを適用した残差 (配分ズレ上限の参考):")
    for mname, magg in motor_aggs.items():
        I = np.array([s["I"] for s in magg["steps"]])
        Y = np.array([s["dB"] for s in magg["steps"]])
        resid = Y - (I[:, None] * a_pool + b_pool)
        rxy = np.hypot(resid[:, 0], resid[:, 1])
        print(f"    {mname}: max|r_xy|={rxy.max():6.2f} µT (I≤{I.max():.1f}A) "
              f"≈ Yaw {math.degrees(math.atan2(rxy.max(), H_mean)):.1f}°")
    results["differential"] = {
        "pair_diff_vec_uT_per_A": dvec.tolist(), "pair_diff_xy_uT_per_A": dxy,
        "yaw_err_deg_per_A": math.degrees(math.atan2(dxy, H_mean)),
    }

    # ------------------------------------------------------------------
    print("\n" + "=" * 88)
    print("E) ドリフト・ヒステリシス・ノイズ床 (KF設計の根拠)")
    print("=" * 88)
    for label, agg in all_aggs.items():
        d = agg["drift"]
        print(f"  [{label}] 基準磁場ドリフト: |Δ|={d['total_uT']:.2f} µT / {d['span_s']:.0f}s "
              f"({d['rate_uT_per_min']:.2f} µT/min)  max step jump={d['max_jump_uT']:.2f} µT  "
              f"温度低下 {d['temp_drop_c']:.1f}°C")
    # ヒステリシス: 同duty up/down差
    hyst_all = []
    for label, agg in all_aggs.items():
        by_duty = {}
        for s in agg["steps"]:
            by_duty.setdefault(s["duty"], {})[s["leg"]] = s["dB"]
        diffs = [v["up"] - v["down"] for v in by_duty.values() if "up" in v and "down" in v]
        if diffs:
            diffs = np.array(diffs)
            hyst_all.append(np.abs(diffs).max(axis=0))
            print(f"  [{label}] up/down ヒステリシス max|Δ|: {np.abs(diffs).max(axis=0).round(3).tolist()} µT")
    # ノイズ床: measure窓内std (中央値)
    stds = []
    for label, agg in all_aggs.items():
        stds.append(np.median([s["std"] for s in agg["steps"]], axis=0))
    stds = np.array(stds)
    print(f"  measure窓内 dB std (中央値, 軸別): {stds.mean(axis=0).round(3).tolist()} µT")
    print(f"  → EMA(α=0.18)後の実効std ≈ ×{math.sqrt(0.18/(2-0.18)):.2f}")
    results["drift"] = {l: all_aggs[l]["drift"] for l in all_aggs}
    results["noise_floor_std_uT"] = stds.mean(axis=0).tolist()
    results["hysteresis_max_uT"] = np.array(hyst_all).max(axis=0).tolist() if hyst_all else None

    # motors-off の窓ごとノイズ床 (Yaw=0 ラン, baseline 窓内 std → ドリフト混入なし)
    run0 = load_run("sweep_20260612_202215")
    c0 = run0["cols"]
    mb = c0["phase"] == "baseline"
    win_stds = []
    for si in sorted({v for v in c0["step_idx"][mb] if not math.isnan(v)}):
        s = mb & (c0["step_idx"] == si)
        if s.sum() >= 5:
            win_stds.append([float(np.std(c0[f"b{a}_cor"][s])) for a in AXES])
    win_stds = np.array(win_stds)
    results["noise_floor_motors_off_window_std_uT"] = {
        "median": np.median(win_stds, axis=0).tolist(),
        "max": win_stds.max(axis=0).tolist(), "n_windows": len(win_stds)}
    print(f"  motors-off 窓内 b_cor std (Yaw=0): 中央値 {np.median(win_stds,axis=0).round(3).tolist()} µT")

    # ------------------------------------------------------------------
    print("\n" + "=" * 88)
    print("F) pooled 2次モデル / LUTブレークポイント / duty→電流fit (v2方式の出典データ)")
    print("=" * 88)
    # pooled 2次: 全域fit → 飛行帯残差 + leave-one-out
    Iq = np.array(I_pool)
    Yq = np.array(Y_pool)
    Cq, *_ = np.linalg.lstsq(np.column_stack([Iq**2, Iq, np.ones_like(Iq)]), Yq, rcond=None)
    quad = {"coef_c2_c1_c0": Cq.tolist(), "fb": {}, "loo_fb": {}}
    for label, agg in all_aggs.items():
        I = np.array([s["I"] for s in agg["steps"]])
        Y = np.array([s["dB"] for s in agg["steps"]])
        fbm = (I >= FLIGHT_BAND_A[0]) & (I <= FLIGHT_BAND_A[1])
        H = agg["horiz_uT"]
        r = Y - np.column_stack([I**2, I, np.ones_like(I)]) @ Cq
        rxy = np.hypot(r[:, 0], r[:, 1])
        quad["fb"][label] = {"rms_xy": float(np.sqrt((rxy[fbm]**2).mean())),
                             "max_xy": float(rxy[fbm].max()),
                             "yaw_max_deg": math.degrees(math.atan2(float(rxy[fbm].max()), H))}
        Il, Yl = [], []
        for l2, agg2 in all_aggs.items():
            if l2 == label:
                continue
            Il += [s["I"] for s in agg2["steps"]] + [agg2["idle"]]
            Yl += [s["dB"] for s in agg2["steps"]] + [np.zeros(3)]
        Il = np.array(Il)
        Cl, *_ = np.linalg.lstsq(np.column_stack([Il**2, Il, np.ones_like(Il)]), np.array(Yl), rcond=None)
        r2 = Y - np.column_stack([I**2, I, np.ones_like(I)]) @ Cl
        r2xy = np.hypot(r2[:, 0], r2[:, 1])
        quad["loo_fb"][label] = {"rms_xy": float(np.sqrt((r2xy[fbm]**2).mean())),
                                 "max_xy": float(r2xy[fbm].max()),
                                 "yaw_max_deg": math.degrees(math.atan2(float(r2xy[fbm].max()), H))}
        print(f"  [{label}] 2次pooled 飛行帯: RMS={quad['fb'][label]['rms_xy']:.3f} µT "
              f"max Yaw {quad['fb'][label]['yaw_max_deg']:.2f}° | "
              f"LOO: RMS={quad['loo_fb'][label]['rms_xy']:.3f} µT "
              f"max Yaw {quad['loo_fb'][label]['yaw_max_deg']:.2f}°")
    results["pooled_quadratic"] = quad

    # LUTブレークポイント: 4姿勢の同(duty,leg)ステップを平均 → 電流順, (idle, 0)起点
    idle_mean = float(np.mean([all_aggs[l]["idle"] for l in all_aggs]))
    lut_pts = [{"I": idle_mean, "dB": [0.0, 0.0, 0.0], "duty": 0.0, "leg": "idle"}]
    steps0 = all_aggs["Yaw=0"]["steps"]
    for i, s0 in enumerate(steps0):
        Is, dBs = [], []
        for label, agg in all_aggs.items():
            match = [t for t in agg["steps"]
                     if np.isclose(t["duty"], s0["duty"]) and t["leg"] == s0["leg"]]
            if match:
                Is.append(match[0]["I"])
                dBs.append(match[0]["dB"])
        lut_pts.append({"I": float(np.mean(Is)), "dB": np.mean(dBs, axis=0).tolist(),
                        "duty": s0["duty"], "leg": s0["leg"]})
    lut_pts.sort(key=lambda p: p["I"])
    results["lut_breakpoints"] = {"note": "4姿勢平均, (idle,0)起点, 電流昇順。区分線形補間用",
                                  "idle_mean_a": idle_mean, "points": lut_pts}
    print(f"  LUT: {len(lut_pts)}点 (I={lut_pts[0]['I']:.3f}–{lut_pts[-1]['I']:.3f} A)")

    # duty→単機電流 2次fit (I_active = c2 d² + c1 d + c0)
    d2c = {}
    for stem, mname in MOTOR_RUNS.items():
        magg = motor_aggs[mname]
        d = np.array([s["duty"] for s in magg["steps"]])
        Ia = np.array([s["I"] for s in magg["steps"]]) - magg["idle"]
        co, *_ = np.linalg.lstsq(np.column_stack([d**2, d, np.ones_like(d)]), Ia, rcond=None)
        rms = float(np.sqrt(((Ia - np.column_stack([d**2, d, np.ones_like(d)]) @ co)**2).mean()))
        d2c[mname] = {"c2_c1_c0": co.tolist(), "idle_a": magg["idle"], "rms_a": rms}
        print(f"  duty→I {mname}: c=[{co[0]:+.4f},{co[1]:+.4f},{co[2]:+.4f}] RMS={rms:.4f}A")
    results["duty_to_current_quadfit"] = d2c

    # ------------------------------------------------------------------ figures
    # 図1: 4姿勢の dB vs I (軸別) + pooled fit
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.5))
    colors = {"Yaw=0": "C0", "Yaw=90": "C1", "Yaw=180": "C2", "Yaw=-90": "C3"}
    for k, ax_name in enumerate(AXES):
        ax = axs[k]
        for label, agg in all_aggs.items():
            I = [s["I"] for s in agg["steps"]]
            Y = [s["dB"][k] for s in agg["steps"]]
            ax.scatter(I, Y, s=18, color=colors[label], label=label, alpha=0.8)
        xs = np.linspace(0, 4.5, 10)
        ax.plot(xs, a_pool[k] * xs + b_pool[k], "k--", lw=1.2, label="pooled fit")
        ax.axvspan(*FLIGHT_BAND_A, color="gray", alpha=0.12)
        ax.set_xlabel("総電流 I [A]")
        ax.set_ylabel(f"ΔB_{ax_name} [µT]")
        ax.grid(alpha=0.3)
        if k == 0:
            ax.legend(fontsize=8)
    fig.suptitle("全機スイープ 4姿勢: ΔB vs 総電流 (body座標, 姿勢間一貫性)")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "F1_orientation_consistency.png", dpi=140)
    plt.close(fig)

    # 図2: pooled補正後の残差 vs I
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
    for label, agg in all_aggs.items():
        I = np.array([s["I"] for s in agg["steps"]])
        Y = np.array([s["dB"] for s in agg["steps"]])
        r = Y - (I[:, None] * a_pool + b_pool)
        rxy = np.hypot(r[:, 0], r[:, 1])
        axs[0].scatter(I, rxy, s=18, color=colors[label], label=label, alpha=0.8)
        axs[1].scatter(I, np.degrees(np.arctan2(rxy, all_aggs[label]["horiz_uT"])),
                       s=18, color=colors[label], alpha=0.8)
    for ax, ylab in zip(axs, ("|r_xy| [µT]", "最悪Yaw誤差 [°]")):
        ax.axvspan(*FLIGHT_BAND_A, color="gray", alpha=0.12)
        ax.set_xlabel("総電流 I [A]")
        ax.set_ylabel(ylab)
        ax.grid(alpha=0.3)
    axs[0].legend(fontsize=8)
    fig.suptitle("pooled①モデル補正後の水平残差とYaw誤差換算")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "F2_pooled_residual.png", dpi=140)
    plt.close(fig)

    # 図3: 加算性
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.5))
    duties = [r["duty"] + (0.0 if r["leg"] == "up" else 0.0) for r in add_rows]
    for k, ax_name in enumerate(AXES):
        ax = axs[k]
        ax.plot([r["I_all"] for r in add_rows], [r["meas"][k] for r in add_rows],
                "o", ms=5, label="全機実測")
        ax.plot([r["I_all"] for r in add_rows], [r["pred_current_space"][k] for r in add_rows],
                "x", ms=6, label="Σ単機 (電流空間)")
        ax.plot([r["I_all"] for r in add_rows],
                [r["pred_duty_space"][k] if r["pred_duty_space"] else np.nan for r in add_rows],
                "+", ms=7, label="Σ単機 (duty一致)")
        ax.set_xlabel("総電流 I [A]")
        ax.set_ylabel(f"ΔB_{ax_name} [µT]")
        ax.grid(alpha=0.3)
        if k == 0:
            ax.legend(fontsize=8)
    fig.suptitle("加算性検証 (Yaw=0): 全機実測 vs Σ単機予測")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "F3_additivity.png", dpi=140)
    plt.close(fig)

    # 図4: per-motor xy平面ベクトル
    fig, ax = plt.subplots(figsize=(6, 6))
    for m, a in zip(("FL", "FR", "RL", "RR"), a_props):
        ax.annotate("", xy=(a[0], a[1]), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color="C0"))
        ax.text(a[0], a[1], f" {m}", fontsize=11)
    ax.annotate("", xy=(a_sum[0] / 4, a_sum[1] / 4), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color="C3", lw=2))
    ax.text(a_sum[0] / 4, a_sum[1] / 4, " mean", color="C3", fontsize=11)
    ax.set_xlabel("a_x [µT/A]")
    ax.set_ylabel("a_y [µT/A]")
    ax.grid(alpha=0.3)
    ax.set_title("モーター別 感度ベクトル a_m (xy平面, I_active空間)")
    ax.axis("equal")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "F4_per_motor_vectors.png", dpi=140)
    plt.close(fig)

    (OUT_DIR / "results.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=1, default=float))
    print(f"\n図とJSON → {OUT_DIR}")


if __name__ == "__main__":
    main()
