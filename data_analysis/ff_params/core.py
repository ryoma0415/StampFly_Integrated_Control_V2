"""FFプロファイル抽出コア (純粋関数: 8ラン → プロファイルdict).

集計・フィットは analyze_feasibility_20260612.py から**そのまま**移植している
(aggregate / fit_affine / fit_prop / LUT生成 / duty→電流2次fit)。
数値挙動を変えないこと (受入テスト: 6/12 results.json と相対誤差 <1e-6 で一致)。
仕様: docs/ff_pipeline_design.md §2-3。
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path

import numpy as np

AXES = ("x", "y", "z")
MOTORS = ("FL", "FR", "RL", "RR")
ALL_MOTOR_KEY = "FL+FR+RL+RR"
# 全機ラン4姿勢の正準ラベルと処理順 (feasibility スクリプトの ALL_RUNS 順に対応)
CANON_ORIENTATIONS = ("Yaw=0°", "Yaw=90°", "Yaw=±180°", "Yaw=-90°")

TOOL_VERSION = "1.0"


def normalize_orientation(s: str) -> str:
    """表記ゆれ吸収: 'Yaw=+-180°' / 'Yaw=±180°' → 'Yaw=±180°'。"""
    return (s or "").strip().replace("+-", "±").replace("+/-", "±")


# ---------------------------------------------------------------- load --------
def _f(v):
    if v is None or v == "":
        return math.nan
    try:
        return float(v)
    except ValueError:
        return math.nan


def load_run(stem: str, results_dir: Path) -> dict:
    """sweep_<stem>_{meta.json,samples.csv} を読み込む (feasibility の load_run 移植)。"""
    results_dir = Path(results_dir)
    meta = json.loads((results_dir / f"{stem}_meta.json").read_text())
    rows = list(csv.DictReader((results_dir / f"{stem}_samples.csv").open()))
    cols = {}
    for key in rows[0].keys():
        if key in ("phase", "motors", "leg"):
            cols[key] = np.array([r.get(key) or "" for r in rows], dtype=object)
        else:
            cols[key] = np.array([_f(r.get(key, "")) for r in rows], dtype=float)
    return {"meta": meta, "cols": cols, "stem": stem}


def aggregate(run: dict) -> dict:
    """measure行を (duty, leg) 別に集計: 平均電流, 平均dB, 窓内std, n. (忠実移植)"""
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
    """軸別 affine LS: y = a*I + b。anchor=(I0, 0) を含められる。(忠実移植)"""
    I = np.asarray(I, float)
    Y = np.asarray(Y, float)
    if anchor is not None:
        I = np.append(I, anchor[0])
        Y = np.vstack([Y, np.zeros(3)])
    A = np.column_stack([I, np.ones_like(I)])
    coef, *_ = np.linalg.lstsq(A, Y, rcond=None)
    return coef[0], coef[1]   # a[3], b[3]


def fit_prop(I_active, Y):
    """原点拘束 y = a*(I-idle)。(忠実移植)"""
    I = np.asarray(I_active, float)
    Y = np.asarray(Y, float)
    denom = (I * I).sum()
    return (I[:, None] * Y).sum(axis=0) / denom


# ---------------------------------------------------------------- fits --------
def build_lut_points(all_aggs: dict) -> tuple[float, list[dict]]:
    """LUTブレークポイント: 4姿勢の同(duty,leg)ステップを平均 → 電流順, (idle, 0)起点。

    analyze_feasibility_20260612.py line 459-476 の忠実移植。
    all_aggs は正準姿勢順 (Yaw=0° 先頭) の dict であること (先頭がテンプレート)。
    """
    labels = list(all_aggs.keys())
    idle_mean = float(np.mean([all_aggs[l]["idle"] for l in all_aggs]))
    lut_pts = [{"I": idle_mean, "dB": [0.0, 0.0, 0.0], "duty": 0.0, "leg": "idle"}]
    steps0 = all_aggs[labels[0]]["steps"]
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
    return idle_mean, lut_pts


def fit_duty_to_current(agg: dict) -> dict:
    """duty→単機電流 2次fit (I_active = c2 d² + c1 d + c0)。

    analyze_feasibility_20260612.py line 479-488 の忠実移植 (1モーター分)。
    """
    d = np.array([s["duty"] for s in agg["steps"]])
    Ia = np.array([s["I"] for s in agg["steps"]]) - agg["idle"]
    co, *_ = np.linalg.lstsq(np.column_stack([d**2, d, np.ones_like(d)]), Ia, rcond=None)
    rms = float(np.sqrt(((Ia - np.column_stack([d**2, d, np.ones_like(d)]) @ co)**2).mean()))
    return {"c2_c1_c0": co.tolist(), "idle_a": agg["idle"], "rms_a": rms}


# ---------------------------------------------------------------- misc --------
def assert_finite(obj, path: str = "profile") -> None:
    """dict/list を再帰的に走査し、非有限 float (NaN/±Inf) を JSONパス付きで報告。

    NaN は JSON に非標準リテラルとして書き出されてしまうため、出力前に検出する。
    None (JSON null) は許容。
    """
    if isinstance(obj, dict):
        for k, v in obj.items():
            assert_finite(v, f"{path}.{k}")
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            assert_finite(v, f"{path}[{i}]")
    elif isinstance(obj, float) and not math.isfinite(obj):
        raise ValueError(f"プロファイルに非有限値が含まれる: {path} = {obj!r}")


def _vbat_of(meta: dict):
    """meta.battery.vbat_start_v → float。欠損/非数値は None (JSON null)。"""
    v = (meta.get("battery") or {}).get("vbat_start_v")
    return float(v) if isinstance(v, (int, float)) and math.isfinite(float(v)) else None


def mag3d_hash(mag3d: dict) -> str:
    """mag3d の正準文字列 (%.9g join ',') の SHA256。

    正準文字列 = offset 3値 + matrix 9値 (行優先) を各 %.9g で整形し ',' 連結。
    """
    vals = list(mag3d["offset"])
    for row in mag3d["matrix"]:
        vals.extend(row)
    canon = ",".join("%.9g" % float(v) for v in vals)
    return "sha256:" + hashlib.sha256(canon.encode("ascii")).hexdigest()


def _mag3d_consistent(m1: dict, m2: dict, rtol: float = 1e-6) -> bool:
    """offset/matrix の相対差 > rtol なら不一致。"""
    a = np.array(list(m1["offset"]) + [v for row in m1["matrix"] for v in row], float)
    b = np.array(list(m2["offset"]) + [v for row in m2["matrix"] for v in row], float)
    denom = np.maximum(np.maximum(np.abs(a), np.abs(b)), 1e-12)
    return bool((np.abs(a - b) / denom <= rtol).all())


def _date_of(meta: dict) -> str:
    """meta.created_at → 'YYYY-MM-DD'。"""
    s = str(meta.get("created_at", ""))
    return s[:10] if len(s) >= 10 else ""


# ---------------------------------------------------------------- classify ----
def classify_runs(runs: list[dict]) -> tuple[list[dict], dict]:
    """8ランを全機×4姿勢 / 単機×4 に自動分類する。

    Returns:
        all_runs: 正準姿勢順 (Yaw=0°, 90°, ±180°, −90°) の全機ラン4本
        motor_runs: {"FL": run, "FR": run, "RL": run, "RR": run}
    Raises:
        ValueError: 構造が合わない場合 (内訳をメッセージに含める)、
                    または中断ラン (meta.aborted=true) が含まれる場合
    """
    aborted = [r["stem"] for r in runs if r["meta"].get("aborted")]
    if aborted:
        raise ValueError("中断ラン (meta.aborted=true) が含まれています。"
                         "完走ランのみで8本を構成してください: " + ", ".join(aborted))

    all_list, motor_map, others = [], {}, []
    for run in runs:
        motors = run["meta"].get("motors", "")
        if motors == ALL_MOTOR_KEY:
            all_list.append(run)
        elif motors in MOTORS:
            motor_map.setdefault(motors, []).append(run)
        else:
            others.append(run)

    def _breakdown():
        lines = [f"  全機ラン: {len(all_list)}本 " +
                 str([(r['stem'], normalize_orientation((r['meta'].get('notes') or {}).get('orientation', '')))
                      for r in all_list])]
        for m in MOTORS:
            lines.append(f"  単機{m}: {len(motor_map.get(m, []))}本 "
                         + str([r["stem"] for r in motor_map.get(m, [])]))
        if others:
            lines.append("  分類不能: " + str([(r["stem"], r["meta"].get("motors")) for r in others]))
        return "\n".join(lines)

    if len(all_list) != 4 or others or any(len(motor_map.get(m, [])) != 1 for m in MOTORS):
        raise ValueError("ラン構成が仕様 (全機×4 + 単機FL/FR/RL/RR×1) と不一致:\n" + _breakdown())

    # 姿勢の検証と正準順への並べ替え
    orient_of = {}
    for run in all_list:
        o = normalize_orientation((run["meta"].get("notes") or {}).get("orientation", ""))
        orient_of[run["stem"]] = o
    orients = [orient_of[r["stem"]] for r in all_list]
    if len(set(orients)) != 4:
        raise ValueError("全機ラン4本の notes.orientation が4種の別姿勢になっていない:\n" + _breakdown())

    def _key(run):
        o = orient_of[run["stem"]]
        try:
            return (0, CANON_ORIENTATIONS.index(o))
        except ValueError:
            return (1, run["stem"])
    all_runs = sorted(all_list, key=_key)
    motor_runs = {m: motor_map[m][0] for m in MOTORS}
    return all_runs, motor_runs


# ---------------------------------------------------------------- extract -----
def extract_profile(all_runs: list[dict], motor_runs: dict, *,
                    name: str, memo: str, source_dir: str = "") -> tuple[dict, dict]:
    """8ラン → FFプロファイルdict (schema stampfly_ff_profile v1) + 検証用内部値。

    all_runs: 正準姿勢順の全機ラン4本 / motor_runs: {"FL":run, ...}
    Returns: (profile, internals)
      internals: 受入テスト・プロット用の生値 (lut_pts(duty/leg付き), pooled a/b,
                 a_prop dict, d2c dict, all_aggs, motor_aggs)
    """
    warnings: list[str] = []

    # --- 集計 (正準姿勢順の dict — feasibility の all_aggs と同構造・同順) ---
    all_aggs: dict[str, dict] = {}
    orient_labels: dict[str, str] = {}
    for run in all_runs:
        label = normalize_orientation((run["meta"].get("notes") or {}).get("orientation", ""))
        orient_labels[run["stem"]] = label
        all_aggs[label] = aggregate(run)
    motor_aggs = {m: aggregate(motor_runs[m]) for m in MOTORS}

    for m, agg in list(all_aggs.items()) + list(motor_aggs.items()):
        if math.isnan(agg["idle"]):
            raise ValueError(f"meta.idle_current_a が欠落 ({m})")

    # --- 姿勢別 affine (全域, idleアンカー込み) → 姿勢間ばらつき ---
    a_rows = []
    for label, agg in all_aggs.items():
        I = [s["I"] for s in agg["steps"]]
        Y = [s["dB"] for s in agg["steps"]]
        a, _b = fit_affine(I, Y, anchor=(agg["idle"], 0.0))
        a_rows.append(a)
    a_mat = np.array(a_rows)                                  # (4,3)
    orientation_slope_std = a_mat.std(axis=0)

    # --- pooled affine (全域・アンカー込み) = affine_ref ---
    # feasibility スクリプト同様、各姿勢の duty-step 点 + idleアンカー(1点/姿勢) を結合
    I_pool, Y_pool = [], []
    for label, agg in all_aggs.items():
        I_pool += [s["I"] for s in agg["steps"]] + [agg["idle"]]
        Y_pool += [s["dB"] for s in agg["steps"]] + [np.zeros(3)]
    a_pool, b_pool = fit_affine(I_pool, Y_pool)
    # フィット品質: pooled 残差RMS (アンカー点を除く duty-step 点上, 軸別)
    Is = np.array([s["I"] for agg in all_aggs.values() for s in agg["steps"]])
    Ys = np.array([s["dB"] for agg in all_aggs.values() for s in agg["steps"]])
    resid = Ys - (Is[:, None] * a_pool + b_pool)
    affine_fit_rms = np.sqrt((resid ** 2).mean(axis=0))

    # --- LUT (4姿勢平均 + (idle_mean,0)起点, 電流昇順) ---
    idle_mean, lut_pts = build_lut_points(all_aggs)

    # --- 単機: 原点拘束 a_m (I_active空間) + duty→電流 2次fit ---
    a_prop = {}
    d2c = {}
    for m in MOTORS:
        agg = motor_aggs[m]
        I = np.array([s["I"] for s in agg["steps"]])
        Y = np.array([s["dB"] for s in agg["steps"]])
        a_prop[m] = fit_prop(I - agg["idle"], Y)
        d2c[m] = fit_duty_to_current(agg)

    a_props = np.array([a_prop[m] for m in MOTORS])           # (4,3) FL,FR,RL,RR
    a_bar = a_props.mean(axis=0)
    a_tilde = {m: (a_prop[m] - a_bar) for m in MOTORS}
    # 対角ペア差 (FL+RR − FR−RL)/2 の水平ノルム
    dvec = (a_props[0] + a_props[3] - a_props[1] - a_props[2]) / 2
    pair_diff_xy = float(np.hypot(dvec[0], dvec[1]))

    # --- 加算性クロージャ: Σa_m/4 ÷ 全機(Yaw=0, 原点拘束) ---
    label0 = list(all_aggs.keys())[0]
    agg0 = all_aggs[label0]
    I0 = np.array([s["I"] for s in agg0["steps"]])
    Y0 = np.array([s["dB"] for s in agg0["steps"]])
    a_all_prop = fit_prop(I0 - agg0["idle"], Y0)
    additivity_closure = a_props.sum(axis=0) / 4 / a_all_prop

    # --- ノイズ床・ヒステリシス (KF設計用) ---
    stds = []
    hyst_all = []
    for label, agg in all_aggs.items():
        stds.append(np.median([s["std"] for s in agg["steps"]], axis=0))
        by_duty = {}
        for s in agg["steps"]:
            by_duty.setdefault(s["duty"], {})[s["leg"]] = s["dB"]
        diffs = [v["up"] - v["down"] for v in by_duty.values() if "up" in v and "down" in v]
        if diffs:
            hyst_all.append(np.abs(np.array(diffs)).max(axis=0))
    noise_floor = np.array(stds).mean(axis=0)
    hysteresis = np.array(hyst_all).max(axis=0) if hyst_all else np.full(3, math.nan)

    # --- mag3d バインディング (全機ラン第1本の値を採用) ---
    mag3d_ref = all_runs[0]["meta"]["mag3d"]
    consistent = True
    for run in all_runs + [motor_runs[m] for m in MOTORS]:
        if not _mag3d_consistent(mag3d_ref, run["meta"]["mag3d"]):
            consistent = False
            warnings.append(f"mag3d が全機ラン第1本と不一致: {run['stem']}")
    binding = {
        "mag3d": {"offset": [float(v) for v in mag3d_ref["offset"]],
                  "matrix": [[float(v) for v in row] for row in mag3d_ref["matrix"]]},
        "mag3d_hash": mag3d_hash(mag3d_ref),
        "consistent_across_runs": consistent,
    }

    # --- provenance ---
    baseline_flag_count = 0
    for run in all_runs + [motor_runs[m] for m in MOTORS]:
        baseline_flag_count += len(run["meta"].get("baseline_flags") or [])
    all_motor_prov = []
    for run in all_runs:
        meta = run["meta"]
        all_motor_prov.append({
            "stem": run["stem"],
            "orientation": orient_labels[run["stem"]],
            "location": (meta.get("notes") or {}).get("location", ""),
            "vbat_start_v": _vbat_of(meta),
            "created_at": meta.get("created_at", ""),
        })
    single_motor_prov = []
    for m in MOTORS:
        meta = motor_runs[m]["meta"]
        single_motor_prov.append({
            "stem": motor_runs[m]["stem"],
            "motor": m,
            "vbat_start_v": _vbat_of(meta),
            "created_at": meta.get("created_at", ""),
        })
    dates = sorted({_date_of(r["meta"]) for r in all_runs + [motor_runs[m] for m in MOTORS]}
                   - {""})
    acquired_span = [dates[0], dates[-1]] if dates else ["", ""]

    profile = {
        "schema": "stampfly_ff_profile",
        "version": 1,
        "name": name,
        "memo": memo,
        "created_at": datetime.now().astimezone().isoformat(),
        "provenance": {
            "tool": "make_ff_profile.py",
            "tool_version": TOOL_VERSION,
            "source_dir": source_dir,
            "all_motor_runs": all_motor_prov,
            "single_motor_runs": single_motor_prov,
            "acquired_span": acquired_span,
        },
        "binding": binding,
        "method_a": {
            "lut": {
                "i_idle_a": idle_mean,
                "points": [{"i_a": float(p["I"]), "db": [float(v) for v in p["dB"]]}
                           for p in lut_pts],
            },
            "affine_ref": {"a": a_pool.tolist(), "b": b_pool.tolist()},
        },
        "method_b": {
            "a_m": {m: a_prop[m].tolist() for m in MOTORS},
            "a_bar": a_bar.tolist(),
            "a_tilde": {m: a_tilde[m].tolist() for m in MOTORS},
            "duty_to_current": {
                m: {"c2": d2c[m]["c2_c1_c0"][0], "c1": d2c[m]["c2_c1_c0"][1],
                    "c0": d2c[m]["c2_c1_c0"][2], "rms_a": d2c[m]["rms_a"]}
                for m in MOTORS
            },
            "pair_diff_xy_uT_per_A": pair_diff_xy,
        },
        "stats": {
            "noise_floor_std_uT": noise_floor.tolist(),
            "hysteresis_max_uT": hysteresis.tolist(),
            "additivity_closure": additivity_closure.tolist(),
            "orientation_slope_std": orientation_slope_std.tolist(),
        },
        "quality": {
            "baseline_flag_count": baseline_flag_count,
            "affine_fit_rms_uT": affine_fit_rms.tolist(),
            "warnings": warnings,
        },
    }
    assert_finite(profile)   # NaN/Inf の JSON 混入を出力前に検出 (C12)
    internals = {
        "all_aggs": all_aggs,
        "motor_aggs": motor_aggs,
        "lut_pts": lut_pts,
        "idle_mean": idle_mean,
        "a_pool": a_pool, "b_pool": b_pool,
        "a_prop": a_prop,
        "d2c": d2c,
        "a_mat": a_mat,
    }
    return profile, internals
