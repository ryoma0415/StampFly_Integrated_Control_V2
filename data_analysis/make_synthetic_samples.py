#!/usr/bin/env python3
"""テスト用の合成 samples CSV + meta JSON を生成する（実機なしで解析パイプラインを確認するため）。

pc_server の bracket-baseline 出力と同じ列・同じ phase 構成を、物理的に妥当な値で再現する:
  - 新フォーマット: step_idx / leg 列つき、往復スイープ 0.1→1.0→0.1（updown, 19 ステップ）
  - 電流ノイズ dB_cor ≈ a·I（a≈[26,26,6] µT/A, ①の実測に近い）+ 小ノイズ
  - down leg には小さなヒステリシス項（残留オフセット ~1.5 µT）→ 図⑩で見える
  - 基準磁場が掃引中にドリフト（duty とともに +x,−y,−z 方向へ ~µT 単位で移動）
  - 各 measure 行は前後ブラケット基準を時間補間して引いた dB_cor / bx_base_cor を持つ
  - 同じ stem の `<stem>_meta.json` を同時出力（pattern/duty_sequence/notes/battery/
    imu_temp_c範囲/baseline_points/baseline_jumps/baseline_flags 等）

使い方:  python make_synthetic_samples.py [出力CSVパス]
"""
import csv
import json
import math
import sys
import time
from pathlib import Path

FIELDS = [
    "t_s", "phase", "motors", "duty_cmd", "step_idx", "leg", "seq", "cv",
    "current_a", "vbat_v", "shunt_uv",
    "bx_raw", "by_raw", "bz_raw",
    "bx_cor", "by_cor", "bz_cor",
    "imu_temp_c",
    "roll_deg", "pitch_deg", "yaw_deg",
    "roll_rate", "pitch_rate", "yaw_rate",
    "mag_total_uT",
    "bx_base_cor", "by_base_cor", "bz_base_cor",
    "dB_cor_x", "dB_cor_y", "dB_cor_z",
]

DUTIES_UP = [round(0.1 * i, 2) for i in range(1, 11)]   # 0.1 → 1.0
# 往復パターン: 上り 0.1→1.0、下り 0.9→0.1（(step_idx, duty, leg) の列）
STEPS = ([(i, d, "up") for i, d in enumerate(DUTIES_UP)] +
         [(len(DUTIES_UP) + j, d, "down")
          for j, d in enumerate(reversed(DUTIES_UP[:-1]))])
DT = 0.05  # 20 Hz
HARD_IRON = [-88.0, 169.0, 143.0]      # raw = cor + hard-iron
BASE0_COR = [5.0, -33.0, -25.0]        # duty=0 motor-off 磁場 (µT)
DRIFT_PER_STEP = [0.30, -0.85, -0.85]  # 各duty後の基準が動く量 (µT/step) = ドリフト
A_COEF = [26.0, 26.0, 6.0]             # 電流ノイズ係数 µT/A
HYST_DOWN = [1.5, -1.2, 0.8]           # down leg の残留オフセット (µT) = ヒステリシス項（図⑩用）
JUMP_WARN_UT = 2.0                     # 隣接基準ジャンプの警告閾値 (µT)
IDLE_CURRENT = 0.16                    # motor-off アイドル電流 (A)
N = {"base": 40, "settle": 30, "measure": 50, "gap_settle": 10, "baseline": 30}


def _rng(seed):
    s = seed
    def r():
        nonlocal s
        s = (1103515245 * s + 12345) & 0x7FFFFFFF
        return s / 0x7FFFFFFF - 0.5
    return r


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/synthetic_samples.csv")
    rnd = _rng(7)
    rows = []
    t = 0.0
    seq = 0
    temp = 24.0

    def baseline_cor(step):  # step 0 = duty0 base, step k = after duty step k
        return [BASE0_COR[a] + DRIFT_PER_STEP[a] * step for a in range(3)]

    # baseline times/vectors を貯めてブラケット補間する
    baseline_t = {}
    baseline_vec = {}

    def emit(phase, duty, n, current, base_vec, dB=None, step_idx="", leg=""):
        nonlocal t, seq, temp
        ts = []
        for _ in range(n):
            seq += 1
            temp += 0.0015 + 0.002 * current  # ゆっくり昇温（電流依存）
            if dB is not None:
                cor = [base_vec[a] + dB[a] + 1.0 * rnd() for a in range(3)]
            else:
                cor = [base_vec[a] + 0.8 * rnd() for a in range(3)]
            raw = [cor[a] + HARD_IRON[a] for a in range(3)]
            row = {
                "t_s": round(t, 4), "phase": phase, "motors": "ALL", "duty_cmd": duty,
                "step_idx": step_idx, "leg": leg,
                "seq": seq, "cv": 1,
                "current_a": round(current + 0.03 * rnd(), 4),
                "vbat_v": round(4.2 - 0.05 * current + 0.01 * rnd(), 4),
                "shunt_uv": round((current) * 1e4, 1),
                "bx_raw": round(raw[0], 3), "by_raw": round(raw[1], 3), "bz_raw": round(raw[2], 3),
                "bx_cor": round(cor[0], 3), "by_cor": round(cor[1], 3), "bz_cor": round(cor[2], 3),
                "imu_temp_c": round(temp, 3),
                "roll_deg": round(0.2 * rnd(), 3), "pitch_deg": round(0.2 * rnd(), 3),
                "yaw_deg": round(30 + 0.5 * rnd(), 3),
                "roll_rate": round(0.01 * rnd(), 4), "pitch_rate": round(0.01 * rnd(), 4),
                "yaw_rate": round(0.01 * rnd(), 4),
                "mag_total_uT": round(math.sqrt(sum(c * c for c in cor)), 3),
                "bx_base_cor": "", "by_base_cor": "", "bz_base_cor": "",
                "dB_cor_x": "", "dB_cor_y": "", "dB_cor_z": "",
            }
            if phase == "measure" and base_vec is not None and dB is not None:
                row["bx_base_cor"], row["by_base_cor"], row["bz_base_cor"] = (round(base_vec[a], 3) for a in range(3))
                row["dB_cor_x"], row["dB_cor_y"], row["dB_cor_z"] = (round(cor[a] - base_vec[a], 4) for a in range(3))
            rows.append(row)
            ts.append(t)
            t += DT
        return ts

    # 初期 base (duty=0) = baseline step 0（step_idx/leg は空欄 = 実機フォーマットと同じ）
    ts = emit("base", 0.0, N["base"], current=IDLE_CURRENT, base_vec=baseline_cor(0))
    baseline_t[0] = sum(ts) / len(ts)
    baseline_vec[0] = baseline_cor(0)

    # 各 step (往復 0.1→1.0→0.1): settle, measure, gap_settle, baseline
    for k, (idx, duty, lg) in enumerate(STEPS, start=1):
        cur = IDLE_CURRENT + 4.0 * duty  # 4基相当
        emit("settle", duty, N["settle"], current=cur, base_vec=baseline_cor(k - 1),
             step_idx=idx, leg=lg)
        # measure: 暫定で baseline step k-1 を base_vec に（後でブラケット補間し直す）
        # down leg にはヒステリシス項 HYST_DOWN を加える
        dB = [A_COEF[a] * cur + (HYST_DOWN[a] if lg == "down" else 0.0) for a in range(3)]
        emit("measure", duty, N["measure"], current=cur,
             base_vec=baseline_cor(k - 1), dB=dB, step_idx=idx, leg=lg)
        emit("gap_settle", duty, N["gap_settle"], current=max(cur * 0.2, IDLE_CURRENT),
             base_vec=baseline_cor(k), step_idx=idx, leg=lg)
        ts = emit("baseline", duty, N["baseline"], current=IDLE_CURRENT,
                  base_vec=baseline_cor(k), step_idx=idx, leg=lg)
        baseline_t[k] = sum(ts) / len(ts)
        baseline_vec[k] = baseline_cor(k)

    # measure 行をブラケット補間で引き直す（pc_server と同じ処理）
    # 往復で同じ duty が2回出るため、duty ではなく step_idx でグループ化する
    by_step = {}
    for r in rows:
        if r["phase"] == "measure":
            by_step.setdefault(r["step_idx"], []).append(r)
    for k, (idx, duty, lg) in enumerate(STEPS, start=1):
        b0, t0 = baseline_vec[k - 1], baseline_t[k - 1]
        b1, t1 = baseline_vec[k], baseline_t[k]
        for r in by_step[idx]:
            frac = (r["t_s"] - t0) / (t1 - t0) if t1 > t0 else 0.5
            frac = min(1.0, max(0.0, frac))
            base_at = [b0[a] + (b1[a] - b0[a]) * frac for a in range(3)]
            cor = [r["bx_cor"], r["by_cor"], r["bz_cor"]]
            r["bx_base_cor"], r["by_base_cor"], r["bz_base_cor"] = (round(base_at[a], 3) for a in range(3))
            r["dB_cor_x"], r["dB_cor_y"], r["dB_cor_z"] = (round(cor[a] - base_at[a], 4) for a in range(3))

    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    # ---- meta JSON（pc_server の sweep_<stamp>_meta.json と同じフィールド構造）----
    vbats = [r["vbat_v"] for r in rows]
    temps = [r["imu_temp_c"] for r in rows]
    # baseline_points: 初期 base (index 0) と各 step 後の baseline（理想値ベース）
    baseline_points = [{
        "index": k,
        "duty": 0.0 if k == 0 else STEPS[k - 1][1],
        "t_s": round(baseline_t[k], 4),
        "n": N["base"] if k == 0 else N["baseline"],
        "cor": [round(v, 4) for v in baseline_vec[k]],
        "raw": [round(baseline_vec[k][a] + HARD_IRON[a], 4) for a in range(3)],
    } for k in sorted(baseline_t)]
    # baseline_jumps: 隣接 baseline 間のジャンプ。> JUMP_WARN_UT は baseline_flags へ
    baseline_jumps = []
    baseline_flags = []
    for k, (idx, duty, lg) in enumerate(STEPS, start=1):
        vec = [round(baseline_vec[k][a] - baseline_vec[k - 1][a], 4) for a in range(3)]
        jump = round(math.sqrt(sum(v * v for v in vec)), 4)
        entry = {"step_idx": idx, "duty": duty, "leg": lg, "jump_uT": jump, "jump_vec_uT": vec}
        baseline_jumps.append(entry)
        if jump > JUMP_WARN_UT:
            baseline_flags.append(entry)
    meta = {
        "schema": "stampfly_sweep_meta",
        "version": 1,
        "method": "bracketed_baseline",
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "aborted": False,
        "samples_csv": out.name,
        "motors": "ALL",
        "pattern": "updown",
        "duty_sequence": [{"step_idx": i, "duty": d, "leg": lg} for i, d, lg in STEPS],
        "notes": {"location": "synthetic", "orientation": "synthetic",
                  "memo": "make_synthetic_samples"},
        "battery": {"vbat_start_v": vbats[0], "vbat_end_v": vbats[-1],
                    "vbat_min_v": min(vbats)},
        "idle_current_a": IDLE_CURRENT,
        "imu_temp_c": {"start": temps[0], "end": temps[-1],
                       "min": min(temps), "max": max(temps)},
        "mag3d": {"offset": HARD_IRON,
                  "matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]},
        "baseline_points": baseline_points,
        "baseline_jump_warn_uT": JUMP_WARN_UT,
        "baseline_jumps": baseline_jumps,
        "baseline_flags": baseline_flags,
        "sample_count": len(rows),
    }
    stem = out.stem[:-len("_samples")] if out.stem.endswith("_samples") else out.stem
    meta_path = out.with_name(stem + "_meta.json")
    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")

    print(f"{len(rows)} 行を {out} に書き出しました"
          f"（updown {len(STEPS)} ステップ, phases: base/settle/measure/gap_settle/baseline, "
          f"down leg に ヒステリシス {HYST_DOWN} µT）")
    print(f"meta JSON を {meta_path} に書き出しました（baseline_flags: {len(baseline_flags)} 件）")


if __name__ == "__main__":
    main()
