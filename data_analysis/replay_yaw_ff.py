#!/usr/bin/env python3
"""FF補正 + 4状態EKF のオフラインリプレイ (スイープCSVで検証).

仕様: docs/ff_pipeline_design.md §5.5 (EKF数式) / yaw_estimation_ff_two_methods.md §2-4。

入力:
  --profile <ff_profiles/*.json>  stampfly_ff_profile v1
  --sweep <stem or csvパス>       sweep_*_samples.csv (静置スイープ)
  --method A|B                     FF方式 (既定 B)

CSV→信号のマッピング:
  bx_cor/by_cor/bz_cor = b_cal (mag3d適用後 body座標)
  current_a            = I_total
  duty_cmd + motors列  → d_m 復元 ('FL+FR+RL+RR'=全機同duty / 単機はそのモーターのみ)
  yaw_rate             = ジャイロ入力 ω_z [rad/s] (アンカー窓平均を起動offsetとして減算)
  roll_deg/pitch_deg   → レベル化 (levelMagVector, ff_two_methods §3.2 の符号のまま)

流れ: アンカー(先頭のモーター停止2s窓: B0, I_idle, B0_horiz, ψ0) →
  各行: ΔB̂=LUT(I_total)+Σã_m·δI_m → b_corr=b_cal−ΔB̂ → EMA(α=0.18, fresh時のみ)
  → レベル化 z=(ℓx,ℓy) → EKF予測(毎行)+更新(fresh時, 適応R+ゲート)。
  非補正系 (b_cal そのままのEMA) も並走させ磁気yawを比較。

出力: data_analysis/graphs/replay_<stem>/ に時系列PNG
  (補正前/後の磁気yaw・ψ_est・b_m・NIS) + コンソール要約
  (静置データなので ψ_est の変動 < 15° を確認)。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.font_manager as fm  # noqa: E402

_INSTALLED = {f.name for f in fm.fontManager.ttflist}
for _jp in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic", "Yu Gothic",
            "Noto Sans CJK JP", "IPAexGothic", "Apple SD Gothic Neo", "Arial Unicode MS"):
    if _jp in _INSTALLED:
        plt.rcParams["font.family"] = _jp
        break
plt.rcParams["axes.unicode_minus"] = False

HERE = Path(__file__).resolve().parent
DEFAULT_RESULTS_DIR = HERE.parent / "pc_server" / "sweep_results"

MOTORS = ("FL", "FR", "RL", "RR")
D2R = math.pi / 180.0

# ---- 推定器定数 (ff_pipeline_design.md §5.5 / ff_two_methods 付録B, rad系に換算) ----
EMA_ALPHA = 0.18
Q_PSI = 5e-4 * D2R ** 2          # deg²/s → rad²/s
Q_BG = 1e-8 * D2R ** 2           # (°/s)²/s → (rad/s)²/s
Q_BM = 0.02                      # µT²/s
TAU_BM = 120.0                   # s
P0 = np.diag([(10 * D2R) ** 2, (0.5 * D2R) ** 2, 4.0 ** 2, 4.0 ** 2])
R_BASE = 4.0                     # µT² = (2.0µT)²
SIGMA_RZ = 3.5                   # µT
KAPPA_FF = 0.03
TAU_RESID = 0.05                 # s
SIGMA_DIFF_COEF = 0.3 * 30.0     # µT/A (0.3 × |δ_xy|≈30µT/A)
NIS_SOFT = 5.99                  # χ²₂(95%)
NIS_HARD = 13.8                  # χ²₂(99.9%)
NORM_GATE_SOFT = 8.0             # µT (8-20µT は R膨張)
NORM_GATE_HARD = 20.0            # µT
Z_GATE = 12.0                    # µT
TILT_GATE_DEG = 25.0
BM_FREEZE_UT = 20.0
DBM_WARN_RATE = 0.3              # µT/s
DBM_WARN_HOLD = 10.0             # s
REJECT_INFLATE_AFTER = 3.0       # s
REJECT_INFLATE_RATE = 1.02       # /s
REJECT_INFLATE_CAP = 10.0        # ×P0


def _f(v):
    if v is None or v == "":
        return math.nan
    try:
        return float(v)
    except ValueError:
        return math.nan


def level_mag(m, roll_rad, pitch_rad):
    """チルト補償 (levelMagVector, ff_two_methods §3.2 の符号のまま)。"""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    return np.array([
        m[0] * cp + m[2] * sp,
        m[0] * sr * sp + m[1] * cr + m[2] * sr * cp,
        -m[0] * cr * sp + m[1] * sr + m[2] * cr * cp,
    ])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class FfModel:
    """プロファイルから ΔB̂ を計算 (方式A: LUT / 方式B: +差動項)。"""

    def __init__(self, profile: dict, method: str):
        self.method = method
        pts = profile["method_a"]["lut"]["points"]
        self.lut_i = np.array([p["i_a"] for p in pts])
        self.lut_db = np.array([p["db"] for p in pts])
        self.iid_ref = float(profile["method_a"]["lut"]["i_idle_a"])
        self.a_ref = np.array(profile["method_a"]["affine_ref"]["a"])
        mb = profile["method_b"]
        self.a_tilde = {m: np.array(mb["a_tilde"][m]) for m in MOTORS}
        self.quad = {m: (mb["duty_to_current"][m]["c2"],
                         mb["duty_to_current"][m]["c1"],
                         mb["duty_to_current"][m]["c0"]) for m in MOTORS}

    def lut(self, i_total: float) -> np.ndarray:
        """区分線形LUT補間。範囲外は端区間の傾きで外挿。"""
        I, dB = self.lut_i, self.lut_db
        if i_total <= I[0]:
            k = 0
        elif i_total >= I[-1]:
            k = len(I) - 2
        else:
            k = int(np.searchsorted(I, i_total) - 1)
        t = (i_total - I[k]) / (I[k + 1] - I[k])
        return dB[k] + t * (dB[k + 1] - dB[k])

    def delta_b(self, i_total: float, duty: dict, i_idle: float):
        """ΔB̂ と |δI|max を返す (方式Aは差動項なし)。"""
        db = self.lut(i_total)
        di_max = 0.0
        if self.method == "B":
            i_hat = {}
            for m in MOTORS:
                d = duty.get(m, 0.0)
                c2, c1, c0 = self.quad[m]
                i_hat[m] = c2 * d * d + c1 * d + c0 if d > 0.0 else 0.0
            s_sum = sum(i_hat.values())
            i_active = i_total - i_idle
            if s_sum >= 0.05:                    # ΣÎ<0.05A なら差動項0
                s = i_active / s_sum
                diff = np.zeros(3)
                for m in MOTORS:
                    d_i = s * i_hat[m] - i_active / 4.0
                    di_max = max(di_max, abs(d_i))
                    diff += self.a_tilde[m] * d_i
                db = db + diff
        return db, di_max


def load_samples(csv_path: Path) -> list[dict]:
    rows = []
    for r in csv.DictReader(csv_path.open()):
        rows.append({
            "t": _f(r["t_s"]),
            "phase": r.get("phase") or "",
            "motors": r.get("motors") or "",
            "duty_cmd": _f(r.get("duty_cmd")),
            "cur": _f(r.get("current_a")),
            "b_cal": np.array([_f(r.get("bx_cor")), _f(r.get("by_cor")), _f(r.get("bz_cor"))]),
            "b_raw": (r.get("bx_raw"), r.get("by_raw"), r.get("bz_raw")),
            "roll": _f(r.get("roll_deg")) * D2R,
            "pitch": _f(r.get("pitch_deg")) * D2R,
            "wz": _f(r.get("yaw_rate")),
        })
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="FF補正+EKF オフラインリプレイ")
    ap.add_argument("--profile", required=True, help="ff_profiles/<name>.json")
    ap.add_argument("--sweep", required=True, help="stem または samples.csv パス")
    ap.add_argument("--method", choices=("A", "B"), default="B", help="FF方式 (既定 B)")
    ap.add_argument("--results-dir", default=str(DEFAULT_RESULTS_DIR))
    args = ap.parse_args(argv)

    profile = json.loads(Path(args.profile).read_text())
    sw = Path(args.sweep)
    if sw.suffix == ".csv" and sw.exists():
        csv_path = sw
        stem = sw.name.replace("_samples.csv", "")
    else:
        stem = args.sweep
        csv_path = Path(args.results_dir) / f"{stem}_samples.csv"
    rows = load_samples(csv_path)
    ff = FfModel(profile, args.method)

    # ---------------- アンカー (先頭のモーター停止 2s窓: §5.3 のリプレイ版) --------
    t0 = rows[0]["t"]
    anchor = [r for r in rows if r["t"] <= t0 + 2.0
              and r["duty_cmd"] == 0.0 and r["cur"] < 0.3]
    if len(anchor) < 10:
        print("エラー: 先頭にモーター停止2s窓が見つからない (アンカー不可)", file=sys.stderr)
        return 2
    B0 = np.mean([r["b_cal"] for r in anchor], axis=0)
    I_idle = float(np.mean([r["cur"] for r in anchor]))
    roll0 = float(np.mean([r["roll"] for r in anchor]))
    pitch0 = float(np.mean([r["pitch"] for r in anchor]))
    wz_off = float(np.mean([r["wz"] for r in anchor]))    # 起動ジャイロoffset相当
    B0_lev = level_mag(B0, roll0, pitch0)
    B0_horiz = B0_lev[:2]
    norm_B0 = float(np.linalg.norm(B0))
    psi0 = 0.0                                            # リプレイはアンカーyaw基準=0
    yaw_mag0 = math.atan2(B0_horiz[1], B0_horiz[0])       # 磁気yawの基準

    # ---------------- EKF 初期化 ------------------------------------------------
    x = np.array([psi0, 0.0, 0.0, 0.0])                   # [ψ, b_g, b_mx, b_my]
    P = P0.copy()
    Qd = np.diag([Q_PSI, Q_BG, Q_BM, Q_BM])

    ema_corr = None
    ema_uncorr = None
    prev_raw = None
    prev_t = rows[0]["t"]
    prev_fresh_t = None
    prev_fresh_cur = None
    reject_since = None
    bm_warn_since = None
    prev_bm = None
    gate_counts = {b: 0 for b in range(7)}
    n_update = n_reject = 0
    n_skipped_nan = 0

    log = {k: [] for k in ("t", "psi", "bmx", "bmy", "nis", "yaw_corr", "yaw_uncorr",
                           "cur", "dbn", "gate", "res_xy", "innov", "measure")}

    for r in rows:
        dt = min(max(r["t"] - prev_t, 0.0), 0.2)
        prev_t = r["t"]

        # ---- 予測 (毎行, ジャイロ入力 = yaw_rate − 起動offset) ----
        wz = r["wz"] - wz_off
        tau_f = 1.0 - dt / TAU_BM
        x[0] = wrap(x[0] + (wz - x[1]) * dt)
        x[2] *= tau_f
        x[3] *= tau_f
        F = np.array([[1.0, -dt, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, tau_f, 0.0],
                      [0.0, 0.0, 0.0, tau_f]])
        P = F @ P @ F.T + Qd * dt

        # ---- fresh 判定 (生磁気値が前回から変化したサンプルのみ) ----
        fresh = r["b_raw"] != prev_raw
        prev_raw = r["b_raw"]
        # 欠損値ガード: cur/b_cal/roll/pitch が非有限の行は fresh 処理をスキップ (C14)
        if fresh and not (math.isfinite(r["cur"]) and np.isfinite(r["b_cal"]).all()
                          and math.isfinite(r["roll"]) and math.isfinite(r["pitch"])):
            n_skipped_nan += 1
            fresh = False
        gate_bits = 0
        nis_val = math.nan
        innov_val = math.nan
        if fresh:
            # FF補正 → EMA (補正系/非補正系で状態分離)
            duty = {m: (r["duty_cmd"] if m in r["motors"].split("+") else 0.0)
                    for m in MOTORS}
            dB_hat, di_max = ff.delta_b(r["cur"], duty, I_idle)
            b_corr = r["b_cal"] - dB_hat
            ema_corr = b_corr if ema_corr is None else \
                EMA_ALPHA * b_corr + (1 - EMA_ALPHA) * ema_corr
            ema_uncorr = r["b_cal"] if ema_uncorr is None else \
                EMA_ALPHA * r["b_cal"] + (1 - EMA_ALPHA) * ema_uncorr

            lev_c = level_mag(ema_corr, r["roll"], r["pitch"])
            lev_u = level_mag(ema_uncorr, r["roll"], r["pitch"])
            z = lev_c[:2]

            # ---- 適応R (§5.5 / ff_two_methods §2.†) ----
            # dt_fresh: fresh サンプル間の経過時間 (P膨張・db_m/dt もこれを使う)
            dt_fresh = (r["t"] - prev_fresh_t) if prev_fresh_t is not None else dt
            if prev_fresh_t is not None and r["t"] > prev_fresh_t:
                didt = abs(r["cur"] - prev_fresh_cur) / (r["t"] - prev_fresh_t)
            else:
                didt = 0.0
            prev_fresh_t, prev_fresh_cur = r["t"], r["cur"]
            sigma_ff = KAPPA_FF * math.hypot(dB_hat[0], dB_hat[1])
            sigma_slew = math.hypot(ff.a_ref[0], ff.a_ref[1]) * didt * TAU_RESID
            sigma_diff = SIGMA_DIFF_COEF * di_max if args.method == "B" else 0.0
            tilt = math.acos(max(-1.0, min(1.0, math.cos(r["roll"]) * math.cos(r["pitch"]))))
            r_eff = (R_BASE + sigma_ff ** 2 + sigma_slew ** 2 + sigma_diff ** 2
                     + (math.sin(tilt) * SIGMA_RZ) ** 2)

            # ---- ゲート (bit: §5.5) ----
            reject = False
            skip = False
            if math.degrees(tilt) > TILT_GATE_DEG:            # bit4
                gate_bits |= 1 << 4
                skip = True
            norm_dev = abs(float(np.linalg.norm(ema_corr)) - norm_B0)
            if norm_dev > NORM_GATE_HARD:                     # bit2
                gate_bits |= 1 << 2
                reject = True
            elif norm_dev > NORM_GATE_SOFT:                   # 8-20µT は R膨張 (bit0扱い)
                gate_bits |= 1 << 0
                r_eff *= (norm_dev / NORM_GATE_SOFT) ** 2
            if abs(float(ema_corr[2]) - float(B0[2])) > Z_GATE:  # bit3
                gate_bits |= 1 << 3
                reject = True
            if math.hypot(x[2], x[3]) > BM_FREEZE_UT:         # bit5: 磁気更新凍結
                gate_bits |= 1 << 5
                skip = True

            if not skip:
                # ---- 観測更新 h(x)=R_z(ψ−ψ0)·B0_horiz+b_m ----
                beta = x[0] - psi0
                cb, sb = math.cos(beta), math.sin(beta)
                h = np.array([cb * B0_horiz[0] - sb * B0_horiz[1] + x[2],
                              sb * B0_horiz[0] + cb * B0_horiz[1] + x[3]])
                dh = np.array([-sb * B0_horiz[0] - cb * B0_horiz[1],
                               cb * B0_horiz[0] - sb * B0_horiz[1]])
                H = np.array([[dh[0], 0.0, 1.0, 0.0],
                              [dh[1], 0.0, 0.0, 1.0]])
                y = z - h
                innov_val = float(np.hypot(y[0], y[1]))
                S = H @ P @ H.T + r_eff * np.eye(2)
                nis_val = float(y @ np.linalg.solve(S, y))
                if nis_val > NIS_HARD:                        # bit1: 棄却
                    gate_bits |= 1 << 1
                    reject = True
                elif nis_val > NIS_SOFT:                      # bit0: R膨張して適用
                    gate_bits |= 1 << 0
                    S = H @ P @ H.T + r_eff * (nis_val / NIS_SOFT) * np.eye(2)
                if not reject:
                    K = P @ H.T @ np.linalg.inv(S)
                    x = x + K @ y
                    x[0] = wrap(x[0])
                    P = (np.eye(4) - K @ H) @ P
                    n_update += 1
                    reject_since = None
                else:
                    n_reject += 1
                    if reject_since is None:
                        reject_since = r["t"]
            # 連続棄却 > 3s: ψ・b_m 対角を 1.02/s で緩膨張 (P0 の10倍上限)
            # (このブロックは fresh 時のみ実行されるため fresh間隔 dt_fresh を使う)
            if reject_since is not None and r["t"] - reject_since > REJECT_INFLATE_AFTER:
                g = REJECT_INFLATE_RATE ** dt_fresh if dt_fresh > 0 else 1.0
                for i in (0, 2, 3):
                    P[i, i] = min(P[i, i] * g, REJECT_INFLATE_CAP * P0[i, i])
            # bit6: |db_m/dt| > 0.3 µT/s 10s継続で警告 (b_m は fresh 時のみ更新)
            if prev_bm is not None and dt_fresh > 0:
                dbm = math.hypot(x[2] - prev_bm[0], x[3] - prev_bm[1]) / dt_fresh
                if dbm > DBM_WARN_RATE:
                    if bm_warn_since is None:
                        bm_warn_since = r["t"]
                    elif r["t"] - bm_warn_since > DBM_WARN_HOLD:
                        gate_bits |= 1 << 6
                else:
                    bm_warn_since = None
            prev_bm = (x[2], x[3])

            for b in range(7):
                if gate_bits & (1 << b):
                    gate_counts[b] += 1

            log["t"].append(r["t"])
            log["psi"].append(x[0])
            log["bmx"].append(x[2])
            log["bmy"].append(x[3])
            log["nis"].append(nis_val)
            log["yaw_corr"].append(wrap(math.atan2(lev_c[1], lev_c[0]) - yaw_mag0))
            log["yaw_uncorr"].append(wrap(math.atan2(lev_u[1], lev_u[0]) - yaw_mag0))
            log["cur"].append(r["cur"])
            log["dbn"].append(float(np.linalg.norm(dB_hat)))
            log["gate"].append(gate_bits)
            # 補正後の水平残差 (静置なので真値=B0_horiz。v2 §2.2 オーダー確認用)
            log["res_xy"].append(float(np.hypot(lev_c[0] - B0_horiz[0],
                                                lev_c[1] - B0_horiz[1])))
            log["innov"].append(innov_val)
            log["measure"].append(r["phase"] == "measure")

    # ---------------- 出力 -------------------------------------------------------
    t = np.array(log["t"])
    psi = np.degrees(np.array(log["psi"]))
    yaw_c = np.degrees(np.array(log["yaw_corr"]))
    yaw_u = np.degrees(np.array(log["yaw_uncorr"]))
    bmx, bmy = np.array(log["bmx"]), np.array(log["bmy"])
    nis = np.array(log["nis"])

    out_dir = HERE / "graphs" / f"replay_{stem}"
    out_dir.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, yaw_u, color="#f472b6", lw=0.8, label="磁気yaw 補正前")
    ax.plot(t, yaw_c, color="C0", lw=0.9, label=f"磁気yaw 補正後(方式{args.method})")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("yaw [deg] (アンカー基準)")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title(f"{stem}: FF補正前後の磁気yaw (profile={profile['name']})")
    fig.tight_layout()
    fig.savefig(out_dir / "mag_yaw.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, psi, color="C2", lw=0.9, label="ψ_est (EKF)")
    ax.axhline(0, color="gray", lw=0.6)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("ψ_est [deg]")
    ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, log["cur"], color="C7", lw=0.5, alpha=0.5, label="I_total")
    ax2.set_ylabel("I_total [A]")
    ax.legend(loc="upper left")
    ax.set_title(f"{stem}: EKF Yaw推定 (静置 → 変動<15°が受入基準)")
    fig.tight_layout()
    fig.savefig(out_dir / "psi_est.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, bmx, lw=0.9, label="b_mx")
    ax.plot(t, bmy, lw=0.9, label="b_my")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("b_m [µT]")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title(f"{stem}: EKF 磁気残差バイアス b_m")
    fig.tight_layout()
    fig.savefig(out_dir / "b_m.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, nis, ".", ms=2.5, label="NIS")
    ax.axhline(NIS_SOFT, color="orange", lw=0.8, label="χ²₂ 95% (R膨張)")
    ax.axhline(NIS_HARD, color="red", lw=0.8, label="χ²₂ 99.9% (棄却)")
    ax.set_yscale("log")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("NIS")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title(f"{stem}: NIS")
    fig.tight_layout()
    fig.savefig(out_dir / "nis.png", dpi=140)
    plt.close(fig)

    psi_span = float(psi.max() - psi.min())
    valid_nis = nis[np.isfinite(nis)]
    print(f"リプレイ {stem} (profile={profile['name']}, 方式{args.method})")
    print(f"  サンプル: {len(rows)}行 / fresh磁気 {len(t)}点 / 更新 {n_update} / 棄却 {n_reject}")
    if n_skipped_nan:
        print(f"  警告: cur/b_cal/roll/pitch の欠損値で {n_skipped_nan} 行の fresh 処理をスキップ")
    print(f"  アンカー: B0={np.round(B0,2).tolist()} µT  |B0_horiz|={np.hypot(*B0_horiz):.1f} µT  "
          f"I_idle={I_idle:.4f} A")
    print(f"  ψ_est: min={psi.min():+.2f}° max={psi.max():+.2f}° 変動幅={psi_span:.2f}° "
          f"(受入基準 <15°: {'OK' if psi_span < 15 else 'NG → 要調査'})")
    print(f"  磁気yaw変動幅: 補正前 {yaw_u.max()-yaw_u.min():.1f}° → "
          f"補正後 {yaw_c.max()-yaw_c.min():.1f}°")
    res = np.array(log["res_xy"])
    mm = np.array(log["measure"])
    if mm.any():
        rm = res[mm]
        inn = np.array(log["innov"])[mm]
        inn = inn[np.isfinite(inn)]
        print(f"  補正後水平残差 (measure行, EMA後, アンカーB0比): "
              f"RMS={np.sqrt((rm**2).mean()):.2f} µT max={rm.max():.2f} µT "
              f"(基準磁場ドリフト混入分は b_m が吸収)")
        print(f"  EKFイノベーション |y| (measure行, b_m吸収後): "
              f"RMS={np.sqrt((inn**2).mean()):.2f} µT max={inn.max():.2f} µT "
              f"(v2 §2.2 オーダー 2-4µT RMS が目安)")
    print(f"  b_m 最終=({bmx[-1]:+.2f},{bmy[-1]:+.2f}) µT  max|b_m|="
          f"{np.hypot(bmx,bmy).max():.2f} µT")
    print(f"  NIS: 中央値={np.median(valid_nis):.2f} 95%={np.percentile(valid_nis,95):.2f} "
          f"max={valid_nis.max():.2f}")
    print(f"  ゲート発動回数 bit0(R膨張)={gate_counts[0]} bit1(NIS棄却)={gate_counts[1]} "
          f"bit2(norm棄却)={gate_counts[2]} bit3(z棄却)={gate_counts[3]} "
          f"bit4(tilt)={gate_counts[4]} bit5(b_m凍結)={gate_counts[5]} "
          f"bit6(db_m/dt警告)={gate_counts[6]}")
    print(f"  図 → {out_dir}")
    return 0 if psi_span < 15 else 1


if __name__ == "__main__":
    sys.exit(main())
