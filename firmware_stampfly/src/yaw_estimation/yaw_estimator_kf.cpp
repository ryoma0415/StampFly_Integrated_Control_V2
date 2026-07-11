#include "yaw_estimator_kf.hpp"

#include <math.h>

#include "angle_utils.hpp"
#include "yaw_config.hpp"

namespace {
float clamp01Abs(float value) {
    if (value > 1.0f) return 1.0f;
    if (value < -1.0f) return -1.0f;
    return value;
}
}  // namespace

void YawEstimatorKf::resetCovariance() {
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {
            P_[r][c] = 0.0f;
        }
    }
    P_[0][0] = FF_EKF_P0_PSI_RAD2;
    P_[1][1] = FF_EKF_P0_BG_RAD2_S2;
    P_[2][2] = FF_EKF_P0_BM_UT2;
    P_[3][3] = FF_EKF_P0_BM_UT2;
}

void YawEstimatorKf::reanchor(float psi0_rad, float b0h_x, float b0h_y, const MagVector& b0_full) {
    psi0_ = wrapPi(psi0_rad);
    b0h_x_ = b0h_x;
    b0h_y_ = b0h_y;
    b0_ = b0_full;
    b0_norm_ = magNorm(b0_full);
    x_[0] = psi0_;
    x_[2] = 0.0f;  // b_m ← 0 (アンカーで基準場を取り直したため)
    x_[3] = 0.0f;
    resetCovariance();
    anchor_valid_ = true;
    mag_frozen_ = false;
    nis_ = 0.0f;
    gate_bits_ = 0;
    time_since_accept_s_ = 0.0f;
    drift_warn_time_s_ = 0.0f;
}

void YawEstimatorKf::reseedYaw(float psi_rad) {
    x_[0] = wrapPi(psi_rad);
    resetCovariance();
    nis_ = 0.0f;
    gate_bits_ &= FF_EKF_GATE_BM_FROZEN;  // 凍結状態はアンカーでのみ解除
    time_since_accept_s_ = 0.0f;
    drift_warn_time_s_ = 0.0f;
}

void YawEstimatorKf::predict(
    float omega_z_rad_s,
    float roll_rate_rad_s,
    float roll_rad,
    float pitch_rad,
    float dt_s
) {
    if (dt_s <= 0.0f || dt_s > 0.2f) {
        dt_s = SENSOR_PERIOD_US * 1.0e-6f;
    }

    const float a = 1.0f - dt_s / FF_EKF_TAU_BM_S;  // Gauss-Markov 減衰

    // ---- チルト運動学予測 (V2改修A) ----
    // 教科書の航空規約では ψ̇=(q·sinφ+r·cosφ)/cosθ だが、本ファームは
    // sensor.cpp で Madgwick に軸入替 (gx=q, gy=p, gz=−r, 対応accel) で入力し
    // Roll_angle=getPitch / Pitch_angle=getRoll / Yaw_angle=−getYaw と取り出す
    // ため、回転行列は R_earth←body = R_z(−ψ)·R_y(φ)·R_x(θ)·T (T=軸入替) となり、
    // オイラー角レートは
    //   ψ̇ = ((r − b_g)·cosθ − p·sinθ) / cosφ
    // になる (q=ピッチレートは ψ̇ に現れず、代わりに p=ロールレートが入る。
    // 特異点は cosφ=0 = ロール±90°)。7/10 実験ログ explog_20260710_203534 の
    // Madgwick 姿勢差分から復元した機体レートとの照合 (3軸 corr 0.90-0.98) と、
    // tilt>10° 区間の per-step 回帰 (本形: corr +0.69 / slope +0.89、教科書形:
    // corr −0.30 で逆効果) で本形を検証済み。
    // b_g は従来どおり Z軸レート ω_z のバイアスとしてのみモデル化を続ける
    // (p のバイアスは非モデル化 — 水平近傍では sinθ≈0 で寄与が小さく、
    //  4状態のまま増やさない設計判断)。
    const float cos_phi = cosf(roll_rad);
    float psi_dot;
    float dpsidot_dbg;  // ∂ψ̇/∂b_g (F[0][1] = dpsidot_dbg·dt のヤコビアン用)
    if (cos_phi >= FF_EKF_TILT_KIN_COS_MIN) {
        const float cos_theta = cosf(pitch_rad);
        const float sin_theta = sinf(pitch_rad);
        const float inv_cos_phi = 1.0f / cos_phi;
        psi_dot =
            ((omega_z_rad_s - x_[1]) * cos_theta - roll_rate_rad_s * sin_theta) * inv_cos_phi;
        dpsidot_dbg = -cos_theta * inv_cos_phi;
    } else {
        // ロール特異点近傍 (|φ|>60°, FF_EKF_TILT_KIN_COS_MIN): 従来式へフォールバック
        psi_dot = omega_z_rad_s - x_[1];
        dpsidot_dbg = -1.0f;
    }
    // 数値安全弁: ±720°/s にクランプ
    if (psi_dot > FF_EKF_PSI_DOT_CLAMP_RAD_S) psi_dot = FF_EKF_PSI_DOT_CLAMP_RAD_S;
    if (psi_dot < -FF_EKF_PSI_DOT_CLAMP_RAD_S) psi_dot = -FF_EKF_PSI_DOT_CLAMP_RAD_S;

    x_[0] = wrapPi(x_[0] + psi_dot * dt_s);
    x_[2] *= a;
    x_[3] *= a;

    // P⁻ = F·P·Fᵀ + Q·dt,  F = [[1,g,0,0],[0,1,0,0],[0,0,a,0],[0,0,0,a]],
    // g = ∂(Δψ)/∂b_g = dpsidot_dbg·dt (従来の −dt はフォールバック時の特殊形)
    const float g01 = dpsidot_dbg * dt_s;
    float FP[4][4];
    for (uint8_t c = 0; c < 4; c++) {
        FP[0][c] = P_[0][c] + g01 * P_[1][c];
        FP[1][c] = P_[1][c];
        FP[2][c] = a * P_[2][c];
        FP[3][c] = a * P_[3][c];
    }
    for (uint8_t r = 0; r < 4; r++) {
        const float col0 = FP[r][0] + g01 * FP[r][1];
        const float col2 = a * FP[r][2];
        const float col3 = a * FP[r][3];
        P_[r][0] = col0;
        // col1 = FP[r][1] そのまま
        P_[r][1] = FP[r][1];
        P_[r][2] = col2;
        P_[r][3] = col3;
    }
    P_[0][0] += FF_EKF_Q_PSI_RAD2_S * dt_s;
    P_[1][1] += FF_EKF_Q_BG_RAD2_S3 * dt_s;
    P_[2][2] += FF_EKF_Q_BM_UT2_S * dt_s;
    P_[3][3] += FF_EKF_Q_BM_UT2_S * dt_s;

    // 連続棄却 > 3s: ψ・b_m の対角を 1.02/s で緩膨張(P0 の 10 倍上限)。
    if (anchor_valid_) {
        time_since_accept_s_ += dt_s;
        if (time_since_accept_s_ > FF_EKF_REJECT_INFLATE_AFTER_S) {
            const float factor = 1.0f + (FF_EKF_REJECT_INFLATE_RATE_PER_S - 1.0f) * dt_s;
            const float psi_cap = FF_EKF_P_INFLATE_MAX_RATIO * FF_EKF_P0_PSI_RAD2;
            const float bm_cap = FF_EKF_P_INFLATE_MAX_RATIO * FF_EKF_P0_BM_UT2;
            P_[0][0] = fminf(P_[0][0] * factor, fmaxf(P_[0][0], psi_cap));
            P_[2][2] = fminf(P_[2][2] * factor, fmaxf(P_[2][2], bm_cap));
            P_[3][3] = fminf(P_[3][3] * factor, fmaxf(P_[3][3], bm_cap));
        }
    }
}

void YawEstimatorKf::update(
    const MagVector& b_corr_filt,
    float roll_rad,
    float pitch_rad,
    float sigma_ff_uT,
    float sigma_slew_uT,
    float sigma_diff_uT,
    float mag_dt_s
) {
    if (!anchor_valid_) {
        return;
    }
    if (mag_dt_s <= 0.0f || mag_dt_s > 0.5f) {
        mag_dt_s = 0.1f;
    }
    // bit5(凍結)/bit6(ドリフト警告)はラッチ、bit0-4 は直近更新の状態。
    uint8_t bits = gate_bits_ & (FF_EKF_GATE_BM_FROZEN | FF_EKF_GATE_DRIFT_WARN);

    // bit4: tilt > 25° → 磁気更新スキップ (レベル化の信頼性が落ちる)
    const float cos_tilt = clamp01Abs(cosf(roll_rad) * cosf(pitch_rad));
    const float tilt_rad = acosf(cos_tilt);
    if (tilt_rad > FF_EKF_TILT_SKIP_RAD) {
        gate_bits_ = bits | FF_EKF_GATE_TILT_SKIP;
        return;
    }

    // bit5: ‖b_m‖>20µT → 磁気更新凍結 (FFモデル破綻。再アンカーで解除)
    if (mag_frozen_) {
        gate_bits_ = bits;
        return;
    }

    // ノルム/z ゲート (基準は geomag プロファイルでなくアンカー実測 B0)
    const float norm = magNorm(b_corr_filt);
    const float norm_dev = fabsf(norm - b0_norm_);
    if (norm_dev > FF_EKF_NORM_GATE_HARD_UT) {
        gate_bits_ = bits | FF_EKF_GATE_NORM_REJECT;
        return;
    }
    if (fabsf(b_corr_filt.z - b0_.z) > FF_EKF_Z_GATE_UT) {
        gate_bits_ = bits | FF_EKF_GATE_Z_REJECT;
        return;
    }

    // 観測 z = レベル化した水平2成分
    const MagVector level = levelMagVectorBody(roll_rad, pitch_rad, b_corr_filt);
    const float zx = level.x;
    const float zy = level.y;

    // h(x) = R_z(ψ−ψ0)·B0_horiz + b_m,  R_z は標準CCW
    const float beta = wrapPi(x_[0] - psi0_);
    const float cb = cosf(beta);
    const float sb = sinf(beta);
    const float hx = cb * b0h_x_ - sb * b0h_y_ + x_[2];
    const float hy = sb * b0h_x_ + cb * b0h_y_ + x_[3];
    // ∂h/∂ψ = R_z'(β)·B0_horiz
    const float dhx = -sb * b0h_x_ - cb * b0h_y_;
    const float dhy = cb * b0h_x_ - sb * b0h_y_;

    const float y0 = zx - hx;
    const float y1 = zy - hy;

    // 適応 R: R_eff = R_base + σ_ff² + σ_slew² + σ_diff² + (sinθ_tilt·σ_rz)²
    const float sin_tilt = sinf(tilt_rad);
    float r_eff = FF_EKF_R_BASE_UT2 +
                  sigma_ff_uT * sigma_ff_uT +
                  sigma_slew_uT * sigma_slew_uT +
                  sigma_diff_uT * sigma_diff_uT +
                  (sin_tilt * FF_EKF_SIGMA_RZ_UT) * (sin_tilt * FF_EKF_SIGMA_RZ_UT);
    // ノルム偏差 8-20µT はソフト側: R 膨張 (bit0 扱い)
    if (norm_dev > FF_EKF_NORM_GATE_SOFT_UT) {
        const float ratio = norm_dev / FF_EKF_NORM_GATE_SOFT_UT;
        r_eff *= ratio * ratio;
        bits |= FF_EKF_GATE_R_INFLATED;
    }

    // S = H·P⁻·Hᵀ + R_eff·I₂  (H = [[dhx,0,1,0],[dhy,0,0,1]])
    // HP の 2 行 (4 列)
    float HP0[4];
    float HP1[4];
    for (uint8_t c = 0; c < 4; c++) {
        HP0[c] = dhx * P_[0][c] + P_[2][c];
        HP1[c] = dhy * P_[0][c] + P_[3][c];
    }
    float s00 = HP0[0] * dhx + HP0[2] + r_eff;
    float s01 = HP0[0] * dhy + HP0[3];
    float s10 = HP1[0] * dhx + HP1[2];
    float s11 = HP1[0] * dhy + HP1[3] + r_eff;

    float det = s00 * s11 - s01 * s10;
    if (det <= 1.0e-9f || !isfinite(det)) {
        gate_bits_ = bits | FF_EKF_GATE_NIS_REJECT;
        return;
    }
    float inv00 = s11 / det;
    float inv01 = -s01 / det;
    float inv10 = -s10 / det;
    float inv11 = s00 / det;

    // NIS = yᵀ·S⁻¹·y (基本 R での値を報告)
    const float nis = y0 * (inv00 * y0 + inv01 * y1) + y1 * (inv10 * y0 + inv11 * y1);
    nis_ = nis;

    // V2改修B-1: ソフト再捕捉。norm/z/tilt ゲートは通過しているのに NIS だけで
    // FF_EKF_RECAPTURE_AFTER_S(5s) を超えて棄却され続けている (NISロック) 場合、
    // 棄却する代わりに「制限付き更新」(bit7) で引き込みを再開する (飛行中も有効)。
    // time_since_accept_s_ は通常受理 (NIS≤13.8) でのみリセットされるため、
    // 引き込みが進んで NIS が下がれば自然に通常経路へ復帰する。
    const bool recapture =
        nis > FF_EKF_NIS_REJECT && time_since_accept_s_ > FF_EKF_RECAPTURE_AFTER_S;
    if (nis > FF_EKF_NIS_REJECT && !recapture) {
        // bit1: NIS > χ²₂(99.9%) = 13.8 → 棄却
        gate_bits_ = bits | FF_EKF_GATE_NIS_REJECT;
        return;
    }
    if (recapture) {
        // 制限付き更新 (1/3): R を (NIS/13.8) 倍に膨張 — 実効 NIS≈13.8 相当まで
        // ゲインを減衰させた上で、後段の Δψ クランプ・b_m ゲインゼロ化を適用する
        r_eff *= nis / FF_EKF_NIS_REJECT;
        bits |= FF_EKF_GATE_RECAPTURE;
    } else if (nis > FF_EKF_NIS_INFLATE) {
        // bit0: NIS > χ²₂(95%) = 5.99 → R×(NIS/5.99) に膨張して採用
        r_eff *= nis / FF_EKF_NIS_INFLATE;
        bits |= FF_EKF_GATE_R_INFLATED;
    }
    if (recapture || nis > FF_EKF_NIS_INFLATE) {
        // 膨張後の R で S・S⁻¹ を再計算
        s00 = HP0[0] * dhx + HP0[2] + r_eff;
        s01 = HP0[0] * dhy + HP0[3];
        s10 = HP1[0] * dhx + HP1[2];
        s11 = HP1[0] * dhy + HP1[3] + r_eff;
        det = s00 * s11 - s01 * s10;
        if (det <= 1.0e-9f || !isfinite(det)) {
            gate_bits_ = bits | FF_EKF_GATE_NIS_REJECT;
            return;
        }
        inv00 = s11 / det;
        inv01 = -s01 / det;
        inv10 = -s10 / det;
        inv11 = s00 / det;
    }

    // K = P⁻·Hᵀ·S⁻¹ (4×2)。P·Hᵀ の列は HP の転置。
    float K[4][2];
    for (uint8_t r = 0; r < 4; r++) {
        const float ph0 = HP0[r];  // (P·Hᵀ)[r][0] = (H·P)[0][r] (P 対称)
        const float ph1 = HP1[r];
        K[r][0] = ph0 * inv00 + ph1 * inv10;
        K[r][1] = ph0 * inv01 + ph1 * inv11;
    }
    if (recapture) {
        // 制限付き更新 (2/3): バイアス (b_g/b_mx/b_my) への補正はゼロ — 大乖離を
        // バイアスに吸収させず、ψ の引き込みのみ行う (b_g を許すと巨大イノベー
        // ションが b_g を蹴り、再捕捉後に ψ が逆側へ数°オーバーシュートして
        // 数十秒残ることを合成シミュレーションで確認済み)。
        // 状態・共分散の両方で同じ制限ゲインを使い整合を保つ。
        K[1][0] = 0.0f;
        K[1][1] = 0.0f;
        K[2][0] = 0.0f;
        K[2][1] = 0.0f;
        K[3][0] = 0.0f;
        K[3][1] = 0.0f;
    }

    const float bm_prev_x = x_[2];
    const float bm_prev_y = x_[3];
    float dx[4];
    for (uint8_t r = 0; r < 4; r++) {
        dx[r] = K[r][0] * y0 + K[r][1] * y1;
    }
    if (recapture) {
        // 制限付き更新 (3/3): Δψ を ±FF_EKF_RECAPTURE_MAX_STEP_RAD(3°)/更新に
        // クランプ (磁気更新は実効 10Hz → 最大 30°/s の引き込みレート)
        if (dx[0] > FF_EKF_RECAPTURE_MAX_STEP_RAD) dx[0] = FF_EKF_RECAPTURE_MAX_STEP_RAD;
        if (dx[0] < -FF_EKF_RECAPTURE_MAX_STEP_RAD) dx[0] = -FF_EKF_RECAPTURE_MAX_STEP_RAD;
    }
    for (uint8_t r = 0; r < 4; r++) {
        x_[r] += dx[r];
    }
    x_[0] = wrapPi(x_[0]);

    // P = (I − K·H)·P⁻,  K·H の r 行 c 列 = K[r][0]·H[0][c] + K[r][1]·H[1][c]
    float KH[4][4];
    for (uint8_t r = 0; r < 4; r++) {
        KH[r][0] = K[r][0] * dhx + K[r][1] * dhy;
        KH[r][1] = 0.0f;
        KH[r][2] = K[r][0];
        KH[r][3] = K[r][1];
    }
    float newP[4][4];
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {
            float acc = P_[r][c];
            for (uint8_t k = 0; k < 4; k++) {
                acc -= KH[r][k] * P_[k][c];
            }
            newP[r][c] = acc;
        }
    }
    // 対称化 (数値誤差の蓄積対策)
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {
            P_[r][c] = 0.5f * (newP[r][c] + newP[c][r]);
        }
    }

    if (recapture) {
        // ソフト再捕捉中は「通常受理」ではない: time_since_accept_s_ は保持し、
        // b_m 不変のため凍結/ドリフト判定もスキップする。
        gate_bits_ = bits;
        return;
    }

    time_since_accept_s_ = 0.0f;

    // bit5: ‖b_m‖ > 20µT → FF モデル破綻 → 磁気更新凍結 (要再アンカー)
    const float bm_norm = sqrtf(x_[2] * x_[2] + x_[3] * x_[3]);
    if (bm_norm > FF_EKF_BM_FREEZE_UT) {
        mag_frozen_ = true;
        bits |= FF_EKF_GATE_BM_FROZEN;
    }

    // bit6: |db_m/dt| > 0.3µT/s が 10s 継続で警告
    const float dbm = sqrtf(
        (x_[2] - bm_prev_x) * (x_[2] - bm_prev_x) + (x_[3] - bm_prev_y) * (x_[3] - bm_prev_y));
    const float bm_rate = dbm / mag_dt_s;
    if (bm_rate > FF_EKF_BM_DRIFT_WARN_UT_S) {
        drift_warn_time_s_ += mag_dt_s;
    } else {
        drift_warn_time_s_ = 0.0f;
        bits &= ~FF_EKF_GATE_DRIFT_WARN;
    }
    if (drift_warn_time_s_ >= FF_EKF_BM_DRIFT_WARN_HOLD_S) {
        bits |= FF_EKF_GATE_DRIFT_WARN;
    }

    gate_bits_ = bits;
}
