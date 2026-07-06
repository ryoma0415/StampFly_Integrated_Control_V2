#include "ff_calibration.hpp"

#include <math.h>
#include <string.h>

namespace {

// staging/confirmed の LUT・モーター係数が有限かつ電流昇順かを検証する。
bool coeffsSane(const float* lut_ia, const float (*lut_db)[3], uint8_t nlut,
                const float (*mot)[6], float iid) {
    if (nlut < FF_LUT_MIN_POINTS || nlut > FF_LUT_MAX_POINTS) {
        return false;
    }
    for (uint8_t k = 0; k < nlut; k++) {
        if (!isfinite(lut_ia[k]) || !isfinite(lut_db[k][0]) ||
            !isfinite(lut_db[k][1]) || !isfinite(lut_db[k][2])) {
            return false;
        }
        if (k > 0 && lut_ia[k] <= lut_ia[k - 1]) {
            return false;  // 電流昇順(狭義)であること
        }
    }
    for (uint8_t m = 0; m < 4; m++) {
        for (uint8_t j = 0; j < 6; j++) {
            if (!isfinite(mot[m][j])) {
                return false;
            }
        }
    }
    return isfinite(iid);
}

}  // namespace

bool FfCalibration::stageBegin(uint8_t nlut) {
    if (nlut < FF_LUT_MIN_POINTS || nlut > FF_LUT_MAX_POINTS) {
        staging_active_ = false;
        return false;
    }
    staging_ = Coeffs{};
    staging_.nlut = nlut;
    staging_active_ = true;
    staged_lut_mask_ = 0;
    staged_mot_mask_ = 0;
    staged_aux_ = false;
    return true;
}

bool FfCalibration::stageLutPoint(uint8_t idx, float ia, float dx, float dy, float dz) {
    if (!staging_active_ || idx >= staging_.nlut ||
        !isfinite(ia) || !isfinite(dx) || !isfinite(dy) || !isfinite(dz)) {
        return false;
    }
    staging_.lut_ia[idx] = ia;
    staging_.lut_db[idx][0] = dx;
    staging_.lut_db[idx][1] = dy;
    staging_.lut_db[idx][2] = dz;
    staged_lut_mask_ |= (1UL << idx);
    return true;
}

bool FfCalibration::stageMotor(uint8_t idx, float ax, float ay, float az, float c2, float c1, float c0) {
    if (!staging_active_ || idx >= 4 ||
        !isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(c2) || !isfinite(c1) || !isfinite(c0)) {
        return false;
    }
    staging_.mot[idx][0] = ax;
    staging_.mot[idx][1] = ay;
    staging_.mot[idx][2] = az;
    staging_.mot[idx][3] = c2;
    staging_.mot[idx][4] = c1;
    staging_.mot[idx][5] = c0;
    staged_mot_mask_ |= (1u << idx);
    return true;
}

bool FfCalibration::stageAux(float iid) {
    if (!staging_active_ || !isfinite(iid)) {
        return false;
    }
    staging_.iid = iid;
    staged_aux_ = true;
    return true;
}

bool FfCalibration::commit(uint32_t crc_expected, const char*& error_message) {
    if (!staging_active_) {
        // 冪等化: commit 成功後に ack がロストしてサーバーが再送した場合、
        // 同一 CRC の確定係数が既に有効なら成功を再ackする (wire-contract C5)。
        if (valid_ && crc_expected == crc_) {
            error_message = "ff calibration committed";
            return true;
        }
        error_message = "no staging in progress (ffcal_begin first)";
        return false;
    }
    const uint32_t full_lut_mask = (staging_.nlut >= 32) ? 0xFFFFFFFFUL : ((1UL << staging_.nlut) - 1UL);
    if (staged_lut_mask_ != full_lut_mask) {
        error_message = "missing lut points";
        return false;
    }
    if (staged_mot_mask_ != 0x0F) {
        error_message = "missing motor coefficients";
        return false;
    }
    if (!staged_aux_) {
        error_message = "missing aux (iid)";
        return false;
    }
    if (!coeffsSane(staging_.lut_ia, staging_.lut_db, staging_.nlut, staging_.mot, staging_.iid)) {
        error_message = "lut points not strictly ascending or non-finite";
        return false;
    }
    float values[FF_LUT_MAX_POINTS * 4 + 25];
    serializeCoeffs(staging_, values);
    const uint32_t crc = crc32Of(values, blobFloatCountFor(staging_.nlut));
    if (crc != crc_expected) {
        error_message = "crc mismatch";
        return false;
    }
    confirmed_ = staging_;
    crc_ = crc;
    valid_ = true;
    staging_active_ = false;
    slew_has_last_ = false;
    error_message = "ff calibration committed";
    return true;
}

void FfCalibration::clear() {
    confirmed_ = Coeffs{};
    valid_ = false;
    crc_ = 0;
    staging_active_ = false;
    staged_lut_mask_ = 0;
    staged_mot_mask_ = 0;
    staged_aux_ = false;
    slew_has_last_ = false;
}

void FfCalibration::serializeCoeffs(const Coeffs& c, float* out) {
    uint16_t n = 0;
    for (uint8_t k = 0; k < c.nlut; k++) {
        out[n++] = c.lut_ia[k];
        out[n++] = c.lut_db[k][0];
        out[n++] = c.lut_db[k][1];
        out[n++] = c.lut_db[k][2];
    }
    for (uint8_t m = 0; m < 4; m++) {
        for (uint8_t j = 0; j < 6; j++) {
            out[n++] = c.mot[m][j];
        }
    }
    out[n++] = c.iid;
}

void FfCalibration::serialize(float* out) const {
    serializeCoeffs(confirmed_, out);
}

bool FfCalibration::restoreFromBlob(const float* values, uint8_t nlut, uint32_t crc) {
    if (nlut < FF_LUT_MIN_POINTS || nlut > FF_LUT_MAX_POINTS) {
        return false;
    }
    Coeffs c;
    c.nlut = nlut;
    uint16_t n = 0;
    for (uint8_t k = 0; k < nlut; k++) {
        c.lut_ia[k] = values[n++];
        c.lut_db[k][0] = values[n++];
        c.lut_db[k][1] = values[n++];
        c.lut_db[k][2] = values[n++];
    }
    for (uint8_t m = 0; m < 4; m++) {
        for (uint8_t j = 0; j < 6; j++) {
            c.mot[m][j] = values[n++];
        }
    }
    c.iid = values[n++];
    if (!coeffsSane(c.lut_ia, c.lut_db, c.nlut, c.mot, c.iid)) {
        return false;
    }
    confirmed_ = c;
    crc_ = crc;
    valid_ = true;
    slew_has_last_ = false;
    return true;
}

uint32_t FfCalibration::crc32Of(const float* values, uint16_t count) {
    // CRC-32 (IEEE 反転多項式 0xEDB88320, zlib.crc32 互換)。ESP32 は
    // little-endian なので float のメモリ表現がそのまま float32 LE 連結になる。
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint16_t i = 0; i < count; i++) {
        uint8_t bytes[4];
        memcpy(bytes, &values[i], 4);
        for (uint8_t b = 0; b < 4; b++) {
            crc ^= bytes[b];
            for (uint8_t bit = 0; bit < 8; bit++) {
                crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
            }
        }
    }
    return ~crc;
}

uint8_t FfCalibration::lutSegment(float i_total_a) const {
    // 範囲外は端区間 (0 または nlut-2) にクランプ → その傾きで外挿される。
    const uint8_t nlut = confirmed_.nlut;
    if (nlut < 2) {
        return 0;
    }
    if (i_total_a <= confirmed_.lut_ia[0]) {
        return 0;
    }
    for (uint8_t k = 0; k + 1 < nlut; k++) {
        if (i_total_a <= confirmed_.lut_ia[k + 1]) {
            return k;
        }
    }
    return nlut - 2;
}

float FfCalibration::lutSlope(float i_total_a, uint8_t axis) const {
    const uint8_t k = lutSegment(i_total_a);
    const float di = confirmed_.lut_ia[k + 1] - confirmed_.lut_ia[k];
    if (di <= 0.0f) {
        return 0.0f;  // commit/restore で昇順検証済みだが安全側に
    }
    return (confirmed_.lut_db[k + 1][axis] - confirmed_.lut_db[k][axis]) / di;
}

float FfCalibration::lutInterp(float i_total_a, uint8_t axis) const {
    const uint8_t k = lutSegment(i_total_a);
    const float di = confirmed_.lut_ia[k + 1] - confirmed_.lut_ia[k];
    if (di <= 0.0f) {
        return confirmed_.lut_db[k][axis];
    }
    const float slope = (confirmed_.lut_db[k + 1][axis] - confirmed_.lut_db[k][axis]) / di;
    return confirmed_.lut_db[k][axis] + slope * (i_total_a - confirmed_.lut_ia[k]);
}

FfCorrection FfCalibration::compute(
    float i_total_a,
    const float duty[4],
    uint8_t mask,
    bool motors_running,
    uint8_t ff_mode,
    float i_idle_anchor_a,
    bool anchor_valid,
    uint32_t now_ms
) {
    FfCorrection out;
    if (!valid_ || ff_mode == 0 || confirmed_.nlut < 2 || !isfinite(i_total_a)) {
        slew_has_last_ = false;
        return out;
    }

    // 方式A: 総電流 LUT (主項)
    out.delta_b.x = lutInterp(i_total_a, 0);
    out.delta_b.y = lutInterp(i_total_a, 1);
    out.delta_b.z = lutInterp(i_total_a, 2);

    // σ_slew = |a_xy|·|dI/dt|·τ_resid — LUT の局所傾きと呼び出し間の電流微分。
    if (slew_has_last_ && now_ms > slew_last_ms_) {
        const float dt_s = static_cast<float>(now_ms - slew_last_ms_) * 1.0e-3f;
        if (dt_s > 0.0f && dt_s < 0.5f) {
            const float didt = fabsf(i_total_a - slew_last_i_a_) / dt_s;
            const float ax = lutSlope(i_total_a, 0);
            const float ay = lutSlope(i_total_a, 1);
            out.sigma_slew_uT = sqrtf(ax * ax + ay * ay) * didt * FF_TAU_RESID_S;
        }
    }
    slew_last_i_a_ = i_total_a;
    slew_last_ms_ = now_ms;
    slew_has_last_ = true;

    // 方式B: 個別モーター差動項 (ヨー操作の盲点を補う)
    if (ff_mode == 2 && motors_running) {
        // I_idle はアンカー実測を優先、未取得時はベンチ参考値 iid。
        const float i_idle = anchor_valid ? i_idle_anchor_a : confirmed_.iid;
        const float i_active = i_total_a - i_idle;
        float i_hat[4];
        float sum_i_hat = 0.0f;
        for (uint8_t m = 0; m < 4; m++) {
            // mask 外は d=0 として q_m(0)=c0 を使う(§5.1)。
            const float d = (mask & (1u << m)) ? duty[m] : 0.0f;
            i_hat[m] = confirmed_.mot[m][3] * d * d + confirmed_.mot[m][4] * d + confirmed_.mot[m][5];
            sum_i_hat += i_hat[m];
        }
        if (sum_i_hat >= FF_DIFF_MIN_SUM_CURRENT_A) {
            const float s = i_active / sum_i_hat;
            float max_abs_di = 0.0f;
            for (uint8_t m = 0; m < 4; m++) {
                const float di = s * i_hat[m] - i_active * 0.25f;
                out.delta_b.x += confirmed_.mot[m][0] * di;
                out.delta_b.y += confirmed_.mot[m][1] * di;
                out.delta_b.z += confirmed_.mot[m][2] * di;
                const float abs_di = fabsf(di);
                if (abs_di > max_abs_di) {
                    max_abs_di = abs_di;
                }
            }
            // |δI| は max_m|δI_m| を採用(§5.1)。
            out.sigma_diff_uT = FF_SIGMA_DIFF_UT_PER_A * max_abs_di;
        }
    }

    out.sigma_ff_uT = FF_KAPPA_FF * sqrtf(out.delta_b.x * out.delta_b.x + out.delta_b.y * out.delta_b.y);
    return out;
}
