#include "yaw_estimator.hpp"
#include "angle_utils.hpp"
#include "yaw_config.hpp"
#include <math.h>

namespace {
float clampAbs(float value, float limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}
}  // namespace

void YawEstimator::reset(float yaw_rad) {
    state_ = YawEstimate{};
    state_.yaw_est_rad = wrapPi(yaw_rad);
    state_.initialized = true;
    mag_yaw_offset_rad_ = 0.0f;
    pending_reference_yaw_rad_ = state_.yaw_est_rad;
    mag_reference_pending_ = true;
    mag_reference_valid_ = false;
    last_yaw_mag_rad_ = 0.0f;
    last_mag_norm_ = 0.0f;
    last_mag_observation_valid_ = false;
    seed_from_mag_pending_ = false;
}

void YawEstimator::setYawZero(float yaw_rad) {
    reset(yaw_rad);
    yaw_zero_user_valid_ = true;
}

void YawEstimator::clearYawZero() {
    state_ = YawEstimate{};
    mag_yaw_offset_rad_ = 0.0f;
    pending_reference_yaw_rad_ = 0.0f;
    mag_reference_pending_ = false;
    mag_reference_valid_ = false;
    yaw_zero_user_valid_ = false;
    last_yaw_mag_rad_ = 0.0f;
    last_mag_norm_ = 0.0f;
    last_mag_observation_valid_ = false;
    seed_from_mag_pending_ = false;
}

void YawEstimator::restoreYawZero(float mag_yaw_offset_rad) {
    reset(0.0f);
    mag_yaw_offset_rad_ = wrapPi(mag_yaw_offset_rad);
    mag_reference_pending_ = false;   // offset comes from NVS, do not re-capture
    mag_reference_valid_ = true;
    yaw_zero_user_valid_ = true;
    seed_from_mag_pending_ = true;    // align the estimate on the first mag sample
}

bool YawEstimator::setGeomagneticReference(const GeomagneticReference& reference) {
    // The range checks also guard the NVS restore path: a persisted
    // out-of-range declination would otherwise be re-applied on every boot.
    if (!reference.valid ||
        !isfinite(reference.declination_east_rad) ||
        !isfinite(reference.inclination_rad) ||
        !isfinite(reference.horizontal_uT) ||
        !isfinite(reference.vertical_uT) ||
        !isfinite(reference.total_uT) ||
        fabsf(reference.declination_east_rad) > PI ||
        fabsf(reference.inclination_rad) > PI / 2.0f ||
        reference.horizontal_uT <= 1.0f ||
        reference.total_uT <= 1.0f) {
        return false;
    }

    GeomagneticReference sanitized = reference;
    if (!isfinite(sanitized.total_tolerance_ratio) || sanitized.total_tolerance_ratio < 0.05f) {
        sanitized.total_tolerance_ratio = GEOMAG_DEFAULT_TOTAL_TOLERANCE_RATIO;
    }
    if (!isfinite(sanitized.horizontal_tolerance_ratio) || sanitized.horizontal_tolerance_ratio < 0.05f) {
        sanitized.horizontal_tolerance_ratio = GEOMAG_DEFAULT_HORIZONTAL_TOLERANCE_RATIO;
    }
    if (!isfinite(sanitized.inclination_tolerance_rad) || sanitized.inclination_tolerance_rad < 1.0f * DEG_TO_RAD) {
        sanitized.inclination_tolerance_rad = GEOMAG_DEFAULT_INCLINATION_TOLERANCE_DEG * DEG_TO_RAD;
    }
    if (!isfinite(sanitized.inclination_z_sign)) {
        sanitized.inclination_z_sign = GEOMAG_INCLINATION_Z_SIGN;
    }
    sanitized.inclination_z_sign = sanitized.inclination_z_sign < 0.0f ? -1.0f : 1.0f;
    sanitized.valid = true;
    geomag_reference_ = sanitized;
    return true;
}

void YawEstimator::clearGeomagneticReference() {
    geomag_reference_ = GeomagneticReference{};
}

YawEstimate YawEstimator::update(
    float gyro_yaw_rate_rad_s,
    float roll_rad,
    float pitch_rad,
    const MagVector& mag_body,
    float dt_s,
    bool mag_sample_fresh,
    float mag_dt_s
) {
    if (dt_s <= 0.0f || dt_s > 0.2f) {
        dt_s = 0.01f;
    }

    float yaw_gyro = wrapPi(state_.yaw_est_rad + gyro_yaw_rate_rad_s * dt_s);
    state_.geomag_reference_valid = geomag_reference_.valid;
    state_.heading_true_valid = geomag_reference_.valid;
    if (mag_sample_fresh) {
        state_.recapture_candidate = false;
    }

    if (!mag_sample_fresh) {
        state_.yaw_est_rad = yaw_gyro;
        if (!state_.initialized) {
            state_.initialized = true;
        }
        return state_;
    }

    if (mag_dt_s <= 0.0f) {
        mag_dt_s = dt_s;
    } else if (mag_dt_s > 0.5f) {
        mag_dt_s = 0.5f;
    }

    const MagVector mag_level = levelMagVectorBody(roll_rad, pitch_rad, mag_body);
    const float yaw_mag_raw = wrapPi(atan2f(mag_level.y, mag_level.x));
    const float norm = magNorm(mag_body);
    const float horizontal = sqrtf(mag_level.x * mag_level.x + mag_level.y * mag_level.y);
    const float inclination_z_sign = geomag_reference_.valid
        ? geomag_reference_.inclination_z_sign
        : GEOMAG_INCLINATION_Z_SIGN;
    const float inclination_z = inclination_z_sign * mag_level.z;
    const float inclination = horizontal > 1.0f ? atan2f(inclination_z, horizontal) : 0.0f;

    state_.heading_mag_rad = yaw_mag_raw;
    state_.heading_true_rad = geomag_reference_.valid
        ? wrapPi(yaw_mag_raw + geomag_reference_.declination_east_rad)
        : yaw_mag_raw;
    state_.mag_total_uT = norm;
    state_.mag_horizontal_uT = horizontal;
    state_.mag_inclination_rad = inclination;
    state_.geomag_total_error_ratio = 0.0f;
    state_.geomag_horizontal_error_ratio = 0.0f;
    state_.geomag_inclination_error_rad = 0.0f;
    state_.geomag_consistent = true;

    if (geomag_reference_.valid) {
        const bool total_ok = geomag_reference_.total_uT <= 1.0f ||
            fabsf(norm - geomag_reference_.total_uT) <= geomag_reference_.total_uT * geomag_reference_.total_tolerance_ratio;
        const bool horizontal_ok = geomag_reference_.horizontal_uT <= 1.0f ||
            fabsf(horizontal - geomag_reference_.horizontal_uT) <= geomag_reference_.horizontal_uT * geomag_reference_.horizontal_tolerance_ratio;
        const bool inclination_ok =
            fabsf(inclination - geomag_reference_.inclination_rad) <= geomag_reference_.inclination_tolerance_rad;

        if (geomag_reference_.total_uT > 1.0f) {
            state_.geomag_total_error_ratio = (norm - geomag_reference_.total_uT) / geomag_reference_.total_uT;
        }
        if (geomag_reference_.horizontal_uT > 1.0f) {
            state_.geomag_horizontal_error_ratio = (horizontal - geomag_reference_.horizontal_uT) / geomag_reference_.horizontal_uT;
        }
        state_.geomag_inclination_error_rad = inclination - geomag_reference_.inclination_rad;
        state_.geomag_consistent = total_ok && horizontal_ok && inclination_ok;
    }

    if (mag_reference_pending_ && norm > 1.0f) {
        mag_yaw_offset_rad_ = wrapPi(yaw_mag_raw - pending_reference_yaw_rad_);
        mag_reference_pending_ = false;
        mag_reference_valid_ = true;
    }

    const float yaw_mag = mag_reference_valid_
        ? wrapPi(yaw_mag_raw - mag_yaw_offset_rad_)
        : yaw_mag_raw;

    if (seed_from_mag_pending_ && norm > 1.0f) {
        // Boot-time restore of a persisted yaw zero: snap the estimate (and the
        // gyro prediction for this update) to the magnetic yaw so the saved
        // heading reference carries over instead of starting from 0 wherever
        // the drone happens to face.
        yaw_gyro = yaw_mag;
        state_.yaw_est_rad = yaw_mag;
        seed_from_mag_pending_ = false;
    }

    if (state_.mag_norm_reference <= 1.0f && norm > 1.0f) {
        state_.mag_norm_reference = norm;
    } else if (state_.mag_valid && norm > 1.0f) {
        state_.mag_norm_reference = 0.995f * state_.mag_norm_reference + 0.005f * norm;
    }

    const bool norm_valid =
        state_.mag_norm_reference <= 1.0f ||
        fabsf(norm - state_.mag_norm_reference) <= state_.mag_norm_reference * YAW_MAG_NORM_TOLERANCE;

    const float innovation = wrapPi(yaw_mag - yaw_gyro);
    const bool innovation_valid = !state_.initialized || fabsf(innovation) <= YAW_MAG_INNOVATION_GATE_RAD;
    const bool geomag_gate_valid =
        !geomag_reference_.valid || !geomag_reference_.use_for_rejection || state_.geomag_consistent;
    const bool mag_valid = norm > 1.0f && norm_valid && innovation_valid && geomag_gate_valid;
    uint8_t reject_reason = YAW_MAG_REJECT_NONE;
    if (!mag_valid) {
        if (norm <= 1.0f) {
            reject_reason = YAW_MAG_REJECT_NO_FIELD;
        } else if (!norm_valid) {
            reject_reason = YAW_MAG_REJECT_NORM;
        } else if (!innovation_valid) {
            reject_reason = YAW_MAG_REJECT_INNOVATION;
        } else if (!geomag_gate_valid) {
            reject_reason = YAW_MAG_REJECT_GEOMAG;
        }
    }

    const bool has_previous_mag = last_mag_observation_valid_ && last_mag_norm_ > 1.0f;
    const float mag_yaw_delta = has_previous_mag ? fabsf(wrapPi(yaw_mag - last_yaw_mag_rad_)) : 0.0f;
    const float mag_norm_delta_ratio = has_previous_mag
        ? fabsf(norm - last_mag_norm_) / last_mag_norm_
        : 0.0f;
    const bool gyro_yaw_stable = fabsf(gyro_yaw_rate_rad_s) <= YAW_RECAPTURE_MAX_YAW_RATE_RAD_S;
    const bool mag_observation_stable = gyro_yaw_stable && norm > 1.0f && norm_valid && geomag_gate_valid &&
        (!has_previous_mag ||
         (mag_yaw_delta <= YAW_RECAPTURE_YAW_STABILITY_RAD &&
          mag_norm_delta_ratio <= YAW_RECAPTURE_NORM_STABILITY_TOLERANCE));

    state_.yaw_mag_rad = yaw_mag;
    state_.mag_valid = mag_valid;

    if (mag_valid) {
        state_.mag_hold_duration_s = 0.0f;
        state_.recapture_stable_duration_s = 0.0f;
    } else {
        state_.mag_hold_duration_s += mag_dt_s;
        if (reject_reason == YAW_MAG_REJECT_INNOVATION && mag_observation_stable) {
            state_.recapture_stable_duration_s += mag_dt_s;
        } else {
            state_.recapture_stable_duration_s = 0.0f;
        }
        state_.recapture_candidate =
            state_.initialized &&
            reject_reason == YAW_MAG_REJECT_INNOVATION &&
            state_.mag_hold_duration_s >= YAW_RECAPTURE_MIN_HOLD_TIME_S &&
            state_.recapture_stable_duration_s >= YAW_RECAPTURE_STABLE_TIME_S;
    }

    if (norm > 1.0f) {
        last_yaw_mag_rad_ = yaw_mag;
        last_mag_norm_ = norm;
        last_mag_observation_valid_ = true;
    } else {
        last_mag_observation_valid_ = false;
    }

    if (!state_.initialized) {
        state_.yaw_est_rad = mag_valid ? yaw_mag : yaw_gyro;
        state_.initialized = true;
        return state_;
    }

    if (mag_valid) {
        const float alpha = 1.0f - expf(-YAW_CORRECTION_GAIN_RAD_S * mag_dt_s);
        state_.yaw_est_rad = wrapPi(yaw_gyro + alpha * innovation);
    } else if (state_.recapture_candidate) {
        const float alpha = 1.0f - expf(-YAW_CORRECTION_GAIN_RAD_S * mag_dt_s);
        const float correction_step = clampAbs(alpha * innovation, YAW_RECAPTURE_MAX_STEP_RAD);
        state_.yaw_est_rad = wrapPi(yaw_gyro + correction_step);
    } else {
        state_.yaw_est_rad = yaw_gyro;
    }
    return state_;
}
