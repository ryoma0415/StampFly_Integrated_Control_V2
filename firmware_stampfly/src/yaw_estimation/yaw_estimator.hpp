#ifndef STAMPFLY_YAW_ESTIMATION_YAW_ESTIMATOR_HPP
#define STAMPFLY_YAW_ESTIMATION_YAW_ESTIMATOR_HPP

#include <Arduino.h>
#include "mag_calibration.hpp"

enum YawMagRejectReason : uint8_t {
    YAW_MAG_REJECT_NONE = 0,
    YAW_MAG_REJECT_NO_FIELD = 1,
    YAW_MAG_REJECT_NORM = 2,
    YAW_MAG_REJECT_INNOVATION = 3,
    YAW_MAG_REJECT_GEOMAG = 4,
};

struct GeomagneticReference {
    float declination_east_rad = 0.0f;
    float inclination_rad = 0.0f;
    float horizontal_uT = 0.0f;
    float vertical_uT = 0.0f;
    float total_uT = 0.0f;
    float total_tolerance_ratio = GEOMAG_DEFAULT_TOTAL_TOLERANCE_RATIO;
    float horizontal_tolerance_ratio = GEOMAG_DEFAULT_HORIZONTAL_TOLERANCE_RATIO;
    float inclination_tolerance_rad = GEOMAG_DEFAULT_INCLINATION_TOLERANCE_DEG * DEG_TO_RAD;
    float inclination_z_sign = GEOMAG_INCLINATION_Z_SIGN;
    bool valid = false;
    bool use_for_rejection = true;
};

struct YawEstimate {
    float yaw_est_rad = 0.0f;
    float yaw_mag_rad = 0.0f;
    float heading_mag_rad = 0.0f;
    float heading_true_rad = 0.0f;
    float mag_total_uT = 0.0f;
    float mag_horizontal_uT = 0.0f;
    float mag_inclination_rad = 0.0f;
    float geomag_total_error_ratio = 0.0f;
    float geomag_horizontal_error_ratio = 0.0f;
    float geomag_inclination_error_rad = 0.0f;
    float mag_norm_reference = 0.0f;
    float mag_hold_duration_s = 0.0f;
    float recapture_stable_duration_s = 0.0f;
    bool mag_valid = false;
    bool heading_true_valid = false;
    bool geomag_reference_valid = false;
    bool geomag_consistent = true;
    bool recapture_candidate = false;
    bool initialized = false;
};

class YawEstimator {
public:
    void reset(float yaw_rad = 0.0f);
    void setYawZero(float yaw_rad = 0.0f);
    void clearYawZero();
    // Boot-time restore of a persisted yaw zero: installs the saved magnetic
    // offset directly (no re-capture) and seeds the estimate from the first
    // usable mag sample, so "yaw 0" keeps pointing at the same physical
    // heading across reboots.
    void restoreYawZero(float mag_yaw_offset_rad);
    bool setGeomagneticReference(const GeomagneticReference& reference);
    void clearGeomagneticReference();
    GeomagneticReference geomagneticReference() const { return geomag_reference_; }
    YawEstimate update(
        float gyro_yaw_rate_rad_s,
        float roll_rad,
        float pitch_rad,
        const MagVector& mag_body,
        float dt_s,
        bool mag_sample_fresh,
        float mag_dt_s
    );
    float yaw() const { return state_.yaw_est_rad; }
    // FF補正系の第2インスタンス専用 (ff_pipeline_design.md §5.3-5.4):
    // アンカー/モード切替の再シード(リファレンスからの状態コピー)後に呼び、
    // ノルムゲート基準を補正後磁場で取り直させる(次の有効サンプルで再取得)。
    // リファレンス(非補正)インスタンスでは呼ばないこと — 既存挙動は不変。
    void resetMagNormReference() { state_.mag_norm_reference = 0.0f; }
    bool yawZeroValid() const { return yaw_zero_user_valid_; }
    // True once the user yaw zero has an actual captured magnetic offset
    // (capture happens on the first valid mag sample after setYawZero).
    bool yawZeroOffsetValid() const { return yaw_zero_user_valid_ && mag_reference_valid_; }
    float magYawOffsetRad() const { return mag_yaw_offset_rad_; }

private:
    // チルト補償は angle_utils.hpp の levelMagVectorBody に一本化
    // (yaw側では private static levelMagVector として複製されていた)。
    YawEstimate state_;
    GeomagneticReference geomag_reference_;
    float mag_yaw_offset_rad_ = 0.0f;
    float pending_reference_yaw_rad_ = 0.0f;
    float last_yaw_mag_rad_ = 0.0f;
    float last_mag_norm_ = 0.0f;
    bool mag_reference_pending_ = true;
    bool mag_reference_valid_ = false;
    bool yaw_zero_user_valid_ = false;
    bool last_mag_observation_valid_ = false;
    bool seed_from_mag_pending_ = false;
};

#endif
