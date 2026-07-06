// ===========================================================================
// angle_utils.hpp — 角度ユーティリティ + チルト補償(yaw側から移植)
//
// levelMagVectorBody は yaw側で yaw_estimator.cpp(private static)と
// yaw_estimator_kf.hpp(inline 複製)の2箇所に存在した実装を、V2契約 §2.1 に
// 従い本ヘッダへ一本化したもの。式・符号は yaw側と完全同一。
// ===========================================================================
#ifndef STAMPFLY_YAW_ESTIMATION_ANGLE_UTILS_HPP
#define STAMPFLY_YAW_ESTIMATION_ANGLE_UTILS_HPP

#include <Arduino.h>

#include "mag_calibration.hpp"

// Wrap an angle in radians into the (-PI, PI] range.
// Large magnitudes are pre-reduced with fmodf: for float32 inputs above
// ~1.3e8 the subtract-2*PI loop would never terminate (2*PI is below the
// ULP, so value -= 2*PI rounds back to value -> infinite loop / WDT reset),
// and even ~1e7 would stall the sensor loop for millions of iterations.
inline float wrapPi(float value) {
    if (!isfinite(value)) return 0.0f;
    if (fabsf(value) > 100.0f) value = fmodf(value, 2.0f * PI);
    while (value > PI) value -= 2.0f * PI;
    while (value < -PI) value += 2.0f * PI;
    return value;
}

inline float radToDeg(float value) {
    return value * 180.0f / PI;
}

// Convert radians to a 0..360 deg compass-style heading.
inline float radToHeadingDeg(float value) {
    float deg = radToDeg(wrapPi(value));
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

// Subtract a mounting/zero offset (radians) and re-wrap to (-PI, PI].
inline float applyAttitudeOffset(float value, float offset) {
    return wrapPi(value - offset);
}

// チルト補償(水平化)。推定ロール/ピッチで機体磁場ベクトルをレベル座標へ倒す。
// YawEstimator(相補フィルタ)と YawEstimatorKf(EKF)の両方が使う唯一の実装。
//
// Leveling transform using estimated roll/pitch. The mz-term sign below is
// tied to this firmware's effective roll/mag-axis sign convention (the BMI270
// and BMM150 axis swaps make it opposite to the textbook R_x*R_y form).
// Verified on hardware: flipping this sign roughly tripled the roll-induced
// heading swing (~10deg -> ~30deg). Do not "orthonormalize" without re-testing.
inline MagVector levelMagVectorBody(float roll_rad, float pitch_rad, const MagVector& mag_body) {
    const float cr = cosf(roll_rad);
    const float sr = sinf(roll_rad);
    const float cp = cosf(pitch_rad);
    const float sp = sinf(pitch_rad);
    return MagVector{
        mag_body.x * cp + mag_body.z * sp,
        mag_body.x * sr * sp + mag_body.y * cr + mag_body.z * sr * cp,
        -mag_body.x * cr * sp + mag_body.y * sr + mag_body.z * cr * cp,
    };
}

#endif
