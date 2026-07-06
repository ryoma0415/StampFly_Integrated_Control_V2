#ifndef STAMPFLY_YAW_ESTIMATION_MAG_CALIBRATION_HPP
#define STAMPFLY_YAW_ESTIMATION_MAG_CALIBRATION_HPP

#include <Arduino.h>
#include "yaw_config.hpp"

struct MagVector {
    float x;
    float y;
    float z;

    constexpr MagVector() : x(0.0f), y(0.0f), z(0.0f) {}
    constexpr MagVector(float x_value, float y_value, float z_value)
        : x(x_value), y(y_value), z(z_value) {}
};

class ExpMagFilter {
public:
    explicit ExpMagFilter(float alpha = MAG_FILTER_ALPHA) : alpha_(alpha) {}
    void reset();
    MagVector update(const MagVector& input);
    MagVector value() const { return value_; }

private:
    float alpha_ = MAG_FILTER_ALPHA;
    bool has_value_ = false;
    MagVector value_;
};

class MagSoftIronCalibration {
public:
    void reset();
    bool set(const MagVector& offset, const float matrix_values[9]);
    MagVector apply(const MagVector& raw_body) const;

    bool valid() const { return valid_; }
    MagVector offset() const { return offset_; }
    const float* matrix() const { return matrix_; }

private:
    bool valid_ = false;
    MagVector offset_;
    float matrix_[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f,
    };
};

MagVector transformMagToBody(const MagVector& sensor_vector);
float magNorm(const MagVector& value);

#endif
