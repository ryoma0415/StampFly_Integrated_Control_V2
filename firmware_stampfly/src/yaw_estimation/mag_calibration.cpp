#include "mag_calibration.hpp"
#include <math.h>
#include <cstring>

namespace {
float componentByIndex(const MagVector& value, int8_t index) {
    switch (index) {
        case 0:
            return value.x;
        case 1:
            return value.y;
        case 2:
        default:
            return value.z;
    }
}
}  // namespace

void ExpMagFilter::reset() {
    has_value_ = false;
    value_ = MagVector{};
}

MagVector ExpMagFilter::update(const MagVector& input) {
    if (!has_value_) {
        value_ = input;
        has_value_ = true;
        return value_;
    }

    value_.x = alpha_ * input.x + (1.0f - alpha_) * value_.x;
    value_.y = alpha_ * input.y + (1.0f - alpha_) * value_.y;
    value_.z = alpha_ * input.z + (1.0f - alpha_) * value_.z;
    return value_;
}

void MagSoftIronCalibration::reset() {
    valid_ = false;
    offset_ = MagVector{};
    const float identity[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f,
    };
    memcpy(matrix_, identity, sizeof(matrix_));
}

bool MagSoftIronCalibration::set(const MagVector& offset, const float matrix_values[9]) {
    if (matrix_values == nullptr) {
        return false;
    }

    for (size_t i = 0; i < 9; i++) {
        if (!isfinite(matrix_values[i])) {
            return false;
        }
    }
    if (!isfinite(offset.x) || !isfinite(offset.y) || !isfinite(offset.z)) {
        return false;
    }

    offset_ = offset;
    memcpy(matrix_, matrix_values, sizeof(matrix_));
    valid_ = true;
    return true;
}

MagVector MagSoftIronCalibration::apply(const MagVector& raw_body) const {
    if (!valid_) {
        return raw_body;
    }

    const float x = raw_body.x - offset_.x;
    const float y = raw_body.y - offset_.y;
    const float z = raw_body.z - offset_.z;
    return MagVector{
        matrix_[0] * x + matrix_[1] * y + matrix_[2] * z,
        matrix_[3] * x + matrix_[4] * y + matrix_[5] * z,
        matrix_[6] * x + matrix_[7] * y + matrix_[8] * z,
    };
}

MagVector transformMagToBody(const MagVector& sensor_vector) {
    return MagVector{
        MAG_BODY_X_SIGN * componentByIndex(sensor_vector, MAG_BODY_X_SOURCE),
        MAG_BODY_Y_SIGN * componentByIndex(sensor_vector, MAG_BODY_Y_SOURCE),
        MAG_BODY_Z_SIGN * componentByIndex(sensor_vector, MAG_BODY_Z_SOURCE),
    };
}

float magNorm(const MagVector& value) {
    return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}
