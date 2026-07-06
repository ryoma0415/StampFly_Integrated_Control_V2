#include "accel_calibration.hpp"
#include <math.h>

namespace {
bool finiteVector(const AccelVector& value) {
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

bool validScaleDenominator(float value) {
    return isfinite(value) && value >= 0.8f && value <= 3.2f;
}
}  // namespace

void AccelSixFaceCalibration::reset() {
    valid_ = false;
    captured_mask_ = 0;
    for (uint8_t i = 0; i < FACE_COUNT; i++) {
        face_mean_[i] = AccelVector{};
    }
    offset_ = AccelVector{};
    scale_ = AccelVector{1.0f, 1.0f, 1.0f};
    last_error_ = "";
}

void AccelSixFaceCalibration::clearCapturedFaces() {
    captured_mask_ = 0;
    for (uint8_t i = 0; i < FACE_COUNT; i++) {
        face_mean_[i] = AccelVector{};
    }
    last_error_ = "";
}

bool AccelSixFaceCalibration::captureFace(AccelCalFace face, const AccelVector& mean_raw_body) {
    if (!finiteVector(mean_raw_body)) {
        setError("accelerometer sample is not finite");
        return false;
    }

    const uint8_t index = faceIndex(face);
    if (index >= FACE_COUNT) {
        setError("invalid accelerometer face");
        return false;
    }

    const float norm = accelNorm(mean_raw_body);
    if (!isfinite(norm) || norm < 0.75f || norm > 1.25f) {
        setError("accelerometer norm is outside the expected stationary range");
        return false;
    }

    const float axis_value = faceAxisValue(face, mean_raw_body);
    const bool positive_face =
        face == AccelCalFace::X_POS || face == AccelCalFace::Y_POS || face == AccelCalFace::Z_POS;
    if ((positive_face && axis_value < 0.60f) || (!positive_face && axis_value > -0.60f)) {
        setError("selected face does not match the measured acceleration direction");
        return false;
    }
    if (!faceOtherAxesWithinLimit(face, mean_raw_body, 0.55f)) {
        setError("accelerometer face is too tilted for capture");
        return false;
    }

    face_mean_[index] = mean_raw_body;
    captured_mask_ |= static_cast<uint8_t>(1U << index);
    last_error_ = "";
    return true;
}

bool AccelSixFaceCalibration::solveFromCapturedFaces() {
    if (!ready()) {
        setError("capture all six accelerometer faces first");
        return false;
    }

    const AccelVector& x_pos = face_mean_[faceIndex(AccelCalFace::X_POS)];
    const AccelVector& x_neg = face_mean_[faceIndex(AccelCalFace::X_NEG)];
    const AccelVector& y_pos = face_mean_[faceIndex(AccelCalFace::Y_POS)];
    const AccelVector& y_neg = face_mean_[faceIndex(AccelCalFace::Y_NEG)];
    const AccelVector& z_pos = face_mean_[faceIndex(AccelCalFace::Z_POS)];
    const AccelVector& z_neg = face_mean_[faceIndex(AccelCalFace::Z_NEG)];

    const float dx = x_pos.x - x_neg.x;
    const float dy = y_pos.y - y_neg.y;
    const float dz = z_pos.z - z_neg.z;
    if (!validScaleDenominator(dx) || !validScaleDenominator(dy) || !validScaleDenominator(dz)) {
        setError("captured accelerometer faces are inconsistent");
        return false;
    }

    const AccelVector offset{
        (x_pos.x + x_neg.x) * 0.5f,
        (y_pos.y + y_neg.y) * 0.5f,
        (z_pos.z + z_neg.z) * 0.5f,
    };
    const AccelVector scale{
        2.0f / dx,
        2.0f / dy,
        2.0f / dz,
    };

    return setCalibration(offset, scale);
}

bool AccelSixFaceCalibration::setCalibration(const AccelVector& offset, const AccelVector& scale) {
    if (!finiteVector(offset) || !finiteVector(scale)) {
        setError("accelerometer calibration contains non-finite values");
        return false;
    }
    if (fabsf(scale.x) < 0.1f || fabsf(scale.y) < 0.1f || fabsf(scale.z) < 0.1f ||
        fabsf(scale.x) > 10.0f || fabsf(scale.y) > 10.0f || fabsf(scale.z) > 10.0f) {
        setError("accelerometer calibration scale is outside the safe range");
        return false;
    }

    offset_ = offset;
    scale_ = scale;
    valid_ = true;
    last_error_ = "";
    return true;
}

AccelVector AccelSixFaceCalibration::apply(const AccelVector& raw_body) const {
    if (!valid_) {
        return raw_body;
    }

    return AccelVector{
        (raw_body.x - offset_.x) * scale_.x,
        (raw_body.y - offset_.y) * scale_.y,
        (raw_body.z - offset_.z) * scale_.z,
    };
}

uint8_t AccelSixFaceCalibration::faceIndex(AccelCalFace face) {
    return static_cast<uint8_t>(face);
}

float AccelSixFaceCalibration::faceAxisValue(AccelCalFace face, const AccelVector& value) {
    switch (face) {
        case AccelCalFace::X_POS:
        case AccelCalFace::X_NEG:
            return value.x;
        case AccelCalFace::Y_POS:
        case AccelCalFace::Y_NEG:
            return value.y;
        case AccelCalFace::Z_POS:
        case AccelCalFace::Z_NEG:
        default:
            return value.z;
    }
}

bool AccelSixFaceCalibration::faceOtherAxesWithinLimit(AccelCalFace face, const AccelVector& value, float limit) {
    switch (face) {
        case AccelCalFace::X_POS:
        case AccelCalFace::X_NEG:
            return fabsf(value.y) <= limit && fabsf(value.z) <= limit;
        case AccelCalFace::Y_POS:
        case AccelCalFace::Y_NEG:
            return fabsf(value.x) <= limit && fabsf(value.z) <= limit;
        case AccelCalFace::Z_POS:
        case AccelCalFace::Z_NEG:
        default:
            return fabsf(value.x) <= limit && fabsf(value.y) <= limit;
    }
}

void AccelSixFaceCalibration::setError(const char* message) {
    last_error_ = message;
}

float accelNorm(const AccelVector& value) {
    return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}
