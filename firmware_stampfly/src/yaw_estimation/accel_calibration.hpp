#ifndef STAMPFLY_YAW_ESTIMATION_ACCEL_CALIBRATION_HPP
#define STAMPFLY_YAW_ESTIMATION_ACCEL_CALIBRATION_HPP

#include <Arduino.h>

struct AccelVector {
    float x;
    float y;
    float z;

    constexpr AccelVector() : x(0.0f), y(0.0f), z(0.0f) {}
    constexpr AccelVector(float x_value, float y_value, float z_value)
        : x(x_value), y(y_value), z(z_value) {}
};

enum class AccelCalFace : uint8_t {
    X_POS = 0,
    X_NEG = 1,
    Y_POS = 2,
    Y_NEG = 3,
    Z_POS = 4,
    Z_NEG = 5,
};

class AccelSixFaceCalibration {
public:
    void reset();
    void clearCapturedFaces();
    bool captureFace(AccelCalFace face, const AccelVector& mean_raw_body);
    bool solveFromCapturedFaces();
    bool setCalibration(const AccelVector& offset, const AccelVector& scale);
    AccelVector apply(const AccelVector& raw_body) const;

    bool valid() const { return valid_; }
    bool ready() const { return captured_mask_ == ALL_FACES_MASK; }
    uint8_t capturedMask() const { return captured_mask_; }
    AccelVector offset() const { return offset_; }
    AccelVector scale() const { return scale_; }
    const char* lastError() const { return last_error_; }

private:
    static constexpr uint8_t FACE_COUNT = 6;
    static constexpr uint8_t ALL_FACES_MASK = 0x3f;

    static uint8_t faceIndex(AccelCalFace face);
    static float faceAxisValue(AccelCalFace face, const AccelVector& value);
    static bool faceOtherAxesWithinLimit(AccelCalFace face, const AccelVector& value, float limit);
    void setError(const char* message);

    bool valid_ = false;
    uint8_t captured_mask_ = 0;
    AccelVector face_mean_[FACE_COUNT];
    AccelVector offset_;
    AccelVector scale_{1.0f, 1.0f, 1.0f};
    const char* last_error_ = "";
};

float accelNorm(const AccelVector& value);

#endif
