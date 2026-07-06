#include "bmm150_driver.hpp"

namespace {
static const uint8_t REG_CHIP_ID = 0x40;
static const uint8_t REG_DATA_X_LSB = 0x42;
static const uint8_t REG_POWER_CONTROL = 0x4B;
static const uint8_t REG_OP_MODE = 0x4C;
static const uint8_t REG_REP_XY = 0x51;
static const uint8_t REG_REP_Z = 0x52;
static const uint8_t REG_TRIM_X1 = 0x5D;
static const uint8_t REG_TRIM_Z4_LSB = 0x62;
static const uint8_t REG_TRIM_Z2_LSB = 0x68;

static const uint8_t POWER_CONTROL_ON = 0x01;
static const uint8_t OP_MODE_NORMAL_10HZ = 0x00;
static const uint8_t REP_XY_REGULAR = 0x04;
static const uint8_t REP_Z_REGULAR = 0x0E;

static const int16_t OVERFLOW_ADCVAL_XYAXES = -4096;
static const int16_t OVERFLOW_ADCVAL_ZAXIS = -16384;
}  // namespace

bool Bmm150Driver::begin(TwoWire& wire, uint8_t address) {
    wire_ = &wire;
    address_ = address;
    chip_id_ = 0;
    trim_ = Bmm150TrimData{};
    initialized_ = false;
    trim_valid_ = false;

    if (!writeRegister(REG_POWER_CONTROL, POWER_CONTROL_ON)) {
        return false;
    }
    delay(5);

    uint8_t id = 0;
    if (!readRegister(REG_CHIP_ID, id)) {
        return false;
    }
    chip_id_ = id;
    if (chip_id_ != BMM150_EXPECTED_CHIP_ID) {
        noteError();
        return false;
    }

    if (!readTrimRegisters()) {
        return false;
    }

    if (!writeRegister(REG_REP_XY, REP_XY_REGULAR)) {
        return false;
    }
    if (!writeRegister(REG_REP_Z, REP_Z_REGULAR)) {
        return false;
    }
    if (!writeRegister(REG_OP_MODE, OP_MODE_NORMAL_10HZ)) {
        return false;
    }

    delay(10);
    initialized_ = true;
    return true;
}

bool Bmm150Driver::readRaw(Bmm150RawSample& sample) {
    if (!initialized_ || wire_ == nullptr) {
        noteError();
        return false;
    }

    uint8_t data[8] = {0};
    if (!readRegisters(REG_DATA_X_LSB, data, sizeof(data))) {
        return false;
    }

    uint16_t x_raw = (static_cast<uint16_t>(data[1]) << 5) | (data[0] >> 3);
    uint16_t y_raw = (static_cast<uint16_t>(data[3]) << 5) | (data[2] >> 3);
    uint16_t z_raw = (static_cast<uint16_t>(data[5]) << 7) | (data[4] >> 1);
    uint16_t rhall_raw = (static_cast<uint16_t>(data[7]) << 6) | (data[6] >> 2);

    sample.x = signExtend(x_raw, 13);
    sample.y = signExtend(y_raw, 13);
    sample.z = signExtend(z_raw, 15);
    sample.rhall = rhall_raw;
    sample.data_ready = (data[6] & 0x01) != 0;
    sample.compensated_sensor = MagVector{};
    sample.compensated_valid = sample.data_ready && compensate(sample, sample.compensated_sensor);
    sample.timestamp_ms = millis();
    return true;
}

bool Bmm150Driver::readTrimRegisters() {
    uint8_t trim_x1y1[2] = {0};
    uint8_t trim_z4xy2[4] = {0};
    uint8_t trim_z2xy1[10] = {0};

    if (!readRegisters(REG_TRIM_X1, trim_x1y1, sizeof(trim_x1y1))) {
        return false;
    }
    if (!readRegisters(REG_TRIM_Z4_LSB, trim_z4xy2, sizeof(trim_z4xy2))) {
        return false;
    }
    if (!readRegisters(REG_TRIM_Z2_LSB, trim_z2xy1, sizeof(trim_z2xy1))) {
        return false;
    }

    trim_.dig_x1 = static_cast<int8_t>(trim_x1y1[0]);
    trim_.dig_y1 = static_cast<int8_t>(trim_x1y1[1]);
    trim_.dig_x2 = static_cast<int8_t>(trim_z4xy2[2]);
    trim_.dig_y2 = static_cast<int8_t>(trim_z4xy2[3]);
    trim_.dig_z1 = static_cast<uint16_t>((static_cast<uint16_t>(trim_z2xy1[3]) << 8) | trim_z2xy1[2]);
    trim_.dig_z2 = static_cast<int16_t>((static_cast<uint16_t>(trim_z2xy1[1]) << 8) | trim_z2xy1[0]);
    trim_.dig_z3 = static_cast<int16_t>((static_cast<uint16_t>(trim_z2xy1[7]) << 8) | trim_z2xy1[6]);
    trim_.dig_z4 = static_cast<int16_t>((static_cast<uint16_t>(trim_z4xy2[1]) << 8) | trim_z4xy2[0]);
    trim_.dig_xy1 = trim_z2xy1[9];
    trim_.dig_xy2 = static_cast<int8_t>(trim_z2xy1[8]);
    trim_.dig_xyz1 = static_cast<uint16_t>((static_cast<uint16_t>(trim_z2xy1[5] & 0x7F) << 8) | trim_z2xy1[4]);

    trim_valid_ = trim_.dig_xyz1 != 0 && trim_.dig_z1 != 0 && trim_.dig_z2 != 0;
    if (!trim_valid_) {
        noteError();
    }
    return trim_valid_;
}

bool Bmm150Driver::compensate(const Bmm150RawSample& sample, MagVector& compensated) const {
    return compensateXY(sample.x, sample.rhall, trim_.dig_x1, trim_.dig_x2, compensated.x)
        && compensateXY(sample.y, sample.rhall, trim_.dig_y1, trim_.dig_y2, compensated.y)
        && compensateZ(sample.z, sample.rhall, compensated.z);
}

// Equations follow the Bosch Sensortec BMM150 Sensor API compensation flow.
// X and Y share the same form; only the per-axis trim values (dig_axis1 from the
// dig_x1/dig_y1 offset and dig_axis2 from the dig_x2/dig_y2 gain) differ.
bool Bmm150Driver::compensateXY(int16_t raw, uint16_t rhall, int8_t dig_axis1, int8_t dig_axis2, float& compensated) const {
    if (!trim_valid_ || raw == OVERFLOW_ADCVAL_XYAXES || rhall == 0 || trim_.dig_xyz1 == 0) {
        compensated = 0.0f;
        return false;
    }

    const float hall_ratio = static_cast<float>(trim_.dig_xyz1) * 16384.0f / static_cast<float>(rhall);
    const float hall_delta = hall_ratio - 16384.0f;
    const float quadratic = static_cast<float>(trim_.dig_xy2) * hall_delta * hall_delta / 268435456.0f;
    const float linear = hall_delta * static_cast<float>(trim_.dig_xy1) / 16384.0f;
    const float axis_gain = static_cast<float>(dig_axis2) + 160.0f;
    compensated = (((static_cast<float>(raw) * ((quadratic + linear + 256.0f) * axis_gain)) / 8192.0f)
        + static_cast<float>(dig_axis1) * 8.0f) / 16.0f;
    return true;
}

bool Bmm150Driver::compensateZ(int16_t raw_z, uint16_t rhall, float& compensated_z) const {
    if (!trim_valid_ || raw_z == OVERFLOW_ADCVAL_ZAXIS || rhall == 0 ||
        trim_.dig_z1 == 0 || trim_.dig_z2 == 0 || trim_.dig_xyz1 == 0) {
        compensated_z = 0.0f;
        return false;
    }

    const float sensor_minus_offset = static_cast<float>(raw_z) - static_cast<float>(trim_.dig_z4);
    const float hall_delta = static_cast<float>(rhall) - static_cast<float>(trim_.dig_xyz1);
    const float hall_z = static_cast<float>(trim_.dig_z3) * hall_delta;
    const float denominator = (static_cast<float>(trim_.dig_z2)
        + static_cast<float>(trim_.dig_z1) * static_cast<float>(rhall) / 32768.0f) * 4.0f;
    if (denominator == 0.0f) {
        compensated_z = 0.0f;
        return false;
    }

    compensated_z = (((sensor_minus_offset * 131072.0f) - hall_z) / denominator) / 16.0f;
    return true;
}

int16_t Bmm150Driver::signExtend(uint16_t value, uint8_t bits) {
    const uint16_t sign_bit = static_cast<uint16_t>(1U << (bits - 1));
    if ((value & sign_bit) != 0) {
        value |= static_cast<uint16_t>(0xFFFFU << bits);
    }
    return static_cast<int16_t>(value);
}

bool Bmm150Driver::writeRegister(uint8_t reg, uint8_t value) {
    if (wire_ == nullptr) {
        noteError();
        return false;
    }

    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    if (wire_->endTransmission() != 0) {
        noteError();
        return false;
    }
    return true;
}

bool Bmm150Driver::readRegister(uint8_t reg, uint8_t& value) {
    if (!readRegisters(reg, &value, 1)) {
        return false;
    }
    return true;
}

bool Bmm150Driver::readRegisters(uint8_t start_reg, uint8_t* buffer, size_t length) {
    if (wire_ == nullptr || buffer == nullptr || length == 0) {
        noteError();
        return false;
    }

    wire_->beginTransmission(address_);
    wire_->write(start_reg);
    if (wire_->endTransmission(false) != 0) {
        noteError();
        return false;
    }

    const uint8_t requested = static_cast<uint8_t>(length);
    size_t received = wire_->requestFrom(static_cast<int>(address_), static_cast<int>(requested));
    if (received != length) {
        noteError();
        while (wire_->available() > 0) {
            wire_->read();
        }
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        buffer[i] = static_cast<uint8_t>(wire_->read());
    }
    return true;
}

void Bmm150Driver::noteError() {
    error_count_++;
}
