#ifndef STAMPFLY_YAW_ESTIMATION_BMM150_DRIVER_HPP
#define STAMPFLY_YAW_ESTIMATION_BMM150_DRIVER_HPP

#include <Arduino.h>
#include <Wire.h>
#include "yaw_config.hpp"
#include "mag_calibration.hpp"

struct Bmm150TrimData {
    int8_t dig_x1 = 0;
    int8_t dig_y1 = 0;
    int8_t dig_x2 = 0;
    int8_t dig_y2 = 0;
    uint16_t dig_z1 = 0;
    int16_t dig_z2 = 0;
    int16_t dig_z3 = 0;
    int16_t dig_z4 = 0;
    uint8_t dig_xy1 = 0;
    int8_t dig_xy2 = 0;
    uint16_t dig_xyz1 = 0;
};

struct Bmm150RawSample {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    uint16_t rhall = 0;
    MagVector compensated_sensor;
    bool data_ready = false;
    bool compensated_valid = false;
    uint32_t timestamp_ms = 0;
};

class Bmm150Driver {
public:
    bool begin(TwoWire& wire, uint8_t address = BMM150_I2C_ADDRESS);
    bool readRaw(Bmm150RawSample& sample);
    bool isInitialized() const { return initialized_; }
    bool trimValid() const { return trim_valid_; }
    uint8_t chipId() const { return chip_id_; }
    uint32_t errorCount() const { return error_count_; }

private:
    static int16_t signExtend(uint16_t value, uint8_t bits);

    bool readTrimRegisters();
    bool compensate(const Bmm150RawSample& sample, MagVector& compensated) const;
    bool compensateXY(int16_t raw, uint16_t rhall, int8_t dig_axis1, int8_t dig_axis2, float& compensated) const;
    bool compensateZ(int16_t raw_z, uint16_t rhall, float& compensated_z) const;
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool readRegisters(uint8_t start_reg, uint8_t* buffer, size_t length);
    void noteError();

    TwoWire* wire_ = nullptr;
    uint8_t address_ = BMM150_I2C_ADDRESS;
    uint8_t chip_id_ = 0;
    Bmm150TrimData trim_;
    bool initialized_ = false;
    bool trim_valid_ = false;
    uint32_t error_count_ = 0;
};

#endif
