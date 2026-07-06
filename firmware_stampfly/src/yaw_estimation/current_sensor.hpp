#ifndef STAMPFLY_YAW_ESTIMATION_CURRENT_SENSOR_HPP
#define STAMPFLY_YAW_ESTIMATION_CURRENT_SENSOR_HPP

#include <Arduino.h>
#include <Wire.h>

// INA3221 battery current/voltage monitor (channel 2 on the shared I2C bus).
// Measures the whole-airframe current draw through the 10 mOhm shunt so motor
// current can be correlated with BMM150 magnetic disturbance.

struct CurrentSample {
    float current_a = 0.0f;      // battery-line current in amperes
    float bus_voltage_v = 0.0f;  // battery bus voltage in volts
    float shunt_uv = 0.0f;       // raw shunt voltage in microvolts
    bool valid = false;
};

// Initialize the INA3221 on the given (already-begun) I2C bus. Returns true if
// the device answers with the expected manufacturer ID.
bool currentSensorInit(TwoWire& wire);

// Read battery current, bus voltage and shunt voltage from channel 2.
CurrentSample currentSensorRead();

#endif
