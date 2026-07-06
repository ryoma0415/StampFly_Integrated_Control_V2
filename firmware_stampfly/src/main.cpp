// ===========================================================================
// main.cpp — StampFly Integrated Control 機体ファーム エントリポイント
//
// 製品版と同じ構成: setup() で init_copter()、loop() で loop_400Hz()。
// すべての実装は flight_control / comm / telemetry / indicators 各モジュールと
// OptiTrack版から流用した飛行実績層(sensor/imu/tof/alt_kalman/pid)にある。
// ===========================================================================
#include <Arduino.h>

#include "flight_control.hpp"

void setup() {
    init_copter();
}

void loop() {
    loop_400Hz();
}
