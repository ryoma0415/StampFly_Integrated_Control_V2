// ===========================================================================
// flight_state.hpp — 飛行制御の共有状態構造体
//
// OptiTrack版 flight_state.hpp の構造化パターンを踏襲しつつ、ジョイスティック
// 時代の死フィールドを除去したもの。SensorState は流用層(sensor.cpp)が
// そのまま書き込むため、フィールド名・volatile指定を原典どおり維持する。
// ===========================================================================
#pragma once

#include <stdint.h>

#include "alt_kalman.hpp"
#include "stampfly_protocol.hpp"

// 飛行状態。数値は PROTOCOL.md の FlightState enum と1対1対応で、
// 流用層(sensor.cpp)が比較演算で使うため plain enum とする。
enum AutoFlightState : uint8_t {
    AUTO_INIT = 0,
    AUTO_CALIBRATION = 1,
    AUTO_WAIT = 2,
    AUTO_TAKEOFF = 3,
    AUTO_HOVER = 4,
    AUTO_LANDING = 5,
    AUTO_COMPLETE = 6,
};

// PROTOCOL.md FlightState との対応をコンパイル時に固定する
static_assert(AUTO_INIT == static_cast<uint8_t>(stampfly::FlightState::INIT), "enum mismatch");
static_assert(AUTO_CALIBRATION == static_cast<uint8_t>(stampfly::FlightState::CALIBRATION), "enum mismatch");
static_assert(AUTO_WAIT == static_cast<uint8_t>(stampfly::FlightState::WAIT), "enum mismatch");
static_assert(AUTO_TAKEOFF == static_cast<uint8_t>(stampfly::FlightState::TAKEOFF), "enum mismatch");
static_assert(AUTO_HOVER == static_cast<uint8_t>(stampfly::FlightState::HOVER), "enum mismatch");
static_assert(AUTO_LANDING == static_cast<uint8_t>(stampfly::FlightState::LANDING), "enum mismatch");
static_assert(AUTO_COMPLETE == static_cast<uint8_t>(stampfly::FlightState::COMPLETE), "enum mismatch");

// ループタイミング。Loop_flag のみISR(400Hzタイマ)が書くため volatile。
struct FlightTimingState {
    volatile uint8_t Loop_flag = 0;   // タイマISRが1にし、ループ先頭で0に戻す
    float Control_period = 0.0f;      // 設計制御周期 [s](init_copterでconfigから設定)
    float Elapsed_time = 0.0f;        // 計測開始(WAIT遷移)からの経過 [s]
    float Old_Elapsed_time = 0.0f;
    float Interval_time = 0.0f;       // 直近の実測制御周期 [s]
    uint32_t S_time = 0;              // 計測開始時刻 [µs]
    uint32_t E_time = 0;              // 今tickの時刻 [µs]
};

// 状態機械の状態
struct FlightModeState {
    AutoFlightState auto_state = AUTO_INIT;
    uint8_t last_reason = 0;        // 直近の遷移理由(stampfly::Reason)
    uint32_t flight_start_ms = 0;   // TAKEOFF開始時刻(最大飛行時間の起点)[ms]
    uint32_t phase_start_ms = 0;    // 現在フェーズの開始時刻 [ms]
    uint8_t Alt_flag = 0;           // 高度制御有効(流用層が高度上限検査で参照)
};

// 制御出力(400Hzループ内のみで読み書きする)
struct ControlOutputState {
    float FrontRight_motor_duty = 0.0f;
    float FrontLeft_motor_duty = 0.0f;
    float RearRight_motor_duty = 0.0f;
    float RearLeft_motor_duty = 0.0f;
    float Roll_rate_reference = 0.0f;    // 角度PID出力 = 角速度目標 [rad/s]
    float Pitch_rate_reference = 0.0f;
    float Yaw_rate_reference = 0.0f;     // ヨーは常に0(無制御)
    float Roll_angle_reference = 0.0f;   // クランプ後の角度目標 [rad]
    float Pitch_angle_reference = 0.0f;
    float Roll_rate_command = 0.0f;      // 角速度PID出力(トルク相当)
    float Pitch_rate_command = 0.0f;
    float Yaw_rate_command = 0.0f;
    float Roll_angle_command = 0.0f;     // 適用中の姿勢指令 [rad](水平保持含む)
    float Pitch_angle_command = 0.0f;
    float Roll_angle_offset = 0.0f;      // 角度誤差計算用オフセット(流用)
    float Pitch_angle_offset = 0.0f;
    float Thrust_command = 0.0f;         // スラスト指令(電圧スケール)
    float Thrust0 = 0.0f;                // 正規化ベーススラスト(0-1)
    float Alt_ref = 0.0f;                // 目標高度 [m](CMD_SETPOINTでクランプ更新)
    float Z_dot_ref = 0.0f;              // 高度PID出力 = 上下速度目標 [m/s]
};

// CMD_SETPOINT の適用追跡(seq_echo・リンク鮮度の根拠)
struct CommandTrackingState {
    bool setpoint_received = false;     // 一度でも有効なsetpointを受けたか
    uint32_t applied_setpoint_seq = 0;  // 最後に適用した CMD_SETPOINT の seq(未受信なら0)
    uint32_t last_setpoint_ms = 0;      // 最後のsetpoint受信時刻 [ms]
    float target_roll_rad = 0.0f;       // 受信した姿勢目標 [rad]
    float target_pitch_rad = 0.0f;
};

struct FlightControlState {
    FlightTimingState timing;
    FlightModeState mode;
    ControlOutputState output;
    CommandTrackingState command;
};

// センサ状態(流用層 sensor.cpp が書き込む。原典 flight_state.hpp と同一)
struct SensorState {
    volatile float Roll_angle = 0.0f;
    volatile float Pitch_angle = 0.0f;
    volatile float Yaw_angle = 0.0f;
    volatile float Roll_rate = 0.0f;
    volatile float Pitch_rate = 0.0f;
    volatile float Yaw_rate = 0.0f;
    volatile float Accel_x_raw = 0.0f;
    volatile float Accel_y_raw = 0.0f;
    volatile float Accel_z_raw = 0.0f;
    volatile float Accel_x = 0.0f;
    volatile float Accel_y = 0.0f;
    volatile float Accel_z = 0.0f;
    volatile float Accel_z_d = 0.0f;
    volatile int16_t RawRange = 0;
    volatile int16_t Range = 0;
    volatile int16_t RawRangeFront = 0;
    volatile int16_t RangeFront = 0;
    volatile float Altitude = 0.0f;
    volatile float Altitude2 = 0.0f;
    volatile float Alt_velocity = 0.0f;
    volatile float Voltage = 0.0f;
    float Acc_norm = 0.0f;
    float Over_g = 0.0f;
    float Over_rate = 0.0f;
    uint8_t OverG_flag = 0;
    uint8_t Range0flag = 0;
    volatile uint8_t Under_voltage_flag = 0;
    volatile float Az = 0.0f;
    volatile float Az_bias = 0.0f;
    Alt_kalman EstimatedAltitude;
};
