// ===========================================================================
// config.hpp — 機体ファームの全チューナブル集約
//
// ARCHITECTURE.md コーディング規約: マジックナンバーはこのファイル以外に
// 置かない。PIDゲインは飛行実績のある OptiTrack版
// (StampFly_OptiTrack_PID_Control_System/M5StampFly/src/flight_control.cpp:77-92)
// の値を踏襲する。機体固有のバイアスはPC側プロファイルで扱い、ここには置かない。
// ===========================================================================
#pragma once

#include <math.h>
#include <stdint.h>

// PIDゲイン1軸分(pid.hpp の PID::set_parameter 引数順に対応)
struct PidGainConfig {
    float kp;   // 比例ゲイン
    float ti;   // 積分時間(大きいほど積分が弱い)
    float td;   // 微分時間
    float eta;  // 不完全微分のフィルタ係数
};

struct FlightConfig {
    // --- カスケードPIDゲイン(OptiTrack版より踏襲) ---
    PidGainConfig roll_rate{0.65f, 0.7f, 0.01f, 0.125f};
    PidGainConfig pitch_rate{0.95f, 0.7f, 0.025f, 0.125f};
    PidGainConfig yaw_rate{3.0f, 0.8f, 0.01f, 0.125f};
    PidGainConfig roll_angle{5.0f, 4.0f, 0.04f, 0.125f};
    PidGainConfig pitch_angle{5.0f, 4.0f, 0.04f, 0.125f};
    PidGainConfig altitude{0.38f, 10.0f, 0.5f, 0.125f};
    PidGainConfig z_velocity{0.08f, 0.95f, 0.08f, 0.125f};

    // --- 制御ループ周期 ---
    float control_period_s = 0.0025f;       // 400Hz 制御周期 [s]
    uint32_t loop_period_us = 2500;         // タイマアラーム周期 [µs]
    uint32_t timer_prescaler = 80;          // 80MHz APBクロック → 1µs ティック
    float altitude_pid_period_s = 0.0333f;  // 高度PIDの設計周期(set_parameter初期値)

    // --- フィルタ時定数 ---
    float duty_filter_tc_s = 0.003f;    // モータデューティ1次LPF
    float thrust_filter_tc_s = 0.01f;   // スラスト指令1次LPF

    // --- モータ・ミキサ ---
    float battery_nominal_v = 3.7f;          // 推力正規化に使う公称電圧 [V]
    float motor_on_duty_threshold = 0.1f;    // これ未満は制御リセット(実質モータ停止)
    float duty_min = 0.0f;                   // モータデューティ下限
    float duty_max = 0.95f;                  // モータデューティ上限
    float mixer_torque_gain = 0.25f;         // 各軸トルク→デューティ配分係数

    // --- 姿勢・高度クランプ(多層クランプのファーム層, ARCHITECTURE.md) ---
    float max_angle_ref_rad = 30.0f * (float)M_PI / 180.0f;  // roll/pitch指令 ±30°
    float alt_ref_min_m = 0.05f;             // CMD_SETPOINT alt_ref クランプ下限 [m]
    float alt_ref_max_m = 1.5f;              // CMD_SETPOINT alt_ref クランプ上限 [m]
    float default_alt_ref_m = 0.3f;          // 離陸時の既定目標高度 [m]
    float alt_sensor_limit_m = 2.0f;         // これ超過で高度センサ喪失扱い(Range0flag加算)

    // --- 離陸シーケンス(OptiTrack版の実績ランプ) ---
    uint16_t takeoff_ramp_phase1_ticks = 500;   // ランプ前半(電圧3.8V仮定の上限)
    uint16_t takeoff_ramp_total_ticks = 1000;   // ランプ全体のtick数
    float takeoff_ramp_divisor = 1000.0f;       // Thrust0 = tick / divisor
    float takeoff_ramp_phase1_v = 3.8f;         // ランプ前半のトリム計算用電圧 [V]
    float takeoff_complete_margin_m = 0.05f;    // alt_ref - margin 到達でHOVERへ

    // --- トリムデューティ(電圧→ホバリングデューティの一次近似) ---
    float trim_duty_slope = -0.2448f;
    float trim_duty_intercept = 1.5892f;

    // --- 高度制御 ---
    float thrust_cmd_max_ratio = 1.15f;     // スラスト指令の Thrust0 比上限
    float thrust_cmd_min_ratio = 0.85f;     // スラスト指令の Thrust0 比下限
    float range_loss_thrust_step = 0.02f;   // 高度センサ喪失時の降下ステップ
    uint8_t range0_flag_max = 20;           // Range0flag の飽和値

    // --- 着陸シーケンス ---
    float landing_z_dot_ref = -0.15f;       // 着陸時の降下速度指令 [m/s]
    float landing_decay_slow = 0.9999f;     // 降下中のスラスト減衰率/tick
    float landing_decay_fast = 0.999f;      // 接地間際のスラスト減衰率/tick
    float landing_near_ground_m = 0.15f;    // 接地間際とみなす高度 [m]
    float landed_threshold_m = 0.1f;        // 着陸完了とみなす高度 [m]

    // --- フェイルセーフ(PROTOCOL.md 規範) ---
    uint32_t link_level_hold_ms = 200;      // setpoint途絶: 水平保持(=setpoint_fresh閾値)
    uint32_t link_loss_landing_ms = 500;    // setpoint途絶: 自動着陸(reason=8)
    float low_voltage_threshold_v = 3.34f;  // 低電圧閾値 [V]
    uint8_t low_voltage_count = 100;        // 低電圧判定の連続tick数
    uint32_t max_flight_time_ms = 120000;   // 最大飛行時間 120s(reason=3)
    float reset_accept_alt_m = 0.15f;       // CMD_RESET 受理高度(COMPLETE時) [m]
    float over_g_threshold_g = 2.0f;        // OverG判定 [g](reason=7)

    // --- キャリブレーション・待機 ---
    uint16_t gyro_calib_samples = 800;      // ジャイロオフセット平均サンプル数
    uint32_t wait_settle_ms = 3000;         // WAITでの静定時間 → AHRSリセット
    uint8_t ahrs_reset_repeat = 20;         // 静定後のAHRSリセット回数

    // --- 通信 ---
    uint8_t wifi_channel = 1;               // ESP-NOWチャネル(PC側機体プロファイルと一致させる)

    // --- テレメトリ・表示レート(400Hzループの分周比) ---
    uint16_t telemetry_state_divider = 16;  // TLM_STATE 25Hz
    uint32_t event_resend_ms = 500;         // TLM_EVENT 2Hz 定期再送
    uint16_t led_show_divider = 16;         // LED更新 25Hz
    uint16_t led_blink_period_ticks = 200;  // 点滅の半周期(0.5s @400Hz)
    uint16_t led_cycle_step_ticks = 21;     // WAITイルミネーションの色送り間隔

    // --- 起動 ---
    uint32_t boot_serial_wait_ms = 1500;    // USBシリアル安定待ち [ms]
};

// 実効コンフィグ(コンパイル時定数)
inline constexpr FlightConfig FLIGHT_CONFIG{};

// ---------------------------------------------------------------------------
// ハードウェア固定値(StampFly ESP32-S3)
// ---------------------------------------------------------------------------

// モータPWM。GPIO5 はモータ(FrontLeft)であり、ブザー等から絶対に触らないこと。
constexpr int PIN_MOTOR_FRONT_LEFT = 5;
constexpr int PIN_MOTOR_FRONT_RIGHT = 42;
constexpr int PIN_MOTOR_REAR_LEFT = 10;
constexpr int PIN_MOTOR_REAR_RIGHT = 41;
constexpr int MOTOR_PWM_FREQ_HZ = 150000;
constexpr int MOTOR_PWM_RESOLUTION_BITS = 8;
constexpr int MOTOR_PWM_MAX_COUNT = (1 << MOTOR_PWM_RESOLUTION_BITS) - 1;
constexpr int LEDC_CH_MOTOR_FRONT_LEFT = 0;
constexpr int LEDC_CH_MOTOR_FRONT_RIGHT = 1;
constexpr int LEDC_CH_MOTOR_REAR_LEFT = 2;
constexpr int LEDC_CH_MOTOR_REAR_RIGHT = 3;

// LED(WS2812: 機体上面2連 + StampS3本体1)
constexpr int PIN_LED_ONBOARD = 39;
constexpr int PIN_LED_ESP = 21;
constexpr int NUM_ONBOARD_LEDS = 2;
constexpr uint8_t LED_BRIGHTNESS = 15;

// ブザー(製品版バグ修正: ピンはGPIO40。GPIO5はモータなので絶対に使わない)
constexpr int PIN_BUZZER = 40;
constexpr int LEDC_CH_BUZZER = 5;  // モータ(ch0-3)とLEDCタイマを共有しないチャネル
constexpr int BUZZER_PWM_RESOLUTION_BITS = 8;
constexpr int BUZZER_BASE_FREQ_HZ = 4000;

// ---------------------------------------------------------------------------
// 流用層(sensor.cpp)互換の定数名。値の正はあくまで FLIGHT_CONFIG。
// ---------------------------------------------------------------------------
inline constexpr float POWER_LIMIT = FLIGHT_CONFIG.low_voltage_threshold_v;
inline constexpr uint8_t UNDER_VOLTAGE_COUNT = FLIGHT_CONFIG.low_voltage_count;
inline constexpr float ALT_LIMIT = FLIGHT_CONFIG.alt_sensor_limit_m;
inline constexpr uint8_t RANGE0_FLAG_MAX = FLIGHT_CONFIG.range0_flag_max;
inline constexpr float OVER_G_THRESHOLD = FLIGHT_CONFIG.over_g_threshold_g;
