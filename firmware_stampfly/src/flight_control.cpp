// ===========================================================================
// flight_control.cpp — 状態機械 + カスケードPID + ミキサ 実装
//
// 制御則(角度→角速度→ミキサ、高度→上下速度→スラスト)と離着陸シーケンスは
// 飛行実績のある OptiTrack版を踏襲。コマンド入出力は PROTOCOL.md v1 に従い
// comm/telemetry モジュール経由で行う。定常状態でUSBシリアルへは出力しない。
// ===========================================================================
#include "flight_control.hpp"

#include <Arduino.h>

#include "comm.hpp"
#include "indicators.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "telemetry.hpp"

FlightControlState flight_control_state;

namespace {

using stampfly::Reason;

// --- PID制御器・フィルタ(OptiTrack版と同構成) ---
PID p_pid;      // ロール角速度
PID q_pid;      // ピッチ角速度
PID r_pid;      // ヨー角速度
PID phi_pid;    // ロール角
PID theta_pid;  // ピッチ角
PID alt_pid;    // 高度 → 上下速度目標
PID z_dot_pid;  // 上下速度 → スラスト補正
Filter thrust_filtered;
Filter duty_filter_fr;
Filter duty_filter_fl;
Filter duty_filter_rr;
Filter duty_filter_rl;

// --- シーケンス内部状態 ---
uint16_t offset_counter = 0;       // ジャイロオフセット平均の進捗
uint16_t auto_takeoff_counter = 0; // 離陸スラストランプの進捗
uint8_t landing_state = 0;         // 着陸シーケンス初期化済みフラグ
uint8_t prev_range0_flag = 0;      // 高度センサ喪失の前回値
bool wait_ready = false;           // WAITでの静定+AHRSリセット完了

// 400Hzタイマ割り込み: フラグを立てるだけ(ISR内で他の処理はしない)
hw_timer_t* loop_timer = nullptr;
void IRAM_ATTR on_loop_timer() {
    flight_control_state.timing.Loop_flag = 1;
}

// ---------------------------------------------------------------------------
// モータPWM
// ---------------------------------------------------------------------------

void set_duty_fr(float duty) { ledcWrite(LEDC_CH_MOTOR_FRONT_RIGHT, (uint32_t)(MOTOR_PWM_MAX_COUNT * duty)); }
void set_duty_fl(float duty) { ledcWrite(LEDC_CH_MOTOR_FRONT_LEFT, (uint32_t)(MOTOR_PWM_MAX_COUNT * duty)); }
void set_duty_rr(float duty) { ledcWrite(LEDC_CH_MOTOR_REAR_RIGHT, (uint32_t)(MOTOR_PWM_MAX_COUNT * duty)); }
void set_duty_rl(float duty) { ledcWrite(LEDC_CH_MOTOR_REAR_LEFT, (uint32_t)(MOTOR_PWM_MAX_COUNT * duty)); }

void init_pwm(void) {
    ledcSetup(LEDC_CH_MOTOR_FRONT_LEFT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_MOTOR_FRONT_RIGHT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_MOTOR_REAR_LEFT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_MOTOR_REAR_RIGHT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_FRONT_LEFT, LEDC_CH_MOTOR_FRONT_LEFT);
    ledcAttachPin(PIN_MOTOR_FRONT_RIGHT, LEDC_CH_MOTOR_FRONT_RIGHT);
    ledcAttachPin(PIN_MOTOR_REAR_LEFT, LEDC_CH_MOTOR_REAR_LEFT);
    ledcAttachPin(PIN_MOTOR_REAR_RIGHT, LEDC_CH_MOTOR_REAR_RIGHT);
}

void motor_stop(void) {
    set_duty_fr(0.0f);
    set_duty_fl(0.0f);
    set_duty_rr(0.0f);
    set_duty_rl(0.0f);
}

// ---------------------------------------------------------------------------
// ユーティリティ
// ---------------------------------------------------------------------------

// 電圧→ホバリング付近のトリムデューティ(OptiTrack版の実測一次近似)
float get_trim_duty(float voltage) {
    return FLIGHT_CONFIG.trim_duty_slope * voltage + FLIGHT_CONFIG.trim_duty_intercept;
}

bool low_voltage_detected(void) {
    return sensor_state.Under_voltage_flag >= UNDER_VOLTAGE_COUNT;
}

// 状態遷移(同一状態への遷移は無視)。TLM_EVENT即時送信+LED/ブザー通知。
void transition_to(AutoFlightState next, Reason reason) {
    auto& mode = flight_control_state.mode;
    const AutoFlightState prev = mode.auto_state;
    if (prev == next) return;
    mode.auto_state = next;
    mode.last_reason = static_cast<uint8_t>(reason);
    mode.phase_start_ms = millis();
    telemetry_notify_transition(static_cast<uint8_t>(next), static_cast<uint8_t>(prev),
                                static_cast<uint8_t>(reason));
    indicators_notify_transition(static_cast<uint8_t>(next), static_cast<uint8_t>(prev),
                                 static_cast<uint8_t>(reason));
}

// WAITへ入る(目標高度を既定値へ戻し、高度推定器を飛行前状態に初期化)
void enter_wait(Reason reason) {
    flight_control_state.output.Alt_ref = FLIGHT_CONFIG.default_alt_ref_m;
    sensor_state.EstimatedAltitude.reset();
    transition_to(AUTO_WAIT, reason);
}

// START拒否(状態遷移なし)。理由つきTLM_EVENTで即時通知する。
void reject_start(Reason reason) {
    auto& mode = flight_control_state.mode;
    mode.last_reason = static_cast<uint8_t>(reason);
    telemetry_notify_transition(static_cast<uint8_t>(mode.auto_state),
                                static_cast<uint8_t>(mode.auto_state),
                                static_cast<uint8_t>(reason));
}

// 飛行中のリンク途絶経過 [ms]。離陸直後は flight_start_ms を起点に猶予を与える。
uint32_t setpoint_link_age_ms(uint32_t now_ms) {
    const auto& cs = flight_control_state.command;
    const auto& mode = flight_control_state.mode;
    uint32_t reference = mode.flight_start_ms;
    if (cs.setpoint_received) {
        // どちらか新しい方を起点にする(millisラップ安全な比較)
        if ((int32_t)(cs.last_setpoint_ms - reference) > 0) reference = cs.last_setpoint_ms;
    }
    return now_ms - reference;
}

// ---------------------------------------------------------------------------
// 制御初期化
// ---------------------------------------------------------------------------

void control_init(void) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    const float h = flight_control_state.timing.Control_period;

    // 角速度制御
    p_pid.set_parameter(cfg.roll_rate.kp, cfg.roll_rate.ti, cfg.roll_rate.td, cfg.roll_rate.eta, h);
    q_pid.set_parameter(cfg.pitch_rate.kp, cfg.pitch_rate.ti, cfg.pitch_rate.td, cfg.pitch_rate.eta, h);
    r_pid.set_parameter(cfg.yaw_rate.kp, cfg.yaw_rate.ti, cfg.yaw_rate.td, cfg.yaw_rate.eta, h);

    // 角度制御
    phi_pid.set_parameter(cfg.roll_angle.kp, cfg.roll_angle.ti, cfg.roll_angle.td, cfg.roll_angle.eta, h);
    theta_pid.set_parameter(cfg.pitch_angle.kp, cfg.pitch_angle.ti, cfg.pitch_angle.td, cfg.pitch_angle.eta, h);

    // 高度制御
    alt_pid.set_parameter(cfg.altitude.kp, cfg.altitude.ti, cfg.altitude.td, cfg.altitude.eta,
                          cfg.altitude_pid_period_s);
    z_dot_pid.set_parameter(cfg.z_velocity.kp, cfg.z_velocity.ti, cfg.z_velocity.td, cfg.z_velocity.eta,
                            cfg.altitude_pid_period_s);

    // 出力フィルタ
    duty_filter_fr.set_parameter(cfg.duty_filter_tc_s, h);
    duty_filter_fl.set_parameter(cfg.duty_filter_tc_s, h);
    duty_filter_rr.set_parameter(cfg.duty_filter_tc_s, h);
    duty_filter_rl.set_parameter(cfg.duty_filter_tc_s, h);
    thrust_filtered.set_parameter(cfg.thrust_filter_tc_s, h);
}

void reset_duty_filters(void) {
    duty_filter_fr.reset();
    duty_filter_fl.reset();
    duty_filter_rr.reset();
    duty_filter_rl.reset();
}

// ---------------------------------------------------------------------------
// コマンド適用
// ---------------------------------------------------------------------------

// 受信した CMD_SETPOINT を適用する(ハートビート時刻・seq_echo の更新を含む)。
// alt_ref は flags bit0 が立っている場合のみ、WAIT/TAKEOFF/HOVER でクランプ更新。
void apply_setpoint(const CommandSnapshot& cmd) {
    if (!cmd.setpoint_pending) return;
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& cs = flight_control_state.command;
    auto& out = flight_control_state.output;
    const AutoFlightState st = flight_control_state.mode.auto_state;

    cs.setpoint_received = true;
    cs.last_setpoint_ms = cmd.setpoint_rx_ms;
    cs.applied_setpoint_seq = cmd.setpoint_seq;
    cs.target_roll_rad = cmd.setpoint.roll_ref;
    cs.target_pitch_rad = cmd.setpoint.pitch_ref;

    if ((cmd.setpoint.flags & stampfly::CmdSetpoint::FLAG_ALT_REF_VALID) != 0 &&
        (st == AUTO_WAIT || st == AUTO_TAKEOFF || st == AUTO_HOVER)) {
        float alt = cmd.setpoint.alt_ref;
        if (alt < cfg.alt_ref_min_m) alt = cfg.alt_ref_min_m;
        if (alt > cfg.alt_ref_max_m) alt = cfg.alt_ref_max_m;
        out.Alt_ref = alt;
    }
}

// 飛行中(TAKEOFF/HOVER)のスラストベース+姿勢指令を更新する(OptiTrack版 get_command 相当)
void update_thrust_and_attitude_command(void) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& out = flight_control_state.output;
    auto& mode = flight_control_state.mode;
    const auto& cs = flight_control_state.command;

    // 高度自動制御ON
    mode.Alt_flag = 1;

    // 離陸スラストランプ(実績シーケンス)
    if (auto_takeoff_counter < cfg.takeoff_ramp_phase1_ticks) {
        out.Thrust0 = (float)auto_takeoff_counter / cfg.takeoff_ramp_divisor;
        const float cap = get_trim_duty(cfg.takeoff_ramp_phase1_v);
        if (out.Thrust0 > cap) out.Thrust0 = cap;
        auto_takeoff_counter++;
    } else if (auto_takeoff_counter < cfg.takeoff_ramp_total_ticks) {
        out.Thrust0 = (float)auto_takeoff_counter / cfg.takeoff_ramp_divisor;
        const float cap = get_trim_duty(sensor_state.Voltage);
        if (out.Thrust0 > cap) out.Thrust0 = cap;
        auto_takeoff_counter++;
    } else {
        out.Thrust0 = get_trim_duty(sensor_state.Voltage);
    }

    // 高度センサ喪失時の安全降下(流用ロジック)
    if ((sensor_state.Range0flag > prev_range0_flag) || (sensor_state.Range0flag == RANGE0_FLAG_MAX)) {
        out.Thrust0 = out.Thrust0 - cfg.range_loss_thrust_step;
        prev_range0_flag = sensor_state.Range0flag;
    }
    out.Thrust_command = out.Thrust0 * cfg.battery_nominal_v;

    // 姿勢指令: setpointが新鮮なら適用、200ms超は水平保持(PROTOCOL.md フェイルセーフ)。
    // 機体差バイアスはPC側プロファイルで加算済み(ファームには置かない)。
    const uint32_t now_ms = millis();
    if (cs.setpoint_received && (now_ms - cs.last_setpoint_ms) < cfg.link_level_hold_ms) {
        out.Roll_angle_command = cs.target_roll_rad;
        out.Pitch_angle_command = cs.target_pitch_rad;
    } else {
        out.Roll_angle_command = 0.0f;
        out.Pitch_angle_command = 0.0f;
    }

    // ヨーは常に0(無制御)
    out.Yaw_rate_reference = 0.0f;
}

// ---------------------------------------------------------------------------
// 制御則(OptiTrack版を踏襲)
// ---------------------------------------------------------------------------

void reset_rate_control(void) {
    auto& out = flight_control_state.output;
    motor_stop();
    out.FrontRight_motor_duty = 0.0f;
    out.FrontLeft_motor_duty = 0.0f;
    out.RearRight_motor_duty = 0.0f;
    out.RearLeft_motor_duty = 0.0f;
    reset_duty_filters();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    alt_pid.reset();
    z_dot_pid.reset();
    out.Roll_rate_reference = 0.0f;
    out.Pitch_rate_reference = 0.0f;
    out.Yaw_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(out.Roll_angle_reference);
    theta_pid.set_error(out.Pitch_angle_reference);
    out.Roll_angle_offset = 0.0f;
    out.Pitch_angle_offset = 0.0f;
}

void reset_angle_control(void) {
    auto& out = flight_control_state.output;
    out.Roll_rate_reference = 0.0f;
    out.Pitch_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(out.Roll_angle_reference);
    theta_pid.set_error(out.Pitch_angle_reference);
    out.Roll_angle_offset = 0.0f;
    out.Pitch_angle_offset = 0.0f;
}

// 角度制御(外側ループ): 高度PID + 姿勢角PID → 角速度目標
void angle_control(void) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& out = flight_control_state.output;
    const float dt = flight_control_state.timing.Interval_time;

    if (out.Thrust_command / cfg.battery_nominal_v < cfg.motor_on_duty_threshold) {
        reset_angle_control();
        return;
    }

    // 高度PID → 上下速度目標
    if (flight_control_state.mode.Alt_flag >= 1) {
        const float alt_err = out.Alt_ref - sensor_state.Altitude2;
        out.Z_dot_ref = alt_pid.update(alt_err, dt);
    }

    // 姿勢角目標(ファーム層クランプ ±30°)
    out.Roll_angle_reference = out.Roll_angle_command;
    out.Pitch_angle_reference = out.Pitch_angle_command;
    if (out.Roll_angle_reference > cfg.max_angle_ref_rad) out.Roll_angle_reference = cfg.max_angle_ref_rad;
    if (out.Roll_angle_reference < -cfg.max_angle_ref_rad) out.Roll_angle_reference = -cfg.max_angle_ref_rad;
    if (out.Pitch_angle_reference > cfg.max_angle_ref_rad) out.Pitch_angle_reference = cfg.max_angle_ref_rad;
    if (out.Pitch_angle_reference < -cfg.max_angle_ref_rad) out.Pitch_angle_reference = -cfg.max_angle_ref_rad;

    // 姿勢角PID → 角速度目標
    const float phi_err = out.Roll_angle_reference - (sensor_state.Roll_angle - out.Roll_angle_offset);
    const float theta_err = out.Pitch_angle_reference - (sensor_state.Pitch_angle - out.Pitch_angle_offset);
    out.Roll_rate_reference = phi_pid.update(phi_err, dt);
    out.Pitch_rate_reference = theta_pid.update(theta_err, dt);
}

// 角速度制御(内側ループ)+高度速度制御+ミキサ
void rate_control(void) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& out = flight_control_state.output;
    const float dt = flight_control_state.timing.Interval_time;

    if (out.Thrust_command / cfg.battery_nominal_v < cfg.motor_on_duty_threshold) {
        reset_rate_control();
        return;
    }

    // 角速度PID
    const float p_err = out.Roll_rate_reference - sensor_state.Roll_rate;
    const float q_err = out.Pitch_rate_reference - sensor_state.Pitch_rate;
    const float r_err = out.Yaw_rate_reference - sensor_state.Yaw_rate;
    out.Roll_rate_command = p_pid.update(p_err, dt);
    out.Pitch_rate_command = q_pid.update(q_err, dt);
    out.Yaw_rate_command = r_pid.update(r_err, dt);

    // 上下速度制御 → スラスト指令
    if (flight_control_state.mode.Alt_flag == 1) {
        const float z_dot_err = out.Z_dot_ref - sensor_state.Alt_velocity;
        out.Thrust_command = thrust_filtered.update(
            (out.Thrust0 + z_dot_pid.update(z_dot_err, dt)) * cfg.battery_nominal_v, dt);
        if (out.Thrust_command / cfg.battery_nominal_v > out.Thrust0 * cfg.thrust_cmd_max_ratio) {
            out.Thrust_command = cfg.battery_nominal_v * out.Thrust0 * cfg.thrust_cmd_max_ratio;
        }
        if (out.Thrust_command / cfg.battery_nominal_v < out.Thrust0 * cfg.thrust_cmd_min_ratio) {
            out.Thrust_command = cfg.battery_nominal_v * out.Thrust0 * cfg.thrust_cmd_min_ratio;
        }
    } else if (flight_control_state.mode.auto_state == AUTO_LANDING) {
        // 着陸時は固定降下速度指令(クランプなし: 実績挙動)
        const float z_dot_err = cfg.landing_z_dot_ref - sensor_state.Alt_velocity;
        out.Thrust_command = thrust_filtered.update(
            (out.Thrust0 + z_dot_pid.update(z_dot_err, dt)) * cfg.battery_nominal_v, dt);
    }

    // ミキサ(X配置: 各軸トルクをデューティへ配分)
    out.FrontRight_motor_duty = duty_filter_fr.update(
        (out.Thrust_command + (-out.Roll_rate_command + out.Pitch_rate_command + out.Yaw_rate_command) *
                                  cfg.mixer_torque_gain) / cfg.battery_nominal_v, dt);
    out.FrontLeft_motor_duty = duty_filter_fl.update(
        (out.Thrust_command + (out.Roll_rate_command + out.Pitch_rate_command - out.Yaw_rate_command) *
                                  cfg.mixer_torque_gain) / cfg.battery_nominal_v, dt);
    out.RearRight_motor_duty = duty_filter_rr.update(
        (out.Thrust_command + (-out.Roll_rate_command - out.Pitch_rate_command - out.Yaw_rate_command) *
                                  cfg.mixer_torque_gain) / cfg.battery_nominal_v, dt);
    out.RearLeft_motor_duty = duty_filter_rl.update(
        (out.Thrust_command + (out.Roll_rate_command - out.Pitch_rate_command + out.Yaw_rate_command) *
                                  cfg.mixer_torque_gain) / cfg.battery_nominal_v, dt);

    // デューティクランプ
    if (out.FrontRight_motor_duty < cfg.duty_min) out.FrontRight_motor_duty = cfg.duty_min;
    if (out.FrontRight_motor_duty > cfg.duty_max) out.FrontRight_motor_duty = cfg.duty_max;
    if (out.FrontLeft_motor_duty < cfg.duty_min) out.FrontLeft_motor_duty = cfg.duty_min;
    if (out.FrontLeft_motor_duty > cfg.duty_max) out.FrontLeft_motor_duty = cfg.duty_max;
    if (out.RearRight_motor_duty < cfg.duty_min) out.RearRight_motor_duty = cfg.duty_min;
    if (out.RearRight_motor_duty > cfg.duty_max) out.RearRight_motor_duty = cfg.duty_max;
    if (out.RearLeft_motor_duty < cfg.duty_min) out.RearLeft_motor_duty = cfg.duty_min;
    if (out.RearLeft_motor_duty > cfg.duty_max) out.RearLeft_motor_duty = cfg.duty_max;

    // 出力(OverG時は即時モータ停止 → COMPLETE)
    if (sensor_state.OverG_flag == 0) {
        set_duty_fr(out.FrontRight_motor_duty);
        set_duty_fl(out.FrontLeft_motor_duty);
        set_duty_rr(out.RearRight_motor_duty);
        set_duty_rl(out.RearLeft_motor_duty);
    } else {
        out.FrontRight_motor_duty = 0.0f;
        out.FrontLeft_motor_duty = 0.0f;
        out.RearRight_motor_duty = 0.0f;
        out.RearLeft_motor_duty = 0.0f;
        motor_stop();
        transition_to(AUTO_COMPLETE, Reason::OVER_G);
    }
}

// 着陸シーケンス1tick分。着陸完了で1を返す。
// (OptiTrack版 auto_landing と等価: 高度履歴配列は実質「1tick前の高度」比較
//  だったため、等価な単一変数 prev_alt に整理)
uint8_t auto_landing_step(void) {
    static float prev_alt = 0.0f;
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& out = flight_control_state.output;

    flight_control_state.mode.Alt_flag = 0;
    if (landing_state == 0) {
        landing_state = 1;
        prev_alt = sensor_state.Altitude2;
        out.Thrust0 = get_trim_duty(sensor_state.Voltage);
    }

    // 降下中はスラストを徐々に絞る
    if (prev_alt >= sensor_state.Altitude2) {
        out.Thrust0 = out.Thrust0 * cfg.landing_decay_slow;
    }
    if (sensor_state.Altitude2 < cfg.landing_near_ground_m) {
        out.Thrust0 = out.Thrust0 * cfg.landing_decay_fast;
    }

    uint8_t landed = 0;
    if (sensor_state.Altitude2 < cfg.landed_threshold_m) {
        landed = 1;
        landing_state = 0;
    }
    prev_alt = sensor_state.Altitude2;

    // 姿勢は水平保持
    out.Roll_angle_command = 0.0f;
    out.Pitch_angle_command = 0.0f;
    out.Yaw_rate_reference = 0.0f;
    return landed;
}

// ---------------------------------------------------------------------------
// 状態ハンドラ
// ---------------------------------------------------------------------------

void handle_init(void) {
    auto& out = flight_control_state.output;
    motor_stop();
    out.Roll_angle_offset = 0.0f;
    out.Pitch_angle_offset = 0.0f;
    sensor_reset_offset();
    offset_counter = 0;
    transition_to(AUTO_CALIBRATION, Reason::NONE);
}

void handle_calibration(void) {
    motor_stop();
    if (offset_counter < FLIGHT_CONFIG.gyro_calib_samples) {
        sensor_calc_offset_avarage();
        offset_counter++;
        return;
    }
    // オフセット確定 → 経過時間計測の起点を設定してWAITへ
    flight_control_state.timing.S_time = micros();
    wait_ready = false;
    enter_wait(Reason::NONE);
}

void handle_wait(const CommandSnapshot& cmd) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& mode = flight_control_state.mode;
    auto& out = flight_control_state.output;

    motor_stop();

    // 静定待ち完了 → AHRSリセット(離陸準備完了)
    if (!wait_ready && (millis() - mode.phase_start_ms) > cfg.wait_settle_ms) {
        for (uint8_t i = 0; i < cfg.ahrs_reset_repeat; i++) {
            ahrs_reset();
        }
        wait_ready = true;
        comm_send_log("calibration complete: ready for START");
    }

    // コマンド処理(優先度 STOP > START。STOPはWAITでは何もしない)
    if (!cmd.stop_pending && cmd.start_pending) {
        if (!wait_ready) {
            reject_start(Reason::START_REJECTED_NOT_READY);
        } else if (low_voltage_detected()) {
            reject_start(Reason::START_REJECTED_LOW_VOLTAGE);
        } else {
            // 離陸開始
            ahrs_reset();
            mode.flight_start_ms = millis();
            auto_takeoff_counter = 0;
            landing_state = 0;
            prev_range0_flag = 0;
            transition_to(AUTO_TAKEOFF, Reason::START_CMD);
        }
    }

    // アイドル状態の各種リセット(離陸へ遷移していない場合のみ。実績挙動を踏襲)
    if (mode.auto_state == AUTO_WAIT) {
        sensor_state.OverG_flag = 0;
        sensor_state.Range0flag = 0;
        out.Thrust0 = 0.0f;
        mode.Alt_flag = 0;
        out.Roll_rate_reference = 0.0f;
        out.Pitch_rate_reference = 0.0f;
        out.Yaw_rate_reference = 0.0f;
        landing_state = 0;
        auto_takeoff_counter = 0;
        prev_range0_flag = 0;
        thrust_filtered.reset();
        reset_duty_filters();
    }
}

void handle_flight(const CommandSnapshot& cmd) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& mode = flight_control_state.mode;
    const uint32_t now_ms = millis();

    // STOP(最優先): 即時着陸
    bool stop_requested = false;
    if (cmd.stop_pending) {
        transition_to(AUTO_LANDING, Reason::STOP_CMD);
        stop_requested = true;
    }

    // 離陸完了判定: 目標高度に到達したらHOVERへ
    if (!stop_requested && mode.auto_state == AUTO_TAKEOFF &&
        sensor_state.Altitude2 >= flight_control_state.output.Alt_ref - cfg.takeoff_complete_margin_m) {
        transition_to(AUTO_HOVER, Reason::NONE);
    }

    // フェイルセーフ(PROTOCOL.md の表)
    if (mode.auto_state != AUTO_LANDING && low_voltage_detected()) {
        transition_to(AUTO_LANDING, Reason::LOW_VOLTAGE);
    }
    if (mode.auto_state != AUTO_LANDING && (now_ms - mode.flight_start_ms) > cfg.max_flight_time_ms) {
        transition_to(AUTO_LANDING, Reason::MAX_FLIGHT_TIME);
    }
    if (mode.auto_state != AUTO_LANDING && setpoint_link_age_ms(now_ms) > cfg.link_loss_landing_ms) {
        transition_to(AUTO_LANDING, Reason::LINK_LOSS);
    }
    if (sensor_state.OverG_flag == 1) {
        transition_to(AUTO_COMPLETE, Reason::OVER_G);
    }

    // 指令更新 → 制御(遷移直後のtickも実績どおり制御を1回実行する。
    // OverG時は rate_control 内で即時モータ停止)
    update_thrust_and_attitude_command();
    angle_control();
    rate_control();
}

void handle_landing(const CommandSnapshot& cmd) {
    (void)cmd;  // STOPは受理するが、すでに着陸中のため挙動は変わらない

    if (auto_landing_step() == 1) {
        enter_wait(Reason::LANDED);
    }
    angle_control();
    rate_control();
}

void handle_complete(const CommandSnapshot& cmd) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& mode = flight_control_state.mode;
    auto& out = flight_control_state.output;

    motor_stop();
    sensor_state.OverG_flag = 0;
    sensor_state.Range0flag = 0;
    out.Thrust0 = 0.0f;
    mode.Alt_flag = 0;
    out.Roll_rate_reference = 0.0f;
    out.Pitch_rate_reference = 0.0f;
    out.Yaw_rate_reference = 0.0f;
    landing_state = 0;
    auto_takeoff_counter = 0;
    thrust_filtered.reset();
    reset_duty_filters();
    // 高度推定器はリセットしない: CMD_RESET の altitude_est<0.15m ガードを有効に保つ

    // CMD_RESET: COMPLETE かつ altitude_est < 0.15m でのみ受理(STOP/STARTは無視)
    if (cmd.reset_pending && sensor_state.Altitude2 < cfg.reset_accept_alt_m) {
        enter_wait(Reason::RESET_CMD);
    }
}

// ループ実測タイミングの更新(loop_dt_us テレメトリの根拠)
void update_loop_timing(void) {
    auto& t = flight_control_state.timing;
    t.E_time = micros();
    t.Old_Elapsed_time = t.Elapsed_time;
    t.Elapsed_time = 1.0e-6f * (t.E_time - t.S_time);
    t.Interval_time = t.Elapsed_time - t.Old_Elapsed_time;
}

}  // namespace

// ---------------------------------------------------------------------------
// 公開API
// ---------------------------------------------------------------------------

void init_copter(void) {
    USBSerial.begin(115200);
    delay(FLIGHT_CONFIG.boot_serial_wait_ms);
    USBSerial.printf("StampFly Integrated Control: boot\r\n");

    flight_control_state.timing.Control_period = FLIGHT_CONFIG.control_period_s;
    flight_control_state.output.Alt_ref = FLIGHT_CONFIG.default_alt_ref_m;

    init_pwm();
    sensor_init();
    control_init();
    indicators_init();
    comm_init();

    // 400Hz割り込み(1µsティック、2500µs周期)
    loop_timer = timerBegin(0, FLIGHT_CONFIG.timer_prescaler, true);
    timerAttachInterrupt(loop_timer, &on_loop_timer, true);
    timerAlarmWrite(loop_timer, FLIGHT_CONFIG.loop_period_us, true);
    timerAlarmEnable(loop_timer);

    USBSerial.printf("StampFly Integrated Control: init done\r\n");
    // 以後、定常状態ではUSBシリアルへ出力しない(人間向けメッセージはLOG_TEXT)
}

void loop_400Hz(void) {
    // タイマ割り込みによる400Hzペーシング(製品版と同一方式)
    while (flight_control_state.timing.Loop_flag == 0) {
    }
    flight_control_state.timing.Loop_flag = 0;

    update_loop_timing();
    sensor_read();

    // 受信コマンドの取り出しと setpoint 適用
    CommandSnapshot cmd;
    comm_consume_commands(&cmd);
    apply_setpoint(cmd);

    switch (flight_control_state.mode.auto_state) {
        case AUTO_INIT:
            handle_init();
            break;
        case AUTO_CALIBRATION:
            handle_calibration();
            break;
        case AUTO_WAIT:
            handle_wait(cmd);
            break;
        case AUTO_TAKEOFF:
        case AUTO_HOVER:
            handle_flight(cmd);
            break;
        case AUTO_LANDING:
            handle_landing(cmd);
            break;
        case AUTO_COMPLETE:
            handle_complete(cmd);
            break;
    }

    telemetry_update();
    indicators_update();
}
