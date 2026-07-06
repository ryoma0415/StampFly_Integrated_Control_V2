// ===========================================================================
// flight_control.cpp — 状態機械 + カスケードPID + ミキサ 実装
//
// 制御則(角度→角速度→ミキサ、高度→上下速度→スラスト)と離着陸シーケンスは
// 飛行実績のある OptiTrack版を踏襲。コマンド入出力は PROTOCOL.md v1 に従い
// comm/telemetry モジュール経由で行う。定常状態でUSBシリアルへは出力しない。
// ===========================================================================
#include "flight_control.hpp"

#include <Arduino.h>

#include <cstring>

#include "comm.hpp"
#include "indicators.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "telemetry.hpp"
#include "yaw_estimation/angle_utils.hpp"
#include "yaw_estimation/persistence.hpp"
#include "yaw_estimation/sensor_hub_ff.hpp"

FlightControlState flight_control_state;

namespace {

using stampfly::Reason;
using stampfly::TlmAck;

// --- PID制御器・フィルタ(OptiTrack版と同構成) ---
PID p_pid;      // ロール角速度
PID q_pid;      // ピッチ角速度
PID r_pid;      // ヨー角速度
PID phi_pid;    // ロール角
PID theta_pid;  // ピッチ角
PID psi_pid;    // ヨー角(v2: 角度誤差 wrapPi → ヨーレート目標)
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

// --- v2: ヨー角制御の途絶ラッチ(契約 §1.1: 途絶>200msで現在推定ヨーを保持) ---
bool yaw_latch_active = false;     // ラッチ保持中か
float yaw_latched_ref = 0.0f;      // ラッチしたヨー角目標 [rad]

// --- v2: モーターテストサービス内部状態(yaw側 motor.cpp のソフトスタート/
//     フェイルセーフを移植。PWM の書き手は MOTOR_TEST 状態の本サービスのみ) ---
float mt_target_duty = 0.0f;    // 指令 duty(ソフトスタートランプの目標)
float mt_applied_duty = 0.0f;   // ランプ後の実効 duty(PWM に出ている値)
uint8_t mt_mask = 0;            // 駆動対象(bit0=FL,1=FR,2=RL,3=RR)
bool mt_running = false;        // 指令上の駆動中(フェイルセーフ監視対象)
uint32_t mt_last_cmd_ms = 0;    // 直近 CMD_MOTOR_RUN/STOP 受理時刻
uint32_t mt_last_ramp_ms = 0;   // ランプ dt の基準時刻

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
// モーターテストサービス(v2 契約 §2.4)
//
// yaw側 motor.cpp のソフトスタート(2.0duty/s)・キープアライブ途絶停止(1.5s)を
// 移植。PWM 出力は既存の set_duty_*(LEDC)経路を使い、書き手は MOTOR_TEST
// 状態の本サービスのみ(飛行ミキサとは状態機械で排他)。ソフトスタートは
// 1S バッテリの突入電流ブラウンアウト対策なので削除しないこと。
// ---------------------------------------------------------------------------

// 実効 duty×マスクを PWM へ出力し、TLM_STATE の duty 表示にもミラーする
void motor_test_write_outputs(void) {
    auto& out = flight_control_state.output;
    const float d = mt_applied_duty;
    const float fl = (mt_mask & stampfly::CmdMotorRun::MASK_FL) ? d : 0.0f;
    const float fr = (mt_mask & stampfly::CmdMotorRun::MASK_FR) ? d : 0.0f;
    const float rl = (mt_mask & stampfly::CmdMotorRun::MASK_RL) ? d : 0.0f;
    const float rr = (mt_mask & stampfly::CmdMotorRun::MASK_RR) ? d : 0.0f;
    set_duty_fl(fl);
    set_duty_fr(fr);
    set_duty_rl(rl);
    set_duty_rr(rr);
    out.FrontLeft_motor_duty = fl;
    out.FrontRight_motor_duty = fr;
    out.RearLeft_motor_duty = rl;
    out.RearRight_motor_duty = rr;
}

// ソフトスタートランプを1step進める(millis 差分ベースでループ周波数非依存)
void motor_test_ramp(void) {
    const uint32_t now_ms = millis();
    const float dt = (now_ms - mt_last_ramp_ms) * 1.0e-3f;
    mt_last_ramp_ms = now_ms;
    if (mt_applied_duty != mt_target_duty) {
        const float step = FLIGHT_CONFIG.motor_test_slew_duty_per_s * dt;
        if (mt_applied_duty < mt_target_duty) {
            mt_applied_duty += step;
            if (mt_applied_duty > mt_target_duty) mt_applied_duty = mt_target_duty;
        } else {
            mt_applied_duty -= step;
            if (mt_applied_duty < mt_target_duty) mt_applied_duty = mt_target_duty;
        }
    }
    motor_test_write_outputs();
}

// 即時無条件停止(yaw側 motorTestStop 踏襲: 停止は負荷が抜ける方向なので
// ランプダウン不要)。MOTOR_TEST 進入時の初期化にも使う。
void motor_test_stop_now(void) {
    mt_target_duty = 0.0f;
    mt_applied_duty = 0.0f;
    mt_mask = 0;
    mt_running = false;
    mt_last_cmd_ms = millis();
    mt_last_ramp_ms = mt_last_cmd_ms;
    motor_test_write_outputs();  // 全ch 0 を強制
}

// CMD_MOTOR_RUN の適用(duty はクランプ、キープアライブタイマ更新)
void motor_test_set(float duty, uint8_t mask) {
    if (!isfinite(duty) || duty < 0.0f) duty = 0.0f;
    if (duty > FLIGHT_CONFIG.motor_test_max_duty) duty = FLIGHT_CONFIG.motor_test_max_duty;
    mask &= (stampfly::CmdMotorRun::MASK_FL | stampfly::CmdMotorRun::MASK_FR |
             stampfly::CmdMotorRun::MASK_RL | stampfly::CmdMotorRun::MASK_RR);
    mt_target_duty = duty;
    mt_mask = mask;
    mt_running = (duty > 0.0f) && (mask != 0);
    mt_last_cmd_ms = millis();
    motor_test_ramp();
}

// 毎tickサービス: キープアライブ途絶の自動停止+ランプ前進
void motor_test_service(void) {
    if (mt_running &&
        (millis() - mt_last_cmd_ms) > FLIGHT_CONFIG.motor_test_failsafe_ms) {
        motor_test_stop_now();
        return;
    }
    motor_test_ramp();
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
    psi_pid.set_parameter(cfg.yaw_angle.kp, cfg.yaw_angle.ti, cfg.yaw_angle.td, cfg.yaw_angle.eta, h);

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

// ヨー角制御のリセット(psi_pid・途絶ラッチ・適用中目標)。
// 既存の角度制御リセット群と同じ箇所(reset_rate/angle_control)から呼ぶ。
void reset_yaw_angle_control(void) {
    auto& out = flight_control_state.output;
    psi_pid.reset();
    yaw_latch_active = false;
    out.Yaw_angle_command = 0.0f;
    out.Yaw_ctrl_active = 0;
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
    // v2: ヨー角目標(flags bit1 有効時のみ制御に使う。無効時は V1 同一動作)
    cs.target_yaw_rad = cmd.setpoint.yaw_ref;
    cs.target_yaw_valid = (cmd.setpoint.flags & stampfly::CmdSetpoint::FLAG_YAW_REF_VALID) != 0;

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
    const bool link_fresh =
        cs.setpoint_received && (now_ms - cs.last_setpoint_ms) < cfg.link_level_hold_ms;
    if (link_fresh) {
        out.Roll_angle_command = cs.target_roll_rad;
        out.Pitch_angle_command = cs.target_pitch_rad;
    } else {
        out.Roll_angle_command = 0.0f;
        out.Pitch_angle_command = 0.0f;
    }

    // --- ヨー角制御(v2 契約 §2.3。flags bit1 無効時は V1 と同一のレートダンピング) ---
    // yaw_used: est_mode==1 かつ EKF 健全なら EKF ψ、est_mode==0 なら Madgwick。
    // est_mode==1 で EKF が不健全(磁気更新凍結/アンカー無効/FF無効)な間は、
    // 飛行中のヨーソース切替による指令段差を作らないため角度制御を止めて
    // レートダンピングに縮退する(PC へは ffg / ff_status bit5 で通知される)。
    bool yaw_source_ok = true;
    float yaw_used = sensor_state.Yaw_angle;
    if (sensor_state.Est_mode == 1) {
        if (sensorHubFfEkfHealthy()) {
            yaw_used = sensor_state.Yaw_ekf_rad;
        } else {
            yaw_source_ok = false;
        }
    }

    bool yaw_cmd_valid = false;
    float yaw_cmd = 0.0f;
    if (link_fresh && cs.target_yaw_valid) {
        yaw_cmd = wrapPi(cs.target_yaw_rad);
        yaw_latch_active = false;
        yaw_cmd_valid = true;
    } else if (!link_fresh && cs.setpoint_received && cs.target_yaw_valid) {
        // 途絶(>200ms): 途絶検出時点の推定ヨーをラッチして保持(契約 §1.1)。
        // 0 指令へ落とすと「離陸方位への回頭」を意味するため角度保持が安全。
        // >500ms は既存フェイルセーフで LANDING(auto_landing_step はヨー0)。
        if (!yaw_latch_active) {
            yaw_latch_active = true;
            yaw_latched_ref = yaw_used;
        }
        yaw_cmd = yaw_latched_ref;
        yaw_cmd_valid = true;
    } else {
        yaw_latch_active = false;
    }

    if (yaw_cmd_valid && yaw_source_ok) {
        // 角度誤差は必ず ±π ラップしてから PID へ(最短経路で回頭する)
        const float dt = flight_control_state.timing.Interval_time;
        float yaw_rate_ref = psi_pid.update(wrapPi(yaw_cmd - yaw_used), dt);
        if (yaw_rate_ref > cfg.yaw_rate_limit_rad_s) yaw_rate_ref = cfg.yaw_rate_limit_rad_s;
        if (yaw_rate_ref < -cfg.yaw_rate_limit_rad_s) yaw_rate_ref = -cfg.yaw_rate_limit_rad_s;
        out.Yaw_rate_reference = yaw_rate_ref;
        out.Yaw_angle_command = yaw_cmd;
        out.Yaw_ctrl_active = 1;
    } else {
        // V1 同一動作: レートダンピングのみ(角度制御 off)
        out.Yaw_rate_reference = 0.0f;
        out.Yaw_angle_command = 0.0f;
        out.Yaw_ctrl_active = 0;
        psi_pid.reset();  // 不使用中に積分を持ち越さない(再開時の段差防止)
    }
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
    reset_yaw_angle_control();
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
    reset_yaw_angle_control();
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

    // 姿勢は水平保持。ヨーは着陸中つねにレートダンピング(=0)を維持し、
    // 角度制御の適用中フラグ/ラッチも落とす(契約 §2.3: auto_landing_step は 0 のまま)
    out.Roll_angle_command = 0.0f;
    out.Pitch_angle_command = 0.0f;
    out.Yaw_rate_reference = 0.0f;
    reset_yaw_angle_control();
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
        reset_yaw_angle_control();
        landing_state = 0;
        auto_takeoff_counter = 0;
        prev_range0_flag = 0;
        thrust_filtered.reset();
        reset_duty_filters();
    }
}

// MOTOR_TEST 状態(v2 契約 §2.4): 飛行制御 PID・ミキサは動かさず、PWM の
// 書き手はモーターテストサービスのみ。CMD_STOP を最優先で脱出する。
void handle_motor_test(const CommandSnapshot& cmd) {
    // STOP(最優先): 即時モーター停止 → WAIT へ
    if (cmd.stop_pending) {
        motor_test_stop_now();
        enter_wait(Reason::STOP_CMD);
        return;
    }
    // MOTOR_TEST 中の CMD_START は拒否(reason=10)
    if (!cmd.stop_pending && cmd.start_pending) {
        reject_start(Reason::START_REJECTED_NOT_READY);
    }
    // キープアライブ途絶(1.5s)の自動停止+ソフトスタートランプ
    motor_test_service();
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
    reset_yaw_angle_control();
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

// ---------------------------------------------------------------------------
// v2 コマンド処理(0x14-0x23。契約 §1.2 / §2.4 / §2.5)
//
// 全コマンドに TLM_ACK で応答する。キャリブ/FF系(0x17-0x23)は
// WAIT / COMPLETE / MOTOR_TEST でのみ受理(飛行中の NVS 書込み禁止)。
// 処理内容は yaw側 command.cpp の各ハンドラをバイナリプロトコルへ写像したもの。
// ---------------------------------------------------------------------------

// キャリブ/FF系コマンドを受理できる状態か(非飛行=NVS書込み可)
bool in_cal_command_state(void) {
    const AutoFlightState st = flight_control_state.mode.auto_state;
    return st == AUTO_WAIT || st == AUTO_COMPLETE || st == AUTO_MOTOR_TEST;
}

// yaw側 resetYawEstimatorKeepingZero の移植: キャリブ変更後にリファレンスCFを
// 現在ヨーで再シードする。reset() は捕捉済み磁気オフセットを消して次の有効
// 磁気サンプルで再捕捉するため、ヨーゼロ有効時は遅延NVS保存を予約する
// (捕捉後に service_pending_yaw_zero_save が永続化する)。
void reset_yaw_estimator_keeping_zero(void) {
    g_yaw_est.yaw_estimator.reset(g_yaw_est.yaw_estimator.yaw());
    if (g_yaw_est.yaw_estimator.yawZeroValid()) {
        g_yaw_est.yaw_zero_save_pending = true;
    }
}

// yaw側 clearAttitudeMountZero の移植
void clear_attitude_mount_zero(void) {
    g_yaw_est.roll_mount_offset_rad = 0.0f;
    g_yaw_est.pitch_mount_offset_rad = 0.0f;
    g_yaw_est.attitude_mount_valid = false;
    saveAttitudeMountZero(false, 0.0f, 0.0f);
}

// yaw_zero の遅延NVS保存(yaw側 commandService の移植): 磁気オフセットが
// 実際に捕捉されてから、非飛行状態でのみ1回だけ書き込む。
void service_pending_yaw_zero_save(void) {
    if (!g_yaw_est.yaw_zero_save_pending) return;
    if (!in_cal_command_state()) return;
    if (!g_yaw_est.yaw_estimator.yawZeroOffsetValid()) return;
    saveYawZero(true, g_yaw_est.yaw_estimator.magYawOffsetRad());
    g_yaw_est.yaw_zero_save_pending = false;
}

// CMD_MODE: WAIT --mode=1--> MOTOR_TEST / MOTOR_TEST --mode=0--> WAIT。
// 他状態では bad_state(契約 §1.2)。
uint8_t handle_cmd_mode(const stampfly::CmdMode& m) {
    if (m.mode > stampfly::CmdMode::MODE_MOTOR_TEST) return TlmAck::STATUS_INVALID_ARG;
    const AutoFlightState st = flight_control_state.mode.auto_state;
    if (m.mode == stampfly::CmdMode::MODE_MOTOR_TEST) {
        if (st == AUTO_MOTOR_TEST) return TlmAck::STATUS_OK;  // 冪等(ACKロスト再送)
        if (st != AUTO_WAIT) return TlmAck::STATUS_BAD_STATE;
        motor_test_stop_now();  // duty/mask/キープアライブタイマを初期化
        transition_to(AUTO_MOTOR_TEST, Reason::MODE_CHANGE);
        return TlmAck::STATUS_OK;
    }
    // mode == MODE_FLIGHT
    if (st == AUTO_WAIT) return TlmAck::STATUS_OK;  // 冪等(ACKロスト再送)
    if (st != AUTO_MOTOR_TEST) return TlmAck::STATUS_BAD_STATE;
    motor_test_stop_now();  // モーター停止後に WAIT へ(契約 §2.4)
    enter_wait(Reason::MODE_CHANGE);
    return TlmAck::STATUS_OK;
}

// CMD_MAG3D_SET: 適用/クリア+NVS 永続化+FF自動無効+アンカー破棄+再シード
uint8_t handle_cmd_mag3d_set(const stampfly::CmdMag3dSet& m) {
    if (m.valid != 0) {
        const MagVector offset{m.offset[0], m.offset[1], m.offset[2]};
        if (!g_yaw_est.mag3d_calibration.set(offset, m.matrix)) {
            return TlmAck::STATUS_INVALID_ARG;
        }
    } else {
        g_yaw_est.mag3d_calibration.reset();
    }
    g_yaw_est.mag_filter.reset();
    saveMag3DCalibration(g_yaw_est.mag3d_calibration);
    reset_yaw_estimator_keeping_zero();
    // b_cal 空間が変わる: アンカー破棄+補正系再シード+ff_mode=0 強制(NVS込み)
    sensorHubFfOnMag3dChange();
    return TlmAck::STATUS_OK;
}

// CMD_ACCEL6_SET: 係数の NVS 復元系(yaw側 accel6_set/accel6_clear 相当)。
// V2 では飛行AHRS(Madgwick)へは未適用(飛行挙動不変の分離設計)だが、
// yaw側仕様どおり姿勢参照(マウントゼロ・AHRS・リファレンスCF)をリセットする。
uint8_t handle_cmd_accel6_set(const stampfly::CmdAccel6Set& m) {
    if (m.valid != 0) {
        const AccelVector offset{m.offset[0], m.offset[1], m.offset[2]};
        const AccelVector scale{m.scale[0], m.scale[1], m.scale[2]};
        if (!g_yaw_est.accel_calibration.setCalibration(offset, scale)) {
            return TlmAck::STATUS_INVALID_ARG;
        }
    } else {
        g_yaw_est.accel_calibration.reset();
    }
    saveAccelCalibration(g_yaw_est.accel_calibration);
    // yaw側 resetAttitudeReferencesAfterAccelChange の移植
    clear_attitude_mount_zero();
    ahrs_reset();  // 非飛行状態でのみ受理されるため安全(Yaw_gyro_integral も0へ)
    reset_yaw_estimator_keeping_zero();
    return TlmAck::STATUS_OK;
}

// CMD_ATTMOUNT_SET: マウントオフセット設定/クリア(磁気レベル化入力にのみ適用)
uint8_t handle_cmd_attmount_set(const stampfly::CmdAttmountSet& m) {
    if (m.valid != 0) {
        // rad かつ ±π 内のみ受理(yaw側 applyAttitudeMountSetCommand と同じ検査)
        if (!isfinite(m.roll_rad) || !isfinite(m.pitch_rad) ||
            fabsf(m.roll_rad) > PI || fabsf(m.pitch_rad) > PI) {
            return TlmAck::STATUS_INVALID_ARG;
        }
        g_yaw_est.roll_mount_offset_rad = m.roll_rad;
        g_yaw_est.pitch_mount_offset_rad = m.pitch_rad;
        g_yaw_est.attitude_mount_valid = true;
        saveAttitudeMountZero(true, m.roll_rad, m.pitch_rad);
    } else {
        clear_attitude_mount_zero();
    }
    reset_yaw_estimator_keeping_zero();
    return TlmAck::STATUS_OK;
}

// CMD_YAWZERO_SET: 保存済みヨーゼロ(磁気オフセット)の復元/クリア
uint8_t handle_cmd_yawzero_set(const stampfly::CmdYawzeroSet& m) {
    if (m.valid != 0) {
        if (!isfinite(m.offset_rad) || fabsf(m.offset_rad) > PI) {
            return TlmAck::STATUS_INVALID_ARG;
        }
        g_yaw_est.yaw_estimator.restoreYawZero(m.offset_rad);
        // 推定器が実際に保持したラップ済み値を永続化(NVSとRAMの乖離防止)
        saveYawZero(true, g_yaw_est.yaw_estimator.magYawOffsetRad());
        g_yaw_est.yaw_zero_save_pending = false;
    } else {
        g_yaw_est.yaw_estimator.clearYawZero();
        g_yaw_est.yaw_zero_save_pending = false;
        saveYawZero(false, 0.0f);
    }
    return TlmAck::STATUS_OK;
}

// CMD_GEOMAG_SET: 地磁気リファレンス設定(deg→rad 換算、許容値は既定値)
uint8_t handle_cmd_geomag_set(const stampfly::CmdGeomagSet& m) {
    GeomagneticReference reference;  // 許容値・z符号は yaw_config.hpp の既定値
    reference.valid = true;
    reference.declination_east_rad = m.declination_east_deg * DEG_TO_RAD;
    reference.inclination_rad = m.inclination_deg * DEG_TO_RAD;
    reference.horizontal_uT = m.horizontal_ut;
    reference.vertical_uT = m.vertical_ut;
    reference.total_uT = m.total_ut;
    if (!g_yaw_est.yaw_estimator.setGeomagneticReference(reference)) {
        return TlmAck::STATUS_INVALID_ARG;
    }
    saveGeomagneticReference(g_yaw_est.yaw_estimator.geomagneticReference());
    return TlmAck::STATUS_OK;
}

// CMD_FF_COMMIT: CRC 照合→NVS 永続化→補正系再シード。冪等(yaw側 C5)。
// commit の失敗理由(静的文字列)を TLM_ACK status へ分類する。
uint8_t handle_cmd_ff_commit(const stampfly::CmdFfCommit& m) {
    const char* message = "";
    if (!g_yaw_est.ff_calibration.commit(m.crc32, message)) {
        if (std::strcmp(message, "crc mismatch") == 0) return TlmAck::STATUS_CRC_MISMATCH;
        if (std::strstr(message, "missing") != nullptr ||
            std::strstr(message, "no staging") != nullptr) {
            return TlmAck::STATUS_INCOMPLETE;
        }
        return TlmAck::STATUS_INVALID_ARG;  // LUT非昇順・非有限値など
    }
    saveFfCalibration(g_yaw_est.ff_calibration, g_yaw_est.ff.ff_mode, g_yaw_est.ff.est_mode);
    sensorHubFfReseed();  // 新係数で補正パスが変わるため再シード
    return TlmAck::STATUS_OK;
}

// CMD_FF_MODE: ff_mode / est_mode の実行時切替+NVS 永続化+再シード
uint8_t handle_cmd_ff_mode(const stampfly::CmdFfMode& m) {
    if (m.ff_mode > stampfly::CmdFfMode::FF_MODE_B ||
        m.est_mode > stampfly::CmdFfMode::EST_MODE_EKF) {
        return TlmAck::STATUS_INVALID_ARG;
    }
    g_yaw_est.ff.ff_mode = m.ff_mode;
    g_yaw_est.ff.est_mode = m.est_mode;
    saveFfModes(m.ff_mode, m.est_mode);
    sensorHubFfReseed();
    return TlmAck::STATUS_OK;
}

// 1件の v2 コマンドを処理して TLM_ACK の status を返す。
// send_cal_data_after: ACK 送信後に TLM_CAL_DATA を送るべきか(CMD_CAL_GET)。
uint8_t process_one_v2_command(const V2Command& c, bool& send_cal_data_after) {
    using stampfly::MsgType;
    const MsgType type = static_cast<MsgType>(c.type);

    // キャリブ/FF系(0x17-0x23)は WAIT/COMPLETE/MOTOR_TEST のみ受理(契約 §1.2)
    if (c.type >= static_cast<uint8_t>(MsgType::CMD_CAL_GET) &&
        c.type <= static_cast<uint8_t>(MsgType::CMD_FF_ANCHOR) &&
        !in_cal_command_state()) {
        return TlmAck::STATUS_BAD_STATE;
    }

    switch (type) {
        case MsgType::CMD_MODE: {
            stampfly::CmdMode m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_mode(m);
        }
        case MsgType::CMD_MOTOR_RUN: {
            // MOTOR_TEST 状態のみ。飛行状態では PWM に触れず bad_state で拒否
            if (flight_control_state.mode.auto_state != AUTO_MOTOR_TEST) {
                return TlmAck::STATUS_BAD_STATE;
            }
            stampfly::CmdMotorRun m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (!isfinite(m.duty) || m.duty < 0.0f || m.duty > 1.0f) {
                return TlmAck::STATUS_INVALID_ARG;
            }
            motor_test_set(m.duty, m.mask);
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_MOTOR_STOP: {
            if (flight_control_state.mode.auto_state != AUTO_MOTOR_TEST) {
                return TlmAck::STATUS_BAD_STATE;
            }
            motor_test_stop_now();
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_CAL_GET:
            send_cal_data_after = true;  // ACK(ok) の後に TLM_CAL_DATA を返す
            return TlmAck::STATUS_OK;
        case MsgType::CMD_MAG3D_SET: {
            stampfly::CmdMag3dSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_mag3d_set(m);
        }
        case MsgType::CMD_ACCEL6_SET: {
            stampfly::CmdAccel6Set m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_accel6_set(m);
        }
        case MsgType::CMD_ATTMOUNT_SET: {
            stampfly::CmdAttmountSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_attmount_set(m);
        }
        case MsgType::CMD_YAWZERO_SET: {
            stampfly::CmdYawzeroSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_yawzero_set(m);
        }
        case MsgType::CMD_GEOMAG_SET: {
            stampfly::CmdGeomagSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_geomag_set(m);
        }
        case MsgType::CMD_FF_BEGIN: {
            stampfly::CmdFfBegin m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (m.nlut < stampfly::CmdFfBegin::NLUT_MIN ||
                m.nlut > stampfly::CmdFfBegin::NLUT_MAX ||
                !g_yaw_est.ff_calibration.stageBegin(m.nlut)) {
                return TlmAck::STATUS_INVALID_ARG;
            }
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_FF_LUT: {
            stampfly::CmdFfLut m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (!g_yaw_est.ff_calibration.stageLutPoint(m.idx, m.i_a, m.db_x, m.db_y, m.db_z)) {
                return TlmAck::STATUS_INVALID_ARG;  // begin 未実行 / idx 範囲外 / 非有限値
            }
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_FF_MOT: {
            stampfly::CmdFfMot m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (!g_yaw_est.ff_calibration.stageMotor(
                    m.idx, m.a_tilde[0], m.a_tilde[1], m.a_tilde[2], m.c2, m.c1, m.c0)) {
                return TlmAck::STATUS_INVALID_ARG;
            }
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_FF_AUX: {
            stampfly::CmdFfAux m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (!g_yaw_est.ff_calibration.stageAux(m.iid_a)) return TlmAck::STATUS_INVALID_ARG;
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_FF_COMMIT: {
            stampfly::CmdFfCommit m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_ff_commit(m);
        }
        case MsgType::CMD_FF_MODE: {
            stampfly::CmdFfMode m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_ff_mode(m);
        }
        case MsgType::CMD_FF_ANCHOR:
            // モーター回転中(実出力あり)/停止窓が未充填なら busy で拒否可(契約 §1.2)
            return sensorHubFfAnchorNow() ? TlmAck::STATUS_OK : TlmAck::STATUS_BUSY;
        default:
            // comm 層のレンジ検査により到達しない(防御的に invalid_arg)
            return TlmAck::STATUS_INVALID_ARG;
    }
}

// 取り出した v2 コマンド群を受信順に処理し、全件へ TLM_ACK を返す
void process_v2_commands(const CommandSnapshot& cmd) {
    for (size_t i = 0; i < cmd.v2_count; i++) {
        const V2Command& c = cmd.v2[i];
        bool send_cal_data_after = false;
        const uint8_t status = process_one_v2_command(c, send_cal_data_after);
        telemetry_send_ack(c.type, c.seq, status);
        if (send_cal_data_after && status == TlmAck::STATUS_OK) {
            telemetry_send_cal_data();
        }
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

    // v2 コマンド(0x14-0x23)の処理+TLM_ACK 応答。CMD_MODE の遷移は
    // この直後の状態スイッチで同tick内に反映される(STOP は各ハンドラが最優先)。
    process_v2_commands(cmd);

    // yaw_zero の遅延NVS保存(磁気オフセット捕捉後、非飛行状態のみ)
    service_pending_yaw_zero_save();

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
        case AUTO_MOTOR_TEST:
            handle_motor_test(cmd);
            break;
    }

    telemetry_update();
    indicators_update();
}

// --- v2: モーターテスト実出力のアクセサ(sensor.cpp の FF duty 配線・telemetry 用) ---

float motor_test_applied_duty(void) {
    return mt_applied_duty;
}

uint8_t motor_test_active_mask(void) {
    return mt_mask;
}

bool motor_test_output_active(void) {
    // ランプダウン中の実出力>0 も「回転中」として扱う(アンカー窓の汚染防止)
    return mt_applied_duty > 0.0f || mt_running;
}
