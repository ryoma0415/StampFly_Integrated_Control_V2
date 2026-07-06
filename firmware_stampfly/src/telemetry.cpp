// ===========================================================================
// telemetry.cpp — TLM_STATE / TLM_EVENT 生成 実装
// ===========================================================================
#include "telemetry.hpp"

#include <Arduino.h>

#include "comm.hpp"
#include "config.hpp"
#include "flight_control.hpp"
#include "sensor.hpp"
#include "stampfly_protocol.hpp"

namespace {

uint16_t state_divider_counter = 0;  // TLM_STATE 分周カウンタ
uint32_t last_event_tx_ms = 0;       // 直近のTLM_EVENT送信時刻
bool event_valid = false;            // 一度でも遷移イベントが発生したか
uint8_t event_state = 0;             // 直近イベントの内容(2Hz再送に使う)
uint8_t event_prev_state = 0;
uint8_t event_reason = 0;

// TLM_STATE/TLM_EVENT 共通の flags(PROTOCOL.md: bit0 low_voltage,
// bit1 setpoint_fresh(<200ms), bit2 flying)
uint8_t build_status_flags(void) {
    uint8_t flags = 0;
    if (sensor_state.Under_voltage_flag >= UNDER_VOLTAGE_COUNT) {
        flags |= stampfly::TlmState::FLAG_LOW_VOLTAGE;
    }
    const auto& cs = flight_control_state.command;
    if (cs.setpoint_received &&
        (millis() - cs.last_setpoint_ms) < FLIGHT_CONFIG.link_level_hold_ms) {
        flags |= stampfly::TlmState::FLAG_SETPOINT_FRESH;
    }
    const AutoFlightState st = flight_control_state.mode.auto_state;
    if (st == AUTO_TAKEOFF || st == AUTO_HOVER || st == AUTO_LANDING) {
        flags |= stampfly::TlmState::FLAG_FLYING;
    }
    return flags;
}

void send_event_frame(void) {
    stampfly::TlmEvent ev;
    ev.state = event_state;
    ev.prev_state = event_prev_state;
    ev.reason = event_reason;
    ev.flags = build_status_flags();
    ev.voltage = sensor_state.Voltage;

    uint8_t payload[stampfly::TlmEvent::PAYLOAD_SIZE];
    if (!stampfly::serialize(ev, payload, sizeof(payload))) return;
    comm_send_payload(stampfly::MsgType::TLM_EVENT, payload, sizeof(payload));
    last_event_tx_ms = millis();
}

void send_state_frame(void) {
    const auto& fc = flight_control_state;
    stampfly::TlmState m;

    m.seq_echo = fc.command.applied_setpoint_seq;  // 未受信なら0
    m.elapsed_ms = millis();                       // 起動からの経過
    m.state = static_cast<uint8_t>(fc.mode.auto_state);
    m.flags = build_status_flags();
    m.reason = fc.mode.last_reason;

    // 実測姿勢・角速度(AHRS / ジャイロ)
    m.roll = sensor_state.Roll_angle;
    m.pitch = sensor_state.Pitch_angle;
    m.yaw = sensor_state.Yaw_angle;
    m.p = sensor_state.Roll_rate;
    m.q = sensor_state.Pitch_rate;
    m.r = sensor_state.Yaw_rate;

    // 適用中の指令
    m.roll_ref = fc.output.Roll_angle_command;
    m.pitch_ref = fc.output.Pitch_angle_command;
    m.alt_ref = fc.output.Alt_ref;

    // 高度系(ToF由来生値 / カルマン推定)
    m.altitude_tof = sensor_state.Altitude;
    m.altitude_est = sensor_state.Altitude2;
    m.alt_velocity = sensor_state.Alt_velocity;
    m.z_dot_ref = fc.output.Z_dot_ref;

    m.voltage = sensor_state.Voltage;

    m.duty_fr = fc.output.FrontRight_motor_duty;
    m.duty_fl = fc.output.FrontLeft_motor_duty;
    m.duty_rr = fc.output.RearRight_motor_duty;
    m.duty_rl = fc.output.RearLeft_motor_duty;

    // フィルタ後加速度 [g]
    m.ax = sensor_state.Accel_x;
    m.ay = sensor_state.Accel_y;
    m.az = sensor_state.Accel_z;

    // 直近の実測制御周期 [µs](u16へ飽和)
    float dt_us = fc.timing.Interval_time * 1.0e6f;
    if (dt_us < 0.0f) dt_us = 0.0f;
    if (dt_us > 65535.0f) dt_us = 65535.0f;
    m.loop_dt_us = static_cast<uint16_t>(dt_us);

    uint8_t payload[stampfly::TlmState::PAYLOAD_SIZE];
    if (!stampfly::serialize(m, payload, sizeof(payload))) return;
    comm_send_payload(stampfly::MsgType::TLM_STATE, payload, sizeof(payload));
}

}  // namespace

void telemetry_notify_transition(uint8_t state, uint8_t prev_state, uint8_t reason) {
    event_state = state;
    event_prev_state = prev_state;
    event_reason = reason;
    event_valid = true;
    send_event_frame();  // 即時送信(リレー未学習時は黙って失敗)
}

void telemetry_update(void) {
    // TLM_EVENT 2Hz定期再送
    if (event_valid && (millis() - last_event_tx_ms) >= FLIGHT_CONFIG.event_resend_ms) {
        send_event_frame();
    }

    // TLM_STATE 25Hz(16分周)
    state_divider_counter++;
    if (state_divider_counter >= FLIGHT_CONFIG.telemetry_state_divider) {
        state_divider_counter = 0;
        send_state_frame();
    }
}
