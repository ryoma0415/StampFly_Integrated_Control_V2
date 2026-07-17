// ===========================================================================
// telemetry.hpp — TLM_STATE / TLM_EVENT / TLM_CTRL 生成
//
// PROTOCOL.md レート規範:
// - TLM_STATE: 25Hz(400Hzループの telemetry_state_divider=16 分周)
// - TLM_EVENT: 状態遷移時に即時送信 + event_resend_ms=500ms(2Hz)で定期再送
// - TLM_CTRL: 全状態で常時 25Hz(TLM_STATE と 4tick 位相をずらす)
// - TLM_EXP: MOTOR_TEST 状態でのみ 25Hz(TLM_STATE と 8tick 位相をずらす)
// - TLM_ACK / TLM_CAL_DATA: コマンド処理(flight_control)からの即時送信
// ===========================================================================
#pragma once

#include <stdint.h>

// 状態遷移(または遷移なしの理由通知: START拒否等)を即時TLM_EVENT送信する。
// state/prev_state は stampfly::FlightState、reason は stampfly::Reason の数値。
void telemetry_notify_transition(uint8_t state, uint8_t prev_state, uint8_t reason);

// 400Hzループから毎tick呼ぶ。内部で分周してTLM_STATE/TLM_EVENT再送/
// TLM_CTRL/TLM_EXPを行う。
void telemetry_update(void);

// v2 コマンド(0x14-0x23)への TLM_ACK を即時送信する(400Hzループ専用)。
// status は stampfly::TlmAck::STATUS_*。
void telemetry_send_ack(uint8_t acked_type, uint32_t acked_seq, uint8_t status);

// CMD_CAL_GET への応答 TLM_CAL_DATA を即時送信する(400Hzループ専用)。
// 内容は g_yaw_est のキャリブレーション状態のスナップショット。
void telemetry_send_cal_data(void);
