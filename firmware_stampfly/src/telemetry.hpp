// ===========================================================================
// telemetry.hpp — TLM_STATE / TLM_EVENT 生成
//
// PROTOCOL.md レート規範:
// - TLM_STATE: 25Hz(400Hzループの telemetry_state_divider=16 分周)
// - TLM_EVENT: 状態遷移時に即時送信 + event_resend_ms=500ms(2Hz)で定期再送
// ===========================================================================
#pragma once

#include <stdint.h>

// 状態遷移(または遷移なしの理由通知: START拒否等)を即時TLM_EVENT送信する。
// state/prev_state は stampfly::FlightState、reason は stampfly::Reason の数値。
void telemetry_notify_transition(uint8_t state, uint8_t prev_state, uint8_t reason);

// 400Hzループから毎tick呼ぶ。内部で分周してTLM_STATE/TLM_EVENT再送を行う。
void telemetry_update(void);
