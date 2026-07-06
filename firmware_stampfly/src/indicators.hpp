// ===========================================================================
// indicators.hpp — LED状態表示 + 非ブロッキングブザー
//
// LED: 製品版 led.cpp の配色を新しい状態機械(INIT..COMPLETE)へ対応付け。
// ブザー: 製品版 buzzer.cpp を非ブロッキング化(LEDCトーン開始+終了期限を
// 毎tick確認。制御パスで vTaskDelay は使わない)。製品版のピンバグも修正
// (ブザーはGPIO40。GPIO5はモータピンであり絶対に操作しない)。
// ===========================================================================
#pragma once

#include <stdint.h>

// LED・ブザーの初期化(起動メロディの再生開始を含む。ブロックしない)
void indicators_init(void);

// 400Hzループから毎tick呼ぶ(LEDパターン更新+ブザー期限処理)
void indicators_update(void);

// 状態遷移の通知(遷移に応じたビープを開始する。ブロックしない)
void indicators_notify_transition(uint8_t state, uint8_t prev_state, uint8_t reason);
