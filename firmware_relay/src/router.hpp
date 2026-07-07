// ===========================================================================
// router.hpp — 型レンジルーティング(PROTOCOL.md「メッセージ型」)
//
//   0x10–0x2F: シリアル→ESP-NOW(単機モードのみ。ターゲット未設定/マルチ
//              モード中はLOG_TEXT警告つきで拒否)
//   0x30–0x4F: ESP-NOW→シリアル(マルチモードでは RLY_MUX_DOWN で包み、
//              TLM_STATE はピア設定 tlm_state_div に従い間引く)
//   0x50–0x5F: リレー自身で処理(SET_TARGET/SET_PEERS反映+ACK、
//              MUX_UP展開→ピア送信、PING→PONG)
//
// リレーはフレームの中身を解釈せず、型レンジだけで転送先を決める。
// RLY_STATS を1Hzで自動発行。人間向け出力はすべて LOG_TEXT(origin=0)
// フレームで行い、UARTに生テキストを書くことは決してない。
// すべて loop コンテキストで動作する(リレー発フレームのseq管理を単一化)。
// ===========================================================================
#pragma once

#include <cstdint>

#include "stampfly_protocol.hpp"

namespace router {

// 起動ログの発行+統計タイマ初期化。uart_link/espnow_link 初期化後に呼ぶ。
void init(bool espnow_ok);

// シリアル受信ハンドラ(uart_link::poll に渡す。loopコンテキスト)。
void on_serial_frame(const stampfly::FrameView& frame);

// ESP-NOW受信キューの排出+1Hz統計発行(loopから毎回呼ぶ)。
void poll(uint32_t now_ms);

}  // namespace router
