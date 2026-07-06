// ===========================================================================
// main.cpp — StampFly Integrated Control リレーファームウェア
// ボード: ESP32-WROOM-32E DevKitC
//
// PC(シリアル/COBS+CRC16) ⇔ ドローン(ESP-NOW/論理フレーム直載せ)の
// 双方向中継。仕様は docs/PROTOCOL.md / docs/ARCHITECTURE.md が正。
//
// 構成(各モジュールの責務は各ヘッダ参照):
//   uart_link   — シリアルRXポンプ+TXキュー/専用書き出しタスク(唯一のSerialライタ)
//   espnow_link — ピア管理+受信cb(検証・キュー投入のみ)+送信
//   router      — 型レンジ転送、RLY_*処理、RLY_STATS 1Hz、LOG_TEXT発行
// ===========================================================================
#include <Arduino.h>

#include "config.hpp"
#include "espnow_link.hpp"
#include "router.hpp"
#include "uart_link.hpp"

void setup() {
  uart_link::init();  // 最初に初期化(以後の LOG_TEXT 送出に必要)
  const bool espnow_ok = espnow_link::init();
  router::init(espnow_ok);  // 起動ログ(LOG_TEXT)+統計タイマ開始
}

void loop() {
  uart_link::poll(router::on_serial_frame);  // シリアルRX→ルーティング
  router::poll(millis());                    // ESP-NOW RX排出+1Hz統計
  vTaskDelay(relay_config::LOOP_IDLE_DELAY_TICKS);  // 他タスクへ譲歩(≈1ms)
}
