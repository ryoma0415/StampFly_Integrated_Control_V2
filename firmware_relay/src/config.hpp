// ===========================================================================
// config.hpp — リレーファームウェア設定
//
// マジックナンバーはすべてここに集約する(ARCHITECTURE.md コーディング規約)。
// ===========================================================================
#pragma once

#include <cstddef>
#include <cstdint>

// UARTボーレートはビルドフラグで上書き可能(-DRELAY_UART_BAUD=115200)。
// 既定 460800 はマルチ機体(最大4機の下り TLM ≈ 15KB/s)を収容するため。
// PC側 server.json の serial.baud と一致させること(PROTOCOL.md)。
#ifndef RELAY_UART_BAUD
#define RELAY_UART_BAUD 460800
#endif

namespace relay_config {

// --- UART(PC⇔リレー。データリンク専用 — 生テキスト出力は全面禁止) ---
constexpr unsigned long UART_BAUD = RELAY_UART_BAUD;  // PROTOCOL.md: 既定 460800 8N1
constexpr size_t UART_DRIVER_RX_BUFFER = 1024;     // ドライバ受信バッファ(ループ遅延への余裕)
constexpr size_t UART_TX_QUEUE_LEN = 128;          // TXキュー深さ(下り最大≈205frame/s(4機、TLM_STATE+TLM_CTRL) → 約0.6秒分)
constexpr uint32_t UART_WRITER_TASK_STACK = 4096;  // バイト単位(ESP-IDFのxTaskCreate準拠)
constexpr uint32_t UART_WRITER_TASK_PRIORITY = 2;  // loopTask(=1) より高くしてTX滞留を防ぐ
constexpr int UART_WRITER_TASK_CORE = 1;           // APP_CPU(WiFiタスクは PRO_CPU=0)

// --- ESP-NOW(リレー⇔ドローン) ---
constexpr size_t ESPNOW_RX_QUEUE_LEN = 64;     // 受信キュー深さ(TLM_STATE 25Hz×4機 → 約0.6秒分)
constexpr uint8_t WIFI_CHANNEL_MIN = 1;        // RLY_SET_TARGET/SET_PEERS の許容チャネル範囲(PROTOCOL.md)
constexpr uint8_t WIFI_CHANNEL_MAX = 13;
constexpr int ESPNOW_DRAIN_MAX_PER_POLL = 16;  // 1回のpollで排出する最大フレーム数(4機分)

// --- ルータ ---
constexpr uint32_t STATS_INTERVAL_MS = 1000;          // RLY_STATS 1Hz(PROTOCOL.md)
constexpr uint32_t NO_TARGET_WARN_INTERVAL_MS = 1000; // ターゲット未設定警告のレート制限
                                                      // (CMD_SETPOINT 50Hz によるログ洪水防止)

// --- メインループ ---
constexpr uint32_t LOOP_IDLE_DELAY_TICKS = 1;  // 1ループごとの譲歩(1 tick ≈ 1ms)

}  // namespace relay_config
