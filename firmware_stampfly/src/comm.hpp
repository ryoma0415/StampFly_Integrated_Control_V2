// ===========================================================================
// comm.hpp — ESP-NOW 送受信(機体側)
//
// PROTOCOL.md「ドローン側の受理規則」の実装:
// - 受信フレームは len==期待値 / CRC一致 / ver==1 のもののみ受理
// - ブート後最初の有効上りフレームの送信元MACをリレーピアとして学習(以後不変)
// - 受信コールバック(WiFiタスク)は検証+portMUXメールボックス格納のみ。
//   出力・ブロッキングは行わない。400Hzループが comm_consume_commands() で消費。
// - WiFiチャネルは FLIGHT_CONFIG.wifi_channel に esp_wifi_set_channel でピン留め
// ===========================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "stampfly_protocol.hpp"

// 400Hzループが1tickごとに取り出すコマンドのスナップショット。
// 優先度 STOP > START > RESET > SETPOINT は消費側(flight_control)が
// stop_pending を最優先で処理することで成立する。
struct CommandSnapshot {
    bool stop_pending = false;
    bool start_pending = false;
    bool reset_pending = false;
    bool setpoint_pending = false;          // 新しい CMD_SETPOINT が届いたか
    stampfly::CmdSetpoint setpoint{};       // 最新の setpoint(pending時のみ有効)
    uint32_t setpoint_seq = 0;              // その論理フレーム seq
    uint32_t setpoint_rx_ms = 0;            // 受信時刻 millis()
};

// WiFi(STA)+チャネルピン留め+ESP-NOW初期化+受信コールバック登録。
// setup段階で1回呼ぶ(ブート時の数行のシリアル出力を含む)。
void comm_init(void);

// メールボックスの内容を取り出してクリアする(400Hzループ専用)。
// 何かしらのコマンドが入っていたら true。
bool comm_consume_commands(CommandSnapshot* out);

// リレーピアを学習済みか(=下りフレームを送れる状態か)
bool comm_relay_ready(void);

// 論理フレーム(ver/type/seq/len/payload/crc)を組み立ててESP-NOW送信する。
// seq は送信ごとに単調増加(1始まり)。リレー未学習なら false。
// 400Hzループ(単一送信コンテキスト)からのみ呼ぶこと。
bool comm_send_payload(stampfly::MsgType type, const uint8_t* payload, size_t payload_len);

// LOG_TEXT(origin=drone)を送る。データ経路に生テキストは流さない(PROTOCOL.md)。
bool comm_send_log(const char* text);
