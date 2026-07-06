// ===========================================================================
// espnow_link.hpp — リレー⇔ドローンのESP-NOWリンク(PROTOCOL.md「トランスポート」)
//
// - ESP-NOWはフレーム境界を保存するため、COBSなしの論理フレームをそのまま
//   ペイロードにする(≦250B、本システムの論理フレームは≦209B)。
// - 受信コールバック(WiFiタスク)は ver/CRC/len 検証+キュー投入のみ。
//   転送判断・Serial出力・ブロッキングは一切行わない(ルーティングは
//   loopコンテキストの router が行う)。
// - ピアは RLY_SET_TARGET で1台のみ管理(add/replace)。チャネルは
//   esp_wifi_set_channel でピン留めする。
// ===========================================================================
#pragma once

#include <cstddef>
#include <cstdint>

#include "stampfly_protocol.hpp"

namespace espnow_link {

// ESP-NOW側カウンタ。受信cb(WiFiタスク)とloopの双方が触るため、内部では
// portMUXクリティカルセクションで保護し、counters_snapshot() で取り出す
// (グローバルvolatile通信を増やさない — ARCHITECTURE.md規約)。
struct Counters {
  uint32_t rx_frames = 0;       // 検証OKでキュー投入できた受信フレーム
  uint32_t rx_crc_errors = 0;   // CRC不一致による破棄
  uint32_t rx_ver_errors = 0;   // ver不一致による破棄
  uint32_t rx_len_errors = 0;   // 長さ不整合による破棄
  uint32_t rx_queue_drops = 0;  // 受信キュー満杯による破棄
  uint32_t rx_filtered = 0;     // ターゲット未設定/送信元MAC不一致の破棄
  uint32_t send_fail = 0;       // esp_now_send即時エラー+送達失敗コールバック
};

// WiFi(STA)+ESP-NOW初期化。成功で true。setup() から1回呼ぶ。
bool init();

// RLY_SET_TARGET の反映(チャネルピン留め+ピアadd/replace)。
// 戻り値は stampfly::RlyTargetAck::STATUS_*。loopコンテキスト専用。
uint8_t set_target(const stampfly::RlySetTarget& req);

// ターゲット設定済みか(0x10–0x2F転送可否の判定に使う)。
bool has_target();

// 論理フレーム(ver..crc16)をターゲットへESP-NOW送信。loopコンテキスト専用。
// 送信API受理で true(送達失敗はコールバック経由で send_fail に計上)。
bool send_logical(const uint8_t* frame, size_t len);

// 受信キューから検証済み論理フレームを1件取り出す(非ブロッキング、loopから)。
bool receive(uint8_t* out, size_t cap, size_t* out_len);

// カウンタのスナップショット(クリティカルセクションでコピー)。
Counters counters_snapshot();

}  // namespace espnow_link
