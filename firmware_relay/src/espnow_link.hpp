// ===========================================================================
// espnow_link.hpp — リレー⇔ドローンのESP-NOWリンク(PROTOCOL.md「トランスポート」)
//
// - ESP-NOWはフレーム境界を保存するため、COBSなしの論理フレームをそのまま
//   ペイロードにする(≦250B、本システムの論理フレームは≦209B)。
// - 受信コールバック(WiFiタスク)は ver/CRC/len 検証+キュー投入のみ。
//   転送判断・Serial出力・ブロッキングは一切行わない(ルーティングは
//   loopコンテキストの router が行う)。
// - ピア管理は2モード排他:
//     単機   — RLY_SET_TARGET で1台(add/replace)
//     マルチ — RLY_SET_PEERS で最大 RLY_MAX_PEERS 台(index = node_id)
//   どちらかを設定するともう一方は解除される。チャネルは
//   esp_wifi_set_channel でピン留めする(全ピア共通 — 無線は1チャネル)。
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

// 受信フレームのノード帰属なし(単機モード)を表す番兵。
constexpr uint8_t NODE_NONE = 0xFF;

// RLY_SET_TARGET の反映(チャネルピン留め+ピアadd/replace)。マルチモードは
// 解除される。戻り値は stampfly::RlyTargetAck::STATUS_*。loopコンテキスト専用。
uint8_t set_target(const stampfly::RlySetTarget& req);

// ターゲット設定済みか(単機モードでの0x10–0x2F転送可否の判定に使う)。
bool has_target();

// RLY_SET_PEERS の反映(チャネルピン留め+最大4ピア登録。count=0 で解除)。
// 単機ターゲットは解除される。戻り値は stampfly::RlyPeersAck::STATUS_*。
// 失敗時は *failed_index に問題のエントリ index(なければ FAILED_NONE)。
// loopコンテキスト専用。
uint8_t set_peers(const stampfly::RlySetPeers& req, uint8_t* failed_index);

// マルチモードが有効か(ピアが1台以上登録済みか)。
bool multi_active();

// 登録済みピア数(0=マルチ無効)。
uint8_t peer_count();

// node の TLM_STATE 間引き設定(1=全転送, n=1/n。未登録 node は 1)。
uint8_t peer_tlm_div(uint8_t node);

// 論理フレーム(ver..crc16)を単機ターゲットへESP-NOW送信。loopコンテキスト専用。
// 送信API受理で true(送達失敗はコールバック経由で send_fail に計上)。
bool send_logical(const uint8_t* frame, size_t len);

// 論理フレームをマルチモードの peers[node] へESP-NOW送信。loopコンテキスト専用。
bool send_logical_to(uint8_t node, const uint8_t* frame, size_t len);

// 受信キューから検証済み論理フレームを1件取り出す(非ブロッキング、loopから)。
// *out_node は送信元ピアの node_id(単機モード受信は NODE_NONE)。
bool receive(uint8_t* out, size_t cap, size_t* out_len, uint8_t* out_node);

// カウンタのスナップショット(クリティカルセクションでコピー)。
Counters counters_snapshot();

}  // namespace espnow_link
