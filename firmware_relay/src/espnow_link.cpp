// ===========================================================================
// espnow_link.cpp — ESP-NOWリンク実装
// ===========================================================================
#include "espnow_link.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <cstring>

#include "config.hpp"
#include "esp_now_callback_compat.hpp"

namespace espnow_link {
namespace {

// 受信キューの1要素 = ESP-NOWペイロード(=検証済み論理フレーム)そのまま。
struct RxItem {
  uint8_t len = 0;
  uint8_t data[stampfly::MAX_FRAME_SIZE] = {};
};

// s_mux 保護下の共有状態(loopの set_target / send_logical と
// WiFiタスクの受信・送信コールバックが競合する)
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
bool s_has_target = false;
uint8_t s_target_mac[6] = {};
Counters s_counters;

QueueHandle_t s_rx_queue = nullptr;
bool s_initialized = false;  // setup()でのみ書く

// ユニキャストとして有効なMACか(全ゼロと I/Gビット=マルチキャスト/
// ブロードキャストを拒否)。
bool mac_is_valid_unicast(const uint8_t mac[6]) {
  if ((mac[0] & 0x01u) != 0) return false;
  for (int i = 0; i < 6; ++i) {
    if (mac[i] != 0) return true;
  }
  return false;
}

// クリティカルセクション付きカウンタ加算(WiFiタスク/loop両対応)。
void count(uint32_t Counters::*field) {
  portENTER_CRITICAL(&s_mux);
  ++(s_counters.*field);
  portEXIT_CRITICAL(&s_mux);
}

// 受信コールバック(WiFiタスクコンテキスト)。
// ver/CRC/len の検証とキュー投入のみを行う。ブロッキング・Serial出力・
// 転送処理は行わない(ルーティングは loop の router::poll が担う)。
void on_recv(const EspNowRecvInfo* info, const uint8_t* data, int len) {
  const uint8_t* src = espNowRecvSourceAddress(info);

  uint8_t target[6];
  bool has_target;
  portENTER_CRITICAL(&s_mux);
  has_target = s_has_target;
  std::memcpy(target, s_target_mac, sizeof(target));
  portEXIT_CRITICAL(&s_mux);

  // 設定済みターゲット以外のフレームは転送しない(同チャネルの他機を遮断)
  if (!has_target || src == nullptr ||
      std::memcmp(src, target, sizeof(target)) != 0) {
    count(&Counters::rx_filtered);
    return;
  }

  if (data == nullptr || len < static_cast<int>(stampfly::FRAME_OVERHEAD) ||
      len > static_cast<int>(stampfly::MAX_FRAME_SIZE)) {
    count(&Counters::rx_len_errors);
    return;
  }

  // 検証(PROTOCOL.md「ESP-NOW区間」: 長さ・CRC・ver)。不正はフレームごと破棄。
  stampfly::FrameView view;
  switch (stampfly::parse_frame(data, static_cast<size_t>(len), &view)) {
    case stampfly::ParseStatus::ok:
      break;
    case stampfly::ParseStatus::bad_crc:
      count(&Counters::rx_crc_errors);
      return;
    case stampfly::ParseStatus::bad_ver:
      count(&Counters::rx_ver_errors);
      return;
    default:  // bad_len(parse_frame は overflow を返さない)
      count(&Counters::rx_len_errors);
      return;
  }

  RxItem item;
  item.len = static_cast<uint8_t>(len);
  std::memcpy(item.data, data, static_cast<size_t>(len));
  // WiFiタスクコンテキストなので非ブロッキングの xQueueSend を使う
  if (xQueueSend(s_rx_queue, &item, 0) == pdTRUE) {
    count(&Counters::rx_frames);
  } else {
    count(&Counters::rx_queue_drops);
  }
}

// 送信結果コールバック(WiFiタスクコンテキスト)。カウンタ加算のみ。
void on_send(const EspNowSendInfo* /*info*/, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    count(&Counters::send_fail);
  }
}

}  // namespace

bool init() {
  s_rx_queue = xQueueCreate(relay_config::ESPNOW_RX_QUEUE_LEN, sizeof(RxItem));
  if (s_rx_queue == nullptr) return false;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();              // APへは接続しない(ESP-NOW専用)
  esp_wifi_set_ps(WIFI_PS_NONE);  // 省電力を切り受信レイテンシを安定させる

  if (esp_now_init() != ESP_OK) return false;
  if (esp_now_register_recv_cb(on_recv) != ESP_OK) return false;
  if (esp_now_register_send_cb(on_send) != ESP_OK) return false;

  s_initialized = true;
  return true;
}

uint8_t set_target(const stampfly::RlySetTarget& req) {
  using stampfly::RlyTargetAck;

  if (!s_initialized) return RlyTargetAck::STATUS_PEER_FAILED;
  if (!mac_is_valid_unicast(req.mac)) return RlyTargetAck::STATUS_INVALID_MAC;
  // チャネル範囲外(PROTOCOL.md: 1-13)は「設定を適用できない」失敗として扱う
  // (RLY_TARGET_ACK にチャネル専用のstatusコードはないため)
  if (req.wifi_channel < relay_config::WIFI_CHANNEL_MIN ||
      req.wifi_channel > relay_config::WIFI_CHANNEL_MAX) {
    return RlyTargetAck::STATUS_PEER_FAILED;
  }

  // チャネルピン留め(機体プロファイルのチャネルと一致させる — PROTOCOL.md)
  if (esp_wifi_set_channel(req.wifi_channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    return RlyTargetAck::STATUS_PEER_FAILED;
  }

  // MACが変わる場合は旧ピアを削除(replace)。失敗は無視してよい(未登録等)。
  uint8_t old_mac[6];
  bool had_target;
  portENTER_CRITICAL(&s_mux);
  had_target = s_has_target;
  std::memcpy(old_mac, s_target_mac, sizeof(old_mac));
  portEXIT_CRITICAL(&s_mux);
  if (had_target && std::memcmp(old_mac, req.mac, sizeof(old_mac)) != 0) {
    esp_now_del_peer(old_mac);
  }

  esp_now_peer_info_t peer = {};
  std::memcpy(peer.peer_addr, req.mac, sizeof(req.mac));
  peer.channel = 0;  // 0 = インターフェースの現在チャネル(直前にピン留め済み)
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const esp_err_t err = esp_now_is_peer_exist(req.mac)
                            ? esp_now_mod_peer(&peer)
                            : esp_now_add_peer(&peer);
  if (err != ESP_OK) return RlyTargetAck::STATUS_PEER_FAILED;

  // 共有状態の更新(受信cbのフィルタが新ターゲットを参照するようになる)
  portENTER_CRITICAL(&s_mux);
  std::memcpy(s_target_mac, req.mac, sizeof(s_target_mac));
  s_has_target = true;
  portEXIT_CRITICAL(&s_mux);
  return RlyTargetAck::STATUS_OK;
}

bool has_target() {
  portENTER_CRITICAL(&s_mux);
  const bool v = s_has_target;
  portEXIT_CRITICAL(&s_mux);
  return v;
}

bool send_logical(const uint8_t* frame, size_t len) {
  uint8_t mac[6];
  bool has_target;
  portENTER_CRITICAL(&s_mux);
  has_target = s_has_target;
  std::memcpy(mac, s_target_mac, sizeof(mac));
  portEXIT_CRITICAL(&s_mux);

  // 論理フレームは最大209B(< ESP-NOW上限250B)。範囲外は送信失敗として計上。
  if (!has_target || frame == nullptr || len == 0 ||
      len > stampfly::MAX_FRAME_SIZE) {
    count(&Counters::send_fail);
    return false;
  }
  if (esp_now_send(mac, frame, len) != ESP_OK) {
    count(&Counters::send_fail);
    return false;
  }
  return true;
}

bool receive(uint8_t* out, size_t cap, size_t* out_len) {
  if (out == nullptr || out_len == nullptr || s_rx_queue == nullptr) return false;
  RxItem item;
  if (xQueueReceive(s_rx_queue, &item, 0) != pdTRUE) return false;
  if (cap < item.len) return false;  // 呼び出し側バッファ不足(設計上発生しない)
  std::memcpy(out, item.data, item.len);
  *out_len = item.len;
  return true;
}

Counters counters_snapshot() {
  portENTER_CRITICAL(&s_mux);
  const Counters snapshot = s_counters;
  portEXIT_CRITICAL(&s_mux);
  return snapshot;
}

}  // namespace espnow_link
