// ===========================================================================
// router.cpp — ルーティング/リレー処理/統計の実装
// ===========================================================================
#include "router.hpp"

#include <Arduino.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "config.hpp"
#include "espnow_link.hpp"
#include "uart_link.hpp"

namespace router {
namespace {

// --- 状態(すべてloopコンテキスト専用) ---------------------------------

uint32_t s_tx_seq = 0;  // リレー発フレームのseq(送信者ごとの単調増加、1始まり)

uint32_t s_up_frames = 0;       // シリアル→ESP-NOW 転送数
uint32_t s_down_frames = 0;     // ESP-NOW→シリアル 転送数
uint32_t s_uplink_refused = 0;  // ターゲット未設定/モード不整合で拒否した上りフレーム数

uint32_t s_last_stats_ms = 0;
uint32_t s_last_no_target_warn_ms = 0;
bool s_no_target_warned = false;  // 初回警告を即時に出すためのフラグ

// マルチモード: node ごとの TLM_STATE 間引きカウンタ(SET_PEERS 受理でリセット)
uint8_t s_tlm_skip[stampfly::RLY_MAX_PEERS] = {};
uint32_t s_tlm_decimated = 0;  // 間引きで転送しなかった TLM_STATE 数(意図的破棄)

// seq は1始まり。0 は seq_echo の「未受信」番兵に予約されているため、
// u32 ラップ時は 0 を飛ばして 1 へ戻す(PROTOCOL.md 論理フレーム)。
uint32_t next_seq() {
  if (++s_tx_seq == 0) s_tx_seq = 1;
  return s_tx_seq;
}

// --- リレー発フレームの送信ヘルパ ---------------------------------------

// リレー発の論理フレームを下り(シリアル)へ送る。
bool send_relay_frame(stampfly::MsgType type, const uint8_t* payload,
                      size_t payload_len) {
  uint8_t logical[stampfly::MAX_FRAME_SIZE];
  size_t logical_len = 0;
  if (stampfly::pack_frame(type, next_seq(), payload, payload_len, logical,
                           sizeof(logical),
                           &logical_len) != stampfly::ParseStatus::ok) {
    return false;
  }
  return uart_link::send_logical(logical, logical_len);
}

// printf形式の LOG_TEXT(origin=0)発行。人間向け出力はこの経路のみ
// (UARTへの生テキスト直書きは全面禁止 — PROTOCOL.md設計原則4)。
__attribute__((format(printf, 1, 2)))
void send_log(const char* fmt, ...) {
  char text[stampfly::MAX_LOG_TEXT_SIZE + 1];
  va_list args;
  va_start(args, fmt);
  const int n = vsnprintf(text, sizeof(text), fmt, args);
  va_end(args);
  if (n <= 0) return;
  // n は切り詰め前の本来の長さ(vsnprintf 戻り値)。超過時は vsnprintf がバイト境界で
  // 切るため、UTF-8 文字境界へ合わせ直して多バイト文字の分断を防ぐ(PROTOCOL.md LOG_TEXT)。
  const size_t text_len = stampfly::utf8_truncate_len(
      reinterpret_cast<const uint8_t*>(text), static_cast<size_t>(n),
      stampfly::MAX_LOG_TEXT_SIZE);

  stampfly::LogText msg;
  msg.origin = stampfly::LogText::ORIGIN_RELAY;
  msg.text = reinterpret_cast<const uint8_t*>(text);
  msg.text_len = text_len;

  uint8_t payload[1 + stampfly::MAX_LOG_TEXT_SIZE];
  size_t payload_len = 0;
  if (!stampfly::serialize(msg, payload, sizeof(payload), &payload_len)) return;
  send_relay_frame(stampfly::MsgType::LOG_TEXT, payload, payload_len);
}

// MAC文字列化("AA:BB:CC:DD:EE:FF" + NUL = 18B)
void format_mac(const uint8_t mac[6], char out[18]) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           static_cast<unsigned>(mac[0]), static_cast<unsigned>(mac[1]),
           static_cast<unsigned>(mac[2]), static_cast<unsigned>(mac[3]),
           static_cast<unsigned>(mac[4]), static_cast<unsigned>(mac[5]));
}

// --- 上り転送(シリアル→ESP-NOW) ---------------------------------------

// 上り拒否の共通処理(レート制限つき LOG_TEXT 警告。CMD_SETPOINT は 50Hz で
// 届くため毎フレーム警告するとログが洪水になる)。
void refuse_uplink(const char* why) {
  ++s_uplink_refused;
  const uint32_t now = millis();
  if (!s_no_target_warned ||
      (now - s_last_no_target_warn_ms >=
       relay_config::NO_TARGET_WARN_INTERVAL_MS)) {
    s_no_target_warned = true;
    s_last_no_target_warn_ms = now;
    send_log("uplink refused: %s (%lu dropped)", why,
             static_cast<unsigned long>(s_uplink_refused));
  }
}

void forward_uplink(const stampfly::FrameView& f) {
  if (espnow_link::multi_active()) {
    // マルチモード中の非エンベロープ上りは宛先が曖昧なため拒否する
    // (意図しない機体への CMD_START 等を構造的に防ぐ)。
    refuse_uplink("multi mode active, wrap in RLY_MUX_UP");
    return;
  }
  if (!espnow_link::has_target()) {
    // ターゲット設定完了まで 0x10–0x2F の転送を拒否(PROTOCOL.md RLY_SET_TARGET)。
    refuse_uplink("no target (send RLY_SET_TARGET first)");
    return;
  }

  // 受信時に検証済みのフレームを再パックしてESP-NOWへ(CRC再計算は決定的で
  // バイト同一。seqは送信元PCの値を保存し、リレーで書き換えない)。
  uint8_t logical[stampfly::MAX_FRAME_SIZE];
  size_t logical_len = 0;
  if (stampfly::pack_frame(f.type, f.seq, f.payload, f.len, logical,
                           sizeof(logical),
                           &logical_len) != stampfly::ParseStatus::ok) {
    return;  // f は検証済みのため到達しない
  }
  if (espnow_link::send_logical(logical, logical_len)) {
    ++s_up_frames;
  }
}

// --- リレー宛フレームの処理(0x50–0x5F) ---------------------------------

void handle_set_target(const stampfly::FrameView& f) {
  stampfly::RlySetTarget req;
  stampfly::RlyTargetAck ack;  // 既定: status=ok, mac=全0, channel=0

  if (!stampfly::deserialize(f.payload, f.len, &req)) {
    // ペイロード長不正 — エコーすべき値が読めないため mac/channel は0のまま返す
    ack.status = stampfly::RlyTargetAck::STATUS_INVALID_MAC;
    send_log("SET_TARGET rejected: bad payload len=%u",
             static_cast<unsigned>(f.len));
  } else {
    ack.status = espnow_link::set_target(req);
    std::memcpy(ack.mac, req.mac, sizeof(ack.mac));
    ack.channel = req.wifi_channel;

    char mac_str[18];
    format_mac(req.mac, mac_str);
    if (ack.status == stampfly::RlyTargetAck::STATUS_OK) {
      send_log("target set: %s ch=%u", mac_str,
               static_cast<unsigned>(req.wifi_channel));
    } else {
      send_log("SET_TARGET failed: status=%u mac=%s ch=%u",
               static_cast<unsigned>(ack.status), mac_str,
               static_cast<unsigned>(req.wifi_channel));
    }
  }

  uint8_t payload[stampfly::RlyTargetAck::PAYLOAD_SIZE];
  if (stampfly::serialize(ack, payload, sizeof(payload))) {
    send_relay_frame(stampfly::MsgType::RLY_TARGET_ACK, payload,
                     sizeof(payload));
  }
}

void handle_set_peers(const stampfly::FrameView& f) {
  stampfly::RlySetPeers req;
  stampfly::RlyPeersAck ack;  // 既定: status=ok, count=0, channel=0, failed=FF

  if (!stampfly::deserialize(f.payload, f.len, &req)) {
    // ペイロード構造不正(count と len の不整合を含む)
    ack.status = stampfly::RlyPeersAck::STATUS_BAD_COUNT;
    send_log("SET_PEERS rejected: bad payload len=%u",
             static_cast<unsigned>(f.len));
  } else {
    ack.status = espnow_link::set_peers(req, &ack.failed_index);
    ack.count = req.count;
    ack.wifi_channel = req.wifi_channel;

    if (ack.status == stampfly::RlyPeersAck::STATUS_OK) {
      // 間引きカウンタをリセット(node 割り当てが変わるため)
      std::memset(s_tlm_skip, 0, sizeof(s_tlm_skip));
      if (req.count == 0) {
        send_log("peers cleared (multi mode off)");
      } else {
        char mac_str[18];
        format_mac(req.peers[0].mac, mac_str);
        send_log("peers set: %u nodes ch=%u node0=%s",
                 static_cast<unsigned>(req.count),
                 static_cast<unsigned>(req.wifi_channel), mac_str);
      }
    } else {
      send_log("SET_PEERS failed: status=%u count=%u ch=%u failed_index=%u",
               static_cast<unsigned>(ack.status),
               static_cast<unsigned>(req.count),
               static_cast<unsigned>(req.wifi_channel),
               static_cast<unsigned>(ack.failed_index));
    }
  }

  uint8_t payload[stampfly::RlyPeersAck::PAYLOAD_SIZE];
  if (stampfly::serialize(ack, payload, sizeof(payload))) {
    send_relay_frame(stampfly::MsgType::RLY_PEERS_ACK, payload,
                     sizeof(payload));
  }
}

void handle_mux_up(const stampfly::FrameView& f) {
  stampfly::RlyMuxView mv;
  if (!stampfly::mux_unwrap(f.payload, f.len, &mv)) {
    refuse_uplink("bad RLY_MUX_UP payload");
    return;
  }
  if (!espnow_link::multi_active()) {
    refuse_uplink("RLY_MUX_UP but multi mode inactive (send RLY_SET_PEERS)");
    return;
  }
  if (mv.node_id >= espnow_link::peer_count()) {
    refuse_uplink("RLY_MUX_UP node out of range");
    return;
  }
  // 内側フレームを検証してから電波に載せる(上り型のみ許可 — 誤って
  // 下り/リレー型を機体へ送る事故を構造的に防ぐ)。
  stampfly::FrameView inner;
  if (stampfly::parse_frame(mv.inner, mv.inner_len, &inner) !=
          stampfly::ParseStatus::ok ||
      !stampfly::is_uplink_type(inner.type)) {
    refuse_uplink("RLY_MUX_UP inner frame invalid");
    return;
  }
  // 内側フレームのバイト列をそのまま ESP-NOW へ(単機時と同一の電波形式)
  if (espnow_link::send_logical_to(mv.node_id, mv.inner, mv.inner_len)) {
    ++s_up_frames;
  }
}

void handle_ping(const stampfly::FrameView& f) {
  stampfly::RlyPong pong;
  pong.echo_seq = f.seq;  // PING の seq をエコー(PROTOCOL.md RLY_PONG)
  uint8_t payload[stampfly::RlyPong::PAYLOAD_SIZE];
  if (stampfly::serialize(pong, payload, sizeof(payload))) {
    send_relay_frame(stampfly::MsgType::RLY_PONG, payload, sizeof(payload));
  }
}

// --- 統計(RLY_STATS 1Hz) -----------------------------------------------

void emit_stats() {
  const stampfly::SerialFrameReceiver::Counters& srx = uart_link::rx_counters();
  const uart_link::TxCounters& stx = uart_link::tx_counters();
  const espnow_link::Counters esn = espnow_link::counters_snapshot();

  stampfly::RlyStats stats;
  stats.up_frames = s_up_frames;
  stats.down_frames = s_down_frames;
  // RLY_STATS の検証エラー欄は crc_errors の一つだけなので、ver/len 不整合に
  // よる破棄もここに合算し、破棄フレームを漏れなく可視化する。
  stats.crc_errors = srx.crc_errors + srx.ver_errors + srx.len_errors +
                     esn.rx_crc_errors + esn.rx_ver_errors + esn.rx_len_errors;
  stats.cobs_errors = srx.cobs_errors;
  stats.espnow_send_fail = esn.send_fail;
  // バッファ/キュー容量起因の破棄はすべて overflow_drops に合算する
  // (シリアルRX 256B上限、UART TXキュー満杯、TX時COBSエンコード失敗、
  // ESP-NOW RXキュー満杯)。集計規則は PROTOCOL.md「RLY_STATSのカウンタ
  // 集計規則」が規範。
  stats.overflow_drops = srx.overflow_drops + stx.queue_drops +
                         stx.encode_errors + esn.rx_queue_drops;

  uint8_t payload[stampfly::RlyStats::PAYLOAD_SIZE];
  if (stampfly::serialize(stats, payload, sizeof(payload))) {
    send_relay_frame(stampfly::MsgType::RLY_STATS, payload, sizeof(payload));
  }
}

// --- 下り転送(ESP-NOW→シリアル) ---------------------------------------

void route_espnow_frame(const uint8_t* logical, size_t len, uint8_t node) {
  // 受信cbで検証済み。type は論理フレームのオフセット1(PROTOCOL.md)。
  const uint8_t type = logical[1];
  if (!stampfly::is_downlink_type(type)) {
    // ドローンから上り型/リレー型が届くことはない — 黙って破棄
    return;
  }

  // モード境界の整合性: ピア表切替をまたいで残存した旧帰属フレームは破棄する
  // (espnow_link 側のキューリセットの残余ウィンドウを閉じる二重防御)。
  const bool multi = espnow_link::multi_active();
  if (multi && node == espnow_link::NODE_NONE) return;   // 単機帰属×マルチ中
  if (!multi && node != espnow_link::NODE_NONE) return;  // マルチ帰属×単機中
  if (node != espnow_link::NODE_NONE &&
      node >= espnow_link::peer_count()) {
    return;                                              // 旧表の範囲外 node
  }

  if (node == espnow_link::NODE_NONE) {
    // 単機モード: 従来どおり素通し(LOG_TEXT(0x40)もこのレンジに含まれる)
    if (uart_link::send_logical(logical, len)) {
      ++s_down_frames;
    }
    return;
  }

  // マルチモード: TLM_STATE のみ node ごとの間引き設定を適用
  // (TLM_ACK/TLM_EVENT 等の制御上重要なフレームは間引かない)
  if (type == static_cast<uint8_t>(stampfly::MsgType::TLM_STATE)) {
    const uint8_t div = espnow_link::peer_tlm_div(node);
    if (div > 1) {
      const uint8_t phase = s_tlm_skip[node]++;
      if (s_tlm_skip[node] >= div) s_tlm_skip[node] = 0;
      if (phase != 0) {
        ++s_tlm_decimated;
        return;
      }
    }
  }

  // RLY_MUX_DOWN(node_id + 内側フレーム)で包んで PC へ
  uint8_t payload[stampfly::MAX_PAYLOAD_SIZE];
  const size_t n = stampfly::mux_wrap(node, logical, len, payload,
                                      sizeof(payload));
  if (n != 0 &&
      send_relay_frame(stampfly::MsgType::RLY_MUX_DOWN, payload, n)) {
    ++s_down_frames;
  }
}

}  // namespace

// --- 公開API ---------------------------------------------------------------

void init(bool espnow_ok) {
  s_last_stats_ms = millis();
  if (espnow_ok) {
    send_log("relay ready (protocol v%u)",
             static_cast<unsigned>(stampfly::PROTOCOL_VERSION));
  } else {
    send_log("relay ERROR: ESP-NOW init failed - uplink unavailable");
  }
}

void on_serial_frame(const stampfly::FrameView& frame) {
  if (stampfly::is_uplink_type(frame.type)) {
    forward_uplink(frame);
    return;
  }
  if (stampfly::is_relay_type(frame.type)) {
    switch (static_cast<stampfly::MsgType>(frame.type)) {
      case stampfly::MsgType::RLY_SET_TARGET:
        handle_set_target(frame);
        break;
      case stampfly::MsgType::RLY_SET_PEERS:
        handle_set_peers(frame);
        break;
      case stampfly::MsgType::RLY_MUX_UP:
        handle_mux_up(frame);
        break;
      case stampfly::MsgType::RLY_PING:
        handle_ping(frame);
        break;
      default:
        // PC発の ACK/STATS/PONG や未知のリレー型(0x59–0x5F)は処理しない
        break;
    }
    return;
  }
  // 下り型(0x30–0x4F)や範囲外型がPCから届いた場合は転送しない(無視)
}

void poll(uint32_t now_ms) {
  // ESP-NOW受信キューの排出(UART帯域とのバランスのため1回あたり上限あり)
  uint8_t frame[stampfly::MAX_FRAME_SIZE];
  size_t frame_len = 0;
  uint8_t node = espnow_link::NODE_NONE;
  for (int i = 0; i < relay_config::ESPNOW_DRAIN_MAX_PER_POLL; ++i) {
    if (!espnow_link::receive(frame, sizeof(frame), &frame_len, &node)) break;
    route_espnow_frame(frame, frame_len, node);
  }

  // RLY_STATS 1Hz自動送信(PROTOCOL.md)
  if (now_ms - s_last_stats_ms >= relay_config::STATS_INTERVAL_MS) {
    s_last_stats_ms = now_ms;
    emit_stats();
  }
}

}  // namespace router
