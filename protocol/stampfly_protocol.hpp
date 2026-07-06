// ===========================================================================
// stampfly_protocol.hpp — StampFly Integrated Control 通信プロトコル(C++実装)
//
// docs/PROTOCOL.md v1 が唯一の正(single source of truth)。
// Python実装 stampfly_protocol.py とのバイト互換は test_vectors.json と
// tests/(pytest + host_test.cpp)により強制される。
//
// 設計上の制約:
// - ヘッダオンリー / 純粋C++17。Arduino / ESP-IDF に依存しない
//   (ホスト g++ テストとファームウェアの両方から同一ソースを使う)。
// - 動的確保なし。すべて呼び出し元提供バッファ+上限長で厳格に境界検査する。
// - シリアライズはフィールドごとの明示的リトルエンディアン書き込み
//   (packed構造体の memcpy / パディングに依存しない)。
// - ISR / 受信コールバックから呼べるよう、ブロッキング・出力は一切行わない。
// ===========================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace stampfly {

static_assert(sizeof(float) == 4, "IEEE-754 binary32 float required");

// ---------------------------------------------------------------------------
// 定数(PROTOCOL.md 「論理フレーム」「トランスポート」)
// ---------------------------------------------------------------------------

constexpr uint8_t PROTOCOL_VERSION = 0x01;

constexpr size_t FRAME_HEADER_SIZE = 7;  // ver(1) + type(1) + seq(4) + len(1)
constexpr size_t FRAME_CRC_SIZE = 2;
constexpr size_t FRAME_OVERHEAD = FRAME_HEADER_SIZE + FRAME_CRC_SIZE;  // = 9
constexpr size_t MAX_PAYLOAD_SIZE = 200;
constexpr size_t MAX_FRAME_SIZE = FRAME_OVERHEAD + MAX_PAYLOAD_SIZE;  // = 209

constexpr uint8_t COBS_DELIMITER = 0x00;
constexpr size_t SERIAL_RX_BUFFER_CAP = 256;  // 受信蓄積バッファ上限(超過 → 次の0x00まで読み捨て)

constexpr size_t MAX_LOG_TEXT_SIZE = 180;  // LOG_TEXT の UTF-8 テキスト上限

// COBSエンコード後の最大長(本実装は満杯ブロック直後にも終端コード0x01を出す方式)
constexpr size_t cobs_max_encoded_size(size_t n) { return n + n / 254 + 2; }
// シリアルワイヤ上の1フレーム最大長(COBS + デリミタ1バイト)
constexpr size_t MAX_WIRE_SIZE = cobs_max_encoded_size(MAX_FRAME_SIZE) + 1;

// ---------------------------------------------------------------------------
// メッセージ型 / enum(PROTOCOL.md 「メッセージ型」「enum定義」)
// ---------------------------------------------------------------------------

enum class MsgType : uint8_t {
  // 上り(PC -> ドローン): 0x10–0x2F
  CMD_START = 0x10,     // 離陸開始(payload 0B)
  CMD_STOP = 0x11,      // 即時着陸(payload 0B、全飛行状態で受理)
  CMD_SETPOINT = 0x12,  // 姿勢+高度目標(13B、ハートビート兼用 50Hz)
  CMD_RESET = 0x13,     // COMPLETE からの復帰(payload 0B)
  // 下り(ドローン -> PC): 0x30–0x4F
  TLM_STATE = 0x30,     // フル状態テレメトリ(97B、25Hz)
  TLM_EVENT = 0x31,     // 状態遷移イベント(8B、即時+2Hz)
  // ログ(リレー/ドローン -> PC)
  LOG_TEXT = 0x40,      // 人間向けテキスト(1〜181B)
  // リレー宛/発: 0x50–0x5F
  RLY_SET_TARGET = 0x50,  // ESP-NOW ピア設定(7B)
  RLY_TARGET_ACK = 0x51,  // SET_TARGET 応答(8B)
  RLY_STATS = 0x52,       // リレー統計(24B、1Hz)
  RLY_PING = 0x53,        // 疎通確認(payload 0B)
  RLY_PONG = 0x54,        // PING 応答(4B)
};

// 型レンジルーティング(リレーは中身を解釈せず型で転送先を決める)
constexpr bool is_uplink_type(uint8_t type) { return type >= 0x10 && type <= 0x2F; }    // ドローン行き
constexpr bool is_downlink_type(uint8_t type) { return type >= 0x30 && type <= 0x4F; }  // PC行き
constexpr bool is_relay_type(uint8_t type) { return type >= 0x50 && type <= 0x5F; }     // リレー宛/発

enum class FlightState : uint8_t {
  INIT = 0,
  CALIBRATION = 1,
  WAIT = 2,
  TAKEOFF = 3,
  HOVER = 4,
  LANDING = 5,
  COMPLETE = 6,
};

enum class Reason : uint8_t {
  NONE = 0,
  START_CMD = 1,
  STOP_CMD = 2,
  MAX_FLIGHT_TIME = 3,
  LOW_VOLTAGE = 4,
  START_REJECTED_LOW_VOLTAGE = 5,
  LANDED = 6,
  OVER_G = 7,
  LINK_LOSS = 8,
  RESET_CMD = 9,
  START_REJECTED_NOT_READY = 10,
};

// ---------------------------------------------------------------------------
// CRC16-CCITT-FALSE(poly 0x1021, init 0xFFFF, 非反転, xorout なし)
// 検証ベクタ: ASCII "123456789" -> 0x29B1
// ---------------------------------------------------------------------------

inline uint16_t crc16_ccitt_false(const uint8_t* data, size_t len, uint16_t crc = 0xFFFFu) {
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

// ---------------------------------------------------------------------------
// 明示的リトルエンディアン読み書き(ホストのエンディアンに依存しない)
// ---------------------------------------------------------------------------

inline void wr_u8(uint8_t* p, uint8_t v) { p[0] = v; }

inline void wr_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFFu);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

inline void wr_u32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFFu);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

inline void wr_f32(uint8_t* p, float v) {
  // floatのビットパターンを取り出してLEで書く(フィールド単位のmemcpyは可搬)
  uint32_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  wr_u32(p, bits);
}

inline uint8_t rd_u8(const uint8_t* p) { return p[0]; }

inline uint16_t rd_u16(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

inline uint32_t rd_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

inline float rd_f32(const uint8_t* p) {
  const uint32_t bits = rd_u32(p);
  float v = 0.0f;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

// ---------------------------------------------------------------------------
// COBS(Consistent Overhead Byte Stuffing)
// ---------------------------------------------------------------------------

// COBSエンコード。出力にデリミタ 0x00 は含まない(送信時は呼び出し元が付加)。
// 「入力末尾が満杯(254B)ブロックで終わる場合にも終端コード0x01を出力する」方式で、
// Python実装と完全に同一のアルゴリズム構造(出力はバイト単位で一致する)。
// 戻り値: 成功なら true、*out_len にエンコード後長。out_cap 不足なら false。
inline bool cobs_encode(const uint8_t* in, size_t in_len,
                        uint8_t* out, size_t out_cap, size_t* out_len) {
  if (out == nullptr || out_len == nullptr) return false;
  if (in == nullptr && in_len > 0) return false;
  size_t oi = 0;
  size_t code_idx = 0;
  uint8_t code = 1;
  if (oi >= out_cap) return false;
  out[oi++] = 0;  // 先頭コードバイトのプレースホルダ
  for (size_t i = 0; i < in_len; ++i) {
    const uint8_t b = in[i];
    if (b == 0) {
      out[code_idx] = code;
      code = 1;
      code_idx = oi;
      if (oi >= out_cap) return false;
      out[oi++] = 0;
    } else {
      if (oi >= out_cap) return false;
      out[oi++] = b;
      ++code;
      if (code == 0xFF) {
        out[code_idx] = code;
        code = 1;
        code_idx = oi;
        if (oi >= out_cap) return false;
        out[oi++] = 0;
      }
    }
  }
  out[code_idx] = code;
  *out_len = oi;
  return true;
}

// COBSデコード。入力にデリミタ 0x00 を含めないこと。
// 0x00 混入・ブロック長不足・空入力・out_cap 超過は false(厳格拒否)。
inline bool cobs_decode(const uint8_t* in, size_t in_len,
                        uint8_t* out, size_t out_cap, size_t* out_len) {
  if (in == nullptr || out == nullptr || out_len == nullptr) return false;
  if (in_len == 0) return false;
  size_t i = 0;
  size_t oi = 0;
  while (i < in_len) {
    const uint8_t code = in[i];
    if (code == 0) return false;  // データ内デリミタは不正
    ++i;
    const size_t block = static_cast<size_t>(code) - 1;
    if (i + block > in_len) return false;  // ブロック切れ
    for (size_t k = 0; k < block; ++k) {
      const uint8_t b = in[i + k];
      if (b == 0) return false;
      if (oi >= out_cap) return false;
      out[oi++] = b;
    }
    i += block;
    if (code != 0xFF && i < in_len) {
      if (oi >= out_cap) return false;
      out[oi++] = 0;
    }
  }
  *out_len = oi;
  return true;
}

// ---------------------------------------------------------------------------
// 論理フレーム pack / parse
// ---------------------------------------------------------------------------

enum class ParseStatus : uint8_t {
  ok = 0,
  bad_ver = 1,
  bad_crc = 2,
  bad_len = 3,
  overflow = 4,
};

// 検証済み論理フレームのビュー。payload は呼び出し元バッファ内を指す(非コピー)。
struct FrameView {
  uint8_t ver = 0;
  uint8_t type = 0;
  uint32_t seq = 0;
  uint8_t len = 0;
  const uint8_t* payload = nullptr;
};

// 論理フレーム(ver..crc16)を out に構築する。
// 戻り値: ok / bad_len(payload_len > 200)/ overflow(out_cap 不足)。
inline ParseStatus pack_frame(uint8_t type, uint32_t seq,
                              const uint8_t* payload, size_t payload_len,
                              uint8_t* out, size_t out_cap, size_t* out_len) {
  if (payload_len > MAX_PAYLOAD_SIZE) return ParseStatus::bad_len;
  if (payload == nullptr && payload_len > 0) return ParseStatus::bad_len;
  const size_t total = FRAME_OVERHEAD + payload_len;
  if (out == nullptr || out_len == nullptr || out_cap < total) return ParseStatus::overflow;
  out[0] = PROTOCOL_VERSION;
  out[1] = type;
  wr_u32(out + 2, seq);
  out[6] = static_cast<uint8_t>(payload_len);
  if (payload_len > 0) std::memcpy(out + FRAME_HEADER_SIZE, payload, payload_len);
  const uint16_t crc = crc16_ccitt_false(out, FRAME_HEADER_SIZE + payload_len);
  wr_u16(out + FRAME_HEADER_SIZE + payload_len, crc);
  *out_len = total;
  return ParseStatus::ok;
}

inline ParseStatus pack_frame(MsgType type, uint32_t seq,
                              const uint8_t* payload, size_t payload_len,
                              uint8_t* out, size_t out_cap, size_t* out_len) {
  return pack_frame(static_cast<uint8_t>(type), seq, payload, payload_len,
                    out, out_cap, out_len);
}

// 論理フレームを検証・分解する。
// 判定順: 構造(bad_len)→ CRC(bad_crc)→ バージョン(bad_ver)。
// CRC を ver 判定より先に行うのは、CRC不一致フレームの ver バイト自体が
// 信用できないため(CRC を通過して ver!=1 のものだけが本当の別バージョン)。
inline ParseStatus parse_frame(const uint8_t* data, size_t len, FrameView* out) {
  if (data == nullptr || out == nullptr) return ParseStatus::bad_len;
  if (len < FRAME_OVERHEAD) return ParseStatus::bad_len;
  const uint8_t payload_len = data[6];
  if (payload_len > MAX_PAYLOAD_SIZE) return ParseStatus::bad_len;
  if (len != FRAME_OVERHEAD + static_cast<size_t>(payload_len)) return ParseStatus::bad_len;
  const size_t crc_off = FRAME_HEADER_SIZE + payload_len;
  const uint16_t crc_calc = crc16_ccitt_false(data, crc_off);
  const uint16_t crc_rx = rd_u16(data + crc_off);
  if (crc_calc != crc_rx) return ParseStatus::bad_crc;
  if (data[0] != PROTOCOL_VERSION) return ParseStatus::bad_ver;
  out->ver = data[0];
  out->type = data[1];
  out->seq = rd_u32(data + 2);
  out->len = payload_len;
  out->payload = data + FRAME_HEADER_SIZE;
  return ParseStatus::ok;
}

// シリアル区間ワイヤ形式を一括生成: COBS(論理フレーム) + 0x00 デリミタ。
// (タスクコンテキスト用。MAX_FRAME_SIZE のスタックバッファを使う)
inline ParseStatus encode_wire_frame(uint8_t type, uint32_t seq,
                                     const uint8_t* payload, size_t payload_len,
                                     uint8_t* out, size_t out_cap, size_t* out_len) {
  uint8_t logical[MAX_FRAME_SIZE];
  size_t logical_len = 0;
  const ParseStatus st = pack_frame(type, seq, payload, payload_len,
                                    logical, sizeof(logical), &logical_len);
  if (st != ParseStatus::ok) return st;
  if (out == nullptr || out_len == nullptr || out_cap == 0) return ParseStatus::overflow;
  size_t enc_len = 0;
  if (!cobs_encode(logical, logical_len, out, out_cap - 1, &enc_len)) {
    return ParseStatus::overflow;
  }
  out[enc_len] = COBS_DELIMITER;
  *out_len = enc_len + 1;
  return ParseStatus::ok;
}

inline ParseStatus encode_wire_frame(MsgType type, uint32_t seq,
                                     const uint8_t* payload, size_t payload_len,
                                     uint8_t* out, size_t out_cap, size_t* out_len) {
  return encode_wire_frame(static_cast<uint8_t>(type), seq, payload, payload_len,
                           out, out_cap, out_len);
}

// ---------------------------------------------------------------------------
// ペイロード (de)serializer
//
// すべてフィールド単位の明示的LE読み書き。serialize は cap、deserialize は
// len を厳格検査し、不一致は false(バッファ外アクセスは決して行わない)。
// ---------------------------------------------------------------------------

// 0x12 CMD_SETPOINT(13B)— 姿勢+高度目標。ハートビートを兼ねる(50Hz)。
struct CmdSetpoint {
  static constexpr MsgType TYPE = MsgType::CMD_SETPOINT;
  static constexpr size_t PAYLOAD_SIZE = 4 + 4 + 4 + 1;
  static constexpr uint8_t FLAG_ALT_REF_VALID = 0x01;  // bit0: alt_ref 有効

  float roll_ref = 0.0f;   // rad
  float pitch_ref = 0.0f;  // rad
  float alt_ref = 0.0f;    // m
  uint8_t flags = 0;
};
static_assert(CmdSetpoint::PAYLOAD_SIZE == 13, "PROTOCOL.md: CMD_SETPOINT payload = 13B");

inline bool serialize(const CmdSetpoint& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdSetpoint::PAYLOAD_SIZE) return false;
  wr_f32(out + 0, m.roll_ref);
  wr_f32(out + 4, m.pitch_ref);
  wr_f32(out + 8, m.alt_ref);
  wr_u8(out + 12, m.flags);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdSetpoint* out) {
  if (in == nullptr || out == nullptr || len != CmdSetpoint::PAYLOAD_SIZE) return false;
  out->roll_ref = rd_f32(in + 0);
  out->pitch_ref = rd_f32(in + 4);
  out->alt_ref = rd_f32(in + 8);
  out->flags = rd_u8(in + 12);
  return true;
}

// 0x30 TLM_STATE(97B)— フル状態テレメトリ(25Hz、400Hzループの16分周)。
// オフセットは PROTOCOL.md の表と1対1対応(宣言順に隙間なくパック)。
struct TlmState {
  static constexpr MsgType TYPE = MsgType::TLM_STATE;
  static constexpr size_t PAYLOAD_SIZE =
      4 + 4 + 1 + 1 + 1 +  // seq_echo, elapsed_ms, state, flags, reason
      4 * 3 +              // roll, pitch, yaw
      4 * 3 +              // p, q, r
      4 * 2 +              // roll_ref, pitch_ref
      4 +                  // alt_ref
      4 * 2 +              // altitude_tof, altitude_est
      4 +                  // alt_velocity
      4 +                  // z_dot_ref
      4 +                  // voltage
      4 * 4 +              // duty_fr, duty_fl, duty_rr, duty_rl
      4 * 3 +              // ax, ay, az
      2;                   // loop_dt_us
  static constexpr uint8_t FLAG_LOW_VOLTAGE = 0x01;     // bit0
  static constexpr uint8_t FLAG_SETPOINT_FRESH = 0x02;  // bit1 (<200ms)
  static constexpr uint8_t FLAG_FLYING = 0x04;          // bit2

  uint32_t seq_echo = 0;      // 最後に適用した CMD_SETPOINT の seq(未受信なら0)
  uint32_t elapsed_ms = 0;    // 起動からの経過 [ms]
  uint8_t state = 0;          // FlightState
  uint8_t flags = 0;
  uint8_t reason = 0;         // Reason(直近の遷移理由)
  float roll = 0.0f;          // rad(実測姿勢, AHRS)
  float pitch = 0.0f;         // rad
  float yaw = 0.0f;           // rad
  float p = 0.0f;             // rad/s(実測角速度)
  float q = 0.0f;             // rad/s
  float r = 0.0f;             // rad/s
  float roll_ref = 0.0f;      // rad(適用中の指令)
  float pitch_ref = 0.0f;     // rad
  float alt_ref = 0.0f;       // m(適用中の目標高度)
  float altitude_tof = 0.0f;  // m(ToF生値)
  float altitude_est = 0.0f;  // m(カルマン推定)
  float alt_velocity = 0.0f;  // m/s
  float z_dot_ref = 0.0f;     // m/s
  float voltage = 0.0f;       // V
  float duty_fr = 0.0f;       // 0–1
  float duty_fl = 0.0f;
  float duty_rr = 0.0f;
  float duty_rl = 0.0f;
  float ax = 0.0f;            // g(フィルタ後加速度)
  float ay = 0.0f;
  float az = 0.0f;
  uint16_t loop_dt_us = 0;    // µs(直近の実測制御周期)
};
static_assert(TlmState::PAYLOAD_SIZE == 97, "PROTOCOL.md: TLM_STATE payload = 97B");

inline bool serialize(const TlmState& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < TlmState::PAYLOAD_SIZE) return false;
  wr_u32(out + 0, m.seq_echo);
  wr_u32(out + 4, m.elapsed_ms);
  wr_u8(out + 8, m.state);
  wr_u8(out + 9, m.flags);
  wr_u8(out + 10, m.reason);
  wr_f32(out + 11, m.roll);
  wr_f32(out + 15, m.pitch);
  wr_f32(out + 19, m.yaw);
  wr_f32(out + 23, m.p);
  wr_f32(out + 27, m.q);
  wr_f32(out + 31, m.r);
  wr_f32(out + 35, m.roll_ref);
  wr_f32(out + 39, m.pitch_ref);
  wr_f32(out + 43, m.alt_ref);
  wr_f32(out + 47, m.altitude_tof);
  wr_f32(out + 51, m.altitude_est);
  wr_f32(out + 55, m.alt_velocity);
  wr_f32(out + 59, m.z_dot_ref);
  wr_f32(out + 63, m.voltage);
  wr_f32(out + 67, m.duty_fr);
  wr_f32(out + 71, m.duty_fl);
  wr_f32(out + 75, m.duty_rr);
  wr_f32(out + 79, m.duty_rl);
  wr_f32(out + 83, m.ax);
  wr_f32(out + 87, m.ay);
  wr_f32(out + 91, m.az);
  wr_u16(out + 95, m.loop_dt_us);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, TlmState* out) {
  if (in == nullptr || out == nullptr || len != TlmState::PAYLOAD_SIZE) return false;
  out->seq_echo = rd_u32(in + 0);
  out->elapsed_ms = rd_u32(in + 4);
  out->state = rd_u8(in + 8);
  out->flags = rd_u8(in + 9);
  out->reason = rd_u8(in + 10);
  out->roll = rd_f32(in + 11);
  out->pitch = rd_f32(in + 15);
  out->yaw = rd_f32(in + 19);
  out->p = rd_f32(in + 23);
  out->q = rd_f32(in + 27);
  out->r = rd_f32(in + 31);
  out->roll_ref = rd_f32(in + 35);
  out->pitch_ref = rd_f32(in + 39);
  out->alt_ref = rd_f32(in + 43);
  out->altitude_tof = rd_f32(in + 47);
  out->altitude_est = rd_f32(in + 51);
  out->alt_velocity = rd_f32(in + 55);
  out->z_dot_ref = rd_f32(in + 59);
  out->voltage = rd_f32(in + 63);
  out->duty_fr = rd_f32(in + 67);
  out->duty_fl = rd_f32(in + 71);
  out->duty_rr = rd_f32(in + 75);
  out->duty_rl = rd_f32(in + 79);
  out->ax = rd_f32(in + 83);
  out->ay = rd_f32(in + 87);
  out->az = rd_f32(in + 91);
  out->loop_dt_us = rd_u16(in + 95);
  return true;
}

// 0x31 TLM_EVENT(8B)— 状態遷移時に即時送信+2Hzで定期再送。
struct TlmEvent {
  static constexpr MsgType TYPE = MsgType::TLM_EVENT;
  static constexpr size_t PAYLOAD_SIZE = 1 + 1 + 1 + 1 + 4;

  uint8_t state = 0;       // FlightState
  uint8_t prev_state = 0;  // FlightState
  uint8_t reason = 0;      // Reason
  uint8_t flags = 0;
  float voltage = 0.0f;    // V
};
static_assert(TlmEvent::PAYLOAD_SIZE == 8, "PROTOCOL.md: TLM_EVENT payload = 8B");

inline bool serialize(const TlmEvent& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < TlmEvent::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.state);
  wr_u8(out + 1, m.prev_state);
  wr_u8(out + 2, m.reason);
  wr_u8(out + 3, m.flags);
  wr_f32(out + 4, m.voltage);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, TlmEvent* out) {
  if (in == nullptr || out == nullptr || len != TlmEvent::PAYLOAD_SIZE) return false;
  out->state = rd_u8(in + 0);
  out->prev_state = rd_u8(in + 1);
  out->reason = rd_u8(in + 2);
  out->flags = rd_u8(in + 3);
  out->voltage = rd_f32(in + 4);
  return true;
}

// 0x40 LOG_TEXT(1〜181B)— 人間向けテキスト。データUARTへの生テキスト直書きの代替。
// text はビュー(deserialize 時は入力バッファ内を指す。コピーしない)。
struct LogText {
  static constexpr MsgType TYPE = MsgType::LOG_TEXT;
  static constexpr uint8_t ORIGIN_RELAY = 0;
  static constexpr uint8_t ORIGIN_DRONE = 1;

  uint8_t origin = 0;            // 0=relay, 1=drone
  const uint8_t* text = nullptr; // UTF-8(NUL終端は不要・含めない)
  size_t text_len = 0;           // <= MAX_LOG_TEXT_SIZE
};

// 可変長のため out_len 付き。text_len > 180 は拒否する。
inline bool serialize(const LogText& m, uint8_t* out, size_t cap, size_t* out_len) {
  if (out == nullptr || out_len == nullptr) return false;
  if (m.text_len > MAX_LOG_TEXT_SIZE) return false;
  if (m.text == nullptr && m.text_len > 0) return false;
  const size_t total = 1 + m.text_len;
  if (cap < total) return false;
  wr_u8(out + 0, m.origin);
  if (m.text_len > 0) std::memcpy(out + 1, m.text, m.text_len);
  *out_len = total;
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, LogText* out) {
  if (in == nullptr || out == nullptr) return false;
  if (len < 1 || len > 1 + MAX_LOG_TEXT_SIZE) return false;
  out->origin = rd_u8(in + 0);
  out->text = in + 1;
  out->text_len = len - 1;
  return true;
}

// UTF-8 文字境界を保ったまま text を max_len バイト以下に切り詰めた長さを返す
// (PROTOCOL.md LOG_TEXT: 多バイト文字を分断しない)。
// 切り詰め位置が多バイト文字の途中になる場合は、その文字の先頭(リード)バイトまで
// 遡って文字ごと落とす。末尾がすでに不正な UTF-8 の場合は新たな分断を作らないこと
// のみ保証し、入力をそのまま通す(受信側 PC は U+FFFD 置換で受理する)。
inline size_t utf8_truncate_len(const uint8_t* text, size_t len, size_t max_len) {
  if (text == nullptr) return 0;
  const size_t cut = (len <= max_len) ? len : max_len;
  if (cut == 0) return 0;
  // 末尾文字のリードバイトを探す(0b10xxxxxx = 継続バイトを遡る)
  size_t lead = cut - 1;
  while (lead > 0 && (text[lead] & 0xC0) == 0x80) --lead;
  const uint8_t b = text[lead];
  size_t expect;  // リードバイトが宣言するシーケンス長
  if (b < 0x80) {
    expect = 1;                            // ASCII
  } else if ((b & 0xE0) == 0xC0) {
    expect = 2;                            // 110xxxxx
  } else if ((b & 0xF0) == 0xE0) {
    expect = 3;                            // 1110xxxx
  } else if ((b & 0xF8) == 0xF0) {
    expect = 4;                            // 11110xxx
  } else {
    return cut;  // 孤立継続バイト等の不正列: 分断回避の対象外
  }
  return (lead + expect <= cut) ? cut : lead;
}

// 0x50 RLY_SET_TARGET(7B)— ESP-NOW ピア設定。
struct RlySetTarget {
  static constexpr MsgType TYPE = MsgType::RLY_SET_TARGET;
  static constexpr size_t PAYLOAD_SIZE = 6 + 1;

  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  uint8_t wifi_channel = 0;  // 1-13
};
static_assert(RlySetTarget::PAYLOAD_SIZE == 7, "PROTOCOL.md: RLY_SET_TARGET payload = 7B");

inline bool serialize(const RlySetTarget& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < RlySetTarget::PAYLOAD_SIZE) return false;
  for (size_t i = 0; i < 6; ++i) wr_u8(out + i, m.mac[i]);  // MACは配列順(エンディアン無関係)
  wr_u8(out + 6, m.wifi_channel);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlySetTarget* out) {
  if (in == nullptr || out == nullptr || len != RlySetTarget::PAYLOAD_SIZE) return false;
  for (size_t i = 0; i < 6; ++i) out->mac[i] = rd_u8(in + i);
  out->wifi_channel = rd_u8(in + 6);
  return true;
}

// 0x51 RLY_TARGET_ACK(8B)— SET_TARGET への応答。
struct RlyTargetAck {
  static constexpr MsgType TYPE = MsgType::RLY_TARGET_ACK;
  static constexpr size_t PAYLOAD_SIZE = 1 + 6 + 1;
  static constexpr uint8_t STATUS_OK = 0;
  static constexpr uint8_t STATUS_INVALID_MAC = 1;
  static constexpr uint8_t STATUS_PEER_FAILED = 2;

  uint8_t status = 0;
  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  uint8_t channel = 0;
};
static_assert(RlyTargetAck::PAYLOAD_SIZE == 8, "PROTOCOL.md: RLY_TARGET_ACK payload = 8B");

inline bool serialize(const RlyTargetAck& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < RlyTargetAck::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.status);
  for (size_t i = 0; i < 6; ++i) wr_u8(out + 1 + i, m.mac[i]);
  wr_u8(out + 7, m.channel);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlyTargetAck* out) {
  if (in == nullptr || out == nullptr || len != RlyTargetAck::PAYLOAD_SIZE) return false;
  out->status = rd_u8(in + 0);
  for (size_t i = 0; i < 6; ++i) out->mac[i] = rd_u8(in + 1 + i);
  out->channel = rd_u8(in + 7);
  return true;
}

// 0x52 RLY_STATS(24B)— リレー統計(1Hz自動送信)。
struct RlyStats {
  static constexpr MsgType TYPE = MsgType::RLY_STATS;
  static constexpr size_t PAYLOAD_SIZE = 4 * 6;

  uint32_t up_frames = 0;
  uint32_t down_frames = 0;
  uint32_t crc_errors = 0;
  uint32_t cobs_errors = 0;
  uint32_t espnow_send_fail = 0;
  uint32_t overflow_drops = 0;
};
static_assert(RlyStats::PAYLOAD_SIZE == 24, "PROTOCOL.md: RLY_STATS payload = 24B");

inline bool serialize(const RlyStats& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < RlyStats::PAYLOAD_SIZE) return false;
  wr_u32(out + 0, m.up_frames);
  wr_u32(out + 4, m.down_frames);
  wr_u32(out + 8, m.crc_errors);
  wr_u32(out + 12, m.cobs_errors);
  wr_u32(out + 16, m.espnow_send_fail);
  wr_u32(out + 20, m.overflow_drops);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlyStats* out) {
  if (in == nullptr || out == nullptr || len != RlyStats::PAYLOAD_SIZE) return false;
  out->up_frames = rd_u32(in + 0);
  out->down_frames = rd_u32(in + 4);
  out->crc_errors = rd_u32(in + 8);
  out->cobs_errors = rd_u32(in + 12);
  out->espnow_send_fail = rd_u32(in + 16);
  out->overflow_drops = rd_u32(in + 20);
  return true;
}

// 0x54 RLY_PONG(4B)— RLY_PING 応答(PING の seq をエコー)。
struct RlyPong {
  static constexpr MsgType TYPE = MsgType::RLY_PONG;
  static constexpr size_t PAYLOAD_SIZE = 4;

  uint32_t echo_seq = 0;
};
static_assert(RlyPong::PAYLOAD_SIZE == 4, "PROTOCOL.md: RLY_PONG payload = 4B");

inline bool serialize(const RlyPong& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < RlyPong::PAYLOAD_SIZE) return false;
  wr_u32(out + 0, m.echo_seq);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlyPong* out) {
  if (in == nullptr || out == nullptr || len != RlyPong::PAYLOAD_SIZE) return false;
  out->echo_seq = rd_u32(in + 0);
  return true;
}

// 既知型の期待ペイロード長。固定長は値、可変長(LOG_TEXT)は -1、未知型は -2。
// ドローン側受理規則「len==期待値」の判定に使用する。
inline int expected_payload_size(uint8_t type) {
  switch (static_cast<MsgType>(type)) {
    case MsgType::CMD_START:
    case MsgType::CMD_STOP:
    case MsgType::CMD_RESET:
    case MsgType::RLY_PING:
      return 0;
    case MsgType::CMD_SETPOINT:
      return static_cast<int>(CmdSetpoint::PAYLOAD_SIZE);
    case MsgType::TLM_STATE:
      return static_cast<int>(TlmState::PAYLOAD_SIZE);
    case MsgType::TLM_EVENT:
      return static_cast<int>(TlmEvent::PAYLOAD_SIZE);
    case MsgType::LOG_TEXT:
      return -1;
    case MsgType::RLY_SET_TARGET:
      return static_cast<int>(RlySetTarget::PAYLOAD_SIZE);
    case MsgType::RLY_TARGET_ACK:
      return static_cast<int>(RlyTargetAck::PAYLOAD_SIZE);
    case MsgType::RLY_STATS:
      return static_cast<int>(RlyStats::PAYLOAD_SIZE);
    case MsgType::RLY_PONG:
      return static_cast<int>(RlyPong::PAYLOAD_SIZE);
    default:
      return -2;
  }
}

// ---------------------------------------------------------------------------
// 逐次シリアルレシーバ
//
// 0x00 区切りのシリアルバイト列から論理フレームを取り出す。リレー(uart_link)
// と PC 側ホストテストの両方で使用する。動的確保・ブロッキング・出力なし。
//
// 回復則(PROTOCOL.md「トランスポート」):
// - COBSデコード失敗 / CRC不一致 / ver不一致 / len不整合はフレームごと破棄
//   (カウンタ加算のみ。部分回復しない)。
// - 蓄積が 256B を超えたら次の 0x00 まで読み捨て(overflow_drops 加算)。
//
// スレッド安全ではない。単一の受信コンテキストから使用すること。
// ---------------------------------------------------------------------------

class SerialFrameReceiver {
 public:
  // 受信統計。フィールド名は Python 側 ReceiverCounters と同一。
  struct Counters {
    uint32_t frames_ok = 0;
    uint32_t cobs_errors = 0;
    uint32_t crc_errors = 0;
    uint32_t ver_errors = 0;
    uint32_t len_errors = 0;
    uint32_t overflow_drops = 0;
  };

  // 1バイト供給(ポーリング型)。完全な有効フレームが復号できたとき true を
  // 返し、frame() が有効になる。frame() の payload ポインタは次に有効フレーム
  // が復号されるまで有効。
  bool feed(uint8_t byte) {
    if (byte == COBS_DELIMITER) {
      if (dropping_) {
        // 読み捨てモード終了(オーバーフロー分のカウントは突入時に済んでいる)
        dropping_ = false;
        rx_len_ = 0;
        return false;
      }
      if (rx_len_ == 0) return false;  // 連続デリミタ / アイドルは無視
      const size_t n = rx_len_;
      rx_len_ = 0;
      size_t decoded_len = 0;
      if (!cobs_decode(rx_, n, decoded_, sizeof(decoded_), &decoded_len)) {
        ++counters_.cobs_errors;
        return false;
      }
      switch (parse_frame(decoded_, decoded_len, &frame_)) {
        case ParseStatus::ok:
          ++counters_.frames_ok;
          return true;
        case ParseStatus::bad_crc:
          ++counters_.crc_errors;
          return false;
        case ParseStatus::bad_ver:
          ++counters_.ver_errors;
          return false;
        default:  // bad_len(parse_frame は overflow を返さない)
          ++counters_.len_errors;
          return false;
      }
    }
    if (dropping_) return false;
    if (rx_len_ >= SERIAL_RX_BUFFER_CAP) {
      // 上限超過 → 次の 0x00 まで読み捨て
      dropping_ = true;
      rx_len_ = 0;
      ++counters_.overflow_drops;
      return false;
    }
    rx_[rx_len_++] = byte;
    return false;
  }

  // バッファ供給+コールバック型。完成フレームごとに on_frame(const FrameView&)
  // を呼ぶ。std::function を使わないテンプレートなのでヒープ確保なし。
  template <typename OnFrame>
  void feed(const uint8_t* data, size_t len, OnFrame&& on_frame) {
    for (size_t i = 0; i < len; ++i) {
      if (feed(data[i])) on_frame(static_cast<const FrameView&>(frame_));
    }
  }

  const FrameView& frame() const { return frame_; }
  const Counters& counters() const { return counters_; }

  // 蓄積バッファと読み捨て状態をクリアする(カウンタは維持)。
  void reset() {
    rx_len_ = 0;
    dropping_ = false;
  }

  void reset_counters() { counters_ = Counters{}; }

 private:
  uint8_t rx_[SERIAL_RX_BUFFER_CAP] = {};
  uint8_t decoded_[SERIAL_RX_BUFFER_CAP] = {};
  size_t rx_len_ = 0;
  bool dropping_ = false;
  FrameView frame_;
  Counters counters_;
};

}  // namespace stampfly
