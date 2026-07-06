// ===========================================================================
// host_test.cpp — stampfly_protocol.hpp のホスト検証
//
// test_vectors.json(Python実装が生成)を読み込み、C++ 実装が独立に
// 同一バイト列を再導出すること・破損系で同一の破棄挙動になることを検証する。
//
// ビルド:  g++ -std=c++17 -Wall -Wextra -Werror -O2 -I.. host_test.cpp -o host_test
// 実行:    ./host_test ../test_vectors.json
// 成功時は "ALL OK" を出力して終了コード 0、失敗時は FAIL 行を出して 1。
// (tests/test_cross_language.py が subprocess で実行する)
// ===========================================================================

#include "stampfly_protocol.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 最小JSONパーサ(test_vectors.json 読み取り専用。\uXXXX は UTF-8 へ復元)
// ---------------------------------------------------------------------------

// コードポイントを UTF-8 にエンコードして追記する(JSON \uXXXX エスケープ復元用。
// Python json.dumps(ensure_ascii=True) は非 ASCII をすべて \uXXXX で出力する)。
void append_utf8(std::string& out, unsigned code) {
  if (code <= 0x7F) {
    out += static_cast<char>(code);
  } else if (code <= 0x7FF) {
    out += static_cast<char>(0xC0 | (code >> 6));
    out += static_cast<char>(0x80 | (code & 0x3F));
  } else if (code <= 0xFFFF) {
    out += static_cast<char>(0xE0 | (code >> 12));
    out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (code & 0x3F));
  } else {
    out += static_cast<char>(0xF0 | (code >> 18));
    out += static_cast<char>(0x80 | ((code >> 12) & 0x3F));
    out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (code & 0x3F));
  }
}

struct JVal {
  enum class Kind { Null, Bool, Num, Str, Arr, Obj };
  Kind kind = Kind::Null;
  bool boolean = false;
  double num = 0.0;
  std::string str;
  std::vector<JVal> arr;
  std::map<std::string, JVal> obj;

  const JVal& at(const std::string& key) const {
    auto it = obj.find(key);
    if (it == obj.end()) {
      std::fprintf(stderr, "JSON key missing: %s\n", key.c_str());
      std::exit(2);
    }
    return it->second;
  }
  bool has(const std::string& key) const { return obj.count(key) != 0; }
};

class JParser {
 public:
  explicit JParser(const std::string& text) : s_(text), i_(0) {}

  JVal parse() {
    JVal v = value();
    ws();
    if (i_ != s_.size()) die("trailing data");
    return v;
  }

 private:
  const std::string& s_;
  size_t i_;

  [[noreturn]] void die(const char* msg) {
    std::fprintf(stderr, "JSON parse error at offset %zu: %s\n", i_, msg);
    std::exit(2);
  }
  void ws() {
    while (i_ < s_.size() &&
           (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) {
      ++i_;
    }
  }
  char peek() {
    if (i_ >= s_.size()) die("unexpected end of input");
    return s_[i_];
  }
  char next() {
    const char c = peek();
    ++i_;
    return c;
  }
  bool consume(char c) {
    ws();
    if (i_ < s_.size() && s_[i_] == c) {
      ++i_;
      return true;
    }
    return false;
  }
  void expect(char c) {
    if (!consume(c)) die("expected punctuation");
  }
  void literal(const char* lit) {
    const size_t n = std::strlen(lit);
    if (s_.compare(i_, n, lit) != 0) die("bad literal");
    i_ += n;
  }
  unsigned hex_digit(char h) {
    if (h >= '0' && h <= '9') return static_cast<unsigned>(h - '0');
    if (h >= 'a' && h <= 'f') return static_cast<unsigned>(h - 'a' + 10);
    if (h >= 'A' && h <= 'F') return static_cast<unsigned>(h - 'A' + 10);
    die("bad hex digit");
  }
  JVal value() {
    ws();
    const char c = peek();
    if (c == '{') return object();
    if (c == '[') return array();
    if (c == '"') {
      JVal v;
      v.kind = JVal::Kind::Str;
      v.str = string();
      return v;
    }
    if (c == 't' || c == 'f') {
      JVal v;
      v.kind = JVal::Kind::Bool;
      if (c == 't') {
        literal("true");
        v.boolean = true;
      } else {
        literal("false");
      }
      return v;
    }
    if (c == 'n') {
      literal("null");
      return JVal{};
    }
    return number();
  }
  JVal number() {
    const size_t start = i_;
    while (i_ < s_.size() && std::strchr("+-0123456789.eE", s_[i_]) != nullptr) ++i_;
    if (i_ == start) die("bad number");
    JVal v;
    v.kind = JVal::Kind::Num;
    // strtod は正しく丸めるため、Python の float(...) とビット一致する
    v.num = std::strtod(s_.substr(start, i_ - start).c_str(), nullptr);
    return v;
  }
  std::string string() {
    expect('"');
    std::string out;
    for (;;) {
      const char c = next();
      if (c == '"') break;
      if (c == '\\') {
        const char e = next();
        switch (e) {
          case '"': out += '"'; break;
          case '\\': out += '\\'; break;
          case '/': out += '/'; break;
          case 'n': out += '\n'; break;
          case 't': out += '\t'; break;
          case 'r': out += '\r'; break;
          case 'b': out += '\b'; break;
          case 'f': out += '\f'; break;
          case 'u': {
            unsigned code = 0;
            for (int k = 0; k < 4; ++k) code = code * 16 + hex_digit(next());
            if (code >= 0xD800 && code <= 0xDBFF) {
              // サロゲートペア(非BMP文字): 続く \uDC00–\uDFFF と結合する
              if (next() != '\\' || next() != 'u') die("unpaired high surrogate");
              unsigned low = 0;
              for (int k = 0; k < 4; ++k) low = low * 16 + hex_digit(next());
              if (low < 0xDC00 || low > 0xDFFF) die("bad low surrogate");
              code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
            } else if (code >= 0xDC00 && code <= 0xDFFF) {
              die("unpaired low surrogate");
            }
            append_utf8(out, code);
            break;
          }
          default:
            die("bad escape");
        }
      } else {
        out += c;
      }
    }
    return out;
  }
  JVal object() {
    expect('{');
    JVal v;
    v.kind = JVal::Kind::Obj;
    ws();
    if (consume('}')) return v;
    for (;;) {
      ws();
      std::string key = string();
      expect(':');
      v.obj[key] = value();
      if (consume(',')) continue;
      expect('}');
      break;
    }
    return v;
  }
  JVal array() {
    expect('[');
    JVal v;
    v.kind = JVal::Kind::Arr;
    ws();
    if (consume(']')) return v;
    for (;;) {
      v.arr.push_back(value());
      if (consume(',')) continue;
      expect(']');
      break;
    }
    return v;
  }
};

// ---------------------------------------------------------------------------
// チェックヘルパ
// ---------------------------------------------------------------------------

int g_checks = 0;
int g_fails = 0;

std::string bytes_to_hex(const std::vector<uint8_t>& data) {
  static const char* digits = "0123456789abcdef";
  std::string out;
  out.reserve(data.size() * 2);
  for (uint8_t b : data) {
    out += digits[b >> 4];
    out += digits[b & 0x0F];
  }
  return out;
}

std::vector<uint8_t> hex_to_bytes(const std::string& hex) {
  if (hex.size() % 2 != 0) {
    std::fprintf(stderr, "odd-length hex string\n");
    std::exit(2);
  }
  std::vector<uint8_t> out;
  out.reserve(hex.size() / 2);
  auto nibble = [](char c) -> unsigned {
    if (c >= '0' && c <= '9') return static_cast<unsigned>(c - '0');
    if (c >= 'a' && c <= 'f') return static_cast<unsigned>(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return static_cast<unsigned>(c - 'A' + 10);
    std::fprintf(stderr, "bad hex digit\n");
    std::exit(2);
  };
  for (size_t i = 0; i < hex.size(); i += 2) {
    out.push_back(static_cast<uint8_t>((nibble(hex[i]) << 4) | nibble(hex[i + 1])));
  }
  return out;
}

void check(bool ok, const std::string& what) {
  ++g_checks;
  if (!ok) {
    ++g_fails;
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
  }
}

void check_bytes(const std::vector<uint8_t>& got, const std::vector<uint8_t>& want,
                 const std::string& what) {
  ++g_checks;
  if (got != want) {
    ++g_fails;
    std::fprintf(stderr, "FAIL: %s\n  got : %s\n  want: %s\n", what.c_str(),
                 bytes_to_hex(got).c_str(), bytes_to_hex(want).c_str());
  }
}

// ---------------------------------------------------------------------------
// ベクタ検証本体
// ---------------------------------------------------------------------------

float jf(const JVal& fields, const char* key) {
  return static_cast<float>(fields.at(key).num);
}

uint32_t ju32(const JVal& fields, const char* key) {
  return static_cast<uint32_t>(fields.at(key).num);
}

uint8_t ju8(const JVal& fields, const char* key) {
  return static_cast<uint8_t>(fields.at(key).num);
}

// "fields" から C++ シリアライザでペイロードを再導出する
std::vector<uint8_t> build_payload(const std::string& kind, const JVal& f) {
  using namespace stampfly;
  uint8_t buf[256];
  size_t n = 0;
  bool ok = false;
  if (kind == "NONE") {
    return {};
  } else if (kind == "CMD_SETPOINT") {
    CmdSetpoint m;
    m.roll_ref = jf(f, "roll_ref");
    m.pitch_ref = jf(f, "pitch_ref");
    m.alt_ref = jf(f, "alt_ref");
    m.flags = ju8(f, "flags");
    ok = serialize(m, buf, sizeof(buf));
    n = CmdSetpoint::PAYLOAD_SIZE;
  } else if (kind == "TLM_STATE") {
    TlmState m;
    m.seq_echo = ju32(f, "seq_echo");
    m.elapsed_ms = ju32(f, "elapsed_ms");
    m.state = ju8(f, "state");
    m.flags = ju8(f, "flags");
    m.reason = ju8(f, "reason");
    m.roll = jf(f, "roll");
    m.pitch = jf(f, "pitch");
    m.yaw = jf(f, "yaw");
    m.p = jf(f, "p");
    m.q = jf(f, "q");
    m.r = jf(f, "r");
    m.roll_ref = jf(f, "roll_ref");
    m.pitch_ref = jf(f, "pitch_ref");
    m.alt_ref = jf(f, "alt_ref");
    m.altitude_tof = jf(f, "altitude_tof");
    m.altitude_est = jf(f, "altitude_est");
    m.alt_velocity = jf(f, "alt_velocity");
    m.z_dot_ref = jf(f, "z_dot_ref");
    m.voltage = jf(f, "voltage");
    m.duty_fr = jf(f, "duty_fr");
    m.duty_fl = jf(f, "duty_fl");
    m.duty_rr = jf(f, "duty_rr");
    m.duty_rl = jf(f, "duty_rl");
    m.ax = jf(f, "ax");
    m.ay = jf(f, "ay");
    m.az = jf(f, "az");
    m.loop_dt_us = static_cast<uint16_t>(f.at("loop_dt_us").num);
    ok = serialize(m, buf, sizeof(buf));
    n = TlmState::PAYLOAD_SIZE;
  } else if (kind == "TLM_EVENT") {
    TlmEvent m;
    m.state = ju8(f, "state");
    m.prev_state = ju8(f, "prev_state");
    m.reason = ju8(f, "reason");
    m.flags = ju8(f, "flags");
    m.voltage = jf(f, "voltage");
    ok = serialize(m, buf, sizeof(buf));
    n = TlmEvent::PAYLOAD_SIZE;
  } else if (kind == "LOG_TEXT") {
    const std::string& text = f.at("text").str;
    LogText m;
    m.origin = ju8(f, "origin");
    m.text = reinterpret_cast<const uint8_t*>(text.data());
    m.text_len = text.size();
    ok = serialize(m, buf, sizeof(buf), &n);
  } else if (kind == "RLY_SET_TARGET") {
    RlySetTarget m;
    const auto& mac = f.at("mac").arr;
    for (size_t i = 0; i < 6; ++i) m.mac[i] = static_cast<uint8_t>(mac.at(i).num);
    m.wifi_channel = ju8(f, "wifi_channel");
    ok = serialize(m, buf, sizeof(buf));
    n = RlySetTarget::PAYLOAD_SIZE;
  } else if (kind == "RLY_TARGET_ACK") {
    RlyTargetAck m;
    m.status = ju8(f, "status");
    const auto& mac = f.at("mac").arr;
    for (size_t i = 0; i < 6; ++i) m.mac[i] = static_cast<uint8_t>(mac.at(i).num);
    m.channel = ju8(f, "channel");
    ok = serialize(m, buf, sizeof(buf));
    n = RlyTargetAck::PAYLOAD_SIZE;
  } else if (kind == "RLY_STATS") {
    RlyStats m;
    m.up_frames = ju32(f, "up_frames");
    m.down_frames = ju32(f, "down_frames");
    m.crc_errors = ju32(f, "crc_errors");
    m.cobs_errors = ju32(f, "cobs_errors");
    m.espnow_send_fail = ju32(f, "espnow_send_fail");
    m.overflow_drops = ju32(f, "overflow_drops");
    ok = serialize(m, buf, sizeof(buf));
    n = RlyStats::PAYLOAD_SIZE;
  } else if (kind == "RLY_PONG") {
    RlyPong m;
    m.echo_seq = ju32(f, "echo_seq");
    ok = serialize(m, buf, sizeof(buf));
    n = RlyPong::PAYLOAD_SIZE;
  } else {
    std::fprintf(stderr, "unknown payload_kind: %s\n", kind.c_str());
    std::exit(2);
  }
  if (!ok) {
    std::fprintf(stderr, "serialize failed for %s\n", kind.c_str());
    std::exit(2);
  }
  return std::vector<uint8_t>(buf, buf + n);
}

// deserialize -> serialize の往復でバイト保存性を確認する
std::vector<uint8_t> reserialize_payload(const std::string& kind,
                                         const std::vector<uint8_t>& payload) {
  using namespace stampfly;
  uint8_t buf[256];
  size_t n = 0;
  bool ok = false;
  if (kind == "CMD_SETPOINT") {
    CmdSetpoint m;
    ok = deserialize(payload.data(), payload.size(), &m) && serialize(m, buf, sizeof(buf));
    n = CmdSetpoint::PAYLOAD_SIZE;
  } else if (kind == "TLM_STATE") {
    TlmState m;
    ok = deserialize(payload.data(), payload.size(), &m) && serialize(m, buf, sizeof(buf));
    n = TlmState::PAYLOAD_SIZE;
  } else if (kind == "TLM_EVENT") {
    TlmEvent m;
    ok = deserialize(payload.data(), payload.size(), &m) && serialize(m, buf, sizeof(buf));
    n = TlmEvent::PAYLOAD_SIZE;
  } else if (kind == "LOG_TEXT") {
    LogText m;
    ok = deserialize(payload.data(), payload.size(), &m) && serialize(m, buf, sizeof(buf), &n);
  } else if (kind == "RLY_SET_TARGET") {
    RlySetTarget m;
    ok = deserialize(payload.data(), payload.size(), &m) && serialize(m, buf, sizeof(buf));
    n = RlySetTarget::PAYLOAD_SIZE;
  } else if (kind == "RLY_TARGET_ACK") {
    RlyTargetAck m;
    ok = deserialize(payload.data(), payload.size(), &m) && serialize(m, buf, sizeof(buf));
    n = RlyTargetAck::PAYLOAD_SIZE;
  } else if (kind == "RLY_STATS") {
    RlyStats m;
    ok = deserialize(payload.data(), payload.size(), &m) && serialize(m, buf, sizeof(buf));
    n = RlyStats::PAYLOAD_SIZE;
  } else if (kind == "RLY_PONG") {
    RlyPong m;
    ok = deserialize(payload.data(), payload.size(), &m) && serialize(m, buf, sizeof(buf));
    n = RlyPong::PAYLOAD_SIZE;
  } else {
    std::fprintf(stderr, "reserialize: unknown payload_kind: %s\n", kind.c_str());
    std::exit(2);
  }
  if (!ok) {
    std::fprintf(stderr, "reserialize failed for %s\n", kind.c_str());
    std::exit(2);
  }
  return std::vector<uint8_t>(buf, buf + n);
}

void run_frame_vector(const JVal& v) {
  using namespace stampfly;
  const std::string& name = v.at("name").str;
  const std::string& kind = v.at("payload_kind").str;
  const uint8_t type = static_cast<uint8_t>(v.at("type").num);
  const uint32_t seq = static_cast<uint32_t>(v.at("seq").num);
  const std::vector<uint8_t> payload_want = hex_to_bytes(v.at("payload_hex").str);
  const std::vector<uint8_t> logical_want = hex_to_bytes(v.at("logical_hex").str);
  const std::vector<uint8_t> wire_want = hex_to_bytes(v.at("wire_hex").str);

  // 1. ペイロードを C++ シリアライザで再導出
  const std::vector<uint8_t> payload_got = build_payload(kind, v.at("fields"));
  check_bytes(payload_got, payload_want, name + ": payload re-derived");

  // 2. 論理フレームを pack_frame で再導出
  uint8_t fbuf[MAX_FRAME_SIZE];
  size_t flen = 0;
  check(pack_frame(type, seq, payload_got.data(), payload_got.size(),
                   fbuf, sizeof(fbuf), &flen) == ParseStatus::ok,
        name + ": pack_frame status ok");
  check_bytes(std::vector<uint8_t>(fbuf, fbuf + flen), logical_want,
              name + ": logical frame re-derived");

  // 3. ワイヤバイト(COBS + デリミタ)を encode_wire_frame で再導出
  uint8_t wbuf[MAX_WIRE_SIZE];
  size_t wlen = 0;
  check(encode_wire_frame(type, seq, payload_got.data(), payload_got.size(),
                          wbuf, sizeof(wbuf), &wlen) == ParseStatus::ok,
        name + ": encode_wire_frame status ok");
  check_bytes(std::vector<uint8_t>(wbuf, wbuf + wlen), wire_want,
              name + ": wire bytes re-derived");

  // 4. COBS 単体往復(デリミタを除いたワイヤ → 論理フレーム)
  uint8_t dbuf[SERIAL_RX_BUFFER_CAP];
  size_t dlen = 0;
  check(cobs_decode(wire_want.data(), wire_want.size() - 1, dbuf, sizeof(dbuf), &dlen) &&
            std::vector<uint8_t>(dbuf, dbuf + dlen) == logical_want,
        name + ": cobs_decode roundtrip");

  // 5. parse_frame 往復
  FrameView fv;
  check(parse_frame(logical_want.data(), logical_want.size(), &fv) == ParseStatus::ok,
        name + ": parse_frame ok");
  check(fv.ver == PROTOCOL_VERSION && fv.type == type && fv.seq == seq &&
            fv.len == payload_want.size() &&
            std::vector<uint8_t>(fv.payload, fv.payload + fv.len) == payload_want,
        name + ": parsed fields match");

  // 6. deserialize -> serialize でバイト保存
  if (kind != "NONE") {
    check_bytes(reserialize_payload(kind, payload_want), payload_want,
                name + ": deserialize/serialize roundtrip");
  }

  // 7. レシーバ経由(ワイヤ → フレーム)
  SerialFrameReceiver rx;
  int got = 0;
  rx.feed(wire_want.data(), wire_want.size(), [&](const FrameView& fr) {
    ++got;
    check(fr.type == type && fr.seq == seq && fr.len == payload_want.size() &&
              std::vector<uint8_t>(fr.payload, fr.payload + fr.len) == payload_want,
          name + ": receiver frame content");
  });
  check(got == 1 && rx.counters().frames_ok == 1, name + ": receiver got 1 frame");
}

// 破損系ベクタ: construct 情報からワイヤバイトを再導出し、レシーバ挙動を検証
void run_corruption_vector(const JVal& v, const std::map<std::string, JVal>& frames_by_name) {
  using namespace stampfly;
  const std::string& name = v.at("name").str;
  const std::vector<uint8_t> wire_want = hex_to_bytes(v.at("wire_hex").str);

  auto logical_of = [&](const std::string& frame_name) -> std::vector<uint8_t> {
    auto it = frames_by_name.find(frame_name);
    if (it == frames_by_name.end()) {
      std::fprintf(stderr, "unknown base frame: %s\n", frame_name.c_str());
      std::exit(2);
    }
    return hex_to_bytes(it->second.at("logical_hex").str);
  };
  auto cobs_of = [&](const std::vector<uint8_t>& logical) -> std::vector<uint8_t> {
    uint8_t buf[2 * SERIAL_RX_BUFFER_CAP];
    size_t n = 0;
    if (!cobs_encode(logical.data(), logical.size(), buf, sizeof(buf), &n)) {
      std::fprintf(stderr, "cobs_encode failed in corruption construct\n");
      std::exit(2);
    }
    return std::vector<uint8_t>(buf, buf + n);
  };

  // construct からワイヤを再導出(C++ 側の独立再現)
  const JVal& con = v.at("construct");
  const std::string& con_kind = con.at("kind").str;
  std::vector<uint8_t> wire_got;
  if (con_kind == "crc_bit_flip") {
    std::vector<uint8_t> logical = logical_of(con.at("base_frame").str);
    logical.back() ^= static_cast<uint8_t>(con.at("xor_last_byte").num);
    wire_got = cobs_of(logical);
    wire_got.push_back(COBS_DELIMITER);
  } else if (con_kind == "concat_no_delimiter") {
    for (const JVal& fn : con.at("frames").arr) {
      const std::vector<uint8_t> enc = cobs_of(logical_of(fn.str));
      wire_got.insert(wire_got.end(), enc.begin(), enc.end());
    }
    wire_got.push_back(COBS_DELIMITER);
  } else if (con_kind == "oversize_junk_then_frame") {
    const auto junk_byte = static_cast<uint8_t>(con.at("junk_byte").num);
    const auto junk_len = static_cast<size_t>(con.at("junk_len").num);
    wire_got.assign(junk_len, junk_byte);
    wire_got.push_back(COBS_DELIMITER);
    const std::vector<uint8_t> enc = cobs_of(logical_of(con.at("base_frame").str));
    wire_got.insert(wire_got.end(), enc.begin(), enc.end());
    wire_got.push_back(COBS_DELIMITER);
  } else {
    std::fprintf(stderr, "unknown corruption construct: %s\n", con_kind.c_str());
    std::exit(2);
  }
  check_bytes(wire_got, wire_want, name + ": corrupted wire re-derived");

  // レシーバ挙動の検証
  SerialFrameReceiver rx;
  std::vector<std::vector<uint8_t>> received;  // 再パックした論理フレーム
  rx.feed(wire_want.data(), wire_want.size(), [&](const FrameView& fr) {
    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = 0;
    if (pack_frame(fr.type, fr.seq, fr.payload, fr.len, buf, sizeof(buf), &n) !=
        ParseStatus::ok) {
      std::fprintf(stderr, "re-pack failed in corruption test\n");
      std::exit(2);
    }
    received.emplace_back(buf, buf + n);
  });

  const auto expect_frames = static_cast<size_t>(v.at("expect_frames").num);
  check(received.size() == expect_frames, name + ": expected frame count");
  check(rx.counters().frames_ok == expect_frames, name + ": frames_ok counter");

  // 期待カウンタ: 列挙されたものは一致、列挙されていないエラーカウンタは 0
  const JVal& exp = v.at("expect_counters");
  const std::map<std::string, uint32_t> got_counters = {
      {"cobs_errors", rx.counters().cobs_errors},
      {"crc_errors", rx.counters().crc_errors},
      {"ver_errors", rx.counters().ver_errors},
      {"len_errors", rx.counters().len_errors},
      {"overflow_drops", rx.counters().overflow_drops},
  };
  for (const auto& kv : got_counters) {
    const uint32_t want =
        exp.has(kv.first) ? static_cast<uint32_t>(exp.at(kv.first).num) : 0;
    check(kv.second == want, name + ": counter " + kv.first);
  }

  // 生き残るべきフレームの論理バイト一致
  if (v.has("expect_frame_logical_hex")) {
    const auto& expected = v.at("expect_frame_logical_hex").arr;
    check(received.size() == expected.size(), name + ": surviving frame count");
    for (size_t i = 0; i < received.size() && i < expected.size(); ++i) {
      check_bytes(received[i], hex_to_bytes(expected[i].str),
                  name + ": surviving frame bytes");
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: host_test <test_vectors.json>\n");
    return 2;
  }
  std::ifstream file(argv[1], std::ios::binary);
  if (!file) {
    std::fprintf(stderr, "cannot open %s\n", argv[1]);
    return 2;
  }
  std::stringstream ss;
  ss << file.rdbuf();
  const std::string text = ss.str();
  const JVal root = JParser(text).parse();

  // 1. CRC16-CCITT-FALSE 検証ベクタ("123456789" -> 0x29B1)
  {
    const JVal& crc = root.at("crc16");
    const std::string& input = crc.at("input_ascii").str;
    const uint16_t got = stampfly::crc16_ccitt_false(
        reinterpret_cast<const uint8_t*>(input.data()), input.size());
    check(got == static_cast<uint16_t>(crc.at("expected").num), "crc16 vector matches JSON");
    check(got == 0x29B1, "crc16(\"123456789\") == 0x29B1");
  }

  // 2. フレームベクタ(全シリアライザの再導出+往復+レシーバ)
  std::map<std::string, JVal> frames_by_name;
  for (const JVal& fv : root.at("frames").arr) {
    frames_by_name[fv.at("name").str] = fv;
    run_frame_vector(fv);
  }

  // 3. 破損系ベクタ(構築の再導出+破棄挙動+カウンタ)
  for (const JVal& cv : root.at("corruption").arr) {
    run_corruption_vector(cv, frames_by_name);
  }

  // 4. UTF-8 文字境界切り詰め(utf8_truncate_len のクロス言語一致)
  {
    const JVal& ut = root.at("utf8_truncate");
    const std::vector<uint8_t> text = hex_to_bytes(ut.at("text_hex").str);
    for (const JVal& c : ut.at("cases").arr) {
      const size_t max_len = static_cast<size_t>(c.at("max_len").num);
      const size_t want = static_cast<size_t>(c.at("expect_len").num);
      const size_t got = stampfly::utf8_truncate_len(text.data(), text.size(), max_len);
      check(got == want,
            "utf8_truncate_len(max_len=" + std::to_string(max_len) + ")");
    }
  }

  std::printf("CHECKS: %d passed, %d failed\n", g_checks - g_fails, g_fails);
  if (g_fails == 0) {
    std::printf("ALL OK\n");
    return 0;
  }
  return 1;
}
