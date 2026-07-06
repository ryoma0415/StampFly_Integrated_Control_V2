#!/usr/bin/env python3
"""test_vectors.json 再生成ツール。

stampfly_protocol.py(PROTOCOL.md 準拠)を正としてベクタを生成し、
書き出す前に Python 実装自身で期待挙動(破損系の破棄・カウンタ)を
セルフチェックする。C++ 側の独立検証は tests/host_test.cpp が行う。

使い方:
    python3 tests/generate_vectors.py        # protocol/test_vectors.json を上書き
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

PROTOCOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROTOCOL_DIR))

import stampfly_protocol as sp  # noqa: E402

OUT_PATH = PROTOCOL_DIR / "test_vectors.json"


def frame_vector(name: str, kind: str, msg_type: int, seq: int,
                 fields: dict, payload: bytes) -> dict:
    logical = sp.pack_frame(msg_type, seq, payload)
    wire = sp.cobs_encode(logical) + bytes([sp.COBS_DELIMITER])
    return {
        "name": name,
        "payload_kind": kind,
        "type": int(msg_type),
        "seq": seq,
        "fields": fields,
        "payload_hex": payload.hex(),
        "logical_hex": logical.hex(),
        "wire_hex": wire.hex(),
    }


def build_vectors() -> dict:
    frames = []

    # --- 2. CMD_SETPOINT seq=0x41424344(旧バグ回帰オマージュ: ペイロード中の
    #     0x41 をヘッダと誤認しないこと)---
    sp1 = sp.CmdSetpoint(roll_ref=0.0524, pitch_ref=-0.0349, alt_ref=0.3, flags=1)
    frames.append(frame_vector(
        "cmd_setpoint_seq_0x41424344", "CMD_SETPOINT",
        sp.MsgType.CMD_SETPOINT, 0x41424344,
        {"roll_ref": 0.0524, "pitch_ref": -0.0349, "alt_ref": 0.3, "flags": 1},
        sp1.to_payload()))

    # --- 3. payload に 0x00 を多数含むフレーム(alt_ref=0.0 等)の COBS 往復 ---
    sp0 = sp.CmdSetpoint(roll_ref=0.0, pitch_ref=0.0, alt_ref=0.0, flags=0)
    frames.append(frame_vector(
        "cmd_setpoint_all_zero_payload", "CMD_SETPOINT",
        sp.MsgType.CMD_SETPOINT, 1,
        {"roll_ref": 0.0, "pitch_ref": 0.0, "alt_ref": 0.0, "flags": 0},
        sp0.to_payload()))

    # --- 4. TLM_STATE: 全フィールド既知値(97B)---
    tlm_fields = {
        "seq_echo": 0x01020304,
        "elapsed_ms": 123456,
        "state": int(sp.FlightState.HOVER),
        "flags": sp.TlmState.FLAG_SETPOINT_FRESH | sp.TlmState.FLAG_FLYING,
        "reason": int(sp.Reason.START_CMD),
        "roll": 0.0123, "pitch": -0.0456, "yaw": 1.5708,
        "p": 0.25, "q": -0.5, "r": 0.75,
        "roll_ref": 0.0524, "pitch_ref": -0.0349,
        "alt_ref": 0.5,
        "altitude_tof": 0.48, "altitude_est": 0.51,
        "alt_velocity": -0.05,
        "z_dot_ref": 0.1,
        "voltage": 3.72,
        "duty_fr": 0.41, "duty_fl": 0.42, "duty_rr": 0.43, "duty_rl": 0.44,
        "ax": 0.01, "ay": -0.02, "az": 0.98,
        "loop_dt_us": 2500,
    }
    tlm = sp.TlmState(**tlm_fields)
    payload = tlm.to_payload()
    assert len(payload) == 97
    frames.append(frame_vector(
        "tlm_state_full", "TLM_STATE", sp.MsgType.TLM_STATE, 1000,
        tlm_fields, payload))

    # --- 追加: 全シリアライザのクロス言語検証用ベクタ ---
    ev = sp.TlmEvent(state=int(sp.FlightState.LANDING), prev_state=int(sp.FlightState.HOVER),
                     reason=int(sp.Reason.LINK_LOSS), flags=1, voltage=3.41)
    frames.append(frame_vector(
        "tlm_event_link_loss", "TLM_EVENT", sp.MsgType.TLM_EVENT, 42,
        {"state": 5, "prev_state": 4, "reason": 8, "flags": 1, "voltage": 3.41},
        ev.to_payload()))

    log = sp.LogText(origin=sp.LogText.ORIGIN_RELAY, text="relay: peer set ch=6")
    frames.append(frame_vector(
        "log_text_relay", "LOG_TEXT", sp.MsgType.LOG_TEXT, 7,
        {"origin": 0, "text": "relay: peer set ch=6"},
        log.to_payload()))

    # PROTOCOL.md は LOG_TEXT を UTF-8 と定義する。多バイト文字(3B日本語+4B非BMP)の
    # クロス言語バイト一致を強制する。ensure_ascii=True で書き出すため、JSON 上は
    # \uXXXX(非BMPはサロゲートペア)となり、host_test.cpp の復元パスも検証される。
    utf8_text = "機体🛸: 高度0.50mに到達"
    log_utf8 = sp.LogText(origin=sp.LogText.ORIGIN_DRONE, text=utf8_text)
    frames.append(frame_vector(
        "log_text_drone_utf8", "LOG_TEXT", sp.MsgType.LOG_TEXT, 8,
        {"origin": 1, "text": utf8_text},
        log_utf8.to_payload()))

    mac = bytes([0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC])
    st = sp.RlySetTarget(mac=mac, wifi_channel=6)
    frames.append(frame_vector(
        "rly_set_target", "RLY_SET_TARGET", sp.MsgType.RLY_SET_TARGET, 2,
        {"mac": list(mac), "wifi_channel": 6},
        st.to_payload()))

    ack = sp.RlyTargetAck(status=sp.RlyTargetAck.STATUS_OK, mac=mac, channel=6)
    frames.append(frame_vector(
        "rly_target_ack_ok", "RLY_TARGET_ACK", sp.MsgType.RLY_TARGET_ACK, 3,
        {"status": 0, "mac": list(mac), "channel": 6},
        ack.to_payload()))

    stats = sp.RlyStats(up_frames=1000, down_frames=2500, crc_errors=3,
                        cobs_errors=1, espnow_send_fail=2, overflow_drops=0)
    frames.append(frame_vector(
        "rly_stats", "RLY_STATS", sp.MsgType.RLY_STATS, 11,
        {"up_frames": 1000, "down_frames": 2500, "crc_errors": 3,
         "cobs_errors": 1, "espnow_send_fail": 2, "overflow_drops": 0},
        stats.to_payload()))

    pong = sp.RlyPong(echo_seq=77)
    frames.append(frame_vector(
        "rly_pong", "RLY_PONG", sp.MsgType.RLY_PONG, 12,
        {"echo_seq": 77},
        pong.to_payload()))

    frames.append(frame_vector(
        "cmd_start_empty_payload", "NONE", sp.MsgType.CMD_START, 5, {}, b""))

    by_name = {f["name"]: f for f in frames}

    def logical_of(name: str) -> bytes:
        return bytes.fromhex(by_name[name]["logical_hex"])

    # --- 5. 破損系 ---
    base = "cmd_setpoint_seq_0x41424344"
    second = "cmd_setpoint_all_zero_payload"

    # 5a. CRC 1ビット反転 → bad_crc で破棄
    corrupted = bytearray(logical_of(base))
    corrupted[-1] ^= 0x01  # CRC上位バイト(LE格納の末尾)を1ビット反転
    crc_flip_wire = sp.cobs_encode(bytes(corrupted)) + b"\x00"

    # 5b. デリミタ欠落 → 2フレームが連結され len 不整合 → 両方破棄
    concat_wire = (sp.cobs_encode(logical_of(base)) +
                   sp.cobs_encode(logical_of(second)) + b"\x00")

    # 5c. 256B超 → 次の 0x00 まで読み捨て、その後の正常フレームは受信できる
    oversize_wire = (b"\xaa" * 300 + b"\x00" +
                     bytes.fromhex(by_name[base]["wire_hex"]))

    corruption = [
        {
            "name": "crc_single_bit_flip",
            "construct": {"kind": "crc_bit_flip", "base_frame": base,
                          "xor_last_byte": 1},
            "wire_hex": crc_flip_wire.hex(),
            "expect_frames": 0,
            "expect_counters": {"crc_errors": 1},
        },
        {
            "name": "missing_delimiter_concatenation",
            "construct": {"kind": "concat_no_delimiter",
                          "frames": [base, second]},
            "wire_hex": concat_wire.hex(),
            "expect_frames": 0,
            "expect_counters": {"len_errors": 1},
        },
        {
            "name": "oversize_drop_then_valid_frame",
            "construct": {"kind": "oversize_junk_then_frame",
                          "junk_byte": 0xAA, "junk_len": 300,
                          "base_frame": base},
            "wire_hex": oversize_wire.hex(),
            "expect_frames": 1,
            "expect_counters": {"overflow_drops": 1},
            "expect_frame_logical_hex": [by_name[base]["logical_hex"]],
        },
    ]

    # --- 6. UTF-8 文字境界切り詰め(utf8_truncate_len)のクロス言語ベクタ ---
    # 1B(ASCII)/3B(日本語)/4B(非BMP)文字の混在テキストで全境界を網羅する。
    truncate_bytes = "log: 高度0.5m🛸到達".encode("utf-8")
    utf8_truncate = {
        "text_hex": truncate_bytes.hex(),
        "cases": [
            {"max_len": n, "expect_len": sp.utf8_truncate_len(truncate_bytes, n)}
            for n in range(len(truncate_bytes) + 1)
        ],
    }

    return {
        "protocol_version": sp.PROTOCOL_VERSION,
        "generator": "protocol/tests/generate_vectors.py (stampfly_protocol.py)",
        "crc16": {"input_ascii": "123456789", "expected": 0x29B1},
        "frames": frames,
        "corruption": corruption,
        "utf8_truncate": utf8_truncate,
    }


def self_check(vectors: dict) -> None:
    """書き出し前に Python 実装自身でベクタの整合性を検証する。"""
    # CRC 検証ベクタ
    assert sp.crc16_ccitt_false(b"123456789") == vectors["crc16"]["expected"] == 0x29B1

    for fv in vectors["frames"]:
        logical = bytes.fromhex(fv["logical_hex"])
        wire = bytes.fromhex(fv["wire_hex"])
        # COBS 往復
        assert sp.cobs_decode(wire[:-1]) == logical
        # parse 往復
        status, frame = sp.parse_frame(logical)
        assert status is sp.ParseStatus.OK and frame is not None
        assert frame.type == fv["type"] and frame.seq == fv["seq"]
        assert frame.payload == bytes.fromhex(fv["payload_hex"])
        # レシーバ経由
        rx = sp.SerialFrameReceiver()
        got = rx.feed(wire)
        assert len(got) == 1 and got[0] == frame
        assert rx.counters.frames_ok == 1

    for cv in vectors["corruption"]:
        rx = sp.SerialFrameReceiver()
        got = rx.feed(bytes.fromhex(cv["wire_hex"]))
        assert len(got) == cv["expect_frames"], cv["name"]
        for key, value in cv["expect_counters"].items():
            assert getattr(rx.counters, key) == value, (cv["name"], key)
        for frame, expected_hex in zip(got, cv.get("expect_frame_logical_hex", [])):
            repacked = sp.pack_frame(frame.type, frame.seq, frame.payload)
            assert repacked == bytes.fromhex(expected_hex), cv["name"]

    # UTF-8 切り詰めベクタ: 期待値が「文字を分断せず、高々1文字しか余分に削らない」こと
    text = bytes.fromhex(vectors["utf8_truncate"]["text_hex"])
    for case in vectors["utf8_truncate"]["cases"]:
        cut = case["expect_len"]
        assert cut == sp.utf8_truncate_len(text, case["max_len"])
        assert cut <= case["max_len"]
        assert min(case["max_len"], len(text)) - cut < 4  # 削るのは高々1文字(≦4B)
        text[:cut].decode("utf-8")  # strict: 分断があれば UnicodeDecodeError


def main() -> None:
    vectors = build_vectors()
    self_check(vectors)
    OUT_PATH.write_text(json.dumps(vectors, indent=2, ensure_ascii=True) + "\n")
    print(f"wrote {OUT_PATH} "
          f"({len(vectors['frames'])} frame vectors, "
          f"{len(vectors['corruption'])} corruption vectors)")


if __name__ == "__main__":
    main()
