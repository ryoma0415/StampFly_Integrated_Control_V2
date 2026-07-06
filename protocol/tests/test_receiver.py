"""SerialFrameReceiver の破損・回復セマンティクスのテスト。

PROTOCOL.md「トランスポート」の規範:
- デコード失敗 / CRC不一致 / ver不一致 / len不整合はフレームごと破棄。
- 蓄積バッファ上限 256B。超過したら次の 0x00 まで読み捨て(カウンタ加算)。
- 再同期は「次の 0x00 まで読み捨て」だけで完了する(ヘッダ走査なし)。
"""

from __future__ import annotations

import struct

import stampfly_protocol as sp

VALID_WIRE = sp.encode_wire(
    sp.MsgType.CMD_SETPOINT, 1,
    sp.CmdSetpoint(roll_ref=0.01, pitch_ref=-0.02, alt_ref=0.3,
                   yaw_ref=0.5, flags=3).to_payload())


def make_raw_frame(ver: int, msg_type: int, seq: int, len_field: int,
                   payload: bytes) -> bytes:
    """検証用に len フィールドを偽装できる低レベルフレームビルダ。"""
    body = struct.pack("<BBIB", ver, msg_type, seq, len_field) + payload
    return body + struct.pack("<H", sp.crc16_ccitt_false(body))


def to_wire(logical: bytes) -> bytes:
    return sp.cobs_encode(logical) + b"\x00"


def assert_counters(rx: sp.SerialFrameReceiver, **expected) -> None:
    """指定カウンタは一致、未指定カウンタは 0 であること。"""
    for key in ("frames_ok", "cobs_errors", "crc_errors", "ver_errors",
                "len_errors", "overflow_drops"):
        assert getattr(rx.counters, key) == expected.get(key, 0), key


def test_counters_start_at_zero():
    assert_counters(sp.SerialFrameReceiver())


def test_single_valid_frame():
    rx = sp.SerialFrameReceiver()
    got = rx.feed(VALID_WIRE)
    assert len(got) == 1
    assert_counters(rx, frames_ok=1)


def test_byte_by_byte_feed():
    rx = sp.SerialFrameReceiver()
    got = []
    for i in range(len(VALID_WIRE)):
        got += rx.feed(VALID_WIRE[i:i + 1])
    assert len(got) == 1
    assert_counters(rx, frames_ok=1)


def test_callback_mode():
    seen = []
    rx = sp.SerialFrameReceiver(on_frame=seen.append)
    returned = rx.feed(VALID_WIRE * 3)
    assert len(seen) == 3
    assert seen == returned


def test_idle_delimiters_are_ignored():
    rx = sp.SerialFrameReceiver()
    assert rx.feed(b"\x00" * 50) == []
    assert_counters(rx)
    # アイドルデリミタの後でも普通に受信できる
    assert len(rx.feed(VALID_WIRE)) == 1
    assert_counters(rx, frames_ok=1)


def test_garbage_then_valid_frame_recovers():
    """旧プロトコルの教訓: ノイズからの再同期は「次の0x00まで」だけで完了する。"""
    rx = sp.SerialFrameReceiver()
    garbage = b"\x41\x42\x43\x44hello\x41\x00"  # 0x41 を含むノイズ(旧バグ回帰)
    got = rx.feed(garbage + VALID_WIRE)
    assert len(got) == 1
    assert rx.counters.frames_ok == 1
    # ノイズ部分は1個のエラーとして数えられる(分類は内容次第: cobs か frame 系)
    errors = (rx.counters.cobs_errors + rx.counters.crc_errors +
              rx.counters.ver_errors + rx.counters.len_errors)
    assert errors == 1


def test_truncated_cobs_block_is_cobs_error():
    # コードバイト 0x05 が 4 バイトの継続を要求するが 2 バイトしかない
    rx = sp.SerialFrameReceiver()
    assert rx.feed(b"\x05\x41\x42\x00") == []
    assert_counters(rx, cobs_errors=1)


def test_crc_corruption_dropped():
    logical = bytearray(sp.cobs_decode(VALID_WIRE[:-1]))
    logical[8] ^= 0x40  # ペイロード中の1ビット反転
    rx = sp.SerialFrameReceiver()
    assert rx.feed(to_wire(bytes(logical))) == []
    assert_counters(rx, crc_errors=1)


def test_bad_version_dropped_and_counted():
    # ver=1(旧 v1 機器の混在)だが CRC は正しいフレーム → bad_ver に分類される
    logical = make_raw_frame(1, int(sp.MsgType.CMD_SETPOINT), 1,
                             sp.CmdSetpoint.PAYLOAD_SIZE,
                             sp.CmdSetpoint().to_payload())
    rx = sp.SerialFrameReceiver()
    assert rx.feed(to_wire(logical)) == []
    assert_counters(rx, ver_errors=1)


def test_len_field_mismatch_with_valid_crc_is_len_error():
    # len=4 と申告しつつ実際は 5 バイト載せる(CRC は全体で正しい)
    logical = make_raw_frame(1, int(sp.MsgType.LOG_TEXT), 1, 4, b"abcde")
    rx = sp.SerialFrameReceiver()
    assert rx.feed(to_wire(logical)) == []
    assert_counters(rx, len_errors=1)


def test_payload_len_over_200_is_len_error():
    # len=201 はプロトコル上限(200)超過 → 構造不正
    logical = make_raw_frame(1, int(sp.MsgType.LOG_TEXT), 1, 201, bytes(201))
    rx = sp.SerialFrameReceiver()
    assert rx.feed(to_wire(logical)) == []
    assert_counters(rx, len_errors=1)


def test_short_fragment_is_len_error():
    # COBSとしては正当だがフレーム最小長(9B)未満
    rx = sp.SerialFrameReceiver()
    assert rx.feed(to_wire(b"\x01\x02\x03")) == []
    assert_counters(rx, len_errors=1)


def test_missing_delimiter_concatenation_drops_both():
    wire2 = sp.encode_wire(sp.MsgType.RLY_PING, 9)
    # フレーム1のデリミタが欠落 → 2フレームが連結されて届く
    rx = sp.SerialFrameReceiver()
    got = rx.feed(VALID_WIRE[:-1] + wire2)
    assert got == []
    assert_counters(rx, len_errors=1)  # 連結塊は1個の不正フレームとして破棄
    # その後の正常フレームは受信できる
    assert len(rx.feed(VALID_WIRE)) == 1


def test_exactly_256_bytes_is_not_overflow():
    # ちょうど 256B はバッファ上限内(COBS/フレーム検証で落ちるのは別カウンタ)
    rx = sp.SerialFrameReceiver()
    assert rx.feed(b"\x01" * 256 + b"\x00") == []
    assert_counters(rx, len_errors=1)  # デコード結果255Bのゼロ列 → len不整合


def test_257_bytes_overflows_and_recovers():
    rx = sp.SerialFrameReceiver()
    assert rx.feed(b"\x41" * 257) == []
    assert_counters(rx, overflow_drops=1)
    # 読み捨て中はデリミタまで何も受理しない
    assert rx.feed(b"\x42" * 100) == []
    assert_counters(rx, overflow_drops=1)
    # デリミタで読み捨て終了 → 以後の正常フレームは受信できる
    got = rx.feed(b"\x00" + VALID_WIRE)
    assert len(got) == 1
    assert_counters(rx, overflow_drops=1, frames_ok=1)


def test_overflow_split_across_feeds():
    """オーバーフロー判定が feed 呼び出し境界に依存しないこと。"""
    rx = sp.SerialFrameReceiver()
    for _ in range(30):
        rx.feed(b"\x41" * 10)  # 計300B
    assert_counters(rx, overflow_drops=1)
    got = rx.feed(b"\x00" + VALID_WIRE)
    assert len(got) == 1
    assert_counters(rx, overflow_drops=1, frames_ok=1)


def test_oversize_only_counts_once_per_drop_window():
    rx = sp.SerialFrameReceiver()
    rx.feed(b"\x41" * 1000)  # 1つの読み捨て区間
    assert_counters(rx, overflow_drops=1)
    rx.feed(b"\x00" + b"\x42" * 300 + b"\x00")  # 2つ目の読み捨て区間
    assert_counters(rx, overflow_drops=2)


def test_reset_clears_partial_state_but_keeps_counters():
    rx = sp.SerialFrameReceiver()
    rx.feed(VALID_WIRE[:5])   # 中途半端な蓄積
    rx.reset()
    got = rx.feed(VALID_WIRE)  # 直後の完全フレームは正常受信
    assert len(got) == 1
    assert_counters(rx, frames_ok=1)


def test_reset_counters():
    rx = sp.SerialFrameReceiver()
    rx.feed(VALID_WIRE)
    rx.reset_counters()
    assert_counters(rx)


def test_error_then_valid_stream_interleaved():
    """エラーフレームに挟まれても正常フレームはすべて受信できること。"""
    corrupted = bytearray(VALID_WIRE)
    corrupted[3] ^= 0x10  # COBS本体の破壊 → cobs か crc エラー
    stream = VALID_WIRE + bytes(corrupted) + VALID_WIRE + b"\x00" + VALID_WIRE
    rx = sp.SerialFrameReceiver()
    got = rx.feed(stream)
    assert len(got) == 3
    assert rx.counters.frames_ok == 3
    total_errors = (rx.counters.cobs_errors + rx.counters.crc_errors +
                    rx.counters.ver_errors + rx.counters.len_errors)
    assert total_errors == 1
