"""StampFly Integrated Control — 通信プロトコル(Python実装)。

docs/PROTOCOL.md v1 が唯一の正(single source of truth)。
C++実装 ``stampfly_protocol.hpp`` とのバイト互換は ``test_vectors.json`` と
``tests/`` により強制される。

ワイヤ仕様の要点:

- 論理フレーム: ``ver(1) | type(1) | seq(4, u32 LE) | len(1) | payload | crc16(2, LE)``
- CRC16-CCITT-FALSE: poly 0x1021 / init 0xFFFF / 非反転 / xorout なし。
  ver〜payload 末尾(crc自身を除く全バイト)に適用。検証ベクタ "123456789" -> 0x29B1。
- シリアル区間: 論理フレーム全体を COBS エンコードし、末尾に 0x00 デリミタを付加。
  受信側は 0x00 区切りで蓄積(上限 256B、超過したら次の 0x00 まで読み捨て)。
  デコード失敗 / CRC不一致 / ver不一致 / len不整合は「フレームごと破棄」(部分回復しない)。
- マルチバイト値はすべてリトルエンディアン。float は IEEE-754 binary32 LE。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional

# ---------------------------------------------------------------------------
# 定数(PROTOCOL.md 「論理フレーム」「トランスポート」)
# ---------------------------------------------------------------------------

PROTOCOL_VERSION = 0x01

FRAME_HEADER_SIZE = 7          # ver(1) + type(1) + seq(4) + len(1)
FRAME_CRC_SIZE = 2
FRAME_OVERHEAD = FRAME_HEADER_SIZE + FRAME_CRC_SIZE   # = 9
MAX_PAYLOAD_SIZE = 200
MAX_FRAME_SIZE = FRAME_OVERHEAD + MAX_PAYLOAD_SIZE    # = 209

COBS_DELIMITER = 0x00
SERIAL_RX_BUFFER_CAP = 256     # 受信蓄積バッファ上限(超過 → 次の 0x00 まで読み捨て)

MAX_LOG_TEXT_SIZE = 180        # LOG_TEXT の UTF-8 テキスト上限


# ---------------------------------------------------------------------------
# メッセージ型 / enum(PROTOCOL.md 「メッセージ型」「enum定義」)
# ---------------------------------------------------------------------------

class MsgType(IntEnum):
    # 上り(PC -> ドローン): 0x10–0x2F
    CMD_START = 0x10
    CMD_STOP = 0x11
    CMD_SETPOINT = 0x12
    CMD_RESET = 0x13
    # 下り(ドローン -> PC): 0x30–0x4F
    TLM_STATE = 0x30
    TLM_EVENT = 0x31
    # ログ(リレー/ドローン -> PC)
    LOG_TEXT = 0x40
    # リレー宛/発: 0x50–0x5F
    RLY_SET_TARGET = 0x50
    RLY_TARGET_ACK = 0x51
    RLY_STATS = 0x52
    RLY_PING = 0x53
    RLY_PONG = 0x54


def is_uplink_type(msg_type: int) -> bool:
    """ドローン行き(リレーが上り ESP-NOW へ転送)の型レンジか。"""
    return 0x10 <= msg_type <= 0x2F


def is_downlink_type(msg_type: int) -> bool:
    """PC行き(リレーが下りシリアルへ転送)の型レンジか。"""
    return 0x30 <= msg_type <= 0x4F


def is_relay_type(msg_type: int) -> bool:
    """リレー自身宛/発の型レンジか。"""
    return 0x50 <= msg_type <= 0x5F


class FlightState(IntEnum):
    INIT = 0
    CALIBRATION = 1
    WAIT = 2
    TAKEOFF = 3
    HOVER = 4
    LANDING = 5
    COMPLETE = 6


class Reason(IntEnum):
    NONE = 0
    START_CMD = 1
    STOP_CMD = 2
    MAX_FLIGHT_TIME = 3
    LOW_VOLTAGE = 4
    START_REJECTED_LOW_VOLTAGE = 5
    LANDED = 6
    OVER_G = 7
    LINK_LOSS = 8
    RESET_CMD = 9
    START_REJECTED_NOT_READY = 10


class ParseStatus(IntEnum):
    """parse_frame の結果。C++ 側 ParseStatus と同順。"""
    OK = 0
    BAD_VER = 1
    BAD_CRC = 2
    BAD_LEN = 3
    OVERFLOW = 4


# ---------------------------------------------------------------------------
# CRC16-CCITT-FALSE
# ---------------------------------------------------------------------------

def crc16_ccitt_false(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-CCITT-FALSE(poly 0x1021, init 0xFFFF, 非反転, xorout なし)。"""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# COBS(Consistent Overhead Byte Stuffing)
# ---------------------------------------------------------------------------

class CobsDecodeError(ValueError):
    """COBS デコード失敗(0x00混入 / ブロック切れ / 空入力)。"""


def cobs_encode(data: bytes) -> bytes:
    """COBS エンコード(デリミタ 0x00 は含まない。送信時は呼び出し元が付加する)。

    本実装は「入力末尾が満杯(254B)ブロックで終わる場合にも終端コード 0x01 を
    出力する」方式。C++ 実装と完全に同一のアルゴリズム構造であり、両実装の
    出力はバイト単位で一致する。
    """
    out = bytearray(b"\x00")   # 先頭コードバイトのプレースホルダ
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code = 1
            code_idx = len(out)
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code = 1
                code_idx = len(out)
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """COBS デコード(入力にデリミタ 0x00 を含めないこと)。

    失敗時は CobsDecodeError。0x00 混入・ブロック長不足を厳格に拒否する。
    """
    n = len(data)
    if n == 0:
        raise CobsDecodeError("empty input")
    out = bytearray()
    i = 0
    while i < n:
        code = data[i]
        if code == 0:
            raise CobsDecodeError("delimiter byte inside data")
        i += 1
        end = i + code - 1
        if end > n:
            raise CobsDecodeError("truncated block")
        block = data[i:end]
        if 0 in block:
            raise CobsDecodeError("zero byte inside block")
        out += block
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------------------
# 論理フレーム pack / parse
# ---------------------------------------------------------------------------

_FRAME_HEADER_FMT = "<BBIB"    # ver, type, seq, len
assert struct.calcsize(_FRAME_HEADER_FMT) == FRAME_HEADER_SIZE


@dataclass(frozen=True)
class Frame:
    """検証済み論理フレーム。"""
    type: int
    seq: int
    payload: bytes
    ver: int = PROTOCOL_VERSION


def pack_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    """論理フレーム(ver..crc16)を構築する。"""
    msg_type = int(msg_type)
    if not 0 <= msg_type <= 0xFF:
        raise ValueError(f"msg_type out of range: {msg_type}")
    if not 0 <= seq <= 0xFFFFFFFF:
        raise ValueError(f"seq must fit u32: {seq}")
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD_SIZE}")
    body = struct.pack(_FRAME_HEADER_FMT, PROTOCOL_VERSION, msg_type, seq, len(payload)) + payload
    return body + struct.pack("<H", crc16_ccitt_false(body))


def parse_frame(data: bytes) -> tuple[ParseStatus, Optional[Frame]]:
    """論理フレームを検証・分解する。

    判定順: 構造(BAD_LEN)→ CRC(BAD_CRC)→ バージョン(BAD_VER)。
    CRC を ver 判定より先に行うのは、CRC不一致フレームの ver バイト自体が
    信用できないため(CRC を通過して ver!=1 のものだけが本当の別バージョン)。
    """
    data = bytes(data)
    if len(data) < FRAME_OVERHEAD:
        return ParseStatus.BAD_LEN, None
    payload_len = data[6]
    if payload_len > MAX_PAYLOAD_SIZE:
        return ParseStatus.BAD_LEN, None
    if len(data) != FRAME_OVERHEAD + payload_len:
        return ParseStatus.BAD_LEN, None
    crc_off = FRAME_HEADER_SIZE + payload_len
    crc_calc = crc16_ccitt_false(data[:crc_off])
    (crc_rx,) = struct.unpack_from("<H", data, crc_off)
    if crc_calc != crc_rx:
        return ParseStatus.BAD_CRC, None
    if data[0] != PROTOCOL_VERSION:
        return ParseStatus.BAD_VER, None
    ver, msg_type, seq, _ = struct.unpack_from(_FRAME_HEADER_FMT, data, 0)
    return ParseStatus.OK, Frame(type=msg_type, seq=seq,
                                 payload=data[FRAME_HEADER_SIZE:crc_off], ver=ver)


def encode_wire(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    """シリアル区間ワイヤ形式: COBS(論理フレーム) + 0x00 デリミタ。"""
    return cobs_encode(pack_frame(msg_type, seq, payload)) + bytes([COBS_DELIMITER])


# ---------------------------------------------------------------------------
# ペイロード (de)serializer
#
# struct フォーマットはすべて '<'(リトルエンディアン・パディングなし)で明示。
# 各 from_payload は長さを厳格に検証し、不一致は ValueError。
# ---------------------------------------------------------------------------

_CMD_SETPOINT_FMT = "<fffB"
_TLM_EVENT_FMT = "<BBBBf"
_TLM_STATE_FMT = "<IIBBB21fH"
_RLY_SET_TARGET_FMT = "<6sB"
_RLY_TARGET_ACK_FMT = "<B6sB"
_RLY_STATS_FMT = "<IIIIII"
_RLY_PONG_FMT = "<I"

# PROTOCOL.md 記載のペイロードサイズと一致することを import 時に強制
# (C++ 側の static_assert に対応)
assert struct.calcsize(_CMD_SETPOINT_FMT) == 13
assert struct.calcsize(_TLM_EVENT_FMT) == 8
assert struct.calcsize(_TLM_STATE_FMT) == 97
assert struct.calcsize(_RLY_SET_TARGET_FMT) == 7
assert struct.calcsize(_RLY_TARGET_ACK_FMT) == 8
assert struct.calcsize(_RLY_STATS_FMT) == 24
assert struct.calcsize(_RLY_PONG_FMT) == 4


def _check_len(name: str, data: bytes, expected: int) -> None:
    if len(data) != expected:
        raise ValueError(f"{name}: payload length {len(data)} != {expected}")


@dataclass
class CmdSetpoint:
    """0x12 CMD_SETPOINT(13B)— 姿勢+高度目標。ハートビートを兼ねる(50Hz)。"""
    roll_ref: float = 0.0    # rad
    pitch_ref: float = 0.0   # rad
    alt_ref: float = 0.0     # m
    flags: int = 0           # bit0 = alt_ref 有効(0なら現在の alt_ref 維持)

    TYPE = MsgType.CMD_SETPOINT
    PAYLOAD_SIZE = 13
    FLAG_ALT_REF_VALID = 0x01

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_SETPOINT_FMT, self.roll_ref, self.pitch_ref,
                           self.alt_ref, self.flags)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdSetpoint":
        _check_len("CMD_SETPOINT", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_SETPOINT_FMT, data))


@dataclass
class TlmState:
    """0x30 TLM_STATE(97B)— フル状態テレメトリ(25Hz)。"""
    seq_echo: int = 0        # 最後に適用した CMD_SETPOINT の seq(未受信なら0)
    elapsed_ms: int = 0      # 起動からの経過 [ms]
    state: int = 0           # FlightState
    flags: int = 0           # bit0 low_voltage, bit1 setpoint_fresh(<200ms), bit2 flying
    reason: int = 0          # Reason(直近の遷移理由)
    roll: float = 0.0        # rad(実測姿勢, AHRS)
    pitch: float = 0.0       # rad
    yaw: float = 0.0         # rad
    p: float = 0.0           # rad/s(実測角速度)
    q: float = 0.0           # rad/s
    r: float = 0.0           # rad/s
    roll_ref: float = 0.0    # rad(適用中の指令)
    pitch_ref: float = 0.0   # rad
    alt_ref: float = 0.0     # m(適用中の目標高度)
    altitude_tof: float = 0.0  # m(ToF生値)
    altitude_est: float = 0.0  # m(カルマン推定)
    alt_velocity: float = 0.0  # m/s
    z_dot_ref: float = 0.0     # m/s
    voltage: float = 0.0       # V
    duty_fr: float = 0.0       # 0–1
    duty_fl: float = 0.0
    duty_rr: float = 0.0
    duty_rl: float = 0.0
    ax: float = 0.0            # g(フィルタ後加速度)
    ay: float = 0.0
    az: float = 0.0
    loop_dt_us: int = 0        # µs(直近の実測制御周期)

    TYPE = MsgType.TLM_STATE
    PAYLOAD_SIZE = 97
    FLAG_LOW_VOLTAGE = 0x01
    FLAG_SETPOINT_FRESH = 0x02
    FLAG_FLYING = 0x04

    def to_payload(self) -> bytes:
        return struct.pack(
            _TLM_STATE_FMT,
            self.seq_echo, self.elapsed_ms, self.state, self.flags, self.reason,
            self.roll, self.pitch, self.yaw,
            self.p, self.q, self.r,
            self.roll_ref, self.pitch_ref, self.alt_ref,
            self.altitude_tof, self.altitude_est,
            self.alt_velocity, self.z_dot_ref, self.voltage,
            self.duty_fr, self.duty_fl, self.duty_rr, self.duty_rl,
            self.ax, self.ay, self.az,
            self.loop_dt_us)

    @classmethod
    def from_payload(cls, data: bytes) -> "TlmState":
        _check_len("TLM_STATE", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_TLM_STATE_FMT, data))


@dataclass
class TlmEvent:
    """0x31 TLM_EVENT(8B)— 状態遷移時に即時送信+2Hzで定期再送。"""
    state: int = 0           # FlightState
    prev_state: int = 0      # FlightState
    reason: int = 0          # Reason
    flags: int = 0
    voltage: float = 0.0     # V

    TYPE = MsgType.TLM_EVENT
    PAYLOAD_SIZE = 8

    def to_payload(self) -> bytes:
        return struct.pack(_TLM_EVENT_FMT, self.state, self.prev_state,
                           self.reason, self.flags, self.voltage)

    @classmethod
    def from_payload(cls, data: bytes) -> "TlmEvent":
        _check_len("TLM_EVENT", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_TLM_EVENT_FMT, data))


@dataclass
class LogText:
    """0x40 LOG_TEXT(1〜181B)— 人間向けメッセージ。生テキスト直書きの代替。"""
    origin: int = 0          # 0=relay, 1=drone
    text: str = ""           # UTF-8 で 180B 以下

    TYPE = MsgType.LOG_TEXT
    ORIGIN_RELAY = 0
    ORIGIN_DRONE = 1

    def to_payload(self) -> bytes:
        encoded = self.text.encode("utf-8")
        if len(encoded) > MAX_LOG_TEXT_SIZE:
            raise ValueError(f"LOG_TEXT: text too long ({len(encoded)}B > {MAX_LOG_TEXT_SIZE}B)")
        if not 0 <= self.origin <= 0xFF:
            raise ValueError(f"LOG_TEXT: origin out of range: {self.origin}")
        return bytes([self.origin]) + encoded

    @classmethod
    def from_payload(cls, data: bytes) -> "LogText":
        if not 1 <= len(data) <= 1 + MAX_LOG_TEXT_SIZE:
            raise ValueError(f"LOG_TEXT: invalid payload length {len(data)}")
        # 表示用途のため、不正な UTF-8 は U+FFFD に置換して受理する
        return cls(origin=data[0], text=bytes(data[1:]).decode("utf-8", errors="replace"))


def utf8_truncate_len(text: bytes, max_len: int) -> int:
    """UTF-8 文字境界を保ったまま text を max_len バイト以下に切り詰めた長さを返す。

    PROTOCOL.md LOG_TEXT: 多バイト文字を分断しない。切り詰め位置が多バイト文字の
    途中になる場合は、その文字の先頭(リード)バイトまで遡って文字ごと落とす。
    末尾がすでに不正な UTF-8 の場合は新たな分断を作らないことのみ保証する
    (C++ 実装 ``stampfly::utf8_truncate_len`` と test_vectors.json で一致を強制)。
    """
    cut = min(len(text), max_len)
    if cut <= 0:
        return 0
    # 末尾文字のリードバイトを探す(0b10xxxxxx = 継続バイトを遡る)
    lead = cut - 1
    while lead > 0 and (text[lead] & 0xC0) == 0x80:
        lead -= 1
    b = text[lead]
    if b < 0x80:                 # ASCII
        expect = 1
    elif (b & 0xE0) == 0xC0:     # 110xxxxx
        expect = 2
    elif (b & 0xF0) == 0xE0:     # 1110xxxx
        expect = 3
    elif (b & 0xF8) == 0xF0:     # 11110xxx
        expect = 4
    else:
        return cut               # 孤立継続バイト等の不正列: 分断回避の対象外
    return cut if lead + expect <= cut else lead


@dataclass
class RlySetTarget:
    """0x50 RLY_SET_TARGET(7B)— ESP-NOW ピア設定。"""
    mac: bytes = b"\x00" * 6
    wifi_channel: int = 0    # 1-13

    TYPE = MsgType.RLY_SET_TARGET
    PAYLOAD_SIZE = 7

    def to_payload(self) -> bytes:
        if len(self.mac) != 6:
            raise ValueError(f"RLY_SET_TARGET: mac must be 6 bytes, got {len(self.mac)}")
        return struct.pack(_RLY_SET_TARGET_FMT, bytes(self.mac), self.wifi_channel)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlySetTarget":
        _check_len("RLY_SET_TARGET", data, cls.PAYLOAD_SIZE)
        mac, channel = struct.unpack(_RLY_SET_TARGET_FMT, data)
        return cls(mac=mac, wifi_channel=channel)


@dataclass
class RlyTargetAck:
    """0x51 RLY_TARGET_ACK(8B)— SET_TARGET への応答。"""
    status: int = 0          # 0=ok, 1=invalid_mac, 2=peer_failed
    mac: bytes = b"\x00" * 6
    channel: int = 0

    TYPE = MsgType.RLY_TARGET_ACK
    PAYLOAD_SIZE = 8
    STATUS_OK = 0
    STATUS_INVALID_MAC = 1
    STATUS_PEER_FAILED = 2

    def to_payload(self) -> bytes:
        if len(self.mac) != 6:
            raise ValueError(f"RLY_TARGET_ACK: mac must be 6 bytes, got {len(self.mac)}")
        return struct.pack(_RLY_TARGET_ACK_FMT, self.status, bytes(self.mac), self.channel)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlyTargetAck":
        _check_len("RLY_TARGET_ACK", data, cls.PAYLOAD_SIZE)
        status, mac, channel = struct.unpack(_RLY_TARGET_ACK_FMT, data)
        return cls(status=status, mac=mac, channel=channel)


@dataclass
class RlyStats:
    """0x52 RLY_STATS(24B)— リレー統計(1Hz自動送信)。"""
    up_frames: int = 0
    down_frames: int = 0
    crc_errors: int = 0
    cobs_errors: int = 0
    espnow_send_fail: int = 0
    overflow_drops: int = 0

    TYPE = MsgType.RLY_STATS
    PAYLOAD_SIZE = 24

    def to_payload(self) -> bytes:
        return struct.pack(_RLY_STATS_FMT, self.up_frames, self.down_frames,
                           self.crc_errors, self.cobs_errors,
                           self.espnow_send_fail, self.overflow_drops)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlyStats":
        _check_len("RLY_STATS", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_RLY_STATS_FMT, data))


@dataclass
class RlyPong:
    """0x54 RLY_PONG(4B)— RLY_PING 応答(PING の seq をエコー)。"""
    echo_seq: int = 0

    TYPE = MsgType.RLY_PONG
    PAYLOAD_SIZE = 4

    def to_payload(self) -> bytes:
        return struct.pack(_RLY_PONG_FMT, self.echo_seq)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlyPong":
        _check_len("RLY_PONG", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_RLY_PONG_FMT, data))


# ペイロードを持たない型(len は常に 0)
EMPTY_PAYLOAD_TYPES = frozenset({
    MsgType.CMD_START, MsgType.CMD_STOP, MsgType.CMD_RESET, MsgType.RLY_PING,
})

# 型 -> ペイロードクラス(可変長 LOG_TEXT を含む)
PAYLOAD_CLASSES = {
    MsgType.CMD_SETPOINT: CmdSetpoint,
    MsgType.TLM_STATE: TlmState,
    MsgType.TLM_EVENT: TlmEvent,
    MsgType.LOG_TEXT: LogText,
    MsgType.RLY_SET_TARGET: RlySetTarget,
    MsgType.RLY_TARGET_ACK: RlyTargetAck,
    MsgType.RLY_STATS: RlyStats,
    MsgType.RLY_PONG: RlyPong,
}

# 型 -> 期待ペイロード長。None は可変長(LOG_TEXT: 1〜181B)。
# ドローン側受理規則「len==期待値」の判定に使用する。
EXPECTED_PAYLOAD_SIZE: dict[MsgType, Optional[int]] = {
    MsgType.CMD_START: 0,
    MsgType.CMD_STOP: 0,
    MsgType.CMD_SETPOINT: CmdSetpoint.PAYLOAD_SIZE,
    MsgType.CMD_RESET: 0,
    MsgType.TLM_STATE: TlmState.PAYLOAD_SIZE,
    MsgType.TLM_EVENT: TlmEvent.PAYLOAD_SIZE,
    MsgType.LOG_TEXT: None,
    MsgType.RLY_SET_TARGET: RlySetTarget.PAYLOAD_SIZE,
    MsgType.RLY_TARGET_ACK: RlyTargetAck.PAYLOAD_SIZE,
    MsgType.RLY_STATS: RlyStats.PAYLOAD_SIZE,
    MsgType.RLY_PING: 0,
    MsgType.RLY_PONG: RlyPong.PAYLOAD_SIZE,
}


def decode_payload(frame: Frame):
    """検証済みフレームのペイロードを型に応じた dataclass に展開する。

    ペイロードなしの型は None を返す。未知の型・長さ不整合は ValueError。
    """
    msg_type = MsgType(frame.type)   # 未知型は ValueError
    if msg_type in EMPTY_PAYLOAD_TYPES:
        if frame.payload:
            raise ValueError(f"{msg_type.name}: expected empty payload, got {len(frame.payload)}B")
        return None
    return PAYLOAD_CLASSES[msg_type].from_payload(frame.payload)


# ---------------------------------------------------------------------------
# 逐次シリアルレシーバ
# ---------------------------------------------------------------------------

@dataclass
class ReceiverCounters:
    """受信統計。フィールド名は C++ 側 SerialFrameReceiver::Counters と同一。"""
    frames_ok: int = 0
    cobs_errors: int = 0
    crc_errors: int = 0
    ver_errors: int = 0
    len_errors: int = 0
    overflow_drops: int = 0


class SerialFrameReceiver:
    """0x00 区切りのシリアルバイト列から論理フレームを取り出す逐次レシーバ。

    回復則(PROTOCOL.md「トランスポート」):

    - COBSデコード失敗 / CRC不一致 / ver不一致 / len不整合はフレームごと破棄
      (カウンタ加算のみ。部分回復しない)。
    - 蓄積が 256B を超えたら次の 0x00 まで読み捨て(overflow_drops 加算)。

    スレッド安全ではない。単一の読み取りスレッドから使用すること
    (共有が必要な場合は呼び出し側で lock を持つ)。
    """

    def __init__(self, on_frame: Optional[Callable[[Frame], None]] = None) -> None:
        self._buf = bytearray()
        self._dropping = False
        self._on_frame = on_frame
        self.counters = ReceiverCounters()

    def reset(self) -> None:
        """蓄積バッファと読み捨て状態をクリアする(カウンタは維持)。"""
        self._buf.clear()
        self._dropping = False

    def reset_counters(self) -> None:
        self.counters = ReceiverCounters()

    def feed(self, data: bytes) -> list[Frame]:
        """受信バイト列を供給し、完成した有効フレームのリストを返す。

        on_frame コールバックが設定されていればフレームごとに呼ばれる
        (コールバックとリスト返却のどちらでも消費できる)。
        """
        data = bytes(data)
        frames: list[Frame] = []
        pos = 0
        n = len(data)
        while pos < n:
            idx = data.find(COBS_DELIMITER, pos)
            seg_end = idx if idx >= 0 else n
            if seg_end > pos:
                # 非デリミタ区間の蓄積(バイト単位処理と等価なセグメント処理)
                if not self._dropping:
                    if len(self._buf) + (seg_end - pos) > SERIAL_RX_BUFFER_CAP:
                        # 上限超過 → 次の 0x00 まで読み捨て
                        self._buf.clear()
                        self._dropping = True
                        self.counters.overflow_drops += 1
                    else:
                        self._buf += data[pos:seg_end]
            if idx < 0:
                break
            pos = idx + 1
            # --- デリミタ到達: 蓄積分を1フレームとして処理 ---
            if self._dropping:
                self._dropping = False
                continue
            if not self._buf:
                continue   # 連続デリミタ / アイドル状態は無視
            raw = bytes(self._buf)
            self._buf.clear()
            try:
                decoded = cobs_decode(raw)
            except CobsDecodeError:
                self.counters.cobs_errors += 1
                continue
            status, frame = parse_frame(decoded)
            if status is ParseStatus.OK:
                assert frame is not None
                self.counters.frames_ok += 1
                frames.append(frame)
                if self._on_frame is not None:
                    self._on_frame(frame)
            elif status is ParseStatus.BAD_CRC:
                self.counters.crc_errors += 1
            elif status is ParseStatus.BAD_VER:
                self.counters.ver_errors += 1
            else:
                self.counters.len_errors += 1
        return frames
