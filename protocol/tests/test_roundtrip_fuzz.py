"""ランダム往復ファズテスト(シード固定で再現可能)。

全メッセージ型について encode -> decode の往復を検証する。
float は一度 f32 に丸めてから使い、ペイロードのバイト一致を主アサーションとする
(値比較は f32 丸めの曖昧さを含むため、バイト一致が本来の契約)。
"""

from __future__ import annotations

import random
import string
import struct

import pytest

import stampfly_protocol as sp

SEED = 0x5F4641  # 固定シード(再現性)
CASES_PER_TYPE = 60         # 33型 x 60 = 1980 ケース(>500)
STREAM_CASES = 50

# LOG_TEXT は UTF-8(PROTOCOL.md)。1B(ASCII)/2B/3B(日本語)/4B(非BMP)の
# 文字を混在させ、マルチバイトパスもファズ対象にする。
LOG_TEXT_ALPHABET = string.printable + "äöüßé高度機体到達中飛行🛸🚁"


def f32(value: float) -> float:
    """f32 に丸めた値を返す(往復の期待値を確定させる)。"""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def random_message(rng: random.Random, msg_type: sp.MsgType):
    """型に応じたランダムなペイロード dataclass(なし型は None)を返す。"""
    if msg_type in sp.EMPTY_PAYLOAD_TYPES:
        return None
    if msg_type == sp.MsgType.CMD_SETPOINT:
        return sp.CmdSetpoint(
            roll_ref=f32(rng.uniform(-0.6, 0.6)),
            pitch_ref=f32(rng.uniform(-0.6, 0.6)),
            alt_ref=f32(rng.uniform(0.0, 1.5)),
            yaw_ref=f32(rng.uniform(-3.15, 3.15)),
            flags=rng.randrange(256))
    if msg_type == sp.MsgType.CMD_POS_ERR:
        return sp.CmdPosErr(
            err_x=f32(rng.uniform(-2.0, 2.0)),
            err_y=f32(rng.uniform(-2.0, 2.0)),
            alt_ref=f32(rng.uniform(0.0, 1.5)),
            yaw_ref=f32(rng.uniform(-3.15, 3.15)),
            mocap_yaw=f32(rng.uniform(-3.15, 3.15)),
            flags=rng.randrange(256))
    if msg_type == sp.MsgType.CMD_MODE:
        return sp.CmdMode(mode=rng.randrange(2))
    if msg_type == sp.MsgType.CMD_MOTOR_RUN:
        return sp.CmdMotorRun(duty=f32(rng.uniform(0.0, 1.0)),
                              mask=rng.randrange(16))
    if msg_type == sp.MsgType.CMD_MAG3D_SET:
        return sp.CmdMag3dSet(
            valid=rng.randrange(2),
            offset=tuple(f32(rng.uniform(-100, 100)) for _ in range(3)),
            matrix=tuple(f32(rng.uniform(-2, 2)) for _ in range(9)))
    if msg_type == sp.MsgType.CMD_ACCEL6_SET:
        return sp.CmdAccel6Set(
            valid=rng.randrange(2),
            offset=tuple(f32(rng.uniform(-0.2, 0.2)) for _ in range(3)),
            scale=tuple(f32(rng.uniform(0.9, 1.1)) for _ in range(3)))
    if msg_type == sp.MsgType.CMD_ATTMOUNT_SET:
        return sp.CmdAttmountSet(valid=rng.randrange(2),
                                 roll_rad=f32(rng.uniform(-0.1, 0.1)),
                                 pitch_rad=f32(rng.uniform(-0.1, 0.1)))
    if msg_type == sp.MsgType.CMD_YAWZERO_SET:
        return sp.CmdYawzeroSet(valid=rng.randrange(2),
                                offset_rad=f32(rng.uniform(-3.15, 3.15)))
    if msg_type == sp.MsgType.CMD_GEOMAG_SET:
        return sp.CmdGeomagSet(
            declination_east_deg=f32(rng.uniform(-15, 15)),
            inclination_deg=f32(rng.uniform(30, 60)),
            horizontal_ut=f32(rng.uniform(20, 40)),
            vertical_ut=f32(rng.uniform(20, 45)),
            total_ut=f32(rng.uniform(40, 55)))
    if msg_type == sp.MsgType.CMD_FF_BEGIN:
        return sp.CmdFfBegin(nlut=rng.randrange(sp.CmdFfBegin.NLUT_MIN,
                                                sp.CmdFfBegin.NLUT_MAX + 1))
    if msg_type == sp.MsgType.CMD_FF_LUT:
        return sp.CmdFfLut(idx=rng.randrange(sp.CmdFfBegin.NLUT_MAX),
                           i_a=f32(rng.uniform(0, 5)),
                           db_x=f32(rng.uniform(-20, 20)),
                           db_y=f32(rng.uniform(-20, 20)),
                           db_z=f32(rng.uniform(-20, 20)))
    if msg_type == sp.MsgType.CMD_FF_MOT:
        return sp.CmdFfMot(idx=rng.randrange(4),
                           a_tilde=tuple(f32(rng.uniform(-2, 2)) for _ in range(3)),
                           c2=f32(rng.uniform(-2, 2)),
                           c1=f32(rng.uniform(-2, 2)),
                           c0=f32(rng.uniform(-0.5, 0.5)))
    if msg_type == sp.MsgType.CMD_FF_AUX:
        return sp.CmdFfAux(iid_a=f32(rng.uniform(0, 0.5)))
    if msg_type == sp.MsgType.CMD_FF_COMMIT:
        return sp.CmdFfCommit(crc32=rng.randrange(2**32))
    if msg_type == sp.MsgType.CMD_FF_MODE:
        return sp.CmdFfMode(ff_mode=rng.randrange(3), est_mode=rng.randrange(2))
    if msg_type == sp.MsgType.CMD_LED_MODE:
        return sp.CmdLedMode(mode=rng.randrange(2))
    if msg_type == sp.MsgType.TLM_STATE:
        return sp.TlmState(
            seq_echo=rng.randrange(2**32),
            elapsed_ms=rng.randrange(2**32),
            state=rng.randrange(8),
            flags=rng.randrange(8),
            reason=rng.randrange(12),
            roll=f32(rng.uniform(-3.2, 3.2)),
            pitch=f32(rng.uniform(-3.2, 3.2)),
            yaw=f32(rng.uniform(-3.2, 3.2)),
            p=f32(rng.uniform(-10, 10)),
            q=f32(rng.uniform(-10, 10)),
            r=f32(rng.uniform(-10, 10)),
            roll_ref=f32(rng.uniform(-0.6, 0.6)),
            pitch_ref=f32(rng.uniform(-0.6, 0.6)),
            alt_ref=f32(rng.uniform(0, 1.5)),
            altitude_tof=f32(rng.uniform(0, 2)),
            altitude_est=f32(rng.uniform(0, 2)),
            alt_velocity=f32(rng.uniform(-1, 1)),
            z_dot_ref=f32(rng.uniform(-1, 1)),
            voltage=f32(rng.uniform(3.0, 4.3)),
            duty_fr=f32(rng.uniform(0, 1)),
            duty_fl=f32(rng.uniform(0, 1)),
            duty_rr=f32(rng.uniform(0, 1)),
            duty_rl=f32(rng.uniform(0, 1)),
            ax=f32(rng.uniform(-2, 2)),
            ay=f32(rng.uniform(-2, 2)),
            az=f32(rng.uniform(-2, 2)),
            loop_dt_us=rng.randrange(2**16),
            yaw_est_rad=f32(rng.uniform(-3.2, 3.2)),
            yaw_gyro_int_rad=f32(rng.uniform(-30, 30)),
            yaw_ref_rad=f32(rng.uniform(-3.2, 3.2)),
            current_a=f32(rng.uniform(0, 8)),
            db_hat_x_ut=f32(rng.uniform(-20, 20)),
            db_hat_y_ut=f32(rng.uniform(-20, 20)),
            bm_x_ut=f32(rng.uniform(-5, 5)),
            bm_y_ut=f32(rng.uniform(-5, 5)),
            nis=f32(rng.uniform(0, 20)),
            ffg=rng.randrange(256),
            ff_status=rng.randrange(128))
    if msg_type == sp.MsgType.TLM_EVENT:
        return sp.TlmEvent(
            state=rng.randrange(8), prev_state=rng.randrange(8),
            reason=rng.randrange(12), flags=rng.randrange(256),
            voltage=f32(rng.uniform(3.0, 4.3)))
    if msg_type == sp.MsgType.TLM_ACK:
        return sp.TlmAck(acked_type=rng.randrange(0x14, 0x24),
                         acked_seq=rng.randrange(2**32),
                         status=rng.randrange(6))
    if msg_type == sp.MsgType.TLM_EXP:
        return sp.TlmExp(
            elapsed_ms=rng.randrange(2**32),
            current_a=f32(rng.uniform(0, 8)),
            vbat_v=f32(rng.uniform(3.0, 4.3)),
            shunt_uv=f32(rng.uniform(0, 5000)),
            bx_raw=f32(rng.uniform(-100, 100)),
            by_raw=f32(rng.uniform(-100, 100)),
            bz_raw=f32(rng.uniform(-100, 100)),
            bx_cal=f32(rng.uniform(-100, 100)),
            by_cal=f32(rng.uniform(-100, 100)),
            bz_cal=f32(rng.uniform(-100, 100)),
            imu_temp_c=f32(rng.uniform(10, 60)),
            roll=f32(rng.uniform(-3.2, 3.2)),
            pitch=f32(rng.uniform(-3.2, 3.2)),
            yaw=f32(rng.uniform(-3.2, 3.2)),
            p=f32(rng.uniform(-10, 10)),
            q=f32(rng.uniform(-10, 10)),
            r=f32(rng.uniform(-10, 10)),
            ax=f32(rng.uniform(-2, 2)),
            ay=f32(rng.uniform(-2, 2)),
            az=f32(rng.uniform(-2, 2)),
            duty_cmd=f32(rng.uniform(0, 1)),
            motors_mask=rng.randrange(16),
            flags=rng.randrange(8))
    if msg_type == sp.MsgType.TLM_CAL_DATA:
        return sp.TlmCalData(
            valid_flags=rng.randrange(64),
            mag3d_offset=tuple(f32(rng.uniform(-100, 100)) for _ in range(3)),
            mag3d_matrix=tuple(f32(rng.uniform(-2, 2)) for _ in range(9)),
            accel6_offset=tuple(f32(rng.uniform(-0.2, 0.2)) for _ in range(3)),
            accel6_scale=tuple(f32(rng.uniform(0.9, 1.1)) for _ in range(3)),
            attmount_roll_rad=f32(rng.uniform(-0.1, 0.1)),
            attmount_pitch_rad=f32(rng.uniform(-0.1, 0.1)),
            yawzero_offset_rad=f32(rng.uniform(-3.15, 3.15)),
            geomag=tuple(f32(rng.uniform(-15, 55)) for _ in range(5)),
            ff_nlut=rng.randrange(sp.CmdFfBegin.NLUT_MIN,
                                  sp.CmdFfBegin.NLUT_MAX + 1),
            ff_crc32=rng.randrange(2**32),
            ff_mode=rng.randrange(3),
            est_mode=rng.randrange(2))
    if msg_type == sp.MsgType.TLM_CTRL:
        return sp.TlmCtrl(
            elapsed_ms=rng.randrange(2**32),
            roll_rate_ref=f32(rng.uniform(-10, 10)),
            pitch_rate_ref=f32(rng.uniform(-10, 10)),
            yaw_rate_ref=f32(rng.uniform(-3.2, 3.2)),
            pid_ang=tuple(f32(rng.uniform(-10, 10)) for _ in range(9)),
            pid_rate=tuple(f32(rng.uniform(-2, 2)) for _ in range(9)),
            flags=rng.randrange(8))
    if msg_type == sp.MsgType.LOG_TEXT:
        # UTF-8 バイト数の上限(budget)内で文字単位に詰める(文字は分断しない)
        budget = rng.randrange(sp.MAX_LOG_TEXT_SIZE + 1)
        chars: list[str] = []
        size = 0
        while True:
            ch = rng.choice(LOG_TEXT_ALPHABET)
            ch_size = len(ch.encode("utf-8"))
            if size + ch_size > budget:
                break
            chars.append(ch)
            size += ch_size
        return sp.LogText(origin=rng.randrange(2), text="".join(chars))
    if msg_type == sp.MsgType.RLY_SET_TARGET:
        return sp.RlySetTarget(mac=bytes(rng.randrange(256) for _ in range(6)),
                               wifi_channel=rng.randrange(1, 14))
    if msg_type == sp.MsgType.RLY_TARGET_ACK:
        return sp.RlyTargetAck(status=rng.randrange(3),
                               mac=bytes(rng.randrange(256) for _ in range(6)),
                               channel=rng.randrange(1, 14))
    if msg_type == sp.MsgType.RLY_STATS:
        return sp.RlyStats(*(rng.randrange(2**32) for _ in range(6)))
    if msg_type == sp.MsgType.RLY_PONG:
        return sp.RlyPong(echo_seq=rng.randrange(2**32))
    if msg_type == sp.MsgType.RLY_SET_PEERS:
        count = rng.randrange(sp.RLY_MAX_PEERS + 1)
        return sp.RlySetPeers(
            wifi_channel=rng.randrange(1, 14) if count else 0,
            peers=tuple(
                sp.RlyPeer(mac=bytes(rng.randrange(256) for _ in range(6)),
                           tlm_state_div=rng.randrange(1, 5))
                for _ in range(count)))
    if msg_type == sp.MsgType.RLY_PEERS_ACK:
        return sp.RlyPeersAck(
            status=rng.randrange(5),
            count=rng.randrange(sp.RLY_MAX_PEERS + 1),
            wifi_channel=rng.randrange(1, 14),
            failed_index=rng.choice(
                [sp.RlyPeersAck.FAILED_NONE, rng.randrange(sp.RLY_MAX_PEERS)]))
    if msg_type in (sp.MsgType.RLY_MUX_UP, sp.MsgType.RLY_MUX_DOWN):
        # 内側フレームもランダム生成(上り/下りの代表型を包む)
        inner_type = rng.choice([sp.MsgType.CMD_SETPOINT, sp.MsgType.CMD_STOP,
                                 sp.MsgType.TLM_EVENT, sp.MsgType.TLM_STATE])
        inner_msg = random_message(rng, inner_type)
        inner_payload = b"" if inner_msg is None else inner_msg.to_payload()
        inner = sp.pack_frame(inner_type, rng.randrange(2**32), inner_payload)
        cls = (sp.RlyMuxUp if msg_type == sp.MsgType.RLY_MUX_UP
               else sp.RlyMuxDown)
        return cls(node_id=rng.randrange(sp.RLY_MAX_PEERS), inner=inner)
    raise AssertionError(f"unhandled type: {msg_type!r}")


def feed_in_random_chunks(rng: random.Random, rx: sp.SerialFrameReceiver,
                          data: bytes) -> list[sp.Frame]:
    """ワイヤバイト列をランダムな境界で分割してレシーバへ供給する。"""
    frames: list[sp.Frame] = []
    pos = 0
    while pos < len(data):
        size = rng.randrange(1, 17)
        frames += rx.feed(data[pos:pos + size])
        pos += size
    return frames


@pytest.mark.parametrize("msg_type", list(sp.MsgType), ids=lambda t: t.name)
def test_roundtrip_fuzz_per_type(msg_type):
    rng = random.Random(SEED ^ int(msg_type))
    for case in range(CASES_PER_TYPE):
        msg = random_message(rng, msg_type)
        payload = b"" if msg is None else msg.to_payload()
        seq = rng.randrange(1, 2**32)

        # 論理フレーム往復
        logical = sp.pack_frame(msg_type, seq, payload)
        status, frame = sp.parse_frame(logical)
        assert status is sp.ParseStatus.OK, case
        assert frame is not None
        assert (frame.type, frame.seq, frame.payload) == (msg_type, seq, payload)

        # COBS 往復(デリミタ以外に 0x00 が現れないことも確認)
        encoded = sp.cobs_encode(logical)
        assert 0 not in encoded
        assert sp.cobs_decode(encoded) == logical

        # ペイロード decode -> encode のバイト保存
        decoded = sp.decode_payload(frame)
        if msg is None:
            assert decoded is None
        else:
            assert decoded is not None
            assert decoded.to_payload() == payload
            assert decoded == msg  # f32 へ事前丸め済みなので値も一致する

        # レシーバ経由(ランダム分割供給)
        rx = sp.SerialFrameReceiver()
        got = feed_in_random_chunks(rng, rx, encoded + b"\x00")
        assert got == [frame]
        assert rx.counters.frames_ok == 1


def test_multi_frame_stream_fuzz():
    """複数フレーム連結ストリーム+余分なデリミタを順序通りに受信できること。"""
    rng = random.Random(SEED)
    types = list(sp.MsgType)
    for _ in range(STREAM_CASES):
        n_frames = rng.randrange(1, 9)
        expected: list[sp.Frame] = []
        stream = bytearray()
        for i in range(n_frames):
            msg_type = rng.choice(types)
            msg = random_message(rng, msg_type)
            payload = b"" if msg is None else msg.to_payload()
            seq = i + 1
            stream += sp.encode_wire(msg_type, seq, payload)
            if rng.random() < 0.3:
                stream += b"\x00" * rng.randrange(1, 4)  # アイドルデリミタ挿入
            expected.append(sp.Frame(type=int(msg_type), seq=seq, payload=payload))
        rx = sp.SerialFrameReceiver()
        got = feed_in_random_chunks(rng, rx, bytes(stream))
        assert got == expected
        assert rx.counters.frames_ok == n_frames
        assert rx.counters.cobs_errors == 0
        assert rx.counters.crc_errors == 0


def test_random_payload_bytes_roundtrip():
    """任意バイト列ペイロード(0〜200B)のフレーム往復(型に依存しない網羅)。"""
    rng = random.Random(SEED + 1)
    for _ in range(200):
        length = rng.randrange(sp.MAX_PAYLOAD_SIZE + 1)
        payload = bytes(rng.randrange(256) for _ in range(length))
        seq = rng.randrange(2**32)
        wire = sp.encode_wire(0x12, seq, payload)
        assert len(wire) <= sp.MAX_FRAME_SIZE + 3  # COBS+デリミタのオーバーヘッド上限
        rx = sp.SerialFrameReceiver()
        got = rx.feed(wire)
        assert len(got) == 1
        assert got[0].payload == payload
        assert got[0].seq == seq


def test_pack_frame_rejects_oversize_payload():
    with pytest.raises(ValueError):
        sp.pack_frame(0x12, 1, bytes(sp.MAX_PAYLOAD_SIZE + 1))


def test_pack_frame_rejects_bad_seq():
    with pytest.raises(ValueError):
        sp.pack_frame(0x12, 2**32, b"")
    with pytest.raises(ValueError):
        sp.pack_frame(0x12, -1, b"")


def test_log_text_rejects_oversize_text():
    with pytest.raises(ValueError):
        sp.LogText(origin=0, text="x" * (sp.MAX_LOG_TEXT_SIZE + 1)).to_payload()


def test_log_text_from_payload_replaces_split_multibyte():
    """多バイト文字が途中で切れたペイロードは U+FFFD 置換で受理されること。"""
    split = "高度".encode("utf-8")[:-1]  # 末尾の3バイト文字を分断
    msg = sp.LogText.from_payload(bytes([sp.LogText.ORIGIN_DRONE]) + split)
    assert msg.origin == sp.LogText.ORIGIN_DRONE
    assert msg.text == "高" + "�"  # U+FFFD REPLACEMENT CHARACTER


def test_utf8_truncate_len_preserves_char_boundaries():
    """utf8_truncate_len が全境界で文字を分断せず、削りすぎないこと。"""
    text = "a高🛸é: ログ切り詰め境界テスト".encode("utf-8")
    for max_len in range(len(text) + 1):
        cut = sp.utf8_truncate_len(text, max_len)
        assert cut <= max_len
        assert min(max_len, len(text)) - cut < 4   # 削るのは高々1文字(≦4B)
        text[:cut].decode("utf-8")                 # strict: 分断があれば例外


def test_utf8_truncate_len_edge_cases():
    assert sp.utf8_truncate_len(b"", 10) == 0
    assert sp.utf8_truncate_len(b"abc", 0) == 0
    assert sp.utf8_truncate_len(b"abc", 3) == 3           # 上限ちょうど
    assert sp.utf8_truncate_len(b"abc", 100) == 3         # 上限未満はそのまま
    # 不正な UTF-8(孤立継続バイト)は新たな分断を作らない範囲でそのまま通す
    assert sp.utf8_truncate_len(b"\x80\x80\x80", 2) == 2


def test_cobs_boundary_lengths():
    """COBS のブロック境界(253/254/255/508B)で両方向の往復が成立すること。"""
    rng = random.Random(SEED + 2)
    for length in (0, 1, 253, 254, 255, 507, 508, 509):
        for variant in ("nonzero", "zeros", "random"):
            if variant == "nonzero":
                data = bytes((i % 255) + 1 for i in range(length))
            elif variant == "zeros":
                data = bytes(length)
            else:
                data = bytes(rng.randrange(256) for _ in range(length))
            encoded = sp.cobs_encode(data)
            assert 0 not in encoded
            assert sp.cobs_decode(encoded) == data
