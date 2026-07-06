"""serial_link: フレーム送受信・型別ディスパッチ・ACK待ち・レイテンシ・統計。"""

from __future__ import annotations

import struct

import pytest

import stampfly_protocol as proto
from core.serial_link import SerialLink, SerialLinkError

from fakes import FakeTransport, make_ack_responder, wait_until

MAC = bytes.fromhex("48CA43389C88")


@pytest.fixture
def link_env(server_config):
    """接続済みの SerialLink + FakeTransport(テスト後に切断)。"""
    transport = FakeTransport()
    disconnects: list[str] = []
    link = SerialLink(server_config,
                      transport_factory=lambda port, baud: transport,
                      on_disconnect=disconnects.append)
    link.connect("FAKE")
    yield link, transport, disconnects
    link.disconnect()


def test_send_encodes_valid_wire_frame(link_env):
    link, transport, _ = link_env
    setpoint = proto.CmdSetpoint(roll_ref=0.0524, pitch_ref=-0.0349,
                                 alt_ref=0.30, flags=1)
    seq = link.send(proto.MsgType.CMD_SETPOINT, setpoint.to_payload())

    # ワイヤは 0x00 デリミタ終端
    assert transport.raw_written[-1] == 0x00
    # FakeTransport 側の protocol レシーバで復号できる=有効なワイヤフレーム
    assert len(transport.sent_frames) == 1
    frame = transport.sent_frames[0]
    assert frame.type == proto.MsgType.CMD_SETPOINT
    assert frame.seq == seq == 1   # seq は1始まり
    # float は f32 量子化を挟むため、復号後の == 比較ではなく
    # シリアライズ後バイト列の完全一致で内容を検証する(決定的)。
    assert frame.payload == setpoint.to_payload()


def test_seq_increments_per_send(link_env):
    link, transport, _ = link_env
    seqs = [link.send(proto.MsgType.CMD_START) for _ in range(3)]
    assert seqs == [1, 2, 3]


def test_dispatch_by_type(link_env):
    link, transport, _ = link_env
    received: list[proto.Frame] = []
    link.register_handler(proto.MsgType.TLM_EVENT, received.append)

    event = proto.TlmEvent(state=proto.FlightState.HOVER,
                           prev_state=proto.FlightState.TAKEOFF,
                           reason=proto.Reason.NONE, flags=0, voltage=3.8)
    transport.push(proto.MsgType.TLM_EVENT, event.to_payload(), seq=7)
    transport.push(proto.MsgType.RLY_PONG, proto.RlyPong(echo_seq=1).to_payload())

    assert wait_until(lambda: len(received) == 1)
    assert received[0].seq == 7
    # voltage(f32)の量子化があるためバイト列で一致を検証(決定的)
    assert received[0].payload == event.to_payload()
    # 未登録型(RLY_PONG)はディスパッチされないがエラーにもならない
    assert link.stats()["rx_frames_ok"] >= 2


def test_set_relay_target_success_first_attempt(server_config):
    transport = FakeTransport(auto_responder=make_ack_responder())
    link = SerialLink(server_config, transport_factory=lambda p, b: transport)
    link.connect("FAKE")
    try:
        ok, ack = link.set_relay_target(MAC, 1)
    finally:
        link.disconnect()
    assert ok is True
    assert ack is not None and ack.status == proto.RlyTargetAck.STATUS_OK
    assert bytes(ack.mac) == MAC
    assert len(transport.frames_of_type(proto.MsgType.RLY_SET_TARGET)) == 1


def test_set_relay_target_retries_three_times_on_timeout(fast_server_config):
    transport = FakeTransport()   # 応答なし
    link = SerialLink(fast_server_config, transport_factory=lambda p, b: transport)
    link.connect("FAKE")
    try:
        ok, ack = link.set_relay_target(MAC, 1)
    finally:
        link.disconnect()
    assert ok is False
    assert ack is None
    # PROTOCOL.md: 1.0s待ち、値一致まで最大3回「再送」= 初回+3回の計4送信
    assert len(transport.frames_of_type(proto.MsgType.RLY_SET_TARGET)) == 4


def test_set_relay_target_rejects_mismatched_ack(fast_server_config):
    # チャネルの一致しない ACK は不一致として再送される
    transport = FakeTransport(auto_responder=make_ack_responder(channel_override=9))
    link = SerialLink(fast_server_config, transport_factory=lambda p, b: transport)
    link.connect("FAKE")
    try:
        ok, ack = link.set_relay_target(MAC, 1)
    finally:
        link.disconnect()
    assert ok is False
    assert ack is not None and ack.channel == 9
    # 初回+最大3回再送 = 4送信(PROTOCOL.md RLY_TARGET_ACK)
    assert len(transport.frames_of_type(proto.MsgType.RLY_SET_TARGET)) == 4


def test_latency_measured_from_seq_echo(link_env):
    link, transport, _ = link_env
    assert link.latency_ms is None
    seq = link.send_setpoint(proto.CmdSetpoint(alt_ref=0.3, flags=1))

    tlm = proto.TlmState(seq_echo=seq, state=proto.FlightState.HOVER)
    transport.push(proto.MsgType.TLM_STATE, tlm.to_payload())

    assert wait_until(lambda: link.latency_ms is not None)
    assert 0.0 <= link.latency_ms < 1000.0


def test_latency_ignores_zero_seq_echo(link_env):
    link, transport, _ = link_env
    link.send_setpoint(proto.CmdSetpoint(alt_ref=0.3, flags=1))
    transport.push(proto.MsgType.TLM_STATE, proto.TlmState(seq_echo=0).to_payload())
    assert wait_until(lambda: link.stats()["rx_frames_ok"] >= 1)
    assert link.latency_ms is None


def test_receiver_recovers_after_garbage(link_env):
    link, transport, _ = link_env
    received: list[proto.Frame] = []
    link.register_handler(proto.MsgType.TLM_EVENT, received.append)

    # 破損バイト列(0x00終端)→ 有効フレーム の順に注入
    transport.push_raw(b"\x41\x41\x41\x00")
    transport.push(proto.MsgType.TLM_EVENT, proto.TlmEvent().to_payload())

    assert wait_until(lambda: len(received) == 1)
    stats = link.stats()
    assert stats["rx_frames_ok"] == 1
    # 破損分はいずれかのエラーカウンタに計上される(部分回復しない)
    assert (stats["rx_cobs_errors"] + stats["rx_crc_errors"]
            + stats["rx_len_errors"]) == 1


def test_write_failure_reports_disconnect_once(link_env):
    link, transport, disconnects = link_env
    transport.fail_writes = True
    with pytest.raises(SerialLinkError):
        link.send(proto.MsgType.CMD_STOP)
    with pytest.raises(SerialLinkError):
        link.send(proto.MsgType.CMD_STOP)
    assert len(disconnects) == 1


def test_send_when_not_connected_raises(server_config):
    link = SerialLink(server_config, transport_factory=lambda p, b: FakeTransport())
    with pytest.raises(SerialLinkError):
        link.send(proto.MsgType.CMD_START)
