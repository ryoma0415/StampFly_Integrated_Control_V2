"""テスト用フェイク(serial transport / NatNet client / clock)。"""

from __future__ import annotations

import threading
import time
import types
from typing import Callable, Optional

import stampfly_protocol as proto


def wait_until(predicate: Callable[[], bool], timeout: float = 2.0,
               interval: float = 0.005) -> bool:
    """predicate が True になるまで実時間でポーリングする。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


class FakeClock:
    """テストから明示的に進める monotonic クロック。"""

    def __init__(self, start: float = 100.0) -> None:
        self._t = start
        self._lock = threading.Lock()

    def __call__(self) -> float:
        with self._lock:
            return self._t

    def advance(self, dt: float) -> float:
        with self._lock:
            self._t += dt
            return self._t


class FakeTransport:
    """pyserial Serial 互換の最小フェイク。

    - PC が write したバイト列を protocol レシーバでフレーム化して記録する
    - auto_responder(frame) -> [(msg_type, payload), ...] | None で
      リレー側の応答(RLY_TARGET_ACK 等)を模擬できる
    - push()/push_raw() でリレー→PC 方向のバイト列を注入する
    """

    def __init__(self, auto_responder: Optional[Callable] = None) -> None:
        self._cond = threading.Condition()
        self._rx = bytearray()
        self.closed = False
        self.fail_writes = False
        self.auto_responder = auto_responder
        self.raw_written = bytearray()
        self.sent_frames: list[proto.Frame] = []
        self._tx_receiver = proto.SerialFrameReceiver()
        self._push_seq = 10000

    # --- pyserial 互換インターフェース ---

    def write(self, data: bytes) -> int:
        if self.fail_writes:
            raise OSError("simulated write failure")
        self.raw_written += data
        responses: list[tuple[int, bytes]] = []
        for frame in self._tx_receiver.feed(bytes(data)):
            self.sent_frames.append(frame)
            if self.auto_responder is not None:
                reply = self.auto_responder(frame)
                if reply:
                    responses.extend(reply)
        for msg_type, payload in responses:
            self.push(msg_type, payload)
        return len(data)

    def read(self, size: int) -> bytes:
        with self._cond:
            if not self._rx:
                self._cond.wait(timeout=0.01)
            n = min(size, len(self._rx))
            data = bytes(self._rx[:n])
            del self._rx[:n]
            return data

    @property
    def in_waiting(self) -> int:
        with self._cond:
            return len(self._rx)

    def close(self) -> None:
        self.closed = True

    # --- テストヘルパ ---

    def push(self, msg_type: int, payload: bytes = b"",
             seq: Optional[int] = None) -> None:
        """リレー→PC 方向に有効なワイヤフレームを注入する。"""
        if seq is None:
            self._push_seq += 1
            seq = self._push_seq
        self.push_raw(proto.encode_wire(msg_type, seq, payload))

    def push_raw(self, data: bytes) -> None:
        with self._cond:
            self._rx += data
            self._cond.notify_all()

    def frames_of_type(self, msg_type: int) -> list[proto.Frame]:
        return [f for f in self.sent_frames if f.type == msg_type]

    def clear_sent(self) -> None:
        self.sent_frames.clear()


def make_ack_responder(status: int = proto.RlyTargetAck.STATUS_OK,
                       channel_override: Optional[int] = None) -> Callable:
    """RLY_SET_TARGET に RLY_TARGET_ACK を返すレスポンダを作る。"""
    def responder(frame: proto.Frame):
        if frame.type != proto.MsgType.RLY_SET_TARGET:
            return None
        request = proto.RlySetTarget.from_payload(frame.payload)
        ack = proto.RlyTargetAck(
            status=status,
            mac=request.mac,
            channel=(channel_override if channel_override is not None
                     else request.wifi_channel),
        )
        return [(proto.MsgType.RLY_TARGET_ACK, ack.to_payload())]
    return responder


class FakeNatNetClient:
    """vendor/NatNetClient.py の最小フェイク(legacy テストのパターン)。"""

    instances: list["FakeNatNetClient"] = []

    def __init__(self) -> None:
        self.shutdown_called = False
        self.run_called = False
        self.new_frame_with_data_listener = None
        FakeNatNetClient.instances.append(self)

    def set_server_address(self, address) -> None:
        self.server_address = address

    def set_client_address(self, address) -> None:
        self.client_address = address

    def set_use_multicast(self, enabled) -> None:
        self.use_multicast = enabled

    def set_print_level(self, level) -> None:
        self.print_level = level

    def run(self, thread_option) -> bool:
        self.run_called = True
        return True

    def connected(self) -> bool:
        return self.run_called and not self.shutdown_called

    def shutdown(self) -> None:
        self.shutdown_called = True


def make_mocap_frame(rigid_body_id: int = 1,
                     pos: tuple = (0.0, 0.3, 0.0),
                     rot: tuple = (0.0, 0.0, 0.0, 1.0),
                     tracking_valid: bool = True,
                     error: float = 0.001,
                     marker_count: int = 4,
                     frame_number: int = 1) -> dict:
    """NatNet new_frame_with_data_listener 引数の模擬(Motive 座標)。"""
    rigid_body = types.SimpleNamespace(
        id_num=rigid_body_id,
        pos=pos,
        rot=rot,
        tracking_valid=tracking_valid,
        error=error,
        rb_marker_list=list(range(marker_count)),
    )
    mocap_data = types.SimpleNamespace(
        rigid_body_data=types.SimpleNamespace(rigid_body_list=[rigid_body]),
        labeled_marker_data=types.SimpleNamespace(labeled_marker_list=[]),
    )
    return {"mocap_data": mocap_data, "frame_number": frame_number}


def make_pose(x: float = 0.0, y: float = 0.0, z: float = 0.3,
              t: float = 0.0, yaw_rad: float = 0.0,
              tracking_valid: bool = True, error: float = 0.001,
              marker_count: int = 4, frame_number: int = 1) -> dict:
    """PositionController.on_mocap_pose に渡す制御座標系 pose dict。"""
    return {
        "t_mono": t,
        "frame_number": frame_number,
        "x": x,
        "y": y,
        "z": z,
        "yaw_rad": yaw_rad,
        "tracking_valid": tracking_valid,
        "error": error,
        "marker_count": marker_count,
        "quality": 1.0 / (1.0 + max(0.0, error)),
    }
