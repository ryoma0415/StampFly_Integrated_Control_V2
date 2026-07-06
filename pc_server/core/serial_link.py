"""PC⇔リレーのシリアルリンク層。

責務(ARCHITECTURE.md):
- 読み取りスレッド + SerialFrameReceiver(COBSデコード・CRC検証は protocol 層)
- 型別ディスパッチ(ハンドラは RX スレッド上で呼ばれる。ブロッキング禁止)
- 書き込みロック(単一ライタ規律: UART への書き込みは send() のみ)
- RLY_TARGET_ACK 待ち合わせ(PROTOCOL.md: 1.0s 待ち、値一致まで最大3回再送
  = 初回送信+最大3回再送で計最大4回送信)
- CMD_SETPOINT seq → TLM_STATE seq_echo による往復レイテンシ計測
- 統計(受信カウンタ + 送信フレーム数)

pyserial の import は open 時まで遅延する(フェイク transport 注入時は不要)。
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Callable, Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由


class SerialLinkError(Exception):
    """シリアルリンクの送受信エラー。"""


class SerialLink:
    """シリアルポート1本ぶんのリンク管理。

    transport は pyserial Serial 互換の最小インターフェースであれば良い:
    ``read(size) -> bytes``(timeout付き)、``write(data)``、``close()``、
    任意で ``in_waiting``。テストではフェイクを注入する。
    """

    def __init__(self, server_config: dict,
                 transport_factory: Optional[Callable[[str, int], object]] = None,
                 on_disconnect: Optional[Callable[[str], None]] = None) -> None:
        serial_cfg = server_config["serial"]
        failsafe_cfg = server_config["failsafe"]
        fresh_cfg = server_config["freshness"]

        self._baudrate: int = serial_cfg["baudrate"]
        self._read_timeout_s: float = serial_cfg["read_timeout_s"]
        self._write_timeout_s: float = serial_cfg["write_timeout_s"]
        self._read_chunk: int = serial_cfg["read_chunk_bytes"]
        self._target_ack_timeout_s: float = failsafe_cfg["target_ack_timeout_s"]
        self._target_ack_max_retries: int = failsafe_cfg["target_ack_max_retries"]
        self._pending_max_age_s: float = fresh_cfg["latency_pending_max_age_s"]

        self._transport_factory = transport_factory or self._open_pyserial
        self._on_disconnect = on_disconnect

        self._transport: Optional[object] = None
        self._port: Optional[str] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # 送信系: 単一ライタ規律(seq 採番と write を同一ロックで保護)
        self._tx_lock = threading.Lock()
        self._tx_seq = 0          # 送信者ごとの単調増加カウンタ(1始まり)
        self._tx_frames = 0

        # 受信系
        self._receiver = proto.SerialFrameReceiver()
        self._handlers: dict[int, Callable[[proto.Frame], None]] = {}
        self._rx_dispatch_errors = 0   # ハンドラ例外の件数(RXスレッドのみ更新)

        # RLY_TARGET_ACK 待ち合わせ
        self._ack_lock = threading.Lock()
        self._ack_event = threading.Event()
        self._last_ack: Optional[proto.RlyTargetAck] = None

        # レイテンシ計測(CMD_SETPOINT seq -> 送信時刻 monotonic)
        self._latency_lock = threading.Lock()
        self._pending_setpoints: dict[int, float] = {}
        self._latency_ms: Optional[float] = None

        # リンク断通知の一回性保証(RX/TX スレッドが同時に失敗しても
        # コールバックを一度しか呼ばないための test-and-set 用ロック)
        self._disconnect_lock = threading.Lock()
        self._disconnect_reported = False

    # ------------------------------------------------------------------
    # 接続管理
    # ------------------------------------------------------------------

    def _open_pyserial(self, port: str, baudrate: int):
        # core/ を pyserial なしでも import 可能に保つため、ここで遅延 import
        import serial  # type: ignore
        # write_timeout は必ず設定する: pyserial 既定(None)だと USB CDC が
        # 固まったときに write が無期限ブロックし、_tx_lock 経由で 50Hz 送信と
        # フェイルセーフ送信(CMD_STOP)まで巻き込んで停止するため。
        # タイムアウト時の SerialTimeoutException は send() の except 節で
        # 捕捉され、リンク断として一度だけ通知される。
        return serial.Serial(port=port, baudrate=baudrate,
                             timeout=self._read_timeout_s,
                             write_timeout=self._write_timeout_s)

    def connect(self, port: str) -> None:
        """ポートを開き、読み取りスレッドを起動する。"""
        if self.is_connected:
            raise SerialLinkError("already connected")
        try:
            transport = self._transport_factory(port, self._baudrate)
        except Exception as exc:
            raise SerialLinkError(f"failed to open {port}: {exc}") from exc
        self._transport = transport
        self._port = port
        with self._disconnect_lock:
            self._disconnect_reported = False
        self._receiver.reset()
        self._receiver.reset_counters()
        self._rx_dispatch_errors = 0
        with self._latency_lock:
            self._pending_setpoints.clear()
            self._latency_ms = None
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="serial-reader", daemon=True)
        self._reader_thread.start()

    def disconnect(self) -> None:
        """読み取りスレッドを停止し、ポートを閉じる。"""
        self._stop_event.set()
        thread = self._reader_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._reader_thread = None
        transport = self._transport
        self._transport = None
        self._port = None
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass

    @property
    def is_connected(self) -> bool:
        return self._transport is not None

    @property
    def port(self) -> Optional[str]:
        return self._port

    # ------------------------------------------------------------------
    # 送信
    # ------------------------------------------------------------------

    def send(self, msg_type: int, payload: bytes = b"") -> int:
        """フレームを送信し、使用した seq を返す(唯一の UART 書き込み口)。"""
        transport = self._transport
        if transport is None:
            raise SerialLinkError("not connected")
        with self._tx_lock:
            # u32 ラップ(1始まり、0 は seq_echo 番兵のため飛ばす — PROTOCOL.md)
            self._tx_seq = (self._tx_seq % 0xFFFFFFFF) + 1
            seq = self._tx_seq
            wire = proto.encode_wire(msg_type, seq, payload)
            try:
                transport.write(wire)
            except Exception as exc:
                self._handle_link_failure(f"write failed: {exc}")
                raise SerialLinkError(f"write failed: {exc}") from exc
            self._tx_frames += 1
        return seq

    def send_setpoint(self, setpoint: proto.CmdSetpoint) -> int:
        """CMD_SETPOINT を送信し、レイテンシ計測用に seq を記録する。"""
        seq = self.send(proto.MsgType.CMD_SETPOINT, setpoint.to_payload())
        now = time.monotonic()
        with self._latency_lock:
            self._pending_setpoints[seq] = now
            # 古い未エコー分を破棄(無限成長防止)
            cutoff = now - self._pending_max_age_s
            stale = [s for s, t in self._pending_setpoints.items() if t < cutoff]
            for s in stale:
                del self._pending_setpoints[s]
        return seq

    def set_relay_target(self, mac: bytes, wifi_channel: int
                         ) -> tuple[bool, Optional[proto.RlyTargetAck]]:
        """RLY_SET_TARGET を送信し ACK を待つ。

        PROTOCOL.md: PC は 1.0s 待ち、値一致まで最大3回「再送」する。
        つまり初回送信+最大3回再送=計最大4回送信(CMD_STOP 再送と同じ解釈)。
        Returns: (成功フラグ, 最後に受信した ACK または None)。
        """
        request = proto.RlySetTarget(mac=bytes(mac), wifi_channel=wifi_channel)
        last_ack: Optional[proto.RlyTargetAck] = None
        for _attempt in range(1 + self._target_ack_max_retries):
            with self._ack_lock:
                self._last_ack = None
                self._ack_event.clear()
            self.send(proto.MsgType.RLY_SET_TARGET, request.to_payload())
            if not self._ack_event.wait(self._target_ack_timeout_s):
                continue   # タイムアウト → 再送
            with self._ack_lock:
                last_ack = self._last_ack
            if (last_ack is not None
                    and last_ack.status == proto.RlyTargetAck.STATUS_OK
                    and bytes(last_ack.mac) == bytes(mac)
                    and last_ack.channel == wifi_channel):
                return True, last_ack
        return False, last_ack

    # ------------------------------------------------------------------
    # 受信ディスパッチ
    # ------------------------------------------------------------------

    def register_handler(self, msg_type: int,
                         handler: Callable[[proto.Frame], None]) -> None:
        """型別ハンドラを登録する(RXスレッド上で呼ばれる。ブロッキング禁止)。"""
        self._handlers[int(msg_type)] = handler

    def _reader_loop(self) -> None:
        """読み取りスレッド本体。受信→フレーム化→ディスパッチのみを行う。"""
        while not self._stop_event.is_set():
            transport = self._transport
            if transport is None:
                break
            try:
                # 1バイトをタイムアウト付きで待ち、残りはまとめて読む(低レイテンシ)
                data = transport.read(1)
                if data:
                    waiting = int(getattr(transport, "in_waiting", 0) or 0)
                    if waiting > 0:
                        data += transport.read(min(waiting, self._read_chunk))
            except Exception as exc:
                self._handle_link_failure(f"read failed: {exc}")
                return
            if not data:
                continue
            try:
                for frame in self._receiver.feed(data):
                    try:
                        self._route_frame(frame)
                    except Exception:
                        # ハンドラ/パース例外で RX スレッドを黙って死なせない。
                        # 統計に計上して次フレームへ進む(RXスレッドでは
                        # print 等のブロッキング出力をしない規約のため)。
                        self._rx_dispatch_errors += 1
            except Exception as exc:
                # feed() 自体の失敗 = 受信経路の異常。リンク断として一度だけ
                # 通知し、supervisor にセッションを畳ませる。
                self._handle_link_failure(f"reader crashed: {exc}")
                return

    def _route_frame(self, frame: proto.Frame) -> None:
        if frame.type == proto.MsgType.RLY_TARGET_ACK:
            try:
                ack = proto.RlyTargetAck.from_payload(frame.payload)
            except ValueError:
                return
            with self._ack_lock:
                self._last_ack = ack
            self._ack_event.set()
        elif frame.type == proto.MsgType.TLM_STATE and len(frame.payload) >= 4:
            # seq_echo(先頭4B)から往復レイテンシを計測
            (seq_echo,) = struct.unpack_from("<I", frame.payload, 0)
            if seq_echo != 0:
                now = time.monotonic()
                with self._latency_lock:
                    sent_at = self._pending_setpoints.pop(seq_echo, None)
                    if sent_at is not None:
                        self._latency_ms = (now - sent_at) * 1000.0

        handler = self._handlers.get(frame.type)
        if handler is not None:
            handler(frame)

    def _handle_link_failure(self, reason: str) -> None:
        """リンク断を一度だけ通知する(RX/TX 双方から呼ばれうる)。"""
        with self._disconnect_lock:
            # check-then-set を同一ロック内で行い、RX/TX 同時失敗でも
            # on_disconnect が二重に呼ばれないことを保証する
            if self._disconnect_reported:
                return
            self._disconnect_reported = True
        self._stop_event.set()
        callback = self._on_disconnect
        if callback is not None:
            callback(reason)

    # ------------------------------------------------------------------
    # 統計
    # ------------------------------------------------------------------

    @property
    def latency_ms(self) -> Optional[float]:
        with self._latency_lock:
            return self._latency_ms

    def stats(self) -> dict:
        c = self._receiver.counters
        return {
            "tx_frames": self._tx_frames,
            "rx_frames_ok": c.frames_ok,
            "rx_cobs_errors": c.cobs_errors,
            "rx_crc_errors": c.crc_errors,
            "rx_ver_errors": c.ver_errors,
            "rx_len_errors": c.len_errors,
            "rx_overflow_drops": c.overflow_drops,
            "rx_dispatch_errors": self._rx_dispatch_errors,
        }
