"""PC⇔リレーのシリアルリンク層。

責務(ARCHITECTURE.md):
- 読み取りスレッド + SerialFrameReceiver(COBSデコード・CRC検証は protocol 層)
- 型別ディスパッチ(ハンドラは RX スレッド上で呼ばれる。ブロッキング禁止)
- 書き込みロック(単一ライタ規律: UART への書き込みは send() のみ)
- RLY_TARGET_ACK 待ち合わせ(PROTOCOL.md: 1.0s 待ち、値一致まで最大3回再送
  = 初回送信+最大3回再送で計最大4回送信)
- TLM_ACK 待ち合わせ(v2: 0x14–0x23 コマンド用。acked_type + acked_seq の
  完全一致で対応付け、1.0s 待ち+最大2回再送 = RLY_SET_TARGET の前例踏襲)
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


# TLM_ACK 保持辞書の上限(構造定数。エコーされない古い ACK の掃除用で、
# チューニング値ではないため config には置かない)
_CMD_ACK_CACHE_MAX = 32


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
        self._command_ack_timeout_s: float = failsafe_cfg["command_ack_timeout_s"]
        self._command_ack_max_retries: int = failsafe_cfg["command_ack_max_retries"]
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

        # TLM_ACK 待ち合わせ(v2)。RX スレッドが (acked_type, acked_seq) を
        # キーに格納し、send_with_ack が Condition で待つ。辞書は肥大化を
        # 防ぐため上限を超えたら古いものから捨てる。
        self._cmd_ack_cond = threading.Condition()
        self._cmd_acks: dict[tuple[int, int], proto.TlmAck] = {}
        # send_with_ack 全体を直列化する(呼び出し元はキャリブ/FF 操作で
        # もともと排他だが、二重呼び出しでも ACK の取り違えを起こさないため)
        self._cmd_ack_session_lock = threading.Lock()

        # レイテンシ計測(CMD_SETPOINT seq -> 送信時刻 monotonic)
        self._latency_lock = threading.Lock()
        self._pending_setpoints: dict[int, float] = {}
        self._latency_ms: Optional[float] = None

        # マルチ機体(RLY_MUX_UP/DOWN)。ノード付きハンドラは
        # handler(node_id, frame) で RX スレッドから呼ばれる(ブロッキング禁止)。
        self._node_handlers: dict[int, Callable[[int, proto.Frame], None]] = {}
        self._rx_mux_errors = 0     # MUX_DOWN の内側フレーム不正(RXスレッドのみ更新)
        self._rx_mux_unhandled = 0  # どのハンドラにも配送できなかった内側フレーム
        # ノード別レイテンシ(_latency_lock で保護。node -> {seq: 送信時刻})
        self._node_pending: dict[int, dict[int, float]] = {}
        self._node_latency_ms: dict[int, float] = {}

        # RLY_PEERS_ACK 待ち合わせ(RLY_TARGET_ACK と同じ1スロット方式)
        self._peers_ack_lock = threading.Lock()
        self._peers_ack_event = threading.Event()
        self._last_peers_ack: Optional[proto.RlyPeersAck] = None

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
            self._node_pending.clear()
            self._node_latency_ms.clear()
        self._rx_mux_errors = 0
        self._rx_mux_unhandled = 0
        with self._cmd_ack_cond:
            self._cmd_acks.clear()
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
        self._record_pending_setpoint(seq)
        return seq

    def send_pos_err(self, pos_err: proto.CmdPosErr) -> int:
        """CMD_POS_ERR を送信し、レイテンシ計測用に seq を記録する。

        機上XY制御モードの 50Hz ストリーム(CMD_SETPOINT の代替)。機体は
        seq を TLM_STATE の seq_echo にエコーするため、往復レイテンシ計測は
        CMD_SETPOINT と同じ仕組みに載る。
        """
        seq = self.send(proto.MsgType.CMD_POS_ERR, pos_err.to_payload())
        self._record_pending_setpoint(seq)
        return seq

    def _record_pending_setpoint(self, seq: int) -> None:
        """seq_echo 往復レイテンシ計測のための送信時刻記録(50Hz ストリーム共通)。"""
        now = time.monotonic()
        with self._latency_lock:
            self._pending_setpoints[seq] = now
            # 古い未エコー分を破棄(無限成長防止)
            cutoff = now - self._pending_max_age_s
            stale = [s for s, t in self._pending_setpoints.items() if t < cutoff]
            for s in stale:
                del self._pending_setpoints[s]

    def send_to(self, node_id: int, msg_type: int, payload: bytes = b"") -> int:
        """フレームを RLY_MUX_UP で包んで peers[node_id] 宛に送信する。

        内側フレームと外側エンベロープは同じ seq を共有する(採番は1回。
        機体がエコーする seq_echo / acked_seq は内側 seq)。戻り値は seq。
        """
        transport = self._transport
        if transport is None:
            raise SerialLinkError("not connected")
        with self._tx_lock:
            # u32 ラップ(1始まり、0 は seq_echo 番兵のため飛ばす — PROTOCOL.md)
            self._tx_seq = (self._tx_seq % 0xFFFFFFFF) + 1
            seq = self._tx_seq
            inner = proto.pack_frame(msg_type, seq, payload)
            wire = proto.encode_wire(proto.MsgType.RLY_MUX_UP, seq,
                                     proto.mux_wrap(node_id, inner))
            try:
                transport.write(wire)
            except Exception as exc:
                self._handle_link_failure(f"write failed: {exc}")
                raise SerialLinkError(f"write failed: {exc}") from exc
            self._tx_frames += 1
        return seq

    def send_setpoint_to(self, node_id: int,
                         setpoint: proto.CmdSetpoint) -> int:
        """CMD_SETPOINT を node_id 宛に送信し、ノード別レイテンシ用に記録する。"""
        seq = self.send_to(node_id, proto.MsgType.CMD_SETPOINT,
                           setpoint.to_payload())
        self._record_node_pending(node_id, seq)
        return seq

    def send_pos_err_to(self, node_id: int, pos_err: proto.CmdPosErr) -> int:
        """CMD_POS_ERR を node_id 宛に送信し、ノード別レイテンシ用に記録する。"""
        seq = self.send_to(node_id, proto.MsgType.CMD_POS_ERR,
                           pos_err.to_payload())
        self._record_node_pending(node_id, seq)
        return seq

    def _record_node_pending(self, node_id: int, seq: int) -> None:
        """ノード別 seq_echo レイテンシ計測の送信時刻記録(50Hz ストリーム共通)。"""
        now = time.monotonic()
        with self._latency_lock:
            pending = self._node_pending.setdefault(node_id, {})
            pending[seq] = now
            # 古い未エコー分を破棄(無限成長防止)
            cutoff = now - self._pending_max_age_s
            stale = [s for s, t in pending.items() if t < cutoff]
            for s in stale:
                del pending[s]

    def send_with_ack(self, msg_type: int, payload: bytes = b"",
                      timeout_s: Optional[float] = None,
                      max_retries: Optional[int] = None
                      ) -> Optional[proto.TlmAck]:
        """フレームを送信し、対応する TLM_ACK を待つ(v2 コマンド用)。

        acked_type == msg_type かつ acked_seq == 送信 seq の TLM_ACK を
        timeout_s 待ち、届かなければ再送する(既定: 1.0s × 最大2回再送)。
        戻り値は受信した TlmAck(status の判定は呼び出し側)。全試行
        タイムアウトなら None。送信自体の失敗は SerialLinkError。
        """
        if timeout_s is None:
            timeout_s = self._command_ack_timeout_s
        if max_retries is None:
            max_retries = self._command_ack_max_retries
        with self._cmd_ack_session_lock:
            for _attempt in range(1 + max_retries):
                seq = self.send(msg_type, payload)
                key = (int(msg_type), seq)
                deadline = time.monotonic() + timeout_s
                with self._cmd_ack_cond:
                    while key not in self._cmd_acks:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            break
                        self._cmd_ack_cond.wait(remaining)
                    ack = self._cmd_acks.pop(key, None)
                if ack is not None:
                    return ack
            return None

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

    def send_with_ack_to(self, node_id: int, msg_type: int,
                         payload: bytes = b"",
                         timeout_s: Optional[float] = None,
                         max_retries: Optional[int] = None
                         ) -> Optional[proto.TlmAck]:
        """ノード宛にフレームを送信し、当該ノードからの TLM_ACK を待つ。

        send_with_ack のマルチ機体版。ACK の対応付けは
        (node_id, acked_type, acked_seq) の完全一致(再送規律も同一:
        既定 1.0s × 最大2回再送)。
        """
        if timeout_s is None:
            timeout_s = self._command_ack_timeout_s
        if max_retries is None:
            max_retries = self._command_ack_max_retries
        with self._cmd_ack_session_lock:
            for _attempt in range(1 + max_retries):
                seq = self.send_to(node_id, msg_type, payload)
                key = (int(node_id), int(msg_type), seq)
                deadline = time.monotonic() + timeout_s
                with self._cmd_ack_cond:
                    while key not in self._cmd_acks:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            break
                        self._cmd_ack_cond.wait(remaining)
                    ack = self._cmd_acks.pop(key, None)
                if ack is not None:
                    return ack
            return None

    def set_relay_peers(self, peers: list[tuple[bytes, int]], wifi_channel: int
                        ) -> tuple[bool, Optional[proto.RlyPeersAck]]:
        """RLY_SET_PEERS を送信し RLY_PEERS_ACK を待つ(マルチ機体モード)。

        peers は (mac, tlm_state_div) の並び(index = node_id)。空でマルチ
        モード解除。再送規律は RLY_SET_TARGET と同一(1.0s 待ち×最大3回再送)。
        Returns: (成功フラグ, 最後に受信した ACK または None)。
        """
        request = proto.RlySetPeers(
            wifi_channel=wifi_channel,
            peers=tuple(proto.RlyPeer(mac=bytes(mac), tlm_state_div=div)
                        for mac, div in peers))
        payload = request.to_payload()
        last_ack: Optional[proto.RlyPeersAck] = None
        for _attempt in range(1 + self._target_ack_max_retries):
            with self._peers_ack_lock:
                self._last_peers_ack = None
                self._peers_ack_event.clear()
            self.send(proto.MsgType.RLY_SET_PEERS, payload)
            if not self._peers_ack_event.wait(self._target_ack_timeout_s):
                continue   # タイムアウト → 再送
            with self._peers_ack_lock:
                last_ack = self._last_peers_ack
            if (last_ack is not None
                    and last_ack.status == proto.RlyPeersAck.STATUS_OK
                    and last_ack.count == len(peers)
                    and last_ack.wifi_channel == wifi_channel):
                return True, last_ack
        return False, last_ack

    # ------------------------------------------------------------------
    # 受信ディスパッチ
    # ------------------------------------------------------------------

    def register_handler(self, msg_type: int,
                         handler: Callable[[proto.Frame], None]) -> None:
        """型別ハンドラを登録する(RXスレッド上で呼ばれる。ブロッキング禁止)。"""
        self._handlers[int(msg_type)] = handler

    def register_node_handler(self, msg_type: int,
                              handler: Callable[[int, proto.Frame], None]
                              ) -> None:
        """RLY_MUX_DOWN の内側フレーム用ハンドラを登録する。

        handler(node_id, frame) が RX スレッド上で呼ばれる(ブロッキング禁止)。
        """
        self._node_handlers[int(msg_type)] = handler

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
        if frame.type == proto.MsgType.TLM_ACK:
            try:
                cmd_ack = proto.TlmAck.from_payload(frame.payload)
            except ValueError:
                return
            with self._cmd_ack_cond:
                if len(self._cmd_acks) >= _CMD_ACK_CACHE_MAX:
                    # 最古のエントリを捨てる(dict は挿入順を保持する)
                    self._cmd_acks.pop(next(iter(self._cmd_acks)))
                self._cmd_acks[(cmd_ack.acked_type, cmd_ack.acked_seq)] = cmd_ack
                self._cmd_ack_cond.notify_all()
            # 登録ハンドラ(あれば)にも届ける
        elif frame.type == proto.MsgType.RLY_TARGET_ACK:
            try:
                ack = proto.RlyTargetAck.from_payload(frame.payload)
            except ValueError:
                return
            with self._ack_lock:
                self._last_ack = ack
            self._ack_event.set()
        elif frame.type == proto.MsgType.RLY_PEERS_ACK:
            try:
                peers_ack = proto.RlyPeersAck.from_payload(frame.payload)
            except ValueError:
                return
            with self._peers_ack_lock:
                self._last_peers_ack = peers_ack
            self._peers_ack_event.set()
        elif frame.type == proto.MsgType.RLY_MUX_DOWN:
            self._route_mux_down(frame)
            return
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

    def _route_mux_down(self, frame: proto.Frame) -> None:
        """RLY_MUX_DOWN を展開し、内側フレームをノード付きで配送する。"""
        try:
            node_id, inner_bytes = proto.mux_unwrap(frame.payload)
        except ValueError:
            self._rx_mux_errors += 1
            return
        status, inner = proto.parse_frame(inner_bytes)
        if status is not proto.ParseStatus.OK or inner is None:
            self._rx_mux_errors += 1
            return
        if inner.type == proto.MsgType.TLM_ACK:
            # ノード宛コマンドの ACK 待ち合わせ(キーは (node, type, seq)。
            # 単機経路の (type, seq) キーとは長さが違うため衝突しない)
            try:
                cmd_ack = proto.TlmAck.from_payload(inner.payload)
            except ValueError:
                return
            with self._cmd_ack_cond:
                if len(self._cmd_acks) >= _CMD_ACK_CACHE_MAX:
                    self._cmd_acks.pop(next(iter(self._cmd_acks)))
                self._cmd_acks[(node_id, cmd_ack.acked_type,
                                cmd_ack.acked_seq)] = cmd_ack
                self._cmd_ack_cond.notify_all()
            return
        if inner.type == proto.MsgType.TLM_STATE and len(inner.payload) >= 4:
            # seq_echo(先頭4B)からノード別の往復レイテンシを計測
            (seq_echo,) = struct.unpack_from("<I", inner.payload, 0)
            if seq_echo != 0:
                now = time.monotonic()
                with self._latency_lock:
                    pending = self._node_pending.get(node_id)
                    sent_at = (pending.pop(seq_echo, None)
                               if pending is not None else None)
                    if sent_at is not None:
                        self._node_latency_ms[node_id] = \
                            (now - sent_at) * 1000.0
        handler = self._node_handlers.get(inner.type)
        if handler is not None:
            handler(node_id, inner)
            return
        # ノードハンドラ未登録の内側フレームは単機ハンドラへフォールバック
        # (LOG_TEXT / TLM_CAL_DATA など。ノード帰属は失われるが、機体発の
        # 診断情報を黙って捨てない)
        fallback = self._handlers.get(inner.type)
        if fallback is not None:
            fallback(inner)
        else:
            self._rx_mux_unhandled += 1

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

    def node_latency_ms(self, node_id: int) -> Optional[float]:
        """ノード別の CMD_SETPOINT 往復レイテンシ(未計測は None)。"""
        with self._latency_lock:
            return self._node_latency_ms.get(node_id)

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
            "rx_mux_errors": self._rx_mux_errors,
            "rx_mux_unhandled": self._rx_mux_unhandled,
        }
