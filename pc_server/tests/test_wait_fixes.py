"""WAIT 固定化バグの修正のテスト。

- 単機テレメトリ途絶の検出 + RLY_SET_TARGET 自動再設定(supervise)
- MocapSource.shutdown が vendor の join ハングに巻き込まれず有限時間で返る
- SessionManager.shutdown が _command_lock 保持中でも有限時間で強制終了する
"""

from __future__ import annotations

import threading
import time

import pytest
import stampfly_protocol as proto

import core.mocap as mocap_module
import core.session as session_module
from core.mocap import MocapSource
from conftest import halt_supervisor

from fakes import wait_until


def _count_set_target(transport) -> int:
    return sum(1 for f in transport.sent_frames
               if f.type == proto.MsgType.RLY_SET_TARGET)


def _drain_logs(session) -> list[str]:
    lines = []
    while not session.events.empty():
        event = session.events.get_nowait()
        if event.get("type") == "log":
            lines.append(event.get("line", ""))
    return lines


class TestTlmStalenessWatchdog:
    def test_stale_telemetry_triggers_warning_and_reassert(
            self, session_factory, server_config):
        session, transport, clock = session_factory()
        assert session.connect("COM-fake")
        halt_supervisor(session)
        session.posture.stop()
        baseline = _count_set_target(transport)
        assert baseline >= 1   # connect 時の初回設定
        _drain_logs(session)

        stale_s = server_config["failsafe"]["tlm_stale_s"]
        now = clock.advance(stale_s + 0.5)
        session.supervise(now)

        # 再設定は別スレッド(ACK 待ちあり)なので完了を待つ
        assert wait_until(lambda: _count_set_target(transport) > baseline)
        logs = "\n".join(_drain_logs(session))
        assert "テレメトリ" in logs and "途絶" in logs

    def test_reassert_rate_limited(self, session_factory, server_config):
        session, transport, clock = session_factory()
        assert session.connect("COM-fake")
        halt_supervisor(session)
        session.posture.stop()
        baseline = _count_set_target(transport)

        stale_s = server_config["failsafe"]["tlm_stale_s"]
        now = clock.advance(stale_s + 0.5)
        session.supervise(now)
        assert wait_until(lambda: _count_set_target(transport) == baseline + 1)

        # 間隔内の連続 supervise では再送しない
        session.supervise(clock.advance(0.1))
        session.supervise(clock.advance(0.1))
        time.sleep(0.1)
        assert _count_set_target(transport) == baseline + 1

    def test_fresh_telemetry_does_not_trigger(self, session_factory,
                                              server_config):
        session, transport, clock = session_factory()
        assert session.connect("COM-fake")
        halt_supervisor(session)
        session.posture.stop()
        baseline = _count_set_target(transport)

        # TLM_STATE を受信させて鮮度を回復させる代わりに、途絶閾値未満の
        # 経過で supervise を回す(connected_at 起点)
        stale_s = server_config["failsafe"]["tlm_stale_s"]
        session.supervise(clock.advance(stale_s * 0.5))
        time.sleep(0.05)
        assert _count_set_target(transport) == baseline


class _HangingNatNetClient:
    """shutdown の join が無期限化する vendor クライアントの模擬。"""

    def __init__(self) -> None:
        self.stop_threads = False
        self.release = threading.Event()
        self.shutdown_entered = threading.Event()
        self.data_port = 0        # ポート無効 → wake は no-op
        self.command_port = 0

    def set_server_address(self, address) -> None: ...
    def set_client_address(self, address) -> None: ...
    def set_use_multicast(self, enabled) -> None: ...
    def set_print_level(self, level) -> None: ...

    def run(self, thread_option) -> bool:
        return True

    def connected(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.shutdown_entered.set()
        self.release.wait(timeout=30.0)   # テスト終了時に解放される


class TestMocapShutdownBounded:
    def test_shutdown_returns_despite_hanging_vendor_join(
            self, control_config, monkeypatch):
        monkeypatch.setattr(mocap_module, "_SHUTDOWN_JOIN_TIMEOUT_S", 0.3)
        client = _HangingNatNetClient()
        source = MocapSource(control_config["natnet"],
                             control_config["coordinate_transform"],
                             client_factory=lambda: client)
        assert source.start()

        started = time.monotonic()
        source.shutdown()
        elapsed = time.monotonic() - started

        assert elapsed < 2.0   # vendor join を待ち続けない
        assert client.shutdown_entered.is_set()
        assert client.stop_threads is True   # 起きたスレッドが終了できる
        client.release.set()   # 看視スレッドの後始末

    def test_default_factory_daemonizes_vendor_threads(self):
        import NatNetClient as natnet_module  # vendor(conftest の shim 経由)
        MocapSource._default_client_factory()
        thread = natnet_module.Thread(target=lambda: None)
        assert thread.daemon is True


class TestSessionShutdownBounded:
    def test_shutdown_forces_teardown_when_lock_held(
            self, session_factory, monkeypatch):
        monkeypatch.setattr(session_module, "_SHUTDOWN_LOCK_TIMEOUT_S", 0.2)
        session, transport, _clock = session_factory()
        assert session.connect("COM-fake")

        # 別スレッドが _command_lock を長時間保持している状況を模擬
        holder_release = threading.Event()
        holder_acquired = threading.Event()

        def hold_lock() -> None:
            with session._command_lock:
                holder_acquired.set()
                holder_release.wait(timeout=30.0)

        holder = threading.Thread(target=hold_lock, daemon=True)
        holder.start()
        assert holder_acquired.wait(timeout=1.0)

        started = time.monotonic()
        session.shutdown()
        elapsed = time.monotonic() - started

        assert elapsed < 3.0            # ロック解放を無期限に待たない
        assert not session.serial.is_connected   # 強制経路でシリアルは閉じる
        holder_release.set()
        holder.join(timeout=1.0)
