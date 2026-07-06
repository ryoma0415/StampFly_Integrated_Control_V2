"""pytest 共通設定: sys.path シムと共有フィクスチャ。

旧プロジェクト(NatNet_PID_Controller/tests)の「フェイク serial / NatNet で
ハードなしにロジックを検証する」パターンを踏襲する。
"""

from __future__ import annotations

import sys
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
PC_SERVER_DIR = TESTS_DIR.parent
if str(PC_SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(PC_SERVER_DIR))

import core  # noqa: E402,F401  (protocol/ と vendor/ の sys.path シム)

import pytest  # noqa: E402

from core import config as cfg  # noqa: E402
from core.session import SessionManager  # noqa: E402

from fakes import (  # noqa: E402
    FakeClock, FakeNatNetClient, FakeTransport, make_ack_responder,
)

# テスト用の機体プロファイル(バイアスあり)
TEST_AIRFRAMES = [
    {
        "name": "test-frame",
        "mac": "AA:BB:CC:DD:EE:01",
        "wifi_channel": 3,
        "roll_bias_deg": 2.0,
        "pitch_bias_deg": -1.5,
        "default_alt_m": 0.4,
        "notes": "unit test profile",
    },
    {
        "name": "zero-bias",
        "mac": "AA:BB:CC:DD:EE:02",
        "wifi_channel": 1,
        "roll_bias_deg": 0.0,
        "pitch_bias_deg": 0.0,
        "default_alt_m": 0.3,
        "notes": "unit test profile",
    },
]


@pytest.fixture
def server_config() -> dict:
    """server.json の新規コピー(テストごとに自由に書き換えてよい)。"""
    return cfg.load_server_config()


@pytest.fixture
def control_config() -> dict:
    """control.json の新規コピー。"""
    return cfg.load_control_config()


@pytest.fixture
def fast_server_config(server_config: dict) -> dict:
    """待ち時間を短縮した server 設定(タイムアウト経路のテスト用)。"""
    server_config["failsafe"]["target_ack_timeout_s"] = 0.05
    return server_config


def halt_supervisor(session: SessionManager) -> None:
    """監視スレッドを止め、supervise() を手動で決定的に呼べるようにする。"""
    session._supervisor_stop.set()
    thread = session._supervisor_thread
    if thread is not None:
        thread.join(timeout=1.0)
    session._supervisor_thread = None


@pytest.fixture
def session_factory(server_config, control_config):
    """FakeTransport / FakeNatNet / FakeClock を注入した SessionManager を作る。"""
    created: list[SessionManager] = []

    def factory(airframes=None, responder=None, clock=None):
        transport = FakeTransport(
            auto_responder=responder if responder is not None
            else make_ack_responder())
        clock = clock or FakeClock()
        session = SessionManager(
            server_config=server_config,
            control_config=control_config,
            airframes=airframes if airframes is not None else TEST_AIRFRAMES,
            transport_factory=lambda port, baud: transport,
            natnet_client_factory=FakeNatNetClient,
            clock=clock,
        )
        created.append(session)
        return session, transport, clock

    yield factory
    for session in created:
        session.disconnect()
