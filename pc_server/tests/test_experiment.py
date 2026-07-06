"""v2 実験モード: CMD_MODE 遷移・モーターテスト・ACK 待ち・スイープ/シーケンス。"""

from __future__ import annotations

import threading
import time

import pytest

import stampfly_protocol as proto

import core.experiment as experiment
from core.session import MODE_EXPERIMENT, MODE_POSTURE

from conftest import halt_supervisor
from fakes import FakeDroneResponder, make_tlm_exp, wait_until


@pytest.fixture
def fast_ack_config(server_config):
    """ACK タイムアウト・キープアライブを短縮した設定(実時間テスト用)。"""
    server_config["failsafe"]["command_ack_timeout_s"] = 0.1
    server_config["experiment"]["motor_keepalive_s"] = 0.05
    return server_config


@pytest.fixture
def exp_session(fast_ack_config, session_factory):
    """実験モードに入った接続済みセッション(フェイク機体つき)。"""
    responder = FakeDroneResponder()
    session, transport, clock = session_factory(responder=responder)
    session.connect("FAKE")
    halt_supervisor(session)
    assert session.set_mode(MODE_EXPERIMENT)
    return session, transport, clock, responder


def start_exp_feeder(session, transport, **tlm_kwargs):
    """TLM_EXP を 5ms 周期で注入するフィーダスレッド(25Hz 相当以上)。

    最初のサンプルが hub に届くまで待ってから返す(スイープ開始の
    鮮度チェックがフレーク しないように)。
    """
    stop = threading.Event()

    def run():
        while not stop.is_set():
            transport.push(proto.MsgType.TLM_EXP,
                           make_tlm_exp(**tlm_kwargs).to_payload())
            time.sleep(0.005)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    assert wait_until(lambda: session.experiment.exp_age_s() is not None)
    return stop


class TestSendWithAck:
    def test_ack_roundtrip(self, exp_session):
        session, transport, clock, responder = exp_session
        ack = session.serial.send_with_ack(proto.MsgType.CMD_FF_ANCHOR)
        assert ack is not None
        assert ack.acked_type == proto.MsgType.CMD_FF_ANCHOR
        assert ack.status == proto.TlmAck.STATUS_OK

    def test_retry_after_lost_ack(self, exp_session):
        session, transport, clock, responder = exp_session
        responder.drop_first_ack_types.add(int(proto.MsgType.CMD_FF_ANCHOR))
        transport.clear_sent()
        ack = session.serial.send_with_ack(proto.MsgType.CMD_FF_ANCHOR)
        assert ack is not None and ack.status == proto.TlmAck.STATUS_OK
        # 初回 ACK ロスト → 1回再送で成功(計2フレーム)
        assert len(transport.frames_of_type(proto.MsgType.CMD_FF_ANCHOR)) == 2

    def test_timeout_returns_none(self, fast_ack_config, session_factory):
        # 既定レスポンダ(リレーのみ、機体なし)では TLM_ACK が来ない
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        transport.clear_sent()
        ack = session.serial.send_with_ack(proto.MsgType.CMD_FF_ANCHOR)
        assert ack is None
        # 初回+最大2回再送 = 3送信(契約 §3.4 の ACK 一般則)
        assert len(transport.frames_of_type(proto.MsgType.CMD_FF_ANCHOR)) == 3


class TestExperimentMode:
    def test_enter_sends_cmd_mode_and_stops_sender(self, exp_session):
        session, transport, clock, responder = exp_session
        frames = transport.frames_of_type(proto.MsgType.CMD_MODE)
        assert len(frames) == 1
        sent = proto.CmdMode.from_payload(frames[0].payload)
        assert sent.mode == proto.CmdMode.MODE_MOTOR_TEST
        assert session.experiment.active
        assert not session.posture.running   # 50Hz 送信は停止

    def test_enter_rejected_by_bad_state(self, fast_ack_config,
                                         session_factory):
        responder = FakeDroneResponder()
        responder.ack_status_overrides[int(proto.MsgType.CMD_MODE)] = \
            proto.TlmAck.STATUS_BAD_STATE
        session, transport, clock = session_factory(responder=responder)
        session.connect("FAKE")
        halt_supervisor(session)
        assert not session.set_mode(MODE_EXPERIMENT)
        assert session._mode == MODE_POSTURE     # 元モードに復帰
        assert session.posture.running           # 送信再開

    def test_start_rejected_in_experiment(self, exp_session):
        session, transport, clock, responder = exp_session
        assert not session.start()

    def test_exit_returns_to_flight_mode(self, exp_session):
        session, transport, clock, responder = exp_session
        transport.clear_sent()
        assert session.set_mode(MODE_POSTURE)
        frames = transport.frames_of_type(proto.MsgType.CMD_MODE)
        assert len(frames) == 1
        assert proto.CmdMode.from_payload(frames[0].payload).mode \
            == proto.CmdMode.MODE_FLIGHT
        assert not session.experiment.active
        assert session.posture.running

    def test_stop_deactivates_and_reactivate(self, exp_session):
        session, transport, clock, responder = exp_session
        session.experiment.motor_start(0.2, 0x0F)
        session.stop()   # SPACE 緊急停止相当(CMD_STOP で機体は WAIT へ)
        assert not session.experiment.motor_status()["running"]
        assert session._experiment_active is False
        # 再有効化で CMD_MODE(1) を送り直す
        transport.clear_sent()
        assert session.activate_experiment()
        assert session._experiment_active is True


class TestMotorTest:
    def test_motor_run_and_keepalive(self, exp_session):
        session, transport, clock, responder = exp_session
        transport.clear_sent()
        result = session.motor_start(0.3, 0x05)
        assert result["ok"]
        # キープアライブ(0.05s に短縮)による再送を確認
        assert wait_until(lambda: len(
            transport.frames_of_type(proto.MsgType.CMD_MOTOR_RUN)) >= 3)
        sent = proto.CmdMotorRun.from_payload(
            transport.frames_of_type(proto.MsgType.CMD_MOTOR_RUN)[0].payload)
        assert sent.duty == pytest.approx(0.3)
        assert sent.mask == 0x05

    def test_motor_stop_sends_three(self, exp_session):
        session, transport, clock, responder = exp_session
        session.motor_start(0.2, 0x0F)
        transport.clear_sent()
        result = session.motor_stop()
        assert result["ok"]
        assert len(transport.frames_of_type(proto.MsgType.CMD_MOTOR_STOP)) \
            == experiment.MOTOR_STOP_REPEAT

    def test_duty_clamped_to_max(self, exp_session):
        session, transport, clock, responder = exp_session
        result = session.motor_start(5.0, 0x0F)
        assert result["duty"] == pytest.approx(1.0)   # motor_max_duty

    def test_rejected_outside_experiment(self, fast_ack_config,
                                         session_factory):
        session, transport, clock = session_factory(
            responder=FakeDroneResponder())
        session.connect("FAKE")
        halt_supervisor(session)
        assert not session.motor_start(0.2, 0x0F)["ok"]


class TestTlmExpFlow:
    def test_latest_sample_updated(self, exp_session):
        session, transport, clock, responder = exp_session
        transport.push(proto.MsgType.TLM_EXP,
                       make_tlm_exp(current_a=1.25, vbat_v=3.85).to_payload())
        assert wait_until(
            lambda: session.experiment.latest_sample()[0] is not None)
        sample, age = session.experiment.latest_sample()
        assert sample["current_a"] == pytest.approx(1.25)
        assert sample["vbat_v"] == pytest.approx(3.85)
        assert sample["cv"] == 1

    def test_cal3d_collects_raw_field(self, exp_session):
        session, transport, clock, responder = exp_session
        session.experiment.cal3d_start()
        for _ in range(5):
            transport.push(proto.MsgType.TLM_EXP,
                           make_tlm_exp(b_raw=(1.0, 2.0, 3.0)).to_payload())
        assert wait_until(
            lambda: session.experiment.cal3d_status()["sample_count"] >= 5)
        samples = session.experiment.cal3d_stop()
        assert samples[0] == [1.0, 2.0, 3.0]


@pytest.fixture
def short_sweep(monkeypatch, tmp_path):
    """スイープのタイミング定数を短縮する(状態機械の検証用)。"""
    monkeypatch.setattr(experiment, "SWEEP_DUTY_STEPS", [0.2, 0.4])
    monkeypatch.setattr(experiment, "SWEEP_BASE_S", 0.06)
    monkeypatch.setattr(experiment, "SWEEP_SETTLE_S", 0.04)
    monkeypatch.setattr(experiment, "SWEEP_MEASURE_S", 0.06)
    monkeypatch.setattr(experiment, "SWEEP_GAP_S", 0.08)
    monkeypatch.setattr(experiment, "SWEEP_GAP_SETTLE_S", 0.02)
    monkeypatch.setattr(experiment, "SEQUENCE_COOLDOWN_S", 0.05)
    return tmp_path


class TestSweepRunner:
    def test_full_sweep_writes_csv_and_meta(self, exp_session, short_sweep):
        session, transport, clock, responder = exp_session
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport)
        try:
            result = session.sweep_start(0x0F, "up", {"location": "test"})
            assert result["ok"], result
            assert wait_until(
                lambda: not session.experiment.sweep.is_running(), timeout=10.0)
        finally:
            feeder.set()
        status = session.experiment.sweep.status()
        assert status["phase"] == "done", status
        last = status["last_result"]
        csv_path = short_sweep / last["samples"]
        meta_path = short_sweep / last["meta"]
        assert csv_path.is_file() and meta_path.is_file()

        import csv as csv_mod
        import json
        with csv_path.open() as fp:
            rows = list(csv_mod.DictReader(fp))
        assert list(rows[0].keys()) == experiment.SweepRunner.SAMPLE_FIELDS
        phases = {r["phase"] for r in rows}
        assert {"base", "settle", "measure", "gap_settle", "baseline"} <= phases
        measure = [r for r in rows if r["phase"] == "measure"]
        assert measure and all(r["dB_cor_x"] != "" for r in measure)

        meta = json.loads(meta_path.read_text())
        assert meta["schema"] == "stampfly_sweep_meta"
        assert meta["version"] == 1
        assert meta["method"] == "bracketed_baseline"
        assert meta["pattern"] == "up"
        assert meta["motors"] == "FL+FR+RL+RR"
        assert meta["idle_current_a"] is not None
        assert meta["sample_count"] == len(rows)

        # モーターは各 duty で駆動され、最後に停止している
        duties = {round(proto.CmdMotorRun.from_payload(f.payload).duty, 2)
                  for f in transport.frames_of_type(proto.MsgType.CMD_MOTOR_RUN)}
        assert {0.2, 0.4} <= duties
        assert transport.frames_of_type(proto.MsgType.CMD_MOTOR_STOP)

        # 受入条件: data_analysis が無改修でこの CSV/meta を読めること
        import sys
        from core import REPO_DIR
        da_dir = str(REPO_DIR / "data_analysis")
        if da_dir not in sys.path:
            sys.path.insert(0, da_dir)
        from ff_params import core as ff_core
        stem = last["samples"][:-len("_samples.csv")]
        run = ff_core.load_run(stem, short_sweep)
        agg = ff_core.aggregate(run)
        assert agg["idle"] == meta["idle_current_a"]
        assert {round(s["duty"], 2) for s in agg["steps"]} == {0.2, 0.4}

    def test_abort_stops_motors(self, exp_session, short_sweep, monkeypatch):
        session, transport, clock, responder = exp_session
        monkeypatch.setattr(experiment, "SWEEP_MEASURE_S", 5.0)
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport)
        try:
            assert session.sweep_start(0x01, "up", None)["ok"]
            assert wait_until(
                lambda: session.experiment.sweep.status()["phase"]
                in ("settle", "measure"), timeout=5.0)
            session.experiment.sweep.abort()
            assert wait_until(
                lambda: not session.experiment.sweep.is_running(), timeout=5.0)
        finally:
            feeder.set()
        assert session.experiment.sweep.status()["phase"] == "aborted"
        assert transport.frames_of_type(proto.MsgType.CMD_MOTOR_STOP)

    def test_manual_motor_rejected_while_running(self, exp_session,
                                                 short_sweep, monkeypatch):
        """スイープ実行中の motor_start / motor_set はサーバ側で拒否する。

        UI の busy ゲートだけに頼ると、別クライアントからの操作が measure 中の
        duty/mask を上書きし CSV の duty_cmd と実回転が食い違う。
        """
        session, transport, clock, responder = exp_session
        monkeypatch.setattr(experiment, "SWEEP_MEASURE_S", 5.0)
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport)
        try:
            assert session.sweep_start(0x01, "up", None)["ok"]
            assert wait_until(
                lambda: session.experiment.sweep.status()["phase"]
                in ("settle", "measure"), timeout=5.0)
            result = session.motor_start(0.9, 0x0F)
            assert result["ok"] is False
            assert "実行中" in result["message"]
            result = session.motor_apply(0.9)
            assert result["ok"] is False
            assert "実行中" in result["message"]
            # hub の duty/mask はスイープの値のまま(キープアライブが 0.9 を
            # 送らない)
            assert session.experiment.motor_status()["duty"] != 0.9
            # motor_stop(緊急停止経路)は従来どおり無条件で受理される
            assert session.motor_stop()["ok"] is True
            session.experiment.sweep.abort()
            assert wait_until(
                lambda: not session.experiment.sweep.is_running(), timeout=5.0)
        finally:
            feeder.set()

    def test_undervolt_aborts(self, exp_session, short_sweep):
        session, transport, clock, responder = exp_session
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport, vbat_v=2.9)   # < 3.0V
        try:
            assert session.sweep_start(0x0F, "up", None)["ok"]
            assert wait_until(
                lambda: not session.experiment.sweep.is_running(), timeout=5.0)
        finally:
            feeder.set()
        status = session.experiment.sweep.status()
        assert status["phase"] == "error"
        assert "過放電" in status["error"]


class TestSequenceRunner:
    def test_two_mask_sequence(self, exp_session, short_sweep):
        session, transport, clock, responder = exp_session
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport)   # vbat 3.9 ≥ 3.5(電池ガード通過)
        try:
            result = session.sequence_start([0x1, 0x2], "up", None, 3.5)
            assert result["ok"], result
            assert wait_until(
                lambda: not session.experiment.sequence.is_running(),
                timeout=15.0)
        finally:
            feeder.set()
        status = session.experiment.sequence.status()
        assert status["phase"] == "done", status
        assert len(status["results"]) == 2
        import json
        meta = json.loads((short_sweep / status["last_meta"]).read_text())
        assert meta["schema"] == "stampfly_sweep_sequence_meta"
        assert meta["completed"] is True
        assert [r["motors"] for r in meta["runs"]] == ["FL", "FR"]
