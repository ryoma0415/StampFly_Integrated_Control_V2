"""v2 FF プロファイル: 分割適用・CRC 照合・リトライ・binding 照合・状態永続化。"""

from __future__ import annotations

import json

import pytest

import stampfly_protocol as proto

from core.ffprofile import ff_crc32, ff_profile_wire_values
from core.session import MODE_EXPERIMENT

from conftest import halt_supervisor
from fakes import FakeDroneResponder

BINDING_OFFSET = [1.0, 2.0, 3.0]
BINDING_MATRIX = [[1.1, 0.0, 0.0], [0.0, 0.9, 0.0], [0.0, 0.0, 1.0]]


def make_ff_profile_dict(nlut: int = 4) -> dict:
    """最小の有効な stampfly_ff_profile v1 dict。"""
    points = [{"i_a": 0.2 + 0.5 * k, "db": [0.1 * k, -0.2 * k, 0.05 * k]}
              for k in range(nlut)]
    motors = {}
    duty = {}
    for i, name in enumerate(("FL", "FR", "RL", "RR")):
        motors[name] = [0.01 * i, -0.02 * i, 0.005 * i]
        duty[name] = {"c2": 1.0 + i, "c1": 0.5, "c0": 0.1}
    return {
        "schema": "stampfly_ff_profile",
        "version": 1,
        "name": "test_ff",
        "memo": "unit test",
        "created_at": "2026-07-06T00:00:00",
        "binding": {"mag3d": {"offset": list(BINDING_OFFSET),
                              "matrix": [list(r) for r in BINDING_MATRIX]}},
        "method_a": {"lut": {"i_idle_a": 0.16, "points": points}},
        "method_b": {"a_tilde": motors, "duty_to_current": duty},
        "quality": {"warnings": []},
    }


@pytest.fixture
def ff_session(server_config, session_factory, tmp_path):
    server_config["failsafe"]["command_ack_timeout_s"] = 0.1
    responder = FakeDroneResponder()
    # 機体の mag3d を binding と一致させる
    responder.cal_data.valid_flags = proto.TlmCalData.VALID_MAG3D
    responder.cal_data.mag3d_offset = tuple(BINDING_OFFSET)
    responder.cal_data.mag3d_matrix = tuple(
        v for row in BINDING_MATRIX for v in row)
    session, transport, clock = session_factory(responder=responder)
    session.connect("FAKE")
    halt_supervisor(session)
    assert session.set_mode(MODE_EXPERIMENT)
    mgr = session.ffprofile
    mgr.profile_dir = tmp_path / "ff_profiles"
    mgr.state_path = tmp_path / "ff_state.json"
    mgr.sweep_result_dir = tmp_path / "sweep_results"
    mgr.profile_dir.mkdir(parents=True)
    (mgr.profile_dir / "test_ff.json").write_text(
        json.dumps(make_ff_profile_dict()), encoding="utf-8")
    return session, transport, clock, responder, mgr


class TestWireValues:
    def test_crc_matches_manual(self):
        wire = ff_profile_wire_values(make_ff_profile_dict())
        assert len(wire["lut"]) == 4
        assert len(wire["mot"]) == 4
        assert wire["crc"] == ff_crc32(wire["lut"], wire["mot"], wire["iid"])

    def test_rejects_non_ascending_lut(self):
        profile = make_ff_profile_dict()
        profile["method_a"]["lut"]["points"][2]["i_a"] = 0.0
        with pytest.raises(ValueError):
            ff_profile_wire_values(profile)

    def test_rejects_wrong_schema(self):
        profile = make_ff_profile_dict()
        profile["schema"] = "other"
        with pytest.raises(ValueError):
            ff_profile_wire_values(profile)


class TestFfApply:
    def test_full_apply_sequence(self, ff_session):
        session, transport, clock, responder, mgr = ff_session
        transport.clear_sent()
        result = mgr.apply("test_ff", ff=2, est=1)
        assert result["ok"], result
        assert result["verified"] is True

        # 分割転送の順序と数(BEGIN → LUT×4 → MOT×4 → AUX → COMMIT → MODE)
        ff_types = [f.type for f in transport.sent_frames
                    if 0x1D <= f.type <= 0x23 or f.type == 0x22]
        assert ff_types.count(proto.MsgType.CMD_FF_BEGIN) == 1
        assert ff_types.count(proto.MsgType.CMD_FF_LUT) == 4
        assert ff_types.count(proto.MsgType.CMD_FF_MOT) == 4
        assert ff_types.count(proto.MsgType.CMD_FF_AUX) == 1
        assert ff_types.count(proto.MsgType.CMD_FF_COMMIT) == 1
        assert ff_types.count(proto.MsgType.CMD_FF_MODE) == 1
        assert ff_types.index(proto.MsgType.CMD_FF_BEGIN) == 0

        # CRC は wire 値と機体状態の双方に一致
        wire = ff_profile_wire_values(make_ff_profile_dict())
        assert responder.cal_data.ff_crc32 == wire["crc"]
        assert responder.cal_data.ff_mode == 2
        assert responder.cal_data.est_mode == 1

        # 適用状態の永続化(yaw側スキーマ: applied + verified、crc は hex8)
        state = json.loads(mgr.state_path.read_text(encoding="utf-8"))
        assert state["verified"] is True
        assert state["applied"]["name"] == "test_ff"
        assert state["applied"]["crc"] == f"{wire['crc']:08x}"
        assert state["applied"]["ff"] == 2
        assert state["applied"]["est"] == 1
        applied = mgr.status()["applied"]
        assert applied["name"] == "test_ff" and applied["verified"] is True

    def test_retry_on_lost_lut_ack(self, ff_session):
        session, transport, clock, responder, mgr = ff_session
        responder.drop_first_ack_types.add(int(proto.MsgType.CMD_FF_LUT))
        transport.clear_sent()
        result = mgr.apply("test_ff")
        assert result["ok"], result
        # 初回 LUT の ACK ロスト → 再送で 5 フレーム(4点+1再送)
        assert len(transport.frames_of_type(proto.MsgType.CMD_FF_LUT)) == 5

    def test_commit_ack_lost_recovered_by_readback(self, ff_session):
        session, transport, clock, responder, mgr = ff_session
        # commit は状態更新されるが ACK が一切返らない(全再送もロスト)
        responder.silent_types.add(int(proto.MsgType.CMD_FF_COMMIT))
        result = mgr.apply("test_ff")
        # CAL_GET 読み戻しの CRC 照合で救済され、適用成功になる(冪等契約)
        assert result["ok"], result
        assert result["verified"] is True

    def test_binding_mismatch_requires_force(self, ff_session):
        session, transport, clock, responder, mgr = ff_session
        responder.cal_data.mag3d_offset = (9.0, 9.0, 9.0)   # 取得時と不一致
        result = mgr.apply("test_ff")
        assert result["ok"] is False
        assert result.get("mag3d_mismatch") is True
        assert result["diffs"]
        # force で強制適用できる
        result = mgr.apply("test_ff", force=True)
        assert result["ok"], result

    def test_missing_profile(self, ff_session):
        session, transport, clock, responder, mgr = ff_session
        assert mgr.apply("no_such_profile")["ok"] is False


class TestFfModeAnchor:
    def test_mode_change_persists(self, ff_session):
        session, transport, clock, responder, mgr = ff_session
        assert mgr.apply("test_ff", ff=2, est=1)["ok"]
        result = mgr.mode(0, 0)
        assert result["ok"], result
        assert responder.cal_data.ff_mode == 0
        state = json.loads(mgr.state_path.read_text(encoding="utf-8"))
        assert state["applied"]["ff"] == 0
        assert state["applied"]["est"] == 0

    def test_anchor(self, ff_session):
        session, transport, clock, responder, mgr = ff_session
        result = mgr.anchor()
        assert result["ok"], result
        assert transport.frames_of_type(proto.MsgType.CMD_FF_ANCHOR)

    def test_anchor_busy_rejected(self, ff_session):
        session, transport, clock, responder, mgr = ff_session
        responder.ack_status_overrides[int(proto.MsgType.CMD_FF_ANCHOR)] = \
            proto.TlmAck.STATUS_BUSY
        result = mgr.anchor()
        assert result["ok"] is False
        assert "busy" in result["message"]


class TestFfDelete:
    def test_delete(self, ff_session):
        session, transport, clock, responder, mgr = ff_session
        assert mgr.delete("test_ff")["ok"]
        assert not (mgr.profile_dir / "test_ff.json").exists()
        assert mgr.status()["profiles"] == []
