"""v2 キャリブレーション: 楕円体フィット・加速度6面・calプロファイル・地磁気。"""

from __future__ import annotations

import json
import math

import pytest

import stampfly_protocol as proto

from core.calibration import (
    ACCEL6_FACES, Accel6Calibrator, fit_ellipsoid, load_mag3d_correction,
)
from core.session import MODE_EXPERIMENT

from conftest import halt_supervisor
from fakes import FakeDroneResponder, make_tlm_exp, wait_until


def synthetic_ellipsoid_points(center=(5.0, -3.0, 10.0),
                               axes=(30.0, 40.0, 50.0), n=16):
    """軸整列楕円体表面のサンプル点(球面グリッド)。"""
    points = []
    for i in range(n):
        theta = math.pi * (i + 0.5) / n
        for j in range(2 * n):
            phi = 2.0 * math.pi * j / (2 * n)
            s = (math.sin(theta) * math.cos(phi),
                 math.sin(theta) * math.sin(phi),
                 math.cos(theta))
            points.append([center[a] + axes[a] * s[a] for a in range(3)])
    return points


class TestFitEllipsoid:
    def test_recovers_center_and_sphere(self):
        points = synthetic_ellipsoid_points()
        result = fit_ellipsoid(points)
        assert result["schema"] == 2
        assert result["offset"] == pytest.approx([5.0, -3.0, 10.0], abs=0.1)
        assert result["relative_rms_error"] < 0.01
        # 補正適用後は全点が同一半径の球面に乗る
        off = result["offset"]
        mat = result["matrix"]
        radii = []
        for p in points[:50]:
            v = [p[a] - off[a] for a in range(3)]
            c = [sum(mat[r][a] * v[a] for a in range(3)) for r in range(3)]
            radii.append(math.sqrt(sum(x * x for x in c)))
        mean_r = sum(radii) / len(radii)
        assert mean_r == pytest.approx(result["target_radius"], rel=0.02)
        assert max(radii) - min(radii) < 0.05 * mean_r

    def test_requires_min_samples(self):
        with pytest.raises(ValueError):
            fit_ellipsoid([[1.0, 2.0, 3.0]] * 10)


class TestAccel6Calibrator:
    FACE_VECTORS = {
        "x_pos": (1.02, 0.01, -0.02), "x_neg": (-0.98, 0.02, 0.01),
        "y_pos": (0.01, 1.03, 0.02), "y_neg": (-0.02, -0.97, 0.01),
        "z_pos": (0.02, -0.01, 1.01), "z_neg": (0.01, 0.02, -0.99),
    }

    def test_solve_offset_scale(self):
        cal = Accel6Calibrator()
        for face, vec in self.FACE_VECTORS.items():
            assert cal.capture_face(face, vec), cal.last_error
        solved = cal.solve()
        assert solved is not None, cal.last_error
        offset, scale = solved
        # offset = 対面平均、scale = 2/(対面差)(firmware と同一数式)
        assert offset[0] == pytest.approx((1.02 - 0.98) / 2)
        assert offset[1] == pytest.approx((1.03 - 0.97) / 2)
        assert offset[2] == pytest.approx((1.01 - 0.99) / 2)
        assert scale[0] == pytest.approx(2.0 / (1.02 + 0.98))
        assert scale[1] == pytest.approx(2.0 / (1.03 + 0.97))
        assert scale[2] == pytest.approx(2.0 / (1.01 + 0.99))

    def test_rejects_tilted_face(self):
        cal = Accel6Calibrator()
        assert not cal.capture_face("z_pos", (0.6, 0.0, 0.8))   # 他軸 > 0.55
        assert "傾き" in cal.last_error

    def test_rejects_wrong_direction(self):
        cal = Accel6Calibrator()
        assert not cal.capture_face("z_pos", (0.0, 0.0, -1.0))

    def test_solve_requires_all_faces(self):
        cal = Accel6Calibrator()
        assert cal.capture_face("z_pos", (0.0, 0.0, 1.0))
        assert cal.solve() is None


@pytest.fixture
def cal_session(server_config, session_factory, tmp_path):
    """実験モード+フェイク機体+一時ディレクトリのセッション。"""
    server_config["failsafe"]["command_ack_timeout_s"] = 0.1
    server_config["experiment"]["accel6_capture_s"] = 0.05
    responder = FakeDroneResponder()
    session, transport, clock = session_factory(responder=responder)
    session.connect("FAKE")
    halt_supervisor(session)
    assert session.set_mode(MODE_EXPERIMENT)
    session.calibration.calprofile_dir = tmp_path / "calibration_profiles"
    session.calibration.mag3d_path = tmp_path / "mag3d_calibration.json"
    return session, transport, clock, responder, tmp_path


class TestMag3dFlow:
    def test_fit_apply_clear(self, cal_session):
        session, transport, clock, responder, tmp_path = cal_session
        session.experiment.cal3d_start()
        for i, p in enumerate(synthetic_ellipsoid_points(n=10)):
            session.experiment.on_tlm_exp(make_tlm_exp(b_raw=tuple(p)), 1000 + i)
        status = session.calibration.mag3d_fit()
        assert status["error"] is None, status
        assert status["fit"]["sample_count"] >= 80
        assert session.calibration.mag3d_path.is_file()

        transport.clear_sent()
        result = session.calibration.mag3d_apply()
        assert result.get("ok"), result
        frames = transport.frames_of_type(proto.MsgType.CMD_MAG3D_SET)
        assert len(frames) == 1
        sent = proto.CmdMag3dSet.from_payload(frames[0].payload)
        assert sent.valid == 1
        # レスポンダの機体状態にも反映されている(read-back 整合)
        assert responder.cal_data.valid_flags & proto.TlmCalData.VALID_MAG3D
        assert load_mag3d_correction(session.calibration.mag3d_path) is not None

        result = session.calibration.mag3d_clear()
        assert result.get("ok"), result
        assert not session.calibration.mag3d_path.exists()
        assert not (responder.cal_data.valid_flags
                    & proto.TlmCalData.VALID_MAG3D)


class TestAccel6Flow:
    def test_capture_and_apply(self, cal_session):
        session, transport, clock, responder, tmp_path = cal_session
        session.calibration.accel6_start()
        vectors = TestAccel6Calibrator.FACE_VECTORS
        for face in ACCEL6_FACES:
            ax, ay, az = vectors[face]
            # キャプチャ窓の間 TLM_EXP を注入し続ける
            import threading
            stop = threading.Event()

            def feed():
                while not stop.is_set():
                    session.experiment.on_tlm_exp(
                        make_tlm_exp(ax=ax, ay=ay, az=az), 0)
                    stop.wait(0.005)

            thread = threading.Thread(target=feed, daemon=True)
            thread.start()
            try:
                result = session.calibration.accel6_capture(face)
            finally:
                stop.set()
                thread.join(timeout=1.0)
            assert result["ok"], result
        transport.clear_sent()
        result = session.calibration.accel6_apply()
        assert result["ok"], result
        sent = proto.CmdAccel6Set.from_payload(
            transport.frames_of_type(proto.MsgType.CMD_ACCEL6_SET)[0].payload)
        assert sent.valid == 1
        assert responder.cal_data.valid_flags & proto.TlmCalData.VALID_ACCEL6


class TestQuickCal:
    def test_attitude_and_yaw_zero(self, cal_session):
        session, transport, clock, responder, tmp_path = cal_session
        session.experiment.on_tlm_exp(
            make_tlm_exp(roll=0.02, pitch=-0.03, yaw=0.5), 1)
        assert session.calibration.attitude_zero()["ok"]
        sent = proto.CmdAttmountSet.from_payload(
            transport.frames_of_type(proto.MsgType.CMD_ATTMOUNT_SET)[0].payload)
        assert sent.roll_rad == pytest.approx(0.02)
        assert sent.pitch_rad == pytest.approx(-0.03)

        assert session.calibration.yaw_zero()["ok"]
        sent = proto.CmdYawzeroSet.from_payload(
            transport.frames_of_type(proto.MsgType.CMD_YAWZERO_SET)[0].payload)
        assert sent.offset_rad == pytest.approx(0.5)

        assert session.calibration.yaw_zero_clear()["ok"]
        assert not (responder.cal_data.valid_flags
                    & proto.TlmCalData.VALID_YAWZERO)


class TestGeomag:
    def test_status_lists_47_prefectures(self, cal_session):
        session, *_ = cal_session
        status = session.calibration.geomag_status()
        assert len(status["profiles"]) == 47
        assert "error" not in status["config"]

    def test_apply_sends_geomag_set(self, cal_session):
        session, transport, clock, responder, tmp_path = cal_session
        result = session.calibration.geomag_apply()
        assert result["ok"], result
        frames = transport.frames_of_type(proto.MsgType.CMD_GEOMAG_SET)
        assert len(frames) == 1
        sent = proto.CmdGeomagSet.from_payload(frames[0].payload)
        assert sent.total_ut > 0.0
        assert responder.cal_data.valid_flags & proto.TlmCalData.VALID_GEOMAG

    def test_select_persists_to_file(self, cal_session):
        session, transport, clock, responder, tmp_path = cal_session
        # 実ファイルを汚さないよう一時コピーへ切り替え
        source = session.calibration.geomag_path
        copy_path = tmp_path / "geomagnetic_profiles.json"
        copy_path.write_text(source.read_text(encoding="utf-8"),
                             encoding="utf-8")
        session.calibration.geomag_path = copy_path
        result = session.calibration.geomag_select("okinawa")
        assert result["ok"], result
        data = json.loads(copy_path.read_text(encoding="utf-8"))
        assert data["selected"] == "okinawa"


class TestCalProfile:
    def test_save_apply_verify_roundtrip(self, cal_session):
        session, transport, clock, responder, tmp_path = cal_session
        # 機体に既存キャリブがある状態を模擬
        responder.cal_data.valid_flags = (
            proto.TlmCalData.VALID_MAG3D | proto.TlmCalData.VALID_ACCEL6
            | proto.TlmCalData.VALID_YAWZERO)
        responder.cal_data.mag3d_offset = (1.0, 2.0, 3.0)
        responder.cal_data.mag3d_matrix = (1.1, 0.0, 0.0,
                                           0.0, 0.9, 0.0,
                                           0.0, 0.0, 1.0)
        responder.cal_data.accel6_offset = (0.01, -0.02, 0.03)
        responder.cal_data.accel6_scale = (1.01, 0.99, 1.0)
        responder.cal_data.yawzero_offset_rad = 0.25

        result = session.calibration.calprofile_save("test_profile")
        assert result["ok"], result
        profile_path = (session.calibration.calprofile_dir
                        / "test_profile.json")
        assert profile_path.is_file()
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
        assert profile["schema"] == "stampfly_calibration_profile"
        assert profile["version"] == 1
        assert profile["mag3d"]["valid"] is True
        assert profile["attitude_mount"]["valid"] is False

        # 機体状態を消してから適用 → 復元+読み戻し検証 OK
        responder.cal_data = proto.TlmCalData()
        transport.clear_sent()
        result = session.calibration.calprofile_apply("test_profile")
        assert result["ok"], result
        assert result["verified"] is True
        # 順序契約: accel6 → attmount → mag3d → yawzero
        cal_types = [f.type for f in transport.sent_frames
                     if f.type in (proto.MsgType.CMD_ACCEL6_SET,
                                   proto.MsgType.CMD_ATTMOUNT_SET,
                                   proto.MsgType.CMD_MAG3D_SET,
                                   proto.MsgType.CMD_YAWZERO_SET)]
        assert cal_types == [proto.MsgType.CMD_ACCEL6_SET,
                             proto.MsgType.CMD_ATTMOUNT_SET,
                             proto.MsgType.CMD_MAG3D_SET,
                             proto.MsgType.CMD_YAWZERO_SET]

    def test_apply_reports_mismatch(self, cal_session):
        session, transport, clock, responder, tmp_path = cal_session
        result = session.calibration.calprofile_save("p2")
        assert result["ok"]
        # 機体が YAWZERO の書き込みを黙って無視する(値が届かない)ケース
        responder.ack_status_overrides[int(proto.MsgType.CMD_YAWZERO_SET)] = \
            proto.TlmAck.STATUS_BAD_STATE
        profile_path = session.calibration.calprofile_dir / "p2.json"
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
        profile["yaw_zero"] = {"valid": True, "offset_rad": 1.0}
        profile_path.write_text(json.dumps(profile), encoding="utf-8")
        result = session.calibration.calprofile_apply("p2")
        assert result["ok"] is False

    def test_delete(self, cal_session):
        session, transport, clock, responder, tmp_path = cal_session
        assert session.calibration.calprofile_save("p3")["ok"]
        assert session.calibration.calprofile_delete("p3")["ok"]
        assert not (session.calibration.calprofile_dir / "p3.json").exists()
