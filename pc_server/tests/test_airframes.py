"""airframes: save/load 往復・update_airframes 検証・飛行ガード・バイアス再適用。"""

from __future__ import annotations

import json
import math

import pytest

import stampfly_protocol as proto
from core import config as cfg
from core.session import PHASE_FLYING

from conftest import TEST_AIRFRAMES, halt_supervisor


def make_profile(**overrides) -> dict:
    profile = {
        "name": "frame",
        "mac": "AA:BB:CC:DD:EE:10",
        "wifi_channel": 1,
        "roll_bias_deg": 0.0,
        "pitch_bias_deg": 0.0,
        "default_alt_m": 0.3,
        "notes": "",
    }
    profile.update(overrides)
    return profile


def copy_airframes(airframes=TEST_AIRFRAMES) -> list[dict]:
    return [dict(p) for p in airframes]


def drain_lines(session) -> list[str]:
    lines = []
    while True:
        try:
            message = session.events.get_nowait()
        except Exception:
            break
        if message.get("type") == "log":
            lines.append(message.get("line", ""))
    return lines


@pytest.fixture(autouse=True)
def airframes_path(tmp_path, monkeypatch):
    """テスト中の save_airframes が実 config を上書きしないよう退避する。"""
    path = tmp_path / "airframes.json"
    monkeypatch.setattr(cfg, "AIRFRAMES_CONFIG_PATH", path)
    return path


class TestSaveLoadRoundtrip:
    def test_roundtrip_preserves_content_and_key_order(self, airframes_path):
        profiles = [make_profile(name="a", notes="メモ 日本語"),
                    make_profile(name="b", mac="", roll_bias_deg=-0.573)]
        cfg.save_airframes(profiles)
        assert cfg.load_airframes() == profiles

        text = airframes_path.read_text(encoding="utf-8")
        assert "メモ 日本語" in text          # ensure_ascii=False
        data = json.loads(text)
        assert list(data["airframes"][0].keys()) == [
            "name", "mac", "wifi_channel", "roll_bias_deg",
            "pitch_bias_deg", "default_alt_m", "notes"]
        # 一時ファイルが残っていないこと(os.replace 済み)
        assert not airframes_path.with_name(airframes_path.name + ".tmp").exists()

    def test_mac_is_set(self):
        assert cfg.mac_is_set("AA:BB:CC:DD:EE:01")
        assert not cfg.mac_is_set("")
        assert not cfg.mac_is_set("   ")
        assert not cfg.mac_is_set(None)


class TestUpdateValidation:
    def test_rejects_non_list_and_empty(self, session_factory):
        session, _, _ = session_factory()
        ok, error = session.update_airframes(None)
        assert ok is False and "配列" in error
        ok, error = session.update_airframes([])
        assert ok is False and "1件以上" in error

    def test_rejects_duplicate_names(self, session_factory):
        session, _, _ = session_factory()
        ok, error = session.update_airframes(
            [make_profile(name="dup"), make_profile(name="dup",
                                                    mac="AA:BB:CC:DD:EE:11")])
        assert ok is False and "重複" in error

    def test_rejects_empty_name(self, session_factory):
        session, _, _ = session_factory()
        ok, error = session.update_airframes([make_profile(name="  ")])
        assert ok is False and "機体名" in error

    def test_rigid_body_id_optional_and_validated(self, session_factory):
        """rigid_body_id は任意(null/省略可)、指定時は 1 以上の整数のみ。"""
        session, _, _ = session_factory()
        ok, error = session.update_airframes([
            make_profile(name="a", rigid_body_id=3),
            make_profile(name="b", mac="AA:BB:CC:DD:EE:11",
                         rigid_body_id=None),
            make_profile(name="c", mac="AA:BB:CC:DD:EE:12"),   # キー省略
        ])
        assert ok is True, error
        by_name = {p["name"]: p for p in session.airframes}
        assert by_name["a"]["rigid_body_id"] == 3
        assert by_name["b"]["rigid_body_id"] is None
        assert by_name["c"]["rigid_body_id"] is None
        for bad in (0, -1, 1.5, "3", True):
            ok, error = session.update_airframes(
                [make_profile(rigid_body_id=bad)])
            assert ok is False and "rigid_body_id" in error, bad

    def test_rejects_bad_mac(self, session_factory):
        session, _, _ = session_factory()
        for bad in ("AA:BB:CC:DD:EE", "GG:BB:CC:DD:EE:FF", "12345"):
            ok, error = session.update_airframes([make_profile(mac=bad)])
            assert ok is False and "MAC" in error, bad

    def test_rejects_non_canonical_mac(self, session_factory):
        """2桁16進オクテット x6 以外("0x"接頭辞・空白混入・1桁)は拒否する。"""
        session, _, _ = session_factory()
        for bad in ("0x1:2:3:4:5:6", "1:2:3:4:5:6", "AA: BB:CC:DD:EE:FF",
                    "AA:BB:CC:DD:EE: F", "AA:BB:CC:DD:EE:+1"):
            ok, error = session.update_airframes([make_profile(mac=bad)])
            assert ok is False and "MAC" in error, bad

    def test_rejects_too_many_profiles(self, session_factory):
        session, _, _ = session_factory()
        limit = session.server_config["airframe_limits"]["max_profiles"]
        too_many = [make_profile(name=f"frame-{i}", mac="")
                    for i in range(limit + 1)]
        ok, error = session.update_airframes(too_many)
        assert ok is False and "最大" in error

    def test_accepts_exactly_max_profiles(self, session_factory):
        session, _, _ = session_factory()
        limit = session.server_config["airframe_limits"]["max_profiles"]
        at_limit = [make_profile(name=f"frame-{i}", mac="")
                    for i in range(limit)]
        ok, error = session.update_airframes(at_limit)
        assert ok is True and error is None

    def test_rejects_too_long_name(self, session_factory):
        session, _, _ = session_factory()
        limit = session.server_config["airframe_limits"]["name_max_chars"]
        ok, error = session.update_airframes(
            [make_profile(name="n" * (limit + 1))])
        assert ok is False and "機体名" in error and "文字以内" in error
        ok, error = session.update_airframes([make_profile(name="n" * limit)])
        assert ok is True and error is None

    def test_rejects_too_long_notes(self, session_factory):
        session, _, _ = session_factory()
        limit = session.server_config["airframe_limits"]["notes_max_chars"]
        ok, error = session.update_airframes(
            [make_profile(notes="メ" * (limit + 1))])
        assert ok is False and "notes" in error and "文字以内" in error
        ok, error = session.update_airframes([make_profile(notes="メ" * limit)])
        assert ok is True and error is None

    def test_accepts_unset_mac(self, session_factory):
        session, _, _ = session_factory()
        ok, error = session.update_airframes(
            copy_airframes() + [make_profile(name="new", mac="")])
        assert ok is True and error is None
        assert session.airframes[-1]["mac"] == cfg.MAC_UNSET

    def test_rejects_channel_out_of_range(self, session_factory):
        session, _, _ = session_factory()
        for bad in (0, 14, 1.5, "1", True):
            ok, error = session.update_airframes(
                [make_profile(wifi_channel=bad)])
            assert ok is False and "wifi_channel" in error, bad

    def test_rejects_bias_out_of_range(self, session_factory):
        session, _, _ = session_factory()
        limit = session.server_config["clamps"]["max_roll_pitch_deg"]
        ok, error = session.update_airframes(
            [make_profile(roll_bias_deg=limit + 0.1)])
        assert ok is False and "roll_bias_deg" in error
        ok, error = session.update_airframes(
            [make_profile(pitch_bias_deg=-(limit + 0.1))])
        assert ok is False and "pitch_bias_deg" in error

    def test_rejects_alt_out_of_range(self, session_factory):
        session, _, _ = session_factory()
        limits = session.server_config["airframe_limits"]
        for bad in (limits["default_alt_min_m"] - 0.01,
                    limits["default_alt_max_m"] + 0.01):
            ok, error = session.update_airframes(
                [make_profile(default_alt_m=bad)])
            assert ok is False and "default_alt_m" in error, bad

    def test_rejects_missing_and_unknown_keys(self, session_factory):
        session, _, _ = session_factory()
        incomplete = make_profile()
        del incomplete["notes"]
        ok, error = session.update_airframes([incomplete])
        assert ok is False and "notes" in error

        extra = make_profile(unexpected=1)
        ok, error = session.update_airframes([extra])
        assert ok is False and "不明なキー" in error

    def test_rejects_non_string_notes(self, session_factory):
        session, _, _ = session_factory()
        ok, error = session.update_airframes([make_profile(notes=123)])
        assert ok is False and "notes" in error

    def test_normalizes_mac_format(self, session_factory):
        session, _, _ = session_factory()
        ok, _ = session.update_airframes(
            [make_profile(mac="aa-bb-cc-dd-ee-1f")])
        assert ok is True
        assert session.airframes[0]["mac"] == "AA:BB:CC:DD:EE:1F"


class TestUpdatePersistAndRefresh:
    def test_update_persists_to_file(self, session_factory, airframes_path):
        session, _, _ = session_factory()
        new_list = copy_airframes()
        new_list[1]["notes"] = "更新テスト"
        ok, error = session.update_airframes(new_list)
        assert ok is True and error is None
        on_disk = json.loads(airframes_path.read_text(encoding="utf-8"))
        assert on_disk["airframes"][1]["notes"] == "更新テスト"
        assert session.airframes == on_disk["airframes"]

    def test_bias_reapplied_when_selected_and_not_flying(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")          # 選択中 = test-frame(bias 2.0 / -1.5)
        halt_supervisor(session)

        new_list = copy_airframes()
        new_list[0]["roll_bias_deg"] = 3.0
        new_list[0]["pitch_bias_deg"] = -0.5
        new_list[0]["default_alt_m"] = 0.6
        ok, _ = session.update_airframes(new_list)
        assert ok is True
        assert session._bias_roll_rad == pytest.approx(math.radians(3.0))
        assert session._bias_pitch_rad == pytest.approx(math.radians(-0.5))
        assert session.posture._target_alt == pytest.approx(0.6)
        # 選択中プロファイルの参照も新リストへ追従している
        assert session._airframe is session.airframes[0]

    def test_mac_change_does_not_resend_target_but_warns(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session._relay_target_ok is True
        transport.clear_sent()
        drain_lines(session)

        new_list = copy_airframes()
        new_list[0]["mac"] = "AA:BB:CC:DD:EE:99"
        ok, _ = session.update_airframes(new_list)
        assert ok is True
        # RLY_SET_TARGET は黙って再送されない(契約)
        assert transport.frames_of_type(proto.MsgType.RLY_SET_TARGET) == []
        assert any("選び直して" in line for line in drain_lines(session))
        # 次の select_airframe で新 MAC が送られる
        assert session.select_airframe("test-frame")
        frames = transport.frames_of_type(proto.MsgType.RLY_SET_TARGET)
        assert len(frames) == 1
        request = proto.RlySetTarget.from_payload(frames[0].payload)
        assert bytes(request.mac) == cfg.parse_mac("AA:BB:CC:DD:EE:99")

    def test_removed_selected_profile_clears_selection(self, session_factory):
        session, _, _ = session_factory()    # 未接続(idle)。選択中 = test-frame
        ok, _ = session.update_airframes(
            [dict(TEST_AIRFRAMES[1])])       # test-frame を削除
        assert ok is True
        assert session._airframe is None
        assert session._bias_roll_rad == 0.0
        assert session._bias_pitch_rad == 0.0


class TestFlyingGuard:
    def _make_flying(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        with session._lock:
            session._phase = PHASE_FLYING
        return session, transport

    def test_refuses_change_to_selected_profile_while_flying(self, session_factory):
        session, _ = self._make_flying(session_factory)
        new_list = copy_airframes()
        new_list[0]["roll_bias_deg"] = 5.0   # 選択中 test-frame を変更
        ok, error = session.update_airframes(new_list)
        assert ok is False and "飛行中" in error
        assert session.airframes == TEST_AIRFRAMES   # 反映されていない

    def test_refuses_removal_of_selected_profile_while_flying(self, session_factory):
        session, _ = self._make_flying(session_factory)
        ok, error = session.update_airframes([dict(TEST_AIRFRAMES[1])])
        assert ok is False and "飛行中" in error

    def test_accepts_unrelated_change_while_flying(self, session_factory):
        session, _ = self._make_flying(session_factory)
        new_list = copy_airframes()
        new_list[1]["notes"] = "飛行中でも他プロファイルは編集可"
        ok, error = session.update_airframes(new_list)
        assert ok is True and error is None
        # 選択中プロファイルのバイアスは維持される
        assert session._bias_roll_rad == pytest.approx(math.radians(2.0))


class TestUnsetMacSelection:
    UNSET_FIRST = [
        make_profile(name="no-mac", mac="",
                     roll_bias_deg=-0.573, pitch_bias_deg=-0.573),
        make_profile(name="with-mac", mac="AA:BB:CC:DD:EE:20"),
    ]

    def test_init_defaults_to_first_profile_with_mac(self, session_factory):
        session, _, _ = session_factory(airframes=copy_airframes(self.UNSET_FIRST))
        assert session._airframe is not None
        assert session._airframe["name"] == "with-mac"

    def test_init_with_no_usable_profile_boots_unselected(self, session_factory):
        session, _, _ = session_factory(
            airframes=[make_profile(name="no-mac", mac="")])
        assert session._airframe is None
        # connect は警告して失敗する(traceback なし)
        assert session.connect("FAKE") is False
        assert any("選択されていません" in line for line in drain_lines(session))

    def test_select_unset_mac_fails_disconnected(self, session_factory):
        session, _, _ = session_factory(airframes=copy_airframes(self.UNSET_FIRST))
        drain_lines(session)
        assert session.select_airframe("no-mac") is False
        assert any("MAC が未設定" in line for line in drain_lines(session))
        assert session._airframe["name"] == "with-mac"   # 選択は維持

    def test_select_unset_mac_fails_connected(self, session_factory):
        session, transport, _ = session_factory(
            airframes=copy_airframes(self.UNSET_FIRST))
        session.connect("FAKE")
        halt_supervisor(session)
        transport.clear_sent()
        drain_lines(session)
        assert session.select_airframe("no-mac") is False
        assert any("MAC が未設定" in line for line in drain_lines(session))
        # RLY_SET_TARGET は送られない
        assert transport.frames_of_type(proto.MsgType.RLY_SET_TARGET) == []

    def test_selected_profile_mac_unset_via_update_clears_selection(
            self, session_factory):
        session, _, _ = session_factory()
        drain_lines(session)
        new_list = copy_airframes()
        new_list[0]["mac"] = ""              # 選択中 test-frame の MAC を未設定に
        ok, _ = session.update_airframes(new_list)
        assert ok is True
        assert session._airframe is None
        assert any("MAC が未設定" in line for line in drain_lines(session))

    def test_start_refused_after_selected_profile_mac_unset_while_connected(
            self, session_factory):
        """接続中に選択中プロファイルの MAC を未設定化 → start は ARM しない。

        リレーのピア設定は残る(STOP は届く)が、プロファイル未選択のまま
        飛行を開始できてはならない。relay_target_ok も false へ落ち、UI の
        リレー表示が選び直しを促す。
        """
        session, transport, _ = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session._relay_target_ok is True

        new_list = copy_airframes()
        new_list[0]["mac"] = ""              # 選択中 test-frame の MAC を未設定に
        ok, _ = session.update_airframes(new_list)
        assert ok is True
        assert session._airframe is None
        assert session._relay_target_ok is False

        transport.clear_sent()
        drain_lines(session)
        assert session.start() is False
        assert transport.frames_of_type(proto.MsgType.CMD_START) == []
        assert any("選択されていません" in line for line in drain_lines(session))
