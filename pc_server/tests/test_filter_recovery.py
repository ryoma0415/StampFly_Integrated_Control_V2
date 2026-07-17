"""フィルタ復帰(2026-07 位置固定化障害の対策)の検証。

障害シナリオ: 飛行中に OptiTrack がトラッキングを失うと、Motive は最後の
ポーズを凍結値のまま配信し続ける。旧実装はこれを正常受理してアンカーが
凍結点に固着し(推定速度も 0 に潰れる)、トラッキング復帰後の実位置が
動的閾値の外に出ると全サンプルを外れ値として永久拒否した(復帰経路なし・
飛行間リセットなし)。本ファイルは対策一式を検証する:

1. tracking_valid=0 サンプルの欠測扱い(アンカー汚染防止)
2. 連続外れ値 / 予測経過時間での強制再シード(filter.py)
3. START 時のフィルタリセットと data_valid ゲート(session.py)
4. データ無効持続のフェイルセーフ(supervise)
"""

from __future__ import annotations

import pytest

import stampfly_protocol as proto
from core.filter import PositionFilter
from core.session import MODE_POSITION, PHASE_ARMED, PHASE_FLYING

from conftest import halt_supervisor
from fakes import make_pose

FRAME_DT = 0.01   # mocap 100Hz 相当

# 「健全な信頼度」の基準値(旧 PC 側 PID の異常解除しきい値と同値。
# PID 経路の削除後もフィルタ復帰の劣化/回復判定の基準として使い続ける)
ANOMALY_CLEAR_CONFIDENCE = 0.5


def make_filter(control_config) -> PositionFilter:
    return PositionFilter.from_config(control_config["filter"])


def feed(f, pos, t, tracking_valid=True):
    return f.process_position(pos, marker_count=4, current_time=t,
                              tracking_valid=tracking_valid,
                              quality_weight=1.0, rigid_body_error=0.001)


class TestTrackingInvalidHandling:
    def test_frozen_untracked_frames_do_not_move_anchor(self, control_config):
        """tracking_valid=0 の凍結ポーズはアンカー・履歴を汚染しない。"""
        f = make_filter(control_config)
        conf_threshold = control_config["control"]["confidence_zero_threshold"]
        t = 0.0
        for _ in range(20):
            t += FRAME_DT
            feed(f, (0.10, 0.10, 0.30), t)
        anchor = f.last_valid_position
        anchor_t = f.last_valid_time
        # トラッキング喪失中(Motive は凍結値を配信し続ける)
        for _ in range(30):
            t += FRAME_DT
            result = feed(f, (0.10, 0.10, 0.30), t, tracking_valid=False)
            assert result["is_outlier"] is False
            assert result["confidence"] < conf_threshold
        assert f.last_valid_position == anchor   # アンカー不変
        assert f.last_valid_time == anchor_t     # 最終受理時刻のまま

    def test_first_sample_untracked_is_not_latched(self, control_config):
        """初回サンプルが tracking_valid=0 でもアンカーにしない。"""
        f = make_filter(control_config)
        result = feed(f, (2.0, 2.0, 2.0), 0.01, tracking_valid=False)
        assert result["filtered_position"] == (2.0, 2.0, 2.0)  # 生値パススルー
        assert result["is_outlier"] is False
        assert f.last_valid_position is None
        # 最初の tracking_valid フレームが初回受理になる
        result = feed(f, (0.0, 0.0, 0.3), 0.02)
        assert result["is_outlier"] is False
        assert result["confidence"] == 1.0


class TestReseedRecovery:
    def test_reseed_after_consecutive_outliers(self, control_config):
        """連続外れ値が上限に達したら生位置に強制再シードする(カウント経路)。"""
        limit = control_config["filter"]["max_consecutive_outliers"]
        f = make_filter(control_config)
        t = 0.0
        # dt=5ms: limit 到達時点でも予測経過 < max_prediction_s(0.5s)に
        # 収まり、カウント経路だけを検証できる
        dt = 0.005
        for _ in range(10):
            t += dt
            feed(f, (0.0, 0.0, 0.3), t)
        # 実位置がアンカーから max_outlier_threshold より遠くへ
        # (旧実装では幾何学的に復帰不能だった条件)
        results = []
        for _ in range(limit):
            t += dt
            results.append(feed(f, (1.0, 1.0, 1.0), t))
        assert all(r["is_outlier"] for r in results)
        assert results[-2]["consecutive_outliers"] == limit - 1
        # limit 発目で再シード: 生位置を新アンカーとして受け直す
        reseed = results[-1]
        assert reseed["consecutive_outliers"] == 0
        assert reseed["filtered_position"] == (1.0, 1.0, 1.0)
        assert reseed["used_prediction"] is False
        # 復帰フレーム自体は外れ値+低信頼度のまま(閉ループは即有効化されない)
        assert reseed["is_outlier"] is True
        assert reseed["confidence"] < ANOMALY_CLEAR_CONFIDENCE
        # 検疫期間: 受理・追従はするが probation=True+劣化信頼度のまま
        probation = control_config["filter"]["reseed_probation_frames"]
        for _ in range(probation):
            t += dt
            r = feed(f, (1.0, 1.0, 1.0), t)
            assert r["is_outlier"] is False
            assert r["probation"] is True
            assert r["confidence"] < ANOMALY_CLEAR_CONFIDENCE
        # 検疫明けから通常受理(閉ループ復帰可能)
        t += dt
        after = feed(f, (1.0, 1.0, 1.0), t)
        assert after["is_outlier"] is False
        assert after["probation"] is False
        assert after["confidence"] > ANOMALY_CLEAR_CONFIDENCE

    def test_reseed_after_prediction_age(self, control_config):
        """最終受理から max_prediction_s 超なら1フレームで再シードする。"""
        f = make_filter(control_config)
        feed(f, (0.0, 0.0, 0.3), 0.01)
        feed(f, (0.0, 0.0, 0.3), 0.02)
        result = feed(f, (1.0, 1.0, 1.0), 1.0)
        assert result["is_outlier"] is True
        assert result["consecutive_outliers"] == 0
        assert result["filtered_position"] == (1.0, 1.0, 1.0)
        # 単発フレームでの再シードは検疫つき(偽ソースへの即食い付き防止)
        assert result["probation"] is True

    def test_untracked_prediction_is_capped(self, control_config):
        """tracking_valid=0 中の予測外挿は max_prediction_s で頭打ちになる。

        推定速度が残ったまま長時間の喪失に入っても、ゴースト位置が
        滑走し続けず「最後の有効値付近で凍結」になることを確認する。
        """
        max_pred = control_config["filter"]["max_prediction_s"]
        f = make_filter(control_config)
        # +x 方向へ 0.5 m/s で移動しながら受理(推定速度を作る)
        t = 0.0
        x = 0.0
        for _ in range(20):
            t += FRAME_DT
            x += 0.5 * FRAME_DT
            feed(f, (x, 0.0, 0.3), t)
        last_x = f.last_valid_position[0]
        # 2 秒間トラッキング喪失(凍結ポーズ配信)
        result = None
        for _ in range(200):
            t += FRAME_DT
            result = feed(f, (x, 0.0, 0.3), t, tracking_valid=False)
        drift = result["filtered_position"][0] - last_x
        # 外挿は速度 × max_prediction_s まで(2秒ぶん滑走しない)
        assert abs(drift) <= 0.5 * max_pred + 0.05

    def test_isolated_glitch_still_rejected(self, control_config):
        """単発の外れ値は従来どおり拒否される(退行なし)。"""
        f = make_filter(control_config)
        t = 0.0
        for _ in range(10):
            t += FRAME_DT
            feed(f, (0.0, 0.0, 0.3), t)
        t += FRAME_DT
        glitch = feed(f, (1.0, 1.0, 1.0), t)
        assert glitch["is_outlier"] is True
        assert glitch["used_prediction"] is True
        assert glitch["filtered_position"][2] == pytest.approx(0.3, abs=0.05)
        t += FRAME_DT
        back = feed(f, (0.0, 0.0, 0.3), t)
        assert back["is_outlier"] is False
        assert back["consecutive_outliers"] == 0

    def test_incident_replay_recovers(self, control_config):
        """2026-07 障害の再現: ホバー中喪失→凍結配信→ジャンプ再捕捉→着陸。

        旧実装はこの列で永久ロックアウト(consecutive_outliers>10000・
        フィルタ出力が空中の凍結点に固定)に陥った。現行実装はトラッキング
        復帰後すみやかに実位置へ追従する。
        """
        f = make_filter(control_config)
        t = 0.0
        for _ in range(100):     # ホバー(z=0.51)
            t += FRAME_DT
            feed(f, (0.062, 0.082, 0.514), t)
        for _ in range(90):      # トラッキング喪失、凍結ポーズ配信(0.9s)
            t += FRAME_DT
            feed(f, (0.062, 0.082, 0.514), t, tracking_valid=False)
        # 再捕捉: 0.29m ジャンプ(実測ログ 20260713_212330 の値)
        t += FRAME_DT
        first = feed(f, (0.348, 0.079, 0.449), t)
        assert first["consecutive_outliers"] == 0   # 予測経過 >0.5s → 即再シード
        # 以後、降下・着陸まで実位置に追従する
        z = 0.449
        last = None
        for _ in range(50):
            t += FRAME_DT
            z = max(0.0, z - 0.01)
            last = feed(f, (0.348, 0.079, z), t)
        assert last["is_outlier"] is False
        assert last["confidence"] > ANOMALY_CLEAR_CONFIDENCE
        assert last["filtered_position"][2] == pytest.approx(z, abs=0.05)


class TestStartGateAndReset:
    def _connected_position(self, session_factory):
        session, transport, clock = session_factory()
        assert session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        return session, transport, clock

    def test_start_rejected_when_data_invalid(self, session_factory):
        """受信が新鮮でもフィルタ/トラッキング無効なら START を拒否する。"""
        session, transport, clock = self._connected_position(session_factory)
        session.position.on_mocap_pose(
            make_pose(t=clock(), tracking_valid=False))
        assert session.start() is False
        assert transport.frames_of_type(proto.MsgType.CMD_START) == []
        # 有効データ復帰で開始できる
        session.position.on_mocap_pose(make_pose(t=clock()))
        assert session.start() is True

    def test_start_reseeds_filter(self, session_factory):
        """START でフィルタを仕切り直す(前飛行のアンカーを持ち越さない)。"""
        session, transport, clock = self._connected_position(session_factory)
        # 前の飛行相当: 遠い位置にアンカーを作る
        for _ in range(5):
            session.position.on_mocap_pose(
                make_pose(x=1.0, y=1.0, z=0.5, t=clock.advance(FRAME_DT)))
        assert session.position.position_filter.last_valid_position is not None
        assert session.start() is True
        assert session.position.position_filter.last_valid_position is None
        # 1.2m 以上離れた実位置でも外れ値にならず即受理される
        session.position.on_mocap_pose(
            make_pose(x=0.0, y=0.0, z=0.0, t=clock()))
        assert session.position.data_valid() is True

    def test_snapshot_reports_validity(self, session_factory):
        """mocap_snapshot の valid が UI ランプの警告表示の根拠になる。"""
        session, transport, clock = self._connected_position(session_factory)
        session.position.on_mocap_pose(make_pose(t=clock()))
        assert session.position.mocap_snapshot(clock())["valid"] is True
        session.position.on_mocap_pose(
            make_pose(t=clock(), tracking_valid=False))
        assert session.position.mocap_snapshot(clock())["valid"] is False

    def test_circle_rejected_when_data_invalid(self, session_factory):
        """円軌道の開始も data_valid ゲートを通す(ゴースト位置から合流しない)。"""
        session, transport, clock = self._connected_position(session_factory)
        session.position.on_mocap_pose(
            make_pose(t=clock(), tracking_valid=False))
        ok, error = session.position.start_circle(
            0.0, 0.0, 0.5, 20.0, False, 0.3, False, now=clock())
        assert ok is False
        assert "無効" in error
        # 有効データ復帰で開始できる
        session.position.on_mocap_pose(make_pose(t=clock()))
        ok, error = session.position.start_circle(
            0.0, 0.0, 0.5, 20.0, False, 0.3, False, now=clock())
        assert ok is True


class TestDataInvalidFailsafe:
    def _armed_session(self, session_factory):
        session, transport, clock = session_factory()
        assert session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        session.position.on_mocap_pose(make_pose(t=clock()))
        assert session.start()
        assert session._phase == PHASE_ARMED
        session.position.stop()   # 50Hz スレッドを止めて決定的に駆動
        transport.clear_sent()
        return session, transport, clock

    def _feed_invalid(self, session, clock, duration_s):
        """新鮮だがトラッキング無効な pose を duration_s ぶん給送する。"""
        for _ in range(int(duration_s / FRAME_DT)):
            session.position.on_mocap_pose(
                make_pose(t=clock.advance(FRAME_DT), tracking_valid=False))

    def test_warns_after_persistent_invalid(self, session_factory):
        session, transport, clock = self._armed_session(session_factory)
        self._feed_invalid(session, clock, 0.6)   # data_invalid_warn_s=0.5
        session.supervise(clock())
        assert session._data_invalid_warned is True
        assert session._data_invalid_stop_sent is False
        assert transport.frames_of_type(proto.MsgType.CMD_STOP) == []

    def test_stops_after_persistent_invalid(self, session_factory):
        session, transport, clock = self._armed_session(session_factory)
        self._feed_invalid(session, clock, 2.2)   # data_invalid_stop_s=2.0
        session.supervise(clock())
        assert len(transport.frames_of_type(proto.MsgType.CMD_STOP)) == 1
        # 同一エピソード内で二重送信しない
        self._feed_invalid(session, clock, 0.1)
        session.supervise(clock())
        assert len(transport.frames_of_type(proto.MsgType.CMD_STOP)) == 1

    def test_recovery_clears_invalid_flags(self, session_factory):
        session, transport, clock = self._armed_session(session_factory)
        self._feed_invalid(session, clock, 0.6)
        session.supervise(clock())
        assert session._data_invalid_warned is True
        session.position.on_mocap_pose(make_pose(t=clock()))   # 有効復帰
        session.supervise(clock())
        assert session._data_invalid_warned is False

    def test_not_triggered_during_dropout(self, session_factory):
        """完全途絶は途絶側のポリシーに委ねる(二重発報しない)。"""
        session, transport, clock = self._armed_session(session_factory)
        clock.advance(0.6)   # ポーズ自体が来ない
        session.supervise(clock())
        assert session._mocap_warned is True
        assert session._data_invalid_warned is False

    def test_active_when_flying_without_start(self, session_factory):
        """START を経ない flying 昇格(飛行中の再接続)でも監視が働く。

        無効フレームの到着時点から計時するため、reset_filter を通らなくても
        持続的データ無効で自動着陸に至る。
        """
        session, transport, clock = session_factory()
        assert session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        # 機体は既に飛行中(connected → flying 昇格)
        session._update_phase_from_drone(
            int(proto.FlightState.HOVER), proto.TlmState.FLAG_FLYING)
        assert session._phase == PHASE_FLYING
        session.position.stop()
        transport.clear_sent()
        self._feed_invalid(session, clock, 2.2)
        session.supervise(clock())
        assert len(transport.frames_of_type(proto.MsgType.CMD_STOP)) == 1


class TestDivergenceFailsafe:
    """単機の XY 発散検知(multi の divergence 検知と同じ規範)。

    偽ソース(RB 取り違え・反射)への再シードでフィルタ上は「有効」なまま
    閉ループが幻の誤差を追い続ける事故の最終防衛線。
    """

    def _flying_session(self, session_factory):
        session, transport, clock = session_factory()
        assert session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        session.position.on_mocap_pose(make_pose(t=clock()))
        assert session.start()
        session._update_phase_from_drone(
            int(proto.FlightState.HOVER), proto.TlmState.FLAG_FLYING)
        assert session._phase == PHASE_FLYING
        session.position.stop()
        transport.clear_sent()
        return session, transport, clock

    def _feed_valid(self, session, clock, duration_s, pos=(0.0, 0.0, 0.3)):
        for _ in range(int(duration_s / FRAME_DT)):
            session.position.on_mocap_pose(
                make_pose(*pos, t=clock.advance(FRAME_DT)))

    def test_sustained_divergence_stops(self, session_factory):
        session, transport, clock = self._flying_session(session_factory)
        session.position.set_target(2.0, 0.0, 0.3)   # 実位置(原点)と 2m 差
        self._feed_valid(session, clock, 0.1)
        session.supervise(clock())                    # 計時開始
        assert transport.frames_of_type(proto.MsgType.CMD_STOP) == []
        self._feed_valid(session, clock, 1.1)         # divergence_hold_s=1.0
        session.supervise(clock())
        assert len(transport.frames_of_type(proto.MsgType.CMD_STOP)) == 1

    def test_short_divergence_does_not_stop(self, session_factory):
        session, transport, clock = self._flying_session(session_factory)
        session.position.set_target(2.0, 0.0, 0.3)
        self._feed_valid(session, clock, 0.1)
        session.supervise(clock())
        self._feed_valid(session, clock, 0.5)         # < divergence_hold_s
        session.supervise(clock())
        # 誤差解消(目標を現在位置へ)で計時がリセットされる
        session.position.set_target(0.0, 0.0, 0.3)
        self._feed_valid(session, clock, 1.5)
        session.supervise(clock())
        assert transport.frames_of_type(proto.MsgType.CMD_STOP) == []

    def test_inactive_before_takeoff(self, session_factory):
        """armed(離陸前)では発散検知しない(flying のみ)。"""
        session, transport, clock = session_factory()
        assert session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        session.position.on_mocap_pose(make_pose(t=clock()))
        assert session.start()
        session.position.stop()
        transport.clear_sent()
        session.position.set_target(2.0, 0.0, 0.3)
        for _ in range(int(1.5 / FRAME_DT)):
            session.position.on_mocap_pose(
                make_pose(t=clock.advance(FRAME_DT)))
            session.supervise(clock())
        assert transport.frames_of_type(proto.MsgType.CMD_STOP) == []
