// ===========================================================================
// sensor_hub_ff.cpp — ヨー推定・電流FF補正の統合フック実装
//
// yaw側 sensor_hub.cpp から FF補正挿入点・アイドルアンカー・3系統推定器の
// 駆動ロジックを抽出移植したもの。IMU/AHRS/ToF はベース(sensor.cpp)側の
// 実装を使うため持ち込まない。数式・符号・定数値は yaw側と同一。
// 差分は入力の取り方のみ:
//   - motors_running / duty[4](FL,FR,RL,RR)/ mask を呼び出し側から受け取る
//     (yaw側は motorTestRunning()/motorTestDuty()×mask だった。飛行中は
//      ミキサ出力4値を FL,FR,RL,RR の順で渡す — ベース変数は FR,FL,RR,RL 順
//      なので呼び出し側で並べ替えること)
//   - ToF 読み tick では磁気/電流の 20Hz スロットをスキップ(位相スタガ)
// ===========================================================================
#include "sensor_hub_ff.hpp"

#include <math.h>

#include "angle_utils.hpp"
#include "persistence.hpp"
#include "yaw_config.hpp"

YawEstimationState g_yaw_est;

namespace {

void noteMagError(uint32_t& counter, const char* what) {
    counter++;
    if (counter % 100 == 1) {
        USBSerial.printf("BMM150 %s: errors=%lu\n", what, static_cast<unsigned long>(counter));
    }
}

// ---- 電流FF補正: アイドルアンカー (yaw側 ff_pipeline_design.md §5.3) ----

// モーター停止中の b_cal / I_total を 20Hz で積む 2s リングバッファ。
struct FfAnchorWindow {
    MagVector b[FF_ANCHOR_WINDOW_SAMPLES];
    float i[FF_ANCHOR_WINDOW_SAMPLES];
    uint8_t head = 0;
    uint8_t count = 0;

    void reset() {
        head = 0;
        count = 0;
    }
    void push(const MagVector& b_cal, float i_total) {
        b[head] = b_cal;
        i[head] = i_total;
        head = static_cast<uint8_t>((head + 1) % FF_ANCHOR_WINDOW_SAMPLES);
        if (count < FF_ANCHOR_WINDOW_SAMPLES) {
            count++;
        }
    }
    bool full() const { return count >= FF_ANCHOR_WINDOW_SAMPLES; }
    void mean(MagVector& b_mean, float& i_mean) const {
        float sx = 0.0f, sy = 0.0f, sz = 0.0f, si = 0.0f;
        for (uint8_t k = 0; k < count; k++) {
            sx += b[k].x;
            sy += b[k].y;
            sz += b[k].z;
            si += i[k];
        }
        const float inv = count > 0 ? 1.0f / static_cast<float>(count) : 0.0f;
        b_mean = MagVector{sx * inv, sy * inv, sz * inv};
        i_mean = si * inv;
    }
};

FfAnchorWindow g_ff_anchor_window;
TwoWire* g_wire = nullptr;          // sensorHubFfInit で渡された I2C バス
bool g_mag_cal_seen = false;        // fresh な b_cal を一度でも観測したか
bool g_ff_boot_anchor_done = false; // ブート後の自動初回アンカー取得済みか
// 直近 update で使ったレベル化入力姿勢(アンカー凍結時の B0 水平化に使う)
float g_last_roll_rad = 0.0f;
float g_last_pitch_rad = 0.0f;

// 補正系推定器の再シード (§5.4): 補正CFへリファレンスCFの内部状態
// (yaw・磁気オフセット・ヨーゼロ)をコピーし、補正系EMAとノルム基準を
// 再初期化。EKF の ψ もリファレンス yaw に合わせる。
void ffReseedCorrectedEstimators() {
    g_yaw_est.yaw_estimator_corr = g_yaw_est.yaw_estimator;
    g_yaw_est.yaw_estimator_corr.resetMagNormReference();
    g_yaw_est.mag_filter_corr.reset();
    g_yaw_est.yaw_kf.reseedYaw(g_yaw_est.yaw_estimator.yaw());
}

// 停止窓が満ちていればアンカーを凍結する: B0 / I_idle / B0_horiz / ψ0 を
// 確定し、b_m←0・P←P0 (reanchor)・補正EMA・補正系norm_ref を再初期化。
bool ffFreezeAnchorFromWindow() {
    if (!g_ff_anchor_window.full()) {
        return false;
    }
    MagVector b0;
    float i_idle = 0.0f;
    g_ff_anchor_window.mean(b0, i_idle);
    const MagVector b0h = levelMagVectorBody(g_last_roll_rad, g_last_pitch_rad, b0);
    g_yaw_est.ff.anchor_b0 = b0;
    g_yaw_est.ff.anchor_b0h_x = b0h.x;
    g_yaw_est.ff.anchor_b0h_y = b0h.y;
    g_yaw_est.ff.anchor_psi0 = g_yaw_est.yaw_estimator.yaw();
    g_yaw_est.ff.anchor_i_idle = i_idle;
    g_yaw_est.ff.anchor_valid = true;
    ffReseedCorrectedEstimators();
    g_yaw_est.yaw_kf.reanchor(g_yaw_est.ff.anchor_psi0, b0h.x, b0h.y, b0);
    return true;
}

// 毎tick呼ぶアンカーサービス: 始動遷移でのアンカー凍結と、停止中20Hzの
// 窓更新、ブート後の自動初回取得を行う。
// 「回転中」判定は呼び出し側の motors_running(PWM実出力ありを含む)に従う:
// ランプダウン中サンプルが停止窓に混入して B0/I_idle を汚染するのを防ぐため、
// 呼び出し側は実出力 duty>0 の間も true を渡すこと(yaw側 C2 と同基準)。
void ffAnchorService(bool motors_running) {
    static uint32_t last_push_ms = 0;
    static bool prev_running = false;
    const bool running = motors_running;

    if (running && !prev_running) {
        // 停止→回転の始動遷移: 直前の停止窓でアンカー凍結。
        // 窓が満ちていない場合は旧アンカーを維持する。
        ffFreezeAnchorFromWindow();
        g_ff_anchor_window.reset();
    } else if (!running && prev_running) {
        // 回転→停止: OFF直後の過渡を含まないよう窓を貯め直す。
        g_ff_anchor_window.reset();
    }
    prev_running = running;

    if (running) {
        return;
    }
    const uint32_t now_ms = millis();
    if (now_ms - last_push_ms < FF_ANCHOR_PERIOD_MS) {
        return;
    }
    last_push_ms = now_ms;
    if (!g_mag_cal_seen || !g_yaw_est.current_ready || !g_yaw_est.current_sample.valid) {
        return;
    }
    g_ff_anchor_window.push(g_yaw_est.frame.mag_cal_body, g_yaw_est.current_sample.current_a);
    if (!g_ff_boot_anchor_done && g_ff_anchor_window.full()) {
        g_ff_boot_anchor_done = ffFreezeAnchorFromWindow();
    }
}

// 20Hz 電流スロット。fresh な読みが起きた tick は current_slot_fired=true。
void updateCurrent() {
    static uint32_t last_current_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_current_ms < YAW_SLOW_SLOT_PERIOD_MS) {
        return;
    }
    last_current_ms = now_ms;
    // スロット発火はデバイス不調でも報告する(ベース側の低電圧判定が
    // 「計測不能な20Hzスロット」を連続カウントして安全側に倒せるように)。
    g_yaw_est.current_slot_fired = true;
    if (!g_yaw_est.current_ready) {
        return;
    }
    g_yaw_est.current_sample = currentSensorRead();
}

// 20Hz 磁気スロット + FF補正パス(yaw側 updateMagnetometer と同一ロジック)。
void updateMagnetometer() {
    static uint32_t last_mag_sample_ms = 0;
    static uint32_t mag_errors = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_mag_sample_ms < YAW_SLOW_SLOT_PERIOD_MS) {
        return;
    }
    last_mag_sample_ms = now_ms;

    if (!g_yaw_est.bmm_ready) {
        static uint32_t last_retry_ms = 0;
        if (g_wire != nullptr && now_ms - last_retry_ms > 1000) {
            last_retry_ms = now_ms;
            g_yaw_est.bmm_ready = g_yaw_est.bmm150.begin(*g_wire, BMM150_I2C_ADDRESS);
        }
        return;
    }

    Bmm150RawSample sample;
    if (!g_yaw_est.bmm150.readRaw(sample)) {
        noteMagError(mag_errors, "read failed");
        return;
    }

    if (!sample.data_ready) {
        return;
    }

    if (!sample.compensated_valid) {
        noteMagError(mag_errors, "compensation failed");
        return;
    }

    YawFfMagFrame& frame = g_yaw_est.frame;
    frame.mag_dt_s = frame.last_mag_fresh_ms == 0
        ? 0.1f
        : static_cast<float>(sample.timestamp_ms - frame.last_mag_fresh_ms) * 1.0e-3f;
    frame.last_mag_fresh_ms = sample.timestamp_ms;

    const MagVector mag_raw_body = transformMagToBody(sample.compensated_sensor);
    const MagVector mag_cal_body = g_yaw_est.mag3d_calibration.valid()
        ? g_yaw_est.mag3d_calibration.apply(mag_raw_body)
        : mag_raw_body;
    frame.mag_raw_body = mag_raw_body;
    frame.mag_cal_body = mag_cal_body;
    g_mag_cal_seen = true;
    // 非補正パス(従来どおり): リファレンスCF・アンカー窓・実験テレメトリ用。
    frame.mag_filtered_body = g_yaw_est.mag_filter.update(mag_cal_body);
    frame.yaw_mag_level_rad =
        wrapPi(atan2f(frame.mag_filtered_body.y, frame.mag_filtered_body.x));
    frame.mag_sample_fresh = true;

    // ---- 電流FF補正パス (ff_pipeline_design.md §5.2) ----
    // b_cal 直後・EMA 前で b_corr = b_cal − ΔB̂。電流は同tickで取得済み
    // (updateCurrent が先に呼ばれる)、duty は呼び出し側が渡した実効値×マスク
    // (飛行中はミキサ出力 FL,FR,RL,RR、モーターテスト中はテストduty×mask)。
    // 電流サンプルが無効 (INA3221 死亡/読み取り失敗) の tick は else 分岐の
    // 非補正縮退 (ΔB̂=0) に落とす — i_total=0 での誤った外挿を避ける。
    const bool ff_active = sensorHubFfCorrectionActive();
    if (ff_active) {
        const FfCorrection corr = g_yaw_est.ff_calibration.compute(
            g_yaw_est.current_sample.current_a,
            g_yaw_est.duty,
            g_yaw_est.motor_mask,
            g_yaw_est.motors_running,
            g_yaw_est.ff.ff_mode,
            g_yaw_est.ff.anchor_i_idle,
            g_yaw_est.ff.anchor_valid,
            now_ms
        );
        g_yaw_est.ff.delta_b = corr.delta_b;
        g_yaw_est.ff.sigma_ff_uT = corr.sigma_ff_uT;
        g_yaw_est.ff.sigma_slew_uT = corr.sigma_slew_uT;
        g_yaw_est.ff.sigma_diff_uT = corr.sigma_diff_uT;
        const MagVector b_corr{
            mag_cal_body.x - corr.delta_b.x,
            mag_cal_body.y - corr.delta_b.y,
            mag_cal_body.z - corr.delta_b.z,
        };
        frame.mag_corr_filtered_body = g_yaw_est.mag_filter_corr.update(b_corr);
    } else {
        g_yaw_est.ff.delta_b = MagVector{};
        g_yaw_est.ff.sigma_ff_uT = 0.0f;
        g_yaw_est.ff.sigma_slew_uT = 0.0f;
        g_yaw_est.ff.sigma_diff_uT = 0.0f;
        frame.mag_corr_filtered_body = frame.mag_filtered_body;
    }
}

}  // namespace

void sensorHubFfInit(TwoWire& wire) {
    g_wire = &wire;
    g_yaw_est.bmm_ready = g_yaw_est.bmm150.begin(wire, BMM150_I2C_ADDRESS);
    if (g_yaw_est.bmm_ready) {
        USBSerial.printf(
            "BMM150 init ok: chip_id=0x%02X trim=%s\n",
            g_yaw_est.bmm150.chipId(),
            g_yaw_est.bmm150.trimValid() ? "ok" : "invalid"
        );
    } else {
        USBSerial.printf("BMM150 init failed: chip_id=0x%02X\n", g_yaw_est.bmm150.chipId());
    }

    g_yaw_est.current_ready = currentSensorInit(wire);
    if (g_yaw_est.current_ready) {
        USBSerial.println("INA3221 init ok: battery current/voltage on CH2 (20Hz)");
    } else {
        USBSerial.println("INA3221 init failed");
    }

    // NVS 復元(V2契約 §2.5 の順): mag3d → accel6 → attmount → geomag →
    // yawzero → ffcal。yawzero はリファレンスCFのリセット後に復元する
    // (リセットが復元済み磁気オフセットを消すため — yaw側 app.cpp 踏襲)。
    loadMag3DCalibration(g_yaw_est.mag3d_calibration);
    loadAccelCalibration(g_yaw_est.accel_calibration);
    loadAttitudeMountZero(
        g_yaw_est.attitude_mount_valid,
        g_yaw_est.roll_mount_offset_rad,
        g_yaw_est.pitch_mount_offset_rad
    );
    loadGeomagneticReference(g_yaw_est.yaw_estimator);
    g_yaw_est.yaw_estimator.reset(0.0f);
    loadYawZero(g_yaw_est.yaw_estimator);
    loadFfCalibration(g_yaw_est.ff_calibration, g_yaw_est.ff.ff_mode, g_yaw_est.ff.est_mode);
    // 補正CFはリファレンスと同じ状態から開始する。
    g_yaw_est.yaw_estimator_corr = g_yaw_est.yaw_estimator;
}

void sensorHubFfUpdate(const SensorHubFfInputs& in) {
    // 入力スナップショット(20Hzスロット内のFF計算・手動アンカーが参照)
    g_yaw_est.motors_running = in.motors_running;
    for (uint8_t m = 0; m < 4; m++) {
        g_yaw_est.duty[m] = in.duty[m];
    }
    g_yaw_est.motor_mask = in.motor_mask;
    g_yaw_est.current_slot_fired = false;

    YawFfMagFrame& frame = g_yaw_est.frame;
    frame.mag_sample_fresh = false;
    frame.mag_dt_s = 0.0f;

    // レベル化に使う姿勢: Madgwick ロール/ピッチにマウントオフセットを適用
    // (yaw側の roll/pitch_tilt_comp_rad 相当。適用先は磁気レベル化のみで、
    //  飛行制御の姿勢には影響しない)。
    const float roll_rad = g_yaw_est.attitude_mount_valid
        ? applyAttitudeOffset(in.roll_rad, g_yaw_est.roll_mount_offset_rad)
        : in.roll_rad;
    const float pitch_rad = g_yaw_est.attitude_mount_valid
        ? applyAttitudeOffset(in.pitch_rad, g_yaw_est.pitch_mount_offset_rad)
        : in.pitch_rad;
    g_last_roll_rad = roll_rad;
    g_last_pitch_rad = pitch_rad;

    float dt_s = in.dt_s;
    if (dt_s <= 0.0f || dt_s > 0.2f) {
        dt_s = SENSOR_PERIOD_US * 1.0e-6f;
    }

    // 20Hz スロット: updateCurrent → updateMagnetometer の順(同tick内順序契約)。
    // ToF 読みが発生した tick ではスキップし次 tick へ繰延べる(位相スタガ:
    // ToF の重量級I2C読みと磁気/電流読みの同tick集中による 2.5ms 予算超過回避)。
    if (!in.tof_read_this_tick) {
        updateCurrent();
        updateMagnetometer();
    }

    // リファレンスCF (既存・非補正mag)。ffモードoff時のアクティブ出力でもある。
    g_yaw_est.yaw_estimator.update(
        in.yaw_rate_rad_s,
        roll_rad,
        pitch_rad,
        frame.mag_filtered_body,
        dt_s,
        frame.mag_sample_fresh,
        frame.mag_dt_s
    );

    // 補正系推定器 (ffモードoff/ffcal無効/電流無効時はリファレンスのみ稼働)。
    const bool ff_active = sensorHubFfCorrectionActive();
    if (ff_active) {
        // 補正CF: 補正済みEMA磁場で第2インスタンスを回す。
        g_yaw_est.yaw_estimator_corr.update(
            in.yaw_rate_rad_s,
            roll_rad,
            pitch_rad,
            frame.mag_corr_filtered_body,
            dt_s,
            frame.mag_sample_fresh,
            frame.mag_dt_s
        );
        // EKF: 予測は毎tick(dt実測)、更新は fresh 磁気サンプルのみ
        // (hold値での二重実行を mag_sample_fresh で防ぐ)。
        g_yaw_est.yaw_kf.predict(in.yaw_rate_rad_s, dt_s);
        if (frame.mag_sample_fresh && g_yaw_est.ff.anchor_valid) {
            g_yaw_est.yaw_kf.update(
                frame.mag_corr_filtered_body,
                roll_rad,
                pitch_rad,
                g_yaw_est.ff.sigma_ff_uT,
                g_yaw_est.ff.sigma_slew_uT,
                g_yaw_est.ff.sigma_diff_uT,
                frame.mag_dt_s
            );
        }
        g_yaw_est.ff.yaw_active_rad = g_yaw_est.ff.est_mode == 1
            ? g_yaw_est.yaw_kf.yaw()
            : g_yaw_est.yaw_estimator_corr.yaw();
    } else {
        g_yaw_est.ff.yaw_active_rad = g_yaw_est.yaw_estimator.yaw();
    }

    // アイドルアンカー窓の更新と始動遷移での凍結 (§5.3)。
    ffAnchorService(in.motors_running);
}

bool sensorHubFfAnchorNow() {
    // 回転中(ランプダウン中の実出力を含む)はまだ磁気汚染があり得るので、
    // 実出力ゼロまで手動アンカーも拒否する (ffAnchorService と同基準)。
    if (g_yaw_est.motors_running) {
        return false;
    }
    return ffFreezeAnchorFromWindow();
}

void sensorHubFfReseed() {
    ffReseedCorrectedEstimators();
}

bool sensorHubFfCorrectionActive() {
    return g_yaw_est.ff.ff_mode != 0 && g_yaw_est.ff_calibration.valid() &&
           g_yaw_est.current_ready && g_yaw_est.current_sample.valid;
}

void sensorHubFfOnMag3dChange() {
    // mag3d較正の変更で b_cal 空間が変わるため、旧空間で凍結した B0 アンカーを
    // 破棄し、窓を貯め直し、ブート時自動アンカーを再武装、補正系推定器を再シード。
    // 係数blobはNVSに残す (ffmode で再有効化可能、ただし旧空間係数の妥当性は
    // ユーザー責任) が、補正モード自体は安全側の off に落として永続化する。
    g_yaw_est.ff.anchor_valid = false;
    g_ff_anchor_window.reset();
    g_ff_boot_anchor_done = false;
    ffReseedCorrectedEstimators();
    if (g_yaw_est.ff.ff_mode != 0) {
        g_yaw_est.ff.ff_mode = 0;
        saveFfModes(g_yaw_est.ff.ff_mode, g_yaw_est.ff.est_mode);
    }
}

bool sensorHubFfEkfHealthy() {
    return sensorHubFfCorrectionActive() && g_yaw_est.yaw_kf.anchorValid() &&
           (g_yaw_est.yaw_kf.gateBits() & FF_EKF_GATE_BM_FROZEN) == 0;
}

bool sensorHubFfMagFresh(uint32_t now_ms) {
    const uint32_t last = g_yaw_est.frame.last_mag_fresh_ms;
    return last != 0 && (now_ms - last) < YAW_MAG_FRESH_TIMEOUT_MS;
}
