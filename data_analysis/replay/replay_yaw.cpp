// ===========================================================================
// replay_yaw.cpp — ヨー4状態EKF(yaw_estimator_kf.cpp)のオフラインハーネス
//
// Code Identity 方式: ファームの yaw_estimator_kf.cpp / mag_calibration.cpp を
// コピーせず直接リンクする(数式・符号・定数は yaw_config.hpp の契約どおり)。
//
// モード:
//  (1) --selftest(既定): 合成データ自己試験。既知真値の軌道(チルト運動学と
//      整合する p/r を逆算)+観測モデル整合の合成磁気(10Hz)で、収束・
//      b_g 同定・NIS 挙動・ソフト再捕捉(bit7)を検証する。終了コード 0=PASS。
//  (2) --csv <v4_log.csv>: 実ログの predict 経路のみの再生。
//      v4 ログには生磁気が無いため磁気更新は再生不能(README の拡張提案参照)。
//      ジャイロ+roll/pitch を fresh TLM レート(~23Hz)で供給し、predict のみの
//      ドリフトを機上EKF(tlm_yaw_est_rad)・mocap 真値と比較する。
//
// 使い方:
//   replay_yaw [--selftest]
//   replay_yaw --csv <v4_log.csv> [--out out.csv]
//   末尾の "SELFTEST ..." / "RESULT ..." 1行は sweep.py が読む機械可読形式。
// ===========================================================================

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "angle_utils.hpp"
#include "yaw_config.hpp"
#include "yaw_estimator_kf.hpp"

namespace {

constexpr float DT_400HZ = 0.0025f;

struct ErrStat {
    double sum2 = 0.0;
    double max_abs = 0.0;
    long n = 0;
    void add(double e) {
        sum2 += e * e;
        if (std::fabs(e) > max_abs) max_abs = std::fabs(e);
        n++;
    }
    double rms() const { return n > 0 ? std::sqrt(sum2 / n) : NAN; }
};

// levelMagVectorBody の逆変換: 望む level 観測 L から機体系磁場 b を合成する。
// レベル化行列は非直交(実機検証済みの非教科書符号)のため 3x3 逆行列で解く。
MagVector unlevelMagVector(float roll_rad, float pitch_rad, const MagVector& level) {
    const float cr = cosf(roll_rad), sr = sinf(roll_rad);
    const float cp = cosf(pitch_rad), sp = sinf(pitch_rad);
    // M(levelMagVectorBody の係数行列)
    const float m[3][3] = {
        {cp, 0.0f, sp},
        {sr * sp, cr, sr * cp},
        {-cr * sp, sr, cr * cp},
    };
    const float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                      m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                      m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    const float inv[3][3] = {
        {(m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det,
         (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det,
         (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det},
        {(m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det,
         (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det,
         (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det},
        {(m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det,
         (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det,
         (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det},
    };
    return MagVector{
        inv[0][0] * level.x + inv[0][1] * level.y + inv[0][2] * level.z,
        inv[1][0] * level.x + inv[1][1] * level.y + inv[1][2] * level.z,
        inv[2][0] * level.x + inv[2][1] * level.y + inv[2][2] * level.z,
    };
}

// ---------------------------------------------------------------------------
// (1) 合成データ自己試験
// ---------------------------------------------------------------------------
int runSelftest() {
    YawEstimatorKf kf;

    // アンカー磁場(レベル観測系): 水平 (28,7) µT + 鉛直 −36 µT
    const float psi0 = 0.3f;
    const float b0h_x = 28.0f, b0h_y = 7.0f, b0_z = -36.0f;
    kf.reanchor(psi0, b0h_x, b0h_y, MagVector{b0h_x, b0h_y, b0_z});

    const float bg_true = 0.01f;  // 真のジャイロバイアス 0.573 °/s
    const float dur_s = 120.0f;
    const int n_ticks = (int)(dur_s / DT_400HZ);
    const int mag_div = 40;  // 磁気更新 10Hz(実効レート)

    std::mt19937 rng(20260718u);
    std::normal_distribution<float> gyro_noise(0.0f, 0.003f);  // rad/s
    std::normal_distribution<float> mag_noise(0.0f, 1.8f);     // µT (~R_base=(2µT)² 相当)

    // 真値軌道: 10s ホールド → ±57° 正弦(peak 20°/s)。t=45s で +40° ステップを
    // 注入(ジャイロには現れない蓄積誤差のモデル → ソフト再捕捉 bit7 の試験)。
    const float step_t = 45.0f, step_rad = 40.0f * (float)DEG_TO_RAD;
    auto psi_true_f = [&](float t) {
        float psi = psi0;
        if (t >= 10.0f) psi += 1.0f * sinf(0.35f * (t - 10.0f));
        if (t >= step_t) psi += step_rad;
        return psi;
    };
    auto psi_dot_f = [&](float t) {
        return (t >= 10.0f) ? 0.35f * cosf(0.35f * (t - 10.0f)) : 0.0f;
    };

    ErrStat conv_err;          // 収束窓 20〜44.9s のヨー誤差 [deg]
    double nis_sum = 0.0;
    long nis_n = 0, upd_n = 0, rej_n = 0;
    bool recapture_seen = false;
    float recover_time = -1.0f;
    float bg_at_44 = NAN;

    for (int i = 0; i < n_ticks; i++) {
        const float t = i * DT_400HZ;
        const float psi_true = psi_true_f(t);
        const float psi_dot = psi_dot_f(t);

        // 姿勢と機体レート(チルト運動学 ψ̇=((r−bg)cosθ−p·sinθ)/cosφ と整合)
        const float roll = 0.15f * sinf(0.9f * t);
        const float pitch = 0.10f * sinf(0.7f * t + 1.0f);
        const float p_rate = 0.15f * 0.9f * cosf(0.9f * t);  // = dφ/dt
        const float r_gyro = bg_true +
                             (psi_dot * cosf(roll) + p_rate * sinf(pitch)) / cosf(pitch) +
                             gyro_noise(rng);

        kf.predict(r_gyro, p_rate, roll, pitch, DT_400HZ);

        if (i % mag_div == 0) {
            // 観測モデル整合の合成磁気: L = R_z(ψ−ψ0)·B0h + noise を機体系へ逆変換
            const float beta = psi_true - psi0;
            const float cb = cosf(beta), sb = sinf(beta);
            const MagVector level{cb * b0h_x - sb * b0h_y + mag_noise(rng),
                                  sb * b0h_x + cb * b0h_y + mag_noise(rng),
                                  b0_z + 0.5f * mag_noise(rng)};
            const MagVector b_body = unlevelMagVector(roll, pitch, level);
            kf.update(b_body, roll, pitch, 0.5f, 0.0f, 0.0f, mag_div * DT_400HZ);
            const uint8_t g = kf.gateBits();
            if (g & FF_EKF_GATE_RECAPTURE) recapture_seen = true;
            // NIS/棄却率は収束窓(ステップ注入前)のみで評価する
            if (t >= 20.0f && t < 44.9f) {
                upd_n++;
                if (g & FF_EKF_GATE_NIS_REJECT) {
                    rej_n++;
                } else if (!(g & FF_EKF_GATE_TILT_SKIP)) {
                    nis_sum += kf.nis();
                    nis_n++;
                }
            }
        }

        const float err_deg = wrapPi(kf.yaw() - psi_true) * (float)RAD_TO_DEG;
        if (t >= 20.0f && t < 44.9f) conv_err.add(err_deg);
        if (t >= 44.85f && t < 44.9f && !std::isfinite(bg_at_44)) bg_at_44 = kf.gyroBias();
        if (t > step_t && recover_time < 0.0f && std::fabs(err_deg) < 5.0f)
            recover_time = t - step_t;
        if (i % (int)(5.0f / DT_400HZ) == 0 && t > step_t - 1.0f)
            std::printf("  t=%5.1fs err=%+7.2f deg  gate=0x%02x\n", t, err_deg, kf.gateBits());
    }
    const float final_err_deg =
        wrapPi(kf.yaw() - psi_true_f(dur_s - DT_400HZ)) * (float)RAD_TO_DEG;
    const float bg_err_deg_s = (bg_at_44 - bg_true) * (float)RAD_TO_DEG;
    const double nis_mean = nis_n > 0 ? nis_sum / nis_n : NAN;
    const double rej_rate = upd_n > 0 ? (double)rej_n / upd_n : NAN;

    // 再捕捉の合否基準は「理想回復」ではなくファームの設計済み挙動に合わせる:
    // 実際の引き込みは P 収縮でゲイン律速(3°/更新クランプ未満)となり、回復後は
    // b_m がステップの一部を吸収して逆側に数° 残る(yaw_estimator_kf.cpp の
    // recapture コメントに記載の既知挙動)。
    struct Check {
        const char* name;
        bool ok;
    } checks[] = {
        {"converged rms<=2deg", conv_err.rms() <= 2.0},
        {"converged max<=6deg", conv_err.max_abs <= 6.0},
        {"bg error<=0.15deg/s", std::fabs(bg_err_deg_s) <= 0.15f},
        {"nis mean in [0.5,4]", nis_mean >= 0.5 && nis_mean <= 4.0},
        {"reject rate<=5%", rej_rate <= 0.05},
        {"recapture(bit7) seen", recapture_seen},
        {"recover(|err|<5deg)<=25s after 40deg step",
         recover_time >= 0.0f && recover_time <= 25.0f},
        {"final |err|<=8deg (b_m 吸収の逆側テール込み)", std::fabs(final_err_deg) <= 8.0f},
    };
    bool all_ok = true;
    std::printf("-- yaw EKF selftest (synthetic, %.0fs @400Hz predict / 10Hz mag) --\n", dur_s);
    std::printf("converged(20-45s): rms=%.3f deg  max=%.3f deg\n", conv_err.rms(),
                conv_err.max_abs);
    std::printf("bg: true=%.3f est=%.3f deg/s (err=%.3f)\n", bg_true * (float)RAD_TO_DEG,
                bg_at_44 * (float)RAD_TO_DEG, bg_err_deg_s);
    std::printf("nis mean=%.2f (expect ~2)  reject=%ld/%ld (%.1f%%)\n", nis_mean, rej_n, upd_n,
                100.0 * rej_rate);
    std::printf("40deg step @45s: recapture_seen=%d  recover=%.1fs  final_err=%.2f deg\n",
                recapture_seen, recover_time, final_err_deg);
    for (const auto& c : checks) {
        std::printf("[%s] %s\n", c.ok ? "PASS" : "FAIL", c.name);
        all_ok = all_ok && c.ok;
    }
    std::printf(
        "SELFTEST rms_deg=%.4f max_deg=%.4f bg_err_deg_s=%.4f nis_mean=%.3f "
        "reject_rate=%.4f recapture_seen=%d recover_s=%.2f final_err_deg=%.3f result=%s\n",
        conv_err.rms(), conv_err.max_abs, bg_err_deg_s, nis_mean, rej_rate,
        (int)recapture_seen, recover_time, final_err_deg, all_ok ? "PASS" : "FAIL");
    return all_ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// (2) v4 CSV アダプタ(predict 経路のみ)
// ---------------------------------------------------------------------------
std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : line) {
        if (c == ',') {
            out.push_back(cur);
            cur.clear();
        } else if (c != '\r') {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

float parseF(const std::string& s) {
    return s.empty() ? NAN : std::strtof(s.c_str(), nullptr);
}

int runCsv(const char* csv_path, const char* out_path) {
    std::ifstream in(csv_path);
    if (!in) {
        std::fprintf(stderr, "cannot open %s\n", csv_path);
        return 1;
    }
    std::string line;
    if (!std::getline(in, line)) return 1;
    std::unordered_map<std::string, int> col;
    {
        auto names = splitCsvLine(line);
        for (size_t i = 0; i < names.size(); i++) col[names[i]] = (int)i;
    }
    for (const char* n : {"elapsed_time", "tlm_elapsed_ms", "tlm_p_rad_s", "tlm_r_rad_s",
                          "tlm_roll_rad", "tlm_pitch_rad", "tlm_yaw_est_rad",
                          "tlm_yaw_gyro_int_rad", "mocap_yaw_deg"}) {
        if (!col.count(n)) {
            std::fprintf(stderr, "column not found: %s\n", n);
            return 1;
        }
    }

    struct Row {
        float t_s, tlm_ms, p, r, roll, pitch, yaw_est, yaw_gyro_int, mocap_yaw;
    };
    std::vector<Row> rows;
    float prev_ms = -1.0f;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto f = splitCsvLine(line);
        if ((int)f.size() <= col["mocap_yaw_deg"]) continue;
        const float ms = parseF(f[col["tlm_elapsed_ms"]]);
        if (!std::isfinite(ms) || ms == prev_ms) continue;  // fresh TLM のみ
        prev_ms = ms;
        rows.push_back(Row{parseF(f[col["elapsed_time"]]), ms, parseF(f[col["tlm_p_rad_s"]]),
                           parseF(f[col["tlm_r_rad_s"]]), parseF(f[col["tlm_roll_rad"]]),
                           parseF(f[col["tlm_pitch_rad"]]), parseF(f[col["tlm_yaw_est_rad"]]),
                           parseF(f[col["tlm_yaw_gyro_int_rad"]]),
                           parseF(f[col["mocap_yaw_deg"]]) * (float)DEG_TO_RAD});
    }
    if (rows.size() < 10) {
        std::fprintf(stderr, "too few fresh TLM samples: %zu\n", rows.size());
        return 1;
    }

    // predict のみ: ψ を機上EKFの初期値に合わせ、b_g=0 から積分する。
    // v4 ログに生磁気が無いため update は呼べない(README のログ拡張提案参照)。
    // 注: 機上は未フィルタの Yaw_rate_raw−offset を使うが、ログには T=0.003 LPF 後の
    // tlm_r_rad_s しか無い(実効 ~23Hz では差はほぼ透過)。
    YawEstimatorKf kf;
    kf.reseedYaw(rows.front().yaw_est);

    std::FILE* out = nullptr;
    if (out_path) {
        out = std::fopen(out_path, "w");
        if (!out) {
            std::fprintf(stderr, "cannot write %s\n", out_path);
            return 1;
        }
        std::fprintf(out, "t_s,yaw_pred_rad,tlm_yaw_est_rad,tlm_yaw_gyro_int_rad,mocap_yaw_rad\n");
    }

    // 比較基準:
    //  - tlm_yaw_gyro_int_rad: 機上 400Hz の Z ジャイロ単純積算(磁気補正なし)。
    //    predict 経路の忠実度チェック(~23Hz ホールド供給+チルト運動学の差のみ)。
    //  - tlm_yaw_est_rad: 機上EKF(磁気補正あり)。差 = 磁気更新+b_g 学習の寄与。
    //  - mocap_yaw_deg は CSV に出すのみ(7/17 ログでは機体ヨーと無相関に回転して
    //    おり真値として使えない — 剛体マーカー配置のヨー曖昧性とみられる)。
    const float gyro_align = wrapPi(rows.front().yaw_gyro_int - rows.front().yaw_est);
    ErrStat err_onb, err_gyro;
    float final_onb = NAN, final_gyro = NAN;
    for (size_t k = 1; k < rows.size(); k++) {
        const Row& s = rows[k];
        float dt = (s.tlm_ms - rows[k - 1].tlm_ms) * 1.0e-3f;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.043f;
        const int n = (dt / DT_400HZ < 1.5f) ? 1 : (int)std::lround(dt / DT_400HZ);
        const float h = dt / n;
        for (int i = 0; i < n; i++) kf.predict(s.r, s.p, s.roll, s.pitch, h);

        if (out)
            std::fprintf(out, "%.3f,%.6f,%.6f,%.6f,%.6f\n", s.t_s, kf.yaw(), s.yaw_est,
                         s.yaw_gyro_int, s.mocap_yaw);
        final_onb = wrapPi(kf.yaw() - s.yaw_est) * (float)RAD_TO_DEG;
        err_onb.add(final_onb);
        if (std::isfinite(s.yaw_gyro_int)) {
            final_gyro = wrapPi(kf.yaw() + gyro_align - s.yaw_gyro_int) * (float)RAD_TO_DEG;
            err_gyro.add(final_gyro);
        }
    }
    if (out) std::fclose(out);

    const float dur = rows.back().t_s - rows.front().t_s;
    std::printf("log: %s (%zu fresh samples, %.1fs)\n", csv_path, rows.size(), dur);
    std::printf("predict-only replay (磁気更新なし: ログに生磁気が無いため再生不能)\n");
    std::printf("vs onboard gyro integral (忠実度): rms=%.2f deg  max=%.2f deg  final=%.2f deg\n",
                err_gyro.rms(), err_gyro.max_abs, final_gyro);
    std::printf("vs onboard EKF (磁気補正の寄与)  : rms=%.2f deg  max=%.2f deg  final=%.2f deg\n",
                err_onb.rms(), err_onb.max_abs, final_onb);
    std::printf(
        "RESULT file=%s dur_s=%.1f pred_vs_gyroint_rms=%.4f pred_vs_gyroint_final=%.4f "
        "pred_vs_onboard_rms=%.4f pred_vs_onboard_final=%.4f\n",
        csv_path, dur, err_gyro.rms(), final_gyro, err_onb.rms(), final_onb);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const char* csv_path = nullptr;
    const char* out_path = nullptr;
    bool selftest = true;
    for (int i = 1; i < argc; i++) {
        if (!std::strcmp(argv[i], "--selftest")) {
            selftest = true;
        } else if (!std::strcmp(argv[i], "--csv") && i + 1 < argc) {
            csv_path = argv[++i];
            selftest = false;
        } else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) {
            out_path = argv[++i];
        } else {
            std::fprintf(stderr,
                         "usage: replay_yaw [--selftest] | --csv <v4_log.csv> [--out out.csv]\n");
            return 2;
        }
    }
    return selftest ? runSelftest() : runCsv(csv_path, out_path);
}
