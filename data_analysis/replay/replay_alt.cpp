// ===========================================================================
// replay_alt.cpp — 高度KF(alt_kalman.cpp)のオフラインリプレイ
//
// stampfly_ecosystem の eskf_replay.cpp と同じ Code Identity 方式:
// ファームの alt_kalman.cpp / pid.cpp(Filter)をコピーせず直接リンクし、
// v4 飛行ログ(109列 CSV)の tlm_* から入力を再構成して PC 上で再実行する。
//
// 入力再構成(sensor.cpp の Az 生成チェーンと同一の式・定数):
//   tlm_az_g(= sensor_state.Accel_z, T=0.003 LPF 後の生加速度[g])を
//   Accel_z_raw の近似として使い、
//     Accel_z_d = raw_az_d_filter(az_g − Accel_z_offset)   [Filter T=0.1]
//     Az        = az_filter(−Accel_z_d)                    [Filter T=0.1]
//     z_sens    = tlm_altitude_tof_m(機上 alt_filter 済み値をそのまま)
//   Accel_z_offset は機上 CALIBRATION 平均(ログに無い)の代替として、
//   離陸前 WAIT(接地・モーター停止)区間の tlm_az_g 平均で推定する。
//
// 制約: TLM_STATE は実効 ~23Hz(400Hz 機上ループの間引き)。fresh サンプル間を
// 400Hz 相当のサブステップ(入力は区間終端値のホールド)で刻んで機上周期を
// 模擬する。飛行中→WAIT 遷移では enter_wait()/sensor.cpp と同様に KF と
// フィルタを reset する。詳細・限界は README.md。
//
// 使い方:
//   replay_alt <v4_log.csv> [--out out.csv] [--asis|--fixed] [--warmup S]
//     --asis  : 現行ファーム互換(加速度 g 単位のまま = 9.81倍欠落バグ互換)
//     --fixed : 加速度を m/s² に是正(Az×9.80665)
//     省略時は両方を計算・報告する(出力CSVは常に両列を持つ)。
//   サマリは stdout。末尾の "RESULT k=v ..." 1行は sweep.py が読む機械可読形式。
// ===========================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "alt_kalman.hpp"
#include "pid.hpp"  // Filter(ファーム同一の1次LPF)

namespace {

constexpr float GRAVITY_EARTH = 9.80665f;  // lib/bmi270/common.h と同値
constexpr float SUBSTEP_S = 0.0025f;       // 機上 400Hz ループ周期

struct FreshSample {
    float t_s;        // elapsed_time(PC 受信時刻基準。mocap 行と同時刻)
    float tlm_ms;     // tlm_elapsed_ms(機上クロック。dt 計算用)
    int state;        // tlm_state(2=WAIT 3=TAKEOFF 4=HOVER 5=LANDING)
    float az_g;       // tlm_az_g
    float alt_tof;    // tlm_altitude_tof_m
    float onboard_est;  // tlm_altitude_est_m(機上KF出力 = 同一性検証の基準)
    float onboard_vel;  // tlm_alt_velocity_m_s
    float mocap_z;    // raw_pos_z(mocap 生位置 = 真値)
};

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
    if (s.empty()) return NAN;
    return std::strtof(s.c_str(), nullptr);
}

// mocap 系と機体高度系の原点差(定数オフセット)を平均で推定して除去した残差統計
struct MocapFit {
    double offset = NAN;
    ErrStat err;
};

MocapFit fitMocap(const std::vector<double>& est, const std::vector<double>& truth) {
    MocapFit f;
    if (est.empty()) return f;
    double s = 0.0;
    for (size_t i = 0; i < est.size(); i++) s += truth[i] - est[i];
    f.offset = s / est.size();
    for (size_t i = 0; i < est.size(); i++) f.err.add(est[i] + f.offset - truth[i]);
    return f;
}

}  // namespace

int main(int argc, char** argv) {
    const char* csv_path = nullptr;
    const char* out_path = nullptr;
    bool report_asis = true, report_fixed = true;
    float warmup_s = 3.0f;

    for (int i = 1; i < argc; i++) {
        if (!std::strcmp(argv[i], "--out") && i + 1 < argc) {
            out_path = argv[++i];
        } else if (!std::strcmp(argv[i], "--asis")) {
            report_fixed = false;
        } else if (!std::strcmp(argv[i], "--fixed")) {
            report_asis = false;
        } else if (!std::strcmp(argv[i], "--warmup") && i + 1 < argc) {
            warmup_s = std::strtof(argv[++i], nullptr);
        } else if (argv[i][0] != '-') {
            csv_path = argv[i];
        } else {
            std::fprintf(stderr, "unknown option: %s\n", argv[i]);
            return 2;
        }
    }
    if (!csv_path) {
        std::fprintf(stderr,
                     "usage: replay_alt <v4_log.csv> [--out out.csv] [--asis|--fixed] [--warmup S]\n");
        return 2;
    }

    std::ifstream in(csv_path);
    if (!in) {
        std::fprintf(stderr, "cannot open %s\n", csv_path);
        return 1;
    }

    // ---- ヘッダから列番号を引く(v4=109列だが名前で解決し列追加に耐える) ----
    std::string line;
    if (!std::getline(in, line)) return 1;
    std::unordered_map<std::string, int> col;
    {
        auto names = splitCsvLine(line);
        for (size_t i = 0; i < names.size(); i++) col[names[i]] = (int)i;
    }
    const char* needed[] = {"elapsed_time", "tlm_elapsed_ms", "tlm_state",  "tlm_az_g",
                            "tlm_altitude_tof_m", "tlm_altitude_est_m",
                            "tlm_alt_velocity_m_s", "raw_pos_z"};
    for (const char* n : needed) {
        if (!col.count(n)) {
            std::fprintf(stderr, "column not found: %s\n", n);
            return 1;
        }
    }

    // ---- fresh TLM サンプル抽出(tlm_elapsed_ms が変化した行のみ) ----
    std::vector<FreshSample> fresh;
    float prev_tlm_ms = -1.0f;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto f = splitCsvLine(line);
        if ((int)f.size() <= col["raw_pos_z"]) continue;
        const float tlm_ms = parseF(f[col["tlm_elapsed_ms"]]);
        if (!std::isfinite(tlm_ms) || tlm_ms == prev_tlm_ms) continue;
        prev_tlm_ms = tlm_ms;
        FreshSample s;
        s.t_s = parseF(f[col["elapsed_time"]]);
        s.tlm_ms = tlm_ms;
        s.state = (int)parseF(f[col["tlm_state"]]);
        s.az_g = parseF(f[col["tlm_az_g"]]);
        s.alt_tof = parseF(f[col["tlm_altitude_tof_m"]]);
        s.onboard_est = parseF(f[col["tlm_altitude_est_m"]]);
        s.onboard_vel = parseF(f[col["tlm_alt_velocity_m_s"]]);
        s.mocap_z = parseF(f[col["raw_pos_z"]]);
        fresh.push_back(s);
    }
    if (fresh.size() < 10) {
        std::fprintf(stderr, "too few fresh TLM samples: %zu\n", fresh.size());
        return 1;
    }

    // ---- Accel_z_offset 推定(離陸前 WAIT 区間の tlm_az_g 平均) ----
    double off_sum = 0.0;
    long off_n = 0;
    for (const auto& s : fresh) {
        if (s.state != 2) break;  // 最初の非WAIT(=離陸)以降は使わない
        if (std::isfinite(s.az_g)) {
            off_sum += s.az_g;
            off_n++;
        }
    }
    if (off_n < 10) {
        std::fprintf(stderr, "warning: pre-takeoff WAIT samples=%ld, offset may be poor\n", off_n);
    }
    const float accel_z_offset = off_n > 0 ? (float)(off_sum / off_n) : -1.0f;

    // ---- リプレイ本体 ----
    // ファーム同一のフィルタ・KF(sensor_init と同じ set_parameter 値)
    Filter raw_az_d_filter, az_filter;
    raw_az_d_filter.set_parameter(0.1, 0.0025);  // sensor.cpp:142 alt158
    az_filter.set_parameter(0.1, 0.0025);        // sensor.cpp:143 alt158
    Alt_kalman kf_asis;   // 現行互換: 加速度 g 単位のまま(9.81倍欠落バグ互換)
    Alt_kalman kf_fixed;  // 是正: Az×9.80665 で m/s² を入力

    std::FILE* out = nullptr;
    if (out_path) {
        out = std::fopen(out_path, "w");
        if (!out) {
            std::fprintf(stderr, "cannot write %s\n", out_path);
            return 1;
        }
        std::fprintf(out,
                     "t_s,state,alt_tof_m,alt_est_asis_m,alt_vel_asis_m_s,"
                     "alt_est_fixed_m,alt_vel_fixed_m_s,onboard_est_m,onboard_vel_m_s,mocap_z_m\n");
    }

    ErrStat id_asis, id_fixed;             // Code Identity: 機上 tlm_altitude_est_m との差
    std::vector<double> mc_truth;          // mocap 比較(飛行窓)
    std::vector<double> mc_tof, mc_onboard, mc_asis, mc_fixed;
    std::vector<double> dt_ms_list;

    const float t0 = fresh.front().t_s;
    for (size_t k = 1; k < fresh.size(); k++) {
        const FreshSample& s = fresh[k];

        // 飛行中→WAIT 遷移: enter_wait() の KF reset と sensor.cpp の
        // フィルタ static reset を模擬(機上と同じ関数を呼ぶ)
        if (s.state == 2 && fresh[k - 1].state > 2) {
            kf_asis.reset();
            kf_fixed.reset();
            raw_az_d_filter.reset();
            az_filter.reset();
        }

        float dt = (s.tlm_ms - fresh[k - 1].tlm_ms) * 1.0e-3f;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.043f;  // TLM 欠落時は実効レート仮定
        dt_ms_list.push_back(dt * 1000.0);

        // 400Hz サブステップ(入力は区間終端サンプルのホールド)
        const int n = (dt / SUBSTEP_S < 1.5f) ? 1 : (int)std::lround(dt / SUBSTEP_S);
        const float h = dt / n;
        for (int i = 0; i < n; i++) {
            const float accel_z_d = raw_az_d_filter.update(s.az_g - accel_z_offset, h);
            const float az = az_filter.update(-accel_z_d, h);
            kf_asis.update(s.alt_tof, az, h);
            kf_fixed.update(s.alt_tof, az * GRAVITY_EARTH, h);
        }

        if (out) {
            std::fprintf(out, "%.3f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                         s.t_s, s.state, s.alt_tof, kf_asis.Altitude, kf_asis.Velocity,
                         kf_fixed.Altitude, kf_fixed.Velocity, s.onboard_est, s.onboard_vel,
                         s.mocap_z);
        }

        if (s.t_s - t0 < warmup_s) continue;

        // Code Identity 検証(全状態。機上KF出力の再現度)
        if (std::isfinite(s.onboard_est)) {
            id_asis.add(kf_asis.Altitude - s.onboard_est);
            id_fixed.add(kf_fixed.Altitude - s.onboard_est);
        }
        // mocap 比較は飛行窓(TAKEOFF/HOVER/LANDING)のみ
        if (s.state >= 3 && s.state <= 5 && std::isfinite(s.mocap_z)) {
            mc_truth.push_back(s.mocap_z);
            mc_tof.push_back(s.alt_tof);
            mc_onboard.push_back(s.onboard_est);
            mc_asis.push_back(kf_asis.Altitude);
            mc_fixed.push_back(kf_fixed.Altitude);
        }
    }
    if (out) std::fclose(out);

    const MocapFit f_tof = fitMocap(mc_tof, mc_truth);
    const MocapFit f_onb = fitMocap(mc_onboard, mc_truth);
    const MocapFit f_asis = fitMocap(mc_asis, mc_truth);
    const MocapFit f_fixed = fitMocap(mc_fixed, mc_truth);

    double dt_med = NAN;
    if (!dt_ms_list.empty()) {
        std::vector<double> tmp = dt_ms_list;
        std::sort(tmp.begin(), tmp.end());
        dt_med = tmp[tmp.size() / 2];
    }

    std::printf("log: %s\n", csv_path);
    std::printf("fresh TLM samples: %zu (median dt %.1f ms), Accel_z_offset=%.5f g (n=%ld)\n",
                fresh.size(), dt_med, accel_z_offset, off_n);
    std::printf("\n-- Code Identity: replay vs onboard tlm_altitude_est_m (t>%.1fs, n=%ld) --\n",
                warmup_s, id_asis.n);
    if (report_asis)
        std::printf("asis  : rms=%.4f m  max=%.4f m\n", id_asis.rms(), id_asis.max_abs);
    if (report_fixed)
        std::printf("fixed : rms=%.4f m  max=%.4f m  (機上との差 = 9.81倍是正の効果量)\n",
                    id_fixed.rms(), id_fixed.max_abs);
    std::printf("\n-- vs mocap raw_pos_z (flight window, const offset removed, n=%zu) --\n",
                mc_truth.size());
    std::printf("tof    : offset=%+.4f m  rms=%.4f m  max=%.4f m\n", f_tof.offset,
                f_tof.err.rms(), f_tof.err.max_abs);
    std::printf("onboard: offset=%+.4f m  rms=%.4f m  max=%.4f m\n", f_onb.offset,
                f_onb.err.rms(), f_onb.err.max_abs);
    if (report_asis)
        std::printf("asis   : offset=%+.4f m  rms=%.4f m  max=%.4f m\n", f_asis.offset,
                    f_asis.err.rms(), f_asis.err.max_abs);
    if (report_fixed)
        std::printf("fixed  : offset=%+.4f m  rms=%.4f m  max=%.4f m\n", f_fixed.offset,
                    f_fixed.err.rms(), f_fixed.err.max_abs);

    std::printf(
        "RESULT file=%s offset_g=%.5f id_asis_rms=%.5f id_asis_max=%.5f "
        "id_fixed_rms=%.5f id_fixed_max=%.5f mocap_tof_rms=%.5f mocap_onboard_rms=%.5f "
        "mocap_asis_rms=%.5f mocap_asis_max=%.5f mocap_fixed_rms=%.5f mocap_fixed_max=%.5f\n",
        csv_path, accel_z_offset, id_asis.rms(), id_asis.max_abs, id_fixed.rms(),
        id_fixed.max_abs, f_tof.err.rms(), f_onb.err.rms(), f_asis.err.rms(), f_asis.err.max_abs,
        f_fixed.err.rms(), f_fixed.err.max_abs);
    return 0;
}
