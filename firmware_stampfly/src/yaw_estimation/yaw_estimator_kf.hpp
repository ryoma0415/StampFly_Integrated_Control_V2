#ifndef STAMPFLY_YAW_ESTIMATION_YAW_ESTIMATOR_KF_HPP
#define STAMPFLY_YAW_ESTIMATION_YAW_ESTIMATOR_KF_HPP

#include <Arduino.h>

#include "angle_utils.hpp"
#include "mag_calibration.hpp"

// 4状態EKF (ff_pipeline_design.md §5.5, yaw_estimation_ff_two_methods.md §4)。
// 状態 x = [ψ, b_g, b_mx, b_my] (rad / rad/s / µT / µT)。
// 予測は sensorHub tick 毎(400Hz, dt 実測, Q は dt スケール)、更新は fresh な
// 磁気サンプルのみ(実効10Hz)。観測は FF 補正後 EMA 磁場をレベル化した水平
// 2成分 z=(ℓx,ℓy)、観測モデル h(x)=R_z(ψ−ψ0)·B0_horiz + b_m。
// R_z は標準CCW (atan2(h_y,h_x) が ψ とともに増える向き — 相補フィルタが
// 成立している既存規約と同一。符号を誤ると正帰還で即発散する)。

// チルト補償(水平化)は angle_utils.hpp の levelMagVectorBody に一本化した
// (yaw側ではここに複製があった。実機検証済みの非教科書符号 — 変更禁止)。

// ゲート状態ビット (テレメトリ ffg, ff_pipeline_design.md §5.5)
enum FfEkfGateBits : uint8_t {
    FF_EKF_GATE_R_INFLATED = 1u << 0,  // NIS>5.99 → R×(NIS/5.99) 膨張適用中 (norm 8-20µT の R 膨張も)
    FF_EKF_GATE_NIS_REJECT = 1u << 1,  // NIS>13.8 → 棄却
    FF_EKF_GATE_NORM_REJECT = 1u << 2, // |‖b_corr,filt‖−‖B0‖|>20µT → 棄却
    FF_EKF_GATE_Z_REJECT = 1u << 3,    // |b_corr,filt.z−B0.z|>12µT → 棄却
    FF_EKF_GATE_TILT_SKIP = 1u << 4,   // tilt>25° → スキップ
    FF_EKF_GATE_BM_FROZEN = 1u << 5,   // ‖b_m‖>20µT → 磁気更新凍結(要再アンカー)
    FF_EKF_GATE_DRIFT_WARN = 1u << 6,  // |db_m/dt|>0.3µT/s 10s 継続の警告
};

class YawEstimatorKf {
public:
    // アンカー確定 (モーター始動遷移 / ffanchor)。ψ←ψ0, b_m←0, P←P0。
    // b_g は継続(ジャイロバイアス推定は磁場に依存しないため)。
    // b0_full はノルム/z ゲートの基準となるアンカー3軸磁場。
    void reanchor(float psi0_rad, float b0h_x, float b0h_y, const MagVector& b0_full);
    // ffmode 切替時の再シード: アンカーは保持したまま ψ をリファレンスCFの
    // 現在 yaw に合わせ、P を P0 へ戻す。
    void reseedYaw(float psi_rad);
    // 予測ステップ (400Hz, dt 実測)。ω_z は既存規約 (−gyro_z − 起動offset)。
    void predict(float omega_z_rad_s, float dt_s);
    // 更新ステップ (fresh 磁気サンプルのみ呼ぶこと — hold 値で二重実行しない)。
    // b_corr_filt は FF 補正後 EMA の 3軸磁場、σ は FF 層の自己申告 [µT]。
    void update(
        const MagVector& b_corr_filt,
        float roll_rad,
        float pitch_rad,
        float sigma_ff_uT,
        float sigma_slew_uT,
        float sigma_diff_uT,
        float mag_dt_s
    );

    float yaw() const { return x_[0]; }
    float gyroBias() const { return x_[1]; }
    float bmx() const { return x_[2]; }
    float bmy() const { return x_[3]; }
    float nis() const { return nis_; }
    uint8_t gateBits() const { return gate_bits_; }
    bool anchorValid() const { return anchor_valid_; }

private:
    void resetCovariance();

    float x_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float P_[4][4] = {};
    float psi0_ = 0.0f;
    float b0h_x_ = 0.0f;
    float b0h_y_ = 0.0f;
    MagVector b0_;
    float b0_norm_ = 0.0f;
    bool anchor_valid_ = false;
    bool mag_frozen_ = false;         // bit5: ‖b_m‖>20µT で凍結、再アンカーで解除
    float nis_ = 0.0f;
    uint8_t gate_bits_ = 0;
    float time_since_accept_s_ = 0.0f;  // 連続棄却の P 緩膨張用
    float drift_warn_time_s_ = 0.0f;    // bit6 の継続時間
};

#endif
