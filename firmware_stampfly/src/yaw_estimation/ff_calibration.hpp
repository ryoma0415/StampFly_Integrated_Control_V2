#ifndef STAMPFLY_YAW_ESTIMATION_FF_CALIBRATION_HPP
#define STAMPFLY_YAW_ESTIMATION_FF_CALIBRATION_HPP

#include <Arduino.h>

#include "yaw_config.hpp"
#include "mag_calibration.hpp"

// 電流フィードフォワード(FF)較正係数モジュール (ff_pipeline_design.md §5.1)。
//
// ステージング領域(ffcal_begin〜ffcal_commit)と確定領域の二段構え。commit 時に
// 完全性(全 idx 受領・電流昇順)と CRC-32(IEEE, zlib.crc32 互換)を検証してから
// ランタイムへ反映する。CRC の対象バイト列は §4 の定義どおり float32
// little-endian の連結:
//   for k in 0..nlut-1: ia_k, dx_k, dy_k, dz_k
//   for m in 0..3 (FL,FR,RL,RR): ax, ay, az, c2, c1, c0
//   iid
// NVS の blob もこれと同一順で保存する(persistence.cpp)。
//
// 計算 (yaw_estimation_ff_two_methods.md §2):
//   方式A: ΔB̂ = LUT(I_total)  区分線形・範囲外は端区間の傾きで外挿
//   方式B: ΔB̂ = LUT(I_total) + Σ_m ã_m·δI_m  (差動項)
//     Î_m = c2·d² + c1·d + c0 (mask 外は d=0), I_active = I_total − I_idle
//     (I_idle はアンカー実測を優先、未取得時はベンチ参考値 iid),
//     s = I_active/ΣÎ_m (ΣÎ < 0.05 A なら差動項 0), δI_m = s·Î_m − I_active/4
// σ_ff/σ_slew/σ_diff の自己申告値も返す(EKF の適応 R 用、同 §2.†)。

// ΔB̂ と FF 不確かさの自己申告値(1回の compute() の結果)。
struct FfCorrection {
    MagVector delta_b;            // 適用すべき ΔB̂ [µT] (b_cal 座標)
    float sigma_ff_uT = 0.0f;     // κ_ff·|ΔB̂_xy|
    float sigma_slew_uT = 0.0f;   // |a_xy|·|dI/dt|·τ_resid
    float sigma_diff_uT = 0.0f;   // 0.3·30µT/A·max_m|δI_m| (方式Bのみ)
};

class FfCalibration {
public:
    // --- ステージング (UDP コマンドから) ---
    bool stageBegin(uint8_t nlut);
    bool stageLutPoint(uint8_t idx, float ia, float dx, float dy, float dz);
    bool stageMotor(uint8_t idx, float ax, float ay, float az, float c2, float c1, float c0);
    bool stageAux(float iid);
    // 完全性 + CRC 照合。成功でステージングを確定領域へ反映して true。
    // 失敗時は error_message に理由(静的文字列)を返す。
    bool commit(uint32_t crc_expected, const char*& error_message);
    // 確定領域・ステージングとも無効化。
    void clear();

    // --- 確定値アクセス ---
    bool valid() const { return valid_; }
    uint8_t nlut() const { return confirmed_.nlut; }
    uint32_t crc() const { return crc_; }
    float iid() const { return confirmed_.iid; }

    // --- NVS 保存/復元用 (persistence.cpp) ---
    // blob の float 数 = nlut*4 + 4*6 + 1
    static uint16_t blobFloatCountFor(uint8_t nlut) { return static_cast<uint16_t>(nlut) * 4u + 25u; }
    uint16_t blobFloatCount() const { return blobFloatCountFor(confirmed_.nlut); }
    // 確定値を §4 の順で書き出す(呼び出し側が blobFloatCount() 分を確保)。
    void serialize(float* out) const;
    // NVS blob からの復元。昇順・有限値を再検証し、成功で確定領域へ反映。
    // CRC の照合は呼び出し側(persistence)が保存済み crc と行う。
    bool restoreFromBlob(const float* values, uint8_t nlut, uint32_t crc);

    // CRC-32 (IEEE 0xEDB88320, zlib.crc32 互換)。values は float32 LE 連結として扱う。
    static uint32_t crc32Of(const float* values, uint16_t count);

    // ΔB̂ と σ 自己申告の計算。ff_mode: 0=off(ゼロを返す), 1=方式A, 2=方式B。
    // duty[4] はランプ後実効 duty (FL,FR,RL,RR)、mask はモーター選択ビット。
    // now_ms は σ_slew 用の dI/dt 差分時刻。
    FfCorrection compute(
        float i_total_a,
        const float duty[4],
        uint8_t mask,
        bool motors_running,
        uint8_t ff_mode,
        float i_idle_anchor_a,
        bool anchor_valid,
        uint32_t now_ms
    );

private:
    struct Coeffs {
        uint8_t nlut = 0;
        float lut_ia[FF_LUT_MAX_POINTS] = {};
        float lut_db[FF_LUT_MAX_POINTS][3] = {};
        float mot[4][6] = {};  // m=0:FL,1:FR,2:RL,3:RR / ax,ay,az(ã_m),c2,c1,c0
        float iid = 0.0f;      // ベンチ参考アイドル電流 [A]
    };

    // LUT 区分線形補間(範囲外は端区間の傾きで外挿)。axis=0/1/2。
    float lutInterp(float i_total_a, uint8_t axis) const;
    // LUT の局所傾き [µT/A] (σ_slew 用)。
    float lutSlope(float i_total_a, uint8_t axis) const;
    // I_total が属する LUT 区間の下端 index (0..nlut-2)。
    uint8_t lutSegment(float i_total_a) const;
    static void serializeCoeffs(const Coeffs& c, float* out);

    Coeffs confirmed_;
    bool valid_ = false;
    uint32_t crc_ = 0;

    Coeffs staging_;
    bool staging_active_ = false;
    uint32_t staged_lut_mask_ = 0;  // bit k = LUT 点 k 受領済み
    uint8_t staged_mot_mask_ = 0;   // bit m = モーター m 受領済み
    bool staged_aux_ = false;

    // σ_slew 用の電流微分トラッキング
    float slew_last_i_a_ = 0.0f;
    uint32_t slew_last_ms_ = 0;
    bool slew_has_last_ = false;
};

#endif
