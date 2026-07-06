#include "current_sensor.hpp"

#include <INA3221.h>
#include <math.h>

#include "yaw_config.hpp"

namespace {

// Battery current/voltage are wired to INA3221 channel 2 on the Stamp Fly HAT
// (same channel the official StampFly firmware reads for battery voltage).
constexpr ina3221_ch_t kBatteryChannel = INA3221_CH2;

// INA3221 manufacturer ID (datasheet constant), used as a presence check.
constexpr uint16_t kManufId = 0x5449;

INA3221 ina3221(INA3221_ADDR40_GND);  // I2C 0x40 (A0 -> GND)
bool g_ready = false;

}  // namespace

bool currentSensorInit(TwoWire& wire) {
    ina3221.begin(&wire);
    ina3221.reset();
    // The hardware shunt (10 mOhm) already matches the library default, but set
    // it explicitly so getCurrent() stays correct if the default ever changes.
    ina3221.setShuntRes(INA3221_SHUNT_MILLIOHM, INA3221_SHUNT_MILLIOHM, INA3221_SHUNT_MILLIOHM);
    // 電流FF補正の同期整備 (ff_pipeline_design.md §5.2 / method_v2 §5.5):
    // 電流計測チャネル(CH2)のみ有効化+変換時間を最短(140µs)に短縮し、AVG16 の
    // ハードウェア平均は維持する。実効平均窓は既定の 3ch×(1.1ms+1.1ms)×16 ≈
    // 106ms から 1ch×(140µs+140µs)×16 ≈ 4.5ms へ縮み、FF補正が磁気サンプルと
    // 同時刻の電流を見られるようになる。バス電圧(vb)も CH2 の同一変換
    // サイクルで測り続けるので cur/vb テレメトリの単位・意味は従来どおり。
    ina3221.setChannelDisable(INA3221_CH1);
    ina3221.setChannelDisable(INA3221_CH3);
    ina3221.setShuntConversionTime(INA3221_REG_CONF_CT_140US);
    ina3221.setBusConversionTime(INA3221_REG_CONF_CT_140US);
    // Average 16 samples in hardware to smooth the motor PWM ripple before a
    // value is read (continuous shunt+bus mode is the reset default).
    ina3221.setAveragingMode(INA3221_REG_CONF_AVG_16);
    g_ready = (ina3221.getManufID() == kManufId);
    return g_ready;
}

CurrentSample currentSensorRead() {
    CurrentSample sample;
    if (!g_ready) {
        return sample;
    }
    sample.shunt_uv = static_cast<float>(ina3221.getShuntVoltage(kBatteryChannel));
    sample.current_a = ina3221.getCurrent(kBatteryChannel);
    sample.bus_voltage_v = ina3221.getVoltage(kBatteryChannel);
    sample.valid = isfinite(sample.current_a) && isfinite(sample.bus_voltage_v);
    return sample;
}
