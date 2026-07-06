// ===========================================================================
// indicators.cpp — LED状態表示 + 非ブロッキングブザー 実装
// ===========================================================================
#include "indicators.hpp"

#include <Arduino.h>
#include <FastLED.h>

#include "config.hpp"
#include "flight_control.hpp"
#include "sensor.hpp"
#include "stampfly_protocol.hpp"

namespace {

// --- LED ---
CRGB led_onboard[NUM_ONBOARD_LEDS];
CRGB led_esp[1];

// 配色(製品版 led.cpp のパレットを新状態へ整理。"PERPLE"等のtypoは継承しない)
constexpr uint32_t COLOR_PURPLE = 0xff00ff;       // INIT/CALIBRATION(製品版: キャリブレーション)
constexpr uint32_t COLOR_FLIGHT = 0x331155;       // TAKEOFF/HOVER(製品版: 高度制御モード)
constexpr uint32_t COLOR_GREEN = 0x00ff00;        // LANDING(製品版: 自動着陸)
constexpr uint32_t COLOR_RED = 0xff0000;          // リンク途絶警告 / COMPLETE(要RESET)
constexpr uint32_t COLOR_LOW_VOLTAGE = 0x18EBF9;  // 低電圧(製品版: POWEROFFCOLOR)
constexpr uint32_t ILLUMINATION_SEED = 255;       // WAITイルミネーションの初期色

uint16_t show_divider_counter = 0;   // FastLED.show の分周
uint16_t blink_counter = 0;          // 点滅カウンタ(400Hz tick)
uint16_t cycle_counter = 0;          // イルミネーション色送りカウンタ
uint32_t illumination_color = ILLUMINATION_SEED;

void set_onboard(uint32_t color1, uint32_t color2) {
    led_onboard[0] = color1;
    led_onboard[1] = color2;
    led_esp[0] = color1;
}

// WAIT中のイルミネーション(製品版PARKING_MODEの24bitローテーション)
void step_illumination(void) {
    cycle_counter++;
    if (cycle_counter >= FLIGHT_CONFIG.led_cycle_step_ticks) {
        cycle_counter = 0;
        if (illumination_color & 0x800000) {
            illumination_color = ((illumination_color << 1) | 1) & 0xFFFFFF;
        } else {
            illumination_color = (illumination_color << 1) & 0xFFFFFF;
        }
    }
}

// 半周期ごとにON/OFFを返す点滅(400Hz tick基準)
bool blink_on(void) {
    return blink_counter < FLIGHT_CONFIG.led_blink_period_ticks;
}

void update_leds(void) {
    const AutoFlightState st = flight_control_state.mode.auto_state;
    const bool low_v = sensor_state.Under_voltage_flag >= UNDER_VOLTAGE_COUNT;

    blink_counter++;
    if (blink_counter >= FLIGHT_CONFIG.led_blink_period_ticks * 2) blink_counter = 0;

    switch (st) {
        case AUTO_INIT:
        case AUTO_CALIBRATION:
            set_onboard(COLOR_PURPLE, COLOR_PURPLE);
            break;

        case AUTO_WAIT:
            if (low_v) {
                // 低電圧: 水色点滅(START拒否されることの予告)
                const uint32_t c = blink_on() ? COLOR_LOW_VOLTAGE : 0x000000;
                set_onboard(c, c);
            } else {
                step_illumination();
                set_onboard(illumination_color, illumination_color);
            }
            break;

        case AUTO_TAKEOFF:
        case AUTO_HOVER: {
            // 飛行中: 高度制御色。setpoint途絶(>200ms)で赤の警告。
            const auto& cs = flight_control_state.command;
            const bool link_fresh =
                cs.setpoint_received &&
                (millis() - cs.last_setpoint_ms) < FLIGHT_CONFIG.link_level_hold_ms;
            const uint32_t color = link_fresh ? COLOR_FLIGHT : COLOR_RED;
            // 製品版同様、低電圧時はLED1を低電圧色にして区別する
            set_onboard(low_v ? COLOR_LOW_VOLTAGE : color, color);
            break;
        }

        case AUTO_LANDING:
            set_onboard(COLOR_GREEN, COLOR_GREEN);
            break;

        case AUTO_COMPLETE: {
            // OverG後: 赤点滅(CMD_RESET待ち)
            const uint32_t c = blink_on() ? COLOR_RED : 0x000000;
            set_onboard(c, c);
            break;
        }
    }

    // 表示更新は分周(25Hz)で行い、制御パスの負荷を抑える
    show_divider_counter++;
    if (show_divider_counter >= FLIGHT_CONFIG.led_show_divider) {
        show_divider_counter = 0;
        FastLED.show();
    }
}

// --- 非ブロッキングブザー ---
// freq_hz == 0 は休符。シーケンスは indicators_update() が期限監視で進める。
struct ToneStep {
    uint16_t freq_hz;
    uint16_t duration_ms;
};

// 音階は製品版 buzzer.h の NOTE_D* に由来
constexpr ToneStep BOOT_MELODY[] = {{294, 200}, {441, 200}, {350, 200}, {393, 200}};
constexpr ToneStep TAKEOFF_BEEP[] = {{4000, 100}};
constexpr ToneStep LANDED_BEEP[] = {{3000, 80}, {0, 40}, {3000, 80}};
constexpr ToneStep FAILSAFE_BEEP[] = {{2500, 120}, {0, 80}, {2500, 120}};
constexpr ToneStep OVER_G_ALARM[] = {{2000, 600}};

const ToneStep* tone_seq = nullptr;  // 再生中シーケンス(nullptr=停止)
uint8_t tone_seq_len = 0;
uint8_t tone_seq_index = 0;
uint32_t tone_step_deadline_ms = 0;

void buzzer_apply_step(void) {
    const ToneStep& step = tone_seq[tone_seq_index];
    ledcWriteTone(LEDC_CH_BUZZER, step.freq_hz);  // 0Hzで消音
    tone_step_deadline_ms = millis() + step.duration_ms;
}

void buzzer_stop(void) {
    ledcWriteTone(LEDC_CH_BUZZER, 0);
    tone_seq = nullptr;
    tone_seq_len = 0;
    tone_seq_index = 0;
}

template <size_t N>
void buzzer_play(const ToneStep (&seq)[N]) {
    tone_seq = seq;
    tone_seq_len = static_cast<uint8_t>(N);
    tone_seq_index = 0;
    buzzer_apply_step();
}

// 期限が来たら次の音へ進める(毎tick呼ばれる。ブロックしない)
void buzzer_poll(void) {
    if (tone_seq == nullptr) return;
    if ((int32_t)(millis() - tone_step_deadline_ms) < 0) return;
    tone_seq_index++;
    if (tone_seq_index >= tone_seq_len) {
        buzzer_stop();
        return;
    }
    buzzer_apply_step();
}

}  // namespace

void indicators_init(void) {
    FastLED.addLeds<WS2812, PIN_LED_ONBOARD, GRB>(led_onboard, NUM_ONBOARD_LEDS);
    FastLED.addLeds<WS2812, PIN_LED_ESP, GRB>(led_esp, 1);
    FastLED.setBrightness(LED_BRIGHTNESS);

    // ブザー: GPIO40 / LEDC ch5(モータのch0-3とは別タイマ)。
    // 製品版の digitalWrite(GPIO5) バグ(=モータピン操作)は持ち込まない。
    ledcSetup(LEDC_CH_BUZZER, BUZZER_BASE_FREQ_HZ, BUZZER_PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_BUZZER, LEDC_CH_BUZZER);

    buzzer_play(BOOT_MELODY);
}

void indicators_update(void) {
    update_leds();
    buzzer_poll();
}

void indicators_notify_transition(uint8_t state, uint8_t prev_state, uint8_t reason) {
    (void)prev_state;
    using stampfly::FlightState;
    using stampfly::Reason;

    switch (static_cast<FlightState>(state)) {
        case FlightState::TAKEOFF:
            buzzer_play(TAKEOFF_BEEP);
            break;
        case FlightState::LANDING:
            // フェイルセーフ起因の着陸は警告音(STOP指令による着陸は無音)
            if (static_cast<Reason>(reason) == Reason::LOW_VOLTAGE ||
                static_cast<Reason>(reason) == Reason::MAX_FLIGHT_TIME ||
                static_cast<Reason>(reason) == Reason::LINK_LOSS) {
                buzzer_play(FAILSAFE_BEEP);
            }
            break;
        case FlightState::WAIT:
            if (static_cast<Reason>(reason) == Reason::LANDED) {
                buzzer_play(LANDED_BEEP);
            }
            break;
        case FlightState::COMPLETE:
            buzzer_play(OVER_G_ALARM);
            break;
        default:
            break;
    }
}
