// ===========================================================================
// flight_control.hpp — 状態機械 + カスケードPID + ミキサ
//
// 状態遷移: INIT -> CALIBRATION -> WAIT -> TAKEOFF -> HOVER -> LANDING (-> WAIT)
//           OverG で任意の飛行状態 -> COMPLETE(CMD_RESETでのみWAITへ復帰)
//           v2: WAIT --CMD_MODE(1)--> MOTOR_TEST --CMD_MODE(0)/CMD_STOP--> WAIT
// 制御則・ミキサは飛行実績のある OptiTrack版 flight_control.cpp を整理して
// 踏襲(死コード・typo・ファーム内バイアスは持ち込まない)。
// PWM の書き手は状態で排他: 飛行状態=ミキサ、MOTOR_TEST=モーターテスト
// サービス、それ以外= motor_stop のみ(二重ライター禁止)。
// ===========================================================================
#pragma once

#include "config.hpp"
#include "flight_state.hpp"

// 機体初期化(センサ・PWM・PID・通信・表示・400Hzタイマ)。setup()から1回呼ぶ。
void init_copter(void);

// 400Hz制御ループ本体。loop()から呼び続ける(内部でタイマ割り込みに同期)。
void loop_400Hz(void);

// --- v2: モーターテストの実出力(sensor.cpp の FF duty 配線と telemetry が読む) ---
// ランプ後の実効 duty(0-1。ソフトスタート/ダウン中は指令に遅れて追従する)
float motor_test_applied_duty(void);
// 駆動対象マスク(bit0=FL,1=FR,2=RL,3=RR = CmdMotorRun::MASK_*)
uint8_t motor_test_active_mask(void);
// 実出力あり(ランプダウン中の duty>0 を含む。アンカー窓の汚染防止に使う)
bool motor_test_output_active(void);

extern FlightControlState flight_control_state;
