// ===========================================================================
// flight_control.hpp — 状態機械 + カスケードPID + ミキサ
//
// 状態遷移: INIT -> CALIBRATION -> WAIT -> TAKEOFF -> HOVER -> LANDING (-> WAIT)
//           OverG で任意の飛行状態 -> COMPLETE(CMD_RESETでのみWAITへ復帰)
// 制御則・ミキサは飛行実績のある OptiTrack版 flight_control.cpp を整理して
// 踏襲(死コード・typo・ファーム内バイアスは持ち込まない)。
// ===========================================================================
#pragma once

#include "config.hpp"
#include "flight_state.hpp"

// 機体初期化(センサ・PWM・PID・通信・表示・400Hzタイマ)。setup()から1回呼ぶ。
void init_copter(void);

// 400Hz制御ループ本体。loop()から呼び続ける(内部でタイマ割り込みに同期)。
void loop_400Hz(void);

extern FlightControlState flight_control_state;
