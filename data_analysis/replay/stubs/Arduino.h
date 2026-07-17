// ===========================================================================
// stubs/Arduino.h — PC(g++)直ビルド用の最小 Arduino スタブ
//
// stampfly_ecosystem の firmware/vehicle/test/esp_log.h と同じ手法:
// ファームソース(alt_kalman.cpp / yaw_estimation/*)は Arduino.h を include
// するが Arduino API は実質未使用のため、ビルドに必要な定数・型だけを補う。
// 値は arduino-esp32 の Arduino.h と同一(数値同一性のため変更しないこと)。
// ===========================================================================
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif
