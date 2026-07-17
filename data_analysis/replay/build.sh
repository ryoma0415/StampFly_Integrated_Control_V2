#!/bin/sh
# ===========================================================================
# build.sh — オフラインリプレイの Code Identity ビルド
#
# stampfly_ecosystem の eskf_sweep.py build() と同じ方式: ファームの推定器
# ソースをコピーせずパス指定で g++ 直ビルドする(Arduino.h は stubs/ の
# 最小スタブで解決)。ビルド物は build/ へ(.gitignore 済み)。
#
# 環境変数(sweep.py のシャドウビルド用):
#   FW_SRC : ファームソースの src ディレクトリ(既定 ../../firmware_stampfly/src)
#   OUT    : 出力ディレクトリ(既定 build)
#   CXX    : コンパイラ(既定 g++)
# ===========================================================================
set -e
cd "$(dirname "$0")"

FW_SRC="${FW_SRC:-../../firmware_stampfly/src}"
OUT="${OUT:-build}"
CXX="${CXX:-g++}"
CXXFLAGS="-std=c++17 -O2 -I stubs"

mkdir -p "$OUT" out

# 高度KF: alt_kalman.cpp + Filter(pid.cpp)を直リンク
$CXX $CXXFLAGS -I "$FW_SRC" \
    replay_alt.cpp "$FW_SRC/alt_kalman.cpp" "$FW_SRC/pid.cpp" \
    -o "$OUT/replay_alt"

# ヨーEKF: yaw_estimator_kf.cpp + mag_calibration.cpp を直リンク
$CXX $CXXFLAGS -I "$FW_SRC/yaw_estimation" \
    replay_yaw.cpp "$FW_SRC/yaw_estimation/yaw_estimator_kf.cpp" \
    "$FW_SRC/yaw_estimation/mag_calibration.cpp" \
    -o "$OUT/replay_yaw"

echo "build OK: $OUT/replay_alt $OUT/replay_yaw (FW_SRC=$FW_SRC)"
