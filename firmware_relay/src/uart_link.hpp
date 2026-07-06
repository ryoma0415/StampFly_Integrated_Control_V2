// ===========================================================================
// uart_link.hpp — PC⇔リレーのシリアルリンク(PROTOCOL.md「トランスポート」)
//
// RX: loopコンテキストから poll() でドライババッファを排出し、protocol層の
//     SerialFrameReceiver(0x00区切りCOBS、256B上限、エラー時フレーム全破棄
//     +カウンタ)に供給する。旧リレーのヘッダ走査・再同期ヒューリスティック
//     は存在しない。
// TX: FreeRTOSキュー+専用書き出しタスク。Serial.write を呼ぶのはこの
//     タスクだけ(単一ライタ規律)。キュー満杯時はエンキュー元をブロック
//     せず破棄+カウント。
// ===========================================================================
#pragma once

#include <cstddef>
#include <cstdint>

#include "stampfly_protocol.hpp"

namespace uart_link {

// 送信側カウンタ。send_logical() と同じ loop コンテキストからのみ
// 読み書きされる(クロスタスク共有でないため保護不要)。
struct TxCounters {
  uint32_t queue_drops = 0;    // TXキュー満杯による破棄
  uint32_t encode_errors = 0;  // COBSエンコード失敗(設計上は発生しない)
};

using FrameHandler = void (*)(const stampfly::FrameView& frame);

// シリアル初期化+TXキュー/書き出しタスク生成。setup() から1回呼ぶ。
void init();

// 受信ポンプ(loop から呼ぶ)。検証済みフレームごとに on_frame を呼ぶ。
// on_frame に渡る FrameView の payload は次フレーム復号まで有効。
void poll(FrameHandler on_frame);

// 論理フレーム(ver..crc16)をCOBS+0x00デリミタでワイヤ化しTXキューへ積む。
// loop コンテキスト専用。キュー満杯時は false(破棄+カウント、非ブロッキング)。
bool send_logical(const uint8_t* frame, size_t len);

// 受信統計(SerialFrameReceiver由来)。loopコンテキストから読むこと。
const stampfly::SerialFrameReceiver::Counters& rx_counters();
const TxCounters& tx_counters();

}  // namespace uart_link
