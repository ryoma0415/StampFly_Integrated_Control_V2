// ===========================================================================
// uart_link.cpp — シリアルリンク実装
// ===========================================================================
#include "uart_link.hpp"

#include <Arduino.h>

#include "config.hpp"

namespace uart_link {
namespace {

// TXキューの1要素 = ワイヤ形式(COBS+0x00デリミタ)の完成フレーム。
// 固定長の値コピーでキューイングする(動的確保なし)。
struct WireFrame {
  uint16_t len = 0;
  uint8_t data[stampfly::MAX_WIRE_SIZE] = {};
};

QueueHandle_t s_tx_queue = nullptr;
stampfly::SerialFrameReceiver s_receiver;  // loopコンテキスト専用
TxCounters s_tx_counters;                  // loopコンテキスト専用

// 唯一の Serial ライタ。キューから取り出して書くだけ。
// これ以外の場所で Serial.write / Serial.print を呼ぶことを禁止する
// (旧リレーのUARTインタリーブ破壊の構造的排除)。
void writer_task(void* /*arg*/) {
  WireFrame item;
  for (;;) {
    if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
      Serial.write(item.data, item.len);
    }
  }
}

}  // namespace

void init() {
  Serial.setRxBufferSize(relay_config::UART_DRIVER_RX_BUFFER);  // begin() より前に設定
  Serial.begin(relay_config::UART_BAUD);                        // 8N1(既定)

  s_tx_queue = xQueueCreate(relay_config::UART_TX_QUEUE_LEN, sizeof(WireFrame));
  configASSERT(s_tx_queue != nullptr);

  xTaskCreatePinnedToCore(
      writer_task, "uart_tx", relay_config::UART_WRITER_TASK_STACK, nullptr,
      static_cast<UBaseType_t>(relay_config::UART_WRITER_TASK_PRIORITY), nullptr,
      relay_config::UART_WRITER_TASK_CORE);
}

void poll(FrameHandler on_frame) {
  // ドライババッファを空になるまで排出(115200bps ≈ 最大11.5KB/s なので軽量)
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;
    if (s_receiver.feed(static_cast<uint8_t>(c))) {
      on_frame(s_receiver.frame());
    }
  }
}

bool send_logical(const uint8_t* frame, size_t len) {
  WireFrame item;
  size_t encoded_len = 0;
  // 末尾デリミタ分の1バイトを残してCOBSエンコード
  if (!stampfly::cobs_encode(frame, len, item.data, sizeof(item.data) - 1,
                             &encoded_len)) {
    ++s_tx_counters.encode_errors;
    return false;
  }
  item.data[encoded_len] = stampfly::COBS_DELIMITER;
  item.len = static_cast<uint16_t>(encoded_len + 1);

  // 満杯なら待たずに破棄(エンキュー元を決してブロックしない)
  if (xQueueSend(s_tx_queue, &item, 0) != pdTRUE) {
    ++s_tx_counters.queue_drops;
    return false;
  }
  return true;
}

const stampfly::SerialFrameReceiver::Counters& rx_counters() {
  return s_receiver.counters();
}

const TxCounters& tx_counters() { return s_tx_counters; }

}  // namespace uart_link
