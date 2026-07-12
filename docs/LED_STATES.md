# LED 状態表示一覧(機体 LED)

機体(StampFly)のオンボード LED 2 個+StampS3 本体 LED 1 個の状態表示の正典。
実装は `firmware_stampfly/src/indicators.cpp`(400Hz tick で更新、`FastLED.show()` は
25Hz に分周)。通常は 3 個とも同色だが、飛行中の低電圧時のみ LED1 が別色になる
(下表参照)。リレー(firmware_relay / M5Stamp)には LED 表示は実装していない。

点滅の周期は `config.hpp` の `led_blink_period_ticks = 200`(400Hz tick)由来で、
**半周期 0.5 秒(1Hz 点滅)**。

## 状態 → 色の一覧

| 状態(AutoFlightState) | 色 | パターン | 意味 | 遷移条件 |
|---|---|---|---|---|
| INIT / CALIBRATION | `0xff00ff` マゼンタ(紫) | 常灯 | 起動直後の初期化・ジャイロ等センサ較正中 | 電源 ON → 較正完了で WAIT へ |
| WAIT(通常) | レインボー | イルミネーション(24bit 色ローテーション) | 待機中。START 受理可 | CMD_START で TAKEOFF、CMD_MODE(1) で MOTOR_TEST |
| WAIT(低電圧) | `0x18EBF9` 水色 | 点滅(1Hz) | 電池低電圧。**START は拒否される**ことの予告 | 電池交換で通常表示へ |
| TAKEOFF / HOVER(リンク正常) | `0x331155` 紫紺(暗い紫) | 常灯 | 飛行中(セットポイント受信が新鮮) | — |
| TAKEOFF / HOVER(リンク途絶) | `0xff0000` 赤 | 常灯 | セットポイント途絶(>200ms、`link_level_hold_ms`)の警告 | 受信再開で紫紺へ。継続すればフェイルセーフ着陸 |
| TAKEOFF / HOVER(低電圧) | LED1 のみ `0x18EBF9` 水色 | 常灯 | 飛行中の低電圧(LED2 は上記の飛行色のまま) | 低電圧継続でフェイルセーフ着陸 |
| LANDING | `0x00ff00` 緑 | 常灯 | 自動着陸中 | 着地判定で WAIT へ |
| COMPLETE | `0xff0000` 赤 | 点滅(1Hz) | OverG 等で停止。**CMD_RESET 待ち** | CMD_RESET で WAIT へ |
| MOTOR_TEST(通常) | `0xff8800` 橙 | 点滅(1Hz) | ベンチ実験モード。プロペラが回り得る注意喚起 | CMD_MODE(0) で WAIT へ |
| MOTOR_TEST(**計測中**) | `0xff00ff` マゼンタ | **常灯** | Experiment の計測(explog 記録)中。下記「計測中インジケータ」参照 | 計測停止・フェイルセーフで橙点滅へ |

## 計測中インジケータ(CMD_LED_MODE、v2.2)

`CMD_LED_MODE`(0x25、`u8 mode` 0=AUTO / 1=RECORDING)で制御するオーバーレイ表示。
詳細なワイヤ仕様は [PROTOCOL.md](PROTOCOL.md) の 0x25 を参照。

- **表示**: mode=1 かつ **MOTOR_TEST 中のみ** LED をマゼンタ `0xff00ff` **常灯**にする。
  点滅させないのは、スマホ動画から「変化の瞬間」を 1 フレーム単位で検出するため。
- **同期規約**: **「LED がマゼンタに変わった瞬間 = 計測開始(explog の t_s=0)=
  スマホ動画のカット位置」**。動画はこの瞬間で頭をカットしてから
  `data_analysis/plot_explog.py` のアニメーション同期に使う。
- **PC 側の運用**: pc_server は計測開始(exp_record_start)成功時に mode=1 を送信し、
  計測中は約 1 秒間隔で再送(キープアライブ)、計測停止で mode=0 を送る。
- **フェイルセーフ**(常灯が残らないための二重の解除):
  1. 最後の mode=1 受信から **3 秒**(`config.hpp` の `led_recording_failsafe_ms=3000`)
     で自動的に AUTO(橙点滅)へ復帰 — リンク断・PC 異常終了対策。
  2. **MOTOR_TEST 離脱で即 AUTO** — 状態遷移側の取りこぼし防止。
- **色の重複について**: マゼンタ `0xff00ff` は INIT/CALIBRATION と同値だが、
  INIT/CALIBRATION は起動直後のみで MOTOR_TEST と同時には成立しない。
  計測ウィンドウ(MOTOR_TEST 中)ではマゼンタ=計測中が一意に定まる。

## ブザー(参考)

LED と同じ `indicators.cpp` が非ブロッキングで再生する。

| イベント | 音 |
|---|---|
| 起動 | 4 音メロディ |
| 離陸(TAKEOFF 遷移) | 4kHz 単音 100ms |
| 着地(WAIT 復帰、reason=LANDED) | 3kHz × 2 |
| フェイルセーフ着陸(低電圧/最大飛行時間/リンク断) | 2.5kHz × 2 |
| COMPLETE(OverG) | 2kHz 600ms |
