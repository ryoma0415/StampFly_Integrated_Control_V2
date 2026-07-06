# StampFly Integrated Control — 通信プロトコル仕様 v1

本書が唯一の正(single source of truth)。`protocol/stampfly_protocol.hpp`(C++)と
`protocol/stampfly_protocol.py`(Python)は本書に従い、`protocol/test_vectors.json` の
バイトベクタで両実装の一致をテストで強制する。

## 設計原則(旧システムの教訓)

旧プロトコルは「ヘッダバイト走査+固定長+8bit加算和」で、ペイロード内の 0x41 を
フレーム開始と誤認するバグ、再同期パスのバッファオーバーフロー、デバッグテキストと
バイナリの同一UART混在によるフレーム破壊を生んだ。本設計はその原因を構造的に排除する:

1. **COBSフレーミング**(シリアル区間): フレームは 0x00 デリミタで区切る。ペイロード内に
   0x00 は現れない(COBSが保証)ため、再同期は「次の 0x00 まで読み捨て」だけで完了する。
   ヘッダ走査・再同期ヒューリスティックは存在しない。
2. **CRC16-CCITT-FALSE**: 誤受理率 ~2^-16(旧: 1/256)。
3. **長さフィールド**を持ち、型ごとの暗黙固定長に依存しない。
4. **テキストはフレーム化**(LOG_TEXT型)。データUARTに生テキストを書くことを禁止する。
5. **UART書き出しは単一ライタ**: リレー/ドローンともTXはキュー+専用書き出しタスク経由。
   ESP-NOW受信コールバック(WiFiタスク)から直接 Serial.write しない。

## 論理フレーム(両ホップ共通)

```
+--------+--------+----------------+--------+------------------+-----------+
| ver(1) | type(1)| seq(4, u32 LE) | len(1) | payload(len byte)| crc16(2 LE)|
+--------+--------+----------------+--------+------------------+-----------+
```

- `ver` = 0x01 固定。不一致フレームは破棄しカウント。
- `seq` = 送信者ごとの単調増加カウンタ。1始まりで、0xFFFFFFFF の次は 0 を飛ばして
  1 に戻る(0 は TLM_STATE.seq_echo の「未受信」番兵として予約。送信者は seq=0 の
  フレームを発行しない)。
- `len` = payloadのバイト数(0〜200)。
- `crc16` = CRC16-CCITT-FALSE(poly 0x1021, init 0xFFFF, 非反転, xorout なし)を
  `ver` から `payload` 末尾まで(crc自身を除く全バイト)に適用。リトルエンディアン格納。
  検証ベクタ: ASCII "123456789" → 0x29B1。
- マルチバイト値はすべてリトルエンディアン。float は IEEE-754 binary32 LE。

### トランスポート

- **シリアル区間(PC⇔リレー, 115200 8N1)**: 論理フレーム全体をCOBSエンコードし、
  末尾に 0x00 を1バイト付加して送る。受信側は 0x00 区切りで蓄積→COBSデコード→CRC検証。
  デコード失敗/CRC不一致/ver不一致/len不整合は**フレームごと破棄**(部分回復しない)。
  受信蓄積バッファ上限 256 バイト。超過したら次の 0x00 まで読み捨て(カウンタ加算)。
- **ESP-NOW区間(リレー⇔ドローン)**: ESP-NOWはフレーム境界を保存するため、COBSなしで
  論理フレームをそのままペイロードにする(≦250B)。受信時は长さ・CRC・verを検証。

## メッセージ型

type値の範囲でルーティングする(リレーは中身を解釈せず型で転送先を決める):
`0x10–0x2F` = ドローン行き(リレーが上りESP-NOWへ転送)、`0x30–0x4F` = PC行き
(リレーが下りシリアルへ転送)、`0x50–0x5F` = リレー自身宛/発。

### 上り(PC → ドローン)

| type | 名前 | payload | 意味 |
|------|------|---------|------|
| 0x10 | CMD_START | なし(0B) | 離陸開始。AUTO_WAIT でのみ受理 |
| 0x11 | CMD_STOP | なし(0B) | 即時着陸。**全飛行状態で受理**(TAKEOFF/LANDING中含む) |
| 0x12 | CMD_SETPOINT | `f32 roll_ref(rad), f32 pitch_ref(rad), f32 alt_ref(m), u8 flags` =13B | 姿勢+高度目標。flags bit0=alt_ref有効(0なら現在のalt_ref維持)。**ハートビートを兼ねる**。PCは飛行の有無に関わらずセッション中50Hzで送信 |
| 0x13 | CMD_RESET | なし(0B) | COMPLETE(OverG後)からの復帰。COMPLETE かつ altitude_est<0.15m でのみ受理 → AUTO_WAIT |

### 下り(ドローン → PC)

| type | 名前 | payload | 意味 |
|------|------|---------|------|
| 0x30 | TLM_STATE | 下表 97B | フル状態テレメトリ。**25Hz**(40ms周期、400Hzループの16分周) |
| 0x31 | TLM_EVENT | `u8 state, u8 prev_state, u8 reason, u8 flags, f32 voltage` =8B | 状態遷移時に即時送信+2Hzで定期再送 |

#### TLM_STATE payload(97B、宣言順に隙間なくパック)

| オフセット | 型 | フィールド | 単位 |
|---|---|---|---|
| 0 | u32 | seq_echo — 最後に適用した CMD_SETPOINT の seq(未受信なら0) | |
| 4 | u32 | elapsed_ms — 起動からの経過 | ms |
| 8 | u8 | state(下記enum) | |
| 9 | u8 | flags: bit0 low_voltage, bit1 setpoint_fresh(<200ms), bit2 flying | |
| 10 | u8 | reason — 直近の遷移理由(下記enum) | |
| 11 | f32×3 | roll, pitch, yaw(実測姿勢, AHRS) | rad |
| 23 | f32×3 | p, q, r(実測角速度) | rad/s |
| 35 | f32×2 | roll_ref, pitch_ref(適用中の指令) | rad |
| 43 | f32 | alt_ref(適用中の目標高度) | m |
| 47 | f32×2 | altitude_tof, altitude_est(ToF生値, カルマン推定) | m |
| 55 | f32 | alt_velocity | m/s |
| 59 | f32 | z_dot_ref | m/s |
| 63 | f32 | voltage | V |
| 67 | f32×4 | duty_fr, duty_fl, duty_rr, duty_rl | 0–1 |
| 83 | f32×3 | ax, ay, az(フィルタ後加速度) | g |
| 95 | u16 | loop_dt_us(直近の実測制御周期) | µs |

帯域: 論理106B → COBS+デリミタ ≈108B × 25Hz ≈ 2.7KB/s(115200bpsの23%)。

### ログ(双方向: リレー/ドローン → PC)

| type | 名前 | payload | 意味 |
|------|------|---------|------|
| 0x40 | LOG_TEXT | `u8 origin(0=relay,1=drone), utf-8テキスト(≦180B)` | 人間向けメッセージ。これ以外の方法でテキストを出さない |

180B を超えるテキストは送信側が **UTF-8 文字境界で**切り詰める(多バイト文字を分断
しない)。受信側(PC)は表示用途のため、不正な UTF-8 を U+FFFD に置換して受理する。

### リレー宛/発(PC ⇔ リレー)

| type | 名前 | payload | 意味 |
|------|------|---------|------|
| 0x50 | RLY_SET_TARGET | `u8 mac[6], u8 wifi_channel(1-13)` =7B | ESP-NOWピア設定。設定完了まで 0x10–0x2F の転送を拒否(LOG_TEXTで警告) |
| 0x51 | RLY_TARGET_ACK | `u8 status(0=ok,1=invalid_mac,2=peer_failed), u8 mac[6], u8 channel` =8B | SET_TARGET への応答。PCは1.0s待ち、値一致まで最大3回再送 |
| 0x52 | RLY_STATS | `u32 up_frames, u32 down_frames, u32 crc_errors, u32 cobs_errors, u32 espnow_send_fail, u32 overflow_drops` =24B | 1Hzで自動送信 |
| 0x53 | RLY_PING | なし | 疎通確認 |
| 0x54 | RLY_PONG | `u32 echo_seq(PINGのseq)` =4B | PING応答 |

#### RLY_STATS のカウンタ集計規則(規範)

RLY_STATS の欄数は限られるため、リレーは内部カウンタを次の規則で合算する
(実装: `firmware_relay/src/router.cpp` の `emit_stats()`)。PC側オペレータは
1Hz統計をこの対応で読むこと:

- `crc_errors` = CRC不一致 + **ver不一致 + len不整合**(シリアルRX・ESP-NOW RXの両方)。
  検証エラー欄はこの1つだけのため、検証起因の破棄をここに漏れなく合算する。
  デリミタ欠落による連結破棄(テストベクタ5、内部では len_errors)もここに現れる。
- `cobs_errors` = シリアルRXのCOBSデコード失敗のみ。
- `espnow_send_fail` = ESP-NOW送信失敗のみ。
- `overflow_drops` = 容量起因の破棄の合算: シリアルRX蓄積バッファ256B超過 +
  UART TXキュー満杯 + TX時COBSエンコード失敗 + ESP-NOW RXキュー満杯。

## enum定義

```
FlightState: 0=INIT, 1=CALIBRATION, 2=WAIT, 3=TAKEOFF, 4=HOVER, 5=LANDING, 6=COMPLETE
Reason:      0=none, 1=start_cmd, 2=stop_cmd, 3=max_flight_time, 4=low_voltage,
             5=start_rejected_low_voltage, 6=landed, 7=over_g, 8=link_loss, 9=reset_cmd,
             10=start_rejected_not_ready
```

## タイミング・フェイルセーフ(規範)

| 条件 | 動作 | 実装場所 |
|---|---|---|
| CMD_SETPOINT 途絶 >200ms(飛行中) | roll/pitch を水平(バイアス込み0)へ、alt_ref 維持 | ファーム |
| CMD_SETPOINT 途絶 >500ms(飛行中) | LANDING へ遷移(reason=8 link_loss) | ファーム |
| 低電圧(<3.34V 100tick連続) | 飛行中: LANDING(reason=4)。WAIT: START拒否(reason=5) | ファーム |
| OverG(>2.0g) | モータ即停止 → COMPLETE(reason=7)。CMD_RESETでのみ復帰 | ファーム |
| 最大飛行時間 120s | LANDING(reason=3) | ファーム |
| MoCap 途絶 >300ms(Positionモード) | setpoint を水平に固定+UI警告 | pc_server |
| MoCap 途絶 >2s(Positionモード) | CMD_STOP 送信(自動着陸) | pc_server |
| STOP 送信後 600ms 以内に LANDING/WAIT イベントなし | CMD_STOP 再送(最大3回)+UI警告 | pc_server |
| シリアル切断 | UI赤色警告(機体側は上記リンク喪失で自律着陸) | pc_server |

レート規範: CMD_SETPOINT 50Hz(PC送信)/ TLM_STATE 25Hz / TLM_EVENT 即時+2Hz /
RLY_STATS 1Hz。シリアル合計使用率 ≈30%(上り≈1.2KB/s、下り≈3.0KB/s)。

## ドローン側の受理規則

- 受信フレームは len==期待値 かつ CRC一致 かつ ver==1 のもののみ受理。
- ブート後最初の有効上りフレームの送信元MACをリレーピアとして学習(以後不変)。
- 受信コールバック(WiFiタスク)は検証+portMUXクリティカルセクションでメールボックスに
  格納するだけ。400Hzループがスナップショットを取り出して消費。優先度 STOP > START >
  RESET > SETPOINT。
- WiFiチャネルはファームconfigの固定値(既定1)に `esp_wifi_set_channel` でピン留めする。
  機体プロファイル(PC側)のチャネルと一致させる。製品版ジョイスティック(CH3)とは別チャネル。

## テストベクタ(test_vectors.json に収録、両言語でアサート)

最低限含めるもの:
1. CRC16: "123456789" → 0x29B1。
2. CMD_SETPOINT: seq=0x41424344(旧バグ回帰オマージュ)、roll=0.0524, pitch=-0.0349,
   alt=0.30, flags=1 の論理フレーム全バイトとCOBS後ワイヤバイト。
3. payload に 0x00 を多数含むフレーム(alt_ref=0.0 等)の COBS 往復。
4. TLM_STATE: 全フィールド既知値の97Bペイロード+フレーム全バイト。
5. 破損系: CRC1ビット反転→破棄(crc_errors)、デリミタ欠落→次フレームと連結され、
   COBSデコード自体は構造的に成功する(連結境界では code≠0xFF かつ入力が続くため
   暗黙の0x00が挿入されるだけでデコードエラーにならない)が、復元バッファが len
   フィールドと不整合になりフレーム検証で破棄(len_errorsに計上)→両方破棄、
   256B超→読み捨て(overflow_drops)。
   注意: デリミタ欠落は cobs_errors には**現れない**。現場でカウンタを読むときは
   len_errors(リレーのRLY_STATSでは crc_errors への合算側)を見ること。
6. LOG_TEXT: 多バイト UTF-8(日本語+非BMP文字)テキストのフレーム全バイト、および
   UTF-8 文字境界切り詰め(`utf8_truncate_len`)の入出力ベクタ。
