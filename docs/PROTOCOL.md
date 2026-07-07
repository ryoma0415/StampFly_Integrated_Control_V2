# StampFly Integrated Control — 通信プロトコル仕様 v2

本書が唯一の正(single source of truth)。`protocol/stampfly_protocol.hpp`(C++)と
`protocol/stampfly_protocol.py`(Python)は本書に従い、`protocol/test_vectors.json` の
バイトベクタで両実装の一致をテストで強制する。

v2 での改定(PROTOCOL_VERSION 0x01 → **0x02**):
CMD_SETPOINT を 17B に拡張(ヨー角目標)、実験系の上りコマンド 0x14–0x23 と
下り 0x32–0x34(TLM_ACK / TLM_EXP / TLM_CAL_DATA)を追加、TLM_STATE を 135B に
末尾拡張、FlightState に MOTOR_TEST(7)、Reason に mode_change(11)を追加。
論理フレーム構造・COBS・CRC・LE 規約・型レンジルーティングは v1 と同一。
ver 不一致フレームは破棄され `ver_errors` として可視化される(新旧混在の検出)。

マルチ機体拡張(追加のみ、PROTOCOL_VERSION は **0x02 のまま**):
リレー宛/発 0x55–0x58(RLY_SET_PEERS / RLY_PEERS_ACK / RLY_MUX_UP /
RLY_MUX_DOWN)を追加し、1台のリレーで最大4機を多重制御する。エンベロープ
(MUX)は**シリアル区間のみ**に存在し、ESP-NOW 区間のバイト列は単機時と
完全に同一(**機体ファームは無改修**。リレーファームは本拡張で改修済み)。
併せてシリアル既定ボーレートを 115200 → **460800** に引き上げた
(「トランスポート」「帯域予算」参照)。

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

- `ver` = 0x02 固定(v2)。不一致フレームは破棄しカウント。
- `seq` = 送信者ごとの単調増加カウンタ。1始まりで、0xFFFFFFFF の次は 0 を飛ばして
  1 に戻る(0 は TLM_STATE.seq_echo の「未受信」番兵として予約。送信者は seq=0 の
  フレームを発行しない)。
- `len` = payloadのバイト数(0〜200)。
- `crc16` = CRC16-CCITT-FALSE(poly 0x1021, init 0xFFFF, 非反転, xorout なし)を
  `ver` から `payload` 末尾まで(crc自身を除く全バイト)に適用。リトルエンディアン格納。
  検証ベクタ: ASCII "123456789" → 0x29B1。
- マルチバイト値はすべてリトルエンディアン。float は IEEE-754 binary32 LE。

### トランスポート

- **シリアル区間(PC⇔リレー, 既定 460800 8N1)**: 論理フレーム全体をCOBSエンコードし、
  末尾に 0x00 を1バイト付加して送る。受信側は 0x00 区切りで蓄積→COBSデコード→CRC検証。
  デコード失敗/CRC不一致/ver不一致/len不整合は**フレームごと破棄**(部分回復しない)。
  受信蓄積バッファ上限 256 バイト。超過したら次の 0x00 まで読み捨て(カウンタ加算)。
  ボーレートはリレー側ビルドフラグ `RELAY_UART_BAUD`(`firmware_relay/src/config.hpp`
  既定 460800。460800 非対応の USB シリアルブリッジ向けに platformio.ini の
  `release-115200` 環境を提供)と PC 側 `server.json` の `serial.baudrate` で設定し、
  **両者を必ず一致させる**。旧既定 115200 はマルチ機体の下り帯域を満たさない
  (「帯域予算」参照)。
- **ESP-NOW区間(リレー⇔ドローン)**: ESP-NOWはフレーム境界を保存するため、COBSなしで
  論理フレームをそのままペイロードにする(≦250B)。受信時は长さ・CRC・verを検証。

## メッセージ型

type値の範囲でルーティングする(リレーは中身を解釈せず型で転送先を決める):
`0x10–0x2F` = ドローン行き(リレーが上りESP-NOWへ転送)、`0x30–0x4F` = PC行き
(リレーが下りシリアルへ転送)、`0x50–0x5F` = リレー自身宛/発。
マルチ機体モード中は素の `0x10–0x2F` を転送せず、エンベロープ経由のみ受け付ける
(「マルチ機体拡張」参照)。

### 上り(PC → ドローン)

| type | 名前 | payload | 意味 |
|------|------|---------|------|
| 0x10 | CMD_START | なし(0B) | 離陸開始。AUTO_WAIT でのみ受理(MOTOR_TEST 中は reason=10 で拒否) |
| 0x11 | CMD_STOP | なし(0B) | 即時着陸。**全飛行状態で受理**(TAKEOFF/LANDING中含む)。MOTOR_TEST 中はモーター停止→WAIT |
| 0x12 | CMD_SETPOINT | `f32 roll_ref(rad), f32 pitch_ref(rad), f32 alt_ref(m), f32 yaw_ref(rad ±π), u8 flags` =**17B**(v2) | 姿勢+高度+ヨー角目標。flags bit0=alt_ref有効(0なら現在のalt_ref維持)、**bit1=yaw_ref有効(=ヨー角制御ON)**。bit1=0 のとき機体は v1 と同一動作(Yaw_rate_reference=0 のレートダンピングのみ)。**ハートビートを兼ねる**。PCは飛行の有無に関わらずセッション中50Hzで送信 |
| 0x13 | CMD_RESET | なし(0B) | COMPLETE(OverG後)からの復帰。COMPLETE かつ altitude_est<0.15m でのみ受理 → AUTO_WAIT |

#### v2 実験・キャリブレーション系(0x14–0x23)

すべて TLM_ACK(0x32)で応答する。キャリブ/FF系(0x17–0x23)は
**WAIT / COMPLETE / MOTOR_TEST 状態でのみ受理**(飛行中の NVS 書込み禁止)。
それ以外の状態では status=bad_state。

| type | 名前 | payload | 意味 |
|------|------|---------|------|
| 0x14 | CMD_MODE | `u8 mode(0=FLIGHT,1=MOTOR_TEST)` =1B | WAIT→MOTOR_TEST(mode=1)、MOTOR_TEST→WAIT(mode=0、モーター停止後)。他状態では bad_state。同一状態への再送は冪等に ok |
| 0x15 | CMD_MOTOR_RUN | `f32 duty(0-1), u8 mask(bit0=FL,1=FR,2=RL,3=RR)` =5B | MOTOR_TEST 状態のみ(飛行状態では bad_state で破棄)。PC は 0.4s 周期で再送(キープアライブ)。機体は 1.5s 途絶で自動停止。ソフトスタート 2.0duty/s |
| 0x16 | CMD_MOTOR_STOP | なし(0B) | モーター即停止(MOTOR_TEST 内。他状態では bad_state・無害) |
| 0x17 | CMD_CAL_GET | なし(0B) | ACK(ok) の後に TLM_CAL_DATA(0x34)を返す |
| 0x18 | CMD_MAG3D_SET | `u8 valid, f32 offset[3], f32 matrix[9](行優先)` =49B | 3D磁気較正の適用/クリア(valid=0)。適用時: NVS 永続化+FF 自動無効(ff_mode=0)+アンカー破棄+ヨー推定器再シード |
| 0x19 | CMD_ACCEL6_SET | `u8 valid, f32 offset[3], f32 scale[3]` =25B | 加速度6面較正。適用時に姿勢参照リセット(ahrs_reset 含む) |
| 0x1A | CMD_ATTMOUNT_SET | `u8 valid, f32 roll_rad, f32 pitch_rad` =9B | マウントオフセット(磁気レベル化入力にのみ適用) |
| 0x1B | CMD_YAWZERO_SET | `u8 valid, f32 offset_rad` =5B | ヨーゼロ(レベル化磁気ヘディング座標系の mag_yaw_offset)の復元/クリア。**復元専用 API**: 推定ヨーは wrapPi(yaw_mag_raw − offset) になる。PC の「Yaw 0」は TLM_CAL_DATA の yawzero_offset_rad と TLM_STATE の yaw_est_rad から offset_new = wrap_pi(offset + yaw_est) を逆算して送る |
| 0x1C | CMD_GEOMAG_SET | `f32 declination_east_deg, inclination_deg, horizontal_uT, vertical_uT, total_uT` =20B | 地磁気リファレンス。NVS 永続化 |
| 0x1D | CMD_FF_BEGIN | `u8 nlut(4-24)` =1B | FF 係数ステージング開始 |
| 0x1E | CMD_FF_LUT | `u8 idx, f32 i_a, f32 db_x, db_y, db_z` =17B | LUT 点 |
| 0x1F | CMD_FF_MOT | `u8 idx(0=FL,1=FR,2=RL,3=RR), f32 a_tilde[3], f32 c2, c1, c0` =25B | モーター係数 |
| 0x20 | CMD_FF_AUX | `f32 iid_a` =4B | ベンチ参考アイドル電流 |
| 0x21 | CMD_FF_COMMIT | `u32 crc32` =4B | CRC-32(IEEE, zlib 互換、float32 LE 連結)照合 → NVS 永続化。冪等 |
| 0x22 | CMD_FF_MODE | `u8 ff_mode(0=off,1=A,2=B), u8 est_mode(0=相補,1=EKF)` =2B | 実行時切替。NVS 永続化 |
| 0x23 | CMD_FF_ANCHOR | なし(0B) | アンカー再取得要求(モーター停止中のみ。回転中/窓未充足は status=busy) |

### 下り(ドローン → PC)

| type | 名前 | payload | 意味 |
|------|------|---------|------|
| 0x30 | TLM_STATE | 下表 **135B**(v2) | フル状態テレメトリ。**25Hz**(40ms周期、400Hzループの16分周) |
| 0x31 | TLM_EVENT | `u8 state, u8 prev_state, u8 reason, u8 flags, f32 voltage` =8B | 状態遷移時に即時送信+2Hzで定期再送 |
| 0x32 | TLM_ACK | `u8 acked_type, u32 acked_seq, u8 status` =6B(v2) | 0x14–0x23 への応答。status: 0=ok, 1=bad_state, 2=invalid_arg, 3=crc_mismatch, 4=busy, 5=incomplete |
| 0x33 | TLM_EXP | 下表 86B(v2) | 実験テレメトリ。**MOTOR_TEST 状態でのみ 25Hz** 送出(TLM_STATE と 8tick 位相をずらす) |
| 0x34 | TLM_CAL_DATA | 下表 112B(v2) | CMD_CAL_GET への応答(キャリブ一括データ) |

#### TLM_STATE payload(135B、宣言順に隙間なくパック)

v2 は**末尾追加のみ**: 既存オフセット 0–96 は v1 と不変
(pc_server の serial_link.py が seq_echo を先頭オフセット直読みするため)。

| オフセット | 型 | フィールド | 単位 |
|---|---|---|---|
| 0 | u32 | seq_echo — 最後に適用した CMD_SETPOINT の seq(未受信なら0) | |
| 4 | u32 | elapsed_ms — 起動からの経過 | ms |
| 8 | u8 | state(下記enum) | |
| 9 | u8 | flags: bit0 low_voltage, bit1 setpoint_fresh(<200ms), bit2 flying | |
| 10 | u8 | reason — 直近の遷移理由(下記enum) | |
| 11 | f32×3 | roll, pitch, yaw(実測姿勢, AHRS)。yaw は ±π・リセット時 0(=離陸方位。ジャイロ積算/CF/EKF と同一基準。旧ファームは [-2π, 0] だった) | rad |
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
| 97 | f32 | yaw_est_rad — アクティブ推定器ヨー(est_mode=1 なら EKF ψ、0 なら補正CF。ff/est 未設定時はリファレンスCF) | rad |
| 101 | f32 | yaw_gyro_int_rad — Z軸角速度の単純積算(400Hz、ahrs_reset でゼロクリア) | rad |
| 105 | f32 | yaw_ref_rad — 適用中ヨー目標(途絶ラッチ後含む。ヨー制御 off 時 0) | rad |
| 109 | f32 | current_a — 総電流(INA3221 CH2、20Hz 更新) | A |
| 113 | f32 | db_hat_x_ut — FF 補正ベクトル ΔB̂ の x | µT |
| 117 | f32 | db_hat_y_ut — 同 y | µT |
| 121 | f32 | bm_x_ut — EKF 磁気バイアス状態 x | µT |
| 125 | f32 | bm_y_ut — 同 y | µT |
| 129 | f32 | nis — 直近 EKF 更新の NIS | — |
| 133 | u8 | ffg — EKF ゲート/健全性ビット(下記) | — |
| 134 | u8 | ff_status — FF/ヨー制御状態ビット(下記) | — |

`ffg` ビット(yaw側 ff_pipeline_design.md §5.5 の定義踏襲):
bit0 R_INFLATED(NIS>5.99 → R 膨張適用中)、bit1 NIS_REJECT(NIS>13.8 棄却)、
bit2 NORM_REJECT(ノルム逸脱 >20µT 棄却)、bit3 Z_REJECT(z 成分逸脱 >12µT 棄却)、
bit4 TILT_SKIP(tilt>25° スキップ)、bit5 BM_FROZEN(‖b_m‖>20µT → 磁気更新凍結、
要再アンカー)、bit6 DRIFT_WARN(|db_m/dt|>0.3µT/s 10s 継続の警告)。

`ff_status` ビット:
bit0-1 ff_mode(0-2)、bit2 est_mode(1=EKF)、bit3 anchor_valid、
bit4 ffcal_loaded、bit5 yaw_ctrl_active、bit6 mag_fresh。

#### TLM_EXP payload(86B、隙間なくパック)

| オフセット | 型 | フィールド |
|---|---|---|
| 0 | u32 | elapsed_ms |
| 4 | f32 | current_a(INA3221 CH2 総電流 [A]) |
| 8 | f32 | vbat_v [V] |
| 12 | f32 | shunt_uv [µV] |
| 16 | f32×3 | bx_raw, by_raw, bz_raw(RHALL補償+軸変換後・mag3D 前 [µT]) |
| 28 | f32×3 | bx_cal, by_cal, bz_cal(mag3D 後 [µT]) |
| 40 | f32 | imu_temp_c [℃] |
| 44 | f32×3 | roll, pitch, yaw(rad、Madgwick。yaw は ±π・リセット時 0 — TLM_STATE と同一規約) |
| 56 | f32×3 | p, q, r [rad/s] |
| 68 | f32×3 | ax, ay, az(g、フィルタ後) |
| 80 | f32 | duty_cmd(モーターテスト指令 duty 0–1) |
| 84 | u8 | motors_mask(CMD_MOTOR_RUN の mask と同ビット割り) |
| 85 | u8 | flags: bit0 current_valid, bit1 mag_fresh, bit2 motors_running |

#### TLM_CAL_DATA payload(112B)

| オフセット | 型 | フィールド |
|---|---|---|
| 0 | u8 | valid_flags: bit0 mag3d, bit1 accel6, bit2 attmount, bit3 yawzero, bit4 geomag, bit5 ffcal |
| 1 | f32×3 | mag3d_offset |
| 13 | f32×9 | mag3d_matrix(行優先) |
| 49 | f32×3 | accel6_offset |
| 61 | f32×3 | accel6_scale |
| 73 | f32 | attmount_roll_rad |
| 77 | f32 | attmount_pitch_rad |
| 81 | f32 | yawzero_offset_rad(現在の mag_yaw_offset。valid ビットに関わらず現行値) |
| 85 | f32×5 | geomag(decl_east_deg, incl_deg, H_uT, V_uT, F_uT) |
| 105 | u8 | ff_nlut |
| 106 | u32 | ff_crc32 |
| 110 | u8 | ff_mode |
| 111 | u8 | est_mode |

帯域: TLM_STATE 論理144B → COBS+デリミタ ≈146B × 25Hz ≈ 3.7KB/s
(既定 460800bps の約8%、旧 115200bps では約32%)。MOTOR_TEST 中は
TLM_EXP ≈97B × 25Hz ≈ 2.4KB/s が加わる(飛行系送信は停止しているため合計は
問題なし)。マルチ機体時の合算は「帯域予算」参照。

### ログ(双方向: リレー/ドローン → PC)

| type | 名前 | payload | 意味 |
|------|------|---------|------|
| 0x40 | LOG_TEXT | `u8 origin(0=relay,1=drone), utf-8テキスト(≦180B)` | 人間向けメッセージ。これ以外の方法でテキストを出さない |

180B を超えるテキストは送信側が **UTF-8 文字境界で**切り詰める(多バイト文字を分断
しない)。受信側(PC)は表示用途のため、不正な UTF-8 を U+FFFD に置換して受理する。

### リレー宛/発(PC ⇔ リレー)

| type | 名前 | payload | 意味 |
|------|------|---------|------|
| 0x50 | RLY_SET_TARGET | `u8 mac[6], u8 wifi_channel(1-13)` =7B | ESP-NOWピア設定(単機)。設定完了まで 0x10–0x2F の転送を拒否(LOG_TEXTで警告)。受理でマルチピア表(0x55)は無効化 |
| 0x51 | RLY_TARGET_ACK | `u8 status(0=ok,1=invalid_mac,2=peer_failed), u8 mac[6], u8 channel` =8B | SET_TARGET への応答。PCは1.0s待ち、値一致まで最大3回再送 |
| 0x52 | RLY_STATS | `u32 up_frames, u32 down_frames, u32 crc_errors, u32 cobs_errors, u32 espnow_send_fail, u32 overflow_drops` =24B | 1Hzで自動送信 |
| 0x53 | RLY_PING | なし | 疎通確認 |
| 0x54 | RLY_PONG | `u32 echo_seq(PINGのseq)` =4B | PING応答 |
| 0x55 | RLY_SET_PEERS | 下表 **可変 2+7×N B**(N=0..4) | マルチ機体ピア設定。count=0 で解除。受理で単機ターゲット(0x50)は無効化 |
| 0x56 | RLY_PEERS_ACK | 下表 =4B | SET_PEERS への応答。PCは1.0s待ち、値一致まで最大3回再送(0x51 と同一規律) |
| 0x57 | RLY_MUX_UP | `u8 node_id, 内側論理フレーム(ver..crc16)` =可変 1+内側長 B | PC→リレー: 機体 node_id 宛エンベロープ |
| 0x58 | RLY_MUX_DOWN | `u8 node_id, 内側論理フレーム(ver..crc16)` =可変 1+内側長 B | リレー→PC: 機体 node_id 発エンベロープ |

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

### マルチ機体拡張(0x55–0x58)

1台のリレーで最大 `RLY_MAX_PEERS`=**4機**を多重制御する追加仕様
(PROTOCOL_VERSION は 0x02 のまま)。エンベロープ(MUX)は**シリアル区間のみ**に
存在する: リレーは RLY_MUX_UP から内側フレームを取り出して peers[node_id] へ
そのまま ESP-NOW 送信し、ピア発の下りフレームを RLY_MUX_DOWN で包んで PC へ送る。
**ESP-NOW 区間のバイト列は単機時と完全に同一**のため、機体ファームは無改修。

#### RLY_SET_PEERS payload(可変長 2+7×N B、N=0..4)

| オフセット | 型 | フィールド | 意味 |
|---|---|---|---|
| 0 | u8 | count | 登録ピア数(0–4。超過は status=bad_count)。**0 でマルチモード解除**(以降のバイトなし=計2B) |
| 1 | u8 | wifi_channel | 全ピア共有のWiFiチャネル(1–13。範囲外は status=bad_channel。count=0 のときは 0 を許容)。無線は1つのため機体ごとに変えられない |
| 2+7i | u8[6] | mac[i] | ピア i の MAC(ユニキャスト必須・重複禁止。違反は status=invalid_mac)。**エントリ index i がそのまま node_id** |
| 8+7i | u8 | tlm_state_div[i] | ピア i の TLM_STATE 間引き(1=全転送、n=1/n 転送。0 は 1 扱い) |

- 単機ターゲット(RLY_SET_TARGET)とは**排他**: どちらか一方の受理でもう一方は
  無効化される(モード切替時に全ESP-NOWピアを削除してから再登録)。
- 間引きは **TLM_STATE(0x30)のみ**に適用する。TLM_ACK / TLM_EVENT / TLM_EXP /
  TLM_CAL_DATA は制御上重要なため間引かない(意図的破棄は内部カウンタ
  tlm_decimated に計上)。SET_PEERS 受理で間引き位相はリセットされる。

#### RLY_PEERS_ACK payload(4B)

| オフセット | 型 | フィールド | 意味 |
|---|---|---|---|
| 0 | u8 | status | 0=ok, 1=invalid_mac(非ユニキャスト/重複), 2=peer_failed, 3=bad_count, 4=bad_channel |
| 1 | u8 | count | 受理したピア数のエコー |
| 2 | u8 | wifi_channel | チャネルのエコー |
| 3 | u8 | failed_index | 失敗したエントリの index(なければ 0xFF) |

PC は RLY_SET_TARGET と同じ再送規律で待ち合わせる: **1.0s 待ち、status=ok かつ
count / wifi_channel 一致まで最大3回再送**(初回送信+最大3回再送=計最大4回送信)。

#### RLY_MUX_UP / RLY_MUX_DOWN payload(可変長 1+内側フレーム長)

| オフセット | 型 | フィールド | 意味 |
|---|---|---|---|
| 0 | u8 | node_id | 0–3(RLY_SET_PEERS のエントリ index) |
| 1 | — | inner | **完全な内側論理フレーム**(ver..crc16、COBSなし)をそのまま格納 |

内側フレームの最大ペイロードは `MAX_MUX_INNER_PAYLOAD` = 200−1−9 = **190B**
(既存最大の TLM_STATE 135B が収まる)。PC 実装は外側エンベロープと内側フレームで
**同一 seq を共有**する(採番は1回。機体がエコーする seq_echo / acked_seq は
内側 seq — ノード別レイテンシ計測の対応付けに使う)。

マルチモード中のリレーの規範動作:

- **上り(RLY_MUX_UP)**: 内側フレームを parse して検証(CRC/ver/len)し、
  **上り型(0x10–0x2F)のみ**許可してから peers[node_id] へ内側バイト列を
  そのまま ESP-NOW 送信する(下り/リレー型を誤って機体へ送る事故を構造的に防ぐ)。
  不正な内側フレーム・範囲外 node_id・マルチモード非アクティブ時の MUX_UP は
  拒否(レート制限つき LOG_TEXT 警告)。
- **素の上りフレーム(0x10–0x2F、非エンベロープ)は拒否**: 宛先が曖昧なため
  転送しない(レート制限つき LOG_TEXT 警告)。単機ターゲットモードの動作は
  従来どおり不変。
- **下り帰属(RLY_MUX_DOWN)**: ESP-NOW 受信フレームは送信元 MAC でピア表を
  逆引きして node_id に帰属させ、RLY_MUX_DOWN で包んで PC へ送る。登録外 MAC
  からのフレームは転送しない(`rx_filtered` に計上)。単機モードの下りは
  従来どおり素通し(エンベロープなし)。

## enum定義

```
FlightState: 0=INIT, 1=CALIBRATION, 2=WAIT, 3=TAKEOFF, 4=HOVER, 5=LANDING, 6=COMPLETE,
             7=MOTOR_TEST(v2)
Reason:      0=none, 1=start_cmd, 2=stop_cmd, 3=max_flight_time, 4=low_voltage,
             5=start_rejected_low_voltage, 6=landed, 7=over_g, 8=link_loss, 9=reset_cmd,
             10=start_rejected_not_ready, 11=mode_change(v2: CMD_MODE による
             WAIT<->MOTOR_TEST 遷移)
```

## タイミング・フェイルセーフ(規範)

| 条件 | 動作 | 実装場所 |
|---|---|---|
| CMD_SETPOINT 途絶 >200ms(飛行中) | roll/pitch を水平(バイアス込み0)へ、alt_ref 維持。**yaw はヨー制御中なら途絶検出時点の推定ヨー角をラッチして保持**(v2。0 指令へ落とすと離陸方位への回頭を意味するため)。復帰で通常追従へ戻る | ファーム |
| CMD_SETPOINT 途絶 >500ms(飛行中) | LANDING へ遷移(reason=8 link_loss)。auto_landing_step のヨーはレートダンピングのみ | ファーム |
| EKF 不健全(磁気更新凍結 ffg bit5 / アンカー無効 / FF無効)で est_mode=1 | ヨー角制御を止めてレートダンピングに縮退(飛行中のヨーソース切替による指令段差を作らない)。ffg / ff_status で PC に通知 | ファーム(v2) |
| CMD_MOTOR_RUN 途絶 >1.5s(MOTOR_TEST 中) | モーター自動停止 | ファーム(v2) |
| 低電圧(<3.34V、20Hz×5サンプル連続 ≈0.25s) | 飛行中: LANDING(reason=4)。WAIT: START拒否(reason=5)(v2: 電圧源は INA3221 の 20Hz 読みに一本化) | ファーム |
| OverG(>2.0g) | モータ即停止 → COMPLETE(reason=7)。CMD_RESETでのみ復帰 | ファーム |
| 最大飛行時間 120s | LANDING(reason=3) | ファーム |
| MoCap 途絶 >300ms(Positionモード) | setpoint を水平に固定+UI警告(円軌道中は軌道位相・接線ヨー目標も凍結) | pc_server |
| MoCap 途絶 >2s(Positionモード) | CMD_STOP 送信(自動着陸)。円軌道中も同一 | pc_server |
| STOP 送信後 600ms 以内に LANDING/WAIT イベントなし | CMD_STOP 再送(最大3回)+UI警告 | pc_server |
| シリアル切断 | UI赤色警告(機体側は上記リンク喪失で自律着陸。MOTOR_TEST 中は CMD_MOTOR_RUN 途絶停止) | pc_server |

マルチ機体モード(pc_server MODE_MULTI)では上記の PC 側規範を
**機体ごとに独立適用**する: ノード宛 CMD_SETPOINT を機体ごとに 50Hz で送信
(ハートビート兼用 — 機体側の 200ms/500ms 途絶フェイルセーフは単機時と同一)、
STOP 送信後 600ms×最大3回の再送監視もノード別、MoCap 途絶 >300ms 水平固定 /
>2s CMD_STOP も機体別に判定する(途絶した機体のみ停止し他機は継続)。
緊急停止(SPACE / STOP)は**全機一斉** CMD_STOP。

レート規範: CMD_SETPOINT 50Hz(PC送信、experiment モード中は停止。
マルチ機体時は**ノードごとに** 50Hz)/
TLM_STATE 25Hz / TLM_EVENT 即時+2Hz / TLM_EXP 25Hz(MOTOR_TEST のみ)/
CMD_MOTOR_RUN 0.4s キープアライブ / RLY_STATS 1Hz。
シリアル使用率は下記「帯域予算」を参照。

### 帯域予算(規範)

ワイヤ長の内訳(COBS は 254B 以下の入力に対し +1B、デリミタ +1B):
TLM_STATE = 135B payload + 9B フレーム + 2B ≈ **146B**。マルチ機体時は
MUX エンベロープ(node_id 1B + 外側フレーム 9B + COBS/デリミタ増分)で
**156B**。CMD_SETPOINT = 17B payload → **28B**、MUX 時 **38B**。
UART は全二重のため上り/下りは方向別に評価する
(115200bps ≈11.5KB/s、460800bps ≈46KB/s)。

| 構成 | 下り(TLM_STATE 25Hz/機) | @115200 | @460800 | 上り(CMD_SETPOINT 50Hz/機) | @115200 | @460800 |
|---|---|---|---|---|---|---|
| 単機(非MUX) | ≈3.7KB/s | 32% | 8% | ≈1.4KB/s | 12% | 3% |
| 2機(MUX) | ≈7.8KB/s | 68% | 17% | ≈3.8KB/s | 33% | 8% |
| 3機(MUX) | ≈11.7KB/s | **102%(不可)** | 25% | ≈5.7KB/s | 50% | 12% |
| 4機(MUX) | ≈15.6KB/s | **135%(不可)** | 34% | ≈7.6KB/s | 66% | 17% |

TLM_EVENT / RLY_STATS / LOG_TEXT の寄与は 0.1KB/s 未満で無視できる。
既定 460800 では 4機でも下り約34%・上り約17%と十分な余裕がある。
460800 非対応ハードウェア(`release-115200`)では **2機以下**とし、
`tlm_state_div=2`(server.json `multi.tlm_state_div`)で下りを半減
(2機 ≈3.9KB/s ≈34%)して運用する。3機以上は 115200 では下りが破綻する。

## ドローン側の受理規則

- 受信フレームは len==期待値 かつ CRC一致 かつ ver==2 のもののみ受理。
- ブート後最初の有効上りフレームの送信元MACをリレーピアとして学習(以後不変)。
- 受信コールバック(WiFiタスク)は検証+portMUXクリティカルセクションでメールボックスに
  格納するだけ。400Hzループがスナップショットを取り出して消費。優先度 STOP > START >
  RESET > SETPOINT。0x14–0x23 のコマンドは専用リングバッファに積み、400Hz ループが
  順に処理して TLM_ACK を返す。
- キャリブ/FF系(0x17–0x23)は WAIT / COMPLETE / MOTOR_TEST でのみ受理
  (NVS 書込みは非飛行状態限定)。CMD_MOTOR_RUN / CMD_MOTOR_STOP は MOTOR_TEST 限定。
- WiFiチャネルはファームconfigの固定値(既定1)に `esp_wifi_set_channel` でピン留めする。
  機体プロファイル(PC側)のチャネルと一致させる。製品版ジョイスティック(CH3)とは別チャネル。

## テストベクタ(test_vectors.json に収録、両言語でアサート)

最低限含めるもの:
1. CRC16: "123456789" → 0x29B1。
2. CMD_SETPOINT: seq=0x41424344(旧バグ回帰オマージュ)、roll=0.0524, pitch=-0.0349,
   alt=0.30 を含む 17B ペイロードの論理フレーム全バイトとCOBS後ワイヤバイト
   (yaw_ref 有効/無効の両方)。
3. payload に 0x00 を多数含むフレーム(alt_ref=0.0 等)の COBS 往復。
4. TLM_STATE: 全フィールド既知値の135Bペイロード+フレーム全バイト。
5. v2 新規メッセージ全型: CMD_MODE / CMD_MOTOR_RUN / CMD_MOTOR_STOP / CMD_CAL_GET /
   CMD_MAG3D_SET / CMD_ACCEL6_SET / CMD_ATTMOUNT_SET / CMD_YAWZERO_SET /
   CMD_GEOMAG_SET / CMD_FF_BEGIN / CMD_FF_LUT / CMD_FF_MOT / CMD_FF_AUX /
   CMD_FF_COMMIT / CMD_FF_MODE / CMD_FF_ANCHOR / TLM_ACK / TLM_EXP / TLM_CAL_DATA。
6. 破損系: CRC1ビット反転→破棄(crc_errors)、デリミタ欠落→次フレームと連結され、
   COBSデコード自体は構造的に成功する(連結境界では code≠0xFF かつ入力が続くため
   暗黙の0x00が挿入されるだけでデコードエラーにならない)が、復元バッファが len
   フィールドと不整合になりフレーム検証で破棄(len_errorsに計上)→両方破棄、
   256B超→読み捨て(overflow_drops)、**ver=0x01 の旧フレーム→破棄(ver_errors)**。
   注意: デリミタ欠落は cobs_errors には**現れない**。現場でカウンタを読むときは
   len_errors(リレーのRLY_STATSでは crc_errors への合算側)を見ること。
7. LOG_TEXT: 多バイト UTF-8(日本語+非BMP文字)テキストのフレーム全バイト、および
   UTF-8 文字境界切り詰め(`utf8_truncate_len`)の入出力ベクタ。
8. マルチ機体拡張(名前固定): `rly_set_peers_two`(2ピア・div混在)/
   `rly_set_peers_clear`(count=0 解除)/ `rly_peers_ack_ok` /
   `rly_mux_up_setpoint_node1`(CMD_SETPOINT 内包)/
   `rly_mux_down_event_node3`(TLM_EVENT 内包)。可変長 RLY_SET_PEERS の2態と
   MUX 両方向について、内側フレーム・論理フレーム・COBS後ワイヤバイトを収録。
