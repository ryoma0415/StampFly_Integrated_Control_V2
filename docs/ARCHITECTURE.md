# StampFly Integrated Control — アーキテクチャ仕様 v1

本書は各コンポーネントの責務・境界・API・コーディング規約を定める契約文書。
通信のワイヤ仕様は `PROTOCOL.md` が正。

## 中心原則: ファームウェアは1本、モードはPC側のみ

機体は常に「roll/pitch角度 + 目標高度のセットポイント追従機」として動作する。
- **Postureモード** = UIのスライダ値 → そのまま CMD_SETPOINT
- **Positionモード** = NatNet位置 → フィルタ → XY PID → CMD_SETPOINT
モード切替にファーム書き換え・再起動は不要。機体差(角度バイアス)は**PC側の機体
プロファイルで指令に加算**し、ファームに機体固有定数を置かない。

## フォルダ構成

```
StampFly_Integrated_Control/
├── docs/                  PROTOCOL.md, ARCHITECTURE.md, README.md(セットアップ+運用手順)
├── protocol/              プロトコル単一真実の実装
│   ├── stampfly_protocol.hpp   ヘッダオンリーC++。Arduino非依存(純粋C++17)。
│   │                           COBS, CRC16, フレームpack/parse, 全enum/struct
│   ├── stampfly_protocol.py    同等のPython実装
│   ├── test_vectors.json       共有バイトベクタ
│   └── tests/                  pytest(Python往復+ベクタ) と host_test.cpp(g++でコンパイル
│                               しベクタを検証、pytestから subprocess で実行)
├── firmware_stampfly/     機体ファーム(PlatformIO, board: esp32-s3-devkitc-1)
│   ├── platformio.ini     release(-O2, CORE_DEBUG_LEVEL=0) / debug(-Og -g3) の2env。
│   │                      build_flags に -I../protocol
│   ├── src/main.cpp       製品版同様 ~40行: setup()→init_copter(), loop()→loop_400Hz()
│   ├── src/config.hpp     FlightConfig: 全マジックナンバーをここに集約(ゲイン, クランプ,
│   │                      タイムアウト, チャネル, レート分周比)
│   ├── src/comm.{hpp,cpp} ESP-NOW送受信。受信cb=検証+メールボックス格納のみ。
│   │                      TXはキュー+送信関数。リレーMAC学習。チャネルピン留め
│   ├── src/flight_control.{hpp,cpp}  状態機械+カスケードPID+ミキサ(OptiTrack版を基に
│   │                      整理。死コード・typo継承禁止)
│   ├── src/telemetry.{hpp,cpp}  TLM_STATE 25Hz(400Hzの16分周)+TLM_EVENT生成
│   ├── src/indicators.{hpp,cpp} LED状態表示(製品版led.cpp流用)+非ブロッキングブザー
│   │                      (製品版の vTaskDelay とGPIO5ピン誤りを修正)
│   ├── src/{sensor,imu,tof,alt_kalman,pid}.{hpp,cpp}  OptiTrack版から流用(飛行実績層)
│   └── lib/               bmi270, vl53l3c, MdgwickAHRS を OptiTrack版からコピー
├── firmware_relay/        リレーファーム(PlatformIO, board: esp32dev)
│   ├── platformio.ini     -I../protocol。release/debug 2env
│   └── src/
│       ├── main.cpp       初期化+ループ(小さく)
│       ├── uart_link.{hpp,cpp}   RX: 0x00区切り蓄積→protocolヘッダのデコーダ呼び出し。
│       │                  TX: FreeRTOSキュー+専用書き出しタスク(唯一のSerialライタ)
│       ├── espnow_link.{hpp,cpp} ピア管理(SET_TARGET反映)、受信cb=キュー投入のみ
│       └── router.{hpp,cpp}      型レンジで転送、RLY_*処理、統計、LOG_TEXT発行
├── pc_server/
│   ├── app.py             FastAPI: 静的UI配信 + WebSocket + REST。uvicornで起動
│   ├── core/
│   │   ├── serial_link.py 読み取りスレッド、COBSデコード、型別ディスパッチ、書き込みロック、
│   │   │                  RLY_TARGET_ACK待ち合わせ、統計
│   │   ├── session.py     SessionManager: 接続/モード/Start/Stop/Reset/ログON-OFF/
│   │   │                  機体プロファイル適用(バイアス加算・MAC設定)を一元管理
│   │   ├── posture.py     PostureController: UI setpoint→スルーレート制限→50Hz送信
│   │   ├── position.py    PositionController: mocap→フィルタ→XY PID→setpoint→50Hz送信。
│   │   │                  目標位置はUIから随時更新
│   │   ├── mocap.py       NatNet接続(vendor/のSDK)、座標変換、PositionFilter
│   │   ├── pid.py         既存pid_controller.pyを移植・整理(負ゲイン規約は廃止し、
│   │   │                  軸符号は座標変換側で扱う)
│   │   ├── filter.py      既存position_filter.pyを移植・整理
│   │   └── logger.py      CSVログ(ON時のみ)。logs/YYYYMMDD_HHMMSS_<mode>.csv。
│   │                      50Hz制御行+最新テレメトリ/mocapスナップショット結合。
│   │                      列定義は docs/LOG_STRUCTURE.md に文書化(旧57列の語彙を継承)
│   ├── config/
│   │   ├── server.json    ポート既定値, レート, クランプ, フェイルセーフ閾値
│   │   ├── airframes.json 機体プロファイル配列: {name, mac, wifi_channel,
│   │   │                  roll_bias_deg, pitch_bias_deg, default_alt_m, notes}
│   │   └── control.json   XY PIDゲイン, フィルタ設定, 軌道設定(既存config.json継承)
│   ├── static/            index.html, app.js, style.css(ビルド不要のvanilla JS。CDN禁止)
│   ├── vendor/            NatNetClient.py, MoCapData.py, DataDescriptions.py(既存流用)
│   ├── tests/             pytest(フェイク serial/NatNet でロジック検証)
│   └── requirements.txt   fastapi, uvicorn[standard], pyserial, numpy, pytest(バージョン固定)
└── logs/                  実行時CSV出力先(.gitignore対象)
```

## pc_server API(UI⇔サーバ契約)

単位規約: **UIとWebSocket JSONは deg / m、プロトコルとcore内部は rad / m**。
変換はsession層で行う。

### REST
- `GET /api/ports` → `[{device, description}]`(シリアルポート列挙)
- `GET /api/airframes` → airframes.json の内容
- `PUT /api/airframes` body `{"airframes":[...]}` →
  `{"ok":bool,"error":str|null,"airframes":[...]}`(UI の機体プロファイル編集。
  session.update_airframes が検証 → airframes.json へ原子的保存(temp+os.replace)
  → セッション反映。`mac` は空文字列 = 未設定を許容(未設定プロファイルは
  select_airframe 不可。MAC は `AA:BB:CC:DD:EE:FF` 形式=2桁16進オクテット
  x6 のみ受理)。飛行中は選択中プロファイルの変更/削除を拒否。
  選択中プロファイルのバイアス/default_alt 変更は非飛行時に即再適用、
  MAC/チャネル変更は次の select_airframe/connect で反映(自動再送しない)。
  選択中プロファイルが削除/MAC 未設定化されたら選択を解除し
  relay_target_ok=false(機体未選択のままの start は拒否)。
  検証の制限値は server.json の `clamps.max_roll_pitch_deg` と `airframe_limits`
  (件数上限 `max_profiles`、文字数上限 `name_max_chars`/`notes_max_chars` を含む))
- `GET /api/config` → server.json+control.json の実効値

### WebSocket `/ws`
- サーバ→UI(20Hz): `{"type":"state","data":{
    "drone": {TLM_STATEの全フィールド(角度はdeg換算), "fresh": bool} | null(TLM_STATE未受信),
    "mocap": {"x","y","z","yaw_deg","confidence","fresh"} | null,
    "session": {"mode":"posture"|"position", "phase":"idle"|"connected"|"flying"|...,
      "serial_connected":bool, "airframe":name, "logging":bool, "log_file":str|null,
      "target":{"x","y","z"}|null, "setpoint":{"roll_deg","pitch_deg","alt_m"},
      "latency_ms":float|null, "relay_stats":{...}|null,
      "relay_fresh":bool(RLY_STATS受信時刻ベースの鮮度), "relay_target_ok":bool}}}`
- サーバ→UI(即時): `{"type":"event", ...}`(TLM_EVENT)、`{"type":"log","origin","line"}`
- UI→サーバ:
  - `{"type":"command","action":"connect","port":...}` / `"disconnect"`
  - `{"type":"command","action":"select_airframe","name":...}`(接続時にRLY_SET_TARGET)
  - `{"type":"command","action":"set_mode","mode":"posture"|"position"|"experiment"}`(v2)
  - `{"type":"command","action":"start"}` / `"stop"` / `"reset"`
  - `{"type":"setpoint","roll_deg":..,"pitch_deg":..,"alt_m":..,"yaw_deg":..}`(Posture時。v2: yaw_deg)
  - `{"type":"yaw","yaw_deg":..}`(v2: 共通ヨースライダ)
  - `{"type":"target","x":..,"y":..,"z":..}`(Position時)
  - `{"type":"command","action":"set_logging","enabled":bool}`
  - v2 追加コマンド: `"experiment_activate"` / `"set_yaw_control"` /
    `"circle_start"`(center_x/center_y/radius_m/period_s/clockwise/alt_m/face_tangent)/
    `"circle_stop"` / `"motor_start"`(duty/mask)/ `"motor_set"`(duty)/ `"motor_stop"`
  - v2 REST(Experiment タブ): `/api/sweep` `/api/sequence` `/api/cal3d` `/api/accel6`
    `/api/quickcal` `/api/geomag` `/api/calprofile` `/api/ffprofile`
  - 緊急停止の優先経路(v2): `stop` / `motor_stop` は受信ループの順序キューを迂回して
    即時実行される(先行する低速コマンドの ACK 待ちに巻き込まれない)。

### UI構成(static/)
ヘッダ: ポート選択+接続、機体プロファイル選択(MAC未設定は「⚠ MAC未設定」表示)
+「編集」ボタン(プロファイル編集モーダル: 行追加/行削除/保存=PUT/キャンセル)、
リンク状態(serial/relay/drone)、電圧。
タブ(v2 で 3 タブ): **Posture**(Start/Stop、roll/pitchスライダ±5°(設定で±10°まで)
=飛行中のみ操作可、高度スライダ0.1–1.0m=接続中なら離陸前から操作可(CMD_SETPOINT
flags bit0 による離陸目標高度の事前設定)、ヨー角スライダ±180°+ヨー角制御トグル+
FF プロファイル欄) / **Position**(Start/Stop、目標XYZ入力+プリセット、XY平面
プロット(現在/目標+目標軌道円の重畳)、指令roll/pitch表示、軌道セレクタ
(ホバリング/円軌道)+円軌道パラメータ+開始/停止、ヨー系は Posture と共通+
「進行方向を向く」オプション) / **Experiment**(v2: モーターテストバー(0.6 以上は
高出力許可チェック)、スイープ、加算性シーケンス、3D磁気/Accel6/Attitude0/Yaw0/地磁気/
キャリブプロファイル/FF 抽出・適用の各パネル。機体は CMD_MODE で MOTOR_TEST 状態)。
共通モニタ: 姿勢数値+バー(EKF ヨー併記)、高度(現在vs目標)、モータデューティ、
飛行状態インジケータ、EKF 健全性(ffg/ff_status)警告バッジ、レイテンシ、
イベント/ログコンソール、ログ保存トグル+ファイル名。
COMPLETE状態のときのみRe-arm(RESET)ボタン表示。
**SPACE 緊急停止は全タブで有効**(stop に加え、Experiment 中は CMD_MOTOR_STOP も送出)。

## 安全クランプ(多層)

| 層 | roll/pitch | alt |
|---|---|---|
| UI | ±5°(既定) | 0.1–1.0m |
| pc_server | ±10° + スルーレート30°/s | 0.1–1.2m + 0.3m/s |
| ファーム | ±30°(継承) | 0.05–1.5m |

## コーディング規約

- 識別子は英語、説明コメントは日本語可。既存コードのtypo(Rall_*, RNAGE0FLAG等)を
  新規コードに持ち込まない。
- マジックナンバー禁止: ファームは config.hpp / FlightConfig、PCは config/*.json。
- C++: グローバルvolatile間通信を新規に増やさない。タスク間は既存のportMUX
  メールボックス/FreeRTOSキューのパターンに従う。ISR/受信コールバック内での
  Serial出力・ブロッキング禁止。
- Python: スレッド共有状態はlockで保護。time.monotonic()のみ使用(time.time()禁止)。
  ブロッキングI/Oをasyncループに持ち込まない(スレッド+queueで橋渡し)。
- テスト: protocol往復・パーサ破損系・コントローラのフェイルセーフはハードなしで
  pytest可能に保つ(既存プロジェクトのフェイクserial/NatNetパターンを踏襲)。

## 検証コマンド

```
# プロトコル
cd protocol && python3 -m pytest tests/ -q
# ファーム(コンパイル)
~/.platformio/penv/bin/pio run -d firmware_stampfly
~/.platformio/penv/bin/pio run -d firmware_relay
# サーバ
cd pc_server && python3 -m pytest tests/ -q
# サーバ起動
cd pc_server && python3 -m uvicorn app:app --host 127.0.0.1 --port 8000
```

## 既存資産の参照元(コピー/移植元のパス)

- 機体ファーム流用層: `../StampFly_OptiTrack_PID_Control_System/M5StampFly/src/`
  (sensor, imu, tof, alt_kalman, pid, flight_state.hpp の構造化パターン,
  esp_now_callback_compat.hpp)と同 `lib/`
- LED/ブザー: `../M5StampFly-main/src/led.*`(流用), `buzzer.*`(非ブロッキング化+
  ピン修正のうえ移植)
- PC側: `../StampFly_OptiTrack_PID_Control_System/NatNet_PID_Controller/` の
  pid_controller.py, position_filter.py, NatNetClient.py ほか、config.json のゲイン値
- 既知の流用禁止物: 旧esp32_relay(両方)、HoveringControllerの__getattr__委譲、
  -O0のplatformio.ini、全マーカー重心フォールバック
