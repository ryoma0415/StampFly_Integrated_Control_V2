# StampFly Integrated Control

StampFly(ESP32-S3 クアッドコプター)を PC のブラウザ UI から運用する統合制御システム。
機体ファームウェアは1本のみで、**Posture(姿勢制御)/ Position(位置制御)の切替は
PC 側だけ**で行う(ファーム書き換え・再起動は不要)。

| 文書 | 内容 |
|---|---|
| 本書(README.md) | セットアップ・UI の使い方・トラブルシューティング |
| [docs/OPERATION_GUIDE.md](docs/OPERATION_GUIDE.md) | 段階的な安全飛行手順・緊急時対応・機体プロファイル較正 |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | 通信ワイヤ仕様(正典) |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | モジュール構成・API 契約・コーディング規約(正典) |
| [docs/LOG_STRUCTURE.md](docs/LOG_STRUCTURE.md) | CSV フライトログの列定義 |

## 1. システム概要

```
(Position モードのみ)
OptiTrack カメラ群 ──> Motive(バージョン未確認 → §4.3。NatNet SDKは4.3.0で確定)
                          │ NatNet(UDP, リジッドボディ pose)
                          v
┌────────────────────── PC (macOS) ──────────────────────────┐
│ pc_server(FastAPI + uvicorn)                               │
│  ・ブラウザ UI http://127.0.0.1:8000                        │
│  ・Posture: UI スライダ → CMD_SETPOINT                      │
│  ・Position: NatNet → フィルタ → XY PID → CMD_SETPOINT      │
│  ・CSV ログ(logs/、ON 時のみ 50Hz)                        │
└───────────────┬─────────────────────────────────────────────┘
                │ USB シリアル 115200 8N1
                │ (COBS フレーミング + CRC16-CCITT-FALSE)
                v
        リレー(ESP32-WROOM-32E DevKitC)
        型レンジでルーティング・統計(RLY_STATS 1Hz)
                │
                │ ESP-NOW(WiFi ch1 既定、論理フレームそのまま)
                v
        機体 StampFly(ESP32-S3, 400Hz 割り込み制御ループ)
        姿勢+高度のセットポイント追従・自律フェイルセーフ
```

主なレート(PROTOCOL.md 規範): CMD_SETPOINT 50Hz(上り、ハートビート兼用)/
TLM_STATE 25Hz(下り)/ TLM_EVENT 即時+2Hz / RLY_STATS 1Hz。

機体ごとの違い(MAC、角度バイアス)は `pc_server/config/airframes.json` の
**機体プロファイル**で吸収する。ファームに機体固有定数は置かない。

## 2. 必要機材

| 機材 | 備考 |
|---|---|
| StampFly 機体(M5 StampFly, ESP32-S3) | board: `esp32-s3-devkitc-1`。バッテリー充電済みであること |
| リレー(ESP32-WROOM-32E DevKitC) | board: `esp32dev`。USB ケーブルで PC に常時接続 |
| macOS PC | Python 3.13(`/usr/bin/env python3`)、PlatformIO(`~/.platformio/penv/bin/pio`、espressif32 導入済み) |
| USB ケーブル ×2 | 書き込み用(機体)+運用用(リレー)。データ通信対応のもの |
| 飛行スペース | 2m×2m 以上。プロペラガード推奨(詳細は OPERATION_GUIDE.md) |

Position モードでは追加で:

| 機材 | 備考 |
|---|---|
| OptiTrack カメラ+Motive PC | Motive のバージョンは未確認(§4.3 の手順で確認。NatNet SDK は 4.3.0 で確定) |
| 反射マーカー | 機体に取り付け、Motive 上でリジッドボディとして登録 |
| ネットワーク | Motive PC → pc_server PC へ NatNet(UDP)が届くこと |

## 3. ファームウェア ビルド・書き込み

このフォルダ(リポジトリルート)で実行する。両プロジェクトとも
`release`(-O2)/ `debug`(-Og -g3)の 2 env があり、**既定は release**
(`platformio.ini` の `default_envs`)。

```sh
# ビルドのみ(コンパイル確認)
~/.platformio/penv/bin/pio run -d firmware_stampfly -e release
~/.platformio/penv/bin/pio run -d firmware_relay -e release

# 書き込み(ボードを USB 接続して)
~/.platformio/penv/bin/pio run -d firmware_stampfly -e release -t upload
~/.platformio/penv/bin/pio run -d firmware_relay -e release -t upload

# デバッグビルドが必要なとき
~/.platformio/penv/bin/pio run -d firmware_stampfly -e debug -t upload
```

- `-e release` は既定値のため省略可だが、**飛行は必ず release ビルドで行う**。
- USB ポートが複数あるときは `--upload-port /dev/cu.usbmodem…` 等を付ける
  (候補は `~/.platformio/penv/bin/pio device list` で確認)。
- **リレーのデータ UART(UART0)はバイナリフレーム専用**。`pio device monitor` で
  覗いても人間可読のテキストは出ない。テキストは LOG_TEXT フレームとして
  pc_server の UI コンソールに表示される(PROTOCOL.md 設計原則4。これは仕様)。
- ESP-NOW の WiFi チャネルはファーム既定で **1**
  (`firmware_stampfly/src/config.hpp` の `wifi_channel`)。機体プロファイル
  (airframes.json)の `wifi_channel` と一致させること。

## 4. pc_server セットアップ

### 4.1 Python 仮想環境(初回のみ)

```sh
cd pc_server

python3 -m venv .venv
.venv/bin/pip install -r requirements.txt   # バージョン固定済み

# ハードなしで動くテスト(フェイク serial/NatNet)。全件 pass を確認
.venv/bin/python -m pytest tests/ -q
```

### 4.2 機体プロファイル(airframes.json)

機体プロファイルは `pc_server/config/airframes.json` の `airframes` 配列。
**UI ヘッダの「編集」ボタンから編集・保存できる**(再起動不要。保存時に
サーバが検証のうえ JSON へ原子的に書き込み、即セッションへ反映する)。
ファイルを直接エディタで書いてもよいが、その場合はサーバ再起動が必要。

```json
{
  "name": "drone 1",
  "mac": "",
  "wifi_channel": 1,
  "roll_bias_deg": -0.573,
  "pitch_bias_deg": -0.573,
  "default_alt_m": 0.3,
  "notes": "MAC未確認。..."
}
```

| キー | 意味 |
|---|---|
| `mac` | 機体の ESP-NOW MAC(STA インタフェース)。`AA:BB:CC:DD:EE:FF` 形式。**空文字列 = 未設定**(プルダウンに「⚠ MAC未設定」と表示され、選択できない) |
| `wifi_channel` | ファームのチャネル(既定 1)と一致させる(1–13) |
| `roll_bias_deg` / `pitch_bias_deg` | 機体差の角度バイアス(±10° 以内)。**PC 側で指令に加算**される。較正手順は OPERATION_GUIDE.md §7 |
| `default_alt_m` | プロファイル選択時の初期目標高度(0.05–1.5m) |
| `notes` | メモ(UI のプルダウンにツールチップ表示) |

**登録済み機体**(2026-06-11 の手書き記録で全5機のMAC対応を確定済み):

| プロファイル | MAC(STA) | 備考 |
|---|---|---|
| drone X | `48:CA:43:38:9C:88` | 旧OptiTrackプロジェクトで使用していた機体 |
| drone test | `34:B7:DA:5D:27:68` | 旧Posture_Control_PySerialで使用していた機体 |
| drone 1 | `48:CA:43:3A:51:30` | |
| drone 2 | `48:CA:43:38:A1:CC` | |
| drone 3 | `48:CA:43:38:F0:60` | |

記載はすべて **STAモードのMAC**(本システムが使用するもの)。ESP32のAPモードMACは
先頭オクテットに+2した値になるが(例: drone X のAPは `4A:CA:...`)、本システムでは
使わない。

**新しい機体を追加するときのMACの調べ方**: 機体を USB で PC に接続して起動すると、
ブートログに `ESP-NOW ready: MAC=XX:XX:XX:XX:XX:XX` が出力される
(`pio device monitor` 等で確認)。確認した MAC を UI の編集画面で
プロファイルに記入して保存する。

**編集の反映ルール**(PUT /api/airframes、ARCHITECTURE.md が正):
バイアス・初期高度の変更は(飛行中でなければ)保存と同時に反映。
MAC・チャネルの変更は**機体を選び直すか再接続したとき**に反映(保存だけでは
リレーへ再送されず、接続中はコンソールに選び直しを促す警告が出る)。
飛行中は選択中プロファイルの変更・削除が拒否される。

### 4.3 Motive / NatNet 設定(Position モードのみ)

**NatNet SDK のバージョンは 4.3.0 で確定**(リポジトリ同梱の `../NatNetSDK/` の
DLL バージョンリソースで確認。本プロジェクトの `pc_server/vendor/NatNetClient.py` は
この SDK 付属 PythonClient のログ出力を調整したもの)。ただし NatNet SDK の
クライアントは旧ビットストリーム(NatNet 2.x/3.x = Motive 1.x/2.x)とも後方互換のため、
**SDK のバージョンから Motive 本体のバージョンは確定できない**。
記憶ベースでは Motive 約 2.3.1 だが未確認。実際に使う Motive で次を確認してから
運用すること:

1. **バージョン確認**: Motive のメニュー **Help → About Motive…** で表示される
   バージョンを確認し、確認できたら本欄に追記する。
   メジャーバージョンが異なる場合(3.x 等)はメニュー配置・NatNet 互換性が
   変わるため、必ず次の受信確認まで実施する。
2. **ストリーミング有効化**: ストリーミング設定パネル
   (Motive 2.x: **View → Data Streaming Pane**、3.x: **Edit → Settings → Streaming**)で
   - **Broadcast Frame Data** を有効化
   - **Local Interface** に pc_server の PC へ届くネットワーク IF を選択
   - **Rigid Bodies** の配信を有効化
3. **リジッドボディ登録**: 機体のマーカーをリジッドボディとして登録し、
   その **ID** を `pc_server/config/control.json` の `natnet.rigid_body_id` に設定。
4. **接続設定**: `control.json` の `natnet` 節
   (`server_address` = Motive PC の IP、`client_address` = 本機の IP、
   `use_multicast`)を環境に合わせる。
5. **座標系**: 座標変換は `control.json` の `coordinate_transform` が正
   (既定: 制御 x ← Motive z、制御 y ← −Motive x、制御 z ← Motive y。
   すなわち **Motive は Y-up 前提**)。Motive 側の Up Axis を変えた場合はここを直す。
6. **受信確認**: pc_server を起動し UI の Position タブで MoCap インジケータが
   「受信中」(緑)になり、座標表示と XY プロットが機体の移動に追従することを確認。

## 5. pc_server 起動と UI の使い方

```sh
cd pc_server
.venv/bin/python -m uvicorn app:app --host 127.0.0.1 --port 8000
```

ブラウザで <http://127.0.0.1:8000> を開く(UI はビルド不要の vanilla JS、CDN 非依存)。
起動直後は「サーバー未接続」オーバーレイが出るが、WebSocket 接続が確立すると消える
(切断時は 1 秒間隔で自動再接続)。

### 5.1 ヘッダ(接続・機体・リンク状態)

1. **ポート**: リレーのシリアルポートをプルダウンで選び「接続」。
   ⟳ ボタンで一覧を再取得できる。接続中はボタンが「切断」に変わる。
2. **機体**: 機体プロファイルをプルダウンで選択。pc_server がリレーへ
   RLY_SET_TARGET(MAC+チャネル)を送り、ACK の値一致を確認する
   (1.0s 待ち×最大4回)。**設定完了までドローン宛コマンドはリレーが転送しない**。
   選択時にそのプロファイルの `default_alt_m` が高度スライダ初期値に反映される。
   飛行中のプロファイル変更は拒否される。**「⚠ MAC未設定」と付くプロファイルは
   選択できない**(§4.2 の手順で MAC を調べて設定する)。
   **「編集」ボタン**でプロファイル編集モーダルが開き、行追加/行削除/保存が
   できる(保存はサーバ検証つき。再起動不要。§4.2)。
3. **リンク状態**(3連インジケータ):
   - **シリアル**: ポート接続中で緑。
   - **リレー**: RLY_STATS(1Hz)の受信時刻ベースで緑。**黄=リレーは生きているが
     ESP-NOW ターゲット未設定**(機体宛コマンドは転送されない)。
   - **機体**: テレメトリ(TLM_STATE)が新鮮(0.3s 以内)なら緑。
4. **電圧**: 機体テレメトリの電圧。3.5V 未満で黄、3.4V 未満で赤。

### 5.2 Posture タブ(姿勢制御)

- **START**(確認ダイアログあり)で離陸、**STOP** で即時着陸。
- **Roll / Pitch スライダ**(UI 既定 ±5°、0.1° 刻み): **飛行中のみ操作可**。
  「中央に戻す」で両方 0° に戻る。
- **高度スライダ**(0.1–1.0m): **接続中なら離陸前から操作可**
  (離陸目標高度の事前設定。CMD_SETPOINT flags bit0 による契約どおりの動作)。
- スライダ操作は 10Hz スロットルでサーバへ送られ、サーバ側で
  ±10° クランプ+スルーレート 30°/s(高度 0.3m/s)の整形を経て 50Hz で送信される。

### 5.3 Position タブ(位置制御)

- **目標 X/Y/Z** 入力(m、制御座標系)+プリセット:
  「**この場で**」= 現在の MoCap 位置 XY を目標に(Z は維持)、「**原点**」= (0,0)。
- **MoCap** インジケータ: NatNet 受信状態と座標・信頼度の表示。
- **XY 平面プロット**: 現在位置(点)・目標(◎)・直近 30 秒の軌跡。表示半幅 2m。
- **指令角**: サーバが計算した適用中の roll/pitch 指令(表示のみ、操作不可)。
- START は **MoCap データが新鮮(0.3s 以内)でないと拒否**される。

### 5.4 共通モニタ・緊急停止・ログ

- **状態バッジ**: 機体の FlightState(INIT/CALIBRATION/WAIT/TAKEOFF/HOVER/LANDING/
  COMPLETE)+ phase / mode / 機体名。
- **Re-arm (RESET)**: OverG 検出などで機体が **COMPLETE** になったときのみ表示。
  機体が静止し**推定高度 0.15m 未満**のときのみファームが受理し WAIT に復帰する。
- **姿勢バー**(±30° フルスケール)、**高度バー**(現在 vs 目標マーカー)、
  **モータデューティ**、**レイテンシ**(CMD_SETPOINT→seq_echo の往復)、
  **リレー統計**(`relay ↑/↓/err`。カウンタの読み方は PROTOCOL.md
  「RLY_STATS のカウンタ集計規則」)。
- **緊急停止**: **SPACE キー = どこからでも即 STOP**(フォーカス位置に関わらず効く)。
  STOP は全飛行状態で受理され、600ms 以内に着陸イベントがなければ自動再送
  (最大3回)される。エスカレーション手順は OPERATION_GUIDE.md §2。
- **ログ保存**: トグル ON の間だけ `logs/YYYYMMDD_HHMMSS_<mode>.csv`
  (リポジトリ直下 `logs/`)に 50Hz で記録。ファイル名はトグル横に表示。
  モード切替時はファイルを開き直す。列定義は
  [docs/LOG_STRUCTURE.md](docs/LOG_STRUCTURE.md)。
- **イベント / ログ コンソール**: 状態遷移(TLM_EVENT)、リレー/機体からの
  LOG_TEXT、サーバ警告を時刻付きで表示(直近 200 行)。

## 6. 検証コマンド(変更を入れたら必ず)

```sh
# プロトコル(Python 往復+C++ host_test を pytest 経由で実行。g++ 必要)
cd protocol && python3 -m pytest tests/ -q

# ファーム(コンパイル)
~/.platformio/penv/bin/pio run -d firmware_stampfly -e release
~/.platformio/penv/bin/pio run -d firmware_relay -e release

# サーバ
cd pc_server && .venv/bin/python -m pytest tests/ -q
```

## 7. トラブルシューティング

| 症状 | 確認すること |
|---|---|
| ポート一覧にリレーが出ない | USB ケーブル(データ通信対応か)、⟳ で再取得、`ls /dev/cu.*`。macOS はデバイス名が接続ごとに変わることがある |
| 「シリアルポートを選択してください」 | プルダウンが「(ポートなし)」のまま接続を押した。上記を確認 |
| リレーが黄色のまま | ESP-NOW ターゲット未設定。機体プロファイルを選び直し、コンソールの「リレーターゲット設定完了」を確認。「設定失敗(status=…)」なら MAC 形式と `wifi_channel`(1–13)を確認 |
| 「MAC が未設定のため選択できません」 | プロファイルに MAC が入っていない。機体を USB 接続し起動ログの `ESP-NOW ready: MAC=...` で MAC を確認し、「編集」から記入・保存する(§4.2) |
| 機体リンクが緑にならない | 機体の電源、プロファイルの MAC/チャネル(ファーム既定 ch1)、機体とリレーの距離。コンソールに機体起動時の LOG_TEXT が出るかも確認 |
| START が効かない | 機体が WAIT 状態か(状態バッジ)。COMPLETE なら Re-arm が先。低電圧だと拒否される(コンソールに「離陸拒否」)。Position では MoCap が新鮮であること |
| 「CMD_STOP への応答がありません」警告 | 機体電源 OFF や圏外で正常に出る(通信確認フェーズでは仕様)。飛行中に出た場合は機体側フェイルセーフ(リンク喪失 500ms で自律着陸)に任せ、距離・電波環境を見直す |
| RLY_STATS の `crc_errors` 増加 | 配線/ノイズ。ver/len 不整合・デリミタ欠落による破棄もここに合算される(PROTOCOL.md 参照) |
| RLY_STATS の `overflow_drops` 増加 | UART 帯域逼迫かキュー詰まり(RX 256B 超過 / TX キュー満杯等の合算) |
| `pio device monitor` が文字化け | 仕様。データ UART はバイナリフレーム専用(§3) |
| Position でプロットが止まる/「途絶」 | Motive のストリーミング設定(§4.3)、`control.json` の `natnet`(アドレス/multicast/`rigid_body_id`)、ファイアウォール |
| UI 全体に「サーバー未接続」 | uvicorn プロセスの生死。ポート使用中なら `lsof -nP -iTCP:8000 -sTCP:LISTEN` で確認 |
| ログファイルができない | トグル ON か、シリアル接続中か(ログはセットポイント送信ごとに記録されるため、未接続では行が増えない) |
