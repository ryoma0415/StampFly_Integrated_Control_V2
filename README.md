# StampFly Integrated Control V2

StampFly(ESP32-S3 クアッドコプター)を PC のブラウザ UI から運用する統合制御システム。
機体ファームウェアは1本のみで、**Posture(姿勢制御)/ Position(位置制御)/
Experiment(ベンチ実験)/ Multi(複数機)の切替は PC 側だけ**で行う
(ファーム書き換え・再起動は不要)。

v2 では **ヨー推定・ヨー角制御**(BMM150 磁気+INA3221 電流によるモーター電流FF補正
+4状態EKF)、**実験モード**(モーターテスト・電流×磁場スイープ・各種キャリブレーション)、
**円軌道モード**、**複数機モード**(2〜4機の同時位置制御。リレーの多重化拡張のみで
機体ファーム無改修)、**飛行ログビューア**が追加された(プロトコルは v2 = 0x02)。

| 文書 | 内容 |
|---|---|
| 本書(README.md) | セットアップ・UI の使い方・トラブルシューティング |
| [docs/OPERATION_GUIDE.md](docs/OPERATION_GUIDE.md) | 段階的な安全飛行手順・緊急時対応・機体プロファイル較正・v2 運用手順(実験モード/ヨー較正/ヨー制御飛行/円軌道/飛行後解析/複数機モード) |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | 通信ワイヤ仕様(正典) |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | モジュール構成・API 契約・コーディング規約(正典) |
| [docs/LOG_STRUCTURE.md](docs/LOG_STRUCTURE.md) | CSV フライトログの列定義(v3: 100列) |
| [docs/FF_PIPELINE.md](docs/FF_PIPELINE.md) | 電流FF較正パイプライン(スイープ→抽出→適用→評価)の契約 |
| [flight_log_viewer/README.md](flight_log_viewer/README.md) | 飛行ログビューアの使い方 |

## 1. システム概要

```
(Position モードのみ)
OptiTrack カメラ群 ──> Motive(バージョン未確認 → §4.3。NatNet SDKは4.3.0で確定)
                          │ NatNet(UDP, リジッドボディ pose)
                          v
┌────────────────────── PC (macOS) ──────────────────────────────┐
│ pc_server(FastAPI + uvicorn)                                   │
│  ・ブラウザ UI http://127.0.0.1:8000(3タブ)                   │
│  ・Posture:   UI スライダ → CMD_SETPOINT(roll/pitch/alt/yaw)  │
│  ・Position:  NatNet → フィルタ → XY PID → CMD_SETPOINT         │
│               (v2: 円軌道 = 目標XYを時間更新して同じPIDへ)     │
│  ・Multi(v2): 2〜4機の同時位置制御(機体ごとに 50Hz 送信)      │
│  ・Experiment(v2): モーターテスト/スイープ/キャリブ/FF 適用     │
│  ・CSV ログ(logs/flight_logs/、START〜着陸 50Hz・100列)       │
│  ・生成データ: pc_server/data/(sweep_results, ff_profiles,     │
│    calibration_profiles, ff_state.json)                        │
└───────────────┬─────────────────────────────────────────────────┘
                │ USB シリアル 460800 8N1(v2 既定。フォールバック 115200)
                │ (COBS フレーミング + CRC16-CCITT-FALSE、ver=0x02)
                v
        リレー(ESP32-WROOM-32E DevKitC)v2: 複数機対応(要再書き込み §3)
        型レンジでルーティング・統計(RLY_STATS 1Hz)
        v2: 最大4機のピア管理+RLY_MUX 多重化(複数機モード)
                │
                │ ESP-NOW(WiFi ch1 既定、論理フレームそのまま — 単機時と同一)
                v
        機体 StampFly(ESP32-S3, 400Hz 割り込み制御ループ)
        姿勢+高度+ヨー角のセットポイント追従・自律フェイルセーフ
        v2: ヨー推定(BMM150 磁気 + INA3221 電流 → モーター電流FF補正 ΔB̂
            → 補正CF / 4状態EKF)+ヨー角制御(psi_pid)
        v2: MOTOR_TEST 状態(ベンチ実験。CMD_MODE で WAIT と相互遷移)
```

主なレート(PROTOCOL.md 規範): CMD_SETPOINT 50Hz(上り、ハートビート兼用。
experiment モード中は停止)/ TLM_STATE 25Hz(下り、135B)/ TLM_EVENT 即時+2Hz /
TLM_EXP 25Hz(MOTOR_TEST 中のみ)/ CMD_MOTOR_RUN 0.4s キープアライブ /
RLY_STATS 1Hz。複数機モードでは CMD_SETPOINT 50Hz・TLM_STATE 25Hz が
**機体ごと**に流れる(このため v2 でシリアルは既定 460800 になった)。

機体ごとの違い(MAC、角度バイアス)は `pc_server/config/airframes.json` の
**機体プロファイル**で吸収する。ファームに機体固有定数は置かない。
機体固有の較正(3D磁気・加速度6面・FF 係数等)は機体の NVS に永続化される
(ARCHITECTURE.md「NVS 永続化一覧」)。

### 1.1 リポジトリ構成(概要)

```
protocol/            通信プロトコルの単一真実(C++/Python 実装+テストベクタ)
firmware_stampfly/   機体ファーム(src/yaw_estimation/ = v2 ヨー推定モジュール)
firmware_relay/      リレーファーム(v2: 複数機多重化対応+UART 460800 化)
pc_server/           FastAPI サーバ+ブラウザ UI+実験/キャリブ/FF 機能
  ├─ config/         設定(server/control/airframes/geomagnetic_profiles ほか)
  └─ data/           生成データ(sweep_results / ff_profiles /
                     calibration_profiles / yaw_eval_results / ff_state.json)
data_analysis/       FF 係数抽出(make_ff_profile.py)+スイープ/実験ログのグラフ化
                     (plot_sweep.py / plot_explog.py)+受入テスト(独立 venv)
flight_log_viewer/   飛行ログ(logs/*.csv)の可視化ツール(独立 venv)
logs/                フライトログ CSV 出力先(.gitignore 対象)
docs/                本文書群
```

詳細なファイル一覧は ARCHITECTURE.md「フォルダ構成」。

## 2. 必要機材

| 機材 | 備考 |
|---|---|
| StampFly 機体(M5 StampFly, ESP32-S3) | board: `esp32-s3-devkitc-1`。バッテリー充電済みであること |
| リレー(ESP32-WROOM-32E DevKitC) | board: `esp32dev`。USB ケーブルで PC に常時接続 |
| macOS PC | Python 3.13(`/usr/bin/env python3`)、PlatformIO(`~/.platformio/penv/bin/pio`、espressif32 導入済み) |
| USB ケーブル ×2 | 書き込み用(機体)+運用用(リレー)。データ通信対応のもの |
| 飛行スペース | 2m×2m 以上。プロペラガード推奨(詳細は OPERATION_GUIDE.md) |
| 固定用テープ等 | Experiment(モーター実験)時に機体を作業台へ固定する(OPERATION_GUIDE.md §11) |

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
- **v2 のプロトコルは ver=0x02**。v1 ファームと v2 サーバ(またはその逆)を
  混在させるとフレームは破棄され `ver_errors`(RLY_STATS では crc_errors に合算)
  として現れる。機体・リレー・PC を揃えて更新すること。
- **リレーは v2 で複数機対応+UART 460800 化されたため再書き込み必須**
  (機体ファームは複数機モードでも無改修)。既定 env(`release`)の UART は
  **460800** で、PC 側 `pc_server/config/server.json` の `serial.baudrate`
  (既定 460800)と一致させる。460800 が動かない USB シリアルブリッジ向けに
  `release-115200` env がある:
  `~/.platformio/penv/bin/pio run -d firmware_relay -e release-115200 -t upload`。
  この場合は `serial.baudrate` を 115200 に戻す(複数機は帯域不足のため
  2機まで+`multi.tlm_state_div` 間引き推奨。OPERATION_GUIDE.md §16)。

## 4. pc_server ほか Python 環境のセットアップ

### 4.1 Python 仮想環境(初回のみ、3箇所)

v2 は用途別に 3 つの venv を使う(依存を混ぜない)。

```sh
# ① pc_server(サーバ本体。必須)
cd pc_server
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt   # バージョン固定済み
.venv/bin/python -m pytest tests/ -q        # ハードなしで動くテスト。全件 pass を確認
cd ..

# ② data_analysis(FF 係数抽出。ヨー較正をするなら必須)
cd data_analysis
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt   # numpy, matplotlib
.venv/bin/python tests/test_ff_extraction.py  # 受入テスト(直接実行)
cd ..

# ③ flight_log_viewer(飛行後解析。推奨)
cd flight_log_viewer
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt   # numpy, pandas, matplotlib
cd ..
```

- UI の「FF 抽出」は `data_analysis/.venv/bin/python` を自動で使う
  (無ければ `python3` にフォールバック)。
- flight_log_viewer のアニメーション出力には ffmpeg 本体、スマホ動画合成には
  opencv-python が別途必要(flight_log_viewer/README.md)。

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
  "rigid_body_id": null,
  "notes": "MAC未確認。..."
}
```

| キー | 意味 |
|---|---|
| `mac` | 機体の ESP-NOW MAC(STA インタフェース)。`AA:BB:CC:DD:EE:FF` 形式。**空文字列 = 未設定**(プルダウンに「⚠ MAC未設定」と表示され、選択できない) |
| `wifi_channel` | ファームのチャネル(既定 1)と一致させる(1–13) |
| `roll_bias_deg` / `pitch_bias_deg` | 機体差の角度バイアス(±10° 以内)。**PC 側で指令に加算**される。較正手順は OPERATION_GUIDE.md §7 |
| `default_alt_m` | プロファイル選択時の初期目標高度(0.05–1.5m) |
| `rigid_body_id` | MoCap リジッドボディの streaming ID(1 以上の整数。`null` = 未設定)。**複数機モード(Multi)で必須**。UI 編集モーダルの「RB ID」列で設定し、Multi タブの「リジッドボディ確認」で ID を照合できる(OPERATION_GUIDE.md §16)。単機 Position モードは従来どおり `control.json` の `natnet.rigid_body_id` を使う |
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

**補足(v2)**: 3D磁気・加速度6面・FF 係数などの較正は機体プロファイルではなく
**機体の NVS** に保存される。PC 側のスナップショットは
`pc_server/data/calibration_profiles/`(キャリブプロファイル)と
`pc_server/data/ff_profiles/`(FF プロファイル)で管理する(§5.5)。

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

タブは **Posture / Position / Experiment / Multi** の 4 つ。タブ切替 = モード切替で、
飛行中は切り替えられない。**SPACE キーの緊急停止は全タブで有効**
(Experiment 中は CMD_MOTOR_STOP も送出。Multi 中は全機一斉 STOP)。

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
4. **電圧**: 機体テレメトリの電圧。3.5V 未満で黄、3.4V 未満で赤
   (v2: 電圧源は INA3221 の 20Hz 読みに一本化)。

### 5.2 Posture タブ(姿勢制御)

- **START**(確認ダイアログあり)で離陸、**STOP** で即時着陸。
- **Roll / Pitch スライダ**(UI 既定 ±5°、0.1° 刻み): **飛行中のみ操作可**。
  「中央に戻す」で両方 0° に戻る。
- **高度スライダ**(0.1–1.0m): **接続中なら離陸前から操作可**
  (離陸目標高度の事前設定。CMD_SETPOINT flags bit0 による契約どおりの動作)。
- スライダ操作は 10Hz スロットルでサーバへ送られ、サーバ側で
  ±10° クランプ+スルーレート 30°/s(高度 0.3m/s)の整形を経て 50Hz で送信される。

### 5.3 ヨー角制御(Posture / Position 共通、v2)

Posture・Position 両タブの下部に共通のヨー操作欄がある:

- **ヨー角スライダ**(±180°、0.1° 刻み)+「ヨー 0°」ボタン。
- **「ヨー角制御」トグル**: ON にすると CMD_SETPOINT flags bit1 を立てて
  yaw_ref を送信する(機体が角度制御で機首方位を保持)。OFF なら機体は
  V1 と同一のレートダンピングのみ。指令はサーバ側で最短経路(±180° 跨ぎで
  遠回りしない)+スルーレート 45°/s に整形される。
- **FF プロファイル欄**: ヨー角制御 ON のとき表示。`pc_server/data/ff_profiles/`
  のプルダウン+「適用」ボタン+適用中バナー。est_mode は既定 EKF。
  **ff_mode=0(FF 補正なし)のまま ON にすると「FF未適用」警告バッジ**が出る
  (飛行は可能だがモーター電流の磁気干渉を受ける。OPERATION_GUIDE.md §13)。
- **ヨー推定モニタ**(右側モニタ列): Madgwick / EKF / ジャイロ積算 /
  (Position では MoCap)のヨー数値、NIS・ffg・電流・FF 状態、
  **EKF 健全性バッジ**(EKF OK / EKF注意 / 相補CF)。姿勢バーの Yaw にも
  EKF ヨーが併記される。
- 段階的な運用手順(OFF での 4 系統比較 → 低ゲインで ON)は
  OPERATION_GUIDE.md §13 を必ず読むこと。

### 5.4 Position タブ(位置制御・円軌道)

- **目標 X/Y/Z** 入力(m、制御座標系)+プリセット:
  「**この場で**」= 現在の MoCap 位置 XY を目標に(Z は維持)、「**原点**」= (0,0)。
- **MoCap** インジケータ: NatNet 受信状態と座標・信頼度の表示。
- **XY 平面プロット**: 現在位置(点)・目標(◎)・直近 30 秒の軌跡。表示半幅 2m。
  円軌道中は目標軌道の円が重畳表示される。
- **指令角**: サーバが計算した適用中の roll/pitch 指令(表示のみ、操作不可)。
- START は **MoCap データが新鮮(0.3s 以内)でないと拒否**される。
- **軌道セレクタ(v2)**: 「ホバリング(固定目標)」/「円軌道」。円軌道の
  パラメータは 中心 X/Y(±2m)・半径(0.05–1.5m)・周期(3–120s)・
  回転方向(CW/CCW)・高度。「円軌道開始」で現在位置から円周上の最近傍点へ
  滑らかに合流し、「停止(ホバ復帰)」で現在目標のホバリングに戻る。
- **「進行方向を向く」チェック(v2)**: ON かつヨー角制御 ON のとき yaw_ref を
  接線方向に追従させる。**周期 8s 以上が必要**(接線ヨー角速度がスルーレート
  上限 45°/s を超えないため。8s 未満は開始拒否)。
- MoCap 途絶時の既存フェイルセーフ(300ms 水平化・2s 自動 STOP)は円軌道中も
  同一(軌道位相は途絶中凍結される)。

### 5.5 Experiment タブ(ベンチ実験、v2)

機体を **MOTOR_TEST 状態**にして行うベンチ実験専用タブ。タブに入ると
50Hz セットポイント送信が止まり、CMD_MODE で機体が MOTOR_TEST に遷移する
(バッジ「有効」)。**実験中は離陸できない**。プロペラが回るため、
**必ず OPERATION_GUIDE.md §11 の安全手順(機体固定)に従うこと**。

パネル構成(上から):

- **固定確認チェック**: 「機体をテープで確実に固定したことを確認した」。
  ON にするまでモーター/スイープ/シーケンスの開始ボタンは無効。
- **モーターテスト**: duty ボタン(0.1〜1.0)+Start/反映/Stop。
  **0.6 以上は「高出力許可」チェックが必要**。実行中は TLM_EXP のライブ値
  (電流・電圧・磁気等)を表示。キープアライブ 0.4s、機体側 1.5s 途絶自動停止。
- **計測(EKF/FF性能ログ)**(v2): モーターテスト中の EKF/FF 性能を
  TLM_EXP 受信ごと(約 25Hz)に CSV へ記録する。開始/停止ボタンと状態表示
  (ファイル名+サンプル数)。保存先は `pc_server/data/exp_logs/` に
  `explog_<日時>.csv` + `explog_<日時>_meta.json`(FF 適用状態・3D磁気較正・
  地磁気プロファイルのスナップショット付き)。列は電流/電圧/磁気(生+較正)/
  Madgwick 姿勢/ジャイロ/加速度に加え、最新 TLM_STATE のヨー推定
  (yaw_est/yaw_gyro_int/yaw_ref・ΔB̂・NIS・ffg/ff_status)を鮮度付きで結合。
  **計測中の制限**: スイープ/シーケンスは開始不可、モーター回転は全モーター
  (FL+FR+RL+RR)のみ受理(停止は常時可)。実験無効化・モード離脱・切断で
  自動停止(meta に aborted=true)。SPACE 緊急停止ではモーターだけ止まり
  計測は継続する。
- **電流×磁場スイープ**: 回すモーター(FL/FR/RL/RR)選択、パターン
  (往復 0.1→1.0→0.1 推奨 / 昇順のみ)、notes(場所・方位・備考)入力、
  進捗バー+中断ボタン。結果は `pc_server/data/sweep_results/` に
  `sweep_<日時>_{samples.csv,meta.json}` で保存。
- **加算性シーケンス**: FL→FR→RL→RR の単機スイープ自動 4 本。電池ガード
  (vbat ≥ 3.5V、不足時は交換待ち一時停止 → 再開)+冷却 10s。
  `sequence_<日時>_meta.json` を出力。
- **3D磁気キャリブレーション**: 収集開始 → 機体を全方位に回す → Fit → 適用
  (CMD_MAG3D_SET)。PC 側スナップショットは `pc_server/config/mag3d_calibration.json`。
  **適用すると機体側で FF は自動無効化される**(要 FF 再適用)。
- **加速度6面(Accel6)**: 6面を静置キャプチャ(約1秒平均)→ 適用
  (CMD_ACCEL6_SET。姿勢参照リセットを伴う)。
- **クイック較正(Attitude 0 / Attitude Clear / Yaw 0 / Yaw Clear)**:
  現在姿勢のマウントオフセット設定・クリア(CMD_ATTMOUNT_SET)/
  現在方位のヨーゼロ設定・クリア(CMD_YAWZERO_SET。Yaw 0 は
  FF モード off のときのみ受理)。
- **地磁気**: 47 都道府県ドロップダウン → 選択で保存+機体へ CMD_GEOMAG_SET。
- **キャリブレーション・プロファイル**: 機体の較正一式(CMD_CAL_GET →
  TLM_CAL_DATA)を `pc_server/data/calibration_profiles/<名前>.json` に保存/
  機体へ適用(適用後に読み戻し照合)。
- **FFプロファイル**: スイープ8本フォルダから抽出(`data_analysis/
  make_ff_profile.py` をサブプロセス実行)→ 適用(CMD_FF_BEGIN→LUT→MOT→AUX→
  COMMIT、CRC照合)→ モード変更(off/方式A/方式B × 相補CF/EKF)/
  アンカー再取得/削除。詳細は docs/FF_PIPELINE.md。

一連のヨー較正手順(どの順で何をやるか)は OPERATION_GUIDE.md §12。

### 5.6 Multi タブ(複数機同時制御、v2)

2〜4機を同時に MoCap 位置制御するタブ(静的目標のみ・ヨー角制御 OFF)。
**リレーが複数機対応ファームであること**(§3)と、各機体プロファイルの
`rigid_body_id` 設定(§4.2)が前提。ログトグル ON なら一斉スタートで
機体ごとの CSV(§5.7)が記録される。

- **機体選択(2〜4機)**: チェックボックスで選び「**選択適用**」。リレーへ
  RLY_SET_PEERS が送られる(全機同一 `wifi_channel`・MAC/RB ID 設定済みが必要)。
- **リジッドボディ確認**: 「確認開始」で観測中の全リジッドボディ
  (`/api/mocap/bodies`、500ms ポーリング)の ID と座標をライブ表示。
  機体を1機ずつ動かして ID を特定し、「編集」の RB ID 列へ設定する。
- **機体別目標**: 機体ごとの X/Y/Z 入力(XY ±2m。目標同士の XY 間隔 0.5m 以上)。
- **共有 XY プロット**: 全機の現在位置・目標を機体別の色で表示。
- **一斉スタート**(確認ダイアログあり): 全機の目標設定・MoCap 鮮度・目標間隔を
  検証してから全機へ CMD_START。**STOP / SPACE は全機一斉着陸**。
- 運用手順と安全規則は OPERATION_GUIDE.md §16 を必ず読むこと。

### 5.7 共通モニタ・緊急停止・ログ

- **状態バッジ**: 機体の FlightState(INIT/CALIBRATION/WAIT/TAKEOFF/HOVER/LANDING/
  COMPLETE/**MOTOR_TEST**(v2))+ phase / mode / 機体名。
- **Re-arm (RESET)**: OverG 検出などで機体が **COMPLETE** になったときのみ表示。
  機体が静止し**推定高度 0.15m 未満**のときのみファームが受理し WAIT に復帰する。
- **姿勢バー**(±30° フルスケール、Yaw は EKF 併記)、**ヨー推定モニタ**(§5.3)、
  **高度バー**(現在 vs 目標マーカー)、**モータデューティ**、
  **レイテンシ**(CMD_SETPOINT→seq_echo の往復)、
  **リレー統計**(`relay ↑/↓/err`。カウンタの読み方は PROTOCOL.md
  「RLY_STATS のカウンタ集計規則」)。
- **緊急停止**: **SPACE キー = どこからでも即 STOP**(フォーカス位置・タブに
  関わらず効く。Experiment 中は CMD_MOTOR_STOP も送出し、スイープ/シーケンスも
  中断。**Multi 中は全機へ一斉 CMD_STOP**)。STOP は全飛行状態で受理され、
  600ms 以内に着陸イベントがなければ自動再送(最大3回)される。
  エスカレーション手順は OPERATION_GUIDE.md §2。
- **ログ保存**: 記録単位は「**1飛行**」。トグル ON は予約で、START
  (CMD_START 受理)で `logs/flight_logs/YYYYMMDD_HHMMSS_<mode>.csv` を
  開いて 50Hz で記録し、飛行終了(着陸・START 猶予切れ・切断)でファイルを
  閉じて**トグルは自動 OFF** になる。飛行中に ON にすれば途中から記録、
  OFF は即閉じ。ファイル名はトグル横に表示。**Multi モードでは機体ごとに
  `<同一日時>_multi_<機体名>.csv` を出力**し、全機着陸で全ファイル close +
  自動 OFF(表示は「<先頭ファイル名> ×N機」)。列定義は
  [docs/LOG_STRUCTURE.md](docs/LOG_STRUCTURE.md)(v3: 100列。ヨー指令・
  ヨー推定・EKF 診断・軌道状態・機上XY制御診断を含む)。experiment モード
  では飛行ログは記録されない(トグルも無効。計測は §5.5 の
  「計測(EKF/FF性能ログ)」を使う)。
- **イベント / ログ コンソール**: 状態遷移(TLM_EVENT)、リレー/機体からの
  LOG_TEXT、サーバ警告を時刻付きで表示(直近 200 行)。

## 6. 検証コマンド(変更を入れたら必ず)

```sh
# プロトコル(Python 往復+C++ host_test を pytest 経由で実行。g++ 必要)
cd protocol && python3 -m pytest tests/ -q

# ファーム(コンパイル)
~/.platformio/penv/bin/pio run -d firmware_stampfly -e release
~/.platformio/penv/bin/pio run -d firmware_relay -e release

# サーバ(フェイク serial/NatNet。実験・キャリブ・FF・円軌道のテストを含む)
cd pc_server && .venv/bin/python -m pytest tests/ -q

# FF 抽出の受入テスト(6/12 照合+付録A再現+sequence 展開。pytest 不要)
cd data_analysis && .venv/bin/python tests/test_ff_extraction.py

# data_analysis の3スクリプトの起動確認(引数なしは対話式のため --help で)
cd data_analysis && for s in make_ff_profile plot_sweep plot_explog; do \
  .venv/bin/python $s.py --help > /dev/null || echo "NG: $s"; done

# flight_log_viewer(ダミーログ生成 → 全出力の動作確認)
cd flight_log_viewer && .venv/bin/python tests/make_dummy_log.py
cd flight_log_viewer && .venv/bin/python visualize.py ../logs/dummy_position.csv --all
```

## 7. トラブルシューティング

| 症状 | 確認すること |
|---|---|
| ポート一覧にリレーが出ない | USB ケーブル(データ通信対応か)、⟳ で再取得、`ls /dev/cu.*`。macOS はデバイス名が接続ごとに変わることがある |
| 「シリアルポートを選択してください」 | プルダウンが「(ポートなし)」のまま接続を押した。上記を確認 |
| リレーが黄色のまま | ESP-NOW ターゲット未設定。機体プロファイルを選び直し、コンソールの「リレーターゲット設定完了」を確認。「設定失敗(status=…)」なら MAC 形式と `wifi_channel`(1–13)を確認 |
| 「MAC が未設定のため選択できません」 | プロファイルに MAC が入っていない。機体を USB 接続し起動ログの `ESP-NOW ready: MAC=...` で MAC を確認し、「編集」から記入・保存する(§4.2) |
| 機体リンクが緑にならない | 機体の電源、プロファイルの MAC/チャネル(ファーム既定 ch1)、機体とリレーの距離。コンソールに機体起動時の LOG_TEXT が出るかも確認 |
| START が効かない | 機体が WAIT 状態か(状態バッジ)。COMPLETE なら Re-arm が先。低電圧だと拒否される(コンソールに「離陸拒否」)。Position では MoCap が新鮮であること。**実験モード中は離陸不可**(「実験モード中は離陸できません」) |
| 「CMD_STOP への応答がありません」警告 | 機体電源 OFF や圏外で正常に出る(通信確認フェーズでは仕様)。飛行中に出た場合は機体側フェイルセーフ(リンク喪失 500ms で自律着陸)に任せ、距離・電波環境を見直す |
| RLY_STATS の `crc_errors` 増加 | 配線/ノイズ。ver/len 不整合・デリミタ欠落による破棄もここに合算される(PROTOCOL.md 参照)。**新旧ファーム混在(v1 の ver=0x01 フレーム)もここに現れる**(§3) |
| RLY_STATS の `overflow_drops` 増加 | UART 帯域逼迫かキュー詰まり(RX 256B 超過 / TX キュー満杯等の合算) |
| `pio device monitor` が文字化け | 仕様。データ UART はバイナリフレーム専用(§3) |
| Position でプロットが止まる/「途絶」 | Motive のストリーミング設定(§4.3)、`control.json` の `natnet`(アドレス/multicast/`rigid_body_id`)、ファイアウォール |
| UI 全体に「サーバー未接続」 | uvicorn プロセスの生死。ポート使用中なら `lsof -nP -iTCP:8000 -sTCP:LISTEN` で確認 |
| ログファイルができない | トグル ON か、シリアル接続中か(ログはセットポイント送信ごとに記録されるため、未接続・experiment モード中は行が増えない) |
| **(v2)「実験モードに入れませんでした(CMD_MODE ACK: bad_state)」** | 機体が WAIT 以外(飛行中・COMPLETE 遷移直後など)。状態バッジで WAIT を確認し「実験モードを有効化」を押し直す。COMPLETE なら先に Re-arm |
| **(v2)Experiment のバッジが「未有効」に戻った** | STOP/SPACE(CMD_STOP)で機体が MOTOR_TEST→WAIT に戻った。実験を続けるなら「実験モードを有効化」で再有効化する |
| **(v2)「実験テレメトリ(TLM_EXP)がありません」** | 機体が MOTOR_TEST 状態か(バッジ「有効」か)、機体リンクが緑か。TLM_EXP は MOTOR_TEST 中しか送出されない |
| **(v2)EKF バッジが「EKF注意」** | EKF 健全性低下(アンカー無効 / 磁気更新凍結 ffg bit5 / ffcal 未ロード)。飛行中ならヨー角制御は自動でレートダンピングに縮退している(OPERATION_GUIDE.md §13)。着陸後にアンカー再取得・FF 再適用・3D磁気再較正を検討 |
| **(v2)「FF未適用」警告バッジ** | ff_mode=0 のままヨー角制御 ON。FF プロファイルを適用する(§5.3、OPERATION_GUIDE.md §12)。未適用でも飛行は可能だが磁気干渉でヨーが劣化する |
| **(v2)FF 適用で「mag3d が取得時と一致しません」** | スイープ取得後に 3D磁気較正をやり直した等。原則はスイープ再取得 → 再抽出。force 適用は係数の前提が崩れることを理解した上で |
| **(v2)「Yaw 0」が拒否される** | FF 補正が有効(ff_mode≠0)。先に FF モードを off にする(FFプロファイルパネルの「モードのみ変更」) |
| **(v2)円軌道が開始できない** | MoCap が新鮮か、パラメータが制限内か(半径 0.05–1.5m・周期 3–120s・中心 ±2m)。「進行方向を向く」ON の場合は周期 8s 以上が必要 |
| **(v2)Multi の「選択適用」が失敗する** | リレーが旧ファーム(複数機非対応)の可能性 → §3 の手順で再書き込み。全機の `wifi_channel` 一致、MAC・`rigid_body_id` の設定・重複なしも確認(OPERATION_GUIDE.md §16) |
| **(v2)一斉スタートが拒否される** | 全機の目標が設定済みか、全機の MoCap(RB)が新鮮か、目標同士の XY 間隔が 0.5m 以上か。コンソールの理由表示を確認 |
| **(v2)接続後にフレームエラーが多発する(460800 化以降)** | リレーと PC のボーレート不一致。リレーの env(`release`=460800 / `release-115200`)と `server.json` の `serial.baudrate` を揃える(§3) |
