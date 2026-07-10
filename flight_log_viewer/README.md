# flight_log_viewer — V2 フライトログ可視化ツール

V2 の飛行ログ(50Hz・**100列**、`docs/LOG_STRUCTURE.md` v3 =
`pc_server/core/logger.py` の `COLUMNS` と 1 対 1)を可視化する
スタンドアロンツール群。旧 `Previous_Version/Drone_Log_Viewer`
(For_Research / For_Presentation)の静止画グラフ・同期アニメーション・
OpenCV トラッキング機能を V2 の列構成で再構築したもの。

本プロジェクトの主目的である**ヨー推定の評価**
(Madgwick / EKF / ジャイロ積算 / MoCap 真値の 4 系統比較、EKF 診断)に
重点を置いている。Posture / Position の単機ログに加え、
**Multi(複数機同時制御)** のグループログにも対応する。

## フォルダ構成(ログと動画の置き場)

```
StampFly_Integrated_Control_V2/
├── logs/
│   ├── flight_logs/     # 飛行ログ CSV の保存先
│   │   ├── <YYYYMMDD_HHMMSS>_posture.csv        # Posture 単機
│   │   ├── <YYYYMMDD_HHMMSS>_position.csv       # Position 単機
│   │   └── <YYYYMMDD_HHMMSS>_multi_<機体名>.csv  # Multi(機体数ぶん、同一 ts で 1 グループ)
│   └── videos/          # スマホ動画置き場(ユーザーが手動で置く。*.mp4 *.mov *.MOV *.avi)
└── flight_log_viewer/   # 本ツール
```

- ログ記録は「**START 押下〜着陸**」の区間。
- Multi のログは同一タイムスタンプ `<ts>` のファイル群(2〜4 機)で
  1 グループとして扱われる(mode 列は `"multi"`)。
- 旧 `logs/` 直下の `*.csv` も後方互換で対話モードの候補に含まれる。

## セットアップ

```bash
cd flight_log_viewer
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

- 必須: numpy / pandas / matplotlib
- **オプション**: `opencv-python` はスマホ動画の同期合成・ROI 追跡を使う場合
  のみ必要。静止画グラフ・レポート・動画なしアニメーションは OpenCV なしで
  動く。

  ```bash
  .venv/bin/pip install opencv-python
  ```

- アニメーション(MP4)出力には ffmpeg 本体が必要:
  `brew install ffmpeg`(macOS)/ `apt install ffmpeg`(Linux)

## 使い方

### 対話モード(推奨)

```bash
.venv/bin/python visualize.py
```

引数なしで実行(VSCode の実行ボタンも同じ)すると、次の順で対話選択する。
各ステップで `q` を入力すると中止できる。

1. **飛行ログモード選択**: [1] Posture / [2] Position / [3] Multi
   (`logs/flight_logs/` をファイル名サフィックスでフィルタ)
2. **出力内容メニュー**(モード別):
   - Posture / Position:
     [1] 静止画+レポート / [2] アニメ(動画なし) /
     [3] アニメ(スマホ動画同期) / [4] すべて / [5] 2 ログのヨー比較
   - Multi:
     [1] 静止画+レポート(全機) / [2] 複数機アニメ(共有XY・動画なし) /
     [3] すべて
3. **データ選択**: 単機 = CSV を番号選択 / Multi = タイムスタンプグループを
   番号選択(所属機体名を表示)。パス直接入力の選択肢もあり。
4. **動画選択**(動画同期を選んだ場合): `logs/videos/` の動画ファイルを
   番号選択(パス直接入力も可)。
5. **ROI 追跡枠**: y/N(y で cv2.selectROI により追跡対象を指定)

### バッチモード(CLI)

```bash
# 静止画グラフ+ヨー解析+サマリレポート(CSV のみ指定時の既定動作)
.venv/bin/python visualize.py ../logs/flight_logs/20260706_120000_position.csv

# アニメーション MP4(動画なし・ログのみ)
.venv/bin/python visualize.py ../logs/flight_logs/xxx_position.csv --animation

# スマホ動画と同期合成(opencv-python 必要)。--track で ROI 追跡枠を合成
.venv/bin/python visualize.py ../logs/flight_logs/xxx_position.csv --animation \
    --video ../logs/videos/flight.mp4 --track

# 切り出し(10〜30 秒区間のみ)・fps 指定
.venv/bin/python visualize.py ../logs/flight_logs/xxx.csv --animation --start 10 --end 30 --fps 15

# すべて生成
.venv/bin/python visualize.py ../logs/flight_logs/xxx.csv --all

# 2 ログのヨー安定性比較(RMS / ドリフト率 / NIS / ゲート発火の対照表)
.venv/bin/python visualize.py ../logs/flight_logs/a.csv --compare ../logs/flight_logs/b.csv

# Multi グループをタイムスタンプで指定して全出力
.venv/bin/python visualize.py --group 20260101_000000 --all

# Multi グループを代表 CSV で指定(グループ全機が処理される)
.venv/bin/python visualize.py --mode multi ../logs/flight_logs/20260101_000000_multi_droneA.csv --figures
```

- `--mode posture|position|multi` を付けると対話モードでもステップ 1 を
  スキップできる。
- `--logs-dir` はログ置き場の**ルート**(既定 `../logs`)で、その配下の
  `flight_logs/`・`videos/` を見る。`--output` で出力ルートを変更できる。
- Multi では動画合成(`--video`)・ROI 追跡(`--track`)は非対応
  (警告して無視)。`--compare` はエラー。

### 動画同期の前提

「**スマホ動画は START 押下(=ログ記録開始)と同時に録画開始**」を前提に、
ログの経過時間で動画をカットして同期する(動画側が長い分は捨てられる)。

## 出力物

### 単機(Posture / Position)— `output/<ログ名>/` 配下

| ファイル | 内容 | モード |
| --- | --- | --- |
| `01_xy_trajectory.png` | XY 軌跡+目標(円軌道の目標軌道も重畳) | Position |
| `02_attitude.png` | 姿勢: 指令 vs 実測 | 全 |
| `03_altitude.png` | 高度(目標/ToF/推定)と昇降速度 | 全 |
| `04_position_tracking.png` | 位置追従(目標 vs 実測+誤差) | Position |
| `05_pid_components.png` | XY PID 成分 | Position |
| `06_duty.png` | モーター duty(FL/FR/RL/RR) | 全 |
| `07_power.png` | 電圧(低電圧しきい値つき)/ 総電流 | 全 |
| `08_latency_loop_dt.png` | 往復レイテンシ / 機体 loop_dt | 全 |
| `09_mocap_diagnostics.png` | MoCap 診断(マーカー数・フレーム間隔) | Position |
| `10_yaw_four_sources.png` | **ヨー4系統比較**(Madgwick/EKF/ジャイロ積算/MoCap) | 全 |
| `11_yaw_error.png` | 対 MoCap ヨー誤差時系列+RMS/ドリフト率 [°/min] | 全 |
| `12_ekf_diagnostics.png` | EKF 診断(NIS・b_m・db̂・ffg ゲートタイムライン) | 全 |
| `13_ff_status.png` | ff_status タイムライン(ff_mode・アンカー等) | 全 |
| `14_yaw_tracking.png` | ヨー指令追従(PC 指令/機体適用目標/実測) | 全 |
| `15_xyz_3d.png` | **3D 軌跡**(plasma 時間カラー散布+カラーバー、始点緑/終点赤) | Position/Multi |
| `16_xy_time.png` | XY 軌跡の時間カラー散布版(plasma+カラーバー) | Position |
| `17_cmd_echo.png` | **指令エコー**: 送信指令 vs 機体適用エコー(Roll/Pitch 2 段重畳、伝達遅延確認) | 全 |
| `summary.txt` | テキストサマリ(飛行時間・RMS・ドリフト率・電圧推移) | 全 |
| `index.html` | 統計テーブル+全グラフをまとめた HTML レポート | 全 |
| `<ログ名>_animation.mp4` | 7 パネル同期アニメーション(1920×1080)。動画同期時は実写パネルが加わり `<ログ名>_animation_with_video.mp4` | 全 |

- 必要列にデータが無いグラフは自動でスキップされる
  (Posture では位置系 01/04/05/09/15/16 が出ない)。
- MoCap 真値が無いログのヨー誤差は Madgwick 基準の相対比較になる
  (レポートに注記が出る)。
- 比較モードは `output/compare_<A>_vs_<B>/comparison.html` に出力。

### Multi — `output/<ts>_multi/` 配下

| ファイル | 内容 |
| --- | --- |
| `M01_multi_xy.png` | 全機の XY 軌跡+目標を機体別色で共有プロット(○=開始 / ×=終了) |
| `<機体名>/` | 機体ごとのサブフォルダに単機と同じ図一式+`index.html`+`summary.txt` |
| `index.html` | Multi レポート: 全機サマリ表(記録/飛行時間・位置RMS・最大高度)+M01+機体別レポートへのリンク |
| `<ts>_multi_animation.mp4` | 複数機アニメーション: 左に共有 XY 大パネル(全機を機体別色で表示)、右に機体ごとの高度・ヨーパネル |

## ダミーログでの動作確認

実ログが無くても全機能を確認できる合成ログ生成スクリプトを同梱している:

```bash
# logs/flight_logs/ に 4 ファイル生成:
#   dummy_posture.csv / dummy_position.csv(40 秒・円軌道+ヨー指令ステップ)
#   20260101_000000_multi_droneA.csv / ..._droneB.csv(2 機グループ、再実行で上書き)
.venv/bin/python tests/make_dummy_log.py

# 単体モード指定(--mode {all,position,posture,multi}、既定 all)
.venv/bin/python tests/make_dummy_log.py --mode posture

# 全出力の確認
.venv/bin/python visualize.py ../logs/flight_logs/dummy_position.csv --all
.venv/bin/python visualize.py --group 20260101_000000 --all
```

ダミーログには Madgwick +1.5°/min・ジャイロ積算 −6°/min のドリフトと
NIS スパイク・ゲート発火が合成してあり、ヨー解析図の見え方を確認できる。

## 構成

```
flight_log_viewer/
├── visualize.py          # 対話式 CLI エントリポイント(モード選択→メニュー→データ選択)
├── viewer/
│   ├── constants.py      # 100 列定義・ffg/ff_status ビット・スタイル定数・機体別色
│   ├── loader.py         # CSV 読み込み(破損末尾復旧)+派生量計算+Multi グループ読込
│   ├── plots.py          # 静止画グラフ一式(01-09, 15-17)
│   ├── yaw_analysis.py   # ヨー4系統比較・EKF 診断(10-14、主目的)
│   ├── multi_plots.py    # Multi 用: 共有 XY 図(M01)+機体別図一式
│   ├── animation.py      # 7 パネルアニメーション(動画同期はオプション)+複数機アニメーション
│   ├── report.py         # サマリ / HTML / 2 ログ比較 / Multi レポート
│   ├── style.py          # ダークテーマ描画ヘルパー
│   └── jp_font.py        # 日本語フォント自動選択(Hiragino Sans 等)
├── tests/make_dummy_log.py  # 100 列ダミーログ生成(単機 2 本+Multi 2 機グループ)
└── output/               # 生成物(gitignore 済)
```

## 注意

- 列定義は `docs/LOG_STRUCTURE.md`(v3・100列)= `pc_server/core/logger.py` の
  `COLUMNS` と 1 対 1。
  列の過不足がある CSV は警告を出しつつ、読める範囲で可視化を続行する
  (必須列は `elapsed_time` のみ)。
- 破損した末尾(記録中の電源断など)は自動で切り捨てて読み込む。
- Multi ログの mode 列は `"multi"`。ファイル名 `<ts>_multi_<機体名>.csv` の
  `<ts>` でグループを束ねる。
