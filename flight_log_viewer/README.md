# flight_log_viewer — V2 フライトログ可視化ツール

V2 の `logs/*.csv`(50Hz・94列、docs/LOG_STRUCTURE.md v2)を可視化する
スタンドアロンツール群。旧 `Previous_Version/Drone_Log_Viewer`
(For_Research / For_Presentation)の静止画グラフ・同期アニメーション・
OpenCV トラッキング機能を V2 の列構成で再構築したもの。

本プロジェクトの主目的である**ヨー推定の評価**
(Madgwick / EKF / ジャイロ積算 / MoCap 真値の 4 系統比較、EKF 診断)に
重点を置いている。

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

1. 出力内容を選択(静止画+レポート / アニメーション / 比較)
2. `../logs/` の CSV から対象を選択(パス直接入力も可)
3. `output/<ログ名>/` に生成される

### バッチモード

```bash
# 静止画グラフ+ヨー解析+サマリレポート(CSV のみ指定時の既定動作)
.venv/bin/python visualize.py ../logs/20260706_120000_position.csv

# アニメーション MP4(動画なし・ログのみ)
.venv/bin/python visualize.py ../logs/xxx.csv --animation

# スマホ動画と同期合成(opencv-python 必要)。--track で ROI 追跡枠を合成
.venv/bin/python visualize.py ../logs/xxx.csv --animation \
    --video ~/Movies/flight.mp4 --track

# 切り出し(10〜30 秒区間のみ)・fps 指定
.venv/bin/python visualize.py ../logs/xxx.csv --animation --start 10 --end 30 --fps 15

# すべて生成
.venv/bin/python visualize.py ../logs/xxx.csv --all

# 2 ログのヨー安定性比較(RMS / ドリフト率 / NIS / ゲート発火の対照表)
.venv/bin/python visualize.py ../logs/a.csv --compare ../logs/b.csv
```

### 動画同期の前提

旧ビューアと同じく「**スマホ動画はログ記録開始と同時に録画開始**」を前提に、
ログの経過時間で動画をカットして同期する(動画側が長い分は捨てられる)。

## 出力物

`output/<ログ名>/` 配下:

| ファイル | 内容 |
| --- | --- |
| `01_xy_trajectory.png` | XY 軌跡+目標(円軌道の目標軌道も重畳) |
| `02_attitude.png` | 姿勢: 指令 vs 実測 |
| `03_altitude.png` | 高度(目標/ToF/推定)と昇降速度 |
| `04_position_tracking.png` | 位置追従(目標 vs 実測+誤差) |
| `05_pid_components.png` | XY PID 成分 |
| `06_duty.png` | モーター duty(FL/FR/RL/RR) |
| `07_power.png` | 電圧(低電圧しきい値つき)/ 総電流 |
| `08_latency_loop_dt.png` | 往復レイテンシ / 機体 loop_dt |
| `09_mocap_diagnostics.png` | MoCap 診断(マーカー数・フレーム間隔) |
| `10_yaw_four_sources.png` | **ヨー4系統比較**(Madgwick/EKF/ジャイロ積算/MoCap) |
| `11_yaw_error.png` | 対 MoCap ヨー誤差時系列+RMS/ドリフト率 [°/min] |
| `12_ekf_diagnostics.png` | EKF 診断(NIS・b_m・db̂・ffg ゲートタイムライン) |
| `13_ff_status.png` | ff_status タイムライン(ff_mode・アンカー等) |
| `14_yaw_tracking.png` | ヨー指令追従(PC 指令/機体適用目標/実測) |
| `summary.txt` | テキストサマリ(飛行時間・RMS・ドリフト率・電圧推移) |
| `index.html` | 統計テーブル+全グラフをまとめた HTML レポート |
| `<ログ名>_animation.mp4` | 7 パネル同期アニメーション |

- Posture モードのログでは位置系のグラフは自動でスキップされる。
- MoCap 真値が無いログのヨー誤差は Madgwick 基準の相対比較になる
  (レポートに注記が出る)。
- 比較モードは `output/compare_<A>_vs_<B>/comparison.html` に出力。

## ダミーログでの動作確認

実ログが無くても全機能を確認できる合成ログ生成スクリプトを同梱している:

```bash
# ../logs/dummy_position.csv を生成(40 秒・円軌道+ヨー指令ステップ)
.venv/bin/python tests/make_dummy_log.py

# posture モード(位置・MoCap 列が空欄)
.venv/bin/python tests/make_dummy_log.py --mode posture

# 全出力の確認
.venv/bin/python visualize.py ../logs/dummy_position.csv --all
```

ダミーログには Madgwick +1.5°/min・ジャイロ積算 −6°/min のドリフトと
NIS スパイク・ゲート発火が合成してあり、ヨー解析図の見え方を確認できる。

## 構成

```
flight_log_viewer/
├── visualize.py          # 対話式 CLI エントリポイント
├── viewer/
│   ├── constants.py      # 94 列定義・ffg/ff_status ビット・スタイル定数
│   ├── loader.py         # CSV 読み込み(破損末尾復旧)+派生量計算
│   ├── plots.py          # 静止画グラフ一式
│   ├── yaw_analysis.py   # ヨー4系統比較・EKF 診断(主目的)
│   ├── animation.py      # 7 パネルアニメーション(動画同期はオプション)
│   ├── report.py         # サマリ / HTML / 2 ログ比較
│   ├── style.py          # ダークテーマ描画ヘルパー
│   └── jp_font.py        # 日本語フォント自動選択(Hiragino Sans 等)
├── tests/make_dummy_log.py  # 94 列ダミーログ生成
└── output/               # 生成物(gitignore 済)
```

## 注意

- 列定義は `docs/LOG_STRUCTURE.md`(v2・94列)= `pc_server/core/logger.py` の
  `COLUMNS` と 1 対 1。列の過不足がある CSV は警告を出しつつ、読める範囲で
  可視化を続行する。
- 破損した末尾(記録中の電源断など)は自動で切り捨てて読み込む。
