# data_analysis — FF 係数抽出・計測データ解析ツール

pc_server が取得したスイープ結果 / Experiment 計測ログをオフラインで解析する
独立 venv のツール群。スクリプトは 3 本のみで、いずれも**引数なしで起動すると
番号選択の対話モード**、引数を渡すと CLI として動く(`--help` あり、`q` で中止)。

| スクリプト | 役割 | 入力 | 出力 |
|---|---|---|---|
| `plot_sweep.py` | スイープ 1 本の校正解析(図12枚)+加算性シーケンス検証 | `sweep_*_samples.csv` / `sequence_*_meta.json` | `graphs/sweep_<stamp>/`・`graphs/additivity_<stem>/` |
| `make_ff_profile.py` | スイープ結果 → FF プロファイル JSON 抽出 | スイープ 8 本(または全機 4 本+sequence meta) | `../pc_server/data/ff_profiles/<name>.json` |
| `plot_explog.py` | Experiment 計測ログのグラフ化(図6枚+summary.txt) | `../pc_server/data/exp_logs/explog_*.csv` | `graphs/explog_<stamp>/` |

`graphs/` は出力専用ディレクトリ(生成物)。数値ロジック本体は `ff_params/core.py`
(純粋関数ライブラリ)にあり、`tests/test_ff_extraction.py` が受入テスト。

## セットアップ

```sh
cd data_analysis
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt   # numpy, matplotlib
.venv/bin/python tests/test_ff_extraction.py  # 受入テスト(pytest 不要、約1秒)
```

## ① plot_sweep.py — スイープのグラフ化・加算性検証

```sh
# 対話モード: [1] スイープ1本 / [2] 加算性シーケンス → 一覧から番号選択(Enter=最新)
.venv/bin/python plot_sweep.py

# CLI: 拡張子/名前で自動判別(.csv → スイープ校正解析、sequence_*_meta.json → 加算性)
.venv/bin/python plot_sweep.py ../pc_server/data/sweep_results/sweep_20260612_202215_samples.csv
.venv/bin/python plot_sweep.py ../pc_server/data/sweep_results/sequence_XXXX_meta.json
# オプション: -o <出力dir> / --results-dir <探索dir>(既定 ../pc_server/data/sweep_results/)
```

- スイープ: ΔB 3D散布・ΔB vs 電流(全域/飛行帯 0.5–0.8 フィット)・|ΔB|・std・
  基準ドリフト・品質チェック・ヒステリシス・サンプル回帰・温度など PNG 12 枚。
- 加算性: Σ単機 vs 全機の比較図+判定サマリ PNG。しきい値は
  `max(4σ·RMS(noise), 0.5 µT)`。判定は stdout にも出る。

## ② make_ff_profile.py — FF プロファイル抽出

pc_server の UI「FF 抽出」からサブプロセスとして呼ばれるのと同じ CLI。
手動でも使える。**入力は「展開後にちょうど 8 本」**:

- **8 本指定(従来)**: 全機 FL+FR+RL+RR × 4 姿勢(Yaw=0°/90°/±180°/-90°)+
  単機 FL/FR/RL/RR 各 1 本の sweep stem 8 個。
- **5 ファイル指定(シーケンス)**: 全機 4 姿勢の sweep stem 4 個+
  `sequence_*_meta.json` 1 個。sequence meta 内の単機 4 ラン
  (phase=done・非 aborted のみ)が自動展開され、従来の 8 本と同じ入力集合になる。

```sh
# 対話モード: sweep_results のサブフォルダ一覧 or stems 手動選択 → name/memo 入力
.venv/bin/python make_ff_profile.py

# フォルダ指定(8ペア、または 4ペア+sequence meta 入り)
.venv/bin/python make_ff_profile.py --folder ../pc_server/data/sweep_results/<セット名>

# stems 指定(8個、または全機4個+sequence meta で計5個)
.venv/bin/python make_ff_profile.py --stems sweep_A sweep_B ... --results-dir <dir>

# 共通オプション: --name <プロファイル名> --memo <1行メモ> -o <出力先> --plots(検証図PNG)
```

既定出力は `../pc_server/data/ff_profiles/<name>.json`(stampfly_ff_profile v1)。
中断ラン(meta.aborted=true)は stem 列挙付きでエラー拒否。詳細仕様は
`docs/FF_PIPELINE.md`。

## ③ plot_explog.py — Experiment 計測ログのグラフ化

```sh
# 対話モード: ../pc_server/data/exp_logs/explog_*.csv を新しい順に番号選択(Enter=最新)
.venv/bin/python plot_explog.py

# CLI
.venv/bin/python plot_explog.py ../pc_server/data/exp_logs/explog_20260710_153000.csv [-o dir]
```

図 6 枚+`summary.txt`(ドリフト率・NIS 統計・ゲート発火数など):

1. `01_yaw_comparison.png` — ヨー3系統(ジャイロ積分/推定/Madgwick)重畳+duty・モーターON帯
2. `02_yaw_error.png` — 開始基準ドリフトとジャイロ積分基準差(RMS/最大付き)
3. `03_current_duty.png` — 電流・バッテリ電圧・duty
4. `04_mag_ff.png` — Δb_cal と FF 予測 db_hat の重畳+残差(x/y/z)
5. `05_ekf_diagnostics.png` — NIS(閾値 2.0/5.99/13.8)+ffg 8ゲートのラスタ(bit7=ソフト再捕捉「再捕捉中」を含む)
6. `06_ff_status.png` — ff_mode(off/A/B)ステップ+ステータスフラグのラスタ

併設の `explog_<ts>_meta.json` があればプロファイル名・ff/est モードを
タイトルとサマリに表示(無くても動作)。必要列が欠けた図は理由付きでスキップ。

## テスト

```sh
.venv/bin/python tests/test_ff_extraction.py   # EXIT=0 で全合格
```

6/12 実測 8 本と `tests/fixtures/results.json`(真値)の照合、付録 A 再現、
sequence meta 展開の検証を含む。数値挙動を変える変更をしたら必ず実行すること。
