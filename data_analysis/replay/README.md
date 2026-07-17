# replay — 推定器のオフラインリプレイ基盤(Code Identity)

stampfly_ecosystem の ESKF リプレイ(`analysis/scripts/eskf_replay.cpp` +
`eskf_sweep.py`)と同じ方式を V2 に移植したもの。**ファームの推定器ソースを
コピーせずパス指定で g++ 直ビルド**し(Arduino.h は `stubs/` の最小スタブで
解決)、実機飛行ログを PC 上で再生・掃引する。リプレイで走る数式は実機と
バイト単位で同一のソース(Code Identity)なので、「PC で試した結果」が
そのまま「実機でこうなる」を意味する。

対象推定器(ファームツリーは一切変更しない。yaw 側は契約
「数式・符号・定数値変更禁止」の読み取り専用):

| 推定器 | ソース | ハーネス |
|---|---|---|
| 高度3状態KF | `firmware_stampfly/src/alt_kalman.cpp` (+`pid.cpp` の Filter) | `replay_alt.cpp` |
| ヨー4状態EKF | `firmware_stampfly/src/yaw_estimation/yaw_estimator_kf.cpp` (+`mag_calibration.cpp`) | `replay_yaw.cpp` |

## 使い方

```sh
cd data_analysis/replay
sh build.sh          # g++ -std=c++17 -O2 直ビルド → build/replay_alt, build/replay_yaw

# 高度KF: v4 飛行ログ(109列)の再生。--asis(現行=g単位バグ互換)/--fixed(m/s²是正)
# 省略時は両方を計算し、出力CSVは常に両列を持つ
./build/replay_alt ../../logs/flight_logs/20260717_195243_position.csv \
    --out out/20260717_195243_alt_replay.csv

# ヨーEKF: 合成データ自己試験(既定。EXIT=0 で全合格)
./build/replay_yaw --selftest
# ヨーEKF: 実ログの predict 経路のみ再生(磁気更新は下記の制約で不可)
./build/replay_yaw --csv ../../logs/flight_logs/20260717_195243_position.csv \
    --out out/20260717_195243_yaw_predict.csv

# パラメータ掃引(下記「sweep.py」参照)
python3 sweep.py alt --log ../../logs/flight_logs/20260717_195243_position.csv \
    --param R --values 1.6e-05,4e-04,4e-03
python3 sweep.py yaw --param FF_EKF_R_BASE_UT2 --values 2.0,4.0,8.0
```

出力: 時系列 CSV(`--out`)+ stdout サマリ。末尾の `RESULT k=v ...` /
`SELFTEST k=v ...` 1行が sweep.py の読む機械可読形式。

## replay_alt — 入力再構成と検証結果

v4 ログの `tlm_az_g`(= `sensor_state.Accel_z`、T=0.003 LPF 後の生加速度[g])を
`Accel_z_raw` の近似とし、sensor.cpp と同一の式・定数でファーム入力を再構成する:

```
Accel_z_d = raw_az_d_filter(tlm_az_g − Accel_z_offset)   [Filter T=0.1]
Az        = az_filter(−Accel_z_d)                        [Filter T=0.1]
z_sens    = tlm_altitude_tof_m(機上 alt_filter 済み値をそのまま)
kf.update(z_sens, Az[--fixed では ×9.80665], h)
```

- `Accel_z_offset`(機上 CALIBRATION 平均。ログに無い)は離陸前 WAIT 区間の
  `tlm_az_g` 平均で代替する。
- TLM_STATE は実効 ~23Hz(400Hz ループの間引き)なので、fresh サンプル間を
  400Hz サブステップ(入力は区間終端値のホールド)で刻んで機上周期を模擬する。
- 飛行中→WAIT 遷移では機上(`enter_wait()`/sensor.cpp の static リセット)と
  同様に KF・フィルタを reset する。
- mocap 比較は「mocap 系と機体高度系の原点差 = 定数オフセット」を平均で推定して
  除去した残差の RMS/max(飛行窓 TAKEOFF/HOVER/LANDING のみ)。

実ログ 2 本(20260717_195243 / 195648)での検証:

- **Code Identity 検証**: `--asis` の再生高度は機上 `tlm_altitude_est_m` を
  **RMS 0.9〜1.0 mm / max 10〜11 mm** で再現(速度も RMS ~4 mm/s)。
  ~23Hz ホールド供給でも再現誤差は mocap 誤差(~30 mm RMS)の 1/30 で、
  この基盤でのオフライン評価が実機挙動を代表することを確認した。
- **g単位バグ(9.81倍欠落)の効果**: 高度推定は R=(4mm)² の ToF 支配のため
  差は小さい(mocap RMS 30.4→31.2 mm / 34.5→35.4 mm と僅かに悪化方向)が、
  **鉛直速度は --fixed で明確に改善**: mocap 微分との RMS 0.096→0.076 m/s
  (−21%)/ 0.110→0.088 m/s(−20%)、相関 0.63→0.72 / 0.62→0.70。
  是正の主効果は高度そのものではなく z_dot 制御が使う速度推定に出る。

## replay_yaw — 自己試験と predict 再生

**(i) `--selftest`**: 既知真値の合成軌道(チルト運動学
ψ̇=((r−b_g)cosθ−p·sinθ)/cosφ と整合する p/r を逆算)+観測モデル整合の合成磁気
(10Hz、レベル化行列の逆変換で機体系へ戻す)で 120s 走らせ、以下を検証する:

- 収束窓(20〜45s)のヨー誤差 rms/max、b_g 同定誤差、NIS 平均・棄却率
- t=45s に +40° ステップ注入 → NIS 棄却 5s → ソフト再捕捉(ffg bit7)発動 →
  引き込み → 回復、のシーケンス

再捕捉の合否基準は「理想回復」ではなく**ファームの設計済み挙動**に合わせている:
実測では引き込みは 3°/更新クランプでなく P 収縮によるゲイン律速(~3.4°/s)で、
回復後は b_m がステップの一部を吸収して逆側に +6〜7° のテールが数十秒残る
(yaw_estimator_kf.cpp の recapture コメントに記載の既知挙動と一致)。

**(ii) `--csv`**: v4 ログのアダプタは **predict 経路のみ**。ジャイロ+roll/pitch を
fresh TLM レート(~23Hz)で供給する。磁気更新は**ログに生磁気が無いため再生不能**
(下記のログ拡張提案参照)。比較基準:

- `tlm_yaw_gyro_int_rad`(機上 400Hz ジャイロ単純積算)= predict 経路の忠実度
  チェック。実ログ 2 本で **RMS 0.7〜1.1° / max ~2°**(95s)— ~23Hz ホールド
  供給+チルト運動学の差はこの程度に収まる。
- `tlm_yaw_est_rad`(機上EKF)との差 = 磁気更新+b_g 学習の寄与
  (実ログで RMS 15〜20°/95s — これが「ログから再生できない部分」の大きさ)。

注意: 7/17 ログの `mocap_yaw_deg` は機体ヨーと無相関に ~−1800° 回転しており
(剛体マーカー配置のヨー曖昧性とみられる)、ヨー真値として使えない。
出力 CSV には参考列として残している。

## sweep.py — 定数掃引(オフライン評価専用)

掃引対象はファームヘッダの const/constexpr(`alt_kalman.hpp` の q1/q2/R/beta、
`yaw_config.hpp` の FF_EKF_*)。これらは**マクロガードされていないため -D では
上書きできない**(-D で同名マクロを定義すると定義行自体が壊れる)。そこで
「-D でビルドし直す」のと同じ意味になる方式として、掃引値ごとに
`build/sweep/` 配下へシャドウソースツリー(全ファイル symlink+対象ヘッダのみ
正規表現置換した生成コピー)を作り、`build.sh` を `FW_SRC=<shadow>` で再ビルド
して実行する。**ファームツリー本体には一切触れない**。

> **契約注意**: yaw_config.hpp は「数式・符号・定数値変更禁止」。本掃引は
> 「もし変えたら」のオフライン評価専用であり、実機値は不変。結果を反映する
> 場合はベンチ・飛行再検証が前提(eco の accel_att_lpf_hz と同じ運用)。

実行例(実ログでの alt R 掃引。id_asis_rms は Code Identity の指標なので
ベースライン値 1.6e-05 で最小になるのが正しい):

```
             R |      id_asis_rms |   mocap_asis_rms |  mocap_fixed_rms
       1.6e-05 |          0.00091 |          0.03037 |          0.03118
        4e-04  |          0.00812 |          0.03442 |          0.03294
        4e-03  |          0.01805 |          0.04322 |          0.03394
```

## 制約まとめ

| 制約 | 内容 |
|---|---|
| ログレート | TLM_STATE 実効 ~23Hz。400Hz サブステップ(終端値ホールド)で模擬。忠実度は実測で高度 RMS ~1mm、ヨー predict RMS ~1°(上記) |
| フィルタ済み入力 | `tlm_az_g`/`tlm_r_rad_s` 等は T=0.003 LPF 後(機上EKFは未フィルタ値を使う)。~23Hz ではほぼ透過だが振動帯域はエイリアスされる |
| Accel_z_offset | ログに無いため WAIT 接地区間平均で代替(7/17 ログは WAIT 1〜2 サンプルしかなく精度限界。--fixed では KF バイアス状態が残差を吸収) |
| 生磁気なし | ヨーEKFの磁気更新(update)は完全再生不能。predict 経路のみ |
| mocap ヨー | 7/17 ログでは無相関回転で真値に使えない(高度 raw_pos_z は有効) |

## ログ拡張提案(完全再生に必要な最小セット。実装はしない)

1. **TLM_STATE 末尾に FF 補正後レベル磁気 2 成分を追加**(`lvl_bx_ut`,
   `lvl_by_ut`, float×2 = 8B)。EKF update の観測 z そのものが残るため、
   ゲート(norm/z は機上判定済みとして)込みの磁気更新をログレートで再生できる。
   併せて適応 R の入力 `σ_ff/σ_slew/σ_diff` の合成 1 値(`sigma_mag_ut`)が
   あれば R_eff も再現可能。
2. **アンカー情報のイベント送信**: 再アンカー時に `psi0, b0h_x, b0h_y, B0.z,
   ‖B0‖` を TLM_EVENT で 1 回送る(update のゲート基準・観測モデルの基準値)。
3. **未フィルタ ω_z と起動オフセット**: `Yaw_rate_offset` を 1 回送るだけでも
   predict の規約(未フィルタ − offset)を厳密に再現できる。
4. **mocap ヨー真値の復旧**: 剛体マーカーの非対称化(ヨー曖昧性の解消)。
   高度側と同様に真値誤差そのものを掃引の目的関数にできるようになる。

## ファイル構成

```
replay/
├── README.md          # 本ファイル
├── build.sh           # Code Identity ビルド(FW_SRC/OUT 環境変数で sweep が再利用)
├── stubs/Arduino.h    # 最小スタブ(PI/DEG_TO_RAD/RAD_TO_DEG + stdint/math)
├── replay_alt.cpp     # 高度KFリプレイ(--asis/--fixed)
├── replay_yaw.cpp     # ヨーEKF 自己試験(--selftest)+ predict 再生(--csv)
├── sweep.py           # 定数掃引ラッパ(シャドウビルド方式)
├── build/             # ビルド物・シャドウツリー(.gitignore)
└── out/               # リプレイ出力 CSV(.gitignore)
```
