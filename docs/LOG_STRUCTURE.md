# StampFly Integrated Control — フライトログ形式(v3・100列)

本書は `pc_server/core/logger.py` が生成する CSV の列構成を定義する。
列定義の実体は `logger.py` の `COLUMNS` であり、本書と1対1で対応させること。
語彙は旧 `NatNet_PID_Controller/LOG_STRUCTURE.md`(57列)を継承し、機体テレメトリ
(TLM_STATE)列で拡張している。

v2 では v1(77列)の**末尾に 17 列を追加**した(既存 77 列の名前・順序は不変):
ヨー指令 3 列(`cmd_yaw_ref_rad` / `cmd_yaw_ref_deg` / `yaw_ctrl_on`)、
TLM_STATE 末尾拡張のスナップショット 11 列(`tlm_yaw_est_rad` 〜 `tlm_ff_status`)、
MoCap ヨーと軌道状態 3 列(`mocap_yaw_deg` / `traj_mode` / `traj_phase_rad`)。
詳細は「v2 追加列」節を参照。

v3 では v2(94列)の**末尾に 6 列を追加**した(既存 94 列の名前・順序は不変):
機上XY制御(CMD_POS_ERR)診断 6 列(`xy_cmd_mode` / `cmd_err_x_m` /
`cmd_err_y_m` / `cmd_xy_valid` / `cmd_mocap_yaw_deg` / `mocap_heading_deg`)。
詳細は「v3 追加列」節を参照。flight_log_viewer(`viewer/constants.py` の
`V2_COLUMNS`)はこの 100 列契約を正として読み込む。

## 出力先・行レート

- 出力先: リポジトリ直下 `logs/YYYYMMDD_HHMMSS_<mode>.csv`(`<mode>` = `posture` |
  `position`。experiment モード中もログトグル ON ならファイルは開かれるが、
  50Hz 送信が停止しているため行は記録されない)
- ログ ON のとき、CMD_SETPOINT の送信(50Hz)ごとに1行を出力する。
- 各行には「その送信時点での最新テレメトリ/mocap スナップショット」を結合する。

## 書式と書き込み挙動(`logger.py` 実装)

- 列数は 100(1行目がヘッダ)。順序は `COLUMNS` の宣言順(v1 の 77 列 →
  v2 追加 17 列 → v3 追加 6 列の順)。
- float は小数 6 桁の固定表記(`FLOAT_DECIMALS = 6`)。bool は `"1"`/`"0"`、
  `None`(未取得)は空文字で出力される。
- `timestamp` / `elapsed_time` は `log_row()` が自動付与する(呼び出し側が
  明示指定した場合はその値が優先)。
- ディスクへのフラッシュは 50 行ごと(`server.json` の
  `logging.flush_every_rows`。50Hz なら約1秒ごと)。異常終了時は最大で
  直近1秒ぶんの行が失われ得る。
- ファイルのライフサイクル: UI のログトグル ON かつシリアル接続中に開かれ、
  トグル OFF・切断・サーバ終了で閉じる。**モード切替時はファイル名に
  `<mode>` を含むため新しいファイルを開き直す**(1ファイル1モード)。
- 行が記録されるのは 50Hz 送信スレッドの動作中(=シリアル接続中)のみ。
  送信失敗時も行は出力され、`send_success=0`・`command_sequence` 空となる。

## 座標系と単位

- 位置(`pos_*`, `raw_pos_*`, `target_*`)は Motive 座標を制御座標系へ変換した後の値 [m]。
  変換は `config/control.json` の `coordinate_transform`(既定: 制御x←Motive z、
  制御y←−Motive x、制御z←Motive y)。旧システムの y 軸負ゲイン規約は廃止し、
  符号はこの座標変換に移した(そのため y 座標の符号は旧ログと反転している)。
- `_rad` はラジアン、`_deg` は度。`_rad_s` は rad/s。
- 時間系(`*_ms`)はミリ秒。`elapsed_time` は秒(time.monotonic 基準)。
- 0/1 フラグは文字列 `"0"`/`"1"`。値が未取得の列は空文字。

## 列リファレンス

### セッション / タイミング

| 列 | 型 | 説明 |
| --- | --- | --- |
| `timestamp` | ISO8601文字列 | 行の記録時刻(ローカル壁時計、ミリ秒精度)。 |
| `elapsed_time` | float (s) | ログ開始からの経過秒(monotonic)。 |
| `mode` | string | `posture` または `position`。 |
| `phase` | string | セッションフェーズ(`idle`/`connected`/`armed`/`flying`)。 |
| `command_sequence` | int | この行で送信した CMD_SETPOINT の seq(送信失敗時は空)。 |
| `send_success` | 0/1 | シリアル書き込み成功フラグ。 |
| `feedback_latency_ms` | float (ms) | TLM_STATE の `seq_echo` から計測した直近の往復遅延。未計測時は空。 |

### 指令(送信した CMD_SETPOINT)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `roll_ref_rad`, `pitch_ref_rad` | float | rad | 実際に送信した角度指令(**機体バイアス加算後**)。 |
| `roll_ref_deg`, `pitch_ref_deg` | float | deg | 上記の度数表記。 |
| `alt_ref_m` | float | m | 送信した目標高度。 |
| `roll_bias_deg`, `pitch_bias_deg` | float | deg | 適用中の機体プロファイルバイアス(指令から引けばバイアス前の値になる)。 |

### 位置と誤差(Position モードのみ。Posture モードでは空)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `pos_x`, `pos_y`, `pos_z` | float | m | 制御に使用したフィルタ後位置。 |
| `raw_pos_x`, `raw_pos_y`, `raw_pos_z` | float | m | フィルタへ入力したリジッドボディ生位置。 |
| `error_x`, `error_y` | float | m | 目標位置に対する XY 誤差。 |
| `target_x`, `target_y`, `target_z` | float | m | 目標位置(z は目標高度の指令元)。 |

### PID 成分(Position モードのみ)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `pid_x_p`, `pid_x_i`, `pid_x_d` | float | rad相当 | X軸PID(ロール)の P/I/D 成分。 |
| `pid_y_p`, `pid_y_i`, `pid_y_d` | float | rad相当 | Y軸PID(ピッチ)の P/I/D 成分。ゲインは正値(符号は座標変換側)。 |

### フィルタ状態とデータ由来(Position モードのみ)

| 列 | 型 | 説明 |
| --- | --- | --- |
| `data_valid` | 0/1 | PID 更新に有効と判断されたか(信頼度・外れ値・トラッキング・フレーム間隔を考慮)。 |
| `control_active` | 0/1 | XY 閉ループが有効か(Start 受理後のみ 1)。 |
| `mocap_dropout` | 0/1 | MoCap 途絶(>300ms)によりセットポイントを水平固定中か。 |
| `is_outlier` | 0/1 | フィルタが外れ値と判定したか。 |
| `used_prediction` | 0/1 | 生データではなく予測位置を使用したか。 |
| `confidence` | float (0-1) | フィルタの信頼度スコア。 |
| `consecutive_outliers` | int | 連続外れ値フレーム数。 |
| `data_source` | string | `"rigid_body"` または `"none"`(本システムはリジッドボディのみ。マーカー重心フォールバックは廃止)。 |
| `filter_threshold` | float (m) | 外れ値判定に使った動的距離しきい値。 |
| `tracking_valid` | 0/1 | Motive の `tracking_valid`。 |

### リジッドボディ / フレーム診断(Position モードのみ)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `rb_error` | float | - | NatNet が報告するリジッドボディ解法エラー。 |
| `rb_marker_count` | int | count | リジッドボディ解法に寄与したマーカー数。 |
| `frame_number` | int | - | 最新の NatNet フレーム番号。 |
| `marker_count` | int | count | このフレームの有効マーカー数(= `rb_marker_count`)。 |
| `frame_dt_ms` | float | ms | 連続する NatNet フレーム間の時間差。 |
| `mocap_age_ms` | float | ms | 最後の有効 pose からこの行までの経過。 |

### 機体テレメトリ(最新 TLM_STATE のスナップショット。未受信時は空)

PROTOCOL.md の TLM_STATE(97B)全フィールドに `tlm_` 接頭辞を付けて記録する。

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_age_ms` | float | ms | TLM_STATE 受信からこの行までの経過(鮮度判定用)。 |
| `tlm_seq_echo` | u32 | - | 機体が最後に適用した CMD_SETPOINT の seq。 |
| `tlm_elapsed_ms` | u32 | ms | 機体起動からの経過。 |
| `tlm_state` | u8 | - | FlightState 数値(0=INIT … 6=COMPLETE)。 |
| `tlm_state_name` | string | - | 上記の名前表記。 |
| `tlm_flags` | u8 | - | bit0 low_voltage, bit1 setpoint_fresh, bit2 flying。 |
| `tlm_reason` | u8 | - | 直近の遷移理由(Reason 数値)。 |
| `tlm_reason_name` | string | - | 上記の名前表記。 |
| `tlm_roll_rad`, `tlm_pitch_rad`, `tlm_yaw_rad` | float | rad | 実測姿勢(AHRS)。yaw は ±π・リセット時 0(旧ファームのログは [-2π, 0]。比較時は wrap_deg で正規化する)。 |
| `tlm_p_rad_s`, `tlm_q_rad_s`, `tlm_r_rad_s` | float | rad/s | 実測角速度。 |
| `tlm_roll_ref_rad`, `tlm_pitch_ref_rad` | float | rad | 機体側で適用中の角度指令(往復確認用)。 |
| `tlm_alt_ref_m` | float | m | 機体側で適用中の目標高度。 |
| `tlm_altitude_tof_m` | float | m | ToF 生値。 |
| `tlm_altitude_est_m` | float | m | カルマン推定高度。 |
| `tlm_alt_velocity_m_s` | float | m/s | 高度速度。 |
| `tlm_z_dot_ref_m_s` | float | m/s | 高度速度指令。 |
| `tlm_voltage_v` | float | V | バッテリ電圧。 |
| `tlm_duty_fr`, `tlm_duty_fl`, `tlm_duty_rr`, `tlm_duty_rl` | float | 0–1 | モータデューティ。 |
| `tlm_ax_g`, `tlm_ay_g`, `tlm_az_g` | float | g | フィルタ後加速度。 |
| `tlm_loop_dt_us` | u16 | µs | 機体の直近制御周期実測値。 |

## v2 追加列(ファイル上の末尾 17 列。順序は下表の掲載順)

### ヨー指令(送信した CMD_SETPOINT のヨー目標)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `cmd_yaw_ref_rad` | float | rad | 送信した CMD_SETPOINT のヨー角目標(整形済み)。ヨー制御 OFF 時は 0。 |
| `cmd_yaw_ref_deg` | float | deg | 上記の度数表記。 |
| `yaw_ctrl_on` | 0/1 | - | flags bit1(FLAG_YAW_REF_VALID)を立てて送信したか(=ヨー角制御 ON)。 |

### 機体テレメトリ v2 拡張(TLM_STATE オフセット 97 以降のスナップショット。未受信時は空)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_yaw_est_rad` | float | rad | アクティブ推定器ヨー(est_mode=1 なら EKF ψ、0 なら補正CF。ff/est 未設定時はリファレンスCF)。 |
| `tlm_yaw_gyro_int_rad` | float | rad | Z軸角速度の単純積算(400Hz、ahrs_reset でゼロクリア)。 |
| `tlm_yaw_ref_rad` | float | rad | 機体側で適用中のヨー目標(途絶ラッチ後含む。ヨー制御 off 時 0)。 |
| `tlm_current_a` | float | A | 総電流(INA3221 CH2、20Hz 更新)。 |
| `tlm_db_hat_x_ut`, `tlm_db_hat_y_ut` | float | µT | 適用中の FF 補正ベクトル ΔB̂ の x/y。 |
| `tlm_bm_x_ut`, `tlm_bm_y_ut` | float | µT | EKF 磁気バイアス状態 b_m の x/y。 |
| `tlm_nis` | float | - | 直近 EKF 更新の NIS。 |
| `tlm_ffg` | u8 | - | EKF ゲート/健全性ビット(PROTOCOL.md の ffg 定義参照)。 |
| `tlm_ff_status` | u8 | - | bit0-1 ff_mode, bit2 est_mode(EKF), bit3 anchor_valid, bit4 ffcal_loaded, bit5 yaw_ctrl_active, bit6 mag_fresh。 |

### MoCap ヨー・軌道状態(Position モードのみ。Posture モードでは空)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `mocap_yaw_deg` | float | deg | MoCap リジッドボディのヨー真値(mocap.py の yaw_rad を deg 換算。制御には未使用・比較用)。 |
| `traj_mode` | int | - | 軌道モード: hover=0 / circle=1。 |
| `traj_phase_rad` | float | rad | 円軌道の現在位相(±π)。**hover 時は空欄**。MoCap 途絶中は位相凍結のため直近値のまま。 |

## v3 追加列(ファイル上の末尾 6 列。機上XY制御 CMD_POS_ERR 診断)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `xy_cmd_mode` | str | - | XY 指令モードの判別: `pc`(CMD_SETPOINT: PC 側 PID)/ `onboard`(CMD_POS_ERR: 機上XY PID)。 |
| `cmd_err_x_m`, `cmd_err_y_m` | float | m | 送信した XY 位置誤差(制御座標系、クランプ後)。onboard モードのみ。bit2=0(無効)中も実誤差を送る。 |
| `cmd_xy_valid` | 0/1 | - | flags bit2(FLAG_XY_ERR_VALID)を立てて送信したか(閉ループ有効+データ有効+MoCap 非途絶)。onboard モードのみ。 |
| `cmd_mocap_yaw_deg` | float | deg | 送信した MoCap 実測ヨー(mocap_yaw、制御座標系)。heading 未取得時は空欄。onboard モードのみ。 |
| `mocap_heading_deg` | float | deg | MoCap 実測の制御座標系ヨー(リジッドボディ前方軸の方位)。pc / onboard 両モードで記録され、機体推定ヨー(`tlm_yaw_est_rad`)とのフレーム整合検証に使う。 |

onboard モードでは PC は roll/pitch 角度指令を計算しないため、
`roll_ref_rad` / `pitch_ref_rad` 等の指令列は 0(バイアスも非加算)になる。
機体側で計算された実際の姿勢指令は `tlm_roll_ref_rad` / `tlm_pitch_ref_rad`
(TLM_STATE スナップショット)に現れる。

## 旧形式からの主な変更点

- `feedback_roll_rad`/`feedback_pitch_rad` 等の `feedback_*` 列は、TLM_STATE 全体を
  `tlm_*` 列として記録する形式に置き換えた(`tlm_roll_ref_rad`/`tlm_pitch_ref_rad` が
  旧 feedback 列に対応)。
- `feedback_sequence` → `tlm_seq_echo`、`feedback_age_ms` → `tlm_age_ms`。
- `loop_time_ms` は廃止(行レートは送信スレッドの 50Hz 固定。機体側の実周期は
  `tlm_loop_dt_us` を参照)。
- `rb_pos_*`, `rb_q*`, `rb_roll_deg` 等のリジッドボディ姿勢列は、姿勢が TLM(AHRS)で
  得られるため `rb_error`/`rb_marker_count` のみ残した(mocap yaw は UI 表示専用)。
- Posture モードでも同一スキーマを使用し、位置系の列は空となる。
