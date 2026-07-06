# 研究テーマ検討書 — StampFly+MoCap プラットフォームを実機検証基盤とする次期研究

作成日: 2026-07-06
位置づけ: 研究計画のための調査・検討文書(V2 システムの実装仕様ではない)。

## 0. この文書について(調査方法と信頼度の読み方)

2023〜2026 年のマイクロドローン研究動向を、(1) 多段検証付きディープリサーチ
(各主張を 3 名の独立検証者が反証を試み、生き残った主張のみ採用)、(2) 6 領域の補完文献調査
(主要論文はアブストラクトを実際に取得して確認)の二段で調査した。信頼度の凡例:

- **[検証済]** — 3-0 票で反証に耐えた主張(最高信頼)
- **[確認済]** — 論文のアブストラクト/書誌を直接取得して確認
- **[未確認]** — 検索結果スニペット由来(着手前に原典確認を推奨)
- 「確認できず」— 本調査の検索範囲で発見できなかったという意味であり、存在しない証明ではない

---

## 1. 前提: 保有プラットフォームの資産棚卸し

研究テーマの差別化は「他の研究室が簡単には持っていないもの」から生まれる。現有資産:

| 資産 | 研究上の意味 |
|---|---|
| 36g 級機体+**400Hz 完全自作ファームウェア** | 制御・推定アルゴリズムを最下層から差し替え可能(市販FC・PX4 では不可能な自由度) |
| **モーター電流計測(INA3221)+duty→電流モデル(機体・モーター毎)** | 世界的にも珍しい計測チャネル(§2.3, §2.6, §2.7 で空白と確認) |
| **電流×磁場スイープ較正+FF補正+4状態EKFヨー推定(実装済み)** | 直接の先行研究が 2019 年の通常サイズ機 1 本のみという空白領域に既に立っている |
| OptiTrack MoCap(真値)+94列 50Hz ログ+flight_log_viewer | 推定器・制御器の定量評価がすぐできる。真値付きベンチマークの作法が整備済み |
| **ESP-NOW リンク+per-packet レイテンシ計測内蔵** | ネットワーク化制御(NCS)実験で「実測遅延」を制御に使える稀有な構成(§2.5) |
| **同一設計 5 機体** | 較正の機体間転移・フリート統計・マルチ機体実験(2機ペア止まりの文献が多い) |
| ESP32-S3(240MHz, FPU) | 文献の標準機 Crazyflie の STM32F405(168MHz)と同等以上 **[検証済]** |

---

## 2. 世界動向の要約(2023–2026)

### 2.1 組込み最適制御 — マイコン上 MPC が国際トップ会議テーマとして確立 **[検証済]**

- **TinyMPC**(Nguyen, Schoedel, Alavilli, Plancher, Manchester, ICRA 2024)が転換点。
  ADMM ベースの省メモリ凸 MPC ソルバで OSQP 比約1桁高速、27g Crazyflie
  (STM32F405)上のオンボード実機で軌道追従・動的障害物回避を実証。
  **ICRA 2024 Best Paper Award in Automation 受賞** [R1-R4]。
- 系譜は活発に継続: **Conic-TinyMPC**(SOCP 対応+コード生成、既存比 10.6〜142.7 倍、
  ICRA 2026 採択)[R5]、Adaptive 版(IROS 2025)[R6]、能動集合法 **DAQP** の
  低レベル MPC(最悪 1375µs @168MHz、2026 プレプリント)[R7]。
- **学習ベース MPC のオンボード化はフロンティア**: Tiny LB MPC(Mechatronics 2025)が
  「53g 機で世界初」を主張する段階(ただし Teensy 4.0 = 600MHz を追加している)[R8]。
  より弱い計算資源(ESP32-S3 級)での実現は未踏。

### 2.2 学習ベース制御 — sim-to-real のレシピが体系化 **[検証済]**

- **Swift**(Kaufmann et al., Nature 2023): シミュレーション内 PPO 訓練+実機データ約50秒の
  経験的残差モデルで人間チャンピオンに勝利。sim-to-real の正典 [R9]。
- **Learning to Fly in Seconds**(Eschmann et al., RA-L 2024): ラップトップ 18 秒訓練の
  RL 方策を STM32F405 上にオンボード展開(直接 RPM 指令)[R10]。
- **SimpleFlight**(Chen et al., RA-L 2025): ゼロショット sim-to-real の設計 5 要因を特定、
  Crazyflie で SOTA 比 50% 超の追従誤差削減。**実験構成は OptiTrack 100Hz+オフボード
  方策+2.4GHz 無線であり、StampFly+OptiTrack+ESP-NOW と同型** [R11]。
- 注意: これらの「オンボード制御」も状態は MoCap 由来であり、完全オンボード推定+制御は
  文献上の空白として残る **[検証済・caveat]**。

### 2.3 状態推定・磁気センサ — あなたの現研究はすでに空白地帯に立っている

- **電流起因の磁気外乱を電流計測で補正する研究の最直接先行は Silic & Mohseni
  (IEEE Sensors Journal 2019)のみ**: 通常サイズ機・総電流比例モデルのオンライン適応
  [R12]。**マイクロ機・モーター毎電流・ベンチ系統較正という構成は確認できず**。
  2023 年以降のフォローは空中磁気探査分野に偏り、電流入力は少数派 [R13]。
- 磁気較正は「オンライン化・推定との同時化」へ: MAGYC(ジャイロのみで磁気+ジャイロ
  バイアス同時推定、2024)[R14]、AMO-HEAD(適応 EKF ヘディング、TIM 2025)[R15]、
  SL(C)AMma(Kok グループ、較正と SLAM の同時推定、2026)[R16]。
- **可観測性駆動の軌道生成が実用段階**: STLOG 最小固有値を最大化する制御(JGCD 2025)
  [R17]、FIM 指標の能動較正軌道(IEEE Sensors J 2025)[R18]、IMU バイアス収束優先の
  情報計画(Auton. Robots 2024)[R19]。**ただし対象は距離計測・外部パラメータ・IMU
  バイアスに偏り、「磁気較正・ヨーバイアスの可観測性を狙う軌道生成」は未開拓**。
- Crazyflie 級では磁気センサはモーター干渉のため事実上使われず、ヨードリフトは放置が
  現状。サブグラム機の最新研究 TinySense(ICRA 2025)もヨーは扱っていない [R20]。

### 2.4 安全制御 — オフボード実証は飽和、オンボード計算と実遅延が最前線

- CBF 安全フィルタの Crazyflie 実証は標準化(トロント大 DSL の予測安全フィルタ MPSF
  [R21][R22]、MIT の GCBF+ スワーム [R23]、gatekeeper(T-RO 2024)[R24])。
- **explicit CBF(QP を閉形式に置換、Ames グループ 2025)は MCU 実験未実施** [R25] —
  「初の MCU 実証」が空いている。
- オフボード安全フィルタにおける**無線遅延の影響の定量化はほぼ皆無**(文献は遅延を暗黙に
  無視)。電流計測で入力制約(実効推力上限)をオンライン適応させる CBF も見当たらない。

### 2.5 ネットワーク化制御(NCS)— 「理論過多・安価実機不在」の明確な空白

- 実無線越し制御の体系的実験は 5G+エッジ MPC(Luleå 工科大、Crazyflie+Vicon)[R26][R27]、
  4G LTE 長距離 [R28]、LoRa 実測 15,000 サンプルの遅延許容限界(Drones 2026)[R29] など
  **高コスト・特殊インフラに集中**。
- **Event-triggered 制御(ETC)×クアッドの 2023-2026 論文はほぼ数値シミュレーションのみ**
  (IJRNC 2025 等で著者自身が明記)[R30]。例外は専用メッシュ+16 機の DMPC-Swarm
  (2025)[R31] という大規模インフラ。
- **ESP-NOW を閉ループ飛行制御リンクとして学術評価した論文は確認できず**。
  per-packet 実測遅延で遅延補償器(Smith predictor / 状態予測器 / NPC [R32])を同一機体で
  横並び比較した研究も不在。

### 2.6 耐故障制御・故障検知 — 電流ベースの「飛行中・オンボード」が空白

- ロータ完全故障飛行は「システム統合問題」へ移行(NYU ARPL の部分損傷→完全故障の連続
  遷移 RA-L 2024 [R33]、学習ベースパッシブ FTC 2025 [R34]、浙江大の未知環境統合 [R35])。
- 検知側の基準は TU Delft のカルマン LOE 検知(検知遅れ 30–130ms、ICUAS 2023)[R36] —
  ただし **RPM フィードバック付き BLDC 機前提**。ブラシモーター機(RPM なし)の系統研究は
  確認できず。
- **電流シグネチャ診断はベンチ試験+オフライン ML 段階**(MST 2024 [R37]、電圧降下補償付き
  FDI は PIL まで [R38])。**飛行中・オンボード・リアルタイムの電流ベース検知は空白**。
- ナノ機の故障データセット CrazyPAD(2024)[R39] が故障注入プロトコルの参考になる。

### 2.7 マルチ機体・近接飛行 — 2機ペアの学習補償から先が空いている

- ダウンウォッシュは「回避」から「モデル化・補償」へ: SO(2) 等変 NN(5 分データ、RA-L
  2024)[R40]、学習 MPC で機体長 1.5 倍未満の密集編隊 [R41]、空中ドッキング(ISER 2023)
  [R42]、残差 RL の ProxFly(ICRA 2025)[R43]、乱流ジェット物理モデル(RA-L 2024)[R44]。
- **実機検証はほぼ 2 機ペア止まり**。3 機以上の重畳流の合成則検証、オンボードセンサ
  (電流・IMU)のみによるダウンウォッシュ検出は確認できず。
- 通信制約付き分散計画は RMADER(平均 49.8ms 遅延メッシュで 6 機、ICRA 2023)[R45]、
  超軽量計画 Primitive-Swarm(T-RO 2025)[R46]。

### 2.8 国内学会動向 — StampFly は「教育」として登場済み、「研究」は空白

- **StampFly は伊藤恒平氏(金沢工業大学、純正ファーム開発者)により第67回自動制御連合
  講演会(2024)[R47]・SCI'26 [R48] で教育プラットフォームとして紹介済み**。勉強会・
  競技会コミュニティも活発。(※当初「伊藤恵理氏」との認識でしたが、正しくは伊藤恒平氏です)
- 研究グレード利用(MoCap 真値付き定量評価)は確認できず。近い例は兵庫県立大・川口氏の
  100g 未満自作実験機(ROBOMECH2025)[R49]、東大系の MoCap+PHM データセット(2025)
  [R50] 程度。
- **J-STAGE 検索で「ドローン×磁気外乱」0 件、「マイコン実装 MPC×ドローン」該当なし、
  sim-to-real RL 実機報告なし、遅延計測付き複数機プラットフォーム報告なし** — あなたの
  実装項目は国内発表空白域に位置する。

---

## 3. 研究テーマ候補(推奨順)

### テーマ A ★最有力: マイクロドローンにおけるモーター電流起因磁気外乱の較正・補正とヨー推定
**— 現在の研究をそのまま学術成果に格上げする**

- **何が新しいか**: 直接先行は Silic & Mohseni 2019(通常サイズ機・総電流比例・オンライン
  適応)のみ [R12]。あなたの方式は (a) モーター毎 duty→電流モデル+差動項(方式B)で
  先行の総電流モデルを既に超えており、(b) 前後ブラケット基準のベンチ系統較正、
  (c) MoCap ヨー真値による定量評価、(d) 同一設計 5 機の較正転移統計、のいずれも文献に
  ない。小型機ほどモーター・磁気センサ間距離が近く外乱が支配的なのに、その領域の定量
  研究が欠落している(補完調査の open problem として明示)。
- **理論的貢献の核**: ホバー中の ψ(ヨー)/b_m(磁気バイアス)1次元縮退の形式的な
  可観測性解析。すでに実装で直面している問題を、リー微分/可観測性グラミアンで解析し
  「どの機動でどれだけ可観測性が回復するか」を定量化する。
- **実機検証プラン**: V2 の Phase 0〜2 飛行データがそのまま実験データになる。
  4 系統ログ(Madgwick/EKF/ジャイロ積算/MoCap)+電流+NIS/ゲートの 94 列ログは
  論文の図がほぼ自動で出る設計。5 機で較正パラメータの個体差分布・転移可否を評価。
- **発表先の目安**: 国内(自動制御連合・SICE SCI/MSCS・ROBOMECH)は確度高。
  国際は IEEE Sensors Journal / IEEE TIM(Silic の掲載誌系列)、ICUAS、状態推定の
  まとめ方次第で IROS。競合注意: AMO-HEAD 系適応 EKF [R15](外乱の検出・減感であり
  電流モデルによる予測補正ではないため、比較対象として共存可能)。
- **リスク**: 低。実装・データ基盤が完成済みで、残るは実験と執筆。

### テーマ B: 磁気較正・ヨーバイアスの可観測性を最大化する励振軌道の自動生成
**— テーマ A の理論を「能動化」する発展形**

- **何が新しいか**: observability-aware 軌道生成は距離計測 [R17]・センサ外部パラメータ
  [R18]・IMU バイアス [R19] で実証済みだが、**磁気較正パラメータ+ヨーの可観測性を狙った
  軌道生成は未開拓**。「較正のための飛行軌道を機械が自分で設計する」を磁気で初めてやる。
- **理論**: FIM/STLOG 最小固有値を目的関数にした軌道最適化+タスク制約(ホバリング任務
  との両立、ヨーディザ振幅の最小化)。
- **実機検証**: ホバー(縮退条件)vs 最適化軌道で b_m 収束速度・推定精度を MoCap 真値
  比較。400Hz 自作ファームで軌道追従と推定器を同一ループに持つため小さな MoCap 領域
  (2m×2m)でも成立する。
- **発表先**: 理論を伴えば ICRA/IROS/RA-L 級を狙える骨格。国内(SICE 系)でも理論+実機で
  強い。**テーマ A の次の一手として最も筋が良い**。
- **リスク**: 中。軌道最適化の理論整備に数ヶ月の学習投資が要る。

### テーマ C: 実測遅延に基づくネットワーク化制御の実機検証(ETC・遅延補償の横並び比較)
**— V2 の遅延計測基盤が最小工数でそのまま研究になる**

- **何が新しいか**: ETC×クアッドの実機検証がほぼ存在しない(sim-to-real ギャップが
  補完調査で裏付け済み)[R30][R31]。ESP-NOW 級の安価リンクでの閉ループ飛行制御の定量
  評価は学術文献に不在。**per-packet 実測遅延を制御に使える構成は先行実験(5G [R26]、
  専用メッシュ [R31])のどれも持っていない**。
- **プラン**: ① ESP-NOW の遅延分布・欠落率・5 機競合の実測特性評価(LoRa 版 [R29] と
  同じ方法論を WiFi PHY で)→ ② 50Hz 位置ループに静的/動的イベントトリガを実装し
  「パケット削減率 vs RMS 追従誤差」の実測トレードオフ曲線 → ③ Smith predictor /
  状態予測器 / バッファ型 NPC [R32] を同一機体・実測遅延で横並び比較。
- **発表先**: ①②だけで国内確度高。③まで通せば ICUAS / IROS / CEP(Control Engineering
  Practice)級。V2 は今日の時点でデータ取得可能なので、**テーマ A と並行できる副戦線**。
- **リスク**: 低〜中。理論障壁が低く、実験系は完成済み。

### テーマ D: ESP32-S3 オンボード組込み最適制御(TinyMPC 移植+explicit CBF の初 MCU 実証)

- **何が新しいか**: TinyMPC 系は STM32F405 一択で、アーキテクチャ横断(Xtensa LX7)の
  実測は存在しない **[検証済・openQuestion]**。explicit CBF [R25] は MCU 実験未実施。
  さらに INA3221 で実効推力上限をオンライン推定し入力制約に反映する「制約適応型 CBF/MPC」
  は文献に見当たらない(電圧垂下は既にスイープ実験で観測済みの現象)。
- **プラン**: TinyMPC(オープンソース)を ESP32-S3 へ移植 → 400Hz ループ統合と求解時間・
  ジッタ実測 → ジオフェンス explicit CBF の 400Hz 実行 → ヨー EKF と組み合わせた
  完全オンボード制御(MoCap なし飛行への布石。§2.2 の caveat の空白を突く)。
- **発表先**: 移植+実測だけなら国内・ICUAS 級。「学習ベース MPC を 240MHz 単体に
  押し込む」[R8] や「完全オンボード推定+MPC」まで行けば RA-L 級。
- **リスク**: 中。ソルバ移植は堅実に進むが、新規性を「初モノ」に仕立てる設計が必要。

### テーマ E: 総電流+磁気センサによる飛行中モーター/プロペラ故障検知

- **何が新しいか**: 電流ベース故障診断はベンチ+オフライン段階 [R37][R38] で、
  **飛行中・オンボード・リアルタイムは空白**。総電流 1ch でも duty→電流モデル残差+
  ミキサ情報で故障ロータを隔離する手法は未確立。さらに**「較正済みの電流→磁気外乱
  モデルを逆に使い、BMM150 を非接触 per-motor 電流センサとして異常を分離する」発想は
  先行研究が確認できず、あなたの磁気較正資産の独創的な再利用になる**。
- **プラン**: CrazyPAD [R39] の故障注入プロトコル(プロペラ片側カット等)を踏襲し、
  MoCap 真値で検知遅延・誤警報率を統計評価。5 機のフリート相互参照異常検知まで拡張。
- **発表先**: 国内確度高、国際は ICUAS / JINT / MST 系。
- **リスク**: 中。BMM150 の SNR で per-motor 寄与を分離できるかが最初の検証課題
  (できなければ総電流 FDI に縮退して成立)。完全ロータ故障のスピン飛行は 36g ブラシ機
  では推力余裕・ジャイロレンジ的にハイリスクなので**主軸にしない**こと。

### テーマ F: マルチ機体近接飛行 — 重畳ダウンウォッシュと「近接飛行が状態推定に与える影響」

- **何が新しいか**: 学習ダウンウォッシュ補償の実機は 2 機ペア止まり [R40-R43]。
  5 機での 3 機以上重畳流の合成則検証は明確なギャップ。さらに
  **「他機ダウンウォッシュ→推力増→電流増→磁気外乱増」という連鎖を定量化できるのは
  電流+磁気較正+MoCap を全部持つこの環境だけ**で、「近接飛行が推定に与える影響」は
  未開拓テーマとして論文性がある(補完調査の open problem)。
- **プラン**: まず Bauersfeld の乱流ジェットスケーリング則 [R44] が 36g+プロペラガード機
  で成立するか検証(それ自体が小さな貢献)→ 2 機の電流ベースダウンウォッシュ検出 →
  5 機重畳。
- **発表先**: 国内〜ICUAS、電流ベース検出が決まれば RA-L 級の可能性。
- **リスク**: 中〜高。多機体運用の実験工数が大きい。テーマ A/C の後段に。

### テーマ G: sim-to-real 強化学習の再現と ESP32 オンボード化

- **位置づけ**: SimpleFlight [R11] の実験構成はあなたの環境と同型なので再現に最適。
  Learning to Fly in Seconds [R10](RLtools は公開)を ESP32-S3+ブラシ機に移植すれば
  プラットフォーム差分(ブラシモーター、36g、ESP-NOW 遅延)の知見で差別化できる。
- **正直な評価**: 再現+移植では国際新規性は薄く、国内発表+技術蓄積(将来の学習ベース
  テーマの足場)として価値がある。単体で主戦線にはしないことを推奨。

### テーマ H(理論・シミュレーション路線): 実機が律速にならない選択肢

実機検証が難しくても成果になり得る理論寄りの枝(いずれも上のテーマの理論部を深掘りする形が効率的):

1. **ヨー/磁気バイアス縮退の可観測性理論**(テーマ A/B の理論部を独立論文化。
   非線形可観測性解析+縮退条件の閉形式化+励振下界)
2. **遅延補償付き安全フィルタの理論**(実測遅延分布を仮定した CBF/MPSF の安全マージン
   設計 — §2.4 の空白。シミュレーションは V2 の実測遅延データで駆動できる)
3. **NPC の遅延バウンド最適化**(Hirche グループの理論 [R32] はシミュレーションのみ —
   実測遅延データでの検証は理論と実験の橋渡しとして狙い目)
4. certified learning-based control / NN 制御器の形式検証は競争が激しく計算機資源も
   要るため、参入障壁が高い(推奨度低)。

---

## 4. 推奨ロードマップ

**短期(〜3ヶ月)— 今ある成果を発表可能な形にする**
1. V2 実機検証(明日〜)で Phase 0 データ収集 → **テーマ A の国内発表**
   (電流磁気較正+EKF ヨー推定+MoCap 定量評価)。J-STAGE「ドローン×磁気外乱」0 件の
   空白に最初の楔を打つ。発表先候補: 自動制御連合講演会(StampFly 開発者・伊藤恒平氏の
   コミュニティとの接続も期待できる)、SICE SCI、ROBOMECH。
2. 並行して**テーマ C の①(ESP-NOW リンク特性の実測評価)**をデータ取りだけ進める
   (V2 に遅延計測が内蔵済みのため追加実装が小さい)。

**中期(3〜12ヶ月)— 国際レベルの柱を立てる**
3. **テーマ A の深化**(per-motor 分離の妥当性、5 機転移統計、縮退の可観測性解析)
   → IEEE Sensors J / TIM / ICUAS / IROS 級の投稿。
4. **テーマ B(可観測性駆動励振軌道)**の理論整備と実験 — テーマ A の理論を能動化する
   自然な次の一手であり、理論+実機の組み合わせで最も「大きな成果」の芽がある。

**長期(12ヶ月〜)— プラットフォームの独自性を横展開**
5. テーマ D(オンボード MPC+完全オンボード化)/ E(電流故障検知)/ F(マルチ機体)から、
   中期の手応えと興味で選択。いずれも「電流計測+磁気較正+MoCap 真値+複数機」という
   資産の再利用で、テーマ間のデータ・コードが相互流用できる。

**横断的な推奨**: 東大系 PHM データセット [R50] の型に倣い、**データセット+コードの公開**
(MoCap ヨー真値付きヘディング推定ベンチマーク)を発表に付けると、国内外での引用・認知が
伸びやすい(MoCap ヨー真値ベンチマークの標準が未確立という空白も突ける)。

## 5. 現プラットフォームでは難しいこと(正直な制約)

- **レース級の高速機動**(Swift 系): MoCap 領域 2m×2m と 36g 機の推力余裕では不可。
- **完全ロータ故障のスピン飛行**: ブラシモーター・RPM センサなし・ジャイロレンジ制約で
  ハイリスク(部分故障 LOE 検知に主軸を置くべき)。
- **ビジョンベース自律**(カメラなし)、**屋外・長距離**(ESP-NOW/機体サイズ)。
- 教育プラットフォームとしての StampFly は既に伊藤氏が確立しており、「教育」を主張点に
  すると新規性が立たない — **「研究グレードの定量評価基盤」への格上げ**が正しい立ち位置。

---

## 6. 参考文献

### 組込み最適制御 [検証済]
- [R1] Nguyen, Schoedel, Alavilli, Plancher, Manchester, "TinyMPC: Model-Predictive Control on Resource-Constrained Microcontrollers," ICRA 2024 (Best Paper Award in Automation). https://arxiv.org/abs/2310.16985
- [R2] TinyMPC プロジェクトサイト. https://tinympc.org/ / https://github.com/TinyMPC/TinyMPC
- [R3] TinyMPC Crazyflie ファームウェア. https://github.com/RoboticExplorationLab/tinympc-crazyflie-firmware
- [R4] Bitcraze blog, "Bringing Model Predictive Control to the Crazyflie with TinyMPC" (2024). https://www.bitcraze.io/2024/07/bringing-model-predictive-control-to-the-crazyflie-with-tinympc/
- [R5] Mahajan, Nguyen, Schoedel et al., "Code Generation and Conic Constraints for Model-Predictive Control on Microcontrollers with Conic-TinyMPC," ICRA 2026. https://arxiv.org/abs/2403.18149
- [R6] Adaptive TinyMPC (first-order adaptive caching), IROS 2025. https://arxiv.org/abs/2507.03231
- [R7] Wikner, Arnström, Axehill, "DAQP on nano quadcopter low-level MPC"(2026 プレプリント・査読未確認). https://arxiv.org/html/2603.09342
- [R8] Akbari, Frank, Greeff, "Tiny Learning-Based MPC," Mechatronics (Elsevier) 2025. https://arxiv.org/abs/2410.23634 / https://robora-lab.github.io/Tiny-Learning-Based-MPC/

### 学習ベース制御 [検証済]
- [R9] Kaufmann et al., "Champion-level drone racing using deep reinforcement learning," Nature 620 (2023). https://www.nature.com/articles/s41586-023-06419-4
- [R10] Eschmann, Albani, Loianno, "Learning to Fly in Seconds," RA-L 2024. https://arxiv.org/abs/2311.13081 / https://github.com/arplaboratory/learning-to-fly
- [R11] Chen et al., "What Matters in Learning A Zero-Shot Sim-to-Real RL Policy for Quadrotor Control? (SimpleFlight)," RA-L 2025. https://arxiv.org/abs/2412.11764 / https://github.com/thu-uav/SimpleFlight

### 状態推定・磁気 [確認済]
- [R12] Silic, Mohseni, "Correcting Current-Induced Magnetometer Errors on UAVs: An Online Model-Based Approach," IEEE Sensors Journal 2019. https://enstrophy.mae.ufl.edu/publications/MyPapers/2019_Silic_IEEEJSens.pdf
- [R13] Chen et al., "Intelligent Dynamic-Enhanced Compensation for UAV Magnetic Interference," Sensors 25(16) 2025. https://pmc.ncbi.nlm.nih.gov/articles/PMC12390355/
- [R14] Rodríguez-Martínez, Troni, "MAGYC: Full Magnetometer and Gyroscope Bias Estimation using Angular Rates," 2024. https://arxiv.org/abs/2412.09690
- [R15] Guo et al., "AMO-HEAD: Adaptive MARG-Only Heading Estimation for UAVs under Magnetic Disturbances," IEEE TIM 2025. https://arxiv.org/abs/2510.10979
- [R16] Edridge, Kok, "SL(C)AMma: Simultaneous Localisation, (Calibration) and Mapping With a Magnetometer Array," 2026. https://arxiv.org/abs/2604.19946
- [R17] Go, Chong, Qian, Liu, "Observability-Aware Control for Quadrotor Formation Flight with Range-only Measurement," J. Guidance, Control, and Dynamics 2025. https://arxiv.org/abs/2411.03747
- [R18] Wang et al., "Observability-Aware Active Calibration of Multi-Sensor Extrinsics via Online Trajectory Optimization," IEEE Sensors Journal 2025. https://arxiv.org/abs/2506.13420
- [R19] Usayiwevu et al., "Continuous planning for inertial-aided systems," Autonomous Robots 48 (2024). https://link.springer.com/article/10.1007/s10514-024-10180-6
- [R20] Yu, ..., Fuller, "TinySense: A Lighter Weight and More Power-efficient Avionics System for Flying Insect-scale Robots," ICRA 2025. https://arxiv.org/abs/2501.03416

### 安全制御 [確認済]
- [R21] Pizarro Bejarano, Brunke, Schoellig, "Multi-Step Model Predictive Safety Filters," CDC 2023. https://arxiv.org/abs/2309.11453
- [R22] Pizarro Bejarano et al., "Safety Filtering While Training," RA-L 2025. https://arxiv.org/abs/2410.11671
- [R23] Zhang, So, Garg, Fan, "GCBF+: A Neural Graph Control Barrier Function Framework," T-RO 2025. https://arxiv.org/abs/2401.14554
- [R24] Agrawal, Chen, Panagou, "gatekeeper: Online Safety Verification and Control," T-RO 2024. https://arxiv.org/abs/2211.14361
- [R25] Mestres, ..., Ames et al., "Explicit Control Barrier Function-based Safety Filters and their Resource-Aware Computation," 2025(MCU 実験なし). https://arxiv.org/abs/2512.10118

### ネットワーク化制御 [確認済]
- [R26] Damigos et al., "A Resilient Framework for 5G-Edge-Connected UAVs based on Switching Edge-MPC and Onboard-PID Control," IEEE ISIE 2023. https://arxiv.org/abs/2310.15849
- [R27] Sankaranarayanan et al., "PACED-5G: Predictive Autonomous Control using Edge for Drones over 5G," 2023. https://arxiv.org/abs/2301.13097
- [R28] Mohamed, Oniz, "Real-Time Long-Range Control of an Autonomous UAV Using 4G LTE Network," Drones 9(12) 2025. https://www.mdpi.com/2504-446X/9/12/812
- [R29] Vera-Amaro et al., "Measurement-Informed Latency Limits for Real-Time UAV Swarm Coordination," Drones 10(4) 2026. https://www.mdpi.com/2504-446X/10/4/310
- [R30] Zhao et al., "Event-Triggered Trajectory Tracking Control for Quadrotor UAVs...," IJRNC 2025(シミュレーションのみ). https://onlinelibrary.wiley.com/doi/10.1002/rnc.7933
- [R31] Gräfe, Eickhoff, Zimmerling, Trimpe, "DMPC-Swarm: Distributed Model Predictive Control on Nano UAV Swarms," Autonomous Robots 2025. https://arxiv.org/abs/2508.20553
- [R32] Beger, Lin, Stanojevic, Hirche, "Optimal Delay Compensation in Networked Predictive Control," 2025. https://arxiv.org/abs/2512.11492

### 耐故障制御・故障検知 [確認済]
- [R33] Mao, Yeom, Nair, Loianno, "From Propeller Damage Estimation and Adaptation to Fault Tolerant Control," RA-L 2024. https://arxiv.org/abs/2310.13091
- [R34] Chen et al., "Learning-Based Passive Fault-Tolerant Control of a Quadrotor with Rotor Failure," 2025. https://arxiv.org/abs/2503.02649
- [R35] Zhou et al.(浙江大 Fei Gao 研), "Rotor-Failure-Aware Quadrotors Flight in Unknown Environments," 2025. https://arxiv.org/abs/2510.11306
- [R36] Strack van Schijndel, Sun, de Visser, "Fast Loss of Effectiveness Detection on a Quadrotor," ICUAS 2023. https://research.tudelft.nl/en/publications/fast-loss-of-effectiveness-detection-on-a-quadrotor-using-onboard-2
- [R37] Chen, Li et al., "Fault diagnosis of drone motors driven by current signal data with few samples," Meas. Sci. Technol. 2024. https://iopscience.iop.org/article/10.1088/1361-6501/ad3d00
- [R38] Baldini et al., "Propeller Fault Detection and Isolation for Multirotor Drones with Adaptation to Battery Voltage Drop," JINT 112 (2026). https://link.springer.com/article/10.1007/s10846-026-02369-x
- [R39] Masalimov et al., "CrazyPAD: A Dataset for Assessing the Impact of Structural Defects on Nano-Quadcopter Performance," Data 9(6) 2024. https://github.com/AerialRoboticsUUST/CrazyPAD

### マルチ機体・近接飛行 [確認済]
- [R40] Smith, Shankar, Gielis, Blumenkamp, Prorok, "SO(2)-Equivariant Downwash Models for Close Proximity Flight," RA-L 2024. https://arxiv.org/abs/2305.18983
- [R41] Chee, Hsieh, Pappas, Hsieh, "Flying Quadrotors in Tight Formations using Learning-based Model Predictive Control," 2024. https://arxiv.org/abs/2410.09727
- [R42] Shankar, Woo, Prorok, "Docking Multirotors in Close Proximity using Learnt Downwash Models," ISER 2023. https://arxiv.org/abs/2311.13988
- [R43] Zhang, Zhang, Mueller, "ProxFly: Robust Control for Close Proximity Quadcopter Flight via Residual Reinforcement Learning," ICRA 2025. https://arxiv.org/abs/2409.13193
- [R44] Bauersfeld, Muller, Ziegler, Coletti, Scaramuzza, "Robotics meets Fluid Dynamics: ... Induced Airflow below a Quadrotor as a Turbulent Jet," RA-L 2024. https://arxiv.org/abs/2403.13321
- [R45] Kondo et al., "Robust MADER: Decentralized Multiagent Trajectory Planner Robust to Communication Delay," ICRA 2023. https://arxiv.org/abs/2303.06222
- [R46] Hou et al., "Primitive-Swarm: An Ultra-lightweight and Scalable Planner for Large-scale Aerial Swarms," T-RO 2025. https://arxiv.org/abs/2502.16887

### 国内 [確認済(J-STAGE 書誌)]
- [R47] 伊藤恒平・LAI JINGMING・高須正和ほか,「プログラマブルドローンStampFlyについて」第67回自動制御連合講演会 (2024). https://www.jstage.jst.go.jp/article/jacc/67/0/67_729/_article/-char/ja
- [R48] 伊藤恒平,「Stampfly Ecosystemによるドローン教育について」SCI'26 (2026). https://www.jstage.jst.go.jp/article/sci/SCI26/0/SCI26_227/_article/-char/ja
- [R49] 川口夏樹,「制御実験に向けた小型ドローン装置開発」ROBOMECH2025. https://www.jstage.jst.go.jp/article/jsmermd/2025/0/2025_2A2-K11/_article/-char/ja
- [R50] 武石・矢入ら,「PHM研究のためのマルチモーダル屋内UAVデータセットの構築と公開」人工知能学会 第二種研究会資料 (2025). https://www.jstage.jst.go.jp/article/jsaisigtwo/2025/SMSHM-004/2025_01/_article/-char/ja
