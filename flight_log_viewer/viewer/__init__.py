"""StampFly V2 フライトログビューア(50Hz・94列 CSV 用)。

モジュール構成:
- constants: 列定義・ビット定義・スタイル定数
- loader: CSV 読み込みと派生量計算(FlightLog)
- plots: 静止画グラフ一式
- yaw_analysis: ヨー4系統比較・EKF 診断(本プロジェクトの主目的)
- animation: 同期アニメーション MP4(スマホ動画合成はオプション)
- report: サマリレポート / 2 ログ比較
"""
