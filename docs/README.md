# StampFly Integrated Control — ドキュメント索引

セットアップ+運用手順はリポジトリ直下の [../README.md](../README.md) に
一本化した(本フォルダと二重管理しない)。

| 文書 | 内容 |
|---|---|
| [../README.md](../README.md) | システム概要、必要機材、セットアップ(venv / pio / Motive)、pc_server 起動と UI の使い方、トラブルシューティング |
| [OPERATION_GUIDE.md](OPERATION_GUIDE.md) | 段階的な安全飛行手順(機体電源 OFF での通信確認 → 短時間ホバー → 本飛行)、緊急時エスカレーション、フェイルセーフ一覧、機体プロファイル較正 |
| [PROTOCOL.md](PROTOCOL.md) | 通信ワイヤ仕様の正典(COBS+CRC16 フレーム、メッセージ型、フェイルセーフ規範、レート) |
| [ARCHITECTURE.md](ARCHITECTURE.md) | モジュール構成・API 契約・コーディング規約の正典 |
| [LOG_STRUCTURE.md](LOG_STRUCTURE.md) | CSV フライトログの列定義(`pc_server/core/logger.py` の `COLUMNS` と1対1) |
