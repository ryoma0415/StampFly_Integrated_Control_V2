"""pc_server.core — ハードウェア非依存の制御ロジック層。

このパッケージは Web フレームワーク(fastapi/uvicorn)を一切 import しない。
app.py だけが Web 層に依存する、というレイヤ分離を保つこと。

ここは sys.path シムの「単一の共有場所」でもある:
- リポジトリの ``protocol/`` を追加して ``stampfly_protocol`` を import 可能にする
- ``pc_server/vendor/`` を追加して NatNet SDK(NatNetClient / MoCapData /
  DataDescriptions、互いに素の module 名で import し合う)を無改変のまま使う
"""

from __future__ import annotations

import sys
from pathlib import Path

PC_SERVER_DIR = Path(__file__).resolve().parent.parent
REPO_DIR = PC_SERVER_DIR.parent
PROTOCOL_DIR = REPO_DIR / "protocol"
VENDOR_DIR = PC_SERVER_DIR / "vendor"

for _path in (PROTOCOL_DIR, VENDOR_DIR):
    _entry = str(_path)
    if _entry not in sys.path:
        sys.path.insert(0, _entry)
