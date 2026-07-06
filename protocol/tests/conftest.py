"""pytest 共通設定: protocol/ ディレクトリを import パスに追加する。"""

import json
import sys
from pathlib import Path

import pytest

PROTOCOL_DIR = Path(__file__).resolve().parents[1]
TESTS_DIR = Path(__file__).resolve().parent
VECTORS_PATH = PROTOCOL_DIR / "test_vectors.json"

sys.path.insert(0, str(PROTOCOL_DIR))


@pytest.fixture(scope="session")
def vectors() -> dict:
    return json.loads(VECTORS_PATH.read_text())
