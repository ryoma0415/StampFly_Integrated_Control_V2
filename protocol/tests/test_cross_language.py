"""クロス言語テスト: C++ 実装が同一ベクタを独立に再導出することを検証する。

tests/host_test.cpp を g++ -std=c++17 -Wall -Wextra -Werror でコンパイルし、
test_vectors.json のパスを渡して実行する。host_test は全ベクタを C++ の
エンコーダで再導出してバイト単位比較し、破損系のレシーバ挙動も検証する。
"""

from __future__ import annotations

import re
import shutil
import subprocess

import pytest

from conftest import PROTOCOL_DIR, TESTS_DIR, VECTORS_PATH

GXX_FLAGS = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-O2"]


@pytest.fixture(scope="module")
def host_test_binary(tmp_path_factory):
    """host_test.cpp を厳格警告でコンパイルする(警告=エラー)。"""
    gxx = shutil.which("g++")
    assert gxx is not None, "g++ not found on PATH"
    exe = tmp_path_factory.mktemp("host_test") / "host_test"
    cmd = [gxx, *GXX_FLAGS, f"-I{PROTOCOL_DIR}",
           str(TESTS_DIR / "host_test.cpp"), "-o", str(exe)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"compile failed:\n{' '.join(cmd)}\n{result.stdout}{result.stderr}")
    return exe


def test_host_cpp_reproduces_all_vectors(host_test_binary, vectors):
    result = subprocess.run([str(host_test_binary), str(VECTORS_PATH)],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, (
        f"host_test failed:\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    assert "ALL OK" in result.stdout
    match = re.search(r"CHECKS: (\d+) passed, (\d+) failed", result.stdout)
    assert match is not None, result.stdout
    passed, failed = int(match.group(1)), int(match.group(2))
    assert failed == 0
    # ベクタ数に応じた最低チェック数(フレーム10件 x 9チェック + α)
    expected_min = 2 + len(vectors["frames"]) * 9 + len(vectors["corruption"]) * 5
    assert passed >= expected_min, (passed, expected_min)


def test_host_cpp_exits_nonzero_on_corrupted_vectors(host_test_binary, tmp_path,
                                                     vectors):
    """ベクタ改竄を C++ 側が検出できること(テストの実効性確認)。"""
    import json
    tampered = json.loads(VECTORS_PATH.read_text())
    frame = tampered["frames"][0]
    logical = bytearray(bytes.fromhex(frame["logical_hex"]))
    logical[0] ^= 0xFF
    frame["logical_hex"] = bytes(logical).hex()
    bad_path = tmp_path / "tampered_vectors.json"
    bad_path.write_text(json.dumps(tampered))
    result = subprocess.run([str(host_test_binary), str(bad_path)],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode != 0
    assert "ALL OK" not in result.stdout
