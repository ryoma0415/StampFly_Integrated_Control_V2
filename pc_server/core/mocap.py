"""NatNet(OptiTrack)接続と座標変換。

- vendor/ の NatNet SDK(NatNetClient.py ほか)を無改変のままラップする。
  出自は NatNet SDK 4.3.0 付属の PythonClient(リポジトリ同梱 ../NatNetSDK/ の
  DLLバージョンリソースで確認)。クライアントは旧ビットストリームと後方互換のため
  Motive 本体のバージョンはこれだけでは確定しない(README §4.3)。
- 座標変換は Motive(Y-up)→ 制御座標系。マッピングは control.json の
  "coordinate_transform" で定義する(legacy hovering_controller と同方式)。
  既定: 制御x ← Motive z、制御y ← -Motive x(y軸符号は旧実装の負ゲイン規約を
  座標変換側へ移したもの)、制御z ← Motive y(高度)。
- 位置はリジッドボディのみを使用する。全マーカー重心フォールバックは
  流用禁止(ARCHITECTURE.md)のため実装しない。
- yaw はクォータニオンから算出するが UI 表示専用(制御には使わない)。

NatNet 受信コールバック(SDKのスレッド)上では print やブロッキングを行わない。
"""

from __future__ import annotations

import threading
import time
from math import asin, atan2, copysign, pi
from typing import Callable, Optional

RAD_TO_DEG = 180.0 / pi
DEG_TO_RAD = pi / 180.0

DEFAULT_COORDINATE_TRANSFORM = {
    "x": {"axis": "z", "sign": 1},
    "y": {"axis": "x", "sign": -1},
    "z": {"axis": "y", "sign": 1},
}

# リジッドボディ品質(0-1)算出時、tracking_valid でない場合の減衰率
_QUALITY_INVALID_SCALE = 0.5
_QUALITY_FLOOR = 0.05


def quaternion_to_euler_xyz(qx: float, qy: float, qz: float, qw: float
                            ) -> tuple[float, float, float]:
    """クォータニオン (x, y, z, w) → (roll, pitch, yaw) [rad]。"""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = copysign(pi / 2.0, sinp)
    else:
        pitch = asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


class CoordinateTransformer:
    """Motive 座標 (X, Y, Z) を制御座標系へ変換する。"""

    AXIS_INDEX = {"x": 0, "y": 1, "z": 2}

    def __init__(self, transform_config: Optional[dict] = None) -> None:
        config = transform_config or DEFAULT_COORDINATE_TRANSFORM
        self.axis_map: dict[str, tuple[int, int]] = {}
        for axis in ("x", "y", "z"):
            axis_cfg = config.get(axis, DEFAULT_COORDINATE_TRANSFORM[axis])
            src_axis = axis_cfg["axis"]
            if src_axis not in self.AXIS_INDEX:
                raise ValueError(f"coordinate_transform: invalid axis {src_axis!r}")
            sign = -1 if float(axis_cfg["sign"]) < 0 else 1
            self.axis_map[axis] = (self.AXIS_INDEX[src_axis], sign)

    def motive_to_control(self, position) -> Optional[tuple[float, float, float]]:
        """Motive 座標タプル → 制御座標タプル。"""
        if position is None:
            return None
        src = tuple(position)
        x_idx, x_sign = self.axis_map["x"]
        y_idx, y_sign = self.axis_map["y"]
        z_idx, z_sign = self.axis_map["z"]
        return (src[x_idx] * x_sign, src[y_idx] * y_sign, src[z_idx] * z_sign)


class MocapSource:
    """NatNet クライアントのライフサイクルとリジッドボディ姿勢の抽出。

    on_pose コールバックには制御座標系へ変換済みの pose dict を渡す:
    ``{t_mono, frame_number, x, y, z, yaw_rad, tracking_valid, error,
       marker_count, quality}``
    対象リジッドボディがフレームに存在しない場合はコールバックを呼ばない
    (消費側は pose の経過時間で途絶を検出する)。
    """

    def __init__(self, natnet_config: dict, transform_config: dict,
                 client_factory: Optional[Callable[[], object]] = None) -> None:
        self._server_address: str = natnet_config["server_address"]
        self._client_address: str = natnet_config["client_address"]
        self._use_multicast: bool = natnet_config["use_multicast"]
        self._rigid_body_id: int = natnet_config["rigid_body_id"]
        self._transformer = CoordinateTransformer(transform_config)
        self._client_factory = client_factory or self._default_client_factory

        self._client: Optional[object] = None
        self._on_pose: Optional[Callable[[dict], None]] = None

        self._lock = threading.Lock()
        self._latest_pose: Optional[dict] = None
        self._frames_total = 0
        self._frames_without_rigid_body = 0

    @staticmethod
    def _default_client_factory():
        # vendor/ は core/__init__.py のシムで sys.path に追加済み
        from NatNetClient import NatNetClient  # type: ignore
        return NatNetClient()

    # ------------------------------------------------------------------
    # ライフサイクル
    # ------------------------------------------------------------------

    def start(self, on_pose: Callable[[dict], None]) -> bool:
        """NatNet クライアントを起動する。Returns: 起動成功フラグ。"""
        if self._client is not None:
            return True
        self._on_pose = on_pose
        client = self._client_factory()
        client.set_server_address(self._server_address)
        client.set_client_address(self._client_address)
        client.set_use_multicast(self._use_multicast)
        client.new_frame_with_data_listener = self._receive_frame
        client.set_print_level(0)
        self._client = client
        ok = bool(client.run("d"))
        if not ok:
            self.shutdown()
        return ok

    def connected(self) -> bool:
        client = self._client
        return bool(client and client.connected())

    def shutdown(self) -> None:
        client = self._client
        self._client = None
        self._on_pose = None
        if client is not None:
            try:
                client.shutdown()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # フレーム処理(NatNet スレッド上で実行: ブロッキング・print 禁止)
    # ------------------------------------------------------------------

    def _receive_frame(self, data_dict: dict) -> None:
        mocap_data = data_dict.get("mocap_data")
        if mocap_data is None:
            return
        with self._lock:
            self._frames_total += 1

        pose = self._extract_rigid_body_pose(
            mocap_data, data_dict.get("frame_number", 0))
        if pose is None:
            # リジッドボディ非検出。重心フォールバックは行わない(流用禁止)。
            with self._lock:
                self._frames_without_rigid_body += 1
            return

        with self._lock:
            self._latest_pose = pose
        callback = self._on_pose
        if callback is not None:
            callback(pose)

    def _extract_rigid_body_pose(self, mocap_data, frame_number: int
                                 ) -> Optional[dict]:
        """指定 ID のリジッドボディを制御座標系の pose dict に変換する。"""
        rigid_body_data = getattr(mocap_data, "rigid_body_data", None)
        if rigid_body_data is None:
            return None
        rigid_bodies = getattr(rigid_body_data, "rigid_body_list", None)
        if not rigid_bodies:
            return None

        for rigid_body in rigid_bodies:
            if getattr(rigid_body, "id_num", None) != self._rigid_body_id:
                continue

            position = self._transformer.motive_to_control(tuple(rigid_body.pos))
            rotation = tuple(rigid_body.rot)
            _, _, yaw_rad = quaternion_to_euler_xyz(*rotation)
            tracking_valid = bool(getattr(rigid_body, "tracking_valid", False))
            error = getattr(rigid_body, "error", None)
            marker_count = len(getattr(rigid_body, "rb_marker_list", []) or [])

            quality = 1.0
            if error is not None:
                quality = 1.0 / (1.0 + max(0.0, error))
            if not tracking_valid:
                quality *= _QUALITY_INVALID_SCALE
            quality = max(_QUALITY_FLOOR, min(1.0, quality))

            return {
                "t_mono": time.monotonic(),
                "frame_number": frame_number,
                "x": position[0],
                "y": position[1],
                "z": position[2],
                "yaw_rad": yaw_rad,
                "tracking_valid": tracking_valid,
                "error": error,
                "marker_count": marker_count,
                "quality": quality,
            }
        return None

    # ------------------------------------------------------------------
    # スナップショット
    # ------------------------------------------------------------------

    def latest_pose(self) -> Optional[dict]:
        """最後に検出した pose のコピーを返す(未検出なら None)。"""
        with self._lock:
            return dict(self._latest_pose) if self._latest_pose else None

    def stats(self) -> dict:
        with self._lock:
            return {
                "frames_total": self._frames_total,
                "frames_without_rigid_body": self._frames_without_rigid_body,
            }
