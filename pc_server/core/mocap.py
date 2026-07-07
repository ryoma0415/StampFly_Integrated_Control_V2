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

# 全ボディインベントリ(紐付け確認用)の保持上限(構造定数。Motive が流す
# リジッドボディ数は高々数個だが、ID の付け直しで無限成長しないための保険)
_INVENTORY_MAX = 64


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
    ``{rigid_body_id, t_mono, frame_number, x, y, z, yaw_rad, tracking_valid,
       error, marker_count, quality}``
    対象リジッドボディがフレームに存在しない場合はコールバックを呼ばない
    (消費側は pose の経過時間で途絶を検出する)。

    マルチ機体: NatNet は毎フレーム全リジッドボディを配信するため、
    subscribe(id, callback) で ID 別のコールバックを追加登録できる
    (1つの NatNet クライアントで N 機ぶんを賄う)。また全ボディの最新
    pose をインベントリとして保持し、bodies_snapshot() で取り出せる
    (UI の「リジッドボディ紐付け確認」用)。
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
        # start/shutdown の check-then-act を直列化する(パッシブ起動と
        # Position モード起動が別スレッドから同時に来てもクライアントを
        # 二重生成しない)。start() が失敗時に shutdown() を呼ぶため RLock。
        self._lifecycle_lock = threading.RLock()

        self._lock = threading.Lock()
        self._latest_pose: Optional[dict] = None
        self._frames_total = 0
        self._frames_without_rigid_body = 0
        # マルチ機体: ID別コールバック(_lock 保護。NatNetスレッドから配送)
        self._subscriptions: dict[int, Callable[[dict], None]] = {}
        # 全リジッドボディの最新 pose(紐付け確認用インベントリ、_lock 保護)
        self._bodies: dict[int, dict] = {}

    @staticmethod
    def _default_client_factory():
        # vendor/ は core/__init__.py のシムで sys.path に追加済み
        from NatNetClient import NatNetClient  # type: ignore
        return NatNetClient()

    # ------------------------------------------------------------------
    # ライフサイクル
    # ------------------------------------------------------------------

    def start(self, on_pose: Optional[Callable[[dict], None]] = None) -> bool:
        """NatNet クライアントを起動する。Returns: 起動成功フラグ。

        on_pose は primary(natnet.rigid_body_id)コールバック。None なら
        パッシブ起動(インベントリ/subscribe のみ使用 — 複数機モードや
        紐付け確認)。既に起動済みの場合、on_pose が指定されていれば
        primary コールバックを差し替える(パッシブ起動が先行しても
        Position モードのコールバック登録が阻害されないように)。
        """
        with self._lifecycle_lock:
            if self._client is not None:
                if on_pose is not None:
                    self._on_pose = on_pose
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
        with self._lifecycle_lock:
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
        frame_number = data_dict.get("frame_number", 0)

        # フレーム内の全リジッドボディを一度の走査で pose 化する
        poses: dict[int, dict] = {}
        rigid_body_data = getattr(mocap_data, "rigid_body_data", None)
        rigid_bodies = (getattr(rigid_body_data, "rigid_body_list", None)
                        if rigid_body_data is not None else None)
        for rigid_body in rigid_bodies or []:
            rb_id = getattr(rigid_body, "id_num", None)
            if rb_id is None:
                continue
            pose = self._pose_from_rigid_body(rigid_body, int(rb_id),
                                              frame_number)
            if pose is not None:
                poses[int(rb_id)] = pose

        primary = poses.get(self._rigid_body_id)
        with self._lock:
            self._frames_total += 1
            if primary is None:
                # 対象リジッドボディ非検出。重心フォールバックは行わない(流用禁止)。
                self._frames_without_rigid_body += 1
            else:
                self._latest_pose = primary
            # インベントリ更新(上限超過は最も古いIDから捨てる)
            self._bodies.update(poses)
            if len(self._bodies) > _INVENTORY_MAX:
                excess = len(self._bodies) - _INVENTORY_MAX
                for stale_id in sorted(
                        self._bodies,
                        key=lambda i: self._bodies[i]["t_mono"])[:excess]:
                    del self._bodies[stale_id]
            # 配送対象をロック内で確定し、コールバック呼び出しはロック外で行う
            deliveries = [(cb, poses[rb_id])
                          for rb_id, cb in self._subscriptions.items()
                          if rb_id in poses]

        callback = self._on_pose
        if primary is not None and callback is not None:
            callback(primary)
        for cb, pose in deliveries:
            cb(pose)

    def _pose_from_rigid_body(self, rigid_body, rb_id: int,
                              frame_number: int) -> Optional[dict]:
        """リジッドボディ1体を制御座標系の pose dict に変換する。"""
        position = self._transformer.motive_to_control(tuple(rigid_body.pos))
        if position is None:
            return None
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
            "rigid_body_id": rb_id,
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

    # ------------------------------------------------------------------
    # マルチ機体: ID別サブスクリプションとインベントリ
    # ------------------------------------------------------------------

    def subscribe(self, rigid_body_id: int,
                  on_pose: Callable[[dict], None]) -> None:
        """ID 別 pose コールバックを登録する(NatNet スレッド上で呼ばれる。
        ブロッキング・print 禁止)。同一 ID への再登録は置き換え。"""
        with self._lock:
            self._subscriptions[int(rigid_body_id)] = on_pose

    def unsubscribe(self, rigid_body_id: int) -> None:
        with self._lock:
            self._subscriptions.pop(int(rigid_body_id), None)

    def clear_subscriptions(self) -> None:
        with self._lock:
            self._subscriptions.clear()

    def bodies_snapshot(self) -> list[dict]:
        """観測済み全リジッドボディの最新 pose 一覧(紐付け確認 UI 用)。

        各要素は pose dict + ``age_s``(最終観測からの経過秒)。ID 昇順。
        """
        now = time.monotonic()
        with self._lock:
            bodies = [dict(pose) for pose in self._bodies.values()]
        for body in bodies:
            body["age_s"] = now - body["t_mono"]
        return sorted(bodies, key=lambda b: b["rigid_body_id"])
