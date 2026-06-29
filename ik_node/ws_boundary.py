#!/usr/bin/env python3
"""
ws_boundary.py
==============
方案 A — SoftClamp   軟邊界：彈性牆取代硬 snap
方案 E — BoundaryMonitor  邊界距離反饋 ROS topic

可獨立 import，零 placo 依賴。

用法（在 placo_ik_node.py 內）：
    from ws_boundary import SoftClamp, BoundaryMonitor

    # __init__
    self._soft_clamp = SoftClamp(margin_m=0.05, box_ws=box_ws)
    self._bdry_mon   = BoundaryMonitor(node, arm)

    # 熱迴圈裡取代舊的硬 snap clamp 呼叫
    new_xyz, bs = self._soft_clamp.apply(
        raw_xyz    = np.array([raw_x, raw_y, raw_z]),
        dx_arm_raw = np.array([dx_arm[0], dx_arm[1], dx_arm[2]]),
    )
    self._bdry_mon.publish(bs)

BoundaryState 欄位（也發佈到 ROS topic）：
    d_to_boundary_mm : float   — 距最近邊界 voxel 的距離（負數 = 已超出）
    outside          : bool    — 是否超出可達空間
    since_outside_ms : float   — 連續超出多少 ms (0 = 在內部)
    normal           : list    — 邊界法向量 (指向內部，長度 1)
    tangential_gain  : float   — 本步切向分量的通過率 (0~1)
    normal_gain      : float   — 本步法向分量的衰減率  (0~1)
"""

import math
import time
import json
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
#  BoundaryState  —  傳遞邊界資訊的純資料類別（方案 A + E 共用）
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class BoundaryState:
    d_to_boundary_mm:  float = 9999.0    # 距最近邊界 mm；負 = 超出
    outside:           bool  = False      # True → outside workspace
    since_outside_ms:  float = 0.0        # 連續超出的累計時間 ms
    normal:            list  = field(default_factory=lambda: [0., 0., 0.])
    tangential_gain:   float = 1.0        # 切向分量通過率
    normal_gain:       float = 1.0        # 法向分量通過率（<=1）
    raw_xyz:           list  = field(default_factory=lambda: [0., 0., 0.])
    clamped_xyz:       list  = field(default_factory=lambda: [0., 0., 0.])

    def to_json(self) -> str:
        return json.dumps({
            "d_to_boundary_mm":  round(self.d_to_boundary_mm, 1),
            "outside":           self.outside,
            "since_outside_ms":  round(self.since_outside_ms, 1),
            "normal":            [round(v, 3) for v in self.normal],
            "tangential_gain":   round(self.tangential_gain, 3),
            "normal_gain":       round(self.normal_gain, 3),
        })


# ─────────────────────────────────────────────────────────────────────────────
#  SoftClamp  —  方案 A：彈性邊界（rectangular box），取代硬 snap
# ─────────────────────────────────────────────────────────────────────────────
class SoftClamp:
    """
    三個區域的處理方式：

    inside (d > margin)      passthrough：完全不衰減
    near   (0 < d < margin)  tangential passthrough + 法向二次衰減
                              g(d) = (d / margin)²  ∈ [0, 1]
                              → 手靠近邊界時「越來越難推進去」
    outside (d < 0)          tangential passthrough（可沿邊滑動）
                              法向分量（往外方向）歸零
                              target 投影到最近 voxel + 保留切向偏移

    Parameters
    ----------
    margin_m   : 開始衰減的距離（公尺），建議 = 1~2 個 voxel step
    box_ws     : rectangular box clamp dict {"x":(lo,hi), ...}
    """

    def __init__(
        self,
        margin_m: float = 0.05,
        box_ws: Optional[dict] = None,
    ):
        self._margin  = float(margin_m)
        self._box     = box_ws
        self._t_outside: Optional[float] = None   # timestamp when went outside

    # ── Public API ────────────────────────────────────────────────────────────
    def apply(
        self,
        raw_xyz:    np.ndarray,     # 完整 target = base_xyz + dx_arm
        dx_arm_raw: np.ndarray,     # 這一幀的 dx_arm 向量（未衰減）
    ) -> Tuple[np.ndarray, BoundaryState]:
        """
        Returns
        -------
        new_xyz     : np.ndarray  shape (3,)  — 衰減後的 target XYZ
        state       : BoundaryState
        """
        if self._box is not None:
            return self._apply_box(raw_xyz, dx_arm_raw)
        else:
            bs = BoundaryState(raw_xyz=raw_xyz.tolist(), clamped_xyz=raw_xyz.tolist())
            return raw_xyz.copy(), bs

    # ── Box-based soft clamp ──────────────────────────────────────────────────
    def _apply_box(
        self,
        raw_xyz: np.ndarray,
        dx_arm_raw: np.ndarray,
    ) -> Tuple[np.ndarray, BoundaryState]:
        """Soft version of the rectangular box clamp."""
        ws = self._box
        lo = np.array([ws["x"][0], ws["y"][0], ws["z"][0]])
        hi = np.array([ws["x"][1], ws["y"][1], ws["z"][1]])

        # Signed distance to nearest box face (positive = inside)
        d_lo = raw_xyz - lo
        d_hi = hi - raw_xyz
        signed_dists = np.minimum(d_lo, d_hi)   # per-axis: positive = inside
        signed_dist_m = float(np.min(signed_dists))   # most constrained axis
        d_mm = signed_dist_m * 1000.0
        inside = bool(np.all(signed_dists >= 0))

        now = time.monotonic()
        if not inside:
            if self._t_outside is None:
                self._t_outside = now
            since_ms = (now - self._t_outside) * 1000.0
        else:
            self._t_outside = None
            since_ms = 0.0

        # Per-axis: damping ratio if near / outside boundary
        dx = dx_arm_raw.copy()
        for i in range(3):
            d_near_lo = d_lo[i]
            d_near_hi = d_hi[i]
            if dx[i] < 0 and d_near_lo < self._margin:
                # pushing toward lo boundary
                g = max(0.0, min(1.0, (d_near_lo / self._margin) ** 2))
                dx[i] *= g
            elif dx[i] > 0 and d_near_hi < self._margin:
                # pushing toward hi boundary
                g = max(0.0, min(1.0, (d_near_hi / self._margin) ** 2))
                dx[i] *= g

        # Recompute target from damped dx (base_xyz = raw_xyz - dx_arm_raw)
        base_components = raw_xyz - dx_arm_raw
        new_xyz = base_components + dx
        # Hard-clip only as a safety net (should almost never trigger)
        new_xyz = np.clip(new_xyz, lo, hi)

        outward_normal = np.zeros(3)  # simplified for box
        bs = BoundaryState(
            d_to_boundary_mm = d_mm,
            outside          = not inside,
            since_outside_ms = since_ms,
            normal           = outward_normal.tolist(),
            tangential_gain  = 1.0,
            normal_gain      = float(np.mean(dx / (dx_arm_raw + 1e-9))),
            raw_xyz          = raw_xyz.tolist(),
            clamped_xyz      = new_xyz.tolist(),
        )
        return new_xyz, bs


# ─────────────────────────────────────────────────────────────────────────────
#  BoundaryMonitor  —  方案 E：ROS 邊界距離反饋
# ─────────────────────────────────────────────────────────────────────────────
class BoundaryMonitor:
    """
    Publishes two ROS topics every IK step:

      /{arm}/boundary_dist_mm  (std_msgs/Float32)
          Current signed distance to workspace boundary in mm.
          Positive = inside,  Negative = outside (OOR).
          Use for visual / haptic feedback in VR client.

      /{arm}/boundary_state    (std_msgs/String, JSON)
          Full BoundaryState as JSON (see ws_boundary.BoundaryState).

    Also prints console warnings at configurable thresholds.

    Usage:
        mon = BoundaryMonitor(ros2_node, arm="right")
        mon.publish(boundary_state)   # call every IK step
    """

    WARN_MM   = 30.0   # console yellow warning threshold
    DANGER_MM = 5.0    # console red warning threshold

    def __init__(self, node, arm: str):
        # Lazy import ros2 types so this module stays importable without ROS
        try:
            from std_msgs.msg import Float32, String
            self._dist_pub  = node.create_publisher(
                Float32, f"/{arm}/boundary_dist_mm", 10)
            self._state_pub = node.create_publisher(
                String,  f"/{arm}/boundary_state",   10)
            self._Float32 = Float32
            self._String  = String
            self._ok      = True
        except Exception:
            self._ok = False

        self._arm          = arm
        self._warn_printed = False

    def publish(self, bs: BoundaryState) -> None:
        """Publish boundary distance + state topics and print console warnings."""
        if not self._ok:
            return

        self._dist_pub.publish(
            self._Float32(data=float(bs.d_to_boundary_mm)))
        self._state_pub.publish(
            self._String(data=bs.to_json()))

        d = bs.d_to_boundary_mm
        if bs.outside:
            self._warn_printed = True
            print(
                f"\r  \033[31m[OOR]\033[0m  {self._arm}  "
                f"d={d:.0f}mm  since={bs.since_outside_ms:.0f}ms",
                end="", flush=True,
            )
        elif d < self.DANGER_MM:
            self._warn_printed = True
            print(
                f"\r  \033[33m[NEAR]\033[0m {self._arm}  d={d:.0f}mm",
                end="", flush=True,
            )
        elif self._warn_printed and d > self.WARN_MM:
            # Just recovered — print once then clear flag
            print(
                f"\r  \033[32m[OK]\033[0m   {self._arm}  d={d:.0f}mm",
                end="", flush=True,
            )
            self._warn_printed = False
