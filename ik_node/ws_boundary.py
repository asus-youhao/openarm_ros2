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
    self._soft_clamp = SoftClamp(ws_mesh, margin_m=0.05)
    self._bdry_mon   = BoundaryMonitor(node, arm)

    # 熱迴圈裡取代舊的 ws_mesh.clamp() 呼叫
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
#  SoftClamp  —  方案 A：彈性邊界，取代 WorkspaceMesh.clamp() 硬 snap
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
    ws_mesh    : WorkspaceMesh  (placo_ws_analyze.WorkspaceMesh)
    margin_m   : 開始衰減的距離（公尺），建議 = 1~2 個 voxel step
    box_ws     : 無 ws_mesh 時退回 box clamp dict {"x":(lo,hi), ...}
    """

    def __init__(
        self,
        ws_mesh=None,
        margin_m: float = 0.05,
        box_ws: Optional[dict] = None,
    ):
        self._mesh    = ws_mesh
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
        if self._mesh is not None:
            return self._apply_mesh(raw_xyz, dx_arm_raw)
        elif self._box is not None:
            return self._apply_box(raw_xyz, dx_arm_raw)
        else:
            bs = BoundaryState(raw_xyz=raw_xyz.tolist(), clamped_xyz=raw_xyz.tolist())
            return raw_xyz.copy(), bs

    # ── Mesh-based soft clamp ─────────────────────────────────────────────────
    def _apply_mesh(
        self,
        raw_xyz: np.ndarray,
        dx_arm_raw: np.ndarray,
    ) -> Tuple[np.ndarray, BoundaryState]:
        nearest_xyz, dist_m = self._mesh.nearest(raw_xyz)
        inside = self._mesh.contains(raw_xyz)

        # Distance convention: positive = inside, negative = outside
        signed_dist_m = dist_m if inside else -dist_m
        d_mm = signed_dist_m * 1000.0

        now = time.monotonic()
        if not inside:
            if self._t_outside is None:
                self._t_outside = now
            since_ms = (now - self._t_outside) * 1000.0
        else:
            self._t_outside  = None
            since_ms = 0.0

        # Boundary normal: unit vector pointing from nearest voxel toward raw_xyz
        # — if inside, it points toward the nearest boundary surface
        # — if outside, it points away from workspace (we'll use -normal for "inward")
        vec = raw_xyz - nearest_xyz
        vec_len = float(np.linalg.norm(vec))
        if vec_len > 1e-6:
            outward_normal = vec / vec_len    # points outward from workspace
        else:
            outward_normal = np.zeros(3)

        if inside:
            if dist_m >= self._margin:
                # ── Well inside: passthrough ──────────────────────────────────
                normal_gain = 1.0
                tangential_gain = 1.0
                new_xyz = raw_xyz.copy()
            else:
                # ── Near boundary: quadratic damping on outward component ─────
                g = (dist_m / self._margin) ** 2    # 0 at boundary, 1 at margin
                dx_normal_proj = float(np.dot(dx_arm_raw, outward_normal))
                if dx_normal_proj > 0:
                    # Moving outward → scale that component
                    dx_tangent = dx_arm_raw - dx_normal_proj * outward_normal
                    scaled_dx  = dx_tangent + g * dx_normal_proj * outward_normal
                    new_xyz    = raw_xyz - dx_arm_raw + scaled_dx
                    # (raw_xyz = base_xyz + dx_arm_raw, so new = base_xyz + scaled_dx)
                    normal_gain = g
                else:
                    # Moving inward (away from boundary) → passthrough
                    new_xyz = raw_xyz.copy()
                    normal_gain = 1.0
                tangential_gain = 1.0
        else:
            # ── Outside: tangential slide, zero outward component ─────────────
            dx_outward_proj = float(np.dot(dx_arm_raw, outward_normal))
            if dx_outward_proj > 0:
                # dx_arm is pushing further outside → remove that component
                dx_tangent = dx_arm_raw - dx_outward_proj * outward_normal
                # Project target onto boundary voxel + tangential offset
                new_xyz = nearest_xyz + dx_tangent
            else:
                # dx_arm is heading back inside → allow full delta
                new_xyz = raw_xyz.copy()
            normal_gain = 0.0
            tangential_gain = 1.0

            # Ensure the projected target is also inside workspace
            new_nearest, new_dist = self._mesh.nearest(new_xyz)
            new_inside = self._mesh.contains(new_xyz)
            if not new_inside:
                new_xyz = new_nearest.copy()

        bs = BoundaryState(
            d_to_boundary_mm = d_mm,
            outside          = not inside,
            since_outside_ms = since_ms,
            normal           = outward_normal.tolist(),
            tangential_gain  = tangential_gain,
            normal_gain      = normal_gain,
            raw_xyz          = raw_xyz.tolist(),
            clamped_xyz      = new_xyz.tolist(),
        )
        return new_xyz, bs

    # ── Box-based soft clamp ──────────────────────────────────────────────────
    def _apply_box(
        self,
        raw_xyz: np.ndarray,
        dx_arm_raw: np.ndarray,      # kept for apply() signature; unused here
    ) -> Tuple[np.ndarray, BoundaryState]:
        """Position-based soft rectangular clamp.

        Each axis is mapped independently through a smooth *saturating* knee
        that starts ``margin`` before the face.  The clamped target is a
        continuous, monotonically-increasing function of ``raw_xyz`` alone
        (it does NOT scale the incremental delta), so:

          * no snap-back to the session reference as the target nears a face
            (the old ``dx *= g`` collapsed the command toward ``base``),
          * no re-opening of the gate once outside (the old ``(d/margin)²``
            was a parabola symmetric about the face → gain climbed back to 1),
          * no hard ``np.clip`` jump — the map asymptotes to the face and
            never crosses it, so out-of-range targets glide to the boundary.

        Saturation law, per axis, upper side (lower side is symmetric):

            over = raw - (hi - margin)              # >0 once inside the band
            enc  = margin * over / (over + margin)  # ∈ [0, margin), slope 1 at
                                                    #   band entry → 0 far out
            new  = (hi - margin) + enc              # strictly < hi

        C¹-continuous at the band entry (slope matches passthrough), so there
        is no velocity discontinuity.  Tangential motion (axes not pressed
        against a face) passes through untouched → the arm slides along the
        boundary instead of sticking.
        """
        ws = self._box
        lo = np.array([ws["x"][0], ws["y"][0], ws["z"][0]])
        hi = np.array([ws["x"][1], ws["y"][1], ws["z"][1]])
        m  = self._margin

        # ── Position-based soft saturation, independent per axis ──────────────
        new_xyz  = raw_xyz.copy()
        min_gain = 1.0                          # local slope of the map (report)
        for i in range(3):
            r          = float(raw_xyz[i])
            lo_i, hi_i = float(lo[i]), float(hi[i])
            if hi_i - lo_i <= 2.0 * m:
                # Degenerate: margins from both faces overlap → hard clamp.
                new_xyz[i] = min(hi_i, max(lo_i, r))
                if r < lo_i or r > hi_i:
                    min_gain = 0.0
                continue
            over_hi  = r - (hi_i - m)           # >0 once within margin of hi
            under_lo = (lo_i + m) - r           # >0 once within margin of lo
            if over_hi > 0.0:
                enc        = m * over_hi / (over_hi + m)
                new_xyz[i] = (hi_i - m) + enc
                min_gain   = min(min_gain, (m * m) / ((over_hi + m) ** 2))
            elif under_lo > 0.0:
                enc        = m * under_lo / (under_lo + m)
                new_xyz[i] = (lo_i + m) - enc
                min_gain   = min(min_gain, (m * m) / ((under_lo + m) ** 2))
            # else: well inside → passthrough (new_xyz[i] already == r)

        # ── Reporting: signed distance of the *raw* target to nearest face ────
        d_lo = raw_xyz - lo
        d_hi = hi - raw_xyz
        signed_dists  = np.minimum(d_lo, d_hi)  # per-axis: positive = inside
        signed_dist_m = float(np.min(signed_dists))
        d_mm          = signed_dist_m * 1000.0
        inside        = bool(np.all(signed_dists >= 0))

        now = time.monotonic()
        if not inside:
            if self._t_outside is None:
                self._t_outside = now
            since_ms = (now - self._t_outside) * 1000.0
        else:
            self._t_outside = None
            since_ms = 0.0

        bs = BoundaryState(
            d_to_boundary_mm = d_mm,
            outside          = not inside,       # raw target OOR (command stays safe)
            since_outside_ms = since_ms,
            normal           = [0.0, 0.0, 0.0],  # axis-aligned; not used for box
            tangential_gain  = 1.0,
            normal_gain      = float(min_gain),
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

        # 本步 target 是否真的被 workspace clamp 改動（raw → clamped）。
        # 這是「box / mesh 實際觸發」最精準的訊號：只要 soft clamp 有衰減
        # 掉往邊界方向的位移，clamped 就會 != raw。
        raw, cl = bs.raw_xyz, bs.clamped_xyz
        cut_mm  = (max(abs(cl[i] - raw[i]) for i in range(3)) * 1000.0
                   if raw and cl else 0.0)
        engaged = cut_mm > 0.1   # >0.1mm 視為 clamp 實際觸發

        if bs.outside:
            # 紅底白字：已超出 workspace（OOR = out of range）
            self._warn_printed = True
            print(
                f"\r  \033[41;97m[OOR]\033[0m {self._arm}  "
                f"d={d:.0f}mm  cut={cut_mm:.0f}mm  since={bs.since_outside_ms:.0f}ms      ",
                end="", flush=True,
            )
        elif engaged:
            # 黃字：soft clamp 正在減速這一步（box 觸發），cut = 被砍掉多少 mm
            self._warn_printed = True
            print(
                f"\r  \033[93m[BOX]\033[0m {self._arm}  "
                f"d={d:.0f}mm  cut={cut_mm:.1f}mm  gain={bs.normal_gain:.2f}      ",
                end="", flush=True,
            )
        elif self._warn_printed and d > self.WARN_MM:
            # 綠字：剛脫離邊界 / clamp 解除，印一次後清旗標
            print(
                f"\r  \033[32m[OK]\033[0m  {self._arm}  d={d:.0f}mm                       ",
                end="", flush=True,
            )
            self._warn_printed = False
