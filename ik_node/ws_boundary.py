#!/usr/bin/env python3
"""
ws_boundary.py
==============
SoftClamp       — 位置飽和式軟邊界：彈性牆取代硬 np.clip
BoundaryMonitor — 邊界距離反饋 ROS topic + console 警示

可獨立 import，零 placo / scipy 依賴。

用法（在 node 內）::

    from ws_boundary import SoftClamp, BoundaryMonitor

    # __init__
    self._soft_clamp = SoftClamp(box_ws=cfg["workspace"], margin_m=0.05)
    self._bdry_mon   = BoundaryMonitor(node, arm)

    # 熱迴圈
    new_xyz, bs = self._soft_clamp.apply(np.array([raw_x, raw_y, raw_z]))
    self._bdry_mon.publish(bs)

只支援 ARM_CONFIG["workspace"] 的矩形 box。舊版另有一條 mesh 路徑
（reachability .npz + scipy KDTree/Delaunay），因 2026-07-23 的
position-saturating 修復只作用在 box 法則、且實際部署一律走 box fallback，
移植到本 repo 時已整條移除。
"""

import json
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
#  BoundaryState  —  傳遞邊界資訊的純資料類別
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class BoundaryState:
    d_to_boundary_mm:  float = 9999.0     # 距最近邊界面 mm；負 = 已超出
    outside:           bool  = False      # True → raw target 超出 workspace
    since_outside_ms:  float = 0.0        # 連續超出的累計時間 ms（0 = 在內部）
    normal:            list  = field(default_factory=lambda: [0., 0., 0.])
    tangential_gain:   float = 1.0        # 切向分量通過率
    normal_gain:       float = 1.0        # 飽和映射的局部斜率（<=1）
    raw_xyz:           list  = field(default_factory=lambda: [0., 0., 0.])
    clamped_xyz:       list  = field(default_factory=lambda: [0., 0., 0.])

    def to_json(self) -> str:
        """JSON payload 發佈到 /{arm}/boundary_state。

        欄位與舊版（含 mesh 路徑時）保持一致，避免破壞 VR 端訂閱者。
        box 法則下 ``normal`` 恆為零向量、``tangential_gain`` 恆為 1.0
        —— 因為 box 是逐軸獨立處理，沒有壓在面上的單一法向量，未被壓的軸
        本身就是 passthrough。
        """
        return json.dumps({
            "d_to_boundary_mm":  round(self.d_to_boundary_mm, 1),
            "outside":           self.outside,
            "since_outside_ms":  round(self.since_outside_ms, 1),
            "normal":            [round(v, 3) for v in self.normal],
            "tangential_gain":   round(self.tangential_gain, 3),
            "normal_gain":       round(self.normal_gain, 3),
        })


# ─────────────────────────────────────────────────────────────────────────────
#  SoftClamp  —  位置飽和式矩形軟邊界
# ─────────────────────────────────────────────────────────────────────────────
class SoftClamp:
    """
    Position-based soft rectangular clamp.

    Each axis is mapped independently through a smooth *saturating* knee that
    starts ``margin`` before the face.  The clamped target is a continuous,
    monotonically-increasing function of ``raw_xyz`` alone (it does NOT scale
    the incremental delta), so:

      * no snap-back to the session reference as the target nears a face
        (the old ``dx *= g`` collapsed the command toward ``base``),
      * no re-opening of the gate once outside (the old ``(d/margin)²`` was a
        parabola symmetric about the face → gain climbed back to 1),
      * no hard ``np.clip`` jump — the map asymptotes to the face and never
        crosses it, so out-of-range targets glide to the boundary.

    Saturation law, per axis, upper side (lower side is symmetric)::

        over = raw - (hi - margin)              # >0 once inside the band
        enc  = margin * over / (over + margin)  # ∈ [0, margin), slope 1 at
                                                #   band entry → 0 far out
        new  = (hi - margin) + enc              # strictly < hi

    C¹-continuous at the band entry (slope matches passthrough), so there is no
    velocity discontinuity.  Tangential motion (axes not pressed against a
    face) passes through untouched → the arm slides along the boundary instead
    of sticking.

    Parameters
    ----------
    box_ws   : workspace dict ``{"x": (lo, hi), "y": ..., "z": ...}``
               （通常直接餵 ``ARM_CONFIG[arm]["workspace"]``）
               None → clamp 停用，``apply()`` 變成 passthrough。
    margin_m : 開始飽和的距離（公尺）
    """

    def __init__(
        self,
        box_ws:   Optional[dict] = None,
        margin_m: float = 0.05,
    ):
        self._box    = box_ws
        self._margin = float(margin_m)
        self._t_outside: Optional[float] = None   # 首次超出的時間戳

    # ── Public API ────────────────────────────────────────────────────────────
    def apply(self, raw_xyz: np.ndarray) -> Tuple[np.ndarray, BoundaryState]:
        """
        Parameters
        ----------
        raw_xyz : 完整 target = base_xyz + dx_arm（未 clamp）

        Returns
        -------
        new_xyz : np.ndarray shape (3,) — 飽和後的 target XYZ
        state   : BoundaryState
        """
        if self._box is None:
            bs = BoundaryState(raw_xyz=raw_xyz.tolist(),
                               clamped_xyz=raw_xyz.tolist())
            return raw_xyz.copy(), bs

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
            d_to_boundary_mm = signed_dist_m * 1000.0,
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
#  BoundaryMonitor  —  ROS 邊界距離反饋
# ─────────────────────────────────────────────────────────────────────────────
class BoundaryMonitor:
    """
    Publishes two ROS topics every IK step:

      /{arm}/boundary_dist_mm  (std_msgs/Float32)
          Signed distance of the raw target to the workspace boundary, in mm.
          Positive = inside,  Negative = outside (OOR).
          Use for visual / haptic feedback in the VR client.

      /{arm}/boundary_state    (std_msgs/String, JSON)
          Full BoundaryState as JSON (see BoundaryState.to_json).

    Also prints single-line console status at configurable thresholds.

    Usage::

        mon = BoundaryMonitor(ros2_node, arm="right")
        mon.publish(boundary_state)   # call every IK step
    """

    WARN_MM   = 30.0   # 脫離邊界後恢復印 [OK] 的門檻
    ENGAGED_MM = 0.1   # clamped 與 raw 差多少 mm 視為 clamp 實際觸發

    def __init__(self, node, arm: str):
        # Lazy import ROS types so this module stays importable without ROS
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
        """Publish boundary distance + state topics and print console status."""
        if not self._ok:
            return

        self._dist_pub.publish(
            self._Float32(data=float(bs.d_to_boundary_mm)))
        self._state_pub.publish(
            self._String(data=bs.to_json()))

        d = bs.d_to_boundary_mm

        # 本步 target 是否真的被 workspace clamp 改動（raw → clamped）。
        # 這是「box 實際觸發」最精準的訊號：只要飽和法則有削掉往邊界方向的
        # 位移，clamped 就會 != raw。
        raw, cl = bs.raw_xyz, bs.clamped_xyz
        cut_mm  = (max(abs(cl[i] - raw[i]) for i in range(3)) * 1000.0
                   if raw and cl else 0.0)
        engaged = cut_mm > self.ENGAGED_MM

        if bs.outside:
            # 紅底白字：raw target 已超出 workspace（OOR = out of range）
            self._warn_printed = True
            print(
                f"\r  \033[41;97m[OOR]\033[0m {self._arm}  "
                f"d={d:.0f}mm  cut={cut_mm:.0f}mm  since={bs.since_outside_ms:.0f}ms      ",
                end="", flush=True,
            )
        elif engaged:
            # 黃字：soft clamp 正在減速這一步，cut = 被削掉多少 mm
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
