#!/usr/bin/env python3
"""
tracker_ee_absolute.py — Absolute EE pose publisher for IK solving
===================================================================
相對版 (tracker_ee_delta.py) 發送的是「相對 reference」的 delta。
本檔發送 tracker 在世界座標系的「絕對 XYZ + 絕對 quaternion」，
讓 IK 直接解 absolute target pose。

座標系: OpenXR → ROS frame (+X=forward, +Y=left, +Z=up)
發布: /ee_target/{side}  geometry_msgs/PoseStamped
  .header.frame_id = "world"
  .pose.position   = absolute XYZ [m]  (after LPF)
  .pose.orientation = absolute quaternion (after LPF, normalized)

與 delta 版的差異:
  tracker_ee_delta.py         → dp = current - ref  (相對, IK compute incremental step)
  tracker_ee_absolute.py      → p  = current         (絕對, IK compute full target pose)

Workflow:
  1. Launch this node
  2. 開機後 tracker 座標直接發送（不需要 SPACE）
  3. 按 SPACE 可以選擇性地「凍結」(暫停發送)，再按恢復
  4. IK node 訂閱 /ee_target/{side}，解 joint angles

Keyboard (click figure first):
    SPACE / Enter   toggle pause / publish
    r               reset LP filter state (clears filter history)
    1/2/3/4         LP filter 5/10/15/20 Hz
    5/6/7/8         RPY display filter 5/10/15/20 Hz  (does NOT affect publish)
    q / Escape      quit

Published while NOT paused, and signal quality >= min_quality:
    /ee_target/left    geometry_msgs/PoseStamped
    /ee_target/right   geometry_msgs/PoseStamped

Rate limiting: --pub-hz (default 40 Hz) — prevents IK queue overflow
  Rule: pub_hz ≤ 0.8 × (1000 / ik_ms)
  IK=20ms → max 50Hz → use 40Hz

Usage:
    python tracker_ee_absolute.py --tracker both
    python tracker_ee_absolute.py --tracker right --filter-hz 10 --pub-hz 40
    python tracker_ee_absolute.py --tracker both  --min-quality 50 --fps 15
"""

import sys, os, argparse, threading, math, collections
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tracker_analysis as ta
from tracker_analysis_noocc import ik_quality_score_noocc, _CSV_ROOT
ta.ik_quality_score = ik_quality_score_noocc

try:
    import rclpy
    from rclpy.node import Node
    _HAS_ROS = True
except ImportError:
    print("[ERROR] rclpy not available — source ROS2 setup.bash"); sys.exit(1)

try:
    from geometry_msgs.msg import PoseStamped
except ImportError:
    print("[ERROR] geometry_msgs not available"); sys.exit(1)

# ── Quaternion helpers (x,y,z,w) ──────────────────────────────────────────────
def _qmul(q1, q2):
    x1,y1,z1,w1 = q1;  x2,y2,z2,w2 = q2
    return np.array([w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2,
                     w1*w2-x1*x2-y1*y2-z1*z2])

def _qnorm(q):
    n = math.sqrt(float(q[0]**2+q[1]**2+q[2]**2+q[3]**2))
    return np.array(q)/n if n > 1e-9 else np.array([0.,0.,0.,1.])

def _qrpy_deg(q):
    """(x,y,z,w) -> roll,pitch,yaw degrees"""
    qx,qy,qz,qw = q
    roll  = math.degrees(math.atan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2)))
    pitch = math.degrees(math.asin(max(-1., min(1., 2*(qw*qy-qz*qx)))))
    yaw   = math.degrees(math.atan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2)))
    return roll, pitch, yaw

def _rpy_to_rot(roll_deg, pitch_deg, yaw_deg):
    """ZYX RPY → 3x3 rotation matrix"""
    r=math.radians(roll_deg); p=math.radians(pitch_deg); y=math.radians(yaw_deg)
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p)
    cy,sy=math.cos(y),math.sin(y)
    return np.array([
        [cy*cp,  cy*sp*sr-sy*cr,  cy*sp*cr+sy*sr],
        [sy*cp,  sy*sp*sr+cy*cr,  sy*sp*cr-cy*sr],
        [-sp,    cp*sr,           cp*cr          ]])

# ── Coordinate frame transform: OpenXR (Pico) → ROS ──────────────────────────
def _pico_to_ros(pose7):
    """
    OpenXR (+X=right, +Y=up, +Z=back) → ROS (+X=fwd, +Y=left, +Z=up)
    ros_x = -pico_z   (forward)
    ros_y = -pico_x   (left)
    ros_z =  pico_y   (up)
    q_ros = q_transform ⊗ q_pico,   q_transform = [0.5, -0.5, -0.5, 0.5]
    """
    px, py, pz, qx, qy, qz, qw = pose7
    ros_pos = np.array([-pz, -px, py])
    q_transform = np.array([0.5, -0.5, -0.5, 0.5])
    q_ros = _qmul(q_transform, [qx, qy, qz, qw])
    q_ros = _qnorm(q_ros)
    return np.concatenate([ros_pos, q_ros])

def _identity_transform(pose7):
    return np.array(pose7, dtype=float)

# ── Causal IIR LP filter — second-order (two cascaded stages) ────────────────
class _IIR2:
    """Second-order causal IIR LP — two cascaded 1st-order stages.
    Roll-off: -40 dB/decade vs -20 dB/decade for 1st-order.
    Alpha is compensated (×1.5538) so -3dB stays at the requested fc_hz.
    Latency: ~2 samples."""
    def __init__(self, fc_hz, fs_est=100.):
        self._s1 = None
        self._s2 = None
        self.set_fc(fc_hz, fs_est)

    def set_fc(self, fc_hz, fs_est=100.):
        fc_each = fc_hz * 1.5538
        dt = 1.0 / max(fs_est, 1.)
        rc = 1.0 / (2.0 * math.pi * max(fc_each, 0.1))
        self._a = dt / (rc + dt)

    def update(self, x7):
        x = np.array(x7, dtype=float)
        if self._s1 is None:
            self._s1 = x.copy()
            self._s2 = x.copy()
        else:
            self._s1[:3] = self._a * x[:3] + (1 - self._a) * self._s1[:3]
            q1 = self._a * x[3:7] + (1 - self._a) * self._s1[3:7]
            self._s1[3:7] = _qnorm(q1)
            self._s2[:3] = self._a * self._s1[:3] + (1 - self._a) * self._s2[:3]
            q2 = self._a * self._s1[3:7] + (1 - self._a) * self._s2[3:7]
            self._s2[3:7] = _qnorm(q2)
        return self._s2.copy()

    def reset(self):
        self._s1 = None
        self._s2 = None

# ── Display-only RPY IIR — second-order wrap-aware ────────────────────────────
class _IIR2_RPY:
    """Second-order wrap-aware IIR LP for RPY Euler angles [degrees].
    Two cascaded wrap-aware stages. -40 dB/decade roll-off.
    Alpha compensated so -3dB stays at fc_hz. Does NOT affect published pose."""
    def __init__(self, fc_hz, fs_est=100.):
        self._s1 = None
        self._s2 = None
        self.set_fc(fc_hz, fs_est)

    def set_fc(self, fc_hz, fs_est=100.):
        fc_each = fc_hz * 1.5538
        dt = 1.0 / max(fs_est, 1.)
        rc = 1.0 / (2.0 * math.pi * max(fc_each, 0.1))
        self._a = dt / (rc + dt)

    def update(self, rpy_deg):
        x = np.array(rpy_deg, dtype=float)
        if self._s1 is None:
            self._s1 = x.copy()
            self._s2 = x.copy()
        else:
            diff1 = x - self._s1
            diff1 = (diff1 + 180.) % 360. - 180.
            self._s1 = self._s1 + self._a * diff1
            diff2 = self._s1 - self._s2
            diff2 = (diff2 + 180.) % 360. - 180.
            self._s2 = self._s2 + self._a * diff2
        return self._s2.copy()

    def reset(self):
        self._s1 = None
        self._s2 = None

# ── Display buffer ─────────────────────────────────────────────────────────────
class AbsBuf:
    """Stores absolute poses for display."""
    def __init__(self, maxlen=600):
        self._lock = threading.Lock()
        self._buf  = collections.deque(maxlen=maxlen)

    def push(self, ts, xyz, rpy_deg, quality):
        with self._lock:
            self._buf.append((ts,
                              xyz[0], xyz[1], xyz[2],
                              rpy_deg[0], rpy_deg[1], rpy_deg[2],
                              quality))

    def clear(self):
        with self._lock: self._buf.clear()

    def snapshot(self):
        with self._lock:
            if len(self._buf) < 2: return None
            return np.array(self._buf)
        # cols: t, x, y, z, roll, pitch, yaw, quality

# ── Per-side controller state ─────────────────────────────────────────────────
class SideState:
    """
    Publishes ABSOLUTE pose (world-space XYZ + quaternion) to /ee_target/{side}.
    No reference subtraction — IK receives the full target pose directly.
    """
    def __init__(self, side, filter_hz, min_quality, coord_system='pico',
                 debug=False, rpy_display_hz=15., pub_hz=40.):
        self.side         = side
        self.min_q        = min_quality
        self.iir          = _IIR2(filter_hz)
        self.filter_hz    = filter_hz
        self.iir_rpy_disp = _IIR2_RPY(rpy_display_hz)   # display only
        self.rpy_disp_hz  = rpy_display_hz
        self.tracker_buf  = ta.TrackerBuffer(maxlen=2000)
        self.abs_buf      = AbsBuf()
        self._lock        = threading.Lock()
        self._debug       = debug
        self._msg_count   = 0
        # Coordinate transform
        self._transform   = _pico_to_ros if coord_system == 'pico' else _identity_transform
        # State machine
        self.publishing   = True    # starts publishing immediately (no SPACE required)
        self.blocked      = False
        self.quality      = 0.
        self.cur_xyz      = np.zeros(3)
        self.cur_rpy      = np.zeros(3)   # display RPY (filtered separately)
        self.cur_q        = np.array([0.,0.,0.,1.])
        self.ready        = False
        # Rate limiting
        self.pub_hz        = pub_hz
        self._pub_interval = 1.0 / max(pub_hz, 1.)
        self._last_pub_t   = 0.0
        self.publisher     = None   # set by TargetNode

    def set_filter(self, fc_hz):
        self.filter_hz = fc_hz
        self.iir.set_fc(fc_hz)

    def reset_filter(self):
        """Reset filter state (clears history, re-initializes on next pose)."""
        self.iir.reset()
        self.iir_rpy_disp.reset()
        self.abs_buf.clear()
        print(f"[{self.side}] Filter state RESET")

    def set_rpy_disp_hz(self, hz):
        self.rpy_disp_hz = hz
        self.iir_rpy_disp.set_fc(hz)

    def set_pub_hz(self, hz):
        self.pub_hz = hz
        self._pub_interval = 1.0 / max(hz, 1.)

    def on_pose(self, ts, pose7):
        """Called from ROS callback. pose7 already in ROS frame."""
        # 1. IIR filter (pos + quat)
        filt = self.iir.update(pose7)
        self._msg_count += 1

        # 2. Display RPY (separate filter)
        raw_rpy = np.array(_qrpy_deg(filt[3:7]))
        disp_rpy = self.iir_rpy_disp.update(raw_rpy)

        # 3. Quality via tracker buffer
        self.tracker_buf.push(ts, filt)
        snap = self.tracker_buf.snapshot()
        t_ = snap['t']
        if len(t_) >= 4:
            mask = t_ >= (t_[-1] - 1.0)
            if mask.sum() >= 4:
                dw = {k: snap[k][mask]
                      for k in ['jerk_mag', 'occ_flag', 'jump_flag', 'vel_mag']}
                self.quality = ik_quality_score_noocc(dw)['total']

        with self._lock:
            self.blocked = self.quality < self.min_q
            self.cur_xyz = filt[:3].copy()
            self.cur_rpy = disp_rpy.copy()
            self.cur_q   = filt[3:7].copy()
            self.ready   = True

        # 4. Debug
        if self._debug and self._msg_count % 100 == 0:
            print(f"[{self.side}] ABS  xyz=[{filt[0]:+.3f},{filt[1]:+.3f},{filt[2]:+.3f}]m  "
                  f"rpy=[{disp_rpy[0]:+.1f},{disp_rpy[1]:+.1f},{disp_rpy[2]:+.1f}]deg  "
                  f"Q={self.quality:.0f}  pub={self.publishing}  blocked={self.blocked}")

        # 5. Push display buffer
        self.abs_buf.push(ts, filt[:3], disp_rpy, self.quality)

        # 6. Publish absolute pose if publishing + not blocked + rate gate
        if self.publishing and not self.blocked and self.publisher is not None:
            if ts - self._last_pub_t >= self._pub_interval:
                msg = PoseStamped()
                msg.header.frame_id = "world"
                msg.header.stamp.sec = int(ts)
                msg.header.stamp.nanosec = int((ts % 1.0) * 1e9)
                msg.pose.position.x = float(filt[0])
                msg.pose.position.y = float(filt[1])
                msg.pose.position.z = float(filt[2])
                msg.pose.orientation.x = float(filt[3])
                msg.pose.orientation.y = float(filt[4])
                msg.pose.orientation.z = float(filt[5])
                msg.pose.orientation.w = float(filt[6])
                try:
                    self.publisher.publish(msg)
                    self._last_pub_t = ts
                except Exception:
                    pass

# ── ROS2 node ──────────────────────────────────────────────────────────────────
class TargetNode(Node):
    def __init__(self, states: dict):
        super().__init__("tracker_ee_absolute_node")
        for side, state in states.items():
            state.publisher = self.create_publisher(PoseStamped, f"/ee_target/{side}", 10)
            self.create_subscription(PoseStamped, f"/tracker/{side}",
                                     lambda m, s=state: self._cb(m, s), 10)
        self.get_logger().info(
            f"ABSOLUTE mode. Topics: /ee_target/{{{list(states.keys())}}}")

    def _cb(self, msg, state: SideState):
        p = msg.pose.position; q = msg.pose.orientation
        if p.x == 0. and p.y == 0. and p.z == 0.: return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose7_raw = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        pose7_ros  = state._transform(pose7_raw)
        state.on_pose(ts, pose7_ros)

# ── Colors ─────────────────────────────────────────────────────────────────────
_C = {"x":"#ff6b6b","y":"#4ecdc4","z":"#45b7d1",
      "r":"#f0a500","p":"#c9f01a","yaw":"#ff4fd8",
      "pub":"#fdcb6e","idle":"#888888","blocked":"#ff4757",
      "trail":"#e17055"}

# ── Figure builder ─────────────────────────────────────────────────────────────
def build_fig(sides):
    n = len(sides)
    fig = plt.figure(figsize=(13*n, 16), facecolor=ta.BG_DARK)

    outer = gridspec.GridSpec(2, 1, height_ratios=[0.055, 1],
                              left=0.04, right=0.98, top=0.97, bottom=0.03,
                              hspace=0.07)
    ax_sb = fig.add_subplot(outer[0])
    ax_sb.set_facecolor(ta.BG_PANEL); ax_sb.set_xticks([]); ax_sb.set_yticks([])
    for sp in ax_sb.spines.values(): sp.set_edgecolor("#444")
    sb_txt = ax_sb.text(0.5, 0.5,
        "[ ABSOLUTE — PUBLISHING ]  Waiting for tracker...  "
        "SPACE=pause  r=reset-filter  1/2/3/4=pos-filter  5/6/7/8=disp-filter  q=quit",
        color=_C["pub"], fontsize=10, fontweight="bold",
        ha="center", va="center", transform=ax_sb.transAxes)

    cols = gridspec.GridSpecFromSubplotSpec(1, n, subplot_spec=outer[1], wspace=0.10)
    ax_d = {}

    for ci, side in enumerate(sides):
        gs = gridspec.GridSpecFromSubplotSpec(
            4, 2, subplot_spec=cols[ci],
            height_ratios=[1.0, 1.6, 1.0, 0.75],
            hspace=0.50, wspace=0.30)
        ax_pos = fig.add_subplot(gs[0, 0])   # XYZ absolute
        ax_rpy = fig.add_subplot(gs[0, 1])   # RPY absolute
        ax_3d  = fig.add_subplot(gs[1, :], projection='3d')
        ax_xy  = fig.add_subplot(gs[2, 0])
        ax_yz  = fig.add_subplot(gs[2, 1])
        ax_q   = fig.add_subplot(gs[3, :])

        for ax in [ax_pos, ax_rpy, ax_xy, ax_yz, ax_q]: ta._sax(ax)

        # 3D absolute trajectory
        ax_3d.set_facecolor(ta.BG_DARK)
        ax_3d.xaxis.pane.fill = False; ax_3d.yaxis.pane.fill = False; ax_3d.zaxis.pane.fill = False
        ax_3d.xaxis.pane.set_edgecolor('#2a2a2a')
        ax_3d.yaxis.pane.set_edgecolor('#2a2a2a')
        ax_3d.zaxis.pane.set_edgecolor('#2a2a2a')
        ax_3d.tick_params(colors='#555', labelsize=5)
        ax_3d.set_xlabel('X [m]', color='#888', fontsize=7, labelpad=1)
        ax_3d.set_ylabel('Y [m]', color='#888', fontsize=7, labelpad=1)
        ax_3d.set_zlabel('Z [m]', color='#888', fontsize=7, labelpad=1)
        ax_3d.set_title(
            f"{side.upper()} | ABSOLUTE 3D trajectory  (ROS: +X=fwd, +Y=left, +Z=up)",
            color=_C['pub'], fontsize=8, loc='left')
        ax_3d.view_init(elev=22., azim=-55.)
        # World origin marker
        ax_3d.scatter([0],[0],[0], color='#636e72', s=60, marker='+', depthshade=False, zorder=5)
        for v, col, lbl in [([0.05,0,0],_C['x'],'+X'),([0,0.05,0],_C['y'],'+Y'),([0,0,0.05],_C['z'],'+Z')]:
            ax_3d.quiver(0,0,0,v[0],v[1],v[2],color=col,lw=1.5,arrow_length_ratio=0.3)
            ax_3d.text(v[0]*1.5, v[1]*1.5, v[2]*1.5, lbl, color=col, fontsize=6)
        l_3d_trail, = ax_3d.plot([], [], [], color=_C['trail'], lw=1.3, alpha=0.8, zorder=3)
        l_3d_dot,   = ax_3d.plot([], [], [], 'o', color='white', ms=8,
                                  markeredgecolor='#888', markeredgewidth=0.5, zorder=6)
        l_3d_ax, = ax_3d.plot([], [], [], color='#ff5555', lw=2.5, zorder=5, label='+X')
        l_3d_ay, = ax_3d.plot([], [], [], color='#55cc55', lw=2.5, zorder=5, label='+Y')
        l_3d_az, = ax_3d.plot([], [], [], color='#5588ff', lw=2.5, zorder=5, label='+Z')
        ax_3d.legend(fontsize=6, loc='upper left',
                     facecolor=ta.BG_PANEL, edgecolor='#333', labelcolor='white')
        txt_3d = ax_3d.text2D(0.02, 0.97, 'waiting...',
                              color='#cccccc', fontsize=7,
                              transform=ax_3d.transAxes, va='top')

        # XYZ absolute time-series
        ax_pos.set_title(f"{side.upper()} | ABS position [m]", color="#cccccc", fontsize=8, loc="left")
        ax_pos.axhline(0., color="#444", lw=0.7)
        ax_pos.set_ylabel("m", color="#888", fontsize=7)
        ax_pos.set_xlabel("t [s]", color="#888", fontsize=7)
        l_x, = ax_pos.plot([], [], color=_C["x"], lw=1.4, label="X")
        l_y, = ax_pos.plot([], [], color=_C["y"], lw=1.4, label="Y")
        l_z, = ax_pos.plot([], [], color=_C["z"], lw=1.4, label="Z")
        ax_pos.legend(fontsize=6.5, facecolor=ta.BG_PANEL, edgecolor="#333",
                      labelcolor="white", loc="upper left", ncol=3)
        txt_xyz = ax_pos.text(0.01, 0.88, "XYZ: --", color="white", fontsize=7,
                              transform=ax_pos.transAxes,
                              bbox=dict(facecolor=ta.BG_PANEL, alpha=0.7, pad=1.5))

        # RPY time-series
        ax_rpy.set_title(f"{side.upper()} | ABS rotation [deg]", color="#cccccc", fontsize=8, loc="left")
        ax_rpy.axhline(0., color="#444", lw=0.7)
        ax_rpy.set_ylabel("deg", color="#888", fontsize=7)
        ax_rpy.set_xlabel("t [s]", color="#888", fontsize=7)
        l_r,    = ax_rpy.plot([], [], color=_C["r"],   lw=1.4, label="Roll")
        l_p,    = ax_rpy.plot([], [], color=_C["p"],   lw=1.4, label="Pitch")
        l_yaw_, = ax_rpy.plot([], [], color=_C["yaw"], lw=1.4, label="Yaw")
        ax_rpy.legend(fontsize=6.5, facecolor=ta.BG_PANEL, edgecolor="#333",
                      labelcolor="white", loc="upper left", ncol=3)
        txt_rpy = ax_rpy.text(0.01, 0.88, "RPY: --", color="white", fontsize=7,
                              transform=ax_rpy.transAxes,
                              bbox=dict(facecolor=ta.BG_PANEL, alpha=0.7, pad=1.5))

        # XY plane trajectory
        ax_xy.set_title(f"{side.upper()} | XY plane", color="#cccccc", fontsize=8, loc="left")
        ax_xy.set_xlabel("X [m]", color="#888", fontsize=7)
        ax_xy.set_ylabel("Y [m]", color="#888", fontsize=7)
        ax_xy.axhline(0., color="#444", lw=0.5); ax_xy.axvline(0., color="#444", lw=0.5)
        l_txy, = ax_xy.plot([], [], color=_C["z"], lw=1.2, alpha=0.7)
        l_dxy, = ax_xy.plot([], [], 'o', color="#ffffff", ms=6, zorder=6)

        # YZ plane trajectory
        ax_yz.set_title(f"{side.upper()} | YZ plane", color="#cccccc", fontsize=8, loc="left")
        ax_yz.set_xlabel("Y [m]", color="#888", fontsize=7)
        ax_yz.set_ylabel("Z [m]", color="#888", fontsize=7)
        ax_yz.axhline(0., color="#444", lw=0.5); ax_yz.axvline(0., color="#444", lw=0.5)
        l_tyz, = ax_yz.plot([], [], color="#ff9f43", lw=1.2, alpha=0.7)
        l_dyz, = ax_yz.plot([], [], 'o', color="#ffffff", ms=6, zorder=6)

        # Quality bar
        ax_q.set_title(f"{side.upper()} | signal quality (1s window) + publish status",
                       color="#cccccc", fontsize=8, loc="left")
        ax_q.set_ylim(0, 108)
        ax_q.axhline(70., color=ta.CL["bar_good"], lw=0.8, ls=":", alpha=0.6)
        ax_q.axhline(40., color=ta.CL["bar_warn"],  lw=0.8, ls=":", alpha=0.6)
        ax_q.text(0.01, 73./108, ">=70 good", color=ta.CL["bar_good"],
                  fontsize=5.5, transform=ax_q.transAxes)
        ax_q.text(0.01, 42./108, ">=40 warn",  color=ta.CL["bar_warn"],
                  fontsize=5.5, transform=ax_q.transAxes)
        ax_q.set_ylabel("quality", color="#888", fontsize=7)
        ax_q.set_xlabel("t [s]", color="#888", fontsize=7)
        l_q, = ax_q.plot([], [], color=ta.CL["bar_good"], lw=2.)
        txt_q     = ax_q.text(0.97, 0.92, "Q: --", color="white", fontsize=9,
                              fontweight="bold", ha="right", transform=ax_q.transAxes)
        txt_block = ax_q.text(0.5, 0.5, "", color=_C["blocked"], fontsize=12,
                               fontweight="bold", ha="center", va="center",
                               transform=ax_q.transAxes)

        ax_d[side] = dict(
            ax_pos=ax_pos, ax_rpy=ax_rpy, ax_xy=ax_xy, ax_yz=ax_yz, ax_q=ax_q,
            l_x=l_x, l_y=l_y, l_z=l_z,
            l_r=l_r, l_p=l_p, l_yaw_=l_yaw_,
            l_txy=l_txy, l_dxy=l_dxy, l_tyz=l_tyz, l_dyz=l_dyz,
            l_q=l_q, txt_q=txt_q, txt_block=txt_block,
            txt_xyz=txt_xyz, txt_rpy=txt_rpy,
            ax_3d=ax_3d, l_3d_trail=l_3d_trail, l_3d_dot=l_3d_dot,
            l_3d_ax=l_3d_ax, l_3d_ay=l_3d_ay, l_3d_az=l_3d_az, txt_3d=txt_3d,
        )

    return fig, ax_d, sb_txt

# ── Per-frame update ───────────────────────────────────────────────────────────
def _upd_side(state: SideState, d: dict, win: float):
    snap = state.abs_buf.snapshot()
    if snap is None: return

    t    = snap[:,0] - snap[0,0]
    mask = t >= max(0., t[-1] - win)
    if mask.sum() < 2: return
    tw   = t[mask] - t[mask][0]

    x = snap[mask,1]; y = snap[mask,2]; z = snap[mask,3]
    roll = snap[mask,4]; pitch = snap[mask,5]; yaw_ = snap[mask,6]
    q_arr= snap[mask,7]

    # XYZ
    d['l_x'].set_data(tw,x); d['l_y'].set_data(tw,y); d['l_z'].set_data(tw,z)
    d['ax_pos'].set_xlim(0.,win); d['ax_pos'].relim(); d['ax_pos'].autoscale_view(scalex=False)

    # RPY
    d['l_r'].set_data(tw,roll); d['l_p'].set_data(tw,pitch); d['l_yaw_'].set_data(tw,yaw_)
    d['ax_rpy'].set_xlim(0.,win); d['ax_rpy'].relim(); d['ax_rpy'].autoscale_view(scalex=False)

    # Trajectories
    d['l_txy'].set_data(x,y); d['l_dxy'].set_data([x[-1]],[y[-1]])
    d['l_tyz'].set_data(y,z); d['l_dyz'].set_data([y[-1]],[z[-1]])
    r = max(0.05, float(np.abs(np.concatenate([x,y,z])).max()) * 1.3)
    d['ax_xy'].set_xlim(-r,r); d['ax_xy'].set_ylim(-r,r)
    d['ax_yz'].set_xlim(-r,r); d['ax_yz'].set_ylim(-r,r)

    # 3D
    d['l_3d_trail'].set_data_3d(x, y, z)
    cx,cy3,cz3 = float(x[-1]), float(y[-1]), float(z[-1])
    d['l_3d_dot'].set_data_3d([cx],[cy3],[cz3])
    R = _rpy_to_rot(float(roll[-1]), float(pitch[-1]), float(yaw_[-1]))
    for i, key in enumerate(('l_3d_ax','l_3d_ay','l_3d_az')):
        tip = R[:,i] * 0.08
        d[key].set_data_3d([cx,cx+tip[0]],[cy3,cy3+tip[1]],[cz3,cz3+tip[2]])
    margin = 0.12
    xr_lo, xr_hi = float(x.min())-margin, float(x.max())+margin
    yr_lo, yr_hi = float(y.min())-margin, float(y.max())+margin
    zr_lo, zr_hi = float(z.min())-margin, float(z.max())+margin
    ax3 = d['ax_3d']
    ax3.set_xlim3d(xr_lo, xr_hi); ax3.set_ylim3d(yr_lo, yr_hi); ax3.set_zlim3d(zr_lo, zr_hi)
    state_lbl = 'PUBLISHING' if state.publishing else 'PAUSED'
    d['txt_3d'].set_text(
        f"{state_lbl}  |  X:{cx:+.3f}  Y:{cy3:+.3f}  Z:{cz3:+.3f} m  |  "
        f"Roll:{float(roll[-1]):+.0f}  Pitch:{float(pitch[-1]):+.0f}  "
        f"Yaw:{float(yaw_[-1]):+.0f} deg")

    # Quality
    d['l_q'].set_data(tw, q_arr)
    d['ax_q'].set_xlim(0., win)
    q_now = float(q_arr[-1])
    d['l_q'].set_color(ta._bc(q_now))
    d['txt_q'].set_text(f"Q: {q_now:.0f}")
    d['txt_q'].set_color(ta._bc(q_now))

    if not state.publishing:
        d['txt_block'].set_text("[ PAUSED ]  press SPACE to resume publishing")
        d['txt_block'].set_color(_C["idle"])
    elif state.blocked:
        d['txt_block'].set_text("BLOCKED — quality too low")
        d['txt_block'].set_color(_C["blocked"])
    else:
        d['txt_block'].set_text("")

    # Numeric labels
    with state._lock:
        xyz = state.cur_xyz.copy()
        rpy = state.cur_rpy.copy()
    d['txt_xyz'].set_text(f"X={xyz[0]:+.3f}  Y={xyz[1]:+.3f}  Z={xyz[2]:+.3f} m")
    d['txt_rpy'].set_text(f"R={rpy[0]:+.1f}  P={rpy[1]:+.1f}  Y={rpy[2]:+.1f} deg")

# ── Keyboard handler ───────────────────────────────────────────────────────────
def make_key_handler(states, sb_txt, sides):
    _pub = [True]   # starts publishing

    def _status():
        fc = list(states.values())[0].filter_hz
        hz = list(states.values())[0].pub_hz
        if _pub[0]:
            bk = [s for s in sides if states[s].blocked]
            if bk:
                return (f"[ ABSOLUTE — PUBLISHING @{hz:.0f}Hz ]  @{fc:.0f}Hz | "
                        f"BLOCKED: {','.join(bk)}  SPACE=pause  q=quit"), _C["blocked"]
            return (f"[ ABSOLUTE — PUBLISHING @{hz:.0f}Hz ]  @{fc:.0f}Hz | "
                    f"SPACE=pause  r=reset-filter  q=quit"), _C["pub"]
        return (f"[ ABSOLUTE — PAUSED ]  @{fc:.0f}Hz | "
                f"SPACE=resume  r=reset-filter  q=quit"), _C["idle"]

    def on_key(event):
        k = (event.key or "").lower()
        if k in (' ', 'enter'):
            _pub[0] = not _pub[0]
            for st in states.values(): st.publishing = _pub[0]
            txt, col = _status(); sb_txt.set_text(txt); sb_txt.set_color(col)
            sb_txt.axes.set_facecolor("#0a2a0a" if _pub[0] else ta.BG_PANEL)

        elif k == 'r':
            for st in states.values(): st.reset_filter()
            sb_txt.set_text("FILTER RESET  " + _status()[0])
            sb_txt.set_color(_status()[1])

        elif k in ('1','2','3','4'):
            hz = {'1':5.,'2':10.,'3':15.,'4':20.}[k]
            for st in states.values(): st.set_filter(hz)
            txt, col = _status(); sb_txt.set_text(txt); sb_txt.set_color(col)

        elif k in ('5','6','7','8'):
            hz = {'5':5.,'6':10.,'7':15.,'8':20.}[k]
            for st in states.values(): st.set_rpy_disp_hz(hz)
            txt, col = _status(); sb_txt.set_text(txt); sb_txt.set_color(col)

        elif k in ('q', 'escape'):
            plt.close('all')

    return on_key, _pub, _status

# ── main ───────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="Absolute EE pose publisher for IK (no reference subtraction)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--tracker",       choices=["left","right","both"], default="both")
    ap.add_argument("--filter-hz",     type=float, default=10.,
                    help="Pos+quat LP cutoff Hz (default:10)")
    ap.add_argument("--rpy-disp-hz",   type=float, default=15.,
                    help="Display-only RPY LP Hz (default:15, does NOT affect published pose)")
    ap.add_argument("--pub-hz",        type=float, default=30.,
                    help="Max /ee_target publish rate [Hz] (default:30). "
                         "Rule: pub_hz ≤ 0.8*(1000/ik_ms). IK=20ms → use 30Hz.")
    ap.add_argument("--min-quality",   type=float, default=40.,
                    help="Min quality to allow publishing (default:40)")
    ap.add_argument("--coord-system",  choices=["pico","ros"], default="pico",
                    help="'pico'=Pico VR → ROS transform, 'ros'=already ROS frame")
    ap.add_argument("--debug",         action="store_true")
    ap.add_argument("--window",        type=float, default=10.)
    ap.add_argument("--fps",           type=int,   default=15)
    ap.add_argument("--save",          action="store_true")
    ap.add_argument("--output-dir",    default="")
    args = ap.parse_args()

    dual  = args.tracker == "both"
    sides = ["left","right"] if dual else [args.tracker]

    _now     = datetime.now()
    date_dir = _now.strftime("%y%m%d")
    time_sfx = _now.strftime("%m%d%H%M")

    states = {s: SideState(s, args.filter_hz, args.min_quality,
                           args.coord_system, debug=args.debug,
                           rpy_display_hz=args.rpy_disp_hz,
                           pub_hz=args.pub_hz)
              for s in sides}

    if args.save:
        out_dir = args.output_dir or os.path.join(_CSV_ROOT, date_dir)
        os.makedirs(out_dir, exist_ok=True)
        for s in sides:
            csv_p = os.path.join(out_dir, f"abs_{s}_{time_sfx}.csv")
            states[s].tracker_buf = ta.TrackerBuffer(maxlen=2000, csv_path=csv_p)
        print(f"  Saving CSV -> {out_dir}/")

    rclpy.init()
    node = TargetNode(states)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, ax_d, sb_txt = build_fig(sides)
    on_key, _pub, _status = make_key_handler(states, sb_txt, sides)
    fig.canvas.mpl_connect('key_press_event', on_key)

    def _upd(_f):
        for s in sides:
            _upd_side(states[s], ax_d[s], args.window)
        txt, col = _status()
        sb_txt.set_text(txt); sb_txt.set_color(col)

    ani = FuncAnimation(fig, _upd, interval=1000//args.fps,  # noqa: F841
                        blit=False, cache_frame_data=False)

    print(f"\n{'='*60}")
    print(f"  Tracker EE Absolute Publisher")
    print(f"  *** ABSOLUTE MODE — sends world-space pose to IK ***")
    print(f"  Sides      : {sides}")
    print(f"  Coord sys  : {args.coord_system}  {'(Pico→ROS transform ON)' if args.coord_system=='pico' else '(no transform)'}")
    print(f"  Pos filter : @{args.filter_hz:.0f}Hz causal IIR")
    print(f"  RPY disp   : @{args.rpy_disp_hz:.0f}Hz (display only — published pose uses main IIR)")
    print(f"  Pub rate   : {args.pub_hz:.0f}Hz  (IK budget: {1000/args.pub_hz:.0f}ms/call)")
    print(f"               Rule: pub_hz ≤ 0.8×(1000/ik_ms)  →  IK=20ms → use 40Hz")
    print(f"  Min quality: {args.min_quality}")
    print(f"  Publishes  : /ee_target/{{side}}  (geometry_msgs/PoseStamped)")
    print(f"               frame_id='world'  — ABSOLUTE XYZ[m] + quaternion")
    print(f"  IK usage   : subscribe /ee_target/{{side}}, use pose as target_pose")
    print(f"  CLICK FIGURE first, then:")
    print(f"    SPACE/Enter = toggle pause / publishing")
    print(f"    r           = reset LP filter state")
    print(f"    1/2/3/4     = pos+quat filter 5/10/15/20 Hz")
    print(f"    5/6/7/8     = display RPY filter 5/10/15/20 Hz")
    print(f"    q/Escape    = quit")
    print(f"{'='*60}\n")

    plt.show()
    for s in sides: states[s].tracker_buf.close()
    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
