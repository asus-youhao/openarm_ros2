#!/usr/bin/env python3
"""
即時繪製 EE task-space 軌跡（XYZ）。兩條連續 pose 串流（節點預設開）：

  /{arm}_eef_pose      waypoint(o)：有 delta 發 IK 目標；無 delta 發 FK(命令關節) hold
  /{arm}_eef_pose_fk   fk(x)：純 FK(命令關節)，不管有無 delta 一直發

  teleop 時兩者不同（目標 vs 實際命令 EE），可疊圖看 IK 追蹤殘差。

畫面：左欄 x(t)/y(t)/z(t) 滾動時間序列 + 右上 XY 俯視 + 右下 XYZ 3D 軌跡。

Usage:
  python3 plot_ee_target_realtime.py                          # right，waypoint+fk 疊圖
  python3 plot_ee_target_realtime.py --arm both               # 雙臂
  python3 plot_ee_target_realtime.py --channel fk             # 只畫 /{arm}_eef_pose_fk
  python3 plot_ee_target_realtime.py --arm both --window 20

  # 不開視窗、只在 terminal 印最新值（無 GUI 環境）:
  python3 plot_ee_target_realtime.py --no-plot
"""

import argparse
import collections
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

try:
    import matplotlib
    for _backend in ("TkAgg", "Qt5Agg", "GTK3Agg", "WXAgg"):
        try:
            matplotlib.use(_backend)
            break
        except Exception:
            continue
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False

ARMS_ALL = ("right", "left")
# 顏色依手臂；線型依 channel index（0=主：深實線，1=次：淺虛線）
ARM_COLOR = {"right": "#1f77b4", "left": "#d62728"}
CH_STYLE  = [("-", 1.9, 1.0), ("--", 1.2, 0.55)]   # (linestyle, lw, alpha)
CH_MARKER = ["o", "x"]                              # waypoint=o, fk=x

# channel_label → topic_template（第 0 個為 primary=waypoint）
CHANNELS = [("waypoint", "/{arm}_eef_pose"),
            ("fk",       "/{arm}_eef_pose_fk")]


class EeTargetPlotter(Node):
    def __init__(self, arms, channels, window_sec, source="eef"):
        super().__init__("plot_eef_pose_realtime")
        self.window_sec = float(window_sec)
        self.source = source
        self.ch_labels = [lbl for lbl, _ in channels]   # 有序，供線型索引
        self._lock = threading.Lock()
        self._t0 = None
        # series[(arm, label)] = deque of (t, x, y, z)
        self.series = {}
        maxlen = max(200, int(self.window_sec * 200))  # ~200 Hz headroom
        for arm in arms:
            for lbl, tmpl in channels:
                self.series[(arm, lbl)] = collections.deque(maxlen=maxlen)
                topic = tmpl.format(arm=arm)
                self.create_subscription(
                    PoseStamped, topic,
                    lambda msg, k=(arm, lbl): self._cb(k, msg), 20)
                self.get_logger().info(f"subscribing {topic}")

    def _cb(self, key, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            if self._t0 is None:
                self._t0 = t
            rel = t - self._t0
            p = msg.pose.position
            self.series[key].append((rel, p.x, p.y, p.z))

    def snapshot(self):
        """回傳 {key: (ts, xs, ys, zs)}，只保留 window 內的點。"""
        out = {}
        with self._lock:
            t_now = 0.0
            for dq in self.series.values():
                if dq:
                    t_now = max(t_now, dq[-1][0])
            t_min = t_now - self.window_sec
            for key, dq in self.series.items():
                pts = [r for r in dq if r[0] >= t_min]
                if pts:
                    ts, xs, ys, zs = zip(*pts)
                    out[key] = (list(ts), list(xs), list(ys), list(zs))
                else:
                    out[key] = ([], [], [], [])
        return out


def _label(arm, kind):
    return f"{arm}·{kind}"


def _set_3d_equal(ax, xs, ys, zs):
    """讓 3D 座標三軸等比例，軌跡不被拉扁。"""
    import numpy as np
    if not xs:
        return
    x = np.asarray(xs); y = np.asarray(ys); z = np.asarray(zs)
    cx, cy, cz = x.mean(), y.mean(), z.mean()
    r = max(np.ptp(x), np.ptp(y), np.ptp(z), 0.05) / 2.0 * 1.1
    ax.set_xlim(cx - r, cx + r)
    ax.set_ylim(cy - r, cy + r)
    ax.set_zlim(cz - r, cz + r)


def run_plot(node: EeTargetPlotter):
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d proj)

    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title("EEF pose trajectory (real-time)")
    # 左欄 3 列時間序列，右欄上 XY 俯視、右欄下 XYZ 3D
    gs = fig.add_gridspec(3, 2, width_ratios=[1.3, 1.0],
                          hspace=0.35, wspace=0.25)
    ax_x = fig.add_subplot(gs[0, 0])
    ax_y = fig.add_subplot(gs[1, 0], sharex=ax_x)
    ax_z = fig.add_subplot(gs[2, 0], sharex=ax_x)
    ax_xy = fig.add_subplot(gs[0, 1])
    ax_3d = fig.add_subplot(gs[1:, 1], projection="3d")

    for ax, name in ((ax_x, "X (m)"), (ax_y, "Y (m)"), (ax_z, "Z (m)")):
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
    ax_z.set_xlabel("t (s)")
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_title("XY 俯視軌跡")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_3d.set_title("XYZ 3D 軌跡")
    ax_3d.set_xlabel("X"); ax_3d.set_ylabel("Y"); ax_3d.set_zlabel("Z")

    lines = {}   # (key, axis) -> Line
    for key in node.series:
        arm, lbl = key
        c = ARM_COLOR.get(arm, None)
        ci = node.ch_labels.index(lbl) if lbl in node.ch_labels else 0
        ls, lw, alpha = CH_STYLE[ci % len(CH_STYLE)]
        for ax, idx in ((ax_x, 1), (ax_y, 2), (ax_z, 3)):
            (ln,) = ax.plot([], [], ls, color=c, lw=lw, alpha=alpha,
                            label=_label(arm, lbl))
            lines[(key, idx)] = ln
        mk = CH_MARKER[ci % len(CH_MARKER)]
        (ln_xy,) = ax_xy.plot([], [], ls, color=c, lw=lw, alpha=alpha,
                              label=_label(arm, lbl))
        lines[(key, "xy")] = ln_xy
        # XY 當下位置 marker（命令 o / 實際 x）—「兩個點」比較用
        (mk_xy,) = ax_xy.plot([], [], mk, color=c, ms=9, mew=2,
                              zorder=5)
        lines[(key, "xy_head")] = mk_xy
        (ln_3d,) = ax_3d.plot([], [], [], ls, color=c, lw=lw, alpha=alpha,
                              label=_label(arm, lbl))
        lines[(key, "3d")] = ln_3d
        # 3D 目前點標記（最新位置）
        (mk_3d,) = ax_3d.plot([], [], [], mk, color=c, ms=6, mew=2)
        lines[(key, "3d_head")] = mk_3d
    ax_x.legend(loc="upper left", fontsize=8, ncol=2)

    # 命令 vs 實際 誤差讀數（僅雙 channel 時有意義）
    err_txt = ax_xy.text(
        0.02, 0.98, "", transform=ax_xy.transAxes, va="top", ha="left",
        fontsize=8, family="monospace",
        bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.85))

    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            snap = node.snapshot()
            all_x, all_y, all_z = [], [], []
            last = {}   # key -> (x,y,z) 當下位置，算誤差用
            for key, (ts, xs, ys, zs) in snap.items():
                lines[(key, 1)].set_data(ts, xs)
                lines[(key, 2)].set_data(ts, ys)
                lines[(key, 3)].set_data(ts, zs)
                lines[(key, "xy")].set_data(xs, ys)
                ln3d = lines[(key, "3d")]
                ln3d.set_data(xs, ys)
                ln3d.set_3d_properties(zs)
                mk_xy = lines[(key, "xy_head")]
                head  = lines[(key, "3d_head")]
                if xs:
                    mk_xy.set_data([xs[-1]], [ys[-1]])
                    head.set_data([xs[-1]], [ys[-1]])
                    head.set_3d_properties([zs[-1]])
                    last[key] = (xs[-1], ys[-1], zs[-1])
                    all_x += xs; all_y += ys; all_z += zs
                else:
                    mk_xy.set_data([], [])
                    head.set_data([], [])
                    head.set_3d_properties([])
            # 每臂：primary(命令) vs secondary(實際) 3D 距離誤差
            err_lines = []
            if len(node.ch_labels) == 2:
                p_lbl, s_lbl = node.ch_labels
                for arm in sorted({a for a, _ in node.series}):
                    kp, ks = (arm, p_lbl), (arm, s_lbl)
                    if kp in last and ks in last:
                        d = sum((last[kp][i] - last[ks][i]) ** 2
                                for i in range(3)) ** 0.5
                        err_lines.append(f"{arm:5s} {p_lbl}↔{s_lbl}: {d*1000:6.1f} mm")
            err_txt.set_text("\n".join(err_lines))
            for ax in (ax_x, ax_y, ax_z, ax_xy):
                ax.relim()
                ax.autoscale_view()
            _set_3d_equal(ax_3d, all_x, all_y, all_z)
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close("all")


def run_headless(node: EeTargetPlotter):
    import time
    try:
        while rclpy.ok():
            snap = node.snapshot()
            parts = []
            for key in sorted(snap):
                ts, xs, ys, zs = snap[key]
                if xs:
                    parts.append(
                        f"{_label(*key)}=({xs[-1]:+.3f},{ys[-1]:+.3f},{zs[-1]:+.3f})")
            print("  " + "  ".join(parts) if parts else "  (waiting...)",
                  end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["right", "left", "both"], default="right")
    ap.add_argument("--channel", choices=["both", "waypoint", "fk"],
                    default="both",
                    help="both=waypoint(o)+fk(x)疊圖；waypoint=/{arm}_eef_pose；"
                         "fk=/{arm}_eef_pose_fk")
    ap.add_argument("--window", type=float, default=15.0,
                    help="時間序列滾動視窗秒數 (default 15)")
    ap.add_argument("--no-plot", action="store_true",
                    help="不開 GUI，只在 terminal 印最新值")
    args = ap.parse_args()

    arms = list(ARMS_ALL) if args.arm == "both" else [args.arm]
    if args.channel == "waypoint":
        channels = [CHANNELS[0]]
    elif args.channel == "fk":
        channels = [CHANNELS[1]]
    else:
        channels = CHANNELS

    rclpy.init()
    node = EeTargetPlotter(arms, channels, args.window)
    spin = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    try:
        if args.no_plot or not HAS_MPL:
            if not HAS_MPL and not args.no_plot:
                node.get_logger().warn("matplotlib 不可用 → headless 模式")
            run_headless(node)
        else:
            run_plot(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
