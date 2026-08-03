#!/usr/bin/env python3
"""
即時繪製 EE task-space 軌跡（XYZ）—— PySide6 + pyqtgraph 版（GPU 加速、順暢）。

訂閱 IK 節點連續廣播的兩條 pose 串流（PoseStamped，base_link frame）：
  /{arm}_eef_pose      target(o)：有 delta 發 IK 目標；無 delta 發 FK(命令關節) hold
  /{arm}_eef_pose_fk   fk(x)：純 FK(命令關節)，不管有無 delta 一直發

teleop 時兩者不同（目標 vs 實際命令 EE）；當下點以虛線相連，長度=追蹤殘差。

畫面：左欄 X/Y/Z 滾動時序 + 右上 XY 俯視 + 右下 XYZ 3D（可拖曳旋轉）。
左側控制列：開關 right/left、waypoint/fk、Pause、Clear、視窗秒數、Autoscale、Record CSV。

執行緒：rclpy 在背景 daemon thread 只寫 buffer；所有繪圖更新在 Qt 主執行緒的
QTimer callback（ROS thread 絕不碰 Qt 物件）。

Usage（建議在有 PySide6+pyqtgraph 的 env）：
  python3 plot_ee_target_realtime.py                     # right，waypoint+fk 疊圖
  python3 plot_ee_target_realtime.py --arm both
  python3 plot_ee_target_realtime.py --channel fk
  python3 plot_ee_target_realtime.py --no-plot           # 無 GUI，terminal 印最新值
"""

import argparse
import collections
import csv
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

ARMS_ALL = ("right", "left")
# 顏色依手臂（RGB 0-1，GL 與 2D 共用）
ARM_RGB = {"right": (0.12, 0.47, 0.71), "left": (0.84, 0.15, 0.16)}
# channel index 0=target（primary），1=fk（secondary）
CHANNELS = [("target", "/{arm}_eef_pose"),
            ("fk",     "/{arm}_eef_pose_fk")]


def _rgb255(arm, alpha=255):
    r, g, b = ARM_RGB[arm]
    return (int(r * 255), int(g * 255), int(b * 255), alpha)


# ── ROS 端：只收訂閱、寫 ring buffer ─────────────────────────────────────────
class RosBridge(Node):
    def __init__(self, arms, channels, window_sec):
        super().__init__("plot_eef_pose_qt")
        self.window_sec = float(window_sec)
        self.ch_labels = [lbl for lbl, _ in channels]
        self._lock = threading.Lock()
        self._t0 = None
        self.series = {}                       # (arm,label) -> deque[(t,x,y,z)]
        self._recv = {}                        # (arm,label) -> deque[wall] (Hz)
        maxlen = max(400, int(self.window_sec * 300))
        for arm in arms:
            for lbl, tmpl in channels:
                key = (arm, lbl)
                self.series[key] = collections.deque(maxlen=maxlen)
                self._recv[key] = collections.deque(maxlen=400)
                topic = tmpl.format(arm=arm)
                self.create_subscription(
                    PoseStamped, topic,
                    lambda msg, k=key: self._cb(k, msg), 20)
                self.get_logger().info(f"subscribing {topic}")

    def _cb(self, key, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall = time.time()
        with self._lock:
            if self._t0 is None:
                self._t0 = t
            p = msg.pose.position
            self.series[key].append((t - self._t0, p.x, p.y, p.z))
            self._recv[key].append(wall)

    def snapshot(self):
        """回傳 (series, hz)：series[key]=(ts,xs,ys,zs)（window 內）；hz[key]=近1秒筆數。"""
        out, hz = {}, {}
        now = time.time()
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
                hz[key] = sum(1 for w in self._recv[key] if now - w <= 1.0)
        return out, hz

    def clear(self):
        with self._lock:
            for dq in self.series.values():
                dq.clear()
            self._t0 = None


def _label(arm, lbl):
    return f"{arm}·{lbl}"


def _add_gl_axes(gl, glv):
    """在 GLViewWidget 畫帶數值刻度的 XYZ 軸線（base_link frame，單位 m）。

    軸線用灰色細線（不搶 right藍/left紅 軌跡），數值刻度灰字每 0.2 m 一格，
    軸末端放彩色 X/Y/Z 字母（X紅 Y綠 Z藍，標準座標色）。
    """
    # name: (unit vec, lo, hi, letter color 0-255)
    AX = {
        "X": (np.array([1., 0., 0.]),  0.0, 0.7, (210, 60, 60)),
        "Y": (np.array([0., 1., 0.]), -0.4, 0.4, (50, 160, 60)),
        "Z": (np.array([0., 0., 1.]),  0.0, 0.8, (70, 110, 220)),
    }
    tick_col = (110, 110, 110, 255)
    for name, (u, lo, hi, lcol) in AX.items():
        seg = np.array([u * lo, u * hi], dtype="f4")
        glv.addItem(gl.GLLinePlotItem(
            pos=seg, color=(0.45, 0.45, 0.45, 0.9), width=1.5, antialias=True))
        # 數值刻度（每 0.2 m；跳過原點避免三軸重疊）
        v = round(lo, 3)
        while v <= hi + 1e-6:
            if abs(v) > 1e-6:
                glv.addItem(gl.GLTextItem(
                    pos=tuple((u * v).tolist()), text=f"{v:.1f}", color=tick_col))
            v = round(v + 0.2, 3)
        # 軸末端字母
        glv.addItem(gl.GLTextItem(
            pos=tuple((u * hi * 1.08).tolist()), text=name, color=(*lcol, 255)))


# ── GUI ─────────────────────────────────────────────────────────────────────
def run_gui(bridge: RosBridge, arms, channels):
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
    import pyqtgraph.opengl as gl

    pg.setConfigOptions(antialias=True)
    pg.setConfigOption("background", "w")
    pg.setConfigOption("foreground", "k")

    app = pg.mkQApp("EEF pose")
    win = QtWidgets.QMainWindow()
    win.setWindowTitle("EEF pose trajectory (real-time)")

    central = QtWidgets.QWidget()
    win.setCentralWidget(central)
    root = QtWidgets.QHBoxLayout(central)

    # ── 控制列 ──────────────────────────────────────────────────────────────
    ctrl = QtWidgets.QVBoxLayout()
    ctrl.setSpacing(6)
    state = {"paused": False, "autoscale": True, "csv": None, "csv_writer": None}

    arm_chk = {}
    for a in arms:
        cb = QtWidgets.QCheckBox(a)
        cb.setChecked(True)
        arm_chk[a] = cb
        ctrl.addWidget(cb)
    ch_chk = {}
    for lbl, _ in channels:
        cb = QtWidgets.QCheckBox(lbl)
        cb.setChecked(True)
        ch_chk[lbl] = cb
        ctrl.addWidget(cb)

    ctrl.addWidget(_hline(QtWidgets))
    btn_pause = QtWidgets.QPushButton("Pause")
    btn_pause.setCheckable(True)
    btn_clear = QtWidgets.QPushButton("Clear")
    cb_auto = QtWidgets.QCheckBox("Autoscale")
    cb_auto.setChecked(True)
    btn_rec = QtWidgets.QPushButton("● Record CSV")
    btn_rec.setCheckable(True)
    ctrl.addWidget(btn_pause)
    ctrl.addWidget(btn_clear)
    ctrl.addWidget(cb_auto)
    ctrl.addWidget(btn_rec)

    ctrl.addWidget(QtWidgets.QLabel("window (s):"))
    win_spin = QtWidgets.QDoubleSpinBox()
    win_spin.setRange(1.0, 120.0)
    win_spin.setValue(bridge.window_sec)
    win_spin.setSingleStep(1.0)
    ctrl.addWidget(win_spin)

    status = QtWidgets.QLabel("")
    status.setStyleSheet("font-family: monospace; font-size: 11px;")
    status.setAlignment(QtCore.Qt.AlignTop)
    status.setWordWrap(True)
    ctrl.addWidget(status)
    ctrl.addStretch(1)

    ctrl_box = QtWidgets.QWidget()
    ctrl_box.setLayout(ctrl)
    ctrl_box.setFixedWidth(180)
    root.addWidget(ctrl_box)

    # ── 繪圖區 ──────────────────────────────────────────────────────────────
    grid = QtWidgets.QGridLayout()
    root.addLayout(grid, stretch=1)

    ax_x = pg.PlotWidget(title="X (m)")
    ax_y = pg.PlotWidget(title="Y (m)")
    ax_z = pg.PlotWidget(title="Z (m)")
    for pw in (ax_x, ax_y, ax_z):
        pw.showGrid(x=True, y=True, alpha=0.3)
        pw.setLabel("bottom", "t (s)")
    ax_y.setXLink(ax_x)
    ax_z.setXLink(ax_x)
    ax_xy = pg.PlotWidget(title="XY 俯視")
    ax_xy.showGrid(x=True, y=True, alpha=0.3)
    ax_xy.setAspectLocked(True)
    ax_xy.setLabel("bottom", "X (m)")
    ax_xy.setLabel("left", "Y (m)")

    glv = gl.GLViewWidget()
    glv.opts["center"] = pg.Vector(0.30, 0.0, 0.45)   # 繞 workspace 中心旋轉
    glv.setCameraPosition(distance=1.5, elevation=20, azimuth=-60)
    gr = gl.GLGridItem()
    gr.setSize(1.6, 1.0)
    gr.setSpacing(0.1, 0.1)
    glv.addItem(gr)
    _add_gl_axes(gl, glv)

    grid.addWidget(ax_x, 0, 0)
    grid.addWidget(ax_xy, 0, 1)
    grid.addWidget(ax_y, 1, 0)
    grid.addWidget(glv, 1, 1, 2, 1)
    grid.addWidget(ax_z, 2, 0)

    # ── 每個 (arm,channel) 建立線與當下點 ────────────────────────────────────
    lines = {}
    for key in bridge.series:
        arm, lbl = key
        ci = bridge.ch_labels.index(lbl) if lbl in bridge.ch_labels else 0
        width = 2.2 if ci == 0 else 1.4
        style = QtCore.Qt.SolidLine if ci == 0 else QtCore.Qt.DashLine
        sym = "o" if ci == 0 else "x"
        pen = pg.mkPen(color=_rgb255(arm), width=width, style=style)
        brush = pg.mkBrush(color=_rgb255(arm))
        name = _label(arm, lbl)
        for pw, k in ((ax_x, "x"), (ax_y, "y"), (ax_z, "z")):
            lines[(key, k)] = pw.plot([], [], pen=pen, name=name)
        lines[(key, "xy")] = ax_xy.plot([], [], pen=pen, name=name)
        lines[(key, "xy_head")] = pg.ScatterPlotItem(
            size=17, symbol=sym, pen=pg.mkPen(_rgb255(arm), width=2.5),
            brush=(brush if sym == "o" else None))
        ax_xy.addItem(lines[(key, "xy_head")])
        rgba = tuple(c / 255.0 for c in _rgb255(arm))
        ln3d = gl.GLLinePlotItem(pos=np.zeros((1, 3)), color=rgba,
                                 width=width, antialias=True)
        glv.addItem(ln3d)
        lines[(key, "3d")] = ln3d
        hd3d = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=rgba,
                                    size=16 if ci == 0 else 11)
        glv.addItem(hd3d)
        lines[(key, "3d_head")] = hd3d

    ax_x.addLegend(offset=(10, 10))

    # target↔fk 誤差連接線（當下點相連，長度=追蹤殘差），XY + 3D
    conn = {}
    if len(bridge.ch_labels) == 2:
        for arm in {a for a, _ in bridge.series}:
            conn[(arm, "xy")] = ax_xy.plot(
                [], [], pen=pg.mkPen((90, 90, 90), width=1.5,
                                     style=QtCore.Qt.DotLine))
            c3 = gl.GLLinePlotItem(pos=np.zeros((1, 3), "f4"),
                                   color=(0.35, 0.35, 0.35, 0.95),
                                   width=1.5, antialias=True)
            glv.addItem(c3)
            conn[(arm, "3d")] = c3

    # ── 控制事件 ─────────────────────────────────────────────────────────────
    def _visible(key):
        arm, lbl = key
        return arm_chk[arm].isChecked() and ch_chk[lbl].isChecked()

    def _apply_visibility():
        for key in bridge.series:
            vis = _visible(key)
            for k in ("x", "y", "z", "xy", "xy_head", "3d", "3d_head"):
                lines[(key, k)].setVisible(vis)
    for cb in list(arm_chk.values()) + list(ch_chk.values()):
        cb.stateChanged.connect(_apply_visibility)

    btn_pause.toggled.connect(
        lambda on: (state.__setitem__("paused", on),
                    btn_pause.setText("Resume" if on else "Pause")))
    btn_clear.clicked.connect(bridge.clear)
    cb_auto.toggled.connect(lambda on: state.__setitem__("autoscale", on))
    win_spin.valueChanged.connect(
        lambda v: setattr(bridge, "window_sec", float(v)))

    def _toggle_rec(on):
        if on:
            fn = time.strftime("eef_pose_%Y%m%d_%H%M%S.csv")
            f = open(fn, "w", newline="")
            w = csv.writer(f)
            hdr = ["t"]
            for key in bridge.series:
                a, l = key
                hdr += [f"{a}_{l}_x", f"{a}_{l}_y", f"{a}_{l}_z"]
            w.writerow(hdr)
            state["csv"], state["csv_writer"] = f, w
            btn_rec.setText(f"■ {fn}")
        else:
            if state["csv"]:
                state["csv"].close()
            state["csv"], state["csv_writer"] = None, None
            btn_rec.setText("● Record CSV")
    btn_rec.toggled.connect(_toggle_rec)

    # ── 週期更新（主執行緒）─────────────────────────────────────────────────
    def _update():
        if state["paused"]:
            return
        snap, hz = bridge.snapshot()
        last = {}
        for key, (ts, xs, ys, zs) in snap.items():
            lines[(key, "x")].setData(ts, xs)
            lines[(key, "y")].setData(ts, ys)
            lines[(key, "z")].setData(ts, zs)
            lines[(key, "xy")].setData(xs, ys)
            if xs:
                lines[(key, "xy_head")].setData([xs[-1]], [ys[-1]])
                pos = np.column_stack([xs, ys, zs]).astype(np.float32)
                lines[(key, "3d")].setData(pos=pos)
                lines[(key, "3d_head")].setData(
                    pos=np.array([[xs[-1], ys[-1], zs[-1]]], dtype=np.float32))
                last[key] = (xs[-1], ys[-1], zs[-1])
            else:
                lines[(key, "xy_head")].setData([], [])
                lines[(key, "3d")].setData(pos=np.zeros((1, 3), np.float32))
                lines[(key, "3d_head")].setData(pos=np.zeros((1, 3), np.float32))

        # target↔fk 誤差連接線
        for arm in {a for a, _ in bridge.series}:
            if (arm, "xy") not in conn:
                continue
            kt, kf = (arm, bridge.ch_labels[0]), (arm, bridge.ch_labels[1])
            show = (kt in last and kf in last and arm_chk[arm].isChecked()
                    and all(ch_chk[c].isChecked() for c in bridge.ch_labels))
            if show:
                conn[(arm, "xy")].setData([last[kt][0], last[kf][0]],
                                          [last[kt][1], last[kf][1]])
                conn[(arm, "3d")].setData(
                    pos=np.array([last[kt], last[kf]], "f4"))
            else:
                conn[(arm, "xy")].setData([], [])
                conn[(arm, "3d")].setData(pos=np.zeros((1, 3), "f4"))

        if state["autoscale"]:
            for pw in (ax_x, ax_y, ax_z, ax_xy):
                pw.enableAutoRange()

        # 狀態列：每臂 waypoint↔fk 誤差 + 各 series Hz + 當下 xyz
        txt = []
        if len(bridge.ch_labels) == 2:
            p, s = bridge.ch_labels
            for a in sorted({arm for arm, _ in bridge.series}):
                kp, ks = (a, p), (a, s)
                if kp in last and ks in last:
                    d = sum((last[kp][i] - last[ks][i]) ** 2
                            for i in range(3)) ** 0.5
                    txt.append(f"{a} {p}↔{s}: {d*1000:6.1f} mm")
        txt.append("")
        for key in bridge.series:
            a, l = key
            h = hz.get(key, 0)
            if key in last:
                x, y, z = last[key]
                txt.append(f"{a[:1]}·{l[:2]} {h:3d}Hz "
                           f"({x:+.3f},{y:+.3f},{z:+.3f})")
            else:
                txt.append(f"{a[:1]}·{l[:2]} {h:3d}Hz  --")
        status.setText("\n".join(txt))

        if state["csv_writer"] is not None:
            row = [f"{time.time():.3f}"]
            for key in bridge.series:
                if key in last:
                    row += [f"{v:.5f}" for v in last[key]]
                else:
                    row += ["", "", ""]
            state["csv_writer"].writerow(row)

    timer = QtCore.QTimer()
    timer.timeout.connect(_update)
    timer.start(30)   # ~33 FPS

    win.resize(1280, 800)
    win.show()
    app.exec()
    if state["csv"]:
        state["csv"].close()


# ── 無 GUI 文字模式（不依賴 Qt）──────────────────────────────────────────────
def run_headless(bridge: RosBridge):
    try:
        while rclpy.ok():
            snap, hz = bridge.snapshot()
            parts = []
            for key in sorted(snap):
                ts, xs, ys, zs = snap[key]
                if xs:
                    parts.append(f"{_label(*key)}=({xs[-1]:+.3f},"
                                 f"{ys[-1]:+.3f},{zs[-1]:+.3f})[{hz.get(key,0)}Hz]")
            print("  " + "  ".join(parts) if parts else "  (waiting...)",
                  end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def _hline(QtWidgets):
    ln = QtWidgets.QFrame()
    ln.setFrameShape(QtWidgets.QFrame.HLine)
    return ln


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["right", "left", "both"], default="right")
    ap.add_argument("--channel", choices=["both", "target", "fk"],
                    default="both",
                    help="both=target(o)+fk(x)疊圖；target=/{arm}_eef_pose；"
                         "fk=/{arm}_eef_pose_fk")
    ap.add_argument("--window", type=float, default=15.0,
                    help="時間序列滾動視窗秒數 (default 15)")
    ap.add_argument("--no-plot", action="store_true",
                    help="不開 GUI，只在 terminal 印最新值（不需 Qt）")
    args = ap.parse_args()

    arms = list(ARMS_ALL) if args.arm == "both" else [args.arm]
    if args.channel == "target":
        channels = [CHANNELS[0]]
    elif args.channel == "fk":
        channels = [CHANNELS[1]]
    else:
        channels = CHANNELS

    rclpy.init()
    bridge = RosBridge(arms, channels, args.window)
    spin = threading.Thread(target=lambda: rclpy.spin(bridge), daemon=True)
    spin.start()

    try:
        if args.no_plot:
            run_headless(bridge)
        else:
            try:
                run_gui(bridge, arms, channels)
            except ImportError as e:
                bridge.get_logger().error(
                    f"缺 GUI 套件（{e}）→ 改用 --no-plot 文字模式")
                run_headless(bridge)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
