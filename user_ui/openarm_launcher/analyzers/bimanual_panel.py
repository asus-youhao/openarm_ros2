"""PySide6 bimanual robot controller panel.

Ports the functionality of scripts/bimanual_gui_controller_hybrid.py
into the openarm_launcher UI as an embedded QGroupBox.

Control modes (selectable at runtime):
  action — FollowJointTrajectory action (joint_trajectory_controller)
  topic  — Float64MultiArray topic  (forward_position_controller)

A 50 ms QTimer drives change-detection and publishes only when a slider
value differs from the last sent value.
"""
from __future__ import annotations

import json
import math
import threading
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

# ---------------------------------------------------------------------------
# Joint metadata (mirrors bimanual_gui_controller_hybrid.py)
# ---------------------------------------------------------------------------

LEFT_ARM_JOINTS: list[str] = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_ARM_JOINTS: list[str] = [f"openarm_right_joint{i}" for i in range(1, 8)]

# O6 hand joints — must match o6_runner.py / o6_bimanual_*.py scripts
RIGHT_HAND_JOINTS: list[str] = [
    "R_thumb_cmc_pitch", "R_thumb_cmc_yaw",
    "R_index_mcp_pitch", "R_middle_mcp_pitch",
    "R_ring_mcp_pitch",  "R_pinky_mcp_pitch",
]
LEFT_HAND_JOINTS: list[str] = [
    "L_thumb_cmc_pitch", "L_thumb_cmc_yaw",
    "L_index_mcp_pitch", "L_middle_mcp_pitch",
    "L_ring_mcp_pitch",  "L_pinky_mcp_pitch",
]

JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "openarm_left_joint1": (-3.491, 1.396),
    "openarm_left_joint2": (-3.316, 0.175),
    "openarm_left_joint3": (-1.571, 1.571),
    "openarm_left_joint4": (0.0, 2.443),
    "openarm_left_joint5": (-1.571, 1.571),
    "openarm_left_joint6": (-0.785, 0.785),
    "openarm_left_joint7": (-1.571, 1.571),
    "openarm_right_joint1": (-1.396, 3.491),
    "openarm_right_joint2": (-0.175, 3.316),
    "openarm_right_joint3": (-1.571, 1.571),
    "openarm_right_joint4": (0.0, 2.443),
    "openarm_right_joint5": (-1.571, 1.571),
    "openarm_right_joint6": (-0.785, 0.785),
    "openarm_right_joint7": (-1.571, 1.571),
    # O6 right hand
    "R_thumb_cmc_pitch": (-0.5, 1.0),
    "R_thumb_cmc_yaw":   (0.0, 1.571),
    "R_index_mcp_pitch": (0.0, 1.571),
    "R_middle_mcp_pitch":(0.0, 1.571),
    "R_ring_mcp_pitch":  (0.0, 1.571),
    "R_pinky_mcp_pitch": (0.0, 1.571),
    # O6 left hand
    "L_thumb_cmc_pitch": (-0.5, 1.0),
    "L_thumb_cmc_yaw":   (0.0, 1.571),
    "L_index_mcp_pitch": (0.0, 1.571),
    "L_middle_mcp_pitch":(0.0, 1.571),
    "L_ring_mcp_pitch":  (0.0, 1.571),
    "L_pinky_mcp_pitch": (0.0, 1.571),
}

_SLIDER_SCALE = 1000  # int ticks per radian

# ---------------------------------------------------------------------------
# Hand preset configuration — loaded from JSON, with built-in fallback.
# Edit  data/presets/hand_presets.json  and click "Reload presets".
# ---------------------------------------------------------------------------
_PRESETS_PATH = (
    Path(__file__).resolve().parent.parent / "presets" / "hand_presets.json"
)
_DEFAULT_GRASP: dict[str, float] = {
    "R_thumb_cmc_pitch": 0.26, "R_thumb_cmc_yaw": 1.05,
    "R_index_mcp_pitch": 0.96, "R_middle_mcp_pitch": 0.96,
    "R_ring_mcp_pitch":  0.87, "R_pinky_mcp_pitch": 0.87,
    "L_thumb_cmc_pitch": 0.26, "L_thumb_cmc_yaw": 1.05,
    "L_index_mcp_pitch": 0.96, "L_middle_mcp_pitch": 0.96,
    "L_ring_mcp_pitch":  0.87, "L_pinky_mcp_pitch": 0.87,
}


def _load_hand_presets() -> dict[str, dict[str, float]]:
    """Return {"grasp": {...}, "open": {...}} from JSON, merged with defaults."""
    presets: dict[str, dict[str, float]] = {
        "grasp": dict(_DEFAULT_GRASP),
        "open": {j: 0.0 for j in RIGHT_HAND_JOINTS + LEFT_HAND_JOINTS},
    }
    try:
        with open(_PRESETS_PATH) as f:
            data = json.load(f)
        for key in ("grasp", "open"):
            if key in data and isinstance(data[key], dict):
                presets[key].update(
                    {k: float(v) for k, v in data[key].items()}
                )
    except FileNotFoundError:
        pass
    except Exception:  # noqa: BLE001
        pass
    return presets


# ---------------------------------------------------------------------------
# Per-joint slider row widget
# ---------------------------------------------------------------------------

class _JointSlider(QWidget):
    def __init__(self, joint_name: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        lo, hi = JOINT_LIMITS.get(joint_name, (-3.14159, 3.14159))

        row = QHBoxLayout(self)
        row.setContentsMargins(2, 1, 2, 1)
        row.setSpacing(4)

        short = (
            joint_name
            .replace("openarm_left_", "L")
            .replace("openarm_right_", "R")
            .replace("right_", "RH_")
        )
        lbl = QLabel(short)
        lbl.setFixedWidth(160)
        row.addWidget(lbl)

        lo_lbl = QLabel(f"{lo:.2f}r\n{math.degrees(lo):.0f}°")
        lo_lbl.setFixedWidth(52)
        lo_lbl.setStyleSheet("font-size: 8px; color: #666;")
        row.addWidget(lo_lbl)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(int(lo * _SLIDER_SCALE), int(hi * _SLIDER_SCALE))
        self._slider.setValue(0)
        self._slider.setFixedWidth(280)
        self._slider.valueChanged.connect(self._on_change)
        row.addWidget(self._slider)

        hi_lbl = QLabel(f"{hi:.2f}r\n{math.degrees(hi):.0f}°")
        hi_lbl.setFixedWidth(52)
        hi_lbl.setStyleSheet("font-size: 8px; color: #666;")
        row.addWidget(hi_lbl)

        self._val_lbl = QLabel("  0.000r  0.0°")
        self._val_lbl.setFixedWidth(100)
        self._val_lbl.setStyleSheet("font-weight: bold; font-size: 9px;")
        row.addWidget(self._val_lbl)

        zero_btn = QPushButton("0")
        zero_btn.setFixedWidth(28)
        zero_btn.clicked.connect(lambda: self._slider.setValue(0))
        row.addWidget(zero_btn)

    def _on_change(self, int_val: int) -> None:
        val = int_val / _SLIDER_SCALE
        self._val_lbl.setText(f"  {val:.3f}r  {math.degrees(val):.1f}°")

    def get_value(self) -> float:
        return self._slider.value() / _SLIDER_SCALE

    def set_value(self, rad: float) -> None:
        lo = self._slider.minimum() / _SLIDER_SCALE
        hi = self._slider.maximum() / _SLIDER_SCALE
        self._slider.setValue(int(max(lo, min(hi, rad)) * _SLIDER_SCALE))


def _make_tab(joint_names: list[str]) -> tuple[QScrollArea, dict[str, _JointSlider]]:
    """Return a scrollable tab widget and a {name: slider} dict."""
    container = QWidget()
    vbox = QVBoxLayout(container)
    vbox.setContentsMargins(4, 4, 4, 4)
    vbox.setSpacing(2)
    sliders: dict[str, _JointSlider] = {}
    for name in joint_names:
        s = _JointSlider(name)
        vbox.addWidget(s)
        sliders[name] = s
    vbox.addStretch()
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setWidget(container)
    return scroll, sliders


# ---------------------------------------------------------------------------
# ROS node (lazily imported so missing ROS doesn't crash the panel)
# ---------------------------------------------------------------------------

def _build_ros_node(mode: str):  # noqa: ANN201
    """Create a bimanual ROS node + executor + spin thread.

    Raises ImportError if rclpy or any required message package is absent.
    Returns (node, executor).
    """
    import rclpy  # noqa: F401  — ImportError propagates to caller
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from rclpy.action import ActionClient
    from .arm_runner import ensure_rclpy

    ensure_rclpy()

    class _BimanualNode(Node):
        def __init__(self) -> None:
            super().__init__("bimanual_controller_panel")
            self.joint_states: dict[str, float] = {}
            self.create_subscription(
                JointState, "/joint_states", self._on_js, 10
            )
            if mode == "topic":
                self._lpub = self.create_publisher(
                    Float64MultiArray,
                    "/left_forward_position_controller/commands", 10,
                )
                self._rpub = self.create_publisher(
                    Float64MultiArray,
                    "/right_forward_position_controller/commands", 10,
                )
                self._rhpub = self.create_publisher(
                    Float64MultiArray,
                    "/right_hand_forward_position_controller/commands", 10,
                )
                self._lhpub = self.create_publisher(
                    Float64MultiArray,
                    "/left_hand_forward_position_controller/commands", 10,
                )
            else:
                self._lclient = ActionClient(
                    self, FollowJointTrajectory,
                    "/left_joint_trajectory_controller/follow_joint_trajectory",
                )
                self._rclient = ActionClient(
                    self, FollowJointTrajectory,
                    "/right_joint_trajectory_controller/follow_joint_trajectory",
                )
                self._rhclient = ActionClient(
                    self, FollowJointTrajectory,
                    "/right_hand_controller/follow_joint_trajectory",
                )
                self._lhclient = ActionClient(
                    self, FollowJointTrajectory,
                    "/left_hand_controller/follow_joint_trajectory",
                )

        def _on_js(self, msg: JointState) -> None:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.joint_states[name] = msg.position[i]

        def publish(
            self,
            controller: str,
            joint_names: list[str],
            positions: list[float],
        ) -> None:
            _pub_map = {
                "left_arm":   getattr(self, "_lpub",   None),
                "right_arm":  getattr(self, "_rpub",   None),
                "right_hand": getattr(self, "_rhpub",  None),
                "left_hand":  getattr(self, "_lhpub",  None),
            }
            _client_map = {
                "left_arm":   getattr(self, "_lclient",  None),
                "right_arm":  getattr(self, "_rclient",  None),
                "right_hand": getattr(self, "_rhclient", None),
                "left_hand":  getattr(self, "_lhclient", None),
            }
            if mode == "topic":
                pub = _pub_map.get(controller)
                if pub is not None:
                    msg = Float64MultiArray()
                    msg.data = list(positions)
                    pub.publish(msg)
            else:
                client = _client_map.get(controller)
                if client is not None:
                    goal = FollowJointTrajectory.Goal()
                    goal.trajectory.joint_names = list(joint_names)
                    pt = JointTrajectoryPoint()
                    pt.positions = list(positions)
                    pt.time_from_start = Duration(sec=0, nanosec=500_000_000)
                    goal.trajectory.points = [pt]
                    client.send_goal_async(goal)

    node = _BimanualNode()
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)
    thread = threading.Thread(target=exec_.spin, daemon=True)
    thread.start()
    return node, exec_


# ---------------------------------------------------------------------------
# BimanualPanel
# ---------------------------------------------------------------------------

class BimanualPanel(QGroupBox):
    log_line = Signal(str)

    def __init__(self) -> None:
        super().__init__("Bimanual Controller")
        self._sliders: dict[str, _JointSlider] = {}
        self._is_sending = False
        self._ros_node = None
        self._ros_exec = None
        self._ros_mode: str | None = None
        self._last_sent: dict[str, float] = {}
        self._saved: list[dict] = []
        self._presets: dict[str, dict[str, float]] = _load_hand_presets()

        self._build_ui()

        self._send_timer = QTimer(self)
        self._send_timer.setInterval(50)
        self._send_timer.timeout.connect(self._send_tick)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        # --- control row ---
        ctrl = QHBoxLayout()
        ctrl.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["action", "topic"])
        ctrl.addWidget(self._mode_combo)

        self._start_btn = QPushButton("START")
        self._start_btn.setStyleSheet(
            "QPushButton { background: #4CAF50; color: white; font-weight: bold; }"
        )
        self._start_btn.clicked.connect(self._start_sending)
        ctrl.addWidget(self._start_btn)

        self._stop_btn = QPushButton("STOP")
        self._stop_btn.setStyleSheet(
            "QPushButton { background: #f44336; color: white; font-weight: bold; }"
        )
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._stop_sending)
        ctrl.addWidget(self._stop_btn)

        self._status_lbl = QLabel("● STOPPED")
        self._status_lbl.setStyleSheet("color: #f44336; font-weight: bold;")
        ctrl.addWidget(self._status_lbl)
        ctrl.addStretch()
        layout.addLayout(ctrl)

        # --- preset row ---
        pre = QHBoxLayout()
        for text, fn in [
            ("Home (all zero)", self._preset_home),
            ("Grasp hand", self._preset_grasp),
            ("Open hand", self._preset_open_hand),
            ("Reload presets", self._reload_presets),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(fn)
            pre.addWidget(btn)
        pre.addStretch()
        layout.addLayout(pre)

        # --- tabs ---
        tabs = QTabWidget()
        for tab_label, joints in [
            ("Left Arm",       LEFT_ARM_JOINTS),
            ("Right Arm",      RIGHT_ARM_JOINTS),
            ("Right Hand (O6)", RIGHT_HAND_JOINTS),
            ("Left Hand (O6)", LEFT_HAND_JOINTS),
        ]:
            scroll, sliders = _make_tab(joints)
            self._sliders.update(sliders)
            tabs.addTab(scroll, tab_label)
        layout.addWidget(tabs)

        # --- recording row ---
        rec = QHBoxLayout()
        rec.addWidget(QLabel("Name:"))
        self._pos_name = QLineEdit("Position 1")
        self._pos_name.setFixedWidth(110)
        rec.addWidget(self._pos_name)

        for text, fn in [
            ("Save pos", self._save_position),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(fn)
            rec.addWidget(btn)

        self._count_lbl = QLabel("Saved: 0")
        rec.addWidget(self._count_lbl)

        for text, fn in [
            ("Export JSON", self._export_json),
            ("Load JSON", self._load_json),
            ("Clear", self._clear_positions),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(fn)
            rec.addWidget(btn)

        rec.addStretch()
        layout.addLayout(rec)

    # ------------------------------------------------------------------
    # ROS lifecycle
    # ------------------------------------------------------------------

    def _ensure_ros(self) -> bool:
        mode = self._mode_combo.currentText()
        if self._ros_node is not None and self._ros_mode == mode:
            return True
        # Mode changed or first call — tear down any existing node.
        self._teardown_ros()
        try:
            self._ros_node, self._ros_exec = _build_ros_node(mode)
            self._ros_mode = mode
            self.log_line.emit(f"[bimanual] ROS node ready ({mode} mode)")
            return True
        except Exception as exc:  # noqa: BLE001
            self.log_line.emit(f"[bimanual] ROS unavailable: {exc}")
            self._ros_mode = None
            return False

    def _teardown_ros(self) -> None:
        if self._ros_exec is not None:
            try:
                self._ros_exec.shutdown()
            except Exception:  # noqa: BLE001
                pass
            self._ros_exec = None
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
            self._ros_node = None
        self._ros_mode = None

    def set_mode(self, mode: str) -> None:
        """Sync mode combo to match the launcher controller (action/topic).
        Only applies when not currently sending.
        """
        if not self._is_sending:
            idx = self._mode_combo.findText(mode)
            if idx >= 0:
                self._mode_combo.setCurrentIndex(idx)

    def shutdown(self) -> None:
        if self._is_sending:
            self._stop_sending()
        self._teardown_ros()

    # ------------------------------------------------------------------
    # START / STOP
    # ------------------------------------------------------------------

    def _start_sending(self) -> None:
        if not self._ensure_ros():  
            return
        # Sync sliders to the robot's current joint positions before sending
        # anything, so the first command matches where the robot already is.
        self._sync_sliders_from_joint_states()
        self._is_sending = True
        self._last_sent.clear()
        self._send_timer.start()
        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._mode_combo.setEnabled(False)
        self._status_lbl.setText("● ACTIVE")
        self._status_lbl.setStyleSheet("color: #4CAF50; font-weight: bold;")
        self.log_line.emit(
            f"[bimanual] sending started ({self._mode_combo.currentText()} mode)"
        )

    def _sync_sliders_from_joint_states(self) -> None:
        """Read the latest /joint_states from the ROS node and move every
        slider to match.  Silently skips joints not yet in the message."""
        if self._ros_node is None:
            return
        js = dict(self._ros_node.joint_states)  # snapshot
        if not js:
            self.log_line.emit(
                "[bimanual] no joint_states received yet — sliders stay at current position"
            )
            return
        synced = 0
        for joint_name, slider in self._sliders.items():
            if joint_name in js:
                slider.set_value(js[joint_name])
                synced += 1
        self.log_line.emit(f"[bimanual] synced {synced} sliders from joint_states")

    def _stop_sending(self) -> None:
        if not self._is_sending:
            return
        self._is_sending = False
        self._send_timer.stop()
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._mode_combo.setEnabled(True)
        self._status_lbl.setText("● STOPPED")
        self._status_lbl.setStyleSheet("color: #f44336; font-weight: bold;")
        self.log_line.emit("[bimanual] sending stopped")

    # ------------------------------------------------------------------
    # 50 ms tick — detect changes and publish
    # ------------------------------------------------------------------

    def _send_tick(self) -> None:
        if not self._is_sending or self._ros_node is None:
            return
        for ctrl, joints in [
            ("left_arm",   LEFT_ARM_JOINTS),
            ("right_arm",  RIGHT_ARM_JOINTS),
            ("right_hand", RIGHT_HAND_JOINTS),
            ("left_hand",  LEFT_HAND_JOINTS),
        ]:
            changed = any(
                self._sliders[j].get_value() != self._last_sent.get(j)
                for j in joints
            )
            if changed:
                positions = [self._sliders[j].get_value() for j in joints]
                for j, v in zip(joints, positions):
                    self._last_sent[j] = v
                try:
                    self._ros_node.publish(ctrl, joints, positions)
                except Exception as exc:  # noqa: BLE001
                    self.log_line.emit(f"[bimanual] publish error: {exc}")

    # ------------------------------------------------------------------
    # Presets
    # ------------------------------------------------------------------

    def _preset_home(self) -> None:
        for s in self._sliders.values():
            s.set_value(0.0)

    def _preset_grasp(self) -> None:
        for name, val in self._presets["grasp"].items():
            if name in self._sliders:
                self._sliders[name].set_value(val)

    def _preset_open_hand(self) -> None:
        for name, val in self._presets["open"].items():
            if name in self._sliders:
                self._sliders[name].set_value(val)

    def _reload_presets(self) -> None:
        self._presets = _load_hand_presets()
        self.log_line.emit(f"[bimanual] presets reloaded from {_PRESETS_PATH}")

    # ------------------------------------------------------------------
    # Position recording
    # ------------------------------------------------------------------

    def _save_position(self) -> None:
        import datetime
        pos = {
            "name": self._pos_name.text(),
            "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "left_arm":   [self._sliders[j].get_value() for j in LEFT_ARM_JOINTS],
            "right_arm":  [self._sliders[j].get_value() for j in RIGHT_ARM_JOINTS],
            "right_hand": [self._sliders[j].get_value() for j in RIGHT_HAND_JOINTS],
            "left_hand":  [self._sliders[j].get_value() for j in LEFT_HAND_JOINTS],
        }
        self._saved.append(pos)
        self._count_lbl.setText(f"Saved: {len(self._saved)}")
        # Auto-increment name suffix
        parts = self._pos_name.text().rsplit(" ", 1)
        if len(parts) == 2 and parts[1].isdigit():
            self._pos_name.setText(f"{parts[0]} {int(parts[1]) + 1}")
        self.log_line.emit(f"[bimanual] saved '{pos['name']}'")

    def _export_json(self) -> None:
        if not self._saved:
            self.log_line.emit("[bimanual] nothing to export")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export positions",
            str(Path.home() / "bimanual_positions.json"),
            "JSON (*.json)",
        )
        if path:
            with open(path, "w") as f:
                json.dump(self._saved, f, indent=2)
            self.log_line.emit(
                f"[bimanual] exported {len(self._saved)} positions → {path}"
            )

    def _load_json(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Load positions", str(Path.home()), "JSON (*.json)"
        )
        if path:
            try:
                with open(path) as f:
                    self._saved = json.load(f)
                self._count_lbl.setText(f"Saved: {len(self._saved)}")
                self.log_line.emit(
                    f"[bimanual] loaded {len(self._saved)} positions from {path}"
                )
            except Exception as exc:  # noqa: BLE001
                self.log_line.emit(f"[bimanual] load error: {exc}")

    def _clear_positions(self) -> None:
        self._saved.clear()
        self._count_lbl.setText("Saved: 0")
        self.log_line.emit("[bimanual] positions cleared")
