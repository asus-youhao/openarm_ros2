"""Record & Replay panel.

Record section
--------------
  action mode — subscribes to /joint_actions  (commanded JointState from
                the bimanual controller)
  topic  mode — subscribes to /joint_states   (actual joint feedback)

  Both modes write a CSV that is directly compatible with the replay scripts:
    stamp_sec, stamp_nanosec, topic, position
  where  topic = /follower_joint_states          → right arm  (7 joints)
                 /left_follower_joint_states      → left arm   (7 joints)
                 /right_o6hand_joint_states       → right hand (6 joints)
                 /left_o6hand_joint_states        → left hand  (6 joints)

Replay section
--------------
  Controller selection maps to the replay script:
    Forward Position  →  replay_follower_joint_bimanual_topic.py
    Joint Trajectory  →  replay_follower_joint_bimanual_action.py

  The replay is launched in an external terminal window (same pattern as
  Ros2Launcher) so it is decoupled from the Qt process.
"""
from __future__ import annotations

import csv
import datetime
import os
import shlex
import shutil
import subprocess
import tempfile
import threading
from pathlib import Path

from PySide6.QtCore import QObject, QSettings, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
)

# ---------------------------------------------------------------------------
# Paths & joint constants
# ---------------------------------------------------------------------------

_WORKSPACE = Path(__file__).resolve().parent.parent.parent
DATA_ROOT = _WORKSPACE / "data" / "record"

RIGHT_ARM_JOINTS: list[str] = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_ARM_JOINTS: list[str] = [f"openarm_left_joint{i}" for i in range(1, 8)]

O6HAND_JOINTS_RIGHT: list[str] = [
    "R_thumb_cmc_yaw",
    "R_thumb_cmc_pitch",
    "R_index_mcp_pitch",
    "R_middle_mcp_pitch",
    "R_ring_mcp_pitch",
    "R_pinky_mcp_pitch",
]
O6HAND_JOINTS_LEFT: list[str] = [
    "L_thumb_cmc_yaw",
    "L_thumb_cmc_pitch",
    "L_index_mcp_pitch",
    "L_middle_mcp_pitch",
    "L_ring_mcp_pitch",
    "L_pinky_mcp_pitch",
]

# ROS topic names written into the CSV (must match what the replay scripts expect)
_RIGHT_CSV_TOPIC       = "/follower_joint_states"
_LEFT_CSV_TOPIC        = "/left_follower_joint_states"
_RIGHT_HAND_CSV_TOPIC  = "/right_o6hand_joint_states"
_LEFT_HAND_CSV_TOPIC   = "/left_o6hand_joint_states"

# Controller label → replay mode
_CONTROLLER_MODE: dict[str, str] = {
    "Forward Position": "topic",
    "Joint Trajectory": "action",
}

# Preference order for terminal emulators (mirrors ros2_launcher.py)
_TERMINALS = ["gnome-terminal", "xterm", "xfce4-terminal", "konsole", "tilix"]


def _find_terminal() -> str | None:
    for t in _TERMINALS:
        if shutil.which(t):
            return t
    return None


def _terminal_argv(terminal: str, title: str, cmd: str) -> list[str]:
    if terminal == "gnome-terminal":
        return [terminal, f"--title={title}", "--", "bash", "-c", cmd]
    if terminal == "konsole":
        return [terminal, "-p", f"tabtitle={title}", "-e", "bash", "-c", cmd]
    if terminal in ("xfce4-terminal", "tilix"):
        return [terminal, f"--title={title}", "-e", f"bash -c '{cmd}'"]
    # xterm fallback
    return [terminal, "-title", title, "-e", "bash", "-c", cmd]


# ---------------------------------------------------------------------------
# Record CSV writer
# ---------------------------------------------------------------------------

class _RecordCsvWriter:
    """Writes one row per arm segment per JointState message.

    Row format:  stamp_sec, stamp_nanosec, topic, position
    where  position  is a comma-joined string of 7 floats (one per joint).
    """

    def __init__(self, mode: str) -> None:
        today = datetime.date.today().isoformat()
        ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        out_dir = DATA_ROOT / "csv" / today
        out_dir.mkdir(parents=True, exist_ok=True)
        self.path = out_dir / f"record_{mode}_{ts}.csv"
        self._fh = open(self.path, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(["stamp_sec", "stamp_nanosec", "topic", "position"])
        self._count = 0

    def write(self, sec: int, nsec: int, topic: str, positions: list[float]) -> None:
        self._writer.writerow(
            [sec, nsec, topic, ",".join(f"{p:.6f}" for p in positions)]
        )
        self._count += 1

    @property
    def row_count(self) -> int:
        return self._count

    def flush_close(self) -> None:
        self._fh.flush()
        self._fh.close()


# ---------------------------------------------------------------------------
# Record runner — ROS2 subscriber in a background thread
# ---------------------------------------------------------------------------

class _RecordRunner(QObject):
    """Subscribes to /joint_actions or /joint_states and writes CSV rows."""

    row_written = Signal(int)   # emits total row count after each write burst
    stopped = Signal()

    def __init__(
        self,
        mode: str,
        csv_w: _RecordCsvWriter,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._mode = mode
        self._csv_w = csv_w
        self._stop_event = threading.Event()
        self._node = None
        self._exec = None

    def start(self) -> None:
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def stop(self) -> None:
        self._stop_event.set()

    # ------------------------------------------------------------------
    def _run(self) -> None:
        try:
            from .arm_runner import ensure_rclpy
            ensure_rclpy()

            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from sensor_msgs.msg import JointState

            mode = self._mode
            csv_w = self._csv_w
            sig = self.row_written
            stop_ev = self._stop_event
            sub_topic = "/joint_actions" if mode == "action" else "/joint_states"

            class _RecorderNode(Node):
                def __init__(self_n) -> None:  # noqa: N805
                    super().__init__("openarm_record_runner")
                    self_n.create_subscription(
                        JointState, sub_topic, self_n._on_msg, 10
                    )

                def _on_msg(self_n, msg: JointState) -> None:  # noqa: N805
                    sec = msg.header.stamp.sec
                    nsec = msg.header.stamp.nanosec
                    names = list(msg.name)
                    pos = list(msg.position)

                    def _extract(joints: list[str]) -> list[float]:
                        result: list[float] = []
                        for jn in joints:
                            if jn in names:
                                i = names.index(jn)
                                result.append(pos[i] if i < len(pos) else 0.0)
                            else:
                                result.append(0.0)
                        return result

                    right_pos      = _extract(RIGHT_ARM_JOINTS)
                    left_pos       = _extract(LEFT_ARM_JOINTS)
                    right_hand_pos = _extract(O6HAND_JOINTS_RIGHT)
                    left_hand_pos  = _extract(O6HAND_JOINTS_LEFT)

                    if any(x != 0.0 for x in right_pos):
                        csv_w.write(sec, nsec, _RIGHT_CSV_TOPIC, right_pos)
                    if any(x != 0.0 for x in left_pos):
                        csv_w.write(sec, nsec, _LEFT_CSV_TOPIC, left_pos)
                    if any(x != 0.0 for x in right_hand_pos):
                        csv_w.write(sec, nsec, _RIGHT_HAND_CSV_TOPIC, right_hand_pos)
                    if any(x != 0.0 for x in left_hand_pos):
                        csv_w.write(sec, nsec, _LEFT_HAND_CSV_TOPIC, left_hand_pos)

                    sig.emit(csv_w.row_count)

            self._node = _RecorderNode()
            self._exec = SingleThreadedExecutor()
            self._exec.add_node(self._node)
            while not stop_ev.is_set():
                self._exec.spin_once(timeout_sec=0.05)

        except Exception:  # noqa: BLE001
            pass
        finally:
            e, n = self._exec, self._node
            self._exec, self._node = None, None
            if e is not None:
                try:
                    e.shutdown()
                except Exception:  # noqa: BLE001
                    pass
            if n is not None:
                try:
                    n.destroy_node()
                except Exception:  # noqa: BLE001
                    pass
            self.stopped.emit()


# ---------------------------------------------------------------------------
# RecordReplayPanel
# ---------------------------------------------------------------------------

class RecordReplayPanel(QGroupBox):
    log_line = Signal(str)

    def __init__(self) -> None:
        super().__init__("Record & Replay")
        self._runner: _RecordRunner | None = None
        self._csv_w: _RecordCsvWriter | None = None
        self._terminal: str | None = _find_terminal()
        self._settings = QSettings()
        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(4)

        # ── Record ────────────────────────────────────────────────────
        root.addWidget(QLabel("<b>Record</b>"))

        rec_mode_row = QHBoxLayout()
        rec_mode_row.addWidget(QLabel("Mode:"))
        self._rec_mode = QComboBox()
        self._rec_mode.addItems(["action", "topic"])
        self._rec_mode.setToolTip(
            "action — records /joint_actions  (commanded joint positions)\n"
            "topic  — records /joint_states   (actual joint feedback)"
        )
        _restore_combo(self._settings, "record_replay/rec_mode", self._rec_mode)
        self._rec_mode.currentTextChanged.connect(
            lambda v: self._settings.setValue("record_replay/rec_mode", v)
        )
        rec_mode_row.addWidget(self._rec_mode)
        rec_mode_row.addStretch()
        root.addLayout(rec_mode_row)

        rec_btn_row = QHBoxLayout()
        self._rec_start = QPushButton("Start Record")
        self._rec_start.setStyleSheet(
            "QPushButton { background: #4CAF50; color: white; font-weight: bold; }"
        )
        self._rec_start.clicked.connect(self._start_record)
        self._rec_stop = QPushButton("Stop Record")
        self._rec_stop.setStyleSheet(
            "QPushButton { background: #f44336; color: white; font-weight: bold; }"
        )
        self._rec_stop.setEnabled(False)
        self._rec_stop.clicked.connect(self._stop_record)
        self._rec_status = QLabel("● STOPPED")
        self._rec_status.setStyleSheet("color: #f44336; font-weight: bold;")
        rec_btn_row.addWidget(self._rec_start)
        rec_btn_row.addWidget(self._rec_stop)
        rec_btn_row.addWidget(self._rec_status)
        rec_btn_row.addStretch()
        root.addLayout(rec_btn_row)

        # ── Replay ────────────────────────────────────────────────────
        root.addSpacing(8)
        root.addWidget(QLabel("<b>Replay</b>"))

        csv_row = QHBoxLayout()
        csv_row.addWidget(QLabel("CSV:"))
        self._csv_path_edit = QLineEdit()
        self._csv_path_edit.setPlaceholderText("Browse or type absolute path to CSV…")
        _restore_text(self._settings, "record_replay/csv_path", self._csv_path_edit)
        self._csv_path_edit.textChanged.connect(
            lambda v: self._settings.setValue("record_replay/csv_path", v)
        )
        csv_row.addWidget(self._csv_path_edit)
        browse_btn = QPushButton("Browse…")
        browse_btn.clicked.connect(self._browse_csv)
        csv_row.addWidget(browse_btn)
        root.addLayout(csv_row)

        ctrl_row = QHBoxLayout()
        ctrl_row.addWidget(QLabel("Controller:"))
        self._ctrl_combo = QComboBox()
        self._ctrl_combo.addItems(list(_CONTROLLER_MODE.keys()))
        self._ctrl_combo.setToolTip(
            "Forward Position  → forward_position_controller  (topic replay)\n"
            "Joint Trajectory  → joint_trajectory_controller  (action replay)"
        )
        _restore_combo(self._settings, "record_replay/controller", self._ctrl_combo)
        self._ctrl_combo.currentTextChanged.connect(
            lambda v: self._settings.setValue("record_replay/controller", v)
        )
        ctrl_row.addWidget(self._ctrl_combo)
        ctrl_row.addSpacing(12)
        ctrl_row.addWidget(QLabel("Speed:"))
        self._speed_combo = QComboBox()
        self._speed_combo.addItems(["1x", "2x", "4x"])
        ctrl_row.addWidget(self._speed_combo)
        ctrl_row.addStretch()
        root.addLayout(ctrl_row)

        replay_btn_row = QHBoxLayout()
        self._replay_btn = QPushButton("Start Replay")
        self._replay_btn.setStyleSheet(
            "QPushButton { background: #2196F3; color: white; font-weight: bold; }"
        )
        self._replay_btn.clicked.connect(self._start_replay)
        replay_btn_row.addWidget(self._replay_btn)
        replay_btn_row.addStretch()
        root.addLayout(replay_btn_row)

        root.addStretch()

    # ------------------------------------------------------------------
    # Record logic
    # ------------------------------------------------------------------

    def _start_record(self) -> None:
        if self._runner is not None:
            return
        mode = self._rec_mode.currentText()
        try:
            self._csv_w = _RecordCsvWriter(mode)
        except Exception as exc:  # noqa: BLE001
            self.log_line.emit(f"[record error] {exc}")
            return

        self._runner = _RecordRunner(mode, self._csv_w, self)
        self._runner.row_written.connect(self._on_row_written)
        self._runner.stopped.connect(self._on_runner_stopped)
        self._runner.start()

        self._rec_start.setEnabled(False)
        self._rec_mode.setEnabled(False)
        self._rec_stop.setEnabled(True)
        self._rec_status.setText("● RECORDING  0")
        self._rec_status.setStyleSheet("color: #4CAF50; font-weight: bold;")
        self.log_line.emit(f"--- record started ({mode}) → {self._csv_w.path} ---")

    def _on_row_written(self, count: int) -> None:
        self._rec_status.setText(f"● RECORDING  {count}")

    def _stop_record(self) -> None:
        if self._runner is None:
            return
        self._runner.stop()
        self._rec_stop.setEnabled(False)
        self._rec_status.setText("● STOPPING…")
        self._rec_status.setStyleSheet("color: #FF9800; font-weight: bold;")

    def _on_runner_stopped(self) -> None:
        if self._csv_w is not None:
            try:
                self._csv_w.flush_close()
                saved_path = str(self._csv_w.path)
                self.log_line.emit(f"[saved] {saved_path}")
                # Pre-fill CSV path field for immediate replay
                self._csv_path_edit.setText(saved_path)
            except Exception:  # noqa: BLE001
                pass
            finally:
                self._csv_w = None
        self._runner = None
        self._rec_start.setEnabled(True)
        self._rec_mode.setEnabled(True)
        self._rec_stop.setEnabled(False)
        self._rec_status.setText("● STOPPED")
        self._rec_status.setStyleSheet("color: #f44336; font-weight: bold;")

    # ------------------------------------------------------------------
    # Replay logic
    # ------------------------------------------------------------------

    def _browse_csv(self) -> None:
        start_dir = str(DATA_ROOT / "csv") if (DATA_ROOT / "csv").exists() else str(DATA_ROOT.parent)
        path, _ = QFileDialog.getOpenFileName(
            self, "Select CSV file", start_dir, "CSV files (*.csv)"
        )
        if path:
            self._csv_path_edit.setText(path)

    def _start_replay(self) -> None:
        csv_path = self._csv_path_edit.text().strip()
        if not csv_path:
            self.log_line.emit("[replay error] no CSV file selected.")
            return
        if not Path(csv_path).is_file():
            self.log_line.emit(f"[replay error] file not found: {csv_path}")
            return
        if self._terminal is None:
            self.log_line.emit(
                "[replay error] no terminal emulator found; tried: "
                + ", ".join(_TERMINALS)
            )
            return

        controller_label = self._ctrl_combo.currentText()
        replay_mode = _CONTROLLER_MODE[controller_label]
        speed = float(self._speed_combo.currentText().rstrip("x"))

        if replay_mode == "topic":
            module = "replay_follower_joint_bimanual_topic"
            cls = "BimanualTopicReplayer"
        else:
            module = "replay_follower_joint_bimanual_action"
            cls = "BimanualActionReplayer"

        # Build a self-contained Python script and write it to a temp file.
        # Using a temp file avoids all shell-quoting complexity with paths that
        # may contain spaces or special characters.
        ws_repr = repr(str(_WORKSPACE))
        csv_repr = repr(csv_path)
        py_lines = [
            "import sys",
            f"sys.path.insert(0, {ws_repr})",
            "import rclpy",
            f"from {module} import {cls}",
            "rclpy.init()",
            f"_node = {cls}({csv_repr})",
            "try:",
            f"    _node.replay(speed={speed})",
            "finally:",
            "    _node.destroy_node()",
            "    rclpy.shutdown()",
            "input('\\nReplay finished. Press Enter to close…')",
        ]

        fd, tmp_path = tempfile.mkstemp(suffix=".py", prefix="openarm_replay_")
        try:
            with os.fdopen(fd, "w") as fh:
                fh.write("\n".join(py_lines) + "\n")
        except Exception as exc:  # noqa: BLE001
            self.log_line.emit(f"[replay error] could not write temp script: {exc}")
            return

        tmp_quoted = shlex.quote(tmp_path)
        cmd_str = f"python3 {tmp_quoted}; rm -f {tmp_quoted}"
        title = f"replay — {controller_label} — {Path(csv_path).name}"
        argv = _terminal_argv(self._terminal, title, cmd_str)
        try:
            subprocess.Popen(argv, start_new_session=True)
            self.log_line.emit(
                f"[replay] launched {replay_mode} replay in terminal "
                f"({speed}x) — {Path(csv_path).name}"
            )
        except Exception as exc:  # noqa: BLE001
            self.log_line.emit(f"[replay error] could not open terminal: {exc}")

    # ------------------------------------------------------------------
    # External sync helpers
    # ------------------------------------------------------------------

    def set_replay_mode(self, mode: str) -> None:
        """Sync controller combo with the launcher's controller selection.

        Called from MainWindow when the controller changes so that the replay
        controller stays in sync automatically.
        """
        label_map = {v: k for k, v in _CONTROLLER_MODE.items()}
        label = label_map.get(mode)
        if label:
            idx = self._ctrl_combo.findText(label)
            if idx >= 0:
                self._ctrl_combo.setCurrentIndex(idx)

    def shutdown(self) -> None:
        """Stop the record runner on application close."""
        if self._runner is not None:
            self._runner.stop()


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def _restore_combo(settings: QSettings, key: str, combo: QComboBox) -> None:
    saved = settings.value(key, "")
    if saved:
        idx = combo.findText(str(saved))
        if idx >= 0:
            combo.setCurrentIndex(idx)


def _restore_text(settings: QSettings, key: str, edit: QLineEdit) -> None:
    saved = settings.value(key, "")
    if saved:
        edit.setText(str(saved))
