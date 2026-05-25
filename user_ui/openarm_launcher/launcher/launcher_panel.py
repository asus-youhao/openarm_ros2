from PySide6.QtCore import QSettings, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)

from .can_detect import CAN_INTERFACES, CanDetector
from .can_setup import CanBringUp
from .ros2_launcher import CONTROLLERS, Ros2Launcher


_LED_COLORS = {"off": "#444", "down": "#cc9900", "up": "#33cc66"}

_ROLES: dict[str, str] = {
    "can0": "O6 right",
    "can1": "O6 left",
    "can2": "OpenArm right (CAN-FD)",
    "can3": "OpenArm left (CAN-FD)",
}


class StatusLed(QFrame):
    def __init__(self, parent: QFrame | None = None) -> None:
        super().__init__(parent)
        self.setFixedSize(14, 14)
        self.set_state("off")

    def set_state(self, state: str) -> None:
        c = _LED_COLORS.get(state, _LED_COLORS["off"])
        self.setStyleSheet(
            f"background-color: {c}; border-radius: 7px; border: 1px solid #222;"
        )


class LauncherPanel(QGroupBox):
    log_line = Signal(str)

    def __init__(self) -> None:
        super().__init__("Launcher")
        layout = QVBoxLayout(self)

        layout.addWidget(QLabel("Hardware status:"))
        self._leds: dict[str, StatusLed] = {}
        self._status_labels: dict[str, QLabel] = {}
        for name in CAN_INTERFACES:
            row = QHBoxLayout()
            led = StatusLed()
            row.addWidget(led)

            name_lbl = QLabel(f"<b>{name}</b>")
            row.addWidget(name_lbl)

            role_lbl = QLabel(_ROLES.get(name, ""))
            role_lbl.setStyleSheet("color: #888;")
            row.addWidget(role_lbl)
            row.addStretch()

            status_lbl = QLabel("not detected")
            status_lbl.setStyleSheet("color: #888;")
            row.addWidget(status_lbl)

            layout.addLayout(row)
            self._leds[name] = led
            self._status_labels[name] = status_lbl

        self._bringup_btn = QPushButton("Bring up CAN (can0..can3)")
        self._bringup_btn.clicked.connect(self._on_bringup_clicked)
        layout.addWidget(self._bringup_btn)

        layout.addSpacing(8)
        layout.addWidget(QLabel("Launch:"))

        ctrl_row = QHBoxLayout()
        ctrl_row.addWidget(QLabel("Controller"))
        self._controller_combo = QComboBox()
        for label in CONTROLLERS:
            self._controller_combo.addItem(label)
        ctrl_row.addWidget(self._controller_combo)
        ctrl_row.addStretch()
        layout.addLayout(ctrl_row)

        self._fake_chk = QCheckBox("use_fake_hardware")
        layout.addWidget(self._fake_chk)

        btn_row = QHBoxLayout()
        self._launch_btn = QPushButton("Launch OpenArm")
        self._launch_btn.clicked.connect(self._on_launch_clicked)
        btn_row.addWidget(self._launch_btn)
        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop_clicked)
        btn_row.addWidget(self._stop_btn)
        layout.addLayout(btn_row)

        layout.addStretch()

        self._detector = CanDetector(self)
        self._detector.state.connect(self._on_state)
        # Defer the initial scan until the event loop runs so any consumer
        # (e.g. MainWindow log) has time to wire up our log_line signal.
        # The timer is parented to self so it is destroyed with us and
        # can't fire on a half-destroyed CanDetector during teardown.
        self._initial_scan = QTimer(self)
        self._initial_scan.setSingleShot(True)
        self._initial_scan.timeout.connect(self._detector.emit_current_state)
        self._initial_scan.start(0)

        self._bringup = CanBringUp(self)
        self._bringup.line.connect(self.log_line)
        self._bringup.finished.connect(self._on_bringup_done)

        self._launcher = Ros2Launcher(self)
        self._launcher.line.connect(self.log_line)
        self._launcher.started.connect(self._on_launch_started)
        self._launcher.finished.connect(self._on_launch_finished)

        self._settings = QSettings()
        saved_ctrl = self._settings.value("launcher/controller", "")
        if saved_ctrl:
            idx = self._controller_combo.findText(str(saved_ctrl))
            if idx >= 0:
                self._controller_combo.setCurrentIndex(idx)
        self._fake_chk.setChecked(
            self._settings.value("launcher/use_fake_hardware", False, type=bool)
        )
        self._controller_combo.currentTextChanged.connect(
            lambda v: self._settings.setValue("launcher/controller", v)
        )
        self._fake_chk.toggled.connect(
            lambda v: self._settings.setValue("launcher/use_fake_hardware", v)
        )

    def shutdown(self) -> None:
        """Stop any running ros2 launch — called from MainWindow.closeEvent."""
        if self._launcher.is_running():
            self._launcher.stop()

    def _on_bringup_clicked(self) -> None:
        if self._bringup.is_running():
            return
        self._bringup_btn.setEnabled(False)
        self._bringup_btn.setText("Bringing up CAN...")
        self.log_line.emit("--- CAN bring-up start ---")
        self._bringup.start()

    def _on_bringup_done(self, ok: bool) -> None:
        self._bringup_btn.setEnabled(True)
        self._bringup_btn.setText("Bring up CAN (can0..can3)")
        if ok:
            self.log_line.emit("--- CAN bring-up done ---")
        else:
            self.log_line.emit(
                "--- CAN bring-up FAILED. "
                "If you see 'sudo: a password is required', run "
                "`sudo ./install_sudoers.sh` once. ---"
            )

    def _on_launch_clicked(self) -> None:
        if self._launcher.is_running():
            return
        self._launch_btn.setEnabled(False)
        self.log_line.emit("--- ros2 launch start ---")
        self._launcher.start(
            self._controller_combo.currentText(),
            self._fake_chk.isChecked(),
        )

    def _on_launch_started(self) -> None:
        self._stop_btn.setEnabled(True)

    def _on_launch_finished(self, code: int) -> None:
        self._launch_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self.log_line.emit(f"--- ros2 launch ended (exit {code}) ---")

    def _on_stop_clicked(self) -> None:
        if not self._launcher.is_running():
            return
        self._stop_btn.setEnabled(False)
        self.log_line.emit("--- requesting ros2 launch stop ---")
        self._launcher.stop()

    def _on_state(self, name: str, operstate) -> None:
        led = self._leds.get(name)
        lbl = self._status_labels.get(name)
        if led is None or lbl is None:
            return

        if operstate is None:
            led.set_state("off")
            lbl.setText("not detected")
            self.log_line.emit(f"[can] {name}: not detected")
        elif operstate == "up":
            led.set_state("up")
            lbl.setText("UP")
            self.log_line.emit(f"[can] {name}: UP")
        else:
            led.set_state("down")
            lbl.setText(str(operstate))
            self.log_line.emit(f"[can] {name}: {operstate}")
