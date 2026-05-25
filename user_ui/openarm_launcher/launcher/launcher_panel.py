from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
)

from .can_detect import CAN_INTERFACES, CanDetector


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

        layout.addWidget(QLabel("<i>Start button arrives in M4/M5.</i>"))
        layout.addStretch()

        self._detector = CanDetector(self)
        self._detector.state.connect(self._on_state)
        # Defer the initial scan until the event loop runs so any consumer
        # (e.g. MainWindow log) has time to wire up our log_line signal.
        QTimer.singleShot(0, self._detector.emit_current_state)

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
