from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGroupBox, QLabel, QPushButton, QVBoxLayout

from ..common.proc import ManagedProcess


class LauncherPanel(QGroupBox):
    log_line = Signal(str)

    def __init__(self) -> None:
        super().__init__("Launcher")
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("CAN status, mode selection, and Start/Stop will live here."))

        # M2 temp dev button — replaced when real Start UI lands in M5.
        self._test_btn = QPushButton("M2 test: run 'ros2 topic list'")
        self._test_btn.clicked.connect(self._run_test)
        layout.addWidget(self._test_btn)

        self._proc = ManagedProcess(self)
        self._proc.line.connect(self.log_line)
        self._proc.finished.connect(lambda code: self.log_line.emit(f"[exit {code}]"))

        layout.addStretch()

    def _run_test(self) -> None:
        if self._proc.is_running():
            self.log_line.emit("[busy] previous test still running")
            return
        self.log_line.emit("$ ros2 topic list")
        self._proc.start("ros2", ["topic", "list"])
