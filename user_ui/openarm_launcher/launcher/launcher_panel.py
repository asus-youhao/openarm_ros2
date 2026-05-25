from PySide6.QtWidgets import QGroupBox, QLabel, QVBoxLayout


class LauncherPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Launcher")
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("CAN status, mode selection, and Start/Stop will live here."))
        layout.addStretch()
