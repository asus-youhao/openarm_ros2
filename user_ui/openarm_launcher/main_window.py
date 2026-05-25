from PySide6.QtCore import Qt
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QMainWindow,
    QPlainTextEdit,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .launcher.launcher_panel import LauncherPanel
from .analyzers.analyzer_panel import AnalyzerPanel


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("OpenArm Launcher")
        self.resize(1100, 720)

        central = QWidget(self)
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        self.launcher_panel = LauncherPanel()
        self.analyzer_panel = AnalyzerPanel()

        top = QSplitter(Qt.Horizontal, central)
        top.addWidget(self.launcher_panel)
        top.addWidget(self.analyzer_panel)
        top.setSizes([550, 550])
        root.addWidget(top, stretch=3)

        self.log = QPlainTextEdit(central)
        self.log.setReadOnly(True)
        self.log.setPlaceholderText("Log output will appear here.")
        root.addWidget(self.log, stretch=1)

        self.launcher_panel.log_line.connect(self.log.appendPlainText)
        self.analyzer_panel.log_line.connect(self.log.appendPlainText)

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 (Qt API)
        try:
            self.launcher_panel.shutdown()
        finally:
            self.analyzer_panel.shutdown()
        super().closeEvent(event)
