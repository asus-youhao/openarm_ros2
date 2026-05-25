from PySide6.QtCore import Qt
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

        top = QSplitter(Qt.Horizontal, central)
        top.addWidget(LauncherPanel())
        top.addWidget(AnalyzerPanel())
        top.setSizes([550, 550])
        root.addWidget(top, stretch=3)

        self.log = QPlainTextEdit(central)
        self.log.setReadOnly(True)
        self.log.setPlaceholderText("Log output will appear here.")
        root.addWidget(self.log, stretch=1)
