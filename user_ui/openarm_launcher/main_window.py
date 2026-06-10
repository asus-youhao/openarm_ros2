from PySide6.QtCore import Qt
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QMainWindow,
    QPlainTextEdit,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .launcher.launcher_panel import LauncherPanel
from .analyzers.analyzer_panel import AnalyzerPanel
from .analyzers.bimanual_panel import BimanualPanel
from .analyzers.record_replay_panel import RecordReplayPanel


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("OpenArm Launcher")
        self.resize(1200, 900)

        central = QWidget(self)
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # Outer vertical splitter: top panels / bimanual controller / log
        vsplit = QSplitter(Qt.Vertical, central)

        top = QSplitter(Qt.Horizontal)
        self.launcher_panel = LauncherPanel()
        self.analyzer_panel = AnalyzerPanel()
        self.record_replay_panel = RecordReplayPanel()

        # Right column: Analyzer on top, Record & Replay below
        right_col = QSplitter(Qt.Vertical)
        right_col.addWidget(self.analyzer_panel)
        right_col.addWidget(self.record_replay_panel)
        right_col.setSizes([300, 200])

        top.addWidget(self.launcher_panel)
        top.addWidget(right_col)
        top.setSizes([550, 550])
        vsplit.addWidget(top)

        self.bimanual_panel = BimanualPanel()
        bm_scroll = QScrollArea()
        bm_scroll.setWidgetResizable(True)
        bm_scroll.setWidget(self.bimanual_panel)
        vsplit.addWidget(bm_scroll)

        self.log = QPlainTextEdit(central)
        self.log.setReadOnly(True)
        self.log.setPlaceholderText("Log output will appear here.")
        vsplit.addWidget(self.log)

        vsplit.setSizes([300, 420, 130])
        root.addWidget(vsplit)

        self.launcher_panel.log_line.connect(self.log.appendPlainText)
        self.analyzer_panel.log_line.connect(self.log.appendPlainText)
        self.bimanual_panel.log_line.connect(self.log.appendPlainText)
        self.record_replay_panel.log_line.connect(self.log.appendPlainText)

        # Auto-sync analyzer + bimanual mode when controller selection changes.
        self.launcher_panel.controller_mode_changed.connect(
            self.analyzer_panel.set_analyzer_mode
        )
        self.launcher_panel.controller_mode_changed.connect(
            self.bimanual_panel.set_mode
        )
        self.launcher_panel.controller_mode_changed.connect(
            self.record_replay_panel.set_replay_mode
        )
        # Apply the initial controller's mode immediately.
        initial_mode = self.launcher_panel.current_analyzer_mode()
        self.analyzer_panel.set_analyzer_mode(initial_mode)
        self.bimanual_panel.set_mode(initial_mode)
        self.record_replay_panel.set_replay_mode(initial_mode)

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 (Qt API)
        try:
            self.launcher_panel.shutdown()
        finally:
            try:
                self.analyzer_panel.shutdown()
            finally:
                try:
                    self.bimanual_panel.shutdown()
                finally:
                    self.record_replay_panel.shutdown()
        super().closeEvent(event)
