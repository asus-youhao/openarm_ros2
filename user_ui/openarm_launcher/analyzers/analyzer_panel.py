from PySide6.QtWidgets import QGroupBox, QLabel, QVBoxLayout


class AnalyzerPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Analyzers")
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Arm and O6 analyzers, plus 'Open data folder', will live here."))
        layout.addStretch()
