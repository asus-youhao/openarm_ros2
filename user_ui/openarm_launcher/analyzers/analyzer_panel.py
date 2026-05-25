import datetime
from pathlib import Path

from PySide6.QtCore import QUrl, Signal
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)

from .arm_runner import ArmRunner, joints_for
from .csv_writer import ArmCsvWriter
from .live_plot import JointErrorPlot


DATA_ROOT = Path(__file__).resolve().parent.parent.parent / "data"
ARM_DATA_ROOT = DATA_ROOT / "arm"


class AnalyzerPanel(QGroupBox):
    log_line = Signal(str)

    def __init__(self) -> None:
        super().__init__("Analyzers")
        layout = QVBoxLayout(self)

        layout.addWidget(QLabel("<b>Arm analyzer</b>"))
        arm_row = QHBoxLayout()
        arm_row.addWidget(QLabel("Arm"))
        self._arm_combo = QComboBox()
        self._arm_combo.addItems(["right", "left"])
        arm_row.addWidget(self._arm_combo)
        arm_row.addStretch()
        layout.addLayout(arm_row)

        arm_btn_row = QHBoxLayout()
        self._arm_start = QPushButton("Start arm analyzer")
        self._arm_start.clicked.connect(self._start_arm)
        self._arm_stop = QPushButton("Stop")
        self._arm_stop.setEnabled(False)
        self._arm_stop.clicked.connect(self._stop_arm)
        arm_btn_row.addWidget(self._arm_start)
        arm_btn_row.addWidget(self._arm_stop)
        layout.addLayout(arm_btn_row)

        layout.addSpacing(8)
        layout.addWidget(QLabel("<b>O6 hand analyzer</b>"))
        layout.addWidget(QLabel("<i>Arrives in M7.</i>"))

        layout.addSpacing(8)
        self._open_folder_btn = QPushButton("Open data folder")
        self._open_folder_btn.clicked.connect(self._on_open_folder)
        layout.addWidget(self._open_folder_btn)

        layout.addStretch()

        self._arm_runner: ArmRunner | None = None
        self._arm_csv: ArmCsvWriter | None = None
        self._arm_plot: JointErrorPlot | None = None

    def _start_arm(self) -> None:
        if self._arm_runner is not None:
            return
        side = self._arm_combo.currentText()
        joint_names = joints_for(side)
        ARM_DATA_ROOT.mkdir(parents=True, exist_ok=True)

        try:
            self._arm_csv = ArmCsvWriter(ARM_DATA_ROOT, side, joint_names)
            self._arm_plot = JointErrorPlot(joint_names, title=f"OpenArm {side} — joint error")
            self._arm_plot.resize(900, 500)
            self._arm_plot.setWindowTitle(f"Arm {side} error")
            self._arm_plot.show()

            self._arm_runner = ArmRunner(side, self)
            self._arm_runner.sample.connect(self._on_arm_sample)
            self._arm_runner.started.connect(
                lambda s=side: self.log_line.emit(f"--- arm {s} analyzer started ---")
            )
            self._arm_runner.stopped.connect(
                lambda s=side: self.log_line.emit(f"--- arm {s} analyzer stopped ---")
            )
            self._arm_runner.start()
        except Exception as exc:  # noqa: BLE001
            self.log_line.emit(f"[error] could not start arm analyzer: {exc}")
            self._cleanup_arm()
            return

        self.log_line.emit(f"[saving to] {self._arm_csv.path}")
        self._arm_start.setEnabled(False)
        self._arm_stop.setEnabled(True)
        self._arm_combo.setEnabled(False)

    def _on_arm_sample(self, t: float, cmd: list, actual: list) -> None:
        errors = [a - c for a, c in zip(actual, cmd)]
        if self._arm_plot is not None:
            self._arm_plot.add_sample(t, errors)
        if self._arm_csv is not None:
            self._arm_csv.write(t, cmd, actual)

    def _stop_arm(self) -> None:
        if self._arm_runner is None:
            return
        side = self._arm_runner.side
        try:
            self._arm_runner.stop()
            if self._arm_plot is not None:
                today = datetime.date.today().isoformat()
                ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
                img_dir = ARM_DATA_ROOT / "img" / today
                img_dir.mkdir(parents=True, exist_ok=True)
                png_path = img_dir / f"{side}_{ts}.png"
                try:
                    self._arm_plot.save_png(png_path)
                    self.log_line.emit(f"[saved] {png_path}")
                except Exception as exc:  # noqa: BLE001
                    self.log_line.emit(f"[warn] could not save PNG: {exc}")
                self._arm_plot.close()
            if self._arm_csv is not None:
                self._arm_csv.close()
                self.log_line.emit(f"[saved] {self._arm_csv.path}")
        finally:
            self._cleanup_arm()

    def _cleanup_arm(self) -> None:
        self._arm_runner = None
        self._arm_csv = None
        self._arm_plot = None
        self._arm_start.setEnabled(True)
        self._arm_stop.setEnabled(False)
        self._arm_combo.setEnabled(True)

    def _on_open_folder(self) -> None:
        DATA_ROOT.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(DATA_ROOT)))
