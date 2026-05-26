import datetime
from pathlib import Path

from PySide6.QtCore import QSettings, QUrl, Signal
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
from .o6_runner import O6Runner, o6_joints_for


DATA_ROOT = Path(__file__).resolve().parent.parent.parent / "data"
ARM_DATA_ROOT = DATA_ROOT / "arm"
O6_DATA_ROOT = DATA_ROOT / "o6"


class AnalyzerPanel(QGroupBox):
    log_line = Signal(str)

    def __init__(self) -> None:
        super().__init__("Analyzers")
        layout = QVBoxLayout(self)

        layout.addWidget(QLabel("<b>Arm analyzer</b>"))
        arm_row = QHBoxLayout()
        arm_row.addWidget(QLabel("Arm"))
        self._arm_combo = QComboBox()
        self._arm_combo.addItems(["right", "left", "both"])
        arm_row.addWidget(self._arm_combo)
        arm_row.addWidget(QLabel("Mode"))
        self._arm_mode_combo = QComboBox()
        self._arm_mode_combo.addItems(["action", "topic"])
        self._arm_mode_combo.setToolTip(
            "action — uses joint_trajectory_controller/controller_state\n"
            "topic  — uses joint_states only (plots absolute position)"
        )
        arm_row.addWidget(self._arm_mode_combo)
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
        o6_row = QHBoxLayout()
        o6_row.addWidget(QLabel("Hand"))
        self._o6_combo = QComboBox()
        self._o6_combo.addItems(["right", "left", "both"])
        o6_row.addWidget(self._o6_combo)
        o6_row.addWidget(QLabel("Mode"))
        self._o6_mode_combo = QComboBox()
        self._o6_mode_combo.addItems(["action", "topic"])
        self._o6_mode_combo.setToolTip(
            "action — uses hand_controller/controller_state\n"
            "topic  — uses joint_states only (plots absolute position)"
        )
        o6_row.addWidget(self._o6_mode_combo)
        o6_row.addStretch()
        layout.addLayout(o6_row)

        o6_btn_row = QHBoxLayout()
        self._o6_start = QPushButton("Start O6 analyzer")
        self._o6_start.clicked.connect(self._start_o6)
        self._o6_stop = QPushButton("Stop")
        self._o6_stop.setEnabled(False)
        self._o6_stop.clicked.connect(self._stop_o6)
        o6_btn_row.addWidget(self._o6_start)
        o6_btn_row.addWidget(self._o6_stop)
        layout.addLayout(o6_btn_row)

        layout.addSpacing(8)
        self._open_folder_btn = QPushButton("Open data folder")
        self._open_folder_btn.clicked.connect(self._on_open_folder)
        layout.addWidget(self._open_folder_btn)

        layout.addStretch()

        # List-based state: one entry per active side (len=1 for single, len=2 for both).
        self._arm_runners: list[ArmRunner] = []
        self._arm_csvs: list[ArmCsvWriter] = []
        self._arm_plots: list[JointErrorPlot] = []

        self._o6_runners: list[O6Runner] = []
        self._o6_csvs: list[ArmCsvWriter] = []
        self._o6_plots: list[JointErrorPlot] = []

        self._settings = QSettings()
        saved_arm = self._settings.value("analyzers/arm_side", "")
        if saved_arm:
            idx = self._arm_combo.findText(str(saved_arm))
            if idx >= 0:
                self._arm_combo.setCurrentIndex(idx)
        saved_arm_mode = self._settings.value("analyzers/arm_mode", "")
        if saved_arm_mode:
            idx = self._arm_mode_combo.findText(str(saved_arm_mode))
            if idx >= 0:
                self._arm_mode_combo.setCurrentIndex(idx)
        saved_o6 = self._settings.value("analyzers/o6_side", "")
        if saved_o6:
            idx = self._o6_combo.findText(str(saved_o6))
            if idx >= 0:
                self._o6_combo.setCurrentIndex(idx)
        saved_o6_mode = self._settings.value("analyzers/o6_mode", "")
        if saved_o6_mode:
            idx = self._o6_mode_combo.findText(str(saved_o6_mode))
            if idx >= 0:
                self._o6_mode_combo.setCurrentIndex(idx)
        self._arm_combo.currentTextChanged.connect(
            lambda v: self._settings.setValue("analyzers/arm_side", v)
        )
        self._arm_mode_combo.currentTextChanged.connect(
            lambda v: self._settings.setValue("analyzers/arm_mode", v)
        )
        self._o6_combo.currentTextChanged.connect(
            lambda v: self._settings.setValue("analyzers/o6_side", v)
        )
        self._o6_mode_combo.currentTextChanged.connect(
            lambda v: self._settings.setValue("analyzers/o6_mode", v)
        )

    def set_analyzer_mode(self, mode: str) -> None:
        """Sync mode combos to match the launcher controller (action/topic).
        Only applies when the respective analyzer is not running.
        """
        if not self._arm_runners:
            idx = self._arm_mode_combo.findText(mode)
            if idx >= 0:
                self._arm_mode_combo.setCurrentIndex(idx)
        if not self._o6_runners:
            idx = self._o6_mode_combo.findText(mode)
            if idx >= 0:
                self._o6_mode_combo.setCurrentIndex(idx)

    def shutdown(self) -> None:
        """Stop any running analyzers — called from MainWindow.closeEvent."""
        if self._arm_runners:
            self._stop_arm()
        if self._o6_runners:
            self._stop_o6()

    # ------------------------------------------------------------------
    # Arm analyzer
    # ------------------------------------------------------------------

    def _start_arm(self) -> None:
        if self._arm_runners:
            return
        selection = self._arm_combo.currentText()
        sides = ["right", "left"] if selection == "both" else [selection]
        ARM_DATA_ROOT.mkdir(parents=True, exist_ok=True)

        for side in sides:
            joint_names = joints_for(side)
            mode = self._arm_mode_combo.currentText()
            is_topic = mode == "topic"
            y_label = "position (rad)" if is_topic else "error (rad)"
            plot_title = (
                f"OpenArm {side} \u2014 joint position"
                if is_topic
                else f"OpenArm {side} \u2014 joint error"
            )
            try:
                csv_w = ArmCsvWriter(ARM_DATA_ROOT, side, joint_names)
                plot = JointErrorPlot(
                    joint_names, title=plot_title, y_label=y_label
                )
                plot.resize(900, 500)
                plot.setWindowTitle(f"Arm {side} {'pos' if is_topic else 'error'}")
                plot.closed_by_user.connect(self._stop_arm)
                plot.show()

                runner = ArmRunner(side, self, mode=mode)
                runner.sample.connect(
                    lambda t, cmd, actual, _p=plot, _c=csv_w:
                        self._on_arm_sample(t, cmd, actual, _p, _c)
                )
                runner.started.connect(
                    lambda _s=side: self.log_line.emit(
                        f"--- arm {_s} analyzer started ({mode}) ---"
                    )
                )
                runner.stopped.connect(
                    lambda _s=side: self.log_line.emit(
                        f"--- arm {_s} analyzer stopped ---"
                    )
                )
                runner.start()
                self._arm_runners.append(runner)
                self._arm_csvs.append(csv_w)
                self._arm_plots.append(plot)
                self.log_line.emit(f"[saving to] {csv_w.path}")
            except Exception as exc:  # noqa: BLE001
                self.log_line.emit(
                    f"[error] could not start arm analyzer ({side}): {exc}"
                )
                self._stop_arm()
                return

        self._arm_start.setEnabled(False)
        self._arm_stop.setEnabled(True)
        self._arm_combo.setEnabled(False)
        self._arm_mode_combo.setEnabled(False)

    def _on_arm_sample(
        self,
        t: float,
        cmd: list,
        actual: list,
        plot: JointErrorPlot,
        csv_w: ArmCsvWriter,
    ) -> None:
        if not plot.isVisible():  # discard late signals after stop
            return
        errors = [a - c for a, c in zip(actual, cmd)]
        plot.add_sample(t, errors)
        csv_w.write(t, cmd, actual)

    def _stop_arm(self) -> None:
        if not self._arm_runners:
            return
        today = datetime.date.today().isoformat()
        ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        for runner, plot, csv_w in zip(
            list(self._arm_runners),
            list(self._arm_plots),
            list(self._arm_csvs),
        ):
            side = runner.side
            # File I/O happens BEFORE stop() so we never stall the Qt thread
            # waiting for the spin thread (stop() is now non-blocking).
            img_dir = ARM_DATA_ROOT / "img" / today
            img_dir.mkdir(parents=True, exist_ok=True)
            png_path = img_dir / f"{side}_{ts}.png"
            try:
                plot.save_png(png_path)
                self.log_line.emit(f"[saved] {png_path}")
            except Exception as exc:  # noqa: BLE001
                self.log_line.emit(f"[warn] could not save PNG: {exc}")
            plot.close_programmatically()
            try:
                csv_w.close()
                self.log_line.emit(f"[saved] {csv_w.path}")
            except Exception:  # noqa: BLE001
                pass
            # Non-blocking: spin thread exits in background.
            try:
                runner.stop()
            except Exception:  # noqa: BLE001
                pass
        self._cleanup_arm()

    def _cleanup_arm(self) -> None:
        self._arm_runners.clear()
        self._arm_csvs.clear()
        self._arm_plots.clear()
        self._arm_start.setEnabled(True)
        self._arm_stop.setEnabled(False)
        self._arm_combo.setEnabled(True)
        self._arm_mode_combo.setEnabled(True)

    # ------------------------------------------------------------------
    # O6 analyzer
    # ------------------------------------------------------------------

    def _start_o6(self) -> None:
        if self._o6_runners:
            return
        selection = self._o6_combo.currentText()
        sides = ["right", "left"] if selection == "both" else [selection]
        O6_DATA_ROOT.mkdir(parents=True, exist_ok=True)

        for side in sides:
            joint_names = o6_joints_for(side)
            mode = self._o6_mode_combo.currentText()
            is_topic = mode == "topic"
            y_label = "position (rad)" if is_topic else "error (rad)"
            plot_title = (
                f"O6 {side} hand \u2014 joint position"
                if is_topic
                else f"O6 {side} hand \u2014 joint error"
            )
            try:
                csv_w = ArmCsvWriter(O6_DATA_ROOT, side, joint_names)
                plot = JointErrorPlot(
                    joint_names, title=plot_title, y_label=y_label
                )
                plot.resize(900, 500)
                plot.setWindowTitle(f"O6 {side} {'pos' if is_topic else 'error'}")
                plot.closed_by_user.connect(self._stop_o6)
                plot.show()

                runner = O6Runner(side, self, mode=mode)
                runner.sample.connect(
                    lambda t, cmd, actual, _p=plot, _c=csv_w:
                        self._on_o6_sample(t, cmd, actual, _p, _c)
                )
                runner.started.connect(
                    lambda _s=side: self.log_line.emit(
                        f"--- O6 {_s} analyzer started ({mode}) ---"
                    )
                )
                runner.stopped.connect(
                    lambda _s=side: self.log_line.emit(
                        f"--- O6 {_s} analyzer stopped ---"
                    )
                )
                runner.start()
                self._o6_runners.append(runner)
                self._o6_csvs.append(csv_w)
                self._o6_plots.append(plot)
                self.log_line.emit(f"[saving to] {csv_w.path}")
            except Exception as exc:  # noqa: BLE001
                self.log_line.emit(
                    f"[error] could not start O6 analyzer ({side}): {exc}"
                )
                self._stop_o6()
                return

        self._o6_start.setEnabled(False)
        self._o6_stop.setEnabled(True)
        self._o6_combo.setEnabled(False)
        self._o6_mode_combo.setEnabled(False)

    def _on_o6_sample(
        self,
        t: float,
        cmd: list,
        actual: list,
        plot: JointErrorPlot,
        csv_w: ArmCsvWriter,
    ) -> None:
        if not plot.isVisible():  # discard late signals after stop
            return
        errors = [a - c for a, c in zip(actual, cmd)]
        plot.add_sample(t, errors)
        csv_w.write(t, cmd, actual)

    def _stop_o6(self) -> None:
        if not self._o6_runners:
            return
        today = datetime.date.today().isoformat()
        ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        for runner, plot, csv_w in zip(
            list(self._o6_runners),
            list(self._o6_plots),
            list(self._o6_csvs),
        ):
            side = runner.side
            # File I/O happens BEFORE stop() so we never stall the Qt thread
            # waiting for the spin thread (stop() is now non-blocking).
            img_dir = O6_DATA_ROOT / "img" / today
            img_dir.mkdir(parents=True, exist_ok=True)
            png_path = img_dir / f"{side}_{ts}.png"
            try:
                plot.save_png(png_path)
                self.log_line.emit(f"[saved] {png_path}")
            except Exception as exc:  # noqa: BLE001
                self.log_line.emit(f"[warn] could not save PNG: {exc}")
            plot.close_programmatically()
            try:
                csv_w.close()
                self.log_line.emit(f"[saved] {csv_w.path}")
            except Exception:  # noqa: BLE001
                pass
            # Non-blocking: spin thread exits in background.
            try:
                runner.stop()
            except Exception:  # noqa: BLE001
                pass
        self._cleanup_o6()

    def _cleanup_o6(self) -> None:
        self._o6_runners.clear()
        self._o6_csvs.clear()
        self._o6_plots.clear()
        self._o6_start.setEnabled(True)
        self._o6_stop.setEnabled(False)
        self._o6_combo.setEnabled(True)
        self._o6_mode_combo.setEnabled(True)

    def _on_open_folder(self) -> None:
        DATA_ROOT.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(DATA_ROOT)))
