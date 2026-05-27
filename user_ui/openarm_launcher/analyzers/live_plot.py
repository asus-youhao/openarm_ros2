from pathlib import Path

import numpy as np
import pyqtgraph as pg
import pyqtgraph.exporters
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)


pg.setConfigOptions(antialias=False)


def _rolling_mean(data: list[float], n: int) -> list[float]:
    """Causal rolling mean of *data* over window of *n* samples."""
    if len(data) == 0:
        return []
    a = np.array(data, dtype=np.float64)
    n = max(1, min(n, len(a)))
    kernel = np.ones(n) / n
    padded = np.pad(a, (n - 1, 0), mode="edge")
    return np.convolve(padded, kernel, mode="valid").tolist()


class JointErrorPlot(QWidget):
    """Rolling-window plot of position error (or raw position) per joint.

    A dashed trend curve (rolling mean) overlays each joint's solid curve.
    A horizontal y = 0 reference line is shown for orientation.
    A row of per-joint checkboxes sits below the plot for visibility toggling.
    Saved CSV data is always full regardless of checkbox state.

    Emits `closed_by_user` when the window is closed via its X button so
    the owning panel can auto-stop the analyzer and save.
    """

    closed_by_user = Signal()

    def __init__(
        self,
        joint_names: list[str],
        title: str = "Joint error",
        window_sec: float = 20.0,
        y_label: str = "error (rad)",
        trend_n: int = 30,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._joint_names = joint_names
        self._window_sec = window_sec
        self._trend_n = trend_n

        self._plot = pg.PlotWidget(background="w")
        self._plot.setTitle(title)
        self._plot.setLabel("left", y_label)
        self._plot.setLabel("bottom", "t (s)")
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.addLegend()

        # y = 0 reference line
        self._zero_line = pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen("#aaaaaa", width=1, style=Qt.DashLine),
            movable=False,
        )
        self._plot.addItem(self._zero_line)

        # Distinct colours per joint.
        cmap = pg.colormap.get("CET-C2") or pg.colormap.get("hsv")
        lut = cmap.getLookupTable(0.0, 1.0, max(len(joint_names), 1))
        colors = [
            (int(lut[i, 0]), int(lut[i, 1]), int(lut[i, 2]))
            for i in range(len(joint_names))
        ]

        self._curves: list = []
        self._trend_curves: list = []
        self._checkboxes: list[QCheckBox] = []

        cb_container = QWidget()
        cb_layout = QHBoxLayout(cb_container)
        cb_layout.setContentsMargins(4, 0, 4, 0)
        cb_layout.setSpacing(6)

        for i, name in enumerate(joint_names):
            r, g, b = colors[i]
            # Main curve (solid)
            curve = self._plot.plot(
                [], [], pen=pg.mkPen((r, g, b), width=1.5), name=name
            )
            self._curves.append(curve)
            # Trend curve (dashed rolling mean, no legend entry)
            trend = self._plot.plot(
                [], [],
                pen=pg.mkPen((r, g, b), width=1.0, style=Qt.DashLine),
            )
            self._trend_curves.append(trend)

            cb = QCheckBox(name)
            cb.setChecked(True)
            cb.setStyleSheet(
                f"QCheckBox {{ color: rgb({r},{g},{b}); font-size: 9px; }}"
            )
            cb.toggled.connect(lambda checked, idx=i: self._on_visibility(idx, checked))
            cb_layout.addWidget(cb)
            self._checkboxes.append(cb)

        cb_layout.addStretch()

        vis_scroll = QScrollArea()
        vis_scroll.setWidgetResizable(True)
        vis_scroll.setWidget(cb_container)
        vis_scroll.setFixedHeight(36)
        vis_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        vis_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self._t: list[float] = []
        self._errs: list[list[float]] = [[] for _ in joint_names]
        self._programmatic_close = False
        self._dirty = False

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(33)  # ~30 fps
        self._refresh_timer.timeout.connect(self._refresh)
        self._refresh_timer.start()

        layout = QVBoxLayout(self)
        layout.addWidget(self._plot)
        layout.addWidget(vis_scroll)

    def _on_visibility(self, idx: int, visible: bool) -> None:
        if visible:
            self._curves[idx].show()
            self._trend_curves[idx].show()
        else:
            self._curves[idx].hide()
            self._trend_curves[idx].hide()

    def close_programmatically(self) -> None:
        self._programmatic_close = True
        self.close()

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802
        if not self._programmatic_close:
            self.closed_by_user.emit()
        super().closeEvent(event)

    def add_sample(self, t: float, errors: list[float]) -> None:
        """Buffer a new sample. Rendering happens via the 30 fps QTimer."""
        self._t.append(t)
        for i, e in enumerate(errors):
            self._errs[i].append(e)
        t_cut = t - self._window_sec
        while self._t and self._t[0] < t_cut:
            self._t.pop(0)
            for arr in self._errs:
                arr.pop(0)
        self._dirty = True

    def _refresh(self) -> None:
        """Called by QTimer at ~30 fps; renders only when the buffer changed."""
        if not self._dirty:
            return
        self._dirty = False
        t_arr = np.array(self._t, dtype=np.float64)
        for i, curve in enumerate(self._curves):
            err_arr = np.array(self._errs[i], dtype=np.float64)
            curve.setData(t_arr, err_arr)
            self._trend_curves[i].setData(
                t_arr, _rolling_mean(self._errs[i], self._trend_n)
            )

    def save_png(self, path: Path) -> None:
        exporter = pyqtgraph.exporters.ImageExporter(self._plot.getPlotItem())
        exporter.parameters()["width"] = 1200
        exporter.export(str(path))


class JointPositionPlot(QWidget):
    """Top-half plot: shows cmd (dashed) and actual (solid) curves per joint.

    Shares the same rolling-window and per-joint checkbox design as
    JointErrorPlot.  Receives raw (t, cmd, actual) via add_sample().
    """

    def __init__(
        self,
        joint_names: list[str],
        title: str = "Joint action & state",
        window_sec: float = 20.0,
        y_label: str = "position (rad)",
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._joint_names = joint_names
        self._window_sec = window_sec

        self._plot = pg.PlotWidget(background="w")
        self._plot.setTitle(title)
        self._plot.setLabel("left", y_label)
        self._plot.setLabel("bottom", "t (s)")
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.addLegend()

        cmap = pg.colormap.get("CET-C2") or pg.colormap.get("hsv")
        lut = cmap.getLookupTable(0.0, 1.0, max(len(joint_names), 1))
        colors = [
            (int(lut[i, 0]), int(lut[i, 1]), int(lut[i, 2]))
            for i in range(len(joint_names))
        ]

        self._cmd_curves: list = []
        self._actual_curves: list = []
        self._checkboxes: list[QCheckBox] = []

        cb_container = QWidget()
        cb_layout = QHBoxLayout(cb_container)
        cb_layout.setContentsMargins(4, 0, 4, 0)
        cb_layout.setSpacing(6)

        for i, name in enumerate(joint_names):
            r, g, b = colors[i]
            # cmd: dashed line (the commanded/reference trajectory)
            cmd_curve = self._plot.plot(
                [], [],
                pen=pg.mkPen((r, g, b), width=1.0, style=Qt.DashLine),
                name=f"{name} cmd",
            )
            self._cmd_curves.append(cmd_curve)
            # actual: solid line (the measured joint state)
            actual_curve = self._plot.plot(
                [], [],
                pen=pg.mkPen((r, g, b), width=1.5),
                name=f"{name} act",
            )
            self._actual_curves.append(actual_curve)

            cb = QCheckBox(name)
            cb.setChecked(True)
            cb.setStyleSheet(
                f"QCheckBox {{ color: rgb({r},{g},{b}); font-size: 9px; }}"
            )
            cb.toggled.connect(lambda checked, idx=i: self._on_visibility(idx, checked))
            cb_layout.addWidget(cb)
            self._checkboxes.append(cb)

        cb_layout.addStretch()

        vis_scroll = QScrollArea()
        vis_scroll.setWidgetResizable(True)
        vis_scroll.setWidget(cb_container)
        vis_scroll.setFixedHeight(36)
        vis_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        vis_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self._t: list[float] = []
        self._cmds: list[list[float]] = [[] for _ in joint_names]
        self._actuals: list[list[float]] = [[] for _ in joint_names]
        self._dirty = False

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(33)  # ~30 fps
        self._refresh_timer.timeout.connect(self._refresh)
        self._refresh_timer.start()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._plot)
        layout.addWidget(vis_scroll)

    def _on_visibility(self, idx: int, visible: bool) -> None:
        if visible:
            self._cmd_curves[idx].show()
            self._actual_curves[idx].show()
        else:
            self._cmd_curves[idx].hide()
            self._actual_curves[idx].hide()

    def add_sample(self, t: float, cmd: list[float], actual: list[float]) -> None:
        """Buffer a new sample. Rendering happens via the 30 fps QTimer."""
        self._t.append(t)
        for i in range(len(self._joint_names)):
            self._cmds[i].append(cmd[i])
            self._actuals[i].append(actual[i])
        t_cut = t - self._window_sec
        while self._t and self._t[0] < t_cut:
            self._t.pop(0)
            for arr in self._cmds:
                arr.pop(0)
            for arr in self._actuals:
                arr.pop(0)
        self._dirty = True

    def _refresh(self) -> None:
        """Called by QTimer at ~30 fps; renders only when the buffer changed."""
        if not self._dirty:
            return
        self._dirty = False
        t_arr = np.array(self._t, dtype=np.float64)
        for i in range(len(self._joint_names)):
            cmd_arr = np.array(self._cmds[i], dtype=np.float64)
            act_arr = np.array(self._actuals[i], dtype=np.float64)
            self._cmd_curves[i].setData(t_arr, cmd_arr)
            self._actual_curves[i].setData(t_arr, act_arr)


class JointDualPlotWindow(QWidget):
    """Window with two stacked plots per arm/hand side:

    * **Top** — ``JointPositionPlot``: shows cmd (dashed) + actual (solid) per joint.
    * **Bottom** — ``JointErrorPlot``: shows position error (actual − cmd) per joint.

    Emits ``closed_by_user`` when the window is closed via its X button so the
    owning panel can auto-stop the analyzer and save.
    """

    closed_by_user = Signal()

    def __init__(
        self,
        joint_names: list[str],
        pos_title: str = "Joint action & state",
        err_title: str = "Position error",
        window_sec: float = 20.0,
        y_label_pos: str = "position (rad)",
        y_label_err: str = "error (rad)",
        trend_n: int = 30,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._programmatic_close = False

        self._pos_plot = JointPositionPlot(
            joint_names,
            title=pos_title,
            window_sec=window_sec,
            y_label=y_label_pos,
        )
        self._err_plot = JointErrorPlot(
            joint_names,
            title=err_title,
            window_sec=window_sec,
            y_label=y_label_err,
            trend_n=trend_n,
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        layout.addWidget(self._pos_plot)
        layout.addWidget(self._err_plot)

    def add_sample(self, t: float, cmd: list[float], actual: list[float]) -> None:
        self._pos_plot.add_sample(t, cmd, actual)
        errors = [a - c for a, c in zip(actual, cmd)]
        self._err_plot.add_sample(t, errors)

    def close_programmatically(self) -> None:
        self._programmatic_close = True
        self.close()

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802
        if not self._programmatic_close:
            self.closed_by_user.emit()
        super().closeEvent(event)

    def save_png(self, path: Path) -> None:
        """Capture the entire window (both plots) as a single PNG."""
        pixmap = self.grab()
        pixmap.save(str(path))
