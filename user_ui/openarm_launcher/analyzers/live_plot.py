from pathlib import Path

import numpy as np
import pyqtgraph as pg
import pyqtgraph.exporters
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)


pg.setConfigOptions(antialias=True)


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
        self._t.append(t)
        for i, e in enumerate(errors):
            self._errs[i].append(e)
        t_cut = t - self._window_sec
        while self._t and self._t[0] < t_cut:
            self._t.pop(0)
            for arr in self._errs:
                arr.pop(0)
        for i, curve in enumerate(self._curves):
            curve.setData(self._t, self._errs[i])
            self._trend_curves[i].setData(
                self._t, _rolling_mean(self._errs[i], self._trend_n)
            )

    def save_png(self, path: Path) -> None:
        exporter = pyqtgraph.exporters.ImageExporter(self._plot.getPlotItem())
        exporter.parameters()["width"] = 1200
        exporter.export(str(path))
