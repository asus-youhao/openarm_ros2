from pathlib import Path

import pyqtgraph as pg
import pyqtgraph.exporters
from PySide6.QtWidgets import QVBoxLayout, QWidget


pg.setConfigOptions(antialias=True)


class JointErrorPlot(QWidget):
    """Rolling-window plot of position error per joint.

    Adds one curve per joint, keeps the last `window_sec` of samples,
    and exposes `save_png(path)` to dump the current view.
    """

    def __init__(
        self,
        joint_names: list[str],
        title: str = "Joint error",
        window_sec: float = 20.0,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._joint_names = joint_names
        self._window_sec = window_sec

        self._plot = pg.PlotWidget(background="w")
        self._plot.setTitle(title)
        self._plot.setLabel("left", "error (rad)")
        self._plot.setLabel("bottom", "t (s)")
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.addLegend()

        # Distinct colours per joint.
        cmap = pg.colormap.get("CET-C2") or pg.colormap.get("hsv")
        lut = cmap.getLookupTable(0.0, 1.0, len(joint_names))
        self._curves: list = []
        for i, name in enumerate(joint_names):
            color = (int(lut[i, 0]), int(lut[i, 1]), int(lut[i, 2]))
            curve = self._plot.plot([], [], pen=pg.mkPen(color, width=1.5), name=name)
            self._curves.append(curve)

        self._t: list[float] = []
        self._errs: list[list[float]] = [[] for _ in joint_names]

        layout = QVBoxLayout(self)
        layout.addWidget(self._plot)

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

    def save_png(self, path: Path) -> None:
        exporter = pyqtgraph.exporters.ImageExporter(self._plot.getPlotItem())
        exporter.parameters()["width"] = 1200
        exporter.export(str(path))
