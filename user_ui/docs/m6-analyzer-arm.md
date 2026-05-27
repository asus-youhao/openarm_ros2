# M6: Arm analyzer (embedded pyqtgraph + CSV/PNG save)

## Goal

Right side of the GUI: pick an arm (`right` / `left`), click Start to bring up
a live position-error plot in a separate window, and have CSV + PNG written
to `user_ui/data/arm/` when the user clicks Stop.

## What was done

- **`openarm_launcher/analyzers/csv_writer.py`** — `ArmCsvWriter`:
  - Creates `data/arm/csv/YYYY-MM-DD/{side}_{ts}.csv` on init.
  - Columns: `t_s, <joint>_cmd, <joint>_actual, <joint>_err` for each
    of the 7 arm joints (21 + 1 = 22 columns).
  - `write(t, cmd, actual)` writes one row; `close()` flushes & closes.
- **`openarm_launcher/analyzers/live_plot.py`** — `JointErrorPlot(QWidget)`:
  - One `pyqtgraph.PlotWidget` with N curves (one per joint) coloured by
    a perceptually-uniform colormap.
  - Rolling window of `window_sec` (20 s default) — old samples are
    popped to keep memory bounded.
  - `save_png(path)` uses `pyqtgraph.exporters.ImageExporter` at width 1200.
- **`openarm_launcher/analyzers/arm_runner.py`** — `ArmRunner(QObject)`:
  - Internal `_ArmNode(rclpy.Node)` subscribes to
    `/<side>_joint_trajectory_controller/controller_state` (cmd from
    `reference.positions`, falling back to `desired.positions` for older
    distros) and `/joint_states` (actual positions).
  - A 20 Hz timer fires the latest `(t, cmd, actual)` triplet via callback
    once both sides have seen data.
  - The node runs inside a `SingleThreadedExecutor` on a dedicated thread;
    `ArmRunner.sample(t, cmd, actual)` is the cross-thread Qt signal.
  - Module-level `ensure_rclpy()` / `shutdown_rclpy()` keep `rclpy.init()`
    idempotent per process. `shutdown_rclpy` is hooked to
    `QApplication.aboutToQuit` in `__main__.py`.
- **`openarm_launcher/analyzers/analyzer_panel.py`** — rebuilt:
  - Arm section: arm combo, Start / Stop button pair.
  - O6 section: placeholder label (M7).
  - "Open data folder" button uses `QDesktopServices.openUrl` on
    `user_ui/data/` (full polish in M8).
  - Owns the `ArmRunner`, `ArmCsvWriter`, and plot window; manages the
    lifecycle (create on Start, save+close on Stop).
- **`main_window.py`** — connects `analyzer_panel.log_line → log.appendPlainText`.

## Design notes

- **Plot lives in its own window**, not embedded in the splitter. Reasons:
  resizable independent of the launcher; can drag onto a second screen;
  closing it doesn't kill the main app. Matches how the existing CLI
  analyzer behaves.
- **Cross-thread signal safety.** `ArmRunner.sample.emit(...)` is invoked
  from the ROS spin thread. PySide6's default `AutoConnection` will route
  it through Qt's event loop into the GUI thread automatically.
- **Reference field probing.** `JointTrajectoryControllerState` had a
  rename — older builds use `desired`, newer use `reference`. We try
  `reference` then `desired` so the same code works across distros.
- **Bounded memory.** The rolling window only keeps `window_sec` of points
  in the plot. The CSV is the full record.
- **rclpy init/shutdown.** rclpy can only be initialised once per process.
  A module-level lock guards the init flag. Shutdown runs once on
  `aboutToQuit`. Re-init after shutdown is not supported here (not needed
  — the user closes the GUI to fully restart).

## How to verify

Headless smoke test (no ROS publishers needed — tests CSV + plot + panel
construct, without spawning rclpy):

```bash
cd user_ui
QT_QPA_PLATFORM=offscreen python3 -u -c "
import tempfile, shutil
from pathlib import Path
from PySide6.QtWidgets import QApplication
from openarm_launcher.analyzers.csv_writer import ArmCsvWriter
from openarm_launcher.analyzers.live_plot import JointErrorPlot

# CSV
tmp = Path(tempfile.mkdtemp())
w = ArmCsvWriter(tmp, 'right', ['j1','j2','j3'])
w.write(0.0, [0.1]*3, [0.11]*3); w.close()
assert w.path.exists() and w.path.stat().st_size > 0
shutil.rmtree(tmp)

# Plot
app = QApplication([])
p = JointErrorPlot(['a','b','c']); p.show()
for i in range(10): p.add_sample(i*0.05, [0.01*i, -0.02*i, 0.005*i])
out = Path(tempfile.mkdtemp()) / 'out.png'
p.save_png(out); assert out.stat().st_size > 1000
shutil.rmtree(out.parent)
print('M6_OK')
"
```

Manual (with the OpenArm controllers running via M5):

```bash
./run.sh
# 1. Launch OpenArm via the left panel (M5).
# 2. Right panel: pick 'right', click 'Start arm analyzer'.
#    A new window appears with a live 7-line error plot.
# 3. Wiggle the arm or send a trajectory.
# 4. Click 'Stop'. Log shows:
#      [saved] .../user_ui/data/arm/img/<date>/right_<ts>.png
#      [saved] .../user_ui/data/arm/csv/<date>/right_<ts>.csv
# 5. Click 'Open data folder' to inspect.
```

## Known issues / TODO

- Only "action mode" (controller_state) supported. The CLI analyzer also
  supports "topic mode" and "telep mode"; not needed for the GUI yet.
- Single arm at a time. Picking 'both' is not exposed — would need two
  runners and a 14-line plot, which we'd defer until a user actually asks.
- No live "expected vs actual" overlay — only error. Add only if users ask.
- The CSV row is written from the GUI thread (via `sample` signal). At
  20 Hz this is comfortably below any throughput concern; if rates rise
  we should batch.
- "Open data folder" is a thin one-liner; richer behaviour (e.g. "open
  latest CSV directly") is in scope for M8.

## Files touched

```
user_ui/
├── docs/m6-analyzer-arm.md                        (new)
└── openarm_launcher/
    ├── __main__.py                                (modified — aboutToQuit hook)
    ├── main_window.py                             (modified — analyzer log wire)
    └── analyzers/
        ├── arm_runner.py                          (new)
        ├── csv_writer.py                          (new)
        ├── live_plot.py                           (new)
        └── analyzer_panel.py                      (rewritten)
```
