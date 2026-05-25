# M7: O6 hand analyzer

## Goal

Mirror the arm analyzer (M6) for the O6 hand: pick `right` / `left`,
start, watch a live 6-joint error plot, and have CSV + PNG written to
`user_ui/data/o6/` on stop.

## What was done

- **`openarm_launcher/analyzers/o6_runner.py`** — `O6Runner(QObject)`:
  - Constants `O6_RIGHT_JOINTS` / `O6_LEFT_JOINTS` (6 joints per hand,
    matching the existing `analy_o6_hand.py` CLI tool).
  - `_O6Node(rclpy.Node)` subscribes to
    `/<side>_hand_controller/controller_state` (cmd from `reference` with
    `desired` fallback) and `/joint_states` (actual). Emits 20 Hz samples
    via callback once both sides have seen data.
  - Re-uses `ensure_rclpy` from `arm_runner` so rclpy is still initialised
    at most once per process.
- **`openarm_launcher/analyzers/analyzer_panel.py`** — adds an O6 section
  symmetric to the arm section: hand picker, Start / Stop pair.
  Re-uses `ArmCsvWriter` and `JointErrorPlot` unchanged — both are joint-
  generic, no rename needed.
- Data path: `data/o6/csv/YYYY-MM-DD/{hand}_{ts}.csv` and
  `data/o6/img/YYYY-MM-DD/{hand}_{ts}.png`. The arm/o6 split lives in
  `analyzer_panel`'s `ARM_DATA_ROOT` / `O6_DATA_ROOT` constants.

## Design notes

- **No abstraction layer for arm/o6 yet.** The arm and o6 runners are 90 %
  duplicate code with three concrete differences (node name prefix,
  controller topic prefix, joint name set). At two callsites this is fine.
  If a third hand/manipulator subscriber appears we should hoist a
  `JointSubscriber` base class.
- **`ArmCsvWriter` is joint-agnostic.** Its only inputs are the data root
  and a list of joint names; "Arm" in the class name is now historical.
  Renaming would require touching the arm runner imports too, which buys
  little — left as a TODO if the name confuses anyone.
- **Independent state on the panel.** The arm and o6 lifecycles are fully
  independent — both can run simultaneously without interfering. They are
  separate `Runner` instances spinning separate rclpy nodes; rclpy only
  needs `init` once per process.

## How to verify

Headless smoke test (verifies imports, joint-name shapes, and panel
construction — does not start any ROS subscription):

```bash
cd user_ui
QT_QPA_PLATFORM=offscreen python3 -u -c "
import sys
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from openarm_launcher.main_window import MainWindow
from openarm_launcher.analyzers.o6_runner import (
    O6_RIGHT_JOINTS, O6_LEFT_JOINTS, o6_joints_for,
)
assert len(O6_RIGHT_JOINTS) == 6 and len(O6_LEFT_JOINTS) == 6
assert o6_joints_for('right') == O6_RIGHT_JOINTS

app = QApplication(sys.argv); win = MainWindow(); win.show()
panel = win.analyzer_panel
assert panel._o6_start.isEnabled() and not panel._o6_stop.isEnabled()
QTimer.singleShot(200, app.quit); app.exec()
print('M7_OK')
"
```

Manual:

```bash
./run.sh
# After launching the O6 stack (M5):
# - Right panel, "O6 hand analyzer": pick 'right', click Start.
# - A new window pops up: live 6-line plot (thumb_cmc_pitch/yaw, four mcp).
# - Click Stop. Log shows two [saved] lines under data/o6/.
```

## Known issues / TODO

- Single hand at a time. Picking 'both' is not exposed — would need two
  runners and a 12-line plot.
- Same as M6: only action mode supported.
- If users want to compare arm and o6 latency side-by-side, the data is
  there in CSVs but no combined view in the GUI.

## Files touched

```
user_ui/
├── docs/m7-analyzer-o6.md                         (new)
└── openarm_launcher/analyzers/
    ├── o6_runner.py                               (new)
    └── analyzer_panel.py                          (modified — O6 section)
```
