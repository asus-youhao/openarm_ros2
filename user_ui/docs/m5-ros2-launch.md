# M5: ros2 launch integration

## Goal

Add a "Launch OpenArm" / "Stop" pair to the launcher panel that runs
`ros2 launch openarm_bringup openarm_o6_bimanual.launch.py` with the
appropriate `right_can_interface` / `left_can_interface` /
`right_o6_can_interface` / `left_o6_can_interface` arguments preset, plus
a controller-mode dropdown and a `use_fake_hardware` checkbox. Add a
`run.sh` entry point that refuses to start if `ros2` isn't sourced.

## What was done

- **`openarm_launcher/launcher/ros2_launcher.py`** — `Ros2Launcher`:
  - Wraps a single `ManagedProcess` running `ros2 launch ...`.
  - `CONTROLLERS` maps UI labels to argument values:
    `"Forward Position" → forward_position_controller`,
    `"Joint Trajectory" → joint_trajectory_controller`.
  - CAN-interface mapping is hard-coded to match the user's existing
    aliases (`can2`/`can3` for arms, `can0`/`can1` for O6 hands). If we
    ever need to change this, it's the only place to edit.
  - `start(controller_label, use_fake_hardware: bool)`,
    `stop()`, `is_running()`.
  - Re-exposes `line(str)`, `started()`, `finished(int)` signals.
- **`launcher_panel`** additions:
  - `QComboBox` for controller selection
  - `QCheckBox` for `use_fake_hardware`
  - `Launch OpenArm` button — disabled while a launch is running
  - `Stop` button — disabled until `started` fires; calls
    `ManagedProcess.stop()` (terminate-then-kill).
  - All output (start/end markers, line-by-line ros2 output, exit code)
    flows through `log_line`.
- **`run.sh`** — pre-flight checks `command -v ros2`. If missing, prints
  a hint to source `setup.bash` and exits non-zero. Otherwise `cd`s into
  `user_ui/` and `exec python3 -m openarm_launcher`.

## Button state machine

```
       click Launch     proc started     proc finished
idle ──────────────▶ pending ─────────▶ running ─────────▶ idle
                                                ▲
       click Stop ─────────────────────────────┘
       (proc.stop() → terminate → kill)
```

Slot mapping:

| Event              | `Launch` btn | `Stop` btn |
|--------------------|:------------:|:----------:|
| idle               | enabled      | disabled   |
| Launch clicked     | disabled     | disabled   |
| `started` signal   | disabled     | enabled    |
| Stop clicked       | disabled     | disabled (transient) |
| `finished` signal  | enabled      | disabled   |

The "pending" gap (clicked but not yet started) is real — `QProcess.start`
returns immediately and the `started` signal arrives a moment later. Both
buttons stay off in this window, which is the right behaviour.

## How to verify

Manual (real hardware):

```bash
cd user_ui
source /opt/ros/humble/setup.bash      # adjust to your distro
source ~/ros2_ws_yh/install/setup.bash
./run.sh
# 1. Click "Bring up CAN" (from M4) — wait for all four LEDs green.
# 2. Pick a controller, optionally tick use_fake_hardware.
# 3. Click "Launch OpenArm" — log fills with ros2 launch output.
# 4. Click "Stop" — process terminates cleanly; log ends with exit code.
```

Manual (fake hardware, no real CAN required):

```bash
./run.sh
# Skip CAN bring-up; tick "use_fake_hardware"; click Launch.
# Controllers should come up against the fake hardware plugin.
```

Headless wiring test (no actual ros2 launch — uses a fake long-running
command to exercise the state transitions):

```bash
cd user_ui
QT_QPA_PLATFORM=offscreen python3 -u -c "
import sys
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from openarm_launcher.main_window import MainWindow

app = QApplication(sys.argv); win = MainWindow(); win.show()
p = win.launcher_panel
def fake_start(label, fake):
    p._launcher._proc.start('bash', ['-c', 'echo fake_running; sleep 5'])
p._launcher.start = fake_start
p._launch_btn.click()
def go_stop():
    assert not p._launch_btn.isEnabled() and p._stop_btn.isEnabled()
    p._stop_btn.click()
    QTimer.singleShot(800, lambda: (
        sys.stdout.write('M5_OK\n'), app.quit()
    ))
QTimer.singleShot(400, go_stop); app.exec()
"
```

Expected: `M5_OK`.

## Known issues / TODO

- `run.sh` does not auto-source `setup.bash`. The user must source it.
  Reason: we don't know which workspaces are relevant. If this becomes
  annoying we'll add a `--source` flag or a config file in M8.
- No "auto-stop on app quit" — if the user closes the GUI window while a
  launch is running, the child process keeps going. Will be wired up in
  M8 via `closeEvent`.
- No coupling between CAN bring-up and Launch. The user can click Launch
  before bring-up, which will likely fail inside the controller node.
  M8 may add a pre-flight guard.

## Files touched

```
user_ui/
├── docs/m5-ros2-launch.md                         (new)
├── run.sh                                         (new, +x)
└── openarm_launcher/launcher/
    ├── ros2_launcher.py                           (new — Ros2Launcher)
    └── launcher_panel.py                          (modified — Launch/Stop UI)
```
