# M3: CAN interface detection (udev + status LEDs)

## Goal

Replace the M2 placeholder launcher panel with live CAN interface status.
Track presence and operstate of `can0`..`can3` via `pyudev` and reflect each
into a coloured LED + status label in the UI.

## What was done

- `openarm_launcher/launcher/can_detect.py` — `CanDetector(QObject)`:
  - Holds a `pyudev.Monitor` filtered on the `net` subsystem, plugged into
    the Qt event loop via `QSocketNotifier` on the monitor's fd.
  - `state(name, operstate)` signal where `operstate` is `"up"`, `"down"`
    (or other kernel string), or `None` (interface absent).
  - `emit_current_state()` does an initial sysfs sweep for the four tracked
    interfaces — used to populate the UI without waiting for udev events.
  - Helper `read_operstate(name)` returns the value of
    `/sys/class/net/<name>/operstate` or None.
- `openarm_launcher/launcher/launcher_panel.py` — rewritten:
  - Four rows, one per CAN interface. Each row: a `StatusLed` (small
    coloured `QFrame`), the interface name, a role caption (e.g.
    "OpenArm right (CAN-FD)"), and a textual status on the right.
  - Owns a `CanDetector`; the panel translates `state(name, operstate)` to:
    - `None` → grey LED, "not detected"
    - `"up"` → green LED, "UP"
    - anything else → amber LED, that state string
  - Each transition also emits `log_line` so the log shows event history.
  - **M2 test button removed** (this was promised in M2 docs).

## Design notes & gotchas

- **Deferred initial scan.** `LauncherPanel.__init__` does
  `QTimer.singleShot(0, self._detector.emit_current_state)` instead of
  calling it directly. Reason: the first scan would otherwise fire `state`
  signals (and downstream `log_line` signals) synchronously during widget
  construction — before `MainWindow` had a chance to wire
  `log_line → log.appendPlainText`. The first four log lines were silently
  lost. Deferring with a 0-ms timer hands control back to the event loop,
  where all wiring is complete.
- **Why `QSocketNotifier`, not a thread.** `pyudev.Monitor` exposes a
  pollable file descriptor; integrating it with the existing Qt event loop
  is one line and stays in the GUI thread. A background thread would force
  signal-thread juggling for no benefit at this rate of events.
- **`object` payload type.** The `state(str, object)` signal carries either
  `str` or `None`; using `object` keeps the signature single without needing
  a sentinel string for absence.
- **Operstate is read live on every event.** `udev` on its own would tell
  us "interface added/changed/removed"; the actual UP/DOWN state still
  has to be re-read from sysfs at that moment. Bundling that read into the
  signal payload means the panel never needs to ask sysfs itself.

## How to verify

Headless smoke test (works because the host currently has can0..can3 UP):

```bash
cd user_ui
QT_QPA_PLATFORM=offscreen python3 -u -c "
import sys
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from openarm_launcher.main_window import MainWindow

app = QApplication(sys.argv); win = MainWindow(); win.show()
def check():
    text = win.log.toPlainText()
    for n in ('can0','can1','can2','can3'):
        assert n in text, f'missing {n}'
    print('M3_OK'); app.quit()
QTimer.singleShot(300, check); app.exec()
"
```

Manual (this is the milestone-defining check):

```bash
python3 -m openarm_launcher
# Plug/unplug a USB-CAN adapter.
# The corresponding row's LED should flip:
#   - removal:    green/amber  →  grey, label "not detected"
#   - re-insert:  grey         →  amber "down" (or green "up" after `ip link set <name> up`)
# Each transition writes a [can] line into the log.
```

## Known issues / TODO

- LEDs only have three states (off / down / up). M4 will add an interim
  "configuring..." state while `ip link` is in flight.
- No de-bounce — udev "change" events can fire several times during
  bring-up. Currently each fires a separate log line. Not a problem until
  it becomes noisy.
- The `StatusLed` colour palette is hard-coded; if we want a theme later it
  should move to `common/style.py`. Not needed yet.

## Files touched

```
user_ui/
├── docs/m3-can-detect.md                          (new)
└── openarm_launcher/launcher/
    ├── can_detect.py                              (new)
    └── launcher_panel.py                          (rewritten — no M2 button)
```
