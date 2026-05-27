# Split CAN bring-up into three buttons

## Motivation

Operators reported that the hardware needs to be physically plugged in in
order: USB-CAN for `can0`, then USB-CAN for `can1`, then PCAN for
`can2`/`can3`. With a single combined bring-up button, the whole sequence
fails on the first missing adapter. Splitting the button lets the
operator bring up each adapter the moment it appears.

## What changed

- **`launcher/can_setup.py`**:
  - Three module-level recipes now live alongside `CanBringUp`:
    `STEPS_O6_RIGHT` (can0), `STEPS_O6_LEFT` (can1),
    `STEPS_OPENARM` (can2 cfg → can3 cfg → can2 up → can3 up).
  - `CanBringUp.__init__` now takes a `steps` argument instead of using
    a class-level constant, so callers can stamp out one per recipe.
- **`launcher/launcher_panel.py`**:
  - Replaces the single "Bring up CAN" button with three stacked
    buttons driven by a small `_BRINGUP_SPECS` table.
  - Each button has its own independent `CanBringUp` instance. While
    one is running, only that button is disabled — the others remain
    clickable, so the operator can run them in parallel if they want.
  - Click/done handlers are shared and take `(button, label)` so the
    "...ing" / "FAILED" text adapts per button without copy-paste.

## How to verify

Headless smoke (offscreen) — confirms button count, labels, and the
"one running, others enabled" UX:

```bash
cd user_ui
QT_QPA_PLATFORM=offscreen timeout 10 python3 -u -c "
import sys
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from openarm_launcher.main_window import MainWindow

app = QApplication(sys.argv)
app.setOrganizationName('o'); app.setApplicationName('o')
win = MainWindow(); win.show()
p = win.launcher_panel
labels = [b.text() for (b, _, _) in p._bringups]
assert len(p._bringups) == 3
assert any('can0' in l for l in labels)
assert any('can1' in l for l in labels)
assert any('can2+can3' in l for l in labels)
p._bringups[0][0].click()
assert not p._bringups[0][0].isEnabled()
assert p._bringups[1][0].isEnabled()
QTimer.singleShot(1500, app.quit); app.exec()
print('SPLIT_OK')
"
```

Manual: each button independently invokes its own `ip link` sequence;
failures only disable / re-enable the clicked button.

## Files touched

```
user_ui/
├── docs/split-can-bringup.md                      (new)
└── openarm_launcher/launcher/
    ├── can_setup.py                               (recipes + ctor takes steps)
    └── launcher_panel.py                          (3 buttons via spec table)
```
