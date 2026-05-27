# M4: Sudoers NOPASSWD + CAN bring-up

## Goal

Click a single "Bring up CAN" button in the launcher panel and have all four
CAN interfaces brought up with the correct bitrate / FD configuration — with
no password prompt at runtime. A one-time install of a NOPASSWD sudoers rule
makes the runtime experience seamless.

## What was done

- **`install_sudoers.sh`** (project root):
  - Run once with `sudo ./install_sudoers.sh`. Refuses to run if not root
    or if invoked directly as root (we need `$SUDO_USER` to know whom to
    grant).
  - Writes `/etc/sudoers.d/openarm-ui` after validating it with `visudo -c`.
  - Grants the invoking user NOPASSWD for `/usr/sbin/ip link set canN *`
    (matches can0–can9 plus can10–can99 patterns).
- **`openarm_launcher/launcher/can_setup.py`** — `CanBringUp(QObject)`:
  - Holds a single `ManagedProcess` and walks the 6-step bring-up
    sequence on each `finished` signal.
  - Each step runs `sudo -n /usr/sbin/ip link set ...`. `-n` is
    non-interactive — if NOPASSWD isn't installed, the command fails
    immediately with `sudo: a password is required` instead of hanging.
  - Signals: `line(str)` per output line, `progress(str)` per step label,
    `finished(bool)` with overall success.
- **`launcher_panel`**: adds a `Bring up CAN (can0..can3)` button below the
  status rows. While bring-up is in flight the button is disabled and the
  label changes to "Bringing up CAN...". On completion the panel emits
  either a "done" or a "FAILED" message — the failure message points the
  user at `install_sudoers.sh`.

## Bring-up sequence

The actual `ip link` calls match the user's existing aliases:

```
ip link set can0 up type can bitrate 1000000          # O6 right
ip link set can1 up type can bitrate 1000000          # O6 left
ip link set can2 type can bitrate 1000000 dbitrate 5000000 fd on   # OpenArm cfg
ip link set can3 type can bitrate 1000000 dbitrate 5000000 fd on
ip link set can2 up                                   # OpenArm bring-up
ip link set can3 up
```

The arm interfaces are configured-then-brought-up in two steps because most
kernels reject `up` + `type` in one command on CAN-FD links.

## How to verify

One-time setup (only this needs a password):

```bash
cd user_ui
sudo ./install_sudoers.sh
sudo -n /usr/sbin/ip link show can0   # should print without prompting
```

Manual run:

```bash
python3 -m openarm_launcher
# Click "Bring up CAN (can0..can3)".
# All four LEDs should turn green within ~1 s. If sudoers is missing the
# log shows: "sudo: a password is required" and a hint to run install_sudoers.sh.
```

Headless wiring test (no sudoers required — just proves the failure path
is correctly reported):

```bash
QT_QPA_PLATFORM=offscreen python3 -u -c "
import sys
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from openarm_launcher.main_window import MainWindow
app = QApplication(sys.argv); win = MainWindow(); win.show()
win.launcher_panel._bringup_btn.click()
QTimer.singleShot(2000, app.quit); app.exec()
print(win.log.toPlainText())
"
```

Expected tail without sudoers installed:
```
sudo: a password is required
[error] step 'can0' exited 1
--- CAN bring-up FAILED. ... run `sudo ./install_sudoers.sh` once. ---
```

## Design notes

- **Why `sudo -n` and not `pkexec`?**
  `pkexec` requires a polkit policy file (XML) and pops a GUI password
  dialog every invocation by default. `sudo -n` + a single sudoers entry
  is shorter, cheaper, and matches what the user is already doing in
  their shell aliases.
- **Sequencing.** Each step waits for the previous `ManagedProcess.finished`
  before starting the next. This keeps the GUI responsive (no `time.sleep`)
  and surfaces failures step-by-step in the log.
- **Idempotency.** Running bring-up twice will typically just re-`up` the
  links; the udev "change" events feed back into `CanDetector` and the
  LEDs update accordingly. No special-case handling needed.
- **Sudoers wildcard scope.** The allowed pattern (`ip link set canN *`)
  cannot run arbitrary commands — only `ip link set <canN-style-name> ...`
  invocations. We deliberately do *not* allow `ip link delete` or other
  subcommands.

## Known issues / TODO

- The button is always enabled, even when no CAN interfaces are detected.
  Could be tied to `CanDetector` state (disable until at least one
  interface is present). Deferred — minor UX nit.
- No "Bring down CAN" inverse — not needed for ops workflow.
- M5's launch button will likely want to call `CanBringUp` automatically if
  any interface is still down. Left to M5 to decide.

## Files touched

```
user_ui/
├── docs/m4-sudoers-canup.md                       (new)
├── install_sudoers.sh                             (new, +x)
└── openarm_launcher/launcher/
    ├── can_setup.py                               (new — CanBringUp)
    └── launcher_panel.py                          (modified — button + wiring)
```
