# M2: ManagedProcess wrapper + log routing

## Goal

Introduce a thin `QProcess` wrapper that the rest of the app can use to spawn
external commands (`ip link`, `ros2 launch`, ROS2 analyzer subprocesses…) and
stream their stdout/stderr into the UI log line-by-line. Prove the plumbing
end-to-end with a temporary "run `ros2 topic list`" button.

## What was done

- `openarm_launcher/common/proc.py` — `ManagedProcess`:
  - `start(program, args)` / `stop(timeout_ms=2000)` / `is_running()`
  - Signals: `line(str)` per merged stdout+stderr line,
    `started()`, `finished(int)` with the exit code.
  - Channel mode is `QProcess.MergedChannels` (one ordered stream).
  - Line buffering: bytes accumulate in an internal buffer; complete lines
    are split on `\n`, decoded as UTF-8 (`errors="replace"`), and the trailing
    `\r` is stripped. Any leftover bytes are flushed on `finished`.
- `openarm_launcher/launcher/launcher_panel.py` — adds a temporary
  "M2 test: run `ros2 topic list`" button and a `log_line(str)` signal.
  The panel owns a `ManagedProcess` whose `line` and `finished` are
  re-emitted through `log_line`.
- `openarm_launcher/main_window.py` — exposes `launcher_panel` /
  `analyzer_panel` as attributes and connects
  `launcher_panel.log_line → log.appendPlainText`.

## Design notes

- **Why a wrapper instead of using `QProcess` directly everywhere?**
  Every caller would otherwise reimplement: line buffering, signal renaming
  (`readyReadStandardOutput` is awkward), terminate-then-kill fallback,
  guard against double-start, and flushing the tail buffer on exit. The
  wrapper centralizes those four concerns; everything else is delegated.
- **One signal for stdout+stderr.** The log only needs ordered output;
  splitting the channels would force us to interleave them anyway. If a
  future caller (e.g. error highlighting) needs them split, we can add a
  second mode then.
- **Decoupling via Signal.** `LauncherPanel` doesn't know about the log
  widget — it just emits `log_line`. `MainWindow` does the wiring. Same
  shape will be reused in M6/M7 for `AnalyzerPanel`.
- **No global log bus.** With only two producers (launcher + analyzers),
  a direct signal-to-slot connection is clearer than introducing an
  intermediate bus class.

## How to verify

Headless smoke test (`ManagedProcess` line streaming):

```bash
cd user_ui
QT_QPA_PLATFORM=offscreen python3 -c "
import sys
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from openarm_launcher.common.proc import ManagedProcess

app = QApplication(sys.argv)
lines, codes = [], []
p = ManagedProcess()
p.line.connect(lines.append)
p.finished.connect(codes.append)
p.start('bash', ['-c', 'printf \"a\nb\nc\n\"'])
p.finished.connect(lambda *_: QTimer.singleShot(50, app.quit))
app.exec()
assert lines == ['a', 'b', 'c'] and codes == [0]
print('OK')
"
```

End-to-end (panel → log) smoke test in the repo's M2 commit message body.

Manual:

```bash
python3 -m openarm_launcher
# Click the "M2 test: run 'ros2 topic list'" button in the Launcher panel.
# The log should show the topic list (if ROS2 is sourced) followed by "[exit 0]".
# Without ROS2 sourced you will see "[exit <nonzero>]" — which still proves
# the wrapper streams output.
```

## Known issues / TODO

- The "M2 test" button is intentionally temporary. **M3 will replace the
  whole `LauncherPanel` body** with CAN status rows; the button goes away
  then. Tracked in this milestone's exit comment.
- `ManagedProcess` runs commands in the current shell environment. If a
  ROS2 launch needs `source /opt/ros/.../setup.bash` first, M5 will wrap
  the call with `bash -lc` to inherit the user's shell init.
- No stdin support. Not needed by any planned milestone; add when required.

## Files touched

```
user_ui/
├── docs/m2-proc-wrapper.md                        (new)
└── openarm_launcher/
    ├── common/__init__.py                         (new)
    ├── common/proc.py                             (new)
    ├── launcher/launcher_panel.py                 (modified: signal + test btn)
    └── main_window.py                             (modified: log wiring)
```
