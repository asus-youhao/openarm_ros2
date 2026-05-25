# Architecture

A short tour of how the pieces fit, for anyone landing on this repo for
the first time.

## Process model

One Python / PySide6 process. Three threads of execution exist:

- **Main / GUI thread** — Qt event loop, all widgets, signal slots that
  paint anything.
- **rclpy spin thread(s)** — one per active analyzer (`ArmRunner`,
  `O6Runner`), each running a `SingleThreadedExecutor` over its node.
  Cross-thread data hand-off is exclusively via Qt signals
  (`AutoConnection` → queued).
- **`QProcess` workers** — `ip link`, `ros2 launch`, etc. are child
  processes. Their stdout is read on the GUI thread via
  `readyReadStandardOutput`.

```
                ┌──────────── MainWindow ────────────┐
                │                                    │
                │  LauncherPanel       AnalyzerPanel │
                │      │ log_line          │ log_line│
                │      ▼                   ▼         │
                │      QPlainTextEdit (the log)      │
                └────┬────────────────────┬──────────┘
                     │                    │
   QProcess  ◀───────┤                    ├──────▶  rclpy nodes
   (ip link,         │                    │        (arm, o6) on
    ros2 launch)     │                    │        worker threads
                     │                    │
              ManagedProcess        Arm/O6 Runner
              (proc.py)             (arm_runner.py,
                                     o6_runner.py)
```

## Package layout

```
openarm_launcher/
├── __main__.py              # QApplication entry + aboutToQuit hook
├── main_window.py           # window + log + closeEvent
├── common/proc.py           # ManagedProcess (QProcess wrapper)
├── launcher/
│   ├── can_detect.py        # CanDetector (pyudev + QSocketNotifier)
│   ├── can_setup.py         # CanBringUp (6-step sudo ip-link runner)
│   ├── ros2_launcher.py     # Ros2Launcher (ManagedProcess of ros2 launch)
│   └── launcher_panel.py    # left UI: CAN status + bring-up + launch
└── analyzers/
    ├── arm_runner.py        # ArmRunner (+ rclpy lifecycle helpers)
    ├── o6_runner.py         # O6Runner (same shape, hand topics)
    ├── csv_writer.py        # ArmCsvWriter (joint-generic)
    ├── live_plot.py         # JointErrorPlot (pyqtgraph)
    └── analyzer_panel.py    # right UI: arm + o6 sections + open folder
```

## Why each major decision

- **PySide6 (LGPL).** Commercial-friendly without per-seat licensing.
- **Nested git repo.** `user_ui/` is portable — it can be moved into a
  separate top-level repo later without disrupting `openarm_ros2`'s
  history. Develop flow keeps milestone diffs reviewable.
- **`sudo -n` + sudoers entry.** No password at runtime, but no daemon to
  install either. `pkexec` would have required a polkit policy file.
- **`pyudev` over polling.** Real-time CAN insertion / removal events
  with one `QSocketNotifier`, no busy loop.
- **`QProcess.MergedChannels`.** The log displays an interleaved stream
  — we don't need stderr highlighting yet.
- **rclpy in a worker thread, signals back.** Avoids any Qt event-loop
  contention; PySide6's `AutoConnection` does the right thread crossing.
- **Plot as its own window.** Lets the operator move it to a second
  monitor and resize independently of the launcher.
- **No premature abstraction over arm/o6 runners.** They're 90 % shared
  code but only differ in three concrete places. With two callsites a
  base class is not yet earning its keep.

## What's deliberately not here

- No auto-source of `setup.bash` — too many possible workspaces; the
  user controls the environment via `run.sh`'s shell.
- No supervisor that "auto-bringup-then-launch" — bring-up is a
  one-off, the operator sees it once and remembers.
- No remote / multi-host support. Single-machine operator console only.
- No telemetry beyond the log + CSV — if metrics are needed later they
  belong in a separate prometheus exporter, not this UI.
