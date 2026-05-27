# M1: Skeleton

## Goal

Set up the `user_ui/` git repository with git-flow style branches, and create
the minimum PySide6 application skeleton — a main window with a left panel,
right panel, and a log area at the bottom. No real functionality yet; this
milestone only proves the project boots.

## What was done

- Initialized a nested git repo at `user_ui/` (independent of the outer
  `openarm_ros2` repo). Branches:
  - `main` — bootstrap commit (README, .gitignore, requirements.txt)
  - `develop` — integration branch, forked from `main`
  - `feature/m1-skeleton` — this milestone, forked from `develop`
- Added the Python package `openarm_launcher/`:
  - `__main__.py` — `python -m openarm_launcher` entry point
  - `main_window.py` — `QMainWindow` with horizontal splitter
    (`LauncherPanel` | `AnalyzerPanel`) on top and a read-only
    `QPlainTextEdit` log on the bottom
  - `launcher/launcher_panel.py` — placeholder `QGroupBox`
  - `analyzers/analyzer_panel.py` — placeholder `QGroupBox`
- Listed runtime deps in `requirements.txt`: PySide6, pyqtgraph, pyudev, numpy.

## How to verify

```bash
cd user_ui/
pip install --user -r requirements.txt   # or just: pip install --user PySide6
python3 -m openarm_launcher
```

Expected: a 1100×720 window opens, showing:

- Left half: "Launcher" group box with placeholder text
- Right half: "Analyzers" group box with placeholder text
- Bottom: empty log area

Smoke test (no display required):

```bash
QT_QPA_PLATFORM=offscreen python3 -c "
import sys
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from openarm_launcher.main_window import MainWindow
app = QApplication(sys.argv); win = MainWindow(); win.show()
QTimer.singleShot(200, app.quit); sys.exit(app.exec())
"
```

## Known issues / TODO

- Panels are placeholders only. Real content lands in M3–M7.
- No log routing yet — `MainWindow.log` is allocated but nothing writes to it.
  Wired up in M2.
- No `run.sh` yet; will be added in M5 once `ros2 launch` integration exists.

## Files touched

```
user_ui/
├── README.md                                      (new on main)
├── .gitignore                                     (new on main)
├── requirements.txt                               (new on main)
├── docs/m1-skeleton.md                            (this file)
└── openarm_launcher/
    ├── __init__.py
    ├── __main__.py
    ├── main_window.py
    ├── launcher/
    │   ├── __init__.py
    │   └── launcher_panel.py
    └── analyzers/
        ├── __init__.py
        └── analyzer_panel.py
```
