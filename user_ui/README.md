# OpenArm User UI

A PySide6 GUI launcher and live analyzer for the OpenArm + O6 bimanual
system. Operators get a single window to bring up CAN, start `ros2 launch`,
and watch / record live joint-error telemetry — without remembering the
shell aliases.

## Features

- **CAN status & one-click bring-up.** Live LED indicators for `can0`–`can3`
  via udev events; a single button runs the six `ip link` configuration
  steps with NOPASSWD sudo.
- **`ros2 launch` integration.** Pick a controller (`forward_position` /
  `joint_trajectory`) and `use_fake_hardware` from dropdowns; launch and
  stop the OpenArm + O6 bimanual stack without typing.
- **Live analyzers.** Per-joint error plots for the arm (7 joints) and
  O6 hand (6 joints), rolling 20 s window. CSV + PNG are written every time
  you press Stop.
- **Settings remembered.** Last-used controller, fake-hardware flag, and
  arm/hand selections persist across restarts via `QSettings`.

## Install

```bash
pip install --user -r requirements.txt
```

System dependencies: a ROS 2 distribution (Humble or newer) and the
`openarm_bringup` package installed in your workspace.

One-time, to make CAN bring-up password-free:

```bash
sudo ./install_sudoers.sh
```

## Run

```bash
# In a shell with your ROS 2 workspace sourced:
source /opt/ros/humble/setup.bash         # or your distro
source ~/ros2_ws_yh/install/setup.bash
./run.sh
```

`run.sh` refuses to start if `ros2` isn't on `$PATH`.

## Typical workflow

1. Plug the USB-CAN adapter (provides `can0` / `can1`) — the corresponding
   LEDs turn amber.
2. Plug PCAN (provides `can2` / `can3`) — those LEDs turn amber.
3. Click **Bring up CAN**. All four LEDs go green.
4. Pick controller mode in the launcher panel; click **Launch OpenArm**.
5. (Optional) In the analyzer panel, pick an arm / hand and click
   **Start**. A plot window appears; CSV is being written.
6. When done, click **Stop** on each running analyzer; PNG + CSV are
   saved under `user_ui/data/`.
7. Click **Open data folder** to inspect.

## Data layout

```
user_ui/data/
├── arm/csv/YYYY-MM-DD/{side}_{ts}.csv     # cmd / actual / error per joint
├── arm/img/YYYY-MM-DD/{side}_{ts}.png     # plot screenshot at Stop
├── o6/csv/YYYY-MM-DD/...
└── o6/img/YYYY-MM-DD/...
```

`data/` is git-ignored.

## Repository layout

This directory is its own git repository (nested inside `openarm_ros2`),
managed git-flow style. Each milestone lived on a dedicated
`feature/mN-...` branch and was merged into `develop`. See `docs/` for
per-milestone notes and `docs/architecture.md` for the big picture.

## License

PySide6 is used under LGPL v3 — commercial distribution is permitted as
long as the LGPL terms are honoured (dynamic linking is the default).
