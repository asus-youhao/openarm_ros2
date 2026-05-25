# OpenArm User UI

A PySide6 GUI launcher and live analyzer for the OpenArm + O6 bimanual system.

Aimed at operators / engineers who want one-click startup without manually
sequencing USB-CAN, PCAN, and `ros2 launch` commands.

## Status

Work-in-progress. Developed milestone-by-milestone on independent feature
branches (git-flow style). See `docs/` for per-milestone notes.

## Layout

```
user_ui/
├── data/                  # runtime data (gitignored)
├── docs/                  # per-milestone notes
└── openarm_launcher/      # the Qt application package
```

## Running (dev)

```bash
pip install --user -r requirements.txt
python3 -m openarm_launcher
```

## License

Internal project. PySide6 is used under LGPL.
