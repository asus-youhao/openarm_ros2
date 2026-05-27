import shutil
import subprocess

from PySide6.QtCore import QObject, Signal


LAUNCH_PKG = "openarm_bringup"
LAUNCH_FILE = "openarm_o6_bimanual.launch.py"

# Display label -> robot_controller argument value.
CONTROLLERS: dict[str, str] = {
    "Forward Position": "forward_position_controller",
    "Joint Trajectory": "joint_trajectory_controller",
}

# Preference order for terminal emulators.
_TERMINALS = [
    "gnome-terminal",
    "xterm",
    "xfce4-terminal",
    "konsole",
    "tilix",
]


def _find_terminal() -> str | None:
    for t in _TERMINALS:
        if shutil.which(t):
            return t
    return None


def _terminal_argv(terminal: str, title: str, cmd: str) -> list[str]:
    """Return argument list to open *terminal* with *title* and run *cmd*."""
    if terminal == "gnome-terminal":
        return [terminal, f"--title={title}", "--", "bash", "-c", cmd]
    if terminal == "konsole":
        return [terminal, "-p", f"tabtitle={title}", "-e", "bash", "-c", cmd]
    if terminal in ("xfce4-terminal", "tilix"):
        return [terminal, f"--title={title}", "-e", f"bash -c '{cmd}'"]
    # xterm fallback
    return [terminal, "-title", title, "-e", "bash", "-c", cmd]


class Ros2Launcher(QObject):
    """Opens a terminal window running ros2 launch.

    The terminal is independent — the user closes it (Ctrl+C / close button).
    `started` emits when the terminal process is spawned.
    `finished` emits with code 0 immediately after spawn (we don't track the
    terminal's lifetime, so the launcher panel re-enables Launch right away).
    """

    line = Signal(str)
    started = Signal()
    finished = Signal(int)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._terminal: str | None = _find_terminal()

    def is_running(self) -> bool:
        """Always False — we don't track the external terminal."""
        return False

    def start(self, controller_label: str, use_fake_hardware: bool) -> None:
        if controller_label not in CONTROLLERS:
            raise ValueError(f"unknown controller '{controller_label}'")

        ros2_args = [
            "ros2", "launch", LAUNCH_PKG, LAUNCH_FILE,
            "right_can_interface:=can2",
            "left_can_interface:=can3",
            "right_o6_can_interface:=can0",
            "left_o6_can_interface:=can1",
            f"robot_controller:={CONTROLLERS[controller_label]}",
        ]
        if use_fake_hardware:
            ros2_args.append("use_fake_hardware:=true")

        cmd_str = " ".join(ros2_args)
        self.line.emit("$ " + cmd_str)

        if self._terminal is None:
            self.line.emit("[error] no terminal emulator found; tried: " + ", ".join(_TERMINALS))
            self.finished.emit(1)
            return

        title = f"ros2 launch — {controller_label}"
        argv = _terminal_argv(self._terminal, title, cmd_str)
        try:
            subprocess.Popen(argv, start_new_session=True)
        except Exception as exc:  # noqa: BLE001
            self.line.emit(f"[error] could not open terminal: {exc}")
            self.finished.emit(1)
            return

        self.started.emit()
        # Terminal is detached — report done immediately so the button
        # re-enables (user can re-launch; they close the terminal themselves).
        self.finished.emit(0)

    def stop(self) -> None:
        """No-op: the terminal is independent; user stops it with Ctrl+C."""
        self.line.emit("[info] terminal is independent — use Ctrl+C inside it to stop.")
