from pathlib import Path

from PySide6.QtCore import QObject, Signal

from ..common.proc import ManagedProcess


# Vendored copy of openarm_ros2/scripts/can/install_can_udev.sh, shipped inside
# this repo so the launcher does not depend on an external workspace path.
#   can_install.py -> launcher/ -> openarm_launcher/ -> <repo root>
SCRIPT_PATH = (
    Path(__file__).resolve().parents[2] / "scripts" / "can" / "install_can_udev.sh"
)


class CanRuleInstall(QObject):
    """Run `install_can_udev.sh install` once, as root, with a cached password.

    The script auto-detects plugged gs_usb / PCAN devices, generates the
    udev naming rules (can0..can3) and installs them to
    /etc/udev/rules.d/, then reloads + triggers udev. We invoke the whole
    script under a single `sudo -S` so its internal SUDO() helper sees
    EUID==0 and runs each privileged step directly (no nested sudo).

    The sudo password is written to stdin (-S) so no NOPASSWD sudoers entry
    or terminal interaction is needed.
    """

    line = Signal(str)
    finished = Signal(bool)  # True if the script exited 0

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._proc = ManagedProcess(self)
        self._proc.line.connect(self._on_line)
        self._proc.finished.connect(self._on_finished)
        self._running = False
        self.last_error: str = ""  # last non-empty output line

    def is_running(self) -> bool:
        return self._running

    def start(self, password: str) -> None:
        if self._running:
            return
        if not SCRIPT_PATH.exists():
            self.line.emit(f"[error] script not found: {SCRIPT_PATH}")
            self.last_error = f"script not found: {SCRIPT_PATH}"
            self.finished.emit(False)
            return
        self._running = True
        self.last_error = ""
        self.line.emit(f"$ sudo bash {SCRIPT_PATH} install")
        # -S: read password from stdin; -p "": suppress the prompt line.
        self._proc.start(
            "sudo",
            ["-S", "-p", "", "bash", str(SCRIPT_PATH), "install"],
            stdin_data=(password + "\n").encode(),
        )

    def _on_line(self, text: str) -> None:
        self.line.emit(text)
        stripped = text.strip()
        if stripped:
            self.last_error = stripped

    def _on_finished(self, code: int) -> None:
        self._running = False
        if code != 0:
            self.line.emit(f"[error] install_can_udev.sh exited {code}")
        self.finished.emit(code == 0)
