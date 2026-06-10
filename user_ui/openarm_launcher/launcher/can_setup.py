from PySide6.QtCore import QObject, Signal

from ..common.proc import ManagedProcess


IP = "/usr/sbin/ip"

# Each recipe is (step-label, ip-link-argv) tuples that get sudo-ed in
# order. Splitting these out lets the launcher panel expose one button
# per recipe so the operator can bring up each adapter when they're
# ready (e.g. after physically plugging it in).
STEPS_O6_RIGHT: list[tuple[str, list[str]]] = [
    ("can0", ["link", "set", "can0", "up", "type", "can", "bitrate", "1000000"]),
]
STEPS_O6_LEFT: list[tuple[str, list[str]]] = [
    ("can1", ["link", "set", "can1", "up", "type", "can", "bitrate", "1000000"]),
]
STEPS_OPENARM: list[tuple[str, list[str]]] = [
    ("can2 cfg", ["link", "set", "can2", "type", "can",
                   "bitrate", "1000000", "dbitrate", "5000000", "fd", "on"]),
    ("can3 cfg", ["link", "set", "can3", "type", "can",
                   "bitrate", "1000000", "dbitrate", "5000000", "fd", "on"]),
    ("can2 up", ["link", "set", "can2", "up"]),
    ("can3 up", ["link", "set", "can3", "up"]),
]

# Bring up every adapter in one shot — mirrors `install_can_udev.sh up`.
# can2/can3 are taken down first so re-applying the CAN-FD config can't fail
# on an already-up interface (the standalone STEPS_OPENARM assumes they're
# down; this combined recipe makes no such assumption).
#
# Steps may carry an optional third element `ignore_fail=True`; the "down"
# steps use it so a missing/already-down interface doesn't abort the recipe
# (matching the script's `ip link set canN down 2>/dev/null || true`).
STEPS_ALL: list[tuple] = [
    ("can0 up", ["link", "set", "can0", "up", "type", "can", "bitrate", "1000000"]),
    ("can1 up", ["link", "set", "can1", "up", "type", "can", "bitrate", "1000000"]),
    ("can2 down", ["link", "set", "can2", "down"], True),
    ("can3 down", ["link", "set", "can3", "down"], True),
    ("can2 cfg", ["link", "set", "can2", "type", "can",
                   "bitrate", "1000000", "dbitrate", "5000000", "fd", "on"]),
    ("can3 cfg", ["link", "set", "can3", "type", "can",
                   "bitrate", "1000000", "dbitrate", "5000000", "fd", "on"]),
    ("can2 up", ["link", "set", "can2", "up"]),
    ("can3 up", ["link", "set", "can3", "up"]),
]


class CanBringUp(QObject):
    """Sequentially run a recipe of `sudo -S ip link ...` steps.

    The caller supplies the sudo password; it is written to the process
    stdin (-S flag) so no terminal interaction or sudoers NOPASSWD entry
    is needed.
    """

    line = Signal(str)
    progress = Signal(str)
    finished = Signal(bool)  # True if every step exited 0

    def __init__(
        self,
        steps: list[tuple[str, list[str]]],
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._steps = list(steps)
        self._proc = ManagedProcess(self)
        self._proc.line.connect(self._on_line)
        self._proc.finished.connect(self._step_done)
        self._idx = 0
        self._ok = True
        self._running = False
        self._password: str = ""
        self.last_error: str = ""  # last non-empty output line from the failed step

    def is_running(self) -> bool:
        return self._running

    def start(self, password: str) -> None:
        if self._running:
            return
        self._password = password
        self._running = True
        self._idx = 0
        self._ok = True
        self._run_current()

    def _on_line(self, text: str) -> None:
        self.line.emit(text)
        stripped = text.strip()
        if stripped:
            self.last_error = stripped

    @staticmethod
    def _unpack(step: tuple) -> tuple[str, list[str], bool]:
        # (label, args) or (label, args, ignore_fail)
        label, args = step[0], step[1]
        ignore_fail = bool(step[2]) if len(step) > 2 else False
        return label, args, ignore_fail

    def _run_current(self) -> None:
        label, args, _ = self._unpack(self._steps[self._idx])
        self.progress.emit(label)
        self.line.emit(f"$ sudo {IP} {' '.join(args)}")
        self.last_error = ""
        # -S: read password from stdin; -p "": suppress the prompt line
        self._proc.start(
            "sudo", ["-S", "-p", "", IP] + args,
            stdin_data=(self._password + "\n").encode(),
        )

    def _step_done(self, code: int) -> None:
        label, _, ignore_fail = self._unpack(self._steps[self._idx])
        if code != 0 and ignore_fail:
            self.line.emit(f"[skip] step '{label}' exited {code} (ignored)")
        elif code != 0:
            self._ok = False
            self._password = ""  # clear immediately on failure
            self.line.emit(f"[error] step '{label}' exited {code}")
            self._running = False
            self.finished.emit(False)
            return

        self._idx += 1
        if self._idx >= len(self._steps):
            self._password = ""  # clear after all steps succeed
            self._running = False
            self.finished.emit(self._ok)
            return
        self._run_current()
