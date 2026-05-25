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


class CanBringUp(QObject):
    """Sequentially run a recipe of `sudo -n ip link ...` steps.

    `sudo -n` is non-interactive: if NOPASSWD isn't configured the call
    fails immediately with a clear error (no hang waiting for password).
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
        self._proc.line.connect(self.line)
        self._proc.finished.connect(self._step_done)
        self._idx = 0
        self._ok = True
        self._running = False

    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._idx = 0
        self._ok = True
        self._run_current()

    def _run_current(self) -> None:
        label, args = self._steps[self._idx]
        self.progress.emit(label)
        self.line.emit(f"$ sudo -n {IP} {' '.join(args)}")
        self._proc.start("sudo", ["-n", IP] + args)

    def _step_done(self, code: int) -> None:
        label, _ = self._steps[self._idx]
        if code != 0:
            self._ok = False
            self.line.emit(f"[error] step '{label}' exited {code}")
            self._running = False
            self.finished.emit(False)
            return

        self._idx += 1
        if self._idx >= len(self._steps):
            self._running = False
            self.finished.emit(self._ok)
            return
        self._run_current()
