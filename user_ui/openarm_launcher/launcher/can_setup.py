from PySide6.QtCore import QObject, Signal

from ..common.proc import ManagedProcess


IP = "/usr/sbin/ip"

# Bring-up sequence for the OpenArm + O6 bimanual setup.
# - can0/can1: classic CAN @ 1Mbps (O6 hands, via USB-CAN adapter)
# - can2/can3: CAN-FD @ 1M/5M (OpenArm arms, via PCAN). Kernel rejects
#   `up` and `type ...` in one command on most drivers, so config and
#   bring-up are split into two steps.
STEPS: list[tuple[str, list[str]]] = [
    ("can0", ["link", "set", "can0", "up", "type", "can", "bitrate", "1000000"]),
    ("can1", ["link", "set", "can1", "up", "type", "can", "bitrate", "1000000"]),
    ("can2 cfg", ["link", "set", "can2", "type", "can",
                   "bitrate", "1000000", "dbitrate", "5000000", "fd", "on"]),
    ("can3 cfg", ["link", "set", "can3", "type", "can",
                   "bitrate", "1000000", "dbitrate", "5000000", "fd", "on"]),
    ("can2 up", ["link", "set", "can2", "up"]),
    ("can3 up", ["link", "set", "can3", "up"]),
]


class CanBringUp(QObject):
    """Sequentially run the 6-step CAN bring-up via `sudo -n ip link ...`.

    `sudo -n` is non-interactive: if NOPASSWD is not configured the call
    fails immediately with a clear error (no hang waiting for password).
    """

    line = Signal(str)
    progress = Signal(str)
    finished = Signal(bool)  # True if every step exited 0

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
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
        label, args = STEPS[self._idx]
        self.progress.emit(label)
        self.line.emit(f"$ sudo -n {IP} {' '.join(args)}")
        self._proc.start("sudo", ["-n", IP] + args)

    def _step_done(self, code: int) -> None:
        label, _ = STEPS[self._idx]
        if code != 0:
            self._ok = False
            self.line.emit(f"[error] step '{label}' exited {code}")
            self._running = False
            self.finished.emit(False)
            return

        self._idx += 1
        if self._idx >= len(STEPS):
            self._running = False
            self.finished.emit(self._ok)
            return
        self._run_current()
