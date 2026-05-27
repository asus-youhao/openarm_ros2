from pathlib import Path

import pyudev
from PySide6.QtCore import QObject, QSocketNotifier, Signal


CAN_INTERFACES: tuple[str, ...] = ("can0", "can1", "can2", "can3")
SYSFS_NET = Path("/sys/class/net")


def read_operstate(name: str) -> str | None:
    """Return the canonical operstate for *name*, or None if the interface
    doesn't exist.

    CAN interfaces only ever report 'unknown' after 'ip link set canX up'
    (no carrier-sense).  We normalise that to 'up' so the UI shows green.
    Ethernet-style 'up' is also accepted unchanged.
    """
    try:
        raw = (SYSFS_NET / name / "operstate").read_text().strip()
    except FileNotFoundError:
        return None
    # Also check IFF_UP in the flags file so we know it's admin-up.
    try:
        flags = int((SYSFS_NET / name / "flags").read_text().strip(), 16)
        admin_up = bool(flags & 0x1)
    except (FileNotFoundError, ValueError):
        admin_up = (raw in ("up", "unknown"))
    # CAN reports 'unknown' when up; Ethernet reports 'up'.
    if admin_up and raw in ("up", "unknown"):
        return "up"
    return raw


class CanDetector(QObject):
    """Watches udev 'net' events for can0..can3 and emits their operstate.

    Signal payload: (interface_name, operstate)
      - operstate is None  → interface absent
      - operstate == "up"  → present and operationally up
      - other string       → present, not up (usually "down")
    """

    state = Signal(str, object)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._ctx = pyudev.Context()
        self._monitor = pyudev.Monitor.from_netlink(self._ctx)
        self._monitor.filter_by("net")
        self._monitor.start()

        self._notifier = QSocketNotifier(
            self._monitor.fileno(), QSocketNotifier.Read, self
        )
        self._notifier.activated.connect(self._drain)

    def emit_current_state(self) -> None:
        """Emit one `state` for every tracked interface with its current value."""
        for name in CAN_INTERFACES:
            self.state.emit(name, read_operstate(name))

    def _drain(self) -> None:
        device = self._monitor.poll(timeout=0)
        while device is not None:
            if device.sys_name in CAN_INTERFACES:
                self.state.emit(device.sys_name, read_operstate(device.sys_name))
            device = self._monitor.poll(timeout=0)
