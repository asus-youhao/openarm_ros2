import os
import signal

from PySide6.QtCore import QObject, QProcess, Signal


class ManagedProcess(QObject):
    """QProcess wrapper that emits one signal per merged stdout/stderr line.

    Children are spawned as session leaders (CreateNewSession) so stop()
    can deliver a Ctrl-C-style SIGINT to the entire process group —
    important for `ros2 launch`, which only shuts its child nodes down
    on SIGINT and ignores SIGTERM to the parent."""

    line = Signal(str)
    started = Signal()
    finished = Signal(int)  # exit code (-1 if killed)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._proc = QProcess(self)
        self._proc.setProcessChannelMode(QProcess.MergedChannels)
        self._proc.readyReadStandardOutput.connect(self._drain)
        self._proc.started.connect(self.started)
        self._proc.finished.connect(self._on_finished)

        # CreateNewSession        → child is its own group leader, so
        #                            os.killpg can reach every descendant.
        # ResetSignalHandlers      → undo the parent's SIG_IGN on SIGINT
        #                            (Qt inherits it from the GUI app); without
        #                            this, the child silently ignores Ctrl-C.
        params = QProcess.UnixProcessParameters()
        params.flags = (
            QProcess.UnixProcessFlag.CreateNewSession
            | QProcess.UnixProcessFlag.ResetSignalHandlers
        )
        self._proc.setUnixProcessParameters(params)

        self._buf = bytearray()

    def start(self, program: str, args: list[str] | None = None) -> None:
        if self.is_running():
            raise RuntimeError(f"process already running (pid={self._proc.processId()})")
        self._buf.clear()
        self._proc.start(program, args or [])

    def stop(self, sigint_timeout_ms: int = 5000, sigterm_timeout_ms: int = 2000) -> None:
        """Shut the child tree down: SIGINT (Ctrl-C) → SIGTERM → SIGKILL.

        Each step is sent to the whole process group (the child + its
        descendants) via killpg, with a wait between escalations.
        """
        if not self.is_running():
            return
        pid = self._proc.processId()
        # SIGINT — gentle, what `ros2 launch` actually listens for.
        if not self._signal_group(pid, signal.SIGINT):
            self._proc.terminate()
        if self._proc.waitForFinished(sigint_timeout_ms):
            return
        # SIGTERM — escalation.
        if not self._signal_group(pid, signal.SIGTERM):
            self._proc.terminate()
        if self._proc.waitForFinished(sigterm_timeout_ms):
            return
        # SIGKILL — last resort.
        self._signal_group(pid, signal.SIGKILL)
        self._proc.kill()
        self._proc.waitForFinished(1000)

    @staticmethod
    def _signal_group(pid: int, sig: int) -> bool:
        if pid <= 0:
            return False
        try:
            os.killpg(pid, sig)
            return True
        except (ProcessLookupError, PermissionError):
            return False

    def is_running(self) -> bool:
        return self._proc.state() != QProcess.NotRunning

    def _drain(self) -> None:
        self._buf.extend(bytes(self._proc.readAllStandardOutput()))
        while b"\n" in self._buf:
            nl = self._buf.index(b"\n")
            chunk = bytes(self._buf[:nl])
            del self._buf[: nl + 1]
            self.line.emit(chunk.decode("utf-8", errors="replace").rstrip("\r"))

    def _on_finished(self, exit_code: int, _status: QProcess.ExitStatus) -> None:
        self._drain()
        if self._buf:
            self.line.emit(bytes(self._buf).decode("utf-8", errors="replace").rstrip("\r"))
            self._buf.clear()
        self.finished.emit(exit_code)
