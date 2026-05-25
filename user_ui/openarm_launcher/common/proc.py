from PySide6.QtCore import QObject, QProcess, Signal


class ManagedProcess(QObject):
    """QProcess wrapper that emits one signal per merged stdout/stderr line."""

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
        self._buf = bytearray()

    def start(self, program: str, args: list[str] | None = None) -> None:
        if self.is_running():
            raise RuntimeError(f"process already running (pid={self._proc.processId()})")
        self._buf.clear()
        self._proc.start(program, args or [])

    def stop(self, timeout_ms: int = 2000) -> None:
        if not self.is_running():
            return
        self._proc.terminate()
        if not self._proc.waitForFinished(timeout_ms):
            self._proc.kill()
            self._proc.waitForFinished(1000)

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
