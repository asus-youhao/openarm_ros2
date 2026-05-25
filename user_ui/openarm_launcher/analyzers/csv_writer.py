import csv
import datetime
from pathlib import Path


class ArmCsvWriter:
    """Writes one row per arm sample: timestamp + cmd/actual/error for each joint."""

    def __init__(self, data_root: Path, side: str, joint_names: list[str]) -> None:
        today = datetime.date.today().isoformat()
        ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        self.dir = data_root / "csv" / today
        self.dir.mkdir(parents=True, exist_ok=True)
        self.path = self.dir / f"{side}_{ts}.csv"

        self._fh = open(self.path, "w", newline="")
        self._writer = csv.writer(self._fh)

        cols = ["t_s"]
        for name in joint_names:
            cols.extend([f"{name}_cmd", f"{name}_actual", f"{name}_err"])
        self._writer.writerow(cols)

    def write(self, t: float, cmd: list[float], actual: list[float]) -> None:
        row: list[str] = [f"{t:.6f}"]
        for c, a in zip(cmd, actual):
            row.extend([f"{c:.6f}", f"{a:.6f}", f"{a - c:.6f}"])
        self._writer.writerow(row)

    def close(self) -> None:
        self._fh.close()
