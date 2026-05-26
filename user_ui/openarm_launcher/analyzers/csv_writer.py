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


class MergedCsvWriter:
    """Writes one row per sample; every row contains the latest known values
    for ALL groups.  Groups not yet seen carry zeros until their first update.
    The 'group' column identifies which runner produced the triggering sample.

    Typical groups: [("arm_right", [...]), ("arm_left", [...]),
                     ("o6_right", [...]),  ("o6_left",  [...])]
    """

    def __init__(
        self,
        data_root: Path,
        groups: list[tuple[str, list[str]]],
    ) -> None:
        today = datetime.date.today().isoformat()
        ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        self.dir = data_root / "csv" / today
        self.dir.mkdir(parents=True, exist_ok=True)
        self.path = self.dir / f"merged_{ts}.csv"
        self._groups = groups
        # Buffer: latest (cmd, actual) per group — initialised to zeros.
        self._latest: dict[str, tuple[list[float], list[float]]] = {
            tag: ([0.0] * len(jnames), [0.0] * len(jnames))
            for tag, jnames in groups
        }
        self._fh = open(self.path, "w", newline="")
        self._writer = csv.writer(self._fh)
        cols = ["t_s", "group"]
        for tag, jnames in groups:
            for name in jnames:
                cols.extend([f"{tag}__{name}_cmd", f"{tag}__{name}_actual", f"{tag}__{name}_err"])
        self._writer.writerow(cols)
        self._fh.flush()

    def update(self, tag: str, t: float, cmd: list[float], actual: list[float]) -> None:
        """Store the latest values for *tag* and write one complete row."""
        self._latest[tag] = (list(cmd), list(actual))
        row: list[str] = [f"{t:.6f}", tag]
        for grp_tag, _ in self._groups:
            gcmd, gactual = self._latest[grp_tag]
            for c, a in zip(gcmd, gactual):
                row.extend([f"{c:.6f}", f"{a:.6f}", f"{a - c:.6f}"])
        self._writer.writerow(row)

    def close(self) -> None:
        self._fh.close()
