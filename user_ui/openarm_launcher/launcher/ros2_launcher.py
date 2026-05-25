from PySide6.QtCore import QObject, Signal

from ..common.proc import ManagedProcess


LAUNCH_PKG = "openarm_bringup"
LAUNCH_FILE = "openarm_o6_bimanual.launch.py"

# Display label -> robot_controller argument value.
CONTROLLERS: dict[str, str] = {
    "Forward Position": "forward_position_controller",
    "Joint Trajectory": "joint_trajectory_controller",
}


class Ros2Launcher(QObject):
    """Runs `ros2 launch openarm_bringup openarm_o6_bimanual.launch.py ...`.

    CAN-interface mapping is fixed to match the user's existing aliases:
      right arm   = can2     left arm   = can3
      right O6    = can0     left O6    = can1
    """

    line = Signal(str)
    started = Signal()
    finished = Signal(int)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._proc = ManagedProcess(self)
        self._proc.line.connect(self.line)
        self._proc.started.connect(self.started)
        self._proc.finished.connect(self.finished)

    def is_running(self) -> bool:
        return self._proc.is_running()

    def start(self, controller_label: str, use_fake_hardware: bool) -> None:
        if controller_label not in CONTROLLERS:
            raise ValueError(f"unknown controller '{controller_label}'")
        args = [
            "launch", LAUNCH_PKG, LAUNCH_FILE,
            "right_can_interface:=can2",
            "left_can_interface:=can3",
            "right_o6_can_interface:=can0",
            "left_o6_can_interface:=can1",
            f"robot_controller:={CONTROLLERS[controller_label]}",
        ]
        if use_fake_hardware:
            args.append("use_fake_hardware:=true")
        self.line.emit("$ ros2 " + " ".join(args))
        self._proc.start("ros2", args)

    def stop(self) -> None:
        self._proc.stop()
