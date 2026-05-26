import threading
import time
from collections.abc import Callable

from control_msgs.msg import JointTrajectoryControllerState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from PySide6.QtCore import QObject, Signal

from .arm_runner import ensure_rclpy


O6_RIGHT_JOINTS: list[str] = [
    "R_thumb_cmc_pitch", "R_thumb_cmc_yaw",
    "R_index_mcp_pitch", "R_middle_mcp_pitch",
    "R_ring_mcp_pitch", "R_pinky_mcp_pitch",
]
O6_LEFT_JOINTS: list[str] = [
    "L_thumb_cmc_pitch", "L_thumb_cmc_yaw",
    "L_index_mcp_pitch", "L_middle_mcp_pitch",
    "L_ring_mcp_pitch", "L_pinky_mcp_pitch",
]


def o6_joints_for(side: str) -> list[str]:
    if side == "right":
        return O6_RIGHT_JOINTS
    if side == "left":
        return O6_LEFT_JOINTS
    raise ValueError(f"unknown side: {side!r}")


class _O6Node(Node):
    """Subscribes to /<side>_hand_controller/controller_state and /joint_states.

    mode="action": waits for controller_state (cmd) before emitting.
    mode="topic":  no controller-state subscription; cmd stays zeros;
                   emits on joint_states alone (plots position).
    """

    def __init__(
        self,
        side: str,
        on_sample: Callable[[float, list[float], list[float]], None],
        rate_hz: float = 20.0,
        mode: str = "action",
    ) -> None:
        super().__init__(f"o6_analyzer_{side}")
        self._joint_names = o6_joints_for(side)
        self._on_sample = on_sample
        self._cmd = [0.0] * len(self._joint_names)
        self._actual = [0.0] * len(self._joint_names)
        self._have_actual = False
        self._t0 = time.monotonic()

        if mode == "action":
            self._have_cmd = False
            ctrl_topic = f"/{side}_hand_controller/controller_state"
            self.create_subscription(
                JointTrajectoryControllerState, ctrl_topic, self._on_ctrl, 10
            )
        else:
            self._have_cmd = True

        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_timer(1.0 / rate_hz, self._emit)

    def _absorb(self, names: list[str], positions: list[float], target: list[float]) -> None:
        for i, jn in enumerate(self._joint_names):
            if jn in names:
                k = names.index(jn)
                if k < len(positions):
                    target[i] = positions[k]

    def _on_ctrl(self, msg) -> None:  # noqa: ANN001
        ref = getattr(msg, "reference", None) or getattr(msg, "desired", None)
        if ref is None:
            return
        self._absorb(list(msg.joint_names), list(ref.positions), self._cmd)
        self._have_cmd = True

    def _on_js(self, msg: JointState) -> None:
        self._absorb(list(msg.name), list(msg.position), self._actual)
        self._have_actual = True

    def _emit(self) -> None:
        if not (self._have_cmd and self._have_actual):
            return
        t = time.monotonic() - self._t0
        self._on_sample(t, list(self._cmd), list(self._actual))


class O6Runner(QObject):
    """Mirrors ArmRunner for the O6 hand (6 joints, /<side>_hand_controller)."""

    sample = Signal(float, list, list)
    started = Signal()
    stopped = Signal()

    def __init__(
        self, side: str, parent: QObject | None = None, mode: str = "action"
    ) -> None:
        super().__init__(parent)
        self._side = side
        self._mode = mode
        self._node: _O6Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    @property
    def side(self) -> str:
        return self._side

    @property
    def joint_names(self) -> list[str]:
        return o6_joints_for(self._side)

    def start(self) -> None:
        if self._thread is not None:
            return
        ensure_rclpy()
        self._node = _O6Node(self._side, self._on_sample, mode=self._mode)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self.started.emit()

    def stop(self) -> None:
        """Non-blocking: signal the spin thread to exit and let a daemon thread
        handle the join + node teardown so the Qt main thread is never stalled.
        """
        if self._thread is None:
            return
        self._stop_evt.set()
        _t, _node, _exec = self._thread, self._node, self._executor
        self._thread = None
        self._node = None
        self._executor = None

        def _do_cleanup() -> None:
            _t.join(timeout=1.0)
            try:
                if _exec is not None and _node is not None:
                    _exec.remove_node(_node)
                    _node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
            self.stopped.emit()  # cross-thread — Qt auto-connection marshals to GUI thread

        threading.Thread(target=_do_cleanup, daemon=True).start()

    def _spin(self) -> None:
        assert self._executor is not None
        while not self._stop_evt.is_set():
            self._executor.spin_once(timeout_sec=0.05)

    def _on_sample(self, t: float, cmd: list[float], actual: list[float]) -> None:
        self.sample.emit(t, cmd, actual)
