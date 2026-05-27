import threading
import time
from collections.abc import Callable

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from PySide6.QtCore import QObject, Signal


RIGHT_ARM_JOINTS: list[str] = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_ARM_JOINTS: list[str] = [f"openarm_left_joint{i}" for i in range(1, 8)]


def joints_for(side: str) -> list[str]:
    if side == "right":
        return RIGHT_ARM_JOINTS
    if side == "left":
        return LEFT_ARM_JOINTS
    raise ValueError(f"unknown side: {side!r}")


_rclpy_lock = threading.Lock()
_rclpy_inited = False

# Serialises concurrent destroy_node() calls from multiple cleanup threads.
_node_cleanup_lock = threading.Lock()


def ensure_rclpy() -> None:
    """Initialise rclpy at most once per process."""
    global _rclpy_inited
    with _rclpy_lock:
        if not _rclpy_inited:
            rclpy.init(args=[])
            _rclpy_inited = True


def shutdown_rclpy() -> None:
    """Shut rclpy down once; idempotent."""
    global _rclpy_inited
    with _rclpy_lock:
        if _rclpy_inited:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass
            _rclpy_inited = False


class _ArmNode(Node):
    """rclpy node: tracks the latest cmd (controller_state.reference)
    and actual (joint_states), then emits via callback at fixed rate.

    mode="action": subscribes to /{side}_joint_trajectory_controller/controller_state
                   for cmd; waits for both cmd and actual before emitting.
    mode="topic":  no controller-state subscription (forward_position_controller
                   has no state feedback); cmd stays at zeros and emission starts
                   as soon as joint_states arrive, effectively plotting position.
    """

    def __init__(
        self,
        side: str,
        on_sample: Callable[[float, list[float], list[float]], None],
        rate_hz: float = 20.0,
        mode: str = "action",
    ) -> None:
        super().__init__(f"arm_analyzer_{side}")
        self._joint_names = joints_for(side)
        self._on_sample = on_sample
        self._cmd = [0.0] * len(self._joint_names)
        self._actual = [0.0] * len(self._joint_names)
        self._have_actual = False
        self._t0 = time.monotonic()

        if mode == "action":
            self._have_cmd = False
            ctrl_topic = f"/{side}_joint_trajectory_controller/controller_state"
            self.create_subscription(
                JointTrajectoryControllerState, ctrl_topic, self._on_ctrl, 10
            )
        else:
            # topic / direct-position mode: no controller-state feedback.
            # cmd stays 0; emit as soon as joint_states arrive (plots position).
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


class ArmRunner(QObject):
    """Owns the rclpy node and its spin thread, re-emits samples as Qt signals."""

    sample = Signal(float, list, list)  # (t_s, cmd, actual)
    started = Signal()
    stopped = Signal()

    def __init__(
        self, side: str, parent: QObject | None = None, mode: str = "action"
    ) -> None:
        super().__init__(parent)
        self._side = side
        self._mode = mode
        self._node: _ArmNode | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._thread: threading.Thread | None = None

    @property
    def side(self) -> str:
        return self._side

    @property
    def joint_names(self) -> list[str]:
        return joints_for(self._side)

    def start(self) -> None:
        if self._thread is not None:
            return
        ensure_rclpy()
        self._node = _ArmNode(self._side, self._on_sample, mode=self._mode)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self.started.emit()

    def stop(self) -> None:
        """Non-blocking: call executor.shutdown() which sends a SIGINT-like
        signal to the spin loop, then join + destroy in a daemon thread so
        the Qt main thread is never stalled.
        """
        if self._thread is None:
            return
        # Snapshot and clear immediately so callers can call start() again.
        _t, _node, _exec = self._thread, self._node, self._executor
        self._thread = None
        self._node = None
        self._executor = None

        def _do_cleanup() -> None:
            try:
                if _exec is not None:
                    _exec.shutdown(timeout_sec=1.0)  # wakes spin() gracefully
            except Exception:  # noqa: BLE001
                pass
            _t.join(timeout=2.0)
            with _node_cleanup_lock:  # serialise across concurrent runner teardowns
                try:
                    if _node is not None:
                        _node.destroy_node()
                except Exception:  # noqa: BLE001
                    pass
            self.stopped.emit()  # cross-thread — Qt auto-connection marshals to GUI thread

        threading.Thread(target=_do_cleanup, daemon=True).start()

    def _spin(self) -> None:
        """Block on executor.spin(); returns when executor.shutdown() is called."""
        assert self._executor is not None
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001
            pass

    def _on_sample(self, t: float, cmd: list[float], actual: list[float]) -> None:
        # Crosses ROS thread -> GUI thread via Qt's queued connection.
        self.sample.emit(t, cmd, actual)
