#!/usr/bin/env python3
"""
common.py — 單臂 / 雙臂 node 共用的無狀態工具與 I/O helper。

抽出這個模組的原因：舊版雙臂 node 為了拿 ``TfPoller`` / ``AsyncCsvWriter`` /
quaternion helper 而 ``from placo_ik_node import ...``，等於「跑雙臂要先把
整個單臂 node 載入一遍」。兩邊都改成 import 本模組後即切斷該耦合。

內容
----
quaternion helpers  : _qmul / _qnorm / _qconj / _qdot / _qfrom_rpy /
                      _rotate_vec / _rotate_quat / _qslerp
_parse_lpf_alpha    : --lpf-alpha 參數解析
TfPoller            : 背景執行緒快取 TF，避免 hot loop 阻塞
AsyncCsvWriter      : 背景執行緒寫 CSV，避免 hot loop 卡 disk I/O
EePosePublisher     : 連續 EE pose 發佈（自帶 timer，見其 docstring）
"""

import csv
import math
import queue
import threading
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped


# ── Quaternion helpers ────────────────────────────────────────────────────────
def _qmul(q1, q2):
    x1, y1, z1, w1 = q1; x2, y2, z2, w2 = q2
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
            w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)

def _qnorm(q):
    x, y, z, w = q; n = math.sqrt(x*x+y*y+z*z+w*w)
    return (x/n, y/n, z/n, w/n) if n > 1e-9 else (0., 0., 0., 1.)

def _qfrom_rpy(r, p, y):
    cr, cp, cy = math.cos(r/2), math.cos(p/2), math.cos(y/2)
    sr, sp, sy = math.sin(r/2), math.sin(p/2), math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)

def _rotate_vec(v, q):
    qx, qy, qz, qw = q; vx, vy, vz = v
    tx = 2*(qy*vz-qz*vy); ty = 2*(qz*vx-qx*vz); tz = 2*(qx*vy-qy*vx)
    return (vx+qw*tx+qy*tz-qz*ty, vy+qw*ty+qz*tx-qx*tz, vz+qw*tz+qx*ty-qy*tx)

def _qconj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def _qdot(q1, q2):
    return sum(a * b for a, b in zip(q1, q2))

def _qslerp(q0, q1, alpha):
    """SLERP from q0 to q1. alpha=1.0 returns q1."""
    q0 = _qnorm(q0)
    q1 = _qnorm(q1)
    alpha = max(0.0, min(1.0, float(alpha)))
    dot = _qdot(q0, q1)
    if dot < 0.0:
        q1 = tuple(-v for v in q1)
        dot = -dot
    if dot > 0.9995:
        return _qnorm(tuple((1.0 - alpha) * a + alpha * b for a, b in zip(q0, q1)))
    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return _qnorm(tuple(s0 * a + s1 * b for a, b in zip(q0, q1)))

def _rotate_quat(q, frame_q):
    """Similarity transform: rotate a delta quaternion through frame_q.
    dq_arm = frame_q * dq * conj(frame_q)"""
    return _qnorm(_qmul(_qmul(frame_q, _qnorm(q)), _qconj(frame_q)))


def _parse_lpf_alpha(spec: str, n_joints: int) -> List[float]:
    """Parse --lpf-alpha as single float (uniform) or comma list of len n_joints."""
    parts = [p.strip() for p in str(spec).split(",")]
    if len(parts) == 1:
        return [float(parts[0])] * n_joints
    if len(parts) != n_joints:
        raise ValueError(
            f"--lpf-alpha needs 1 or {n_joints} values, got {len(parts)}: {spec!r}"
        )
    return [float(p) for p in parts]


# ── TF Poller ─────────────────────────────────────────────────────────────────
class TfPoller:
    """
    Daemon thread that polls TF2 every poll_sec and caches the result.

    The IK hot-loop calls get() which returns instantly from the cache,
    eliminating the 0–300 ms blocking lookup that caused jitter.
    """

    def __init__(
        self,
        tf_buffer,
        base_link: str,
        ee_link:   str,
        poll_sec:  float = 0.05,
    ):
        self._buf      = tf_buffer
        self._base     = base_link
        self._ee       = ee_link
        self._poll_sec = poll_sec
        self._pose: Optional[Tuple] = None
        self._lock     = threading.Lock()
        self._thread   = threading.Thread(
            target=self._run, daemon=True, name="tf_poller")

    def start(self) -> None:
        self._thread.start()

    def get(self) -> Optional[Tuple]:
        """Instant non-blocking cache read."""
        with self._lock:
            return self._pose

    def _run(self) -> None:
        timeout = rclpy.duration.Duration(seconds=self._poll_sec)
        while True:
            try:
                t  = self._buf.lookup_transform(
                    self._base, self._ee,
                    rclpy.time.Time(), timeout=timeout)
                tr, ro = t.transform.translation, t.transform.rotation
                with self._lock:
                    self._pose = (tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w)
            except Exception:
                pass
            time.sleep(self._poll_sec * 0.5)   # 2× faster than timeout


# ── Async CSV Writer ──────────────────────────────────────────────────────────
class AsyncCsvWriter:
    """
    Daemon thread that drains a row queue to disk.

    The IK hot-loop calls put() which is non-blocking — no disk-I/O stalls.
    Flushes automatically whenever the queue drains to empty.
    """

    def __init__(self, filepath: str, fields: List[str]):
        self._fields = fields
        self._path   = filepath
        self._fh     = open(filepath, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(fields)
        self._q      = queue.Queue(maxsize=2000)
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="csv_writer")

    def start(self) -> None:
        self._thread.start()

    def put(self, row_dict: Dict) -> None:
        """Non-blocking enqueue; silently drops if queue is full."""
        try:
            self._q.put_nowait([row_dict.get(f, "") for f in self._fields])
        except queue.Full:
            pass

    def close(self) -> None:
        """Drain remaining rows then close the file."""
        if self._thread.is_alive():
            self._q.put(None)          # sentinel
            self._thread.join(timeout=3.0)
        self._fh.flush()
        self._fh.close()

    def _run(self) -> None:
        while True:
            row = self._q.get()
            if row is None:
                break
            self._writer.writerow(row)
            if self._q.empty():
                self._fh.flush()       # flush only when queue drains


# ── 連續 EE pose 發佈 ─────────────────────────────────────────────────────────
class EePosePublisher:
    """
    每臂發兩個 PoseStamped，node 一起來就開始、直到關閉都不中斷：

      /{arm}_eef_pose_fk
          永遠是 FK(當下命令關節)。unfold / homing / idle / teleop 全程連續。

      /{arm}_eef_pose
          「waypoint」語意：teleop 有新目標時發該目標（已過 SoftClamp + SLERP）；
          目標超過 ``target_ttl_sec`` 沒更新（idle / pause / homing）則改發
          FK hold，因此同樣全程連續。

    設計上刻意**自帶 ROS timer**，不掛在 IK hot loop 上。舊版是在 hot loop、
    homing ramp、``_build_target`` 三處手動插呼叫，並用一個 ``eef_pose_done``
    布林在函式間傳遞「本 tick 是否已發過」——漏一處就斷流，而且 unfold 階段
    根本還沒進 hot loop，開機那幾秒是完全沒有輸出的。改成 timer 之後這些
    呼叫點與那個布林全部消失。

    執行緒安全
    ----------
    ``fk_fn`` 由 timer 執行緒呼叫，與 hot loop 的 ``solve_step()`` 併發。
    ``PlacoSession`` / ``PlacoBimanualSession`` 為此各自持有一個**專用於 FK 的
    RobotWrapper**（見其 ``fk()``），所以這裡不需要也不應該去鎖 solver。

    Parameters
    ----------
    node       : rclpy Node
    arms       : 要發佈的臂，如 ``("right",)`` 或 ``("right", "left")``
    frame_id   : header.frame_id（通常是 ``cfg["base_link"]``）
    fk_fn      : ``(arm, joints) -> [x,y,z,qx,qy,qz,qw] | None``
    joints_fn  : ``(arm) -> List[float] | None`` 取當下命令關節；
                 回 None 表示還沒有可用關節（尚未收到 /joint_states）→ 跳過該臂
    rate_hz    : 發佈頻率
    target_ttl_sec : 目標多久沒更新就視為 idle、退回 FK hold
    """

    def __init__(
        self,
        node,
        arms:      Sequence[str],
        frame_id:  str,
        fk_fn:     Callable[[str, List[float]], Optional[Sequence[float]]],
        joints_fn: Callable[[str], Optional[List[float]]],
        rate_hz:   float = 50.0,
        target_ttl_sec: float = 0.10,
    ):
        self._node      = node
        self._arms      = tuple(arms)
        self._frame_id  = frame_id
        self._fk_fn     = fk_fn
        self._joints_fn = joints_fn
        self._ttl       = float(target_ttl_sec)

        self._pose_pub = {
            a: node.create_publisher(PoseStamped, f"/{a}_eef_pose", 10)
            for a in self._arms
        }
        self._fk_pub = {
            a: node.create_publisher(PoseStamped, f"/{a}_eef_pose_fk", 10)
            for a in self._arms
        }

        # arm -> (t_monotonic, xyz, quat)；由 hot loop 寫、timer 讀
        self._target: Dict[str, Tuple[float, Sequence[float], Sequence[float]]] = {}
        self._lock = threading.Lock()

        self._timer = node.create_timer(1.0 / float(rate_hz), self._tick)

    # ── 由 hot loop 呼叫 ──────────────────────────────────────────────────────
    def set_target(self, arm: str, xyz: Sequence[float],
                   quat: Sequence[float]) -> None:
        """記錄本步 teleop 目標；下一個 timer tick 會發到 /{arm}_eef_pose。"""
        with self._lock:
            self._target[arm] = (time.monotonic(),
                                 (float(xyz[0]), float(xyz[1]), float(xyz[2])),
                                 (float(quat[0]), float(quat[1]),
                                  float(quat[2]), float(quat[3])))

    # ── timer ────────────────────────────────────────────────────────────────
    def _tick(self) -> None:
        now = time.monotonic()
        for arm in self._arms:
            joints = self._joints_fn(arm)
            if joints is None:
                continue
            fk = self._fk_fn(arm, list(joints))
            if fk is None:
                continue

            # FK topic：一律發
            self._publish(self._fk_pub[arm], fk[:3], fk[3:7])

            # waypoint topic：新鮮目標優先，否則 FK hold
            with self._lock:
                tgt = self._target.get(arm)
            if tgt is not None and (now - tgt[0]) <= self._ttl:
                self._publish(self._pose_pub[arm], tgt[1], tgt[2])
            else:
                self._publish(self._pose_pub[arm], fk[:3], fk[3:7])

    def _publish(self, pub, xyz: Sequence[float], quat: Sequence[float]) -> None:
        msg = PoseStamped()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        pub.publish(msg)
