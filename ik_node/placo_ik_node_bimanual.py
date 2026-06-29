#!/usr/bin/env python3
"""
placo_ik_node_bimanual.py
=========================
ROS2 Node：雙臂合併單一 QP 的即時 IK 控制器 + profiler（v1，2026-06-12）。

與單臂版 (placo_ik_node.py × 2 instances) 的差異
------------------------------------------------
本檔是**新檔案**，不修改舊版；設計細節見 docs/bimanual_single_qp.md。

  架構：一個 node、一個 hot loop、一個 PlacoBimanualSession（14-DOF QP）。
        舊版是兩個 node + 兩條 IK thread + MultiThreadedExecutor。

  1. 【單一 hot loop】兩臂 timing 不再互相漂移；只要任一臂有新 tracker 訊息
     就 solve 一次，閒置臂的 target 維持上次值（QP hold-in-place）。
  2. 【jump guard 移除】被 session 內的「統一 Δq 縮放」取代——
     不再逐軸 clip（會破壞 7 軸耦合），改整臂等比例縮放（roadmap #1）。
  3. 【scale 改變 → 自動 reanchor】修掉 roadmap #4 的瞬跳：tracker delta 是
     anchor 累積量 × scale，中途改 scale 會瞬間重縮放整段 offset。
     本版偵測到 kbd scale 改變即對兩臂觸發 reanchor。
  4. 【兩臂 calib 預設 0°】實測校正（docs issue N2，2026-06-15）：右臂 0° 正常，
     左臂舊值 180° 反而造成 X/Y 反向，故兩臂統一 0°。
  5. 【fail 時 _pose 用 ee_cmd_xyz】= 縮放後實際發布關節的 FK，
     比舊版（solver 內部 EE，與發布值不符）準確。
  6. 【reanchor 事件寫入 CSV】event 欄（docs issue #15）。

  沿用（直接 import 舊模組，不複製）：
    TfPoller / AsyncCsvWriter / quaternion helpers / _parse_lpf_alpha
                                          ← placo_ik_node.py
    KbdController（全域鍵盤控制）          ← kbd_controller.py
    SoftClamp / BoundaryMonitor（per-arm） ← ws_boundary.py
    ARM_CONFIG / paths                     ← config / paths.py

  v1 不支援（見 docs TODO）：
    keyboard teleop 模式（kbd 只做 quit/pause/reset/scale/home）、
    --use-traj（只走 ForwardCommandController）、
    placo self-collision constraint（kin-URDF 無 collision geometry，
    以 session 的 EE 近距節流代替）。
"""

import os as _os, sys as _sys
_HERE = _os.path.dirname(_os.path.abspath(__file__))
_ROOT = _os.path.dirname(_HERE)
_sys.path.insert(0, _ROOT)
_sys.path.insert(0, _HERE)
_sys.path.insert(0, _os.path.join(_ROOT, "ik_solver"))
_sys.path.insert(0, _os.path.join(_ROOT, "ws_mesh"))
_sys.path.insert(0, _os.path.join(_ROOT, "config"))

import collections
import json
import math
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from placo_ik_solver import _find_urdf, _quat_to_rot
from placo_ws_analyze import WorkspaceMesh
from placo_ik_session_bimanual import PlacoBimanualSession, ARMS, _MAX_ITER
from kbd_controller import KbdController
from ws_boundary import SoftClamp, BoundaryMonitor
from paths import csv_path as _csv_path, png_for as _png_for
from arm_config import ARM_CONFIG

# 共用元件直接取自單臂 node（import 不修改）
from placo_ik_node import (
    TfPoller, AsyncCsvWriter,
    _qmul, _qnorm, _qfrom_rpy, _rotate_vec, _qslerp, _rotate_quat,
    _parse_lpf_alpha,
)

_PFX = {"right": "r", "left": "l"}   # CSV 欄位前綴

CSV_FIELDS = (
    ["t", "event",
     "ik_ms", "setup_ms", "loop_ms", "iterations",
     "deadline_missed", "ee_dist_mm", "ee_dist_cmd_mm", "prox_scale", "mem_kb"]
    + [f"{p}_{c}"
       for p in _PFX.values()
       for c in (["x", "y", "z", "success", "pos_err_mm", "ori_err_deg",
                  "track_err_mm", "sigma_min", "lambda_extra", "w_ori",
                  "budget_scale"]
                 + [f"q_cmd_{i}" for i in range(7)]
                 + ["tf_x", "tf_y", "tf_z"])]
)


class PlacoBimanualNode(Node):
    """雙臂單一 QP IK 控制器。

    Input  : /ee_delta/right + /ee_delta/left (PoseStamped, anchor 累積 delta)
    Output : 兩臂 ForwardCommandController + latency + 合併 profile JSON + CSV
    """

    def __init__(self, args):
        super().__init__("placo_ik_bimanual")
        self.args = args
        self._cbg = ReentrantCallbackGroup()

        self._rate_hz     = float(args.rate)
        self._deadline_ms = 1000.0 / self._rate_hz
        self._gap_sec     = float(args.ee_delta_gap_sec)
        self._no_rot      = bool(args.no_rot_tracking)
        self._ori_alpha   = max(0.0, min(1.0, float(args.ori_lpf_alpha)))
        self._ori_active  = self._ori_alpha < 0.999

        # ── per-arm 狀態 ──────────────────────────────────────────────────
        # arm_overrides: 入口 script 由 CLI/YAML 整理好的 per-arm 參數
        ov = getattr(args, "arm_overrides", {}) or {}
        self.S: Dict[str, Dict] = {}
        for arm in ARMS:
            o   = ov.get(arm, {})
            cfg = ARM_CONFIG[arm]
            crpy = o.get("calib_rpy")
            if crpy:
                rr, rp, ry = [math.radians(float(v)) for v in str(crpy).split(",")]
            else:
                rr, rp = 0.0, 0.0
                # 預設兩臂都 0°（無翻轉）。
                #
                # 實測校正（2026-06-15，docs issue N2）：
                #   右臂 0° 動作正常；左臂舊預設 180° 造成 X、Y 同時反向
                #   （delta_x 往前→arm 往後、delta_y 往左→arm 往右、Z 一致），
                #   這正是 180° yaw 旋轉的特徵 → 多套了一個翻轉。
                #   Pico 左右手 controller 的「位置」共用同一 global tracking
                #   space，無左右鏡像，故兩臂本該用相同 calib。
                #   舊版「左臂需 180° 修鏡像」的假設經實測證明為誤。
                # YAML / CLI 的 calib-yaw-right / calib-yaw-left 仍可個別覆寫。
                ry = math.radians(float(o.get("calib_yaw", 0.0)))
            self.S[arm] = {
                "cfg":              cfg,
                "calib_q":          _qfrom_rpy(rr, rp, ry),
                "calib_yaw_deg":    math.degrees(ry),
                # tracker session
                "pending":          None,
                "pending_t_recv":   None,
                "reanchor_pending": False,
                "ref_xyz":          None,
                "ref_q":            None,
                "last_msg_t":       None,
                "ori_filt_q":       None,
                # joint / pose 簿記
                "pose":             list(cfg["home_pose"]),
                "last_joints":      list(cfg["home_joints"]),
                "filt_joints":      None,
                "target_xyz":       np.array(cfg["home_pose"][:3]),
                "target_q":         tuple(cfg["home_pose"][3:7]),
                "fail_streak":      0,
                "reanchor_event":   False,   # 本 tick 是否發生 reanchor（CSV 用）
                "lpf_alpha":        _parse_lpf_alpha(
                    o.get("lpf_alpha", args.lpf_alpha), 7),
            }
            self.S[arm]["lpf_active"] = any(a < 0.999
                                            for a in self.S[arm]["lpf_alpha"])

        self._delta_lock  = threading.Lock()
        self._js_lock     = threading.Lock()
        self._joints_lock = threading.Lock()
        self._joint_states: dict = {}
        self._msg_count   = 0

        # ── go-home service 的請求/完成同步 ────────────────────────────────
        # service callback（executor thread）只掛旗標；homing 一律由 hot loop
        # 執行（與鍵盤 h 同一條路徑），否則 homing 期間 hot loop 仍在發 IK
        # 命令，兩股命令會互相打架（安全問題）。
        self._home_request = threading.Event()
        self._home_done    = threading.Event()
        self._home_result  = {"ok": False, "msg": ""}

        self._init_workspace(args, ov)
        self._init_session(args)
        self._init_ros(args)
        self._init_telemetry(args)

        self._kbd = KbdController(init_mode="tracker")
        self._kbd_last_scale = self._kbd.scale

        for arm in ARMS:
            self.S[arm]["bdry"] = BoundaryMonitor(self, arm)

        self._print_banner()

    # ── init ──────────────────────────────────────────────────────────────────
    def _init_workspace(self, args, ov):
        for arm in ARMS:
            s = self.S[arm]
            mesh = None
            npz  = ov.get(arm, {}).get("ws_mesh")
            if npz:
                if not os.path.isfile(npz):
                    raise FileNotFoundError(f"ws_mesh not found ({arm}): {npz}")
                mesh = WorkspaceMesh.load(npz)
            s["ws_mesh"]  = mesh
            s["ws_clamp"] = bool(args.ws_clamp) or mesh is not None
            s["soft_clamp"] = SoftClamp(
                ws_mesh  = mesh,
                margin_m = float(args.boundary_margin),
                box_ws   = s["cfg"]["workspace"] if mesh is None else None,
            )

    def _init_session(self, args):
        urdf = _find_urdf()
        self._session = PlacoBimanualSession(
            urdf            = urdf,
            rate_hz         = self._rate_hz,
            max_iter        = int(args.max_iter),
            vel_limits      = not args.no_vel_limits,
            wrist_vel_cap   = float(args.wrist_vel_cap),
            j3j4_couple     = bool(args.j3j4_couple),
            tick_budget_deg = float(args.tick_budget_deg),
            min_ee_dist     = float(args.min_ee_dist),
        )

    def _init_ros(self, args):
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        for arm in ARMS:
            s, cfg = self.S[arm], self.S[arm]["cfg"]
            s["tf_poller"] = TfPoller(
                self._tf_buffer, cfg["base_link"], cfg["ee_link"], poll_sec=0.05)
            s["fwd_pub"]     = self.create_publisher(
                Float64MultiArray, cfg["fwd_cmd_topic"], 10)
            s["latency_pub"] = self.create_publisher(
                Float32, cfg["latency_topic"], 10)
            s["ik_cmd_pub"]  = self.create_publisher(
                JointState, f"/{arm}_arm_ik_commands", 10)
            self.create_subscription(
                PoseStamped, cfg["ee_delta_topic"],
                lambda msg, a=arm: self._ee_delta_cb(a, msg),
                10, callback_group=self._cbg)
        self._profile_pub = self.create_publisher(
            String, "/bimanual/placo_profile", 10)
        self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10,
            callback_group=self._cbg)
        # go-home service：必須掛 ReentrantCallbackGroup——callback 阻塞等待
        # homing 完成時，executor 其他 thread 仍要能服務 /joint_states、TF
        self.create_service(
            Trigger, "/bimanual/go_home", self._home_srv_cb,
            callback_group=self._cbg)

    def _init_telemetry(self, args):
        _MAXLEN = 6000
        self._records: collections.deque = collections.deque(maxlen=_MAXLEN)
        self._stat_n, self._stat_miss = 0, 0
        self._stat_ok = {a: 0 for a in ARMS}
        self._ring_ik = collections.deque(maxlen=500)
        csv_p = args.csv or _csv_path("placo_bimanual", "both", "qp")
        self._csv_writer = AsyncCsvWriter(csv_p, CSV_FIELDS)
        self._csv_path   = csv_p

    # ── ROS callbacks ─────────────────────────────────────────────────────────
    def _js_cb(self, msg: JointState):
        with self._js_lock:
            for n, p in zip(msg.name, msg.position):
                self._joint_states[n] = p

    def _ee_delta_cb(self, arm: str, msg: PoseStamped):
        t_recv = time.time()
        if msg.header.frame_id == "reanchor":
            with self._delta_lock:
                self.S[arm]["reanchor_pending"] = True
            self._msg_count += 1
            return
        with self._delta_lock:
            self.S[arm]["pending"]        = msg
            self.S[arm]["pending_t_recv"] = t_recv
        self._msg_count += 1

    # ── TF helper（blocking，僅 startup / home 用） ───────────────────────────
    def _get_tf(self, arm: str, timeout_sec: float = 0.3) -> Optional[Tuple]:
        try:
            cfg = self.S[arm]["cfg"]
            t = self._tf_buffer.lookup_transform(
                cfg["base_link"], cfg["ee_link"], rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec))
            tr, ro = t.transform.translation, t.transform.rotation
            return (tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w)
        except Exception:
            return None

    # ── Startup / home（雙臂並行） ────────────────────────────────────────────
    def _wait_joint_states(self, timeout_sec: float = 3.0) -> Optional[dict]:
        """等到兩臂 14 joints 都出現在 /joint_states，逾時回 None。"""
        names = [n for a in ARMS for n in self.S[a]["cfg"]["joint_names"]]
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            with self._js_lock:
                js = dict(self._joint_states)
            if all(n in js for n in names):
                return js
            time.sleep(0.1)
        return None

    def _publish_fwd(self, arm: str, joints: List[float]):
        msg = Float64MultiArray()
        msg.data = list(joints)
        self.S[arm]["fwd_pub"].publish(msg)

    def _ramp_both(self, start: Dict[str, List[float]],
                   target: Dict[str, List[float]],
                   label: str, deg_per_sec: float = 30.0,
                   ramp_hz: float = 50.0) -> Dict[str, List[float]]:
        """兩臂同時 cosine ramp 到各自 target；時長取全域最大 delta。"""
        max_delta = max(abs(t - s)
                        for a in ARMS
                        for s, t in zip(start[a], target[a]))
        if max_delta < math.radians(1.0):
            print(f"  [ramp] {label}: already near target — skip")
            return {a: list(target[a]) for a in ARMS}
        ramp_sec = max(0.5, max_delta / math.radians(deg_per_sec))
        n_steps  = int(ramp_sec * ramp_hz)
        dt       = 1.0 / ramp_hz
        print(f"  [ramp] {label}: {math.degrees(max_delta):.1f}° max  "
              f"{ramp_sec:.1f}s  {n_steps} steps")
        cur = {a: list(start[a]) for a in ARMS}
        for step in range(n_steps + 1):
            alpha = 0.5 * (1.0 - math.cos(step / max(1, n_steps) * math.pi))
            for a in ARMS:
                cur[a] = [s + alpha * (t - s) for s, t in zip(start[a], target[a])]
                self._publish_fwd(a, cur[a])
            time.sleep(dt)
        print(f"  [ramp] {label}: done")
        return cur

    def joint_unfold_sequence_both(self, j4_skip_thresh_deg: float = 70.0):
        """雙臂並行 unfold（邏輯同單臂版的 full/short 模式，分臂判斷、同步執行）。

        full  (j4 ≤ 70°)：stage1 j3→±90°、stage2 j4→90°、stage3 j3→0°
        short (j4 > 70°)：stage1 全關節同步 → home layout（j4=90° 其餘 0°），
                          stage2/3 hold。
        """
        print("  [unfold] -- bimanual unfold start --")
        js = self._wait_joint_states(3.0)
        if js is None:
            print("  [unfold] ✗ joint states missing after 3s — skip unfold")
            return
        cur = {a: [js[n] for n in self.S[a]["cfg"]["joint_names"]] for a in ARMS}
        j3_sign = {"right": 1.0, "left": -1.0}
        stages: List[Dict[str, List[float]]] = []
        for k in range(3):
            stages.append({})
        for a in ARMS:
            j4_deg = math.degrees(cur[a][3])
            if j4_deg > j4_skip_thresh_deg:
                # short：stage1 直上 home layout，之後 hold
                t1 = [0.0] * 7
                t1[3] = math.pi / 2.0
                stages[0][a] = t1
                stages[1][a] = t1
                stages[2][a] = t1
                print(f"  [unfold] {a}: j4={j4_deg:.0f}° > {j4_skip_thresh_deg:.0f}°"
                      f" → short mode")
            else:
                t1 = cur[a][:]
                t1[2] = j3_sign[a] * math.pi / 2.0       # j3 → ±90°
                t2 = t1[:]
                t2[3] = math.pi / 2.0                     # j4 → 90°
                t3 = t2[:]
                t3[2] = 0.0                               # j3 → 0°
                stages[0][a], stages[1][a], stages[2][a] = t1, t2, t3
                print(f"  [unfold] {a}: j4={j4_deg:.0f}° → full 3-stage")
        for k, tgt in enumerate(stages, start=1):
            cur = self._ramp_both(cur, tgt, f"unfold-stage{k}")
            time.sleep(0.3)
        print("  [unfold] complete")

    def send_home_fwd_both(self, pos_tol: float = 0.025,
                           motion_sec: float = 3.5) -> bool:
        """兩臂同時 ramp 到 home，TF 驗證後同步內部狀態。"""
        js = self._wait_joint_states(3.0)
        if js is None:
            print("  [home] ✗ no joint states — abort homing")
            return False
        start  = {a: [js[n] for n in self.S[a]["cfg"]["joint_names"]] for a in ARMS}
        target = {a: list(self.S[a]["cfg"]["home_joints"]) for a in ARMS}
        self._ramp_both(start, target, "home", deg_per_sec=30.0)
        time.sleep(0.5)

        ok = True
        for a in ARMS:
            s = self.S[a]
            tf = self._get_tf(a, 1.0)
            home_xyz = s["cfg"]["home_pose"][:3]
            if tf is not None:
                dist = math.sqrt(sum((tf[i] - home_xyz[i])**2 for i in range(3)))
                print(f"  [home] {a}: dist={dist*100:.1f}cm "
                      f"{'✓' if dist <= pos_tol else '⚠'}")
                s["pose"] = list(tf)
                ok = ok and dist <= pos_tol
            else:
                print(f"  [home] {a}: ⚠ no TF — using home_pose")
                s["pose"] = list(s["cfg"]["home_pose"])
                ok = False
            with self._joints_lock:
                s["last_joints"] = list(target[a])
            s["filt_joints"] = None
            s["ori_filt_q"]  = None
            s["target_xyz"]  = np.array(s["pose"][:3])
            s["target_q"]    = tuple(s["pose"][3:7])
        return ok

    def _startup_sync(self):
        """同步兩臂 EE pose + joints（優先序同單臂版：js → TF → FK → home）。"""
        print("\n  Syncing both arms from TF2 and /joint_states...")
        js = self._wait_joint_states(3.0) or {}
        for a in ARMS:
            s = self.S[a]
            jnames = s["cfg"]["joint_names"]
            if all(n in js for n in jnames):
                real = [js[n] for n in jnames]
                with self._joints_lock:
                    s["last_joints"] = real
                print(f"  [{a}] joints synced")
            else:
                real = None
                print(f"  [{a}] ⚠ /joint_states not ready — home_joints as seed")
            tf = self._get_tf(a, 1.0)
            if tf:
                s["pose"] = list(tf)
                print(f"  [{a}] TF EE: {[f'{v:.4f}' for v in tf[:3]]}")
            elif real is not None:
                fkp = self._session.fk(a, real)
                if fkp is not None:
                    s["pose"] = fkp
                    print(f"  [{a}] ⚠ TF unavailable — pose via FK")
                else:
                    print(f"  [{a}] ⚠ TF & FK unavailable — home_pose")
            else:
                print(f"  [{a}] ⚠ TF & joints unavailable — home_pose")
            s["target_xyz"] = np.array(s["pose"][:3])
            s["target_q"]   = tuple(s["pose"][3:7])

    # ── go-home service（executor thread：只掛旗標 + 等待結果） ──────────────
    def _home_srv_cb(self, req, resp):
        if self._home_request.is_set():
            resp.success, resp.message = False, "homing already in progress"
            return resp
        self._home_done.clear()
        self._home_request.set()
        print("\n  [srv] /bimanual/go_home requested")
        if self._home_done.wait(timeout=20.0):
            resp.success = self._home_result["ok"]
            resp.message = self._home_result["msg"]
        else:
            resp.success = False
            resp.message = "timeout — hot loop not running?"
        return resp

    def _execute_home_request(self):
        """在 hot loop 內實際執行回 home（service 與鍵盤 h 的共同路徑）。"""
        ok = self.send_home_fwd_both()
        # 防跳：homing 期間 tracker 累積的 pending 作廢 + 強制 reanchor，
        # 不依賴 gap 時序（service 可在任意時機觸發）
        with self._delta_lock:
            for a in ARMS:
                self.S[a]["pending"]          = None
                self.S[a]["pending_t_recv"]   = None
                self.S[a]["reanchor_pending"] = True
        self._home_result.update(
            ok=ok, msg="both arms homed" if ok else "TF verification failed")
        self._home_request.clear()
        self._home_done.set()

    # ── 鍵盤事件（全域；scale 改變 → 雙臂 reanchor，修 roadmap #4） ──────────
    def _handle_kbd_events(self) -> bool:
        kbd = self._kbd
        if kbd.request_quit:
            print("\n  [kbd] quit — exiting")
            return True
        if kbd.request_home:
            kbd.request_home = False
            print("\n  [kbd] homing both arms...")
            self._home_request.set()   # 與 service 同一條執行路徑
        if self._home_request.is_set():
            self._execute_home_request()
        if kbd.reset_ref:
            kbd.reset_ref = False
            for a in ARMS:
                self._reset_session_ref(a)
            print("\n  [kbd] both session references reset")
        if abs(kbd.scale - self._kbd_last_scale) > 1e-9:
            # roadmap #4：delta 是 anchor 累積量 × scale，中途改 scale 必須
            # reanchor，否則整段 offset 被瞬間重縮放 → 手臂跳
            self._kbd_last_scale = kbd.scale
            with self._delta_lock:
                for a in ARMS:
                    self.S[a]["reanchor_pending"] = True
            print(f"\n  [kbd] scale → {kbd.scale:.2f}×  (auto re-anchor both arms)")
        return False

    def _reset_session_ref(self, arm: str):
        s = self.S[arm]
        s["ref_xyz"], s["ref_q"] = None, None
        s["last_msg_t"]  = None
        s["ori_filt_q"]  = None

    # ── target 構建（per-arm；邏輯同單臂版 tracker 分支） ─────────────────────
    def _build_target_arm(self, arm: str, msg: PoseStamped, t_wall: float) -> bool:
        """更新 S[arm]["target_xyz"/"target_q"]。回傳是否有效更新。

        anchor / reanchor / 首訊息丟棄 / calib 旋轉 / SoftClamp / SLERP
        全部沿用單臂版語意。
        """
        s = self.S[arm]
        _sc = self._kbd.scale
        dx = msg.pose.position.x * _sc
        dy = msg.pose.position.y * _sc
        dz = msg.pose.position.z * _sc
        dq = (msg.pose.orientation.x, msg.pose.orientation.y,
              msg.pose.orientation.z, msg.pose.orientation.w)
        if abs(dq[3]) < 0.01 and all(abs(v) < 0.01 for v in dq[:3]):
            dq = (0., 0., 0., 1.)

        gap = (t_wall - s["last_msg_t"]) if s["last_msg_t"] else 999.0
        with self._delta_lock:
            reanchor = s["reanchor_pending"]
            s["reanchor_pending"] = False
        is_new = (s["ref_xyz"] is None or reanchor or gap > self._gap_sec)
        s["last_msg_t"] = t_wall

        if is_new:
            ref = s["tf_poller"].get()
            src = "sentinel" if reanchor else "TF"
            if ref is not None:
                s["ref_xyz"] = tuple(ref[:3])
                s["ref_q"]   = tuple(ref[3:7])
                s["pose"]    = list(ref)
                print(f"\n  [EE-δ:{arm}] NEW ref={[f'{v:.4f}' for v in ref[:3]]} ({src})")
            else:
                s["ref_xyz"] = tuple(s["pose"][:3])
                s["ref_q"]   = tuple(s["pose"][3:7])
            s["ori_filt_q"] = None
            s["reanchor_event"] = True   # CSV event 標記（docs #15）
            # 丟棄首訊息 delta（dead-man 釋放期間 tracker 累積量不可靠）
            # target 維持原值 → 該臂 hold
            return False

        base_xyz = s["ref_xyz"] or tuple(s["pose"][:3])
        base_q   = s["ref_q"]   or tuple(s["pose"][3:7])
        dx_arm   = _rotate_vec((dx, dy, dz), s["calib_q"])
        dq       = _rotate_quat(dq, s["calib_q"])

        raw = np.array([base_xyz[0] + dx_arm[0],
                        base_xyz[1] + dx_arm[1],
                        base_xyz[2] + dx_arm[2]])
        if s["ws_clamp"]:
            new_xyz, _bs = s["soft_clamp"].apply(raw, np.array(dx_arm))
            s["bdry"].publish(_bs)
        else:
            new_xyz = raw

        new_q_raw = _qnorm(_qmul(dq, base_q))
        if self._no_rot:
            new_q = base_q
        elif self._ori_active and s["ori_filt_q"] is not None:
            s["ori_filt_q"] = _qslerp(s["ori_filt_q"], new_q_raw, self._ori_alpha)
            new_q = s["ori_filt_q"]
        else:
            s["ori_filt_q"] = new_q_raw
            new_q = new_q_raw

        s["target_xyz"] = np.asarray(new_xyz, dtype=float)
        s["target_q"]   = new_q
        return True

    # ── 輸出 LPF（per-arm，同單臂版） ─────────────────────────────────────────
    def _filter_joints(self, arm: str, raw: List[float]) -> List[float]:
        s = self.S[arm]
        if not s["lpf_active"] or s["filt_joints"] is None:
            s["filt_joints"] = list(raw)
            return list(raw)
        s["filt_joints"] = [
            a * r + (1.0 - a) * p
            for a, r, p in zip(s["lpf_alpha"], raw, s["filt_joints"])
        ]
        return list(s["filt_joints"])

    # ── 主迴圈 ────────────────────────────────────────────────────────────────
    def run(self):
        dt_sec = 1.0 / self._rate_hz
        self._kbd.start()
        self._startup_sync()
        for a in ARMS:
            self.S[a]["tf_poller"].start()
        self._csv_writer.start()

        print(f"\n  Waiting for /ee_delta/right + /ee_delta/left ...\n")
        self._no_msg_t = time.time()
        step_count = 0

        while rclpy.ok():
            t_step = time.perf_counter()
            if self._handle_kbd_events():
                break

            with self._delta_lock:
                msgs = {}
                for a in ARMS:
                    msgs[a] = self.S[a]["pending"]
                    self.S[a]["pending"] = None
                    self.S[a]["pending_t_recv"] = None

            updated_any = False
            if not self._kbd.paused:
                t_wall = time.time()
                for a in ARMS:
                    if msgs[a] is not None:
                        if self._build_target_arm(a, msgs[a], t_wall):
                            updated_any = True

            if updated_any:
                self._no_msg_t = time.time()
                step_count += 1
                self._solve_and_publish(step_count, t_step)
            else:
                self._print_idle(step_count)

            sleep_sec = dt_sec - (time.perf_counter() - t_step)
            if sleep_sec > 0:
                time.sleep(sleep_sec)

    def _solve_and_publish(self, step_count: int, t_step: float):
        """一步雙臂 QP solve → per-arm LPF → publish → 簿記 → CSV。"""
        targets, seeds = {}, {}
        with self._joints_lock:
            for a in ARMS:
                s = self.S[a]
                targets[a] = {"xyz": s["target_xyz"],
                              "R":   _quat_to_rot(*s["target_q"])}
                seeds[a]   = list(s["filt_joints"] or s["last_joints"])

        t0  = time.perf_counter()
        res = self._session.solve_step(targets, seeds, no_rot=self._no_rot)
        ik_ms = res["timing"]["solve_ms"]
        total_ms = (time.perf_counter() - t0) * 1000.0

        joints_out = {}
        for a in ARMS:
            r = res[a]
            joints_out[a] = self._filter_joints(a, r["joints"])
            with self._joints_lock:
                self.S[a]["last_joints"] = list(joints_out[a])

            # _pose 簿記（Set2-D 語意；fail 改用 ee_cmd_xyz = 發布值的 FK）
            s = self.S[a]
            if r["success"]:
                s["pose"] = (list(map(float, s["target_xyz"]))
                             + list(s["target_q"]))
                s["fail_streak"] = 0
            else:
                ee = r["ee_cmd_xyz"]
                s["pose"][0], s["pose"][1], s["pose"][2] = ee[0], ee[1], ee[2]
                s["fail_streak"] += 1
                if s["fail_streak"] % 20 == 0:
                    print(f"\n  [{a}] IK pos_err={r['pos_err_mm']:.1f}mm "
                          f"streak={s['fail_streak']} — publishing partial")

            if not self.args.dry_run:
                self._publish_fwd(a, joints_out[a])
                ik_cmd = JointState()
                ik_cmd.header.stamp = self.get_clock().now().to_msg()
                ik_cmd.name     = list(s["cfg"]["joint_names"])
                ik_cmd.position = list(joints_out[a])
                s["ik_cmd_pub"].publish(ik_cmd)
            s["latency_pub"].publish(Float32(data=float(ik_ms)))

        loop_wall_ms = (time.perf_counter() - t_step) * 1000.0
        deadline_missed = int(loop_wall_ms > self._deadline_ms)
        self._write_step(res, joints_out, ik_ms, total_ms,
                         deadline_missed, step_count)

    # ── 記錄 ──────────────────────────────────────────────────────────────────
    def _write_step(self, res, joints_out, ik_ms, total_ms,
                    deadline_missed, step_count):
        tm = res["timing"]
        events = [a for a in ARMS if self.S[a].pop("reanchor_event", False)]
        for a in ARMS:
            self.S[a]["reanchor_event"] = False

        self._profile_pub.publish(String(data=json.dumps({
            "step":            step_count,
            "ik_ms":           round(ik_ms, 3),
            "iterations":      tm["iterations"],
            "ee_dist_mm":      round(res["ee_dist_after_m"] * 1000.0, 1),
            "ee_dist_cmd_mm":  round(res["ee_dist_cmd_m"] * 1000.0, 1),
            "prox_scale":      round(res["prox_scale"], 3),
            "deadline_missed": deadline_missed,
            **{f"{_PFX[a]}_{k}": round(float(res[a][k]), 4)
               for a in ARMS
               for k in ("pos_err_mm", "ori_err_deg", "sigma_min",
                         "budget_scale")},
            **{f"{_PFX[a]}_success": res[a]["success"] for a in ARMS},
        })))

        row = {
            "t":               round(time.time(), 6),
            "event":           "+".join(f"reanchor_{a}" for a in events),
            "ik_ms":           round(ik_ms, 4),
            "setup_ms":        round(tm["setup_ms"], 4),
            "loop_ms":         round(tm["loop_ms"], 4),
            "iterations":      tm["iterations"],
            "deadline_missed": deadline_missed,
            "ee_dist_mm":      round(res["ee_dist_after_m"] * 1000.0, 2),
            "ee_dist_cmd_mm":  round(res["ee_dist_cmd_m"] * 1000.0, 2),
            "prox_scale":      round(res["prox_scale"], 4),
            "mem_kb":          tm["mem_kb"],
        }
        for a in ARMS:
            p, s, r = _PFX[a], self.S[a], res[a]
            track = float(np.linalg.norm(
                np.array(r["ee_cmd_xyz"]) - s["target_xyz"])) * 1000.0
            row.update({
                f"{p}_x": round(s["pose"][0], 6),
                f"{p}_y": round(s["pose"][1], 6),
                f"{p}_z": round(s["pose"][2], 6),
                f"{p}_success":      r["success"],
                f"{p}_pos_err_mm":   round(r["pos_err_mm"], 4),
                f"{p}_ori_err_deg":  round(r["ori_err_deg"], 4),
                f"{p}_track_err_mm": round(track, 4),
                f"{p}_sigma_min":    round(r["sigma_min"], 6),
                f"{p}_lambda_extra": round(r["lambda_extra"], 8),
                f"{p}_w_ori":        round(r["w_ori"], 4),
                f"{p}_budget_scale": round(r["budget_scale"], 4),
            })
            for i, q in enumerate(joints_out[a][:7]):
                row[f"{p}_q_cmd_{i}"] = round(float(q), 6)
            tf = s["tf_poller"].get()
            if tf is not None:
                row[f"{p}_tf_x"] = round(tf[0], 6)
                row[f"{p}_tf_y"] = round(tf[1], 6)
                row[f"{p}_tf_z"] = round(tf[2], 6)
        self._csv_writer.put(row)
        self._records.append(row)

        self._stat_n    += 1
        self._stat_miss += deadline_missed
        for a in ARMS:
            self._stat_ok[a] += int(res[a]["success"])
        self._ring_ik.append(ik_ms)

        if step_count % 5 == 0 or self.args.verbose:
            tags = "".join("✓" if res[a]["success"] else "✗" for a in ARMS)
            print(f"\r  {step_count:5d}  ik={ik_ms:6.2f}ms  it={tm['iterations']:3d}"
                  f"  R={res['right']['pos_err_mm']:5.1f}mm"
                  f"  L={res['left']['pos_err_mm']:5.1f}mm"
                  f"  dist={res['ee_dist_after_m']*100:5.1f}cm"
                  f"  prox={res['prox_scale']:.2f}  {tags}",
                  end="", flush=True)
        if step_count % 50 == 0:
            n = self._stat_n
            med = float(np.median(list(self._ring_ik))) if self._ring_ik else 0.0
            print(f"\n  ─── [{n}] sr R={self._stat_ok['right']/n*100:.0f}%"
                  f" L={self._stat_ok['left']/n*100:.0f}%"
                  f"  ik_med={med:.2f}ms"
                  f"  ddl={self._stat_miss}/{n}")

    def _print_idle(self, step_count: int):
        idle_s = time.time() - self._no_msg_t
        if idle_s >= 2.0:
            self._no_msg_t = time.time()
            paused = "  [PAUSED]" if self._kbd.paused else ""
            print(f"\r  [WAIT]{paused}  cb={self._msg_count}  steps={step_count}"
                  f"  idle={idle_s:.0f}s  ×{self._kbd.scale:.2f}",
                  end="", flush=True)

    # ── Final stats / plot ────────────────────────────────────────────────────
    def print_final_stats(self):
        rows = list(self._records)
        n = self._stat_n
        if n == 0:
            return
        print(f"\n\n{'═'*65}")
        print(f"  Bimanual final stats  n={n}"
              + (f"  (metrics from last {len(rows)} steps)" if len(rows) < n else ""))
        print(f"{'═'*65}")
        for key, unit in [
            ("ik_ms",       "ms — QP total"),
            ("setup_ms",    "ms — task setup"),
            ("loop_ms",     "ms — solve loop"),
            ("iterations",  "iters"),
            ("ee_dist_mm",     "mm — EE-EE distance (QP solution)"),
            ("ee_dist_cmd_mm", "mm — EE-EE distance (published cmd)"),
            ("prox_scale",  "— proximity throttle"),
        ]:
            vals = [float(r.get(key, 0) or 0) for r in rows]
            print(f"  {key:<16} mean={np.mean(vals):8.3f}  "
                  f"median={np.median(vals):8.3f}  "
                  f"p95={np.percentile(vals,95):8.3f}  [{unit}]")
        for a in ARMS:
            p = _PFX[a]
            for key in (f"{p}_pos_err_mm", f"{p}_sigma_min", f"{p}_budget_scale"):
                vals = [float(r.get(key, 0) or 0) for r in rows]
                print(f"  {key:<16} mean={np.mean(vals):8.3f}  "
                      f"p95={np.percentile(vals,95):8.3f}")
            print(f"  success_{a:<8} : {self._stat_ok[a]}/{n} "
                  f"= {self._stat_ok[a]/n*100:.1f}%")
        print(f"  deadline_miss   : {self._stat_miss}/{n} "
              f"(budget={self._deadline_ms:.1f}ms)")
        print(f"\n  CSV: {self._csv_path}")
        try:
            self._save_plot()
        except Exception as e:
            print(f"  [plot] skipped ({e})")

    def _save_plot(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from placo_ik_session_bimanual import _DLS_SIGMA_THRESH

        rows = list(self._records)
        if not rows:
            return
        plot_path = self.args.plot or _png_for(self._csv_path)
        t = list(range(len(rows)))

        def col(key):
            return [float(r.get(key, 0) or 0) for r in rows]

        fig, axes = plt.subplots(3, 2, figsize=(14, 12))
        fig.suptitle(f"Placo Bimanual Single-QP  n={len(rows)}  "
                     f"rate={self._rate_hz:.0f}Hz", fontsize=11)

        ax = axes[0, 0]
        ax.plot(t, col("ik_ms"), linewidth=0.7, color="steelblue")
        ax.axhline(self._deadline_ms, color="red", linestyle="--",
                   label=f"deadline={self._deadline_ms:.0f}ms")
        ax.set_title("QP solve time (ms)"); ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        ax = axes[0, 1]
        ax.plot(t, col("iterations"), linewidth=0.7, color="purple")
        ax.set_title("Iterations"); ax.grid(True, alpha=0.3)

        ax = axes[1, 0]
        ax.plot(t, col("r_pos_err_mm"), linewidth=0.7, label="right",
                color="tomato")
        ax.plot(t, col("l_pos_err_mm"), linewidth=0.7, label="left",
                color="royalblue")
        ax.set_title("IK residual (mm)"); ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        ax = axes[1, 1]
        ax.plot(t, col("r_sigma_min"), linewidth=0.7, label="right σ_min",
                color="tomato")
        ax.plot(t, col("l_sigma_min"), linewidth=0.7, label="left σ_min",
                color="royalblue")
        ax.axhline(_DLS_SIGMA_THRESH, color="red", linestyle="--",
                   label=f"σ_thresh={_DLS_SIGMA_THRESH}")
        ax.set_title("σ_min per arm"); ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        ax = axes[2, 0]
        ax.plot(t, col("ee_dist_cmd_mm"), linewidth=0.7, color="darkgreen",
                label="cmd (actual)")
        ax.plot(t, col("ee_dist_mm"), linewidth=0.7, color="gray", alpha=0.5,
                label="QP solution")
        ax.axhline(float(self.args.min_ee_dist) * 1000.0, color="orange",
                   linestyle="--", label="min_ee_dist")
        ax.set_title("EE-EE distance (mm)"); ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        ax = axes[2, 1]
        ax.plot(t, col("r_budget_scale"), linewidth=0.7, label="right",
                color="tomato")
        ax.plot(t, col("l_budget_scale"), linewidth=0.7, label="left",
                color="royalblue")
        ax.plot(t, col("prox_scale"), linewidth=0.7, label="prox",
                color="darkgreen", alpha=0.6)
        ax.set_ylim(-0.05, 1.05)
        ax.set_title("Δq budget scale / proximity throttle")
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(plot_path, dpi=120)
        plt.close()
        print(f"  [plot] → {plot_path}")

    # ── Cleanup / banner ──────────────────────────────────────────────────────
    def destroy_node(self):
        self._kbd.stop()
        self._csv_writer.close()
        super().destroy_node()

    def _print_banner(self):
        a = self.args
        print(f"\n{'═'*65}")
        print(f"  Placo Bimanual Single-QP IK  (v1)")
        print(f"  rate={self._rate_hz:.0f}Hz  deadline={self._deadline_ms:.1f}ms"
              f"  max_iter={a.max_iter}")
        for arm in ARMS:
            s = self.S[arm]
            ws = (f"mesh({s['ws_mesh'].summary()['n_reachable_voxels']}vox)"
                  if s["ws_mesh"] is not None
                  else ("box" if s["ws_clamp"] else "OFF"))
            print(f"  [{arm:>5}] ee_delta={s['cfg']['ee_delta_topic']}"
                  f"  cmd={s['cfg']['fwd_cmd_topic']}")
            print(f"  [{arm:>5}] calib_yaw={s['calib_yaw_deg']:.0f}°  ws={ws}"
                  f"  lpf={'%.2f' % s['lpf_alpha'][0] if s['lpf_active'] else 'OFF'}")
        print(f"  Δq budget : {a.tick_budget_deg:.1f}°/tick (uniform scaling,"
              f" replaces per-joint jump guard)")
        print(f"  EE guard  : throttle below {a.min_ee_dist*100:.0f}cm"
              f" (approach only)")
        print(f"  rot_track : {'OFF' if self._no_rot else 'ON'}"
              f"  ori_lpf={'%.2f' % self._ori_alpha if self._ori_active else 'OFF'}")
        print(f"  service   : /bimanual/go_home  (std_srvs/Trigger,"
              f" 兩臂同時回 home)")
        print(f"  CSV       : {self._csv_path}")
        print(f"{'═'*65}")
        print(f"  1-9=scale(auto re-anchor)  +/-=fine  p=pause  r=reset"
              f"  h=home  Ctrl-C=quit")
        print(f"{'═'*65}\n")
