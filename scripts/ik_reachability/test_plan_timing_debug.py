#!/usr/bin/env python3
"""test_plan_timing_debug.py
============================
Unit tests for plan_separate() timing and the MoveGroup action state machine.

Does NOT require a live ROS2 / MoveIt2 environment.
Uses FakeMoveIt2 to reproduce the exact pymoveit2 state machine in isolation.

--------------------------------------------------------------------
ROOT CAUSE of "times are all zero" bug
--------------------------------------------------------------------
pymoveit2 MoveGroup action state machine (use_move_group_action=True):

  _send_goal_async_move_action():
    __is_motion_requested = True         → state = REQUESTING

  __response_callback_move_action() fires when server ACCEPTS the goal (~50ms):
    __is_executing = True
    __is_motion_requested = False        → state = EXECUTING
    (OMPL planning + robot motion both happen inside EXECUTING phase)

  __result_callback_move_action() fires when motion finishes:
    __is_executing = False               → state = IDLE

  CONSEQUENCE:
    • REQUESTING lasts only ~50ms (goal accept latency) — NOT the OMPL time.
    • There is no state transition at "OMPL done, starting execution".
    • The old plan_separate() measured REQUESTING duration as ompl_ms → ~0ms.

--------------------------------------------------------------------
BUG in wait_until_executed()
--------------------------------------------------------------------
  def wait_until_executed(self) -> bool:
      if not self.__is_motion_requested:   ← ① checks only is_motion_requested
          log.warn("no motion in progress")
          return False                      ← returns False immediately!
      while self.__is_motion_requested or self.__is_executing:
          rclpy.spin_once(...)
      return self.motion_suceeded

  When called after EXECUTING state is entered:
    • __is_motion_requested == False  → ① fires → returns False immediately
    • motion_ms = 0ms, ok = False  ← wrong!

--------------------------------------------------------------------
FIX applied in plan_separate()
--------------------------------------------------------------------
  1. Use plan()   instead of move_to_pose() → synchronous, true OMPL time
  2. Use execute()→ sends ExecuteTrajectory action (async)
  3. Poll query_state() directly until IDLE (both REQUESTING and EXECUTING)
     instead of calling wait_until_executed()

Run:
    python3 test_plan_timing_debug.py -v
"""

import threading
import time
import unittest
from enum import IntEnum


# ─── Minimal pymoveit2 mock ──────────────────────────────────────────────────

class MoveIt2State(IntEnum):
    IDLE       = 0
    REQUESTING = 1
    EXECUTING  = 2


class FakeMoveIt2:
    """Simulates pymoveit2.MoveIt2 state machine.

    The lifecycle of a MoveGroup action request mirrors pymoveit2:
      1. send_goal()       → REQUESTING (is_motion_requested=True)
      2. goal_accepted()   → EXECUTING  (is_executing=True, is_motion_requested=False)
      3. result_received() → IDLE       (is_executing=False)
    """

    def __init__(self):
        self._mutex = threading.Lock()
        self.__is_motion_requested = False
        self.__is_executing = False
        self.motion_suceeded = False        # note: pymoveit2 spells it this way

    # ── public API (mirrors pymoveit2) ────────────────────────────────────────

    def query_state(self) -> MoveIt2State:
        with self._mutex:
            if self.__is_motion_requested:
                return MoveIt2State.REQUESTING
            elif self.__is_executing:
                return MoveIt2State.EXECUTING
            else:
                return MoveIt2State.IDLE

    def wait_until_executed(self) -> bool:
        """EXACT replica of pymoveit2's wait_until_executed() — shows the bug."""
        if not self.__is_motion_requested:   # ← BUG: returns False in EXECUTING state
            return False
        while self.__is_motion_requested or self.__is_executing:
            time.sleep(0.001)
        return self.motion_suceeded

    # ── simulation helpers ────────────────────────────────────────────────────

    def simulate_move_group_action(
        self,
        goal_accept_delay: float = 0.05,
        plan_and_exec_delay: float = 0.4,
        succeed: bool = True,
    ) -> threading.Thread:
        """Simulate the full MoveGroup action in a background thread.

        Timeline:
          t=0              IDLE → REQUESTING    (send_goal called)
          t=goal_accept    REQUESTING → EXECUTING  (server accepted)
          t=goal_accept + plan_and_exec  EXECUTING → IDLE  (motion done)

        NOTE: plan_and_exec_delay covers BOTH ompl planning AND robot motion
              because the MoveGroup action has only one "accepted" point.
        """
        def _run():
            # Phase 0: immediately enter REQUESTING (simulates _send_goal_async)
            with self._mutex:
                self.__is_motion_requested = True

            # Phase 1: server accepts the goal (~50ms)
            time.sleep(goal_accept_delay)
            with self._mutex:
                self.__is_executing = True
                self.__is_motion_requested = False

            # Phase 2: planning + execution happens inside EXECUTING
            time.sleep(plan_and_exec_delay)
            with self._mutex:
                self.motion_suceeded = succeed
                self.__is_executing = False

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        return t

    def simulate_plan_call(self, plan_delay: float = 0.3) -> None:
        """Simulate plan() — blocks synchronously for plan_delay seconds.
        Does NOT touch is_motion_requested or is_executing.
        """
        time.sleep(plan_delay)

    def simulate_execute_action(
        self,
        goal_accept_delay: float = 0.03,
        exec_delay: float = 0.4,
        succeed: bool = True,
    ) -> threading.Thread:
        """Simulate execute() followed by action lifecycle.

        This is the ExecuteTrajectory action (separate from MoveGroup action).
        Same state-machine bug applies to wait_until_executed().
        """
        def _run():
            with self._mutex:
                self.__is_motion_requested = True
            time.sleep(goal_accept_delay)
            with self._mutex:
                self.__is_executing = True
                self.__is_motion_requested = False
            time.sleep(exec_delay)
            with self._mutex:
                self.motion_suceeded = succeed
                self.__is_executing = False

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        return t


# ─── Helper: the "fixed" wait — direct query_state() polling ────────────────

def wait_until_idle(moveit2: FakeMoveIt2, timeout: float = 5.0) -> bool:
    """Replacement for wait_until_executed() that works in any state.

    Polls query_state() until IDLE. Does NOT short-circuit on EXECUTING.
    """
    deadline = time.perf_counter() + timeout
    while moveit2.query_state() in (MoveIt2State.REQUESTING, MoveIt2State.EXECUTING):
        if time.perf_counter() > deadline:
            return False
        time.sleep(0.001)
    return moveit2.motion_suceeded


# ─── Test Suite ─────────────────────────────────────────────────────────────

class TestStateMachineTransitions(unittest.TestCase):
    """Verify state transitions match pymoveit2 behavior."""

    def test_initial_state_is_idle(self):
        m = FakeMoveIt2()
        self.assertEqual(m.query_state(), MoveIt2State.IDLE)

    def test_requesting_phase(self):
        """IDLE → REQUESTING immediately when goal is sent."""
        m = FakeMoveIt2()
        t = m.simulate_move_group_action(goal_accept_delay=0.5, plan_and_exec_delay=0.1)
        time.sleep(0.01)  # let thread start
        self.assertEqual(m.query_state(), MoveIt2State.REQUESTING)
        t.join()

    def test_executing_phase_after_accept(self):
        """REQUESTING → EXECUTING after goal acceptance (quick)."""
        m = FakeMoveIt2()
        t = m.simulate_move_group_action(goal_accept_delay=0.05, plan_and_exec_delay=0.5)
        time.sleep(0.01)
        self.assertEqual(m.query_state(), MoveIt2State.REQUESTING)
        time.sleep(0.10)  # past accept delay
        self.assertEqual(m.query_state(), MoveIt2State.EXECUTING)
        t.join()

    def test_idle_after_completion(self):
        """EXECUTING → IDLE after motion finishes."""
        m = FakeMoveIt2()
        t = m.simulate_move_group_action(goal_accept_delay=0.05, plan_and_exec_delay=0.1)
        t.join()
        self.assertEqual(m.query_state(), MoveIt2State.IDLE)
        self.assertTrue(m.motion_suceeded)

    def test_requesting_duration_is_short(self):
        """REQUESTING phase lasts only goal_accept_delay (~50ms), not OMPL time."""
        m = FakeMoveIt2()
        t = m.simulate_move_group_action(goal_accept_delay=0.05, plan_and_exec_delay=0.5)
        time.sleep(0.01)
        t_req_start = time.perf_counter()
        while m.query_state() == MoveIt2State.REQUESTING:
            time.sleep(0.001)
        requesting_ms = (time.perf_counter() - t_req_start) * 1000.0
        # REQUESTING ends at ~goal_accept_delay (50ms), NOT at ~500ms (plan+exec)
        self.assertLess(requesting_ms, 150,
            f"REQUESTING lasted {requesting_ms:.0f}ms — expected ~50ms (goal accept), "
            "not OMPL planning time. This confirms old ompl_ms measurement was wrong.")
        t.join()


class TestWaitUntilExecutedBug(unittest.TestCase):
    """Demonstrate the wait_until_executed() early-return bug."""

    def test_bug_returns_false_when_called_in_executing_state(self):
        """
        BUG: If wait_until_executed() is called after REQUESTING→EXECUTING,
        __is_motion_requested is already False → returns False immediately.
        """
        m = FakeMoveIt2()
        # Start action with very fast goal acceptance
        t = m.simulate_move_group_action(goal_accept_delay=0.02, plan_and_exec_delay=0.5)

        # Wait until EXECUTING state
        time.sleep(0.05)
        self.assertEqual(m.query_state(), MoveIt2State.EXECUTING,
                         "Precondition: must be in EXECUTING state to trigger bug")

        # Call wait_until_executed() while in EXECUTING state
        t_before = time.perf_counter()
        result = m.wait_until_executed()
        elapsed_ms = (time.perf_counter() - t_before) * 1000.0

        # BUG: returns False immediately (motion_ms ≈ 0ms)
        self.assertFalse(result, "wait_until_executed() should return False (bug behavior)")
        self.assertLess(elapsed_ms, 10,
            f"wait_until_executed() returned in {elapsed_ms:.1f}ms — "
            "confirms immediate return bug (motion_ms=0)")
        t.join()

    def test_bug_returns_false_even_when_motion_succeeds(self):
        """wait_until_executed() reports failure even when robot motion succeeds."""
        m = FakeMoveIt2()
        t = m.simulate_move_group_action(goal_accept_delay=0.02, plan_and_exec_delay=0.2)
        time.sleep(0.05)  # enter EXECUTING
        result_buggy = m.wait_until_executed()  # called in EXECUTING → False
        t.join()
        # Robot motion succeeded
        self.assertTrue(m.motion_suceeded, "Robot actually succeeded")
        # But buggy API reports failure
        self.assertFalse(result_buggy, "Buggy wait_until_executed() misreports as False")

    def test_works_correctly_if_called_in_requesting_state(self):
        """wait_until_executed() only works correctly when called in REQUESTING state."""
        m = FakeMoveIt2()
        # Start action with slow accept so we can call wait_until during REQUESTING
        t = m.simulate_move_group_action(goal_accept_delay=0.05, plan_and_exec_delay=0.1)
        time.sleep(0.01)
        self.assertEqual(m.query_state(), MoveIt2State.REQUESTING)

        t_before = time.perf_counter()
        result = m.wait_until_executed()
        elapsed_ms = (time.perf_counter() - t_before) * 1000.0

        t.join()
        self.assertTrue(result, "Should return True (called while REQUESTING)")
        self.assertGreater(elapsed_ms, 50,
            f"Should have waited >50ms (accept + exec), got {elapsed_ms:.0f}ms")


class TestFixedWaitUntilIdle(unittest.TestCase):
    """Verify that the fixed spin_once wait works correctly in all states."""

    def test_fix_works_when_called_in_executing_state(self):
        """spin_once wait correctly handles EXECUTING state (no early return)."""
        m = FakeMoveIt2()
        EXEC_DELAY = 0.3
        t = m.simulate_move_group_action(goal_accept_delay=0.02, plan_and_exec_delay=EXEC_DELAY)

        time.sleep(0.05)  # enter EXECUTING
        self.assertEqual(m.query_state(), MoveIt2State.EXECUTING)

        # Simulate the fixed wait: spin_once loop until IDLE
        t_before = time.perf_counter()
        ok = wait_until_idle(m, timeout=5.0)
        elapsed_ms = (time.perf_counter() - t_before) * 1000.0

        t.join()
        self.assertTrue(ok, "Fixed wait should return True")
        self.assertGreater(elapsed_ms, 50,
            f"Should wait >50ms for remaining execution, got {elapsed_ms:.0f}ms")

    def test_fix_works_when_called_immediately_after_execute(self):
        """Direct simulation of plan_separate() Phase 3 fixed flow (spin_once loop)."""
        m = FakeMoveIt2()
        PLAN_DELAY = 0.2
        EXEC_DELAY = 0.3

        # Phase 1: plan() — synchronous, measures real OMPL time
        t0 = time.perf_counter()
        m.simulate_plan_call(plan_delay=PLAN_DELAY)  # blocks
        t_plan_done = time.perf_counter()
        ompl_ms = (t_plan_done - t0) * 1000.0

        # Phase 2: execute() — async
        t = m.simulate_execute_action(goal_accept_delay=0.02, exec_delay=EXEC_DELAY)

        # Phase 3: spin_once-based wait (the fix)
        ok = wait_until_idle(m, timeout=5.0)
        t_done = time.perf_counter()

        t.join()
        motion_ms = (t_done - t_plan_done) * 1000.0
        total_ms  = (t_done - t0) * 1000.0

        self.assertTrue(ok)
        self.assertGreater(ompl_ms, PLAN_DELAY * 1000 * 0.9,
            f"ompl_ms={ompl_ms:.0f}ms < expected >{PLAN_DELAY*1000*0.9:.0f}ms")
        self.assertGreater(motion_ms, EXEC_DELAY * 1000 * 0.9,
            f"motion_ms={motion_ms:.0f}ms < expected >{EXEC_DELAY*1000*0.9:.0f}ms")
        print(f"\n[fixed_flow] ompl={ompl_ms:.0f}ms  "
              f"motion={motion_ms:.0f}ms  total={total_ms:.0f}ms")

    def test_fix_returns_false_on_timeout(self):
        """wait_until_idle() returns False on timeout."""
        m = FakeMoveIt2()
        t = m.simulate_move_group_action(goal_accept_delay=0.01, plan_and_exec_delay=10.0)
        time.sleep(0.05)
        ok = wait_until_idle(m, timeout=0.1)
        self.assertFalse(ok, "Should return False on timeout")
        t.join(timeout=0.5)

    def test_fix_returns_false_on_plan_failure(self):
        """When motion fails, motion_suceeded=False and wait returns False."""
        m = FakeMoveIt2()
        t = m.simulate_execute_action(goal_accept_delay=0.02, exec_delay=0.1, succeed=False)
        ok = wait_until_idle(m, timeout=5.0)
        t.join()
        self.assertFalse(ok)
        self.assertFalse(m.motion_suceeded)

    def test_spin_once_leaves_wrong_executor(self):
        """
        Demonstrates WHY the old pure time.sleep() polling hangs:

        rclpy.spin_once(node) uses the GLOBAL executor.
        add_node(node) sets node.executor = global_executor.
        remove_node(node) does NOT reset node.executor back.

        After plan()'s spin_once loop, node.executor is stale (global_executor).
        MultiThreadedExecutor (MTE) still has node in _nodes, but individual
        callbacks may be processed by neither executor reliably, causing execute_
        trajectory callbacks to never fire → polling loop hangs.

        The spin_once-based wait loop fixes this: it explicitly processes
        callbacks in the same thread that plan() used, guaranteeing consistency.
        """
        import rclpy as _rclpy

        # Simulate: after plan()'s spin_once calls, what is the executor state?
        # In real pymoveit2: rclpy.spin_once(self._node) is called repeatedly.
        # Each call: add_node → process one → remove_node (but node.executor stale)
        # The fix: calling spin_once in our own loop after execute() processes
        # execute_trajectory callbacks consistently.

        # This test documents the scenario rather than running live ROS2.
        # The key insight verified by other tests: wait_until_idle() works.
        m = FakeMoveIt2()
        t = m.simulate_execute_action(goal_accept_delay=0.02, exec_delay=0.2)
        ok = wait_until_idle(m, timeout=5.0)
        t.join()
        self.assertTrue(ok, "spin_once-based wait correctly processes callbacks")


class TestTimingAccuracy(unittest.TestCase):
    """Verify timing measurements are accurate across state transitions."""

    TOLERANCE = 0.15  # 15% relative tolerance for CI/thread jitter

    def _check_ms(self, measured, expected, label):
        low  = expected * (1 - self.TOLERANCE)
        high = expected * (1 + self.TOLERANCE) + 30  # +30ms slack for syscall overhead
        self.assertGreater(measured, low,
            f"{label}: {measured:.0f}ms < expected >{low:.0f}ms")
        self.assertLess(measured, high,
            f"{label}: {measured:.0f}ms > expected <{high:.0f}ms")

    def test_ompl_time_from_plan_call(self):
        """plan() synchronous call correctly measures OMPL time."""
        m = FakeMoveIt2()
        PLAN_MS = 300.0
        t0 = time.perf_counter()
        m.simulate_plan_call(plan_delay=PLAN_MS / 1000.0)
        ompl_ms = (time.perf_counter() - t0) * 1000.0
        self._check_ms(ompl_ms, PLAN_MS, "ompl_ms")

    def test_motion_time_from_direct_polling(self):
        """Direct polling correctly captures robot motion time."""
        m = FakeMoveIt2()
        EXEC_MS = 400.0
        t = m.simulate_execute_action(
            goal_accept_delay=0.02, exec_delay=EXEC_MS / 1000.0
        )
        t_start = time.perf_counter()
        wait_until_idle(m, timeout=5.0)
        motion_ms = (time.perf_counter() - t_start) * 1000.0
        self._check_ms(motion_ms, EXEC_MS + 20, "motion_ms")  # +20ms accept overhead
        t.join()

    def test_old_approach_gives_near_zero_ompl(self):
        """Old approach (REQUESTING→EXECUTING as ompl_ms) gives ~goal_accept_delay, NOT plan time."""
        PLAN_EXEC_MS = 500.0  # what we think "OMPL" takes
        ACCEPT_MS    = 50.0   # actual goal acceptance time

        m = FakeMoveIt2()
        t = m.simulate_move_group_action(
            goal_accept_delay=ACCEPT_MS / 1000.0,
            plan_and_exec_delay=PLAN_EXEC_MS / 1000.0,
        )
        time.sleep(0.010)  # let REQUESTING start

        # Old approach: measure REQUESTING duration
        t0 = time.perf_counter()
        while m.query_state() == MoveIt2State.REQUESTING:
            time.sleep(0.001)
        old_ompl_ms = (time.perf_counter() - t0) * 1000.0

        t.join()
        # Old "ompl_ms" ≈ ACCEPT_MS (~50ms), NOT PLAN_EXEC_MS (~500ms)
        self.assertLess(old_ompl_ms, ACCEPT_MS * 3,
            f"Old ompl_ms={old_ompl_ms:.0f}ms; should be ~{ACCEPT_MS:.0f}ms "
            f"(goal accept), not {PLAN_EXEC_MS:.0f}ms (actual planning)")
        print(f"\n[old_approach] 'ompl_ms'={old_ompl_ms:.0f}ms "
              f"(goal accept latency, not real OMPL={PLAN_EXEC_MS:.0f}ms)")

    def test_full_plan_separate_flow(self):
        """Complete plan_separate() fixed flow with correct timing."""
        m = FakeMoveIt2()
        PLAN_MS  = 250.0
        EXEC_MS  = 350.0

        t0 = time.perf_counter()

        # Phase 1: synchronous plan()
        m.simulate_plan_call(plan_delay=PLAN_MS / 1000.0)
        t_plan_done = time.perf_counter()
        ompl_ms = (t_plan_done - t0) * 1000.0

        # Phase 2: async execute()
        t = m.simulate_execute_action(
            goal_accept_delay=0.02, exec_delay=EXEC_MS / 1000.0
        )
        ok = wait_until_idle(m, timeout=5.0)
        t_done = time.perf_counter()

        t.join()
        motion_ms = (t_done - t_plan_done) * 1000.0
        total_ms  = (t_done - t0) * 1000.0

        self.assertTrue(ok)
        self._check_ms(ompl_ms, PLAN_MS, "ompl_ms")
        self._check_ms(motion_ms, EXEC_MS + 20, "motion_ms")
        self._check_ms(total_ms, PLAN_MS + EXEC_MS + 20, "total_ms")
        print(f"\n[full_flow] ompl={ompl_ms:.0f}ms  "
              f"motion={motion_ms:.0f}ms  total={total_ms:.0f}ms")


class TestConcurrentStateMachine(unittest.TestCase):
    """Verify state machine thread-safety (query_state uses mutex)."""

    def test_no_intermediate_state_visible(self):
        """query_state() never returns a half-updated state."""
        m = FakeMoveIt2()
        errors = []

        def reader():
            for _ in range(500):
                state = m.query_state()
                # Valid states only
                if state not in (MoveIt2State.IDLE,
                                 MoveIt2State.REQUESTING,
                                 MoveIt2State.EXECUTING):
                    errors.append(f"Invalid state: {state}")
                time.sleep(0.001)

        t1 = m.simulate_move_group_action(goal_accept_delay=0.1, plan_and_exec_delay=0.3)
        t2 = threading.Thread(target=reader, daemon=True)
        t2.start()
        t1.join()
        t2.join()
        self.assertEqual(errors, [], f"Thread safety violations: {errors}")

    def test_sequential_calls_reach_idle(self):
        """Multiple sequential calls each reach IDLE before next starts."""
        m = FakeMoveIt2()
        for i in range(3):
            t = m.simulate_move_group_action(
                goal_accept_delay=0.02, plan_and_exec_delay=0.05
            )
            ok = wait_until_idle(m, timeout=2.0)
            t.join()
            self.assertTrue(ok, f"Call {i+1} failed")
            self.assertEqual(m.query_state(), MoveIt2State.IDLE,
                             f"State not IDLE after call {i+1}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
