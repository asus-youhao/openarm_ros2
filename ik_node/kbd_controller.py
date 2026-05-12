#!/usr/bin/env python3
"""
kbd_controller.py
=================
Standalone non-blocking keyboard controller for IK teleoperation.

Runs a raw-mode termios daemon thread.  The main control loop calls
``pop_xyz()`` / ``pop_rpy()`` each iteration to consume accumulated deltas.

XYZ (position) keys  — KEYBOARD mode only
------------------------------------------
  w / s    +Y / −Y   (forward / back)
  a / d    −X / +X   (left  / right)
  q / e    +Z / −Z   (up    / down)

RPY (orientation) keys — KEYBOARD mode only
--------------------------------------------
  i / k    pitch +/−  (rotate around Y)
  j / l    yaw   +/−  (rotate around Z)
  u / o    roll  +/−  (rotate around X)

General keys (always active)
-----------------------------
  t       toggle input mode: TRACKER ↔ KEYBOARD
  1-9     scale preset  (0.10× … 5.00×)
  + / =   scale ×1.25
  -       scale ×0.80
  0       reset scale to 1.0
  p       pause / resume publishing
  r       request session-reference reset on next step
  v       toggle verbose print
  h       request send-home
  ?       print this help
  Ctrl-C  quit

Scale is applied at key-press time: ``pop_xyz()`` / ``pop_rpy()`` return
already-scaled values that can be used directly by the IK loop.
"""

import math
import select
import sys
import termios
import threading
from typing import Optional, Tuple

# ── Scale presets ─────────────────────────────────────────────────────────────
SCALE_PRESETS: dict = {
    "1": 0.10,   # very fine  (10 %)
    "2": 0.25,   # fine       (25 %)
    "3": 0.50,   # half       (50 %)
    "4": 0.75,   # gentle     (75 %)
    "5": 1.00,   # normal    (100 %)  ← default
    "6": 1.50,   # boosted   (150 %)
    "7": 2.00,   # double    (200 %)
    "8": 3.00,   # large     (300 %)
    "9": 5.00,   # maximum   (500 %)
}


class KbdController:
    """
    Non-blocking terminal keyboard controller.

    Instantiate once, call ``start()`` to begin the daemon thread, and
    ``stop()`` on shutdown to restore terminal settings.

    Parameters
    ----------
    step_m : float
        XYZ step size in metres per key-press at scale 1× (default 0.005 m).
    step_deg : float
        RPY step size in degrees per key-press at scale 1× (default 2°).
    init_mode : str
        Starting input mode, either ``"tracker"`` or ``"keyboard"``.
    """

    def __init__(
        self,
        step_m:    float = 0.005,
        step_deg:  float = 2.0,
        init_mode: str   = "tracker",
    ):
        # ── Public flags (read from main loop) ────────────────────────────
        self.scale:        float = 1.0
        self.paused:       bool  = False
        self.reset_ref:    bool  = False
        self.verbose:      bool  = False
        self.request_home: bool  = False
        self.request_quit: bool  = False
        self.mode:         str   = init_mode   # "tracker" | "keyboard"

        # ── Configuration ─────────────────────────────────────────────────
        self._step_m:   float = step_m
        self._step_rad: float = math.radians(step_deg)

        # ── Accumulated deltas (consumed via pop_*) ────────────────────────
        self._xyz: list = [0.0, 0.0, 0.0]   # metres, scale already applied
        self._rpy: list = [0.0, 0.0, 0.0]   # radians, scale already applied

        # ── Internal ──────────────────────────────────────────────────────
        self._lock          = threading.Lock()
        self._old_settings  = None
        self._thread        = threading.Thread(
            target=self._run, daemon=True, name="kbd_controller")

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the background listener (no-op if stdin is not a tty)."""
        if not sys.stdin.isatty():
            print("  [kbd] stdin is not a tty — keyboard control disabled")
            return
        try:
            self._old_settings = termios.tcgetattr(sys.stdin.fileno())
        except termios.error:
            print("  [kbd] termios not available — keyboard control disabled")
            return
        self._thread.start()
        self._print_help()

    def stop(self) -> None:
        """Restore terminal settings (call on node shutdown)."""
        if self._old_settings is not None:
            try:
                termios.tcsetattr(
                    sys.stdin.fileno(), termios.TCSADRAIN, self._old_settings)
            except termios.error:
                pass

    # ── Delta consumers ───────────────────────────────────────────────────────

    def pop_xyz(self) -> Optional[Tuple[float, float, float]]:
        """
        Consume and return the accumulated XYZ position delta (metres,
        scale already applied).  Returns ``None`` if no keys were pressed.
        """
        with self._lock:
            d = self._xyz[:]
            self._xyz = [0.0, 0.0, 0.0]
        return (d[0], d[1], d[2]) if any(v != 0.0 for v in d) else None

    def pop_rpy(self) -> Optional[Tuple[float, float, float]]:
        """
        Consume and return the accumulated RPY orientation delta (radians,
        scale already applied).  Returns ``None`` if no keys were pressed.
        """
        with self._lock:
            d = self._rpy[:]
            self._rpy = [0.0, 0.0, 0.0]
        return (d[0], d[1], d[2]) if any(v != 0.0 for v in d) else None

    # ── Internal ──────────────────────────────────────────────────────────────

    def _set_raw(self) -> None:
        import tty
        tty.setraw(sys.stdin.fileno())

    def _print_help(self) -> None:
        print(
            f"\n  ┌─ Keyboard Controls ─────────────────────────────────────────\n"
            f"  │  t       toggle mode: TRACKER ↔ KEYBOARD\n"
            f"  │  ── KEYBOARD position (XYZ) ─────────────────────────────────\n"
            f"  │  w / s   +Y / −Y   (forward / back)\n"
            f"  │  a / d   −X / +X   (left  / right)\n"
            f"  │  q / e   +Z / −Z   (up    / down)    step={self._step_m*100:.1f} cm × scale\n"
            f"  │  ── KEYBOARD orientation (RPY) ──────────────────────────────\n"
            f"  │  i / k   pitch +/−  (rotate Y)\n"
            f"  │  j / l   yaw   +/−  (rotate Z)\n"
            f"  │  u / o   roll  +/−  (rotate X)   step={math.degrees(self._step_rad):.1f}° × scale\n"
            f"  │  ── General ─────────────────────────────────────────────────\n"
            f"  │  1-9     scale preset (0.10× … 5.00×)\n"
            f"  │  + / -   scale ×1.25 / ×0.80     0 = reset to 1.0\n"
            f"  │  p       pause / resume\n"
            f"  │  r       reset session reference\n"
            f"  │  v       toggle verbose\n"
            f"  │  h       send arm to home\n"
            f"  │  ?       show this help\n"
            f"  │  Ctrl-C  quit\n"
            f"  └─────────────────────────────────────────────────────────────\n"
        )

    def _run(self) -> None:
        fd = sys.stdin.fileno()
        self._set_raw()
        try:
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not rlist:
                    continue
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                _print_help = False
                with self._lock:
                    _sm = self._step_m   * self.scale   # scaled position step
                    _sr = self._step_rad * self.scale   # scaled rotation step

                    # ── Scale controls ─────────────────────────────────────
                    if ch in SCALE_PRESETS:
                        self.scale = SCALE_PRESETS[ch]
                        print(f"\n  [kbd] scale → {self.scale:.2f}×", flush=True)
                    elif ch in ('+', '='):
                        self.scale = min(self.scale * 1.25, 10.0)
                        print(f"\n  [kbd] scale ↑ {self.scale:.2f}×", flush=True)
                    elif ch == '-':
                        self.scale = max(self.scale * 0.80, 0.01)
                        print(f"\n  [kbd] scale ↓ {self.scale:.2f}×", flush=True)
                    elif ch == '0':
                        self.scale = 1.0
                        print(f"\n  [kbd] scale → 1.00× (reset)", flush=True)

                    # ── General flags ──────────────────────────────────────
                    elif ch in ('p', 'P'):
                        self.paused = not self.paused
                        print(f"\n  [kbd] {'PAUSED' if self.paused else 'RUNNING'}", flush=True)
                    elif ch in ('r', 'R'):
                        self.reset_ref = True
                        print(f"\n  [kbd] session ref will reset on next step", flush=True)
                    elif ch in ('v', 'V'):
                        self.verbose = not self.verbose
                        print(f"\n  [kbd] verbose → {self.verbose}", flush=True)
                    elif ch in ('h', 'H'):
                        self.request_home = True
                        print(f"\n  [kbd] home requested", flush=True)
                    elif ch == '?':
                        _print_help = True   # outside lock
                    elif ch == '\x03':       # Ctrl-C → always quit
                        self.request_quit = True
                        print(f"\n  [kbd] quit", flush=True)
                        break

                    # ── Mode toggle ────────────────────────────────────────
                    elif ch in ('t', 'T'):
                        self.mode = "keyboard" if self.mode == "tracker" else "tracker"
                        print(f"\n  [kbd] mode → {self.mode.upper()}", flush=True)

                    # ── Movement keys (keyboard mode only) ────────────────
                    elif self.mode == "keyboard":
                        # XYZ position
                        if   ch in ('w', 'W'): self._xyz[1] += _sm
                        elif ch in ('s', 'S'): self._xyz[1] -= _sm
                        elif ch in ('a', 'A'): self._xyz[0] -= _sm
                        elif ch in ('d', 'D'): self._xyz[0] += _sm
                        elif ch in ('q', 'Q'): self._xyz[2] += _sm
                        elif ch in ('e', 'E'): self._xyz[2] -= _sm
                        # RPY orientation
                        elif ch in ('i', 'I'): self._rpy[1] += _sr   # pitch +
                        elif ch in ('k', 'K'): self._rpy[1] -= _sr   # pitch −
                        elif ch in ('j', 'J'): self._rpy[2] += _sr   # yaw   +
                        elif ch in ('l', 'L'): self._rpy[2] -= _sr   # yaw   −
                        elif ch in ('u', 'U'): self._rpy[0] += _sr   # roll  +
                        elif ch in ('o', 'O'): self._rpy[0] -= _sr   # roll  −

                    # ── Tracker mode: q = quit ─────────────────────────────
                    elif ch in ('q', 'Q'):
                        self.request_quit = True
                        print(f"\n  [kbd] quit", flush=True)
                        break

                if _print_help:
                    self._print_help()

        except Exception:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, self._old_settings)
