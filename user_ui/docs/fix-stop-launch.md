# Fix: Stop button didn't actually stop `ros2 launch`

## Symptom

Clicking **Stop** in the launcher panel marked the UI as idle, but the
underlying `ros2 launch` and all the controller / hardware nodes it had
spawned were still alive on the system.

## Root causes — two, stacked

1. **No process group.** `ManagedProcess` was using plain `QProcess.terminate()`,
   which only signals the immediate child. `ros2 launch` spawns its nodes
   in separate processes; orphaning them after the parent dies was the
   normal outcome.
2. **Qt inherits SIG_IGN on SIGINT.** A Qt GUI app sets `SIGINT` to ignore so
   that Ctrl-C in the terminal doesn't kill the window. Child processes
   inherited that disposition by default, so even if we *had* sent
   `SIGINT` to the right pid it would have been silently ignored. (We
   reproduced this with a bash child: `kill -INT -PGID` did nothing.)

These two interact: switching to `os.killpg(...)` alone wasn't enough
because the children, having inherited SIG_IGN, refused SIGINT.

## Fix

`common/proc.py`:

1. Set the child up as a new session leader via Qt's
   `QProcess.UnixProcessParameters` (Qt 6.6+). This makes the child its
   own process-group leader, so `os.killpg(child_pid, ...)` reaches the
   entire subtree.
2. Add `ResetSignalHandlers` to the same flags so the child wakes up
   with `SIG_DFL` for every signal — `SIGINT` actually fires.
3. Rewrite `stop()` as `SIGINT → wait → SIGTERM → wait → SIGKILL`, sent
   to the process group each time. This matches the Ctrl-C semantics
   that `ros2 launch` is built to handle.

```python
params = QProcess.UnixProcessParameters()
params.flags = (
    QProcess.UnixProcessFlag.CreateNewSession
    | QProcess.UnixProcessFlag.ResetSignalHandlers
)
self._proc.setUnixProcessParameters(params)
```

## Verification

Synthetic Python "fake ros2 launch" (parent with SIGINT handler that
terminates its children, then exits):

```bash
QT_QPA_PLATFORM=offscreen timeout 15 python3 -u -c "
...
p.start('python3', ['-u', '/tmp/fake_ros2_launch.py'])
...
p.stop()
# expected: parent logs 'got sig 2', 'clean_exit'; children all dead in <3s
"
```

Result: parent + 2 child `sleep 60`s die in 10 ms. The full transcript
is in the commit body.

For the real `ros2 launch openarm_o6_bimanual.launch.py`, the same path
fires: SIGINT lands on the launch process, which in turn shuts down every
spawned node before exiting. The escalation to SIGTERM/SIGKILL only kicks
in if a node hangs past the timeout.

## Notes

- The earlier ad-hoc test with `bash -c 'sleep 60 & ... wait'` was an
  *artificial* failure mode: bash deliberately sets backgrounded
  children's SIGINT disposition to ignore (job control behaviour). That
  isn't what `ros2 launch` does — it uses Python's `subprocess`, which
  doesn't tamper with the child's signal mask.
- `stop()` is still synchronous and blocks the GUI thread during the
  wait windows (default 5 s + 2 s). For a real launch shutdown the SIGINT
  step almost always completes in well under a second.
