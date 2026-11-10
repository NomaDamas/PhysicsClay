"""Planning that never stalls the server loop (docs/V1.md P0b "Refusals don't stall the loop").

A planning call (a pick search, a plan card's dry run, a goal's set-aside spots) can take seconds: one IK trace of a
path is 0.1-0.6 s on a busy machine and a refusal tries a dozen. A `Job` runs such a call across server ticks:

* the call runs on a helper thread that only ever runs while the server's thread waits for it (they hand control to each
  other: a stackful coroutine, so the planner's own code stays plain, synchronous Python);
* the planner calls `point()` at safe places (before each IK waypoint, between ways in); when the job's slice of wall
  time is used up, control goes back to the server, which steps the plant, renders and streams a frame, and resumes the
  job on a later tick;
* between slices the state the server loop acts on (the visible task, the followed path, the hand targets, the IK
  obstacles, the gripper commands: a `Swap`) is the server's own; the planner's copy is put back when it resumes, so a
  half-made plan never moves the robot. When the job ends its state is kept (a task that starts) or dropped (a dry run);
* `cancel()` unwinds the planner's stack (`Cancelled` at its next point, its `finally` blocks run in its own state).

Outside a job `point()` does nothing: scripts, the benchmark and headless runs plan in one go, exactly as before.
The server sets a per-frame allowance (`new_frame`) that every job's slices share.
"""
import threading
import time
import traceback

_tls = threading.local()
_MISSING = object()
_frame = {"budget": None, "used": 0.0, "stretch": 0.0}
DEFAULT_BUDGET = 0.25
STRETCH_DECAY = 0.95      # per slice: how fast the estimate of the longest stretch between pause points comes back down


class Cancelled(BaseException):
    """Raised in a dropped job at its next pause point (a BaseException, so `except Exception` in planner code can't eat it)."""


def point():
    """A safe place to pause a planning job. A no-op outside one."""
    job = getattr(_tls, "job", None)
    if job is not None:
        job._point()


def in_job():
    return getattr(_tls, "job", None) is not None


def new_frame(budget):
    """Server loop, once per frame: how much wall time planning may use in this frame (shared by every job)."""
    _frame["budget"], _frame["used"] = float(budget), 0.0


def allowance():
    """What is left of this frame's planning time (DEFAULT_BUDGET per slice when no server loop sets frames)."""
    if _frame["budget"] is None:
        return DEFAULT_BUDGET
    return max(0.0, _frame["budget"] - _frame["used"])


def used():
    return _frame["used"]


def stretch():
    """How far past its budget a slice may run (s): the longest recent stretch between two pause points, or from the
    last one back to the server, decaying. A few ms of IK on an idle machine; hundreds of ms when the machine is so busy
    that the planner's thread waits for a CPU (the server takes it off the next frames' planning time)."""
    return _frame["stretch"]


class Swap:
    """State the planner and the server loop each keep their own copy of while a job is paused: object attributes (an
    instance attribute that is absent stays absent: a patched method), and (capture, apply) pairs for the rest."""

    def __init__(self):
        self.items = []

    def attrs(self, obj, names):
        for n in names:
            self.items.append((lambda o=obj, n=n: o.__dict__.get(n, _MISSING), lambda v, o=obj, n=n: _put(o, n, v)))
        return self

    def add(self, capture, apply):
        self.items.append((capture, apply))
        return self

    def capture(self):
        return [c() for c, _ in self.items]

    def apply(self, vals):
        for (_, a), v in zip(self.items, vals):
            a(v)


def _put(o, n, v):
    if v is _MISSING:
        o.__dict__.pop(n, None)
    else:
        setattr(o, n, v)


class Job:
    """Run fn() across server ticks. keep: when it ends, its state stands (a task that starts moving) instead of the
    server's (a dry run). After step() returns True: .result, or .error (an exception it raised) / .cancelled."""

    def __init__(self, fn, swap=None, keep=False, name="plan"):
        self.fn, self.swap, self.keep, self.name = fn, swap, keep, name
        self.done = self.cancelled = False
        self.result = self.error = None
        self.slices, self.slice_max, self.busy = 0, 0.0, 0.0
        self._go, self._back = threading.Semaphore(0), threading.Semaphore(0)
        self._thread = None
        self._mine = None                         # the planner's copy of the swapped state while it is paused
        self._budget, self._t0 = DEFAULT_BUDGET, 0.0

    def step(self, budget=None):
        """Server thread: run until the job pauses (about `budget` s of wall time, default: this frame's allowance) or
        ends. -> done?"""
        if self.done:
            return True
        self._budget = allowance() if budget is None else float(budget)
        if budget is None and self._thread is not None and self._budget < 0.005:   # this frame's planning time is used up:
            return False                                                           # on the next one
        self._t0 = self._pt = time.time()
        self._gap = 0.0
        theirs = self.swap.capture() if self.swap is not None else None
        if self._mine is not None:
            self.swap.apply(self._mine)
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True, name=f"pc-{self.name}")
            self._thread.start()
        else:
            self._go.release()
        self._back.acquire()                      # (the planner runs; this thread waits)
        dt = time.time() - self._t0
        _frame["used"] += dt
        _frame["stretch"] = max(self._gap, dt - self._budget, _frame["stretch"] * STRETCH_DECAY)
        self.slices += 1
        self.slice_max = max(self.slice_max, dt)
        self.busy += dt
        if self.swap is not None:
            if not self.done:
                self._mine = self.swap.capture()
                self.swap.apply(theirs)
            elif not (self.keep and self.error is None):
                self.swap.apply(theirs)
        return self.done

    def cancel(self):
        """Server thread: drop it. Its stack unwinds with its own state in place, then the server's state is put back."""
        if self.done:
            return
        self.cancelled = True
        if self._thread is None:
            self.done = True
            return
        theirs = self.swap.capture() if self.swap is not None else None
        if self._mine is not None:
            self.swap.apply(self._mine)
        self._go.release()
        self._back.acquire()
        if self.swap is not None:
            self.swap.apply(theirs)
        self.done = True

    def _point(self):                             # (planner thread)
        if self.cancelled:
            raise Cancelled()
        now = time.time()
        self._gap, self._pt = max(self._gap, now - self._pt), now
        if now - self._t0 < self._budget:
            return
        self._back.release()
        self._go.acquire()
        if self.cancelled:
            raise Cancelled()

    def _run(self):
        _tls.job = self
        try:
            self.result = self.fn()
        except Cancelled:
            pass
        except BaseException as e:               # handed to the owner (it reports it as the task's failure)
            self.error = e
            traceback.print_exc()
        finally:
            _tls.job = None
            self.done = True
            self._back.release()
