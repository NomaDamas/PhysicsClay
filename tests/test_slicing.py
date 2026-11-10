"""pps/slicing.py: planning jobs that pause at points and hand the server loop its frames (no MuJoCo needed)."""

import time

import pytest

from pps import slicing


class Box:
    pass


def _planner(state, n=6, log=None):
    """A stand-in planner: changes the shared state, pauses at points, returns a value."""
    state.task = "planning-own"
    for i in range(n):
        state.hand = i
        slicing.point()
        if log is not None:
            log.append(state.hand)
    return "plan"


def _box():
    s = Box()
    s.task, s.hand = "main", -1
    return s, slicing.Swap().attrs(s, ("task", "hand"))


def test_point_outside_a_job_is_a_noop():
    slicing.point()
    assert not slicing.in_job()


def test_job_runs_across_steps_and_swaps_state():
    s, sw = _box()
    log = []
    job = slicing.Job(lambda: _planner(s, log=log), sw, keep=False)
    steps = 0
    while not job.step(0.0):  # budget 0: it pauses at every point
        steps += 1
        assert (s.task, s.hand) == ("main", -1)  # between slices the loop sees its own state
    assert job.result == "plan" and job.error is None and steps >= 5
    assert log == list(range(6))  # after a pause the planner sees its own state again
    assert (s.task, s.hand) == ("main", -1)  # keep=False (a dry run) leaves nothing behind
    assert job.slices == steps + 1


def test_keep_leaves_the_planner_state():
    s, sw = _box()
    job = slicing.Job(lambda: _planner(s), sw, keep=True)
    while not job.step(0.0):
        pass
    assert (s.task, s.hand) == ("planning-own", 5)


def test_the_loop_keeps_its_own_changes_between_slices():
    s, sw = _box()
    job = slicing.Job(lambda: _planner(s), sw, keep=False)
    job.step(0.0)
    s.task = "the loop changed it"  # (e.g. a stop): survives the next slice and the end
    while not job.step(0.0):
        assert s.task == "the loop changed it"
    assert s.task == "the loop changed it"


def test_budget_runs_to_the_end_in_one_step():
    s, sw = _box()
    job = slicing.Job(lambda: _planner(s), sw, keep=True)
    assert job.step(5.0) and job.slices == 1 and job.result == "plan"


def test_absent_attributes_stay_absent():
    s = Box()

    def fn():
        s.patched = lambda: 1  # (a dry run's stand-in method)
        slicing.point()
        assert s.patched() == 1
        return True

    job = slicing.Job(fn, slicing.Swap().attrs(s, ("patched",)))
    assert not job.step(0.0)
    assert not hasattr(s, "patched")  # the loop never sees the stand-in
    assert job.step(0.0) and job.result is True
    assert not hasattr(s, "patched")


def test_cancel_unwinds_in_the_planner_state():
    s = Box()
    s.v = "main"
    seen = {}

    def fn():
        s.v = "mine"
        try:
            while True:
                slicing.point()
        finally:
            seen["v"] = s.v  # the planner's finally runs with its own state in place
            s.v = "unwound"

    job = slicing.Job(fn, slicing.Swap().attrs(s, ("v",)))
    assert not job.step(0.0)
    assert s.v == "main"
    job.cancel()
    job._thread.join(2.0)
    assert job.done and job.cancelled and job.error is None
    assert seen["v"] == "mine" and s.v == "main" and not job._thread.is_alive()


def test_errors_are_handed_to_the_owner():
    def fn():
        slicing.point()
        raise ValueError("boom")

    job = slicing.Job(fn, None)
    while not job.step(0.0):
        pass
    assert isinstance(job.error, ValueError)


def test_frame_allowance_is_shared():
    def slow():
        for _ in range(40):
            time.sleep(0.005)
            slicing.point()
        return 1

    slicing.new_frame(0.05)
    try:
        a, b = slicing.Job(slow, None), slicing.Job(slow, None)
        assert not a.step()  # uses (about) the frame's 50 ms
        assert slicing.allowance() < 0.01
        b.step()  # what is left: at most one point's worth
        assert b.busy < 0.03
        t0 = time.time()
        assert a.step() is False and time.time() - t0 < 0.01  # nothing left: it waits for the next frame
        slicing.new_frame(10.0)
        assert a.step() and a.result == 1
    finally:
        slicing._frame.update(budget=None, used=0.0)


@pytest.mark.parametrize("n", [0, 1])
def test_jobs_without_points_finish_in_one_step(n):
    job = slicing.Job(lambda: n, None)
    assert job.step(0.0) and job.result == n


def test_drain_closes_a_dropped_step():
    """tasks._drain: a step dropped while paused between its yields is closed in the planner (its finally runs then)."""
    from pps.tasks import _drain

    s = Box()
    s.v, out = "main", []

    def gen():
        try:
            for _ in range(10):
                s.v = "mine"
                yield
        finally:
            out.append(s.v)

    job = slicing.Job(lambda: _drain(gen()), slicing.Swap().attrs(s, ("v",)))
    assert not job.step(0.0)
    job.cancel()
    assert out == ["mine"] and s.v == "main"
