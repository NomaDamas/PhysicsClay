"""P2b teleop (pps/features/teleop.py) without a server: clamping, rate limiting, P5 safety hooks, the gripper.
The runtime and task layer are small stand-ins with the attributes teleop touches."""

import time
from types import SimpleNamespace

import numpy as np
import pytest

from pps import ext
from pps import tasks as tasks_mod
from pps.features import teleop as TP


class FakeRT:
    def __init__(self):
        self.hands = ["right_palm"]
        self.hands_mid = {"right_palm": 0}
        self.palm_sid = {"right_palm": 0}
        self.data = SimpleNamespace(
            mocap_pos=np.array([[0.40, -0.20, 0.95]]),
            mocap_quat=np.array([[1.0, 0.0, 0.0, 0.0]]),
            site_xpos=np.array([[0.40, -0.20, 0.95]]),
        )
        self.retargeter = SimpleNamespace(robot_shoulders={"right_palm": np.array([0.0, -0.15, 1.10])})
        self.mode = "MIRROR"
        self.engaged = False
        self.capabilities = ("pick_place", "reach", "follow_path")


class FakeTasks:
    def __init__(self, rt):
        self.rt = rt
        self.holding = {"right_palm": None}
        self.objs = {"mug": {"r": 0.065, "half_h": 0.07}}
        self.centre = {"mug": np.array([0.40, -0.20, 0.81])}
        self.task = None
        self.cur = None
        self.events = []

    def mug_center(self, n):
        return self.centre[n]

    def limb(self, side, what="hand"):
        return "gripper"

    def _event(self, name, palm):
        self.events.append(name)
        if name == "grasp":
            self.holding[palm] = self.cur
        else:
            self.holding[palm] = None


def make_ctx():
    rt = FakeRT()
    tk = FakeTasks(rt)
    ctx = ext.make_ctx(rt=rt, tasks=tk)
    ctx.dispatch = lambda m: None
    return ctx


def age(ctx, dt):
    a = ctx.store[TP.KEY]["applied"]
    a["right_palm"] = a.get("right_palm", time.time()) - dt


def send(ctx, **msg):
    TP.on_teleop(ctx, dict(type="teleop", **msg))
    return ctx.store[TP.KEY]


def test_start_and_state():
    ctx = make_ctx()
    s = send(ctx, op="start", role="right", id="a")
    assert s["active"] and s["reply"]["ok"] and s["role"] == "right"
    st = TP.teleop_state(ctx)["teleop"]
    assert st["active"] and st["hands"]["right"]["p"] == [0.4, -0.2, 0.95]
    s = send(ctx, op="stop", id="b")
    assert not s["active"] and s["reply"]["op"] == "stop"


def test_rate_limited_and_converges():
    ctx = make_ctx()
    send(ctx, op="start")
    p0 = ctx.rt.data.mocap_pos[0].copy()
    target = p0 + np.array([0.0, 0.3, 0.0])
    send(ctx, p=target.tolist())
    moved = float(np.linalg.norm(ctx.rt.data.mocap_pos[0] - p0))
    assert 0 < moved <= TP.MAX_SPEED * 0.1 + 1e-9  # one command can't jump 30 cm
    for _ in range(40):  # a stream of commands gets there
        age(ctx, 1 / 30)
        send(ctx, p=target.tolist())
    assert np.allclose(ctx.rt.data.mocap_pos[0], target, atol=1e-6)


def test_physics_rate_limit_runs_on_the_robots_clock():
    """With physics the target's speed is per second of the robot's (sim) time: a busy machine that runs the plant at a
    third of real time gets a third of the motion per wall second, so the arm can follow it."""
    ctx = make_ctx()
    ctx.rt.plant = SimpleNamespace(d=SimpleNamespace(time=10.0))
    d = ctx.rt.data

    def follow():  # an arm that keeps up: the hand is where the target is
        d.site_xpos[0] = d.mocap_pos[0]

    send(ctx, op="start")
    p0 = d.mocap_pos[0].copy()
    target = (p0 + np.array([0.0, 0.3, 0.0])).tolist()
    send(ctx, p=target)  # the first command: one frame's worth (1/30 s)
    m1 = float(np.linalg.norm(d.mocap_pos[0] - p0))
    assert 0 < m1 <= TP.MAX_SPEED / 30 + 1e-9
    follow()
    time.sleep(0.05)  # wall time passes, but no robot time: the target stays
    send(ctx, p=target)
    assert np.isclose(float(np.linalg.norm(d.mocap_pos[0] - p0)), m1)
    ctx.rt.plant.d.time += 0.04  # 40 ms of robot time: 4 cm more at 1 m/s
    send(ctx, p=target)
    assert np.isclose(float(np.linalg.norm(d.mocap_pos[0] - p0)), m1 + TP.MAX_SPEED * 0.04)
    ctx.rt.plant.d.time = 0.0  # a scene reset restarts the clock: no jump
    p1 = ctx.rt.data.mocap_pos[0].copy()
    send(ctx, p=target)
    assert np.allclose(ctx.rt.data.mocap_pos[0], p1)


def test_physics_target_waits_while_the_arm_catches_up():
    """With physics a trailing arm (still moving toward the target, more than LEAD behind) holds the target where it is,
    so the hand goes the way it was pointed instead of cutting the corner; a hand that has stopped short (an IK limit)
    doesn't hold it back, and back toward the hand is always allowed."""
    ctx = make_ctx()
    ctx.rt.plant = SimpleNamespace(d=SimpleNamespace(time=0.0))
    d = ctx.rt.data
    send(ctx, op="start")
    ee0 = d.site_xpos[0].copy()
    want = ee0 + np.array([0.0, 0.4, 0.0])

    def tick(hand_dy=0.0):
        ctx.rt.plant.d.time += 1 / 30
        d.site_xpos[0] = d.site_xpos[0] + [0.0, hand_dy, 0.0]
        send(ctx, p=want.tolist())
        return d.mocap_pos[0].copy()

    for _ in range(3):  # the hand hasn't moved yet: the target goes on
        tick()
    t1 = d.mocap_pos[0].copy()
    assert t1[1] - ee0[1] > TP.LEAD
    t2 = tick(0.01)  # the hand moves toward it (0.3 m/s): the target waits
    assert np.allclose(t2, t1) and ctx.store[TP.KEY]["limited"]
    for _ in range(3):
        tick(0.01)
    assert np.allclose(d.mocap_pos[0], t1)
    t3 = tick(0.0)  # stopped short (lagging but not moving): it goes on
    assert t3[1] > t1[1]
    t4 = tick(0.02)
    assert np.allclose(t4, t3)  # moving again and still behind: waits again
    ctx.rt.plant.d.time += 1 / 30  # back toward the hand is always allowed
    send(ctx, p=(t4 - [0.0, 0.03, 0.0]).tolist())
    assert np.allclose(d.mocap_pos[0], t4 - [0.0, 0.03, 0.0])


def test_clamped_to_reach_and_surface():
    ctx = make_ctx()
    send(ctx, op="start")
    for _ in range(80):
        age(ctx, 0.1)
        send(ctx, p=[3.0, -0.2, 0.95])
    c, R = TP.reach_of(ctx.rt, "right_palm")
    assert float(np.linalg.norm(ctx.rt.data.mocap_pos[0] - c)) <= R + 1e-6
    assert ctx.store[TP.KEY]["clamped"]
    for _ in range(40):
        age(ctx, 0.1)
        send(ctx, p=[0.5, -0.2, 0.30])  # below the table top
    assert ctx.rt.data.mocap_pos[0][2] >= tasks_mod.TABLE_TOP + TP.SURFACE_MARGIN - 1e-9


def test_safety_stop_holds_and_scale_slows():
    ctx = make_ctx()
    send(ctx, op="start")
    p0 = ctx.rt.data.mocap_pos[0].copy()
    ctx.rt.safety_scale = 0.0  # P5 protective stop
    send(ctx, p=(p0 + [0, 0.1, 0]).tolist())
    assert np.allclose(ctx.rt.data.mocap_pos[0], p0) and "safety" in ctx.store[TP.KEY]["held"]
    ctx.rt.safety_scale = 0.5  # P5 slowing: half the speed
    ctx.store[TP.KEY]["applied"]["right_palm"] = time.time() - 0.1
    send(ctx, p=(p0 + [0, 0.3, 0]).tolist())
    moved = float(np.linalg.norm(ctx.rt.data.mocap_pos[0] - p0))
    assert moved <= TP.MAX_SPEED * 0.1 * 0.5 + 1e-3 and moved > 0
    assert ctx.store[TP.KEY]["held"] is None


def test_safety_hook_refuses_target_inside_person():
    ctx = make_ctx()
    ctx.tasks.safety = SimpleNamespace(
        stopped=False, check_point=lambda p, role: {"ok": p[1] < 0.0, "why": "your torso"}
    )
    send(ctx, op="start")
    p0 = ctx.rt.data.mocap_pos[0].copy()
    send(ctx, p=[0.4, 0.2, 0.95])
    assert np.allclose(ctx.rt.data.mocap_pos[0], p0) and "into you" in ctx.store[TP.KEY]["held"]
    ctx.tasks.safety.stopped = True
    send(ctx, p=[0.4, -0.25, 0.95])
    assert np.allclose(ctx.rt.data.mocap_pos[0], p0)


def test_busy_runtime_is_left_alone():
    ctx = make_ctx()
    ctx.rt.mode = "FOLLOW"
    p0 = ctx.rt.data.mocap_pos[0].copy()
    send(ctx, p=[0.4, 0.0, 0.95])  # a bare command while a path runs
    assert np.allclose(ctx.rt.data.mocap_pos[0], p0) and ctx.store[TP.KEY]["held"].startswith("busy")


def test_kinematic_grip_grasps_and_releases():
    ctx = make_ctx()
    send(ctx, op="start")
    ctx.rt.data.site_xpos[0] = ctx.rt.data.mocap_pos[0] = [0.40, -0.20, 0.86]  # the gripper at the mug
    p = ctx.rt.data.mocap_pos[0].tolist()
    send(ctx, p=p, grip=1.0)
    assert ctx.tasks.holding["right_palm"] == "mug" and ctx.tasks.events == ["grasp"]
    send(ctx, p=p, grip=1.0)  # held closed: no second grasp
    assert ctx.tasks.events == ["grasp"]
    send(ctx, p=p, grip=0.0)
    assert ctx.tasks.holding["right_palm"] is None and ctx.tasks.events == ["grasp", "release"]


def test_physical_gripper_when_runtime_has_one():
    ctx = make_ctx()
    got = []
    ctx.rt.set_gripper = lambda palm, v: got.append((palm, v))
    send(ctx, op="start")
    send(ctx, p=ctx.rt.data.mocap_pos[0].tolist(), grip=0.7)
    assert got == [("right_palm", 0.7)] and ctx.tasks.events == []


def test_orientation_rate_limited():
    ctx = make_ctx()
    send(ctx, op="start")
    q = [0.0, 0.0, 0.0, 1.0]  # 180 degrees about z
    send(ctx, p=ctx.rt.data.mocap_pos[0].tolist(), q=q)
    from pps import traj as T

    ang = T.qangle([1, 0, 0, 0], ctx.rt.data.mocap_quat[0])
    assert 0 < ang <= TP.MAX_TURN * 0.1 + 1e-6


def test_idle_timeout_ends_teleop():
    ctx = make_ctx()
    send(ctx, op="start")
    ctx.store[TP.KEY]["last"] = time.time() - TP.IDLE_S - 1
    TP.teleop_tick(ctx)
    assert not ctx.store[TP.KEY]["active"]


@pytest.mark.parametrize("bad", [None, [1, 2], ["a", 0, 0], [float("nan"), 0, 0]])
def test_bad_commands_are_rejected(bad):
    ctx = make_ctx()
    s = send(ctx, p=bad, id="x")
    assert s["reply"]["ok"] is False
