"""Unit tests for pps/safety.py (v1 P5): capsule distances, the SSM formula, the speed-scale curve with hysteresis, and
the contact policy table. Pure: no server, no MuJoCo except the last test (robot capsules for every profile).

    .venv/bin/python -m pytest tests/test_safety.py -q
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from pps import safety as S  # noqa: E402

CFG = S.Config()


def robot(*caps, kinds=None, ee=None):
    """[(a, b, r)] -> the dict RobotGeometry.capsules() returns."""
    a = np.array([c[0] for c in caps], float)
    b = np.array([c[1] for c in caps], float)
    r = np.array([c[2] for c in caps], float)
    n = len(caps)
    kinds = kinds or ["ee"] * n
    return {
        "a": a,
        "b": b,
        "r": r,
        "names": [f"link{i}" for i in range(n)],
        "kind": kinds,
        "ee": ee or [("right_palm" if k == "ee" else None) for k in kinds],
    }


def part(region, a, b, r=0.05, name=None):
    return {"part": name or region, "region": region, "a": np.asarray(a, float), "b": np.asarray(b, float), "r": r}


# ---------------------------------------------------------------------- distance math
def test_seg_seg_parallel_and_crossing():
    d, p, q = S.seg_seg([0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0])
    assert math.isclose(float(d), 1.0, abs_tol=1e-9)
    d, p, q = S.seg_seg([0, 0, 0], [1, 0, 0], [0.5, -1, 1], [0.5, 1, 1])  # skew: closest at (0.5, 0, 0)-(0.5, 0, 1)
    assert math.isclose(float(d), 1.0, abs_tol=1e-9) and np.allclose(p, [0.5, 0, 0]) and np.allclose(q, [0.5, 0, 1])


def test_seg_seg_endpoints_and_points():
    d, _, _ = S.seg_seg([0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0])  # collinear, apart
    assert math.isclose(float(d), 1.0, abs_tol=1e-9)
    d, _, q = S.seg_seg([0, 0, 0], [0, 0, 0], [-1, 1, 0], [1, 1, 0])  # point vs segment
    assert math.isclose(float(d), 1.0, abs_tol=1e-9) and np.allclose(q, [0, 1, 0])
    d, _, _ = S.seg_seg([1, 2, 3], [1, 2, 3], [1, 2, 5], [1, 2, 5])  # point vs point
    assert math.isclose(float(d), 2.0, abs_tol=1e-9)


def test_seg_seg_vectorised_matches_brute_force():
    rng = np.random.default_rng(3)
    P0, P1, Q0, Q1 = (rng.normal(size=(40, 3)) for _ in range(4))
    d, _, _ = S.seg_seg(P0, P1, Q0, Q1)
    ts = np.linspace(0, 1, 201)
    for k in range(40):
        a = P0[k] + ts[:, None] * (P1[k] - P0[k])
        b = Q0[k] + ts[:, None] * (Q1[k] - Q0[k])
        brute = np.min(np.linalg.norm(a[:, None] - b[None], axis=-1))
        assert d[k] <= brute + 1e-9 and d[k] >= brute - 0.02


def test_capsule_surface_distance():
    assert math.isclose(
        S.capsule_distance([0, 0, 0], [0, 0, 1], 0.1, [0.5, 0, 0], [0.5, 0, 1], 0.15), 0.25, abs_tol=1e-9
    )
    assert S.capsule_distance([0, 0, 0], [0, 0, 1], 0.3, [0.5, 0, 0], [0.5, 0, 1], 0.3) < 0  # overlapping


def test_human_capsules_from_pose():
    from pps.features.copresence import to_world
    from pps.human import synth
    from pps.human.model import ground, pose33

    pose, _ = pose33(ground(to_world(synth.frame_at("rest", 0.0))))
    caps = S.human_capsules(pose)
    regions = {c["region"] for c in caps}
    assert len(caps) == 17 and regions == {"head", "torso", "arm", "forearm", "hand", "leg"}
    head = next(c for c in caps if c["part"] == "head")
    assert head["a"][2] > 1.4  # the head is up there
    missing = list(pose)
    missing[15] = None  # left wrist lost
    names = {c["part"] for c in S.human_capsules(missing)}
    assert "left_forearm" not in names and "left_hand" not in names and "right_hand" in names


# ---------------------------------------------------------------------- SSM formula + speed curve
def test_ssm_formula():
    c = S.Config(t_react=0.1, t_stop=0.2, d_stop=0.12)
    # S = v_h (T_r + T_s) + v_r T_r + v_r T_s / 2 + C
    assert math.isclose(S.protective_distance(c, 0.0, 0.0, 0.12), 0.12)
    assert math.isclose(S.protective_distance(c, 0.5, 0.0, 0.12), 0.12 + 0.5 * 0.1 + 0.5 * 0.1)
    assert math.isclose(S.protective_distance(c, 0.0, 1.0, 0.12), 0.12 + 1.0 * 0.3)
    assert math.isclose(S.protective_distance(c, 0.0, 9.0, 0.12), 0.12 + c.v_human_max * 0.3)  # human speed capped
    assert S.protective_distance(c, 1.0, 0.0, 0.12) > S.protective_distance(
        c, 0.2, 0.0, 0.12
    )  # faster robot, more room


def test_speed_curve():
    c = S.Config(d_slow=0.5, d_stop=0.12, min_scale=0.2)
    sp = S.protective_distance(c, 0.0, 0.0, c.d_stop)
    assert S.scale_for(c, 0.8, sp, c.d_stop) == 1.0
    assert S.scale_for(c, 0.5, sp, c.d_stop) == 1.0
    assert S.scale_for(c, 0.11, sp, c.d_stop) == 0.0
    assert math.isclose(S.scale_for(c, 0.12, sp, c.d_stop), 0.2)
    mid = S.scale_for(c, 0.31, sp, c.d_stop)
    assert math.isclose(mid, 0.2 + 0.8 * (0.31 - 0.12) / (0.5 - 0.12))
    xs = np.linspace(0.12, 0.6, 50)
    ys = [S.scale_for(c, x, sp, c.d_stop) for x in xs]
    assert all(b >= a for a, b in zip(ys, ys[1:]))  # monotonic
    # a moving robot slows (and stops) further out: the band shifts by the reaction + braking distance
    sp_fast = S.protective_distance(c, 1.0, 0.0, c.d_stop)
    assert S.scale_for(c, 0.3, sp_fast, c.d_stop) < S.scale_for(c, 0.3, sp, c.d_stop)
    assert S.scale_for(c, 0.2, sp_fast, c.d_stop) == 0.0


def _at(x, region="arm"):
    """Robot end effector capsule at the origin; a person part at distance x (surface) along +x."""
    return robot(([0, 0, 0], [0, 0, 0], 0.05)), [part(region, [x + 0.10, 0, -0.2], [x + 0.10, 0, 0.2], 0.05)]


def test_hysteresis_and_resume_policy():
    c = S.Config(resume_s=1.0, hysteresis=0.05, ramp_up=1.0)
    m = S.Monitor(c)
    t = 0.0
    r, h = _at(0.30)
    st = m.update(r, h, t)
    assert 0.2 < st["speed_scale"] < 1.0 and not st["stop"] and st["zone"] == "slow"
    r, h = _at(0.10)
    st = m.update(r, h, t := t + 0.05, v_robot=0.2)
    assert st["stop"] and st["speed_scale"] == 0.0 and st["zone"] == "stop" and st["reason"] == "body"
    # just outside d_stop but inside the hysteresis band: still stopped (no chatter)
    for _ in range(40):
        r, h = _at(c.d_stop + 0.02)
        st = m.update(r, h, t := t + 0.05)
        assert st["stop"]
    # clear by more than the band, but not for long enough: still stopped
    r, h = _at(0.40)
    st = m.update(r, h, t := t + 0.05)
    st = m.update(r, h, t := t + 0.5)
    assert st["stop"]
    st = m.update(r, h, t := t + 0.6)  # clear for > resume_s: automatic resume
    assert not st["stop"] and st["speed_scale"] <= c.ramp_up * 0.5 + 1e-9  # from 0, at ramp_up per second
    prev = st["speed_scale"]
    st = m.update(r, h, t := t + 0.1)
    assert prev < st["speed_scale"] <= prev + c.ramp_up * 0.1 + 1e-9  # ramps up, never jumps
    for _ in range(20):
        st = m.update(r, h, t := t + 0.1)
    assert st["speed_scale"] > 0.5
    r, h = _at(0.30)  # slowing down again is immediate
    st2 = m.update(r, h, t := t + 0.05)
    assert st2["speed_scale"] < st["speed_scale"]


def test_no_person_is_clear():
    m = S.Monitor()
    st = m.update(robot(([0, 0, 0], [0, 0, 0], 0.05)), [], 0.0)
    assert st["d_min"] is None and st["speed_scale"] == 1.0 and not st["stop"] and st["zone"] is None


# ---------------------------------------------------------------------- contact policy table
@pytest.mark.parametrize(
    "region,kind,coop,force,expect",
    [
        # region    robot  coop   force  -> (C, cap, contact allowed, why)
        ("head", "ee", False, None, (CFG.d_stop_critical, 1.0, False, "critical")),
        ("head", "ee", True, 5.0, (CFG.d_stop_critical, 1.0, False, "critical")),  # assist never allows the head
        ("torso", "ee", True, 5.0, (CFG.d_stop_critical, 1.0, False, "critical")),
        ("arm", "ee", True, 5.0, (CFG.d_stop, 1.0, False, "body")),  # upper arm: no contact
        ("leg", "ee", True, None, (CFG.d_stop, 1.0, False, "body")),
        ("hand", "ee", False, 5.0, (CFG.d_stop, 1.0, False, "body")),  # not cooperating: no contact
        ("hand", "arm", True, 5.0, (CFG.d_stop, 1.0, False, "body")),  # only the end effector
        ("hand", "ee", True, None, (CFG.d_touch, CFG.coop_scale, False, "no_force")),  # can't feel force: no contact
        ("forearm", "ee", True, None, (CFG.d_touch, CFG.coop_scale, False, "no_force")),
        ("hand", "ee", True, 5.0, (-1.0, CFG.coop_scale, True, "coop")),  # measured and under the cap
        ("hand", "ee", True, 99.0, (-1.0, CFG.coop_scale, False, "coop")),  # over the cap
    ],
)
def test_contact_policy_table(region, kind, coop, force, expect):
    assert S.pair_rule(CFG, region, kind, coop, force) == expect


def test_head_and_torso_stop_in_every_mode():
    for coop, force in ((False, None), (True, None), (True, 5.0)):
        m = S.Monitor()
        r, h = _at(0.13, "head")  # outside d_stop, inside d_stop_critical
        st = m.update(r, h, 0.0, coop=coop, force=force)
        assert st["stop"] and st["reason"] == "critical"
        assert S.describe(st) == "Stopped: too close to your head"


def test_assist_allows_hand_contact_only_with_force():
    r, h = _at(0.03, "hand")
    st = S.Monitor().update(r, h, 0.0)  # not cooperating: stop
    assert st["stop"] and st["reason"] == "body"
    st = S.Monitor().update(r, h, 0.0, coop=True)  # cooperating, near: slow, not stopped
    assert not st["stop"] and 0 < st["speed_scale"] <= CFG.coop_scale
    r, h = _at(-0.01, "hand")  # touching
    st = S.Monitor().update(r, h, 0.0, coop=True, force=None)
    assert st["stop"] and st["reason"] == "no_force"
    st = S.Monitor().update(r, h, 0.0, coop=True, force=12.0)
    assert not st["stop"] and st["speed_scale"] <= CFG.coop_scale
    m = S.Monitor()
    st = m.update(r, h, 0.0, coop=True, force=80.0)  # over the force cap: latched stop
    assert st["stop"] and st["latched"] and st["reason"] == "force"
    r2, h2 = _at(1.0, "hand")
    for k in range(40):
        st = m.update(r2, h2, 0.1 * (k + 1))
    assert st["stop"]  # clear, but latched until confirmed
    assert m.confirm()
    for k in range(15):
        st = m.update(r2, h2, 4.1 + 0.1 * k)
    assert not st["stop"]
    r, h = _at(0.03, "torso")  # the torso during assist: stop
    st = S.Monitor().update(r, h, 0.0, coop=True, force=5.0)
    assert st["stop"] and st["reason"] == "critical"


def test_check_point_body_zone():
    m = S.Monitor()
    m.human = [part("torso", [1.0, 0, 0.9], [1.0, 0, 1.4], 0.13), part("hand", [0.7, 0, 1.0], [0.75, 0, 1.0], 0.05)]
    assert not m.check_point([0.9, 0, 1.1])["ok"]  # in the chest
    r = m.check_point([0.9, 0, 1.1])
    assert r["region"] == "torso" and "body zone" in r["why"]
    assert m.check_point([0.4, 0, 0.8])["ok"]
    assert not m.check_point([0.60, 0, 1.0])["ok"]  # in front of the hand, not cooperating
    assert m.check_point([0.60, 0, 1.0], coop=True)["ok"]  # handing over: up to the hand is fine
    assert not m.check_point([0.66, 0, 1.0], coop=True)["ok"]  # ...but not into it (no force reading)
    assert S.Monitor().check_point([0, 0, 0])["ok"]  # nobody there


def test_config_cannot_switch_safety_off():
    c = S.Config.from_dict(
        {"enabled": False, "d_stop": 0.0, "min_scale": 0.0, "d_slow": 0.0, "f_contact_max": 1e6, "d_stop_critical": -1}
    )
    assert c.d_stop >= 0.05 and c.min_scale > 0 and c.d_slow > c.d_stop_critical >= c.d_stop and c.f_contact_max <= 140
    assert not hasattr(c, "enabled")


def test_describe_lines():
    assert S.describe({"zone": "slow", "d_min": 0.304, "closest": {"region": "torso"}}) == "Slowing: you're 30 cm away"
    assert (
        S.describe(
            {
                "zone": "stop",
                "stop": True,
                "reason": "critical",
                "d_min": 0.1,
                "binding": {"region": "head", "human_part": "neck"},
            }
        )
        == "Stopped: too close to your head"
    )
    assert S.describe({"zone": "clear", "d_min": 0.9}) is None


def test_robot_capsules_every_profile():
    """Every robot profile yields capsules with end-effector links, and the distance query is cheap."""
    import time

    from pps.features.copresence import to_world
    from pps.human import synth
    from pps.human.model import ground, pose33
    from pps.world import profiles, scene
    from pps.world.runtime import make_runtime

    pose, _ = pose33(ground(to_world(synth.frame_at("rest", 0.0))))
    hum = S.human_capsules(pose)
    for pid in profiles.PROFILES:
        w = scene.build(pid, None)
        rt = make_runtime(w)
        geo = S.RobotGeometry(rt.model, w.robot_bodies, rt.palm_sid)
        caps = geo.capsules(rt.data)
        assert len(caps["r"]) > 3 and "ee" in caps["kind"], pid
        t0 = time.perf_counter()
        ev = S.Monitor().evaluate(caps, hum)
        assert time.perf_counter() - t0 < 0.05, pid
        assert ev["d_min"] is not None and ev["d_min"] > 0.1, (pid, ev["closest"])  # the person stands clear
