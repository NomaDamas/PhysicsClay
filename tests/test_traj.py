"""Unit tests for pps/traj.py (pure: no server, no MuJoCo). Run: .venv/bin/python -m pytest tests -q"""
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from pps import traj as T  # noqa: E402

DOWN_Q = [0.0, 1.0, 0.0, 0.0]          # a gripper pointing down (180 deg about x)
WORLD = {"objects": {
    "vase": {"pos": [0.42, 0.0, 0.86], "r": 0.04, "h": 0.12, "kind": "vase"},
    "red_cup": {"pos": [0.35, -0.2, 0.78], "r": 0.035, "h": 0.04, "kind": "cup"},
    "bowl": {"pos": [0.35, 0.2, 0.77], "r": 0.07, "h": 0.03, "kind": "bowl"}},
    "surface": 0.74, "hand": [0.25, -0.1, 0.95], "markers": {"A": [0.40, 0.10, 0.74], "P": [0.30, 0.05, 0.74]},
    "regions": {"left": [0.31, 0.22], "right": [0.31, -0.22], "front": [0.27, 0.0]}}


def speeds(d):
    return np.linalg.norm(np.diff(d["p"], axis=0), axis=1) / np.maximum(np.diff(d["t"]), 1e-9)


def line(*pts, **kw):
    return T.normalize(dict({"points": [{"p": p} for p in pts]}, **kw))


# ---------------------------------------------------------------------------- schema
def test_schema_round_trip_and_canonical_keys():
    raw = {"id": "t7", "role": "left_palm", "frame": "object: Red Cup", "source": "language", "label": "x" * 300,
           "points": [[0.1, 0.2, 0.3], {"p": [0.2, 0.2, 0.3], "q": [2, 0, 0, 0], "speed": 99, "grip": 3, "yaw": 270, "tilt": 120,
                                        "hold": 0, "force": -5, "note": ""}],
           "pattern": {"kind": "weave", "amp": 0.004, "freq": 2, "axis": "normal"}, "check": {"ok": [1, 0, 1], "why": "x", "duration": 2},
           "meta": {"from": "test"}}
    tr = T.normalize(raw)
    assert T.normalize(tr) == tr                                         # idempotent
    assert json.loads(json.dumps(tr)) == tr                              # JSON-safe
    assert tr["role"] == "left" and tr["frame"] == "object:red_cup" and len(tr["label"]) == 160
    assert list(tr) == ["id", "role", "frame", "source", "label", "points", "pattern", "check", "meta"]
    p0, p1 = tr["points"]
    assert set(T.POINT_KEYS) <= set(p0) and p0["q"] is None and p0["speed"] is None
    assert p1["q"] == [1.0, 0.0, 0.0, 0.0] and p1["speed"] == 1.5 and p1["grip"] == 1.0 and p1["force"] == 0.0
    assert p1["yaw"] == -90.0 and p1["tilt"] == 80.0 and "hold" not in p1 and p1["note"] is None
    assert tr["pattern"] == {"kind": "zigzag", "amp": 0.004, "freq": 2.0, "axis": "normal"}
    assert tr["check"]["ok"] == [True, False]


@pytest.mark.parametrize("bad", [None, {}, {"points": []}, {"points": [{"q": [1, 0, 0, 0]}]}, {"points": [[0, 0]]},
                                 {"points": [[0, 0, float("nan")]]}, {"points": [[0, 0, 20]]}, {"points": [[0, 0, 0]], "role": "middle"},
                                 {"points": [[0, 0, 0]], "source": "telepathy"}, {"points": [[0, 0, 0]], "frame": "camera"},
                                 {"points": [[0, 0, 0]], "pattern": {"kind": "spiral3"}}])
def test_schema_rejects_with_a_sentence(bad):
    with pytest.raises(T.TrajError) as e:
        T.normalize(bad)
    assert str(e.value) and str(e.value)[0].islower()


# ---------------------------------------------------------------------------- densify: timing
def test_densify_constant_speed_with_min_jerk_ramps():
    tr = line([0.3, -0.2, 0.85], [0.3, 0.2, 0.85])                      # 40 cm
    d = T.densify(tr, q0=DOWN_Q, kind="arm")
    v = speeds(d)
    v0 = T.DEFAULT_SPEED["arm"]
    assert abs(v.max() - v0) < 0.01                                      # cruises at the robot kind's default speed
    assert v[0] < 0.05 * v0 and v[-1] < 0.05 * v0                        # starts and stops at rest
    nominal = 0.4 / v0
    assert abs(d["duration"] - (nominal + T.RAMP)) < 0.05                # ramps cost half a ramp at each end
    assert np.allclose(d["p"][0], [0.3, -0.2, 0.85]) and np.allclose(d["p"][-1], [0.3, 0.2, 0.85])
    assert d["key"] == [0, len(d["t"]) - 1] and np.all(np.diff(d["t"]) > 0)
    acc = np.diff(v) / np.diff(d["t"][1:])
    assert np.abs(acc).max() < 3 * v0 / T.RAMP                          # smooth, no jumps


def test_per_point_speed_slows_that_part():
    fast = line([0.3, -0.3, 0.85], [0.3, 0.0, 0.85], [0.3, 0.3, 0.85])
    slow = T.normalize(dict(fast, points=[dict(p, speed=0.05) if i == 1 else p for i, p in enumerate(fast["points"])]))
    d0, d1 = T.densify(fast, speed=0.2), T.densify(slow, speed=0.2)
    # the speed changes linearly along the path between points: each half takes L / (v1 - v0) * ln(v1 / v0)
    assert abs(d1["duration"] - (2 * 0.3 / 0.15 * math.log(4) + T.RAMP)) < 0.05
    assert abs(d0["duration"] - (0.6 / 0.2 + T.RAMP)) < 0.05
    v = speeds(d1)
    mid = d1["key"][1]
    assert v[mid - 1] < 0.06                                             # passing point 2 at its own speed


def test_grip_change_stops_the_hand_and_holds():
    tr = T.normalize({"points": [{"p": [0.3, -0.1, 0.9]}, {"p": [0.3, 0.0, 0.85], "grip": 1}, {"p": [0.3, 0.1, 0.9], "hold": 1.0},
                                 {"p": [0.3, 0.2, 0.9], "grip": 0}]})
    d = T.densify(tr, speed=0.2)
    t = d["t"]
    k1 = d["key"][1]
    still = [j for j in range(len(t) - 1) if np.linalg.norm(d["p"][j + 1] - d["p"][j]) < 1e-9]
    dwell = sum(t[j + 1] - t[j] for j in still)
    assert dwell >= T.GRIP_DWELL * 2 + 1.0 - 1e-6                       # two gripper changes + a 1 s hold
    ev = [(e[1], e[2], e[3]) for e in d["events"]]
    assert ev == [("grip", 1.0, 1), ("grip", 0.0, 3)]
    assert abs(d["events"][0][0] - t[k1]) < 1e-9                         # fires on arrival, then the hand waits
    assert np.isnan(d["grip"][0]) and d["grip"][k1] == 1.0 and d["grip"][-1] == 0.0


def test_timed_points_keep_their_timing():
    tr = T.normalize({"source": "demo", "points": [{"p": [0.3, 0, 0.9], "t": 1.0}, {"p": [0.35, 0, 0.9], "t": 2.0},
                                                   {"p": [0.35, 0.05, 0.9], "t": 4.0}]})
    d = T.densify(tr)
    assert abs(d["duration"] - 3.0) < 1e-9
    j = int(np.argmin(np.abs(d["t"] - 1.0)))
    assert np.allclose(d["p"][j], [0.35, 0, 0.9], atol=1e-3)


# ---------------------------------------------------------------------------- orientation
def test_orientation_default_yaw_tilt_and_slerp():
    tr = T.normalize({"points": [{"p": [0.3, -0.1, 0.9]}, {"p": [0.3, 0.1, 0.9], "yaw": 90}]})
    d = T.densify(tr, q0=DOWN_Q)
    assert T.qangle(d["q"][0], DOWN_Q) < 1e-6
    assert abs(math.degrees(T.qangle(d["q"][-1], DOWN_Q)) - 90) < 0.5
    tool = T.qrot(d["q"][-1], [0, 0, 1])                                 # the approach axis still points down after a yaw
    assert tool[2] < -0.999
    mid = d["q"][len(d["q"]) // 2]
    assert 20 < math.degrees(T.qangle(mid, DOWN_Q)) < 70                 # slerped in between
    tilted = T.densify(T.normalize({"points": [{"p": [0.3, 0, 0.9]}, {"p": [0.4, 0, 0.9], "tilt": 20}]}), q0=DOWN_Q)
    tip = T.qrot(tilted["q"][-1], [0, 0, 1])                             # leans the tip toward travel (+x)
    assert tip[0] > 0.3 and abs(math.degrees(math.acos(-tip[2])) - 20) < 0.5


# ---------------------------------------------------------------------------- pattern
@pytest.mark.parametrize("kind", ["zigzag", "circle", "figure8"])
def test_pattern_amplitude_and_frequency(kind):
    amp, freq = 0.006, 2.0
    tr = T.normalize({"points": [[0.3, -0.2, 0.85], [0.3, 0.2, 0.85]], "pattern": {"kind": kind, "amp": amp, "freq": freq}})
    d = T.densify(tr, speed=0.04)                                        # 10 s of travel along +y
    lat = d["p"][:, 0] - 0.3                                             # lateral = across the path (x)
    t = d["t"]
    plateau = (t > 1.0) & (t < d["duration"] - 1.0)
    assert abs(np.abs(lat[plateau]).max() - amp) < 0.06 * amp           # peak = amp
    z = lat[plateau]
    crossings = np.sum((z[:-1] < 0) & (z[1:] >= 0))                      # upward zero crossings = cycles
    span = t[plateau][-1] - t[plateau][0]
    assert abs(crossings / span - freq) < 0.15 * freq
    assert np.allclose(d["p"][0], [0.3, -0.2, 0.85]) and np.allclose(d["p"][-1], [0.3, 0.2, 0.85], atol=1e-6)  # fades in / out
    assert np.allclose(d["p"][:, 2], 0.85)                               # lateral stays horizontal


def test_pattern_vertical_axis_and_span():
    tr = T.normalize({"points": [[0.3, -0.2, 0.85], [0.3, 0.0, 0.85], [0.3, 0.2, 0.85]],
                      "pattern": {"kind": "zigzag", "amp": 0.01, "freq": 1.5, "axis": "vertical", "span": [1, 2]}})
    d = T.densify(tr, speed=0.05)
    k1 = d["key"][1]
    assert np.allclose(d["p"][:k1 + 1, 2], 0.85)                         # nothing before the span
    assert np.abs(d["p"][k1:, 2] - 0.85).max() > 0.009


def test_demo_pattern_is_descriptive_not_applied():
    tr = T.normalize({"source": "demo", "points": [[0.3, -0.2, 0.85], [0.3, 0.2, 0.85]],
                      "pattern": {"kind": "zigzag", "amp": 0.01, "freq": 2}})
    assert np.allclose(T.densify(tr)["p"][:, 0], 0.3)


# ---------------------------------------------------------------------------- frames
def test_object_relative_frame_follows_the_object():
    poses = T.object_poses(WORLD)
    top = poses["red_cup"]
    assert abs(top[2] - 0.82) < 1e-9                                     # origin: the top centre
    tr = T.normalize({"frame": "object:red_cup", "points": [[0, 0, 0.1], [0.05, 0, 0.05]]})
    w = T.to_world(tr, poses)
    assert w["frame"] == "world" and np.allclose(w["points"][0]["p"], [0.35, -0.2, 0.92])
    moved = dict(poses, red_cup=[0.45, 0.1, 0.82])
    w2 = T.to_world(tr, moved)
    assert np.allclose(np.array(w2["points"][1]["p"]) - np.array(w["points"][1]["p"]), [0.10, 0.30, 0.0])
    back = T.to_frame(w, "object:red_cup", poses)
    assert back["points"][0]["p"] == tr["points"][0]["p"]
    with pytest.raises(T.TrajError):
        T.to_world(T.normalize({"frame": "object:banana", "points": [[0, 0, 0]]}), poses)


# ---------------------------------------------------------------------------- language keypoints
def obstacles(world):
    return [{"name": n, **o} for n, o in world["objects"].items()]


def test_keypoints_resolve_relative_to_objects():
    spec = {"keypoints": [{"rel": "above", "of": "the red cup", "dz": 0.1}, {"rel": "right_of", "of": "bowl", "dist": 0.05},
                          {"marker": "A", "dz": 0.05}, {"region": "left", "height": 0.2}, {"dxyz": [0.0, 0.0, -0.05]}]}
    tr, notes = T.from_keypoints(spec, dict(WORLD, hand=[0.36, -0.2, 0.95]))     # start right above the cup
    P = [p["p"] for p in tr["points"]]
    assert np.allclose(P[0], [0.35, -0.2, 0.92])
    assert any(np.allclose(p, [0.35, 0.2 - 0.07 - 0.05, 0.84]) for p in P)        # right of the bowl = -y, 10 cm above the table
    assert any(np.allclose(p, [0.40, 0.10, 0.79]) for p in P)
    assert np.allclose(P[-2], [0.31, 0.22, 0.94]) and np.allclose(P[-1], [0.31, 0.22, 0.89])
    assert tr["source"] == "language" and tr["meta"]["keypoints"] == spec["keypoints"]


def test_keypoints_clear_an_obstacle_on_the_way():
    spec = {"label": "from the cup to the left of the bowl", "keypoints": [{"rel": "above", "of": "red_cup", "dz": 0.05},
                                                                          {"rel": "left_of", "of": "bowl", "dz": 0.05}]}
    tr, notes = T.from_keypoints(spec, WORLD)
    assert "vase" in tr["meta"]["lifted"] and any("vase" in n for n in notes)
    d = T.densify(tr)
    vase = WORLD["objects"]["vase"]
    over = [p for p in d["p"] if math.hypot(p[0] - vase["pos"][0], p[1] - vase["pos"][1]) < vase["r"] + 0.04]
    assert over and min(p[2] for p in over) >= vase["pos"][2] + vase["h"] + T.CLEARANCE - 0.005
    res = T.check(d, [T.surface_checker(0.74), T.obstacle_checker(obstacles(WORLD))])
    assert all(res["ok"]) and all(res["sample_ok"]) and res["why"] is None


def test_via_over_and_around_and_going_down_on_purpose():
    tr, _ = T.from_keypoints({"keypoints": [{"via": "over", "of": "vase", "clearance": 0.08}]}, WORLD)
    assert any(abs(p["p"][2] - (0.98 + 0.08)) < 1e-6 for p in tr["points"])
    tr, _ = T.from_keypoints({"keypoints": [{"rel": "around", "of": "bowl", "dist": 0.04, "turns": 1}]}, WORLD)
    ring = [p["p"] for p in tr["points"]][-13:]
    r = [math.hypot(p[0] - 0.35, p[1] - 0.2) for p in ring]
    assert len(tr["points"]) >= 13 and np.allclose(r, 0.07 + 0.04, atol=1e-3)
    tr, _ = T.from_keypoints({"keypoints": [{"rel": "above", "of": "red_cup", "dz": 0.1}, {"rel": "at", "of": "red_cup", "grip": 1}]}, WORLD)
    assert np.allclose(tr["points"][-1]["p"], [0.35, -0.2, 0.84]) and tr["points"][-1]["grip"] == 1.0   # not lifted away


def test_keypoints_filled_in_by_a_model_use_their_form():
    """What a real model sent (every field filled, an enum value for every enum): `where` decides what is read."""
    filled = {"where": "", "from": "hand", "of": "", "rel": "above", "via": "over", "xyz": [0, 0, 0], "dxyz": [0, 0, 0],
              "marker": "", "region": "left", "dz": 0.1, "dist": 0.06, "dx": 0, "dy": 0, "height": 0.1, "clearance": 0.05,
              "speed": 0.05, "tol": 0.01, "yaw": 0, "tilt": 0, "hold": 0, "note": "", "pattern": False}
    kps = [dict(filled, where="pass", of="vase"), dict(filled, where="object", of="bowl", rel="left_of", height=None)]
    tr, notes = T.from_keypoints({"keypoints": kps}, WORLD)
    P = [p["p"] for p in tr["points"]]
    assert any(abs(p[0] - 0.42) < 1e-6 and abs(p[1]) < 1e-6 and p[2] > 0.98 + 0.04 for p in P)          # over the vase
    assert len({tuple(p) for p in P}) == len(P) and all(p.get("speed") == 0.05 for p in tr["points"])
    assert T.keypoint_form({"of": "vase", "via": "over", "rel": "above", "from": "hand"})[0] == "pass"      # no where: guessed
    assert T.keypoint_form({"xyz": [0.3, 0, 0.9], "from": "hand"})[0] == "point"
    assert T.keypoint_form({"from": "hand"}) == ("hand", {"from": "hand"})
    from pps.agent import _clean_keypoint
    c = _clean_keypoint(dict(filled, where="region"))
    assert c == {"where": "region", "region": "left", "dz": 0.1, "height": 0.1, "speed": 0.05, "tol": 0.01}


def test_keypoint_errors_name_what_is_there():
    with pytest.raises(T.TrajError) as e:
        T.from_keypoints({"keypoints": [{"rel": "above", "of": "banana"}]}, WORLD)
    assert "vase" in str(e.value)
    with pytest.raises(T.TrajError):
        T.from_keypoints({"keypoints": [{"marker": "F"}]}, WORLD)
    with pytest.raises(T.TrajError):
        T.from_keypoints({"keypoints": []}, WORLD)


def test_keypoints_in_an_object_frame_with_pattern_marks():
    spec = {"object": "bowl", "pattern": {"kind": "zigzag", "amp": 0.004, "freq": 2},
            "keypoints": [{"rel": "above", "of": "bowl", "dz": 0.03}, {"dxyz": [0, -0.05, 0], "pattern": True},
                          {"dxyz": [0, -0.05, 0], "pattern": True}, {"dxyz": [0, 0, 0.05]}]}
    tr, _ = T.from_keypoints(spec, dict(WORLD, hand=[0.35, 0.2, 0.9]))
    assert tr["frame"] == "object:bowl" and np.allclose(tr["points"][0]["p"], [0, 0, 0.03])
    assert tr["pattern"]["span"] == [1, 2]


# ---------------------------------------------------------------------------- checks
def test_check_reports_points_and_spans():
    tr = T.normalize({"points": [[0.3, 0, 0.8], [0.5, 0, 0.8], [0.9, 0, 0.8], [0.95, 0.1, 0.8]]})
    d = T.densify(tr)
    far = lambda pts, qs, info: [f"out of my reach by {(p[0] - 0.6) * 100:.0f} cm" if p[0] > 0.6 else None for p in pts]
    r = T.check(d, [far, T.surface_checker(0.74)], approach=np.array([[0.2, 0, 0.8]]))
    assert r["ok"] == [True, True, False, False] and r["approach_ok"] == [True]
    assert r["why"] == "Out of my reach by 35 cm from point 2 to point 4."
    low = T.normalize({"points": [[0.3, 0, 0.8], [0.3, 0.1, 0.70]]})
    r = T.check(T.densify(low), [T.surface_checker(0.74)])
    assert r["ok"] == [True, False] and "below the surface" in r["why"]
    crash = lambda pts, qs, info: 1 / 0
    r = T.check(T.densify(low), [crash])
    assert r["why"] and "check failed" in r["why"].lower()


# ---------------------------------------------------------------------------- edits
def test_select_and_edit_by_voice():
    tr, _ = T.from_keypoints({"keypoints": [{"rel": "above", "of": "red_cup"}, {"via": "over", "of": "vase"},
                                            {"rel": "left_of", "of": "bowl"}]}, WORLD)
    w = T.to_world(tr, T.object_poses(WORLD))
    near_vase = T.select(w, near="vase", world=WORLD)
    assert near_vase and all(math.hypot(w["points"][i]["p"][0] - 0.42, w["points"][i]["p"][1]) < 0.04 + 0.08 for i in near_vase)
    assert T.select(w, near="end", world=WORLD) == [len(w["points"]) - 1] and T.select(w, index=-1) == [len(w["points"]) - 1]
    assert T.select(w, near="there", world=WORLD)                         # marker P
    slower = T.edit(tr, near_vase, {"speed_factor": 0.5, "_default_speed": 0.2})
    assert all(slower["points"][i]["speed"] == 0.1 for i in near_vase)
    higher = T.edit(tr, [0], {"dz": 0.05})
    assert abs(higher["points"][0]["p"][2] - tr["points"][0]["p"][2] - 0.05) < 1e-9
    h = T.edit(tr, [0], {"height": 0.2}, surface=0.74)
    assert abs(h["points"][0]["p"][2] - 0.94) < 1e-9
    wr = T.edit(tr, [1], {"yaw": 45})
    assert wr["points"][1]["yaw"] == 45.0
    fewer = T.edit(tr, [1], {"remove": True})
    assert len(fewer["points"]) == len(tr["points"]) - 1
    more = T.insert(tr, 0)
    assert len(more["points"]) == len(tr["points"]) + 1
    assert np.allclose(more["points"][1]["p"], (np.array(tr["points"][0]["p"]) + tr["points"][1]["p"]) / 2, atol=1e-4)
    with pytest.raises(T.TrajError):
        T.select(w, index=99)
    with pytest.raises(T.TrajError):
        T.edit(T.normalize({"points": [[0, 0, 0]]}), [0], {"remove": True})


# ---------------------------------------------------------------------------- P3 demonstrations
def demo_file():
    """The format pps/human/demo.py writes (recordings/demos/<name>.json), built by its own stub extractor."""
    from pps.human import demo as demo_mod
    frames = []
    for k in range(60):
        t = k / 15
        frames.append({"t": t, "p": [0.9 - 0.004 * k, 0.1 + 0.01 * math.sin(2 * math.pi * 2 * t), 1.0], "q": [0.7, 0.7, 0, 0],
                       "pinch": 0.02 if 20 <= k < 40 else 0.09, "open": None})
    demo = {"name": "weave_test", "frames": frames, "anchor": {"object": "red_cup", "pos": [0.35, -0.2, 0.78]},
            "pattern": {"kind": "zigzag", "amp": 0.01, "freq": 2.0, "travel": 0.06, "axis": [0, 1, 0], "dir": [1, 0, 0]},
            "activity": "weave", "tool": "torch"}
    demo["trajectory"] = demo_mod.to_trajectory(demo)
    return demo


def test_demo_file_loads_cleanly():
    demo = demo_file()
    raw = json.loads(json.dumps(demo))                                   # as read back from disk
    tr = T.from_demo(raw)
    assert tr["source"] == "demo" and tr["frame"] == "object:red_cup" and tr["meta"]["demo"] == "weave_test"
    assert all(p["q"] is None for p in tr["points"])                     # the person's hand orientation is not the gripper's
    assert T.all_timed(tr) and tr["pattern"]["apply"] is False
    grips = [p["grip"] for p in tr["points"]]
    assert max(grips) == 1.0 and min(grips) == 0.0
    w = T.to_world(tr, T.object_poses(WORLD))
    d = T.densify(w)
    assert abs(d["duration"] - (demo["frames"][-1]["t"] - demo["frames"][0]["t"])) < 0.1
    assert [e[1] for e in d["events"]][:2] == ["grip", "grip"]
    assert np.allclose(w["points"][0]["p"], np.array([0.35, -0.2, 0.82]) + [0, 0, 0.08], atol=1e-3)   # starts above the cup
    no_anchor = dict(raw, trajectory=dict(raw["trajectory"], frame="world"))
    with pytest.raises(T.TrajError):
        T.from_demo(no_anchor)
    tr2 = T.from_demo(no_anchor, hand=[0.3, 0.0, 0.9])
    assert tr2["frame"] == "world" and np.allclose(tr2["points"][0]["p"], [0.3, 0.0, 0.98], atol=1e-3)


# ---------------------------------------------------------------------------- runtime glue
class FakeRT:
    def __init__(self, path):
        self.data = SimpleNamespace(mocap_pos=np.array([[0.2, 0.0, 0.9]]), mocap_quat=np.array([[0.0, 1.0, 0.0, 0.0]]))
        self.hands_mid = {"right_palm": 0}
        self.speed_scale, self.follow_s, self.mode, self.path = 1.0, 0.0, "FOLLOW", path
        self.grip = []

    def set_gripper(self, palm, v):
        self.grip.append(v)


def test_follow_step_runs_the_samples_in_time():
    tr = T.normalize({"points": [{"p": [0.3, -0.1, 0.9]}, {"p": [0.3, 0.1, 0.9], "grip": 1}, {"p": [0.35, 0.1, 0.9], "grip": 0}]})
    d = T.densify(tr, q0=DOWN_Q)
    path = {"pts": d["p"][d["key"]], "palm": "right_palm", "i": 0, "dense": dict(d, di=np.searchsorted(d["key"], np.arange(len(d["t"])), side="right") - 1)}
    rt = FakeRT(path)
    steps = 0
    while rt.mode == "FOLLOW" and steps < 5000:
        T.follow_step(rt, 0.01)
        steps += 1
    ta = path["_run"]["Ta"]
    assert rt.mode == "MIRROR" and abs(steps * 0.01 - (ta + d["duration"])) < 0.03
    assert np.allclose(rt.data.mocap_pos[0], [0.35, 0.1, 0.9]) and path["i"] == 2
    assert rt.grip == [1.0, 0.0]
    rt.mode, rt.follow_s, rt.speed_scale = "FOLLOW", 0.0, 0.5            # a restart at half speed takes twice as long
    rt.data.mocap_pos[0] = [0.2, 0.0, 0.9]
    steps = 0
    while rt.mode == "FOLLOW" and steps < 10000:
        T.follow_step(rt, 0.01)
        steps += 1
    assert abs(steps * 0.01 - 2 * (ta + d["duration"])) < 0.06


def test_follow_step_follows_points_nudged_while_running():
    tr = T.normalize({"points": [[0.3, -0.1, 0.9], [0.3, 0.0, 0.9], [0.3, 0.1, 0.9]]})
    d = T.densify(tr, q0=DOWN_Q)
    idx = np.array(d["key"])
    pts = d["p"][idx]
    path = {"pts": pts.copy(), "pts0": pts.copy(), "palm": "right_palm", "i": 0,
            "dense": dict(d, di=np.searchsorted(idx, np.arange(len(d["t"])), side="right") - 1, didx=idx)}
    rt = FakeRT(path)
    rt.data.mocap_pos[0] = [0.3, -0.1, 0.9]
    for _ in range(20):
        T.follow_step(rt, 0.01)
    path["pts"] = path["pts"] + np.array([0.0, 0.0, 0.03])          # the nudge pad: 3 cm up for the rest of the path
    while rt.mode == "FOLLOW":
        T.follow_step(rt, 0.01)
    assert np.allclose(rt.data.mocap_pos[0], [0.3, 0.1, 0.93], atol=1e-6)
