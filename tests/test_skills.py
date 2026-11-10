"""Unit tests for P4 skills (pps/skills, pps/human/periodic.py): pure, no server, no MuJoCo.
Run: .venv/bin/python -m pytest tests -q"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from pps import skills as SK  # noqa: E402
from pps import traj as T
from pps.features.copresence import to_world  # noqa: E402
from pps.human import periodic, synth  # noqa: E402
from pps.human.activity import ActivityTracker  # noqa: E402
from pps.human.model import ground  # noqa: E402
from pps.skills import geom  # noqa: E402
from pps.skills.extract import tool_angles  # noqa: E402

FS = 30.0
TOP = 0.748  # plate top in the weld cell
SEAM_A = [[0.45, -0.345, TOP], [0.45, -0.075, TOP]]
SEAM_L = [[0.37, 0.201, TOP], [0.481, 0.201, TOP], [0.481, 0.09, TOP]]
WORLD = {
    "surface": 0.74,
    "hand": [0.51, 0.13, 1.07],
    "caps": ["pick_place", "reach", "follow_path"],
    "objects": {
        "plate_a": {
            "pos": [0.404, -0.21, 0.744],
            "r": 0.15,
            "h": 0.004,
            "kind": "steel_plate",
            "size": [0.09, 0.30, 0.008],
            "shape": "box",
        },
        "plate_b": {
            "pos": [0.496, -0.21, 0.744],
            "r": 0.15,
            "h": 0.004,
            "kind": "steel_plate",
            "size": [0.09, 0.30, 0.008],
            "shape": "box",
        },
        "plate_c": {
            "pos": [0.42, 0.14, 0.744],
            "r": 0.06,
            "h": 0.004,
            "kind": "steel_plate",
            "size": [0.12, 0.12, 0.008],
            "shape": "box",
        },
        "tray": {
            "pos": [0.40, 0.0, 0.75],
            "r": 0.15,
            "h": 0.01,
            "kind": "plate",
            "size": [0.20, 0.30, 0.02],
            "shape": "box",
        },
        "bowl": {
            "pos": [0.35, 0.25, 0.77],
            "r": 0.075,
            "h": 0.03,
            "kind": "bowl",
            "size": [0.15, 0.15, 0.06],
            "shape": "cyl",
        },
        "button": {
            "pos": [0.30, -0.25, 0.75],
            "r": 0.03,
            "h": 0.01,
            "kind": "block",
            "shape": "box",
            "size": [0.06, 0.06, 0.02],
        },
    },
    "seams": {
        "seam_a": {"points": SEAM_A, "label": "the straight seam", "between": ["plate_a", "plate_b"]},
        "seam_l": {"points": SEAM_L, "label": "the L seam", "between": ["plate_c", "plate_l1", "plate_l2"]},
    },
}


@pytest.fixture(autouse=True)
def _skills_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("PC_SKILLS_DIR", str(tmp_path / "skills"))


def demo_from(t, P, q=None, other=None, **kw):
    frames = []
    for i, ti in enumerate(t):
        frames.append(
            {
                "t": float(ti),
                "p": [float(x) for x in P[i]],
                "q": None if q is None else [float(x) for x in q[i]],
                "other": None if other is None else [float(x) for x in other[i]],
                "pinch": None,
                "open": False,
            }
        )
    d = {
        "name": kw.pop("name", "t"),
        "frames": frames,
        "activity": kw.pop("activity", "weave"),
        "tool": kw.pop("tool", None),
        "reps": kw.pop("reps", 0),
        "duration": float(t[-1] - t[0]),
    }
    d.update(kw)
    return d


def weave_path(amp, freq, speed, secs=6.0, noise=0.0, shape="sin", seed=0, fs=FS, lead=0.0):
    """A person's hand weaving across the table edge: oscillation along x (toward / away from the robot), travel along -y."""
    rng = np.random.default_rng(seed)
    t = np.arange(0, secs, 1 / fs)
    ph = 2 * math.pi * freq * t
    w = np.sin(ph) if shape == "sin" else (2 / math.pi) * np.arcsin(np.sin(ph))
    P = np.stack([0.79 + amp * w, 0.12 - speed * t, 0.98 + 0 * t], axis=1)
    if lead:  # the hand moves into place first (not part of the weave)
        k = t < lead
        P[k] = P[np.argmax(~k)] + np.outer(lead - t[k], [0.05, 0.08, 0.10])
    return t, P + rng.normal(0, noise, P.shape)


def jitter_path(amp, secs=8.0, seed=0, fs=FS):
    """A still, hanging hand as MediaPipe's world landmarks jitter it: quasi-periodic ~0.9 Hz wander of 1-3 cm."""
    rng = np.random.default_rng(seed)
    t = np.arange(0, secs, 1 / fs)
    ph = rng.uniform(0, 2 * math.pi, (3, 3))
    walk = np.cumsum(rng.normal(0, 0.12, len(t)))
    P = np.stack(
        [
            1.0
            + amp
            * (
                np.sin(2 * math.pi * 0.9 * t + ph[0, c] + walk)
                + 0.5 * np.sin(2 * math.pi * 0.55 * t + ph[1, c])
                + 0.25 * np.sin(2 * math.pi * 1.7 * t + ph[2, c])
            )
            / 1.75
            for c in range(3)
        ],
        axis=1,
    )
    return t, P + rng.normal(0, 0.002, P.shape)


def rel(a, b):
    return abs(a - b) / abs(b)


# ---------------------------------------------------------------------------- weave extraction
@pytest.mark.parametrize(
    "amp,freq,speed",
    [
        (0.004, 2.0, 0.005),
        (0.005, 2.0, 0.01),
        (0.006, 3.0, 0.008),
        (0.015, 2.0, 0.03),
        (0.008, 1.2, 0.006),
        (0.005, 4.5, 0.012),
    ],
)
@pytest.mark.parametrize("noise", [0.0005, 0.001])
def test_weave_extraction_recovers_amp_freq_speed_within_15pct(amp, freq, speed, noise):
    errs = []
    for seed in range(3):
        sk = SK.extract(demo_from(*weave_path(amp, freq, speed, noise=noise, seed=seed)), kind="weave_seam")
        p = sk["params"]
        errs.append((rel(p["weave"]["amp"], amp), rel(p["weave"]["freq"], freq), rel(p["travel_speed"], speed)))
        assert p["weave"]["kind"] == "zigzag"
    worst = np.max(errs, axis=0)
    assert all(worst <= 0.15), f"amp/freq/speed errors {worst}"


def test_weave_extraction_triangle_wave_and_lead_in():
    sk = SK.extract(
        demo_from(*weave_path(0.006, 2.0, 0.01, secs=8, noise=0.0005, shape="tri", lead=1.5)), kind="weave_seam"
    )
    p, m = sk["params"], sk["measured"]
    assert (
        rel(p["weave"]["amp"], 0.006) <= 0.15
        and rel(p["weave"]["freq"], 2.0) <= 0.05
        and rel(p["travel_speed"], 0.01) <= 0.15
    )
    assert m["segment_s"][0] >= 1.0, "the lead-in before the weave is skipped"


def test_smoothing_gain_is_undone():
    """The co-presence EMA narrows a fast weave; the demo records it and extraction divides its gain out."""
    t, P = weave_path(0.006, 3.0, 0.01, noise=0.0003)
    S = P.copy()
    for i in range(1, len(S)):
        S[i] = 0.5 * P[i] + 0.5 * S[i - 1]
    raw = SK.extract(demo_from(t, S), kind="weave_seam")["params"]["weave"]["amp"]
    fixed = SK.extract(demo_from(t, S, smoothing={"ema": 0.5, "dt": 1 / FS}), kind="weave_seam")["params"]["weave"][
        "amp"
    ]
    assert raw < 0.006 * 0.85 and rel(fixed, 0.006) <= 0.1


def test_weave_shapes_circle_and_figure8():
    t = np.arange(0, 6, 1 / FS)
    amp, f, v = 0.006, 2.0, 0.01
    lat, ph = amp * np.sin(2 * math.pi * f * t), 2 * math.pi * f * t
    circ = np.stack([0.8 + lat, 0.1 - v * t - amp * np.cos(ph), 0.98 + 0 * t], 1)
    fig8 = np.stack([0.8 + lat, 0.1 - v * t - 0.5 * amp * np.sin(2 * ph), 0.98 + 0 * t], 1)
    assert periodic.analyze(t, circ)["kind"] == "circle"
    assert periodic.analyze(t, fig8)["kind"] == "figure8"


# ---------------------------------------------------------------------------- the weave false positive (PR #40)
@pytest.mark.parametrize("amp", [0.01, 0.02, 0.03])
@pytest.mark.parametrize("seed", range(4))
def test_still_jittery_hand_is_not_a_weave(amp, seed):
    t, P = jitter_path(amp, seed=seed)
    ok, why = periodic.is_weave(periodic.analyze(t, P))
    assert not ok, f"jitter read as a weave: {periodic.analyze(t, P)['amp']}"
    for w0 in np.arange(0, 6, 0.5):  # no 1.3-3 s window the live reader uses passes either
        for win in (1.3, 2.0, 3.0):
            m = (t >= w0) & (t <= w0 + win)
            assert not periodic.is_weave(periodic.analyze(t[m], P[m]))[0]
    assert not any(r["kind"] == "weave_seam" for r in SK.rank(demo_from(t, P, activity="still")))


@pytest.mark.parametrize("seed", range(4))
def test_live_reader_jitter_still_not_weave_and_weld_weave_is(seed):
    def acts(frames):
        tr, seen = ActivityTracker(), []
        for i, f in enumerate(frames):
            tr.update(i / 20, ground(to_world(f)), {"speed": 0.1})
            seen.append(tr.activity)
        return seen

    for a in (0.01, 0.02, 0.03):
        assert "weave" not in acts(synth.frames("jitter", 8, amp=a, seed=seed))
    assert "weave" in acts(synth.frames("weave", 6, amp=0.005, freq=2.0, travel=0.01))
    assert acts(synth.frames("wave", 4, z=0.0))[-1] == "wave", "a wave at shoulder height is a wave"


# ---------------------------------------------------------------------------- compile: geometry gives precision
def _dense(tr, apply=True):
    return T.densify(tr, q0=[0, 1, 0, 0], kind="arm", apply_pattern=apply)


def test_weave_compile_snaps_to_the_seam_polyline():
    sk = SK.make(
        "weave_seam", {"travel_speed": 0.02, "weave": {"kind": "zigzag", "amp": 0.005, "freq": 2.0}, "standoff": 0.015}
    )
    out = SK.compile(sk, WORLD, target="the L seam")
    tr = out["traj"]
    assert out["target"]["seam"] == "seam_l" and tr["source"] == "skill" and tr["meta"]["work_on"]
    i0, i1 = tr["meta"]["skill"]["arc"]
    poly = np.array(SEAM_L)
    for apply, tol in ((False, 0.0015), (True, 0.005 + 0.0015)):
        D = _dense(tr, apply)
        seg = D["p"][D["key"][i0] : D["key"][i1] + 1]
        d = [geom.dist_to_polyline(p, poly)[0] for p in seg]
        assert max(d) <= tol, f"apply={apply}: {max(d) * 1000:.1f} mm off the seam"
        assert np.allclose(seg[:, 2], TOP + 0.015, atol=1e-4)
    D = _dense(tr, True)
    seg = D["p"][D["key"][i0] : D["key"][i1] + 1]
    lat = max(geom.dist_to_polyline(p, poly)[0] for p in seg)
    assert lat >= 0.004, "the weave is there"
    # the hand is nearer the seam's far end (0.481, 0.09): auto direction starts there
    hand_near_end = dict(WORLD, hand=[0.5, 0.05, 1.0])
    p0 = SK.compile(sk, hand_near_end, "seam_l")["traj"]["points"][1]["p"]
    assert abs(p0[0] - 0.481) < 1e-3 and abs(p0[1] - 0.09) < 1e-3


def test_weave_compile_angles_and_seam_choices():
    sk = SK.make("weave_seam", {"travel_angle": 15, "work_angle": 80})
    tr = SK.compile(sk, WORLD, "seam_a")["traj"]
    pts = tr["points"][1:-1]
    assert all(p["tilt"] == 15 and p["roll"] == 10 for p in pts)
    assert pts[0]["note"] == "arc on" and pts[-1]["note"] == "arc off" and pts[0]["hold"] > 0
    assert SK.compile(sk, WORLD, "plate_c")["target"]["seam"] == "seam_l"  # an object -> its seam
    assert SK.compile(sk, dict(WORLD, welded=["seam_a"]), None)["target"]["seam"] == "seam_l"  # the next unwelded seam
    assert SK.compile(sk, dict(WORLD, welded=["seam_l"]), "other")["target"]["seam"] == "seam_a"
    custom = SK.compile(sk, WORLD, [[0.4, 0.0, 0.75], [0.5, 0.0, 0.75]])["traj"]
    assert custom["points"][1]["p"][2] == pytest.approx(0.765, abs=1e-4)
    with pytest.raises(SK.SkillError):
        SK.compile(sk, dict(WORLD, seams={}), None)
    with pytest.raises(SK.SkillError):
        SK.compile(sk, WORLD, "the purple seam")


def test_roll_and_tilt_lean_the_tool():
    q = T.point_quat({"roll": 20}, [0, 1, 0, 0], np.array([1.0, 0, 0]))
    tip = T.qrot(q, [0, 0, 1])  # the down-pointing approach axis of q0 = [0,1,0,0]
    assert tip[1] > 0.3 and tip[2] < 0, "roll > 0 leans the tip to the left of the travel"
    q = T.point_quat({"tilt": 20}, [0, 1, 0, 0], np.array([1.0, 0, 0]))
    assert T.qrot(q, [0, 0, 1])[0] > 0.3, "tilt > 0 leans the tip forward (push)"


def test_wipe_raster_covers_the_region():
    sk = SK.make("wipe", {"pattern": "raster", "spacing": 0.03, "force": 6, "speed": 0.1})
    out = SK.compile(sk, WORLD, "tray")
    tr = out["traj"]
    D = _dense(tr)
    x0, y0, x1, y1 = geom.footprint(WORLD["objects"]["tray"])
    contact = D["p"][np.isfinite(D["force"]) & (D["force"] > 0)]
    assert len(contact) and np.allclose(contact[:, 2], 0.763, atol=1e-3)
    gx, gy = np.meshgrid(np.linspace(x0 + 0.02, x1 - 0.02, 30), np.linspace(y0 + 0.02, y1 - 0.02, 40))
    grid = np.stack([gx.ravel(), gy.ravel()], 1)
    near = np.min(np.linalg.norm(grid[:, None, :] - contact[None, :, :2], axis=2), axis=1)
    assert np.mean(near <= 0.03 / 2 + 0.003) >= 0.97, f"coverage {np.mean(near <= 0.018):.2f}"
    assert [e for e in D["events"] if e[1] == "force"][0][2] == 6
    assert tr["meta"]["work_on"] == ["tray"]
    sp = SK.compile(SK.make("wipe", {"pattern": "spiral", "spacing": 0.02}), WORLD, "tray")["traj"]
    r = np.hypot(*(np.array([p["p"][:2] for p in sp["points"][1:-1]]) - np.array([0.40, 0.0])).T)
    assert r[-1] > r[0] + 0.05 and np.all(np.diff(r) >= -1e-6), "a spiral grows outward"


def test_wipe_extraction_from_strokes():
    t = np.arange(0, 8, 1 / FS)
    n, L, sp = 5, 0.16, 0.03  # 5 strokes of 16 cm, 3 cm apart (along y, stepping in x)
    k = np.minimum((t / 8 * n).astype(int), n - 1)
    u = (t / 8 * n) % 1.0
    y = np.where(k % 2 == 0, u, 1 - u) * L
    P = np.stack([0.75 - sp * k - sp * u * 0.0, y, 0.80 + 0 * t], 1)
    d = demo_from(t, P, activity="moving", tool="sponge")
    r = SK.rank(d)
    assert r and r[0]["kind"] == "wipe"
    sk = SK.extract(d)
    assert sk["kind"] == "wipe" and sk["params"]["pattern"] == "raster"
    assert rel(sk["params"]["spacing"], sp) <= 0.25 and sk["measured"]["strokes"] == n


def test_stir_circle_compile_and_extract():
    sk = SK.make("stir", {"radius": 0.04, "freq": 1.0, "turns": 2, "depth": 0.02})
    out = SK.compile(sk, WORLD, "bowl")
    tr = out["traj"]
    assert tr["frame"] == "object:bowl"
    W = T.to_world(tr, {"bowl": [0.35, 0.25, 0.80]})
    ring = np.array([p["p"] for p in W["points"][1:-1]])
    r = np.hypot(ring[:, 0] - 0.35, ring[:, 1] - 0.25)
    assert np.allclose(r, r[0], atol=1e-4) and r[0] <= 0.075 - 0.012 + 1e-6, "inside the bowl's wall"
    assert np.allclose(ring[:, 2], 0.78, atol=1e-4)
    assert (
        abs(
            np.unwrap(np.arctan2(ring[:, 1] - 0.25, ring[:, 0] - 0.35))[-1]
            - np.unwrap(np.arctan2(ring[:, 1] - 0.25, ring[:, 0] - 0.35))[0]
        )
        >= 2 * 2 * math.pi - 1e-6
    )
    # extraction: the person stirs 3 turns of 5 cm counter-clockwise at 0.8 Hz -> mirrored, the robot stirs clockwise
    t = np.arange(0, 3.75, 1 / FS)
    a = 2 * math.pi * 0.8 * t
    P = np.stack([0.8 + 0.05 * np.cos(a), 0.0 + 0.05 * np.sin(a), 0.85 + 0 * t], 1)
    d = demo_from(t, P + np.random.default_rng(0).normal(0, 0.001, P.shape), activity="moving", tool="spoon")
    assert SK.rank(d, WORLD)[0]["kind"] == "stir"
    p = SK.extract(d, world=WORLD)["params"]
    assert rel(p["radius"], 0.05) <= 0.1 and rel(p["freq"], 0.8) <= 0.1 and p["turns"] == 3 and p["direction"] == "cw"


def test_press_compile_and_extract():
    out = SK.compile(SK.make("press", {"force": 12, "hold_s": 2}), WORLD, "button")
    pts = out["traj"]["points"]
    assert pts[1]["force"] == 12 and pts[1]["hold"] == 2 and pts[1]["p"][2] == pytest.approx(0.763, abs=1e-4)
    assert out["press"] == {"target": "button", "force": 12.0, "hold_s": 2.0}
    t = np.arange(0, 4, 1 / FS)
    z = np.where(t < 1, 0.95 - 0.12 * t, np.where(t < 2.5, 0.83, 0.83 + 0.12 * (t - 2.5)))
    d = demo_from(t, np.stack([0.8 + 0 * t, 0 * t, z], 1), activity="still")
    assert SK.rank(d)[0]["kind"] == "press"
    assert rel(SK.extract(d)["params"]["hold_s"], 1.5) <= 0.2


def test_spot_from_a_lift_and_its_watch_message():
    t = np.arange(0, 8, 1 / FS)
    z = 1.2 - 0.2 * (1 - np.cos(2 * math.pi * t / 2.0)) / 2
    L = np.stack([0.9 + 0 * t, 0.25 + 0 * t, z], 1)
    R = L * [1, -1, 1]
    d = demo_from(t, R, other=L, activity="squat", reps=4, tool="barbell")
    assert SK.rank(d)[0]["kind"] == "spot"
    sk = SK.extract(d)
    v = sk["params"]["assist_when"]["bar_velocity_below"]
    assert 0.02 <= v <= 0.15 and sk["params"]["object"] == "barbell"
    out = SK.compile(sk, WORLD)
    assert out["kind"] == "watch" and out["msg"]["mode"] == "assist" and out["msg"]["spot"]["v_min"] == v
    assert "help when it slows below" in SK.describe(sk)


def test_mirror_motion_wraps_play_skill_and_needs_imitation():
    sk = SK.make("mirror_motion", {"clip": "dance"})
    with pytest.raises(SK.SkillError):
        SK.compile(sk, WORLD)  # an arm can't imitate a whole-body motion
    out = SK.compile(sk, dict(WORLD, caps=["imitate_video", "follow_path"]))
    assert out["msg"] == {"type": "task", "action": "play_skill", "name": "dance"}


# ---------------------------------------------------------------------------- kinds, params, files
def test_rank_asks_when_weave_and_wipe_fit_equally_and_the_tool_decides():
    t, P = weave_path(0.03, 1.2, 0.02, secs=8, noise=0.0005)  # a wide, slow zigzag while moving: scrubbing or weaving?
    r = SK.rank(demo_from(t, P, activity="weave"))
    kinds = [x["kind"] for x in r[:2]]
    assert set(kinds) == {"weave_seam", "wipe"} and SK.ambiguous(r)
    assert SK.rank(demo_from(t, P, tool="torch"))[0]["kind"] == "weave_seam"
    assert SK.rank(demo_from(t, P, tool="sponge"))[0]["kind"] == "wipe"


def test_torch_angles_from_the_palm():
    def palm_q(push_deg, lean_deg):
        # the person's torch (palm y axis) points down, tipped along their travel (-y in the world) and sideways
        a = np.array([0.0, 0.0, -1.0])
        a = np.array([0.0, -math.sin(math.radians(push_deg)), -math.cos(math.radians(push_deg))])
        a = a + np.array([-math.sin(math.radians(lean_deg)), 0, 0])
        y = a / np.linalg.norm(a)
        x = np.cross(y, [0, 0, 1.0])
        x /= np.linalg.norm(x)
        z = np.cross(x, y)
        from pps.human.model import mat_to_quat

        return mat_to_quat(np.column_stack([x, y, z]))

    qs = np.array([palm_q(15, 0)] * 30)
    a = tool_angles(qs, np.array([0.0, -1.0, 0.0]))  # mirrored travel direction is still -y
    assert a and abs(a["travel"] - 15) < 2 and abs(a["work"] - 90) < 2
    a = tool_angles(np.array([palm_q(-12, 0)] * 30), np.array([0.0, -1.0, 0.0]))
    assert abs(a["travel"] + 12) < 2, "a drag angle is negative"
    up = np.array([palm_q(15, 0)] * 30) * [1, 1, 1, 1]
    flat = tool_angles(np.array([[1.0, 0, 0, 0]] * 30), np.array([0.0, -1.0, 0.0]))
    assert flat is None, "a hand that doesn't point a torch down gives no angles (defaults are used)"
    assert len(up) == 30


def test_param_edit_round_trip_and_files(tmp_path):
    t, P = weave_path(0.005, 2.0, 0.01, noise=0.0005)
    sk = SK.extract(demo_from(t, P, name="p4 test weave"), kind="weave_seam")
    assert sk["name"] == "p4_test_weave"
    amp = sk["params"]["weave"]["amp"]
    sk2, said = SK.edit_text(sk, "wider and a bit slower")
    assert sk2["params"]["weave"]["amp"] == pytest.approx(round(amp * 1.25, 4), abs=1e-4)
    assert sk2["params"]["travel_speed"] < sk["params"]["travel_speed"] and len(said) == 2
    sk3, _ = SK.edit(sk2, {"weave.freq": 3.0, "travel_angle": -12, "weave.kind": "circle"})
    assert "3 Hz" in SK.describe(sk3) and "12° drag angle" in SK.describe(sk3) and "circle" in SK.describe(sk3)
    sk4, _ = SK.edit(sk3, {"weave.amp": 1.0, "standoff": {"add": -1}})  # clamped to the bounds
    assert sk4["params"]["weave"]["amp"] == 0.03 and sk4["params"]["standoff"] == 0.005
    SK.save(sk4)
    back = SK.load("p4_test_weave")
    assert back["params"] == sk4["params"] and SK.normalize(SK.normalize(back)) == SK.normalize(back)
    assert json.loads(json.dumps(back)) == back
    names = [k["name"] for k in SK.listing()]
    assert "p4_test_weave" in names and "weave_zigzag_4mm" in names and SK.load("spot_bench")["example"]
    assert SK.delete("p4_test_weave") and not SK.delete("spot_bench")
    with pytest.raises(SK.SkillError):
        SK.edit_text(sk, "sing me a song")
    with pytest.raises(SK.SkillError):
        SK.edit(sk, {"radius": 2})  # a weave has no radius
    with pytest.raises(SK.SkillError):
        SK.normalize({"kind": "juggle"})


def test_describe_sentences():
    assert (
        SK.describe(
            SK.make("weave_seam", {"weave": {"amp": 0.004, "freq": 2}, "travel_speed": 0.005, "travel_angle": 15})
        )
        == "4 mm zigzag at 2 Hz, 5 mm/s, 15° push angle"
    )
    assert SK.describe(SK.make("press", {"force": 10, "hold_s": 1.5})) == "press with 10 N for 1.5 s"
    assert "raster wipe, strokes 3 cm apart, 5 N, 1 pass" == SK.describe(SK.make("wipe"))


def test_phrases():
    c = SK.changes_from_text
    assert c("4 mm at 3 Hz, 6 mm/s, 15 degrees drag", "weave_seam") == {
        "weave.amp": 0.004,
        "weave.freq": 3.0,
        "travel_speed": 0.006,
        "travel_angle": -15.0,
    }
    assert c("a bit wider", "weave_seam") == {"weave.amp": {"mul": 1.12}}
    assert c("drag", "weave_seam", {"travel_angle": 12}) == {"travel_angle": -12}
    assert c("3 turns counter clockwise", "stir") == {"direction": "ccw", "turns": 3.0}
    assert c("help earlier, 60 N", "spot")["max_force"] == 60.0
    assert c("hello", "weave_seam") == {}


def test_packs_build_specs_with_exact_geometry():
    from pps.world import packs, scene

    ids = {p["id"] for p in packs.listing()}
    assert {"weld_cell", "gym", "workshop"} <= ids
    robot, spec = packs.target("set up a weld cell".replace("set up a ", ""), "panda")
    assert robot == "panda" and spec["pack"] == "weld_cell"  # the current body stays when it fits
    assert packs.target("gym", "panda")[0] == "g1"
    n, _ = scene.normalize(spec)
    assert n["tool"] == {"kind": "torch", "tcp": 0.16} and set(n["seams"]) == {"seam_a", "seam_l"}
    objs = {o["name"]: o for o in n["objects"]}
    a, b = objs["plate_a"], objs["plate_b"]
    assert a["pos"] == [0.404, -0.21] and a["size"][2] == 0.008, "exact: not moved, not spread, thin plates kept"
    gap_x = (b["pos"][0] - b["size"][0] / 2) - (a["pos"][0] + a["size"][0] / 2)
    assert (
        0.001 <= gap_x <= 0.004
        and abs((a["pos"][0] + a["size"][0] / 2 + gap_x / 2) - n["seams"]["seam_a"]["points"][0][0]) < 1e-3
    )
    g, _ = scene.normalize(packs.target("gym")[1])
    assert g["template"] == "gym" and {o["name"] for o in g["objects"]} == {"bench", "barbell"}
    assert next(o for o in g["objects"] if o["name"] == "barbell")["mass"] == 20.0
    assert scene.normalize(scene.DEFAULT_SCENE)[0].keys() == {"template", "objects"}, "the default scene is unchanged"
    assert "weld" in packs.prompt("weld_cell").lower()


def test_a_steady_circle_is_a_stir_not_strokes_and_a_growing_one_is_a_spiral_wipe():
    t = np.arange(0, 4, 1 / FS)
    a = 2 * math.pi * 0.75 * t
    circle = np.stack([0.8 + 0.05 * np.cos(a), 0.05 * np.sin(a), 0.85 + 0 * t], 1)
    assert [x["kind"] for x in SK.rank(demo_from(t, circle, activity="moving"))] == ["stir"]
    rr = 0.01 + 0.02 * t
    spiral = np.stack([0.8 + rr * np.cos(a), rr * np.sin(a), 0.80 + 0 * t], 1)
    d = demo_from(t, spiral, activity="moving", tool="cloth")
    assert SK.rank(d)[0]["kind"] == "wipe" and SK.extract(d)["params"]["pattern"] == "spiral"


def test_which_kind_question_matches_words():
    from pps.features.candidates import pick_option

    opts = [
        {"key": "1", "name": "weave_seam", "label": "weave along a seam", "color": None},
        {"key": "2", "name": "wipe", "label": "wipe an area", "color": None},
    ]
    assert pick_option(opts, "the weave")["name"] == "weave_seam"
    assert pick_option(opts, "wiping")["name"] == "wipe"
    assert pick_option(opts, "two")["name"] == "wipe"
