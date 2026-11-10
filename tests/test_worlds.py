"""Generated worlds (issue #48, docs/worlds.md): every template builds for three seeds with a Panda and once with every
robot kind that fits it, compiles in MuJoCo, its clutter rests without interpenetrating, its fixtures' joints stay within
their limits, a conveyor carries what is on it, and the same seed always gives the same layout. Headless: no server."""

import mujoco
import numpy as np
import pytest

from pps.world import fixtures as FX
from pps.world import objects as OBJ
from pps.world import rooms, scene

SEEDS = (0, 1, 2)
KIND_BODIES = {
    "arm": "ur5e",
    "humanoid": "g1",
    "bimanual": "aloha",
    "mobile_manipulator": "tidybot",
    "quadruped_arm": "spot",
}
_BUILT = {}


def built(tpl, seed, robot="panda", physics=True):
    key = (tpl, seed, robot, physics)
    if key not in _BUILT:
        _BUILT[key] = scene.build(robot, {"template": tpl, "seed": seed}, physics=physics)
    return _BUILT[key]


def settled(w, sec=0.4):
    """A plant (MjData) of the world stepped for `sec` with the robot held at home."""
    m = w.model
    d = mujoco.MjData(m)
    if w.home_q is not None:
        d.qpos[: len(w.home_q)] = w.home_q
    mujoco.mj_forward(m, d)
    for _ in range(int(sec / m.opt.timestep)):
        mujoco.mj_step(m, d)
    return m, d


# ---------------------------------------------------------------------------------------------- generation (no MuJoCo world)
@pytest.mark.parametrize("tpl", list(rooms.ROOMS))
def test_same_seed_same_layout_and_seeds_differ(tpl):
    a, b = rooms.generate(tpl, 3), rooms.generate(tpl, 3)
    assert a == b  # deterministic, physics settling included
    c = rooms.generate(tpl, 4)
    sig = lambda s: (
        s["surface"],
        [(f["name"], f["pos"], f["size"]) for f in s["fixtures"]],
        [(o["name"], o["pos"]) for o in s["objects"]],
    )
    assert sig(a) != sig(c)


@pytest.mark.parametrize("tpl", list(rooms.ROOMS))
def test_generated_spec_is_sane(tpl):
    s = rooms.generate(tpl, 1)
    sf = s["surface"]
    assert (
        sf["x"][0] == pytest.approx(0.22) and sf["x"][1] > 0.4 and sf["y"][0] < 0 < sf["y"][1]
    )  # the station: near edge at 0.22
    assert s["fixtures"] and s["objects"]
    assert {f["kind"] for f in s["fixtures"]} <= set(FX.KINDS)
    names = [o["name"] for o in s["objects"]]
    assert len(names) == len(set(names))
    for o in s["objects"]:
        assert len(o["pos"]) == 3 and o["exact"] and abs(np.linalg.norm(o["quat"]) - 1) < 1e-3
        assert o["kind"] in scene.KINDS
    assert rooms.ROOMS[tpl]["group"] in ("home", "industrial")


def test_listing_and_templates():
    ids = [t["id"] for t in rooms.listing()]
    assert set(ids) == {
        "kitchen_full",
        "dining",
        "living_room",
        "laundry",
        "assembly_line",
        "warehouse",
        "machine_tending",
        "electronics_bench",
        "lab",
        "packing",
    }
    assert {t["id"] for t in scene.templates()} >= set(ids)


def test_kitchen_has_what_the_check_needs():
    """Every kitchen seed: a top drawer with a spoon in it, a sink, drawers that open toward the robot."""
    for seed in SEEDS:
        s = rooms.generate("kitchen_full", seed)
        spoon = next(o for o in s["objects"] if o["name"] == "spoon")
        assert spoon["in"] == "drawer_1"
        assert any(f["kind"] == "sink" for f in s["fixtures"])


def test_object_kinds_have_geometry():
    for kind, k in OBJ.KINDS.items():
        half = tuple(v / 2 for v in k["size"])
        assert OBJ.visual(kind, half, "1 0 0 1").count("<geom") >= 1
        assert k["mass"] > 0 and k["cls"] in ("box", "round", "long", "flat")
        if k["cls"] == "long":
            assert OBJ.grip(kind, half) is not None


def test_normalize_keeps_a_generated_world_and_adds_objects_on_the_surface():
    spec, _ = scene.normalize({"template": "warehouse", "seed": 2, "objects": [{"name": "red cup", "kind": "cup"}]})
    assert spec["fixtures"] and spec["surface"]["top"] > 0.5
    cup = next(o for o in spec["objects"] if o["name"] == "red_cup")
    assert len(cup["pos"]) == 2  # placed on the pick table like any added object
    again, _ = scene.normalize(spec)  # round trip (state.world.scene -> world set)
    assert [o["name"] for o in again["objects"]] == [o["name"] for o in spec["objects"]]
    assert again["fixtures"] == spec["fixtures"]


# ---------------------------------------------------------------------------------------------- built worlds (MuJoCo)
@pytest.mark.parametrize("tpl", list(rooms.ROOMS))
@pytest.mark.parametrize("seed", SEEDS)
def test_builds_with_physics_and_settles(tpl, seed):
    w = built(tpl, seed)
    m = w.model
    assert w.fixtures is not None and m.geom("table") is not None  # the main work surface is the geom called table
    assert w.ik_model.nq < m.nq  # planner: robot only; plant: + objects and fixture joints
    m, d = settled(w, 0.3)
    obj_geoms = {
        g
        for g in range(m.ngeom)
        if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "").startswith("obj_")
    }
    worst = min(
        [d.contact[i].dist for i in range(d.ncon) if d.contact[i].geom1 in obj_geoms or d.contact[i].geom2 in obj_geoms]
        or [0.0]
    )
    assert worst > -0.006, f"{tpl}/{seed}: objects interpenetrate by {-worst * 1000:.1f} mm"
    assert np.all(np.isfinite(d.qpos))


@pytest.mark.parametrize("tpl", list(rooms.ROOMS))
def test_every_robot_kind_that_fits_builds_clear_of_the_furniture(tpl):
    """One body of every kind that fits: it builds (planner + plant) and stands / is bolted clear of the fixtures."""
    from pps.world.runtime import make_runtime

    fits = set(rooms.ROOMS[tpl]["robots"])
    for kind, rid in KIND_BODIES.items():
        if rid not in fits:
            continue
        w = built(tpl, 0, rid)
        assert w.model.nbody > 10 and w.profile.kind == kind
        rt = make_runtime(w)
        pl, m, d = rt.plant, rt.model, rt.data
        for _ in range(10):
            pl.step(1 / 30, rt.configuration.q, rt.base_pose())
        clash = [
            mujoco.mj_id2name(
                m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2 if m.geom_bodyid[c.geom1] in pl.robot_bodies else c.geom1
            )
            for c in (d.contact[i] for i in range(d.ncon))
            if (m.geom_bodyid[c.geom1] in pl.robot_bodies) != (m.geom_bodyid[c.geom2] in pl.robot_bodies)
            and c.dist < -0.003
        ]
        assert not [n for n in clash if n != "floor"], f"{tpl}: the {rid} starts inside {clash}"


@pytest.mark.parametrize("tpl", list(rooms.ROOMS))
def test_fixture_joints_stay_within_limits(tpl):
    w = built(tpl, 1)
    m = w.model
    d = mujoco.MjData(m)
    if w.home_q is not None:
        d.qpos[: len(w.home_q)] = w.home_q
    parts = [p for p in w.fixtures.parts if p["kind"] not in ("slat", "roller")]
    for p in parts:  # push every part hard toward open
        j = m.joint(p["joint"]).id
        d.qfrc_applied[m.jnt_dofadr[j]] = 60.0 if p["type"] == "slide" else 12.0
    for _ in range(int(0.6 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    moved = 0
    for p in parts:
        j = m.joint(p["joint"]).id
        q = float(d.qpos[m.jnt_qposadr[j]])
        lo, hi = p["range"]
        assert lo - 0.02 <= q <= hi + 0.03, f"{p['name']}: {q:.3f} outside [{lo}, {hi}]"
        moved += q > lo + 0.02
    assert not parts or moved >= len(parts) // 2


def test_conveyor_carries_parts_and_stops_at_the_eye():
    from pps.world.runtime import make_runtime

    w = built("assembly_line", 1)
    rt = make_runtime(w)
    pl = rt.plant
    start = {n: pl.d.xpos[o["bid"]].copy() for n, o in pl.objects.items()}
    belt = rt.fixtures.belts[0]
    for _ in range(int(9.0 * 30)):
        pl.step(1 / 30, rt.configuration.q, rt.base_pose())
        if belt["held"]:
            break
    ax = np.asarray(belt["axis"]) * np.sign(belt["speed"])
    travelled = max(float(np.dot(pl.d.xpos[o["bid"]] - start[n], ax)) for n, o in pl.objects.items())
    assert travelled > 0.1  # parts ride the moving slats
    assert belt["held"] in pl.objects  # and the belt stops for the one at the photo-eye


def test_kinematic_world_has_fixtures_as_mocap():
    w = built("kitchen_full", 0, "panda", physics=False)
    from pps.world.runtime import make_runtime

    rt = make_runtime(w)
    fx = rt.fixtures
    p = fx.by_name["drawer_1"]
    assert p["mid"] >= 0 and p["jid"] is None
    fx.set_q(p, 0.2)
    assert fx.frac(p) == pytest.approx(0.2 / p["range"][1], abs=1e-6)
    h0, _ = fx.handle_at(p, 0.0)
    h1, _ = fx.handle_at(p, 0.2)
    assert np.linalg.norm(h1 - h0) == pytest.approx(0.2, abs=1e-6)


def test_find_parts_and_containers_by_what_people_say():
    from pps import tasks as T
    from pps.world.runtime import make_runtime

    w = built("kitchen_full", 0, "panda", physics=False)
    T.apply_surface(w.surface, w.regions)
    tk = T.Tasks(make_runtime(w), w.kinds)
    assert tk.find_part("the top drawer")[0]["name"] == "drawer_1"
    assert tk.find_part("second drawer")[0]["name"] == "drawer_2"
    assert tk.find_part("drawer_3")[0]["name"] == "drawer_3"
    assert tk.find_part("the fridge door")[0]["name"] == "fridge_door"
    p, why = tk.find_part("the drawer")
    assert p is None and why.startswith("Which")  # several: ask
    assert tk.find_container("the sink")[0]["name"] == "sink"
    assert "spoon" in tk.contents(tk.find_container("drawer_1")[0])
    w2 = built("lab", 1, "panda", physics=False)
    T.apply_surface(w2.surface, w2.regions)
    tk2 = T.Tasks(make_runtime(w2), w2.kinds)
    c, _ = tk2.find_container("the tube rack")
    assert c["kind"] == "slot" and c["fixture"] == "rack" and not tk2.contents(c)   # a free slot of the right rack


def test_lower_drawer_knob_is_reached_from_the_front():
    """The second drawer's knob sits right under the top drawer's: from above the fingers would sweep into it, from the
    front they don't; the plan to open it is made the way that works (physics: the planner keeps the arm off the
    furniture and the jaws must reach the knob), and the hand comes in along the knob's outward direction."""
    import math

    from pps import tasks as T
    from pps.manip import HANDLE_PITCHES
    from pps.world.runtime import make_runtime
    w = built("kitchen_full", 1, "panda", physics=True)
    T.apply_surface(w.surface, w.regions)
    rt = make_runtime(w)
    tk = T.Tasks(rt, w.kinds)
    p = tk.fx.by_name["drawer_2"]
    h, out = tk.fx.handle_now(p)
    o = np.array([out[0], out[1], 0.0]) / np.hypot(out[0], out[1])
    a = np.cross([0.0, 0.0, 1.0], o)
    assert tk._handle_blocked(p, h, np.array([0.0, 0.0, -1.0]), a) == "the top drawer"     # from above: the knob above
    assert tk._handle_blocked(p, h, -o, a) is None                                          # from the front: clear
    up = tk.fx.by_name["drawer_1"]
    hu, _ = tk.fx.handle_now(up)
    assert tk._handle_blocked(up, hu, np.array([0.0, 0.0, -1.0]), a) is None                # the top drawer: from above
    tk.operate("t", "open", "the second drawer", 0.8)
    assert tk.task["status"] == "running", tk.task["message"]
    pitch = tk.fxop["pitch"]
    assert pitch in HANDLE_PITCHES and pitch < 90
    k = next(i for i, e in tk.events.items() if e == "grasp_fx")
    pre, grasp = np.asarray(rt.path["pts"][k - 3]), np.asarray(rt.path["pts"][k])
    d = (grasp - pre) / np.linalg.norm(grasp - pre)                                         # the way in: along -out, pitched
    assert float(np.dot(d[:2], -o[:2])) == pytest.approx(math.cos(math.radians(pitch)), abs=0.15)
    knob = tk._knob_geoms(p)
    assert knob and all(rt.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER for g in knob)


def test_kinematic_mode_plans_the_same_ways():
    """Kinematic mode (no plant: the drawer follows the hand) reaches a lower drawer's knob the same way."""
    from pps import tasks as T
    from pps.world.runtime import make_runtime
    w = built("kitchen_full", 1, "panda", physics=False)
    T.apply_surface(w.surface, w.regions)
    rt = make_runtime(w)
    tk = T.Tasks(rt, w.kinds)
    tk.operate("k", "open", "the second drawer", 0.8)
    assert tk.task["status"] == "running", tk.task["message"]
    assert tk.fxop["pitch"] < 90
