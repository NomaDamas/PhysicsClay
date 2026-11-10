"""P6 calibration, headless (docs/calibration.md): parameter apply / restore, the randomizer, the latency buffer, the
driver protocol, the log format and system identification (a tiny model, and the Panda plant on a short swing).

Run: .venv/bin/python -m pytest tests/test_calib.py -q        (no server; about 15 s)"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pps import physics as phys  # noqa: E402
from pps.bridge import LeRobotDriver, RobotDriver, ROS2Driver, SimDriver, conforms  # noqa: E402
from pps.calib import params as P  # noqa: E402
from pps.calib import probes as PR  # noqa: E402
from pps.calib import sysid  # noqa: E402
from pps.calib.log import Log  # noqa: E402
from pps.calib.randomize import Randomizer  # noqa: E402

SCENE = {
    "template": "table",
    "objects": [{"name": "red_block", "kind": "block", "color": "red", "pos": [0.42, -0.15], "movable": True}],
}


@pytest.fixture(scope="module")
def panda():
    """A headless Panda world with one block: (world, runtime, tasks, driver)."""
    return PR.headless_robot("panda", SCENE)


def _arrays(m):
    return {
        k: getattr(m, k).copy()
        for k in (
            "geom_friction",
            "body_mass",
            "body_inertia",
            "dof_damping",
            "dof_armature",
            "dof_frictionloss",
            "actuator_gainprm",
            "actuator_biasprm",
        )
    }


# ------------------------------------------------------------------ parameters
def test_params_apply_restore_round_trip(panda):
    _, rt, _, _ = panda
    pl = rt.plant
    snap, before = P.snapshot(pl), _arrays(pl.m)
    p = {
        "friction_scale": 0.6,
        "mass_scale": 1.8,
        "damping_scale": 1.4,
        "kp_scale": 0.7,
        "latency_ms": 25.0,
        "armature_scale": 1.5,
    }
    for _ in range(2):  # applying twice must not compound (always relative to the base)
        got = P.apply(pl, p)
    m = pl.m
    blk = pl.objects["red_block"]["bid"]
    assert got["latency_ms"] == 25.0
    assert np.isclose(m.body_mass[blk], before["body_mass"][blk] * 1.8)
    assert np.allclose(m.geom_friction[:, 0], before["geom_friction"][:, 0] * 0.6)
    assert np.allclose(m.dof_damping, before["dof_damping"] * 1.4)
    a = pl.track[0][0]
    assert np.isclose(m.actuator_gainprm[a, 0], before["actuator_gainprm"][a, 0] * 0.7)
    assert np.isclose(m.actuator_biasprm[a, 2], before["actuator_biasprm"][a, 2] * math.sqrt(0.7))
    assert np.allclose(m.dof_armature[: pl.nv_r], before["dof_armature"][: pl.nv_r] * 1.5)
    P.restore(pl, snap)
    after = _arrays(pl.m)
    assert all(np.array_equal(before[k], after[k]) for k in before), [
        k for k in before if not np.array_equal(before[k], after[k])
    ]
    assert P.snapshot(pl) == snap


def test_params_space_and_message(panda):
    _, rt, _, _ = panda
    specs = P.space()
    x = P.to_unit({"kp_scale": 1.3, "latency_ms": 20.0}, specs)
    back = P.from_unit(x, specs)
    assert abs(back["kp_scale"] - 1.3) < 1e-9 and abs(back["latency_ms"] - 20.0) < 1e-9
    assert set(P.fit_set(rt.plant)) == {"friction_scale", "mass_scale", "damping_scale", "kp_scale", "latency_ms"}
    msg = P.physics_message({"friction_scale": 0.5, "lighting": 1})
    assert msg == {"type": "physics", "op": "set", "params": {"friction_scale": 0.5}}
    assert all(n in phys.DEFAULTS for n in P.PARAMS)


def test_model_params_on_a_bare_model():
    m = mujoco.MjModel.from_xml_string(TINY)
    mp = P.ModelParams(m, objects=[m.body("box").id], servos=[0], robot_dofs=[0])
    g0, b0 = m.actuator_gainprm.copy(), m.body_mass.copy()
    mp.set_params({"kp_scale": 2.0, "mass_scale": 3.0})
    assert m.actuator_gainprm[0, 0] == pytest.approx(2 * g0[0, 0]) and m.body_mass[m.body("box").id] == pytest.approx(
        3 * b0[m.body("box").id]
    )
    mp.reset_params()
    assert np.array_equal(m.actuator_gainprm, g0) and np.array_equal(m.body_mass, b0)


# ------------------------------------------------------------------ randomization
def test_randomizer_bounds_and_determinism():
    r1, r2, r3 = Randomizer(seed=3), Randomizer(seed=3), Randomizer(seed=4)
    a, b, c = r1.sample_n(400), r2.sample_n(400), r3.sample_n(400)
    assert a == b and a != c
    assert all(r1.contains(p) for p in a)
    for n, rg in r1.ranges().items():
        v = np.array([p[n] for p in a])
        assert v.min() >= rg["lo"] and v.max() <= rg["hi"]
        if rg["log"]:  # log-uniform: the median sits at the geometric middle
            assert abs(math.log(np.median(v)) - 0.5 * (math.log(rg["lo"]) + math.log(rg["hi"]))) < 0.15
    near = r1.around({"friction_scale": 0.6, "latency_ms": 30.0}, rel=0.1, abs_ms=5.0)
    s = near.sample_n(100)
    assert all(
        0.54 - 1e-9 <= p["friction_scale"] <= 0.66 + 1e-9 and 25 - 1e-9 <= p["latency_ms"] <= 35 + 1e-9 for p in s
    )
    seen = []
    hooked = Randomizer(["kp_scale"], seed=1).on("lighting", lambda target, v: seen.append(v))
    p = hooked.sample()
    assert set(p) == {"kp_scale", "lighting"} and hooked.message(p)["params"] == {"kp_scale": p["kp_scale"]}


# ------------------------------------------------------------------ latency buffer
@pytest.mark.parametrize("lat_ms, hold", [(0.0, 0), (20.0, 10), (5.0, 2)])
def test_latency_buffer_delays_by_the_right_number_of_substeps(panda, lat_ms, hold):
    _, rt, _, _ = panda
    pl = rt.plant
    snap = P.snapshot(pl)
    try:
        P.apply(pl, {"latency_ms": lat_ms})
        q0 = np.array(rt.configuration.q, float)
        pl.reset(q0)
        a, adr, _ = pl.track[0]
        q1 = q0.copy()
        q1[adr] += 0.2
        ctrl = []
        for _ in range(16):  # one 2 ms substep per call: the setpoint ramps toward q1
            pl.step(phys.DT, q1)
            ctrl.append(float(pl.d.ctrl[a]))
        moved = [abs(c - ctrl[0]) > 1e-12 or abs(c - q0[adr]) > 1e-12 for c in ctrl]
        assert all(abs(c - q0[adr]) < 1e-12 for c in ctrl[:hold]), ctrl[: hold + 1]
        assert moved[hold], f"the setpoint should move at substep {hold + 1}: {ctrl[:hold + 2]}"
    finally:
        P.restore(pl, snap)
        pl.reset(np.array(rt.configuration.q, float))


# ------------------------------------------------------------------ driver protocol
def test_sim_driver_conforms(panda):
    _, rt, _, _ = panda
    drv = SimDriver(rt.plant, base_pose=rt.base_pose)
    assert isinstance(drv, RobotDriver) and conforms(drv) == []
    s = drv.read_state()
    assert len(s["q"]) == len(drv.capabilities["joints"]) == rt.plant.nq_r and set(s["ee_force"]) == {"right"}
    assert 0.0 <= s["gripper"]["right"] <= 1.0 and "red_block" in s["objects"]
    with pytest.raises(ValueError):
        drv.send_q_des(np.zeros(3), 0.0)
    drv.send_q_des(s["q"], s["t"])
    t0 = s["t"]
    drv.advance(0.02)
    assert drv.read_state()["t"] == pytest.approx(t0 + 0.02, abs=1e-9)
    drv.send_gripper("right", 1.0)
    assert drv.commanded_grip() == {"right": 1.0}
    drv.send_gripper("right", 0.0)
    noisy = SimDriver(rt.plant, noise={"q": 1e-3}, seed=0)
    assert not np.allclose(noisy.read_state()["q"], drv.read_state()["q"])


def test_hardware_stubs_say_what_they_need():
    for cls in (LeRobotDriver, ROS2Driver):
        d = cls(joints=["j1", "j2"])
        problems = conforms(d)
        assert problems and "Needs" in problems[-1]
        with pytest.raises(NotImplementedError, match="Needs"):
            d.send_q_des([0, 0], 0.0)


# ------------------------------------------------------------------ system identification
TINY = """<mujoco><option timestep="0.002"/>
  <worldbody><geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="link"><joint name="j" type="hinge" axis="0 1 0" damping="0.3"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/></body>
    <body name="box" pos="0.6 0 0.03"><freejoint/><geom type="box" size="0.03 0.03 0.03" mass="0.2"/></body>
  </worldbody><actuator><position joint="j" kp="40" kv="1.5"/></actuator></mujoco>"""


def test_sysid_recovers_kp_on_a_tiny_model():
    """A one-joint arm holding against gravity: the step response and the sag both depend on the servo stiffness."""
    m = mujoco.MjModel.from_xml_string(TINY)
    d = mujoco.MjData(m)
    mp = P.ModelParams(m, objects=[m.body("box").id], servos=[0], robot_dofs=[0])

    def rollout(kp):
        mp.set_params({"kp_scale": kp})
        mujoco.mj_resetData(m, d)
        out = []
        for k in range(400):
            d.ctrl[0] = 0.0 if k < 50 else 0.5
            mujoco.mj_step(m, d)
            out.append(d.qpos[0])
        return np.array(out)

    real = rollout(1.37) + np.random.default_rng(0).normal(0, 2e-4, 400)
    specs = [P.spec("kp_scale")]
    J = lambda xs: [float(np.mean((rollout(P.from_unit(x, specs)["kp_scale"]) - real) ** 2)) for x in xs]
    x, f, sd, gens = sysid._minimize(J, P.to_unit({"kp_scale": 1.0}, specs), 0.3, 20, popsize=6, seed=0)
    kp = P.from_unit(x, specs)["kp_scale"]
    assert abs(kp - 1.37) / 1.37 < 0.03, kp


def test_replay_reproduces_and_fit_recovers_kp_on_the_panda(panda):
    """A swing of one joint on a robot with secret stiffness: replaying with the truth reproduces it, the fit finds it."""
    w, rt, tasks, _ = panda
    secret = {"kp_scale": 0.72}
    pl = rt.plant
    snap = P.snapshot(pl)
    try:
        P.apply(pl, secret)
        pl.reset(np.array(rt.configuration.q, float))
        pl.settle()
        drv = SimDriver(pl, base_pose=rt.base_pose)
        logs = PR.run_headless(
            PR.ProbeRunner(rt, tasks, drv, [PR.Swing(joints=1)], meta={"robot": "panda", "scene": w.spec})
        )
    finally:
        P.restore(pl, snap)
    assert len(logs) == 1 and logs[0].probe == "swing" and len(logs[0]) > 100
    lg = logs[0]
    sim = sysid.PlantSim("panda", w.spec)
    e_true = sysid.errors(lg, sim.rollout(lg, secret))
    e_nom = sysid.errors(lg, sim.rollout(lg, {}))
    assert math.degrees(math.sqrt(e_true["q"])) < 0.02 < math.degrees(math.sqrt(e_nom["q"]))
    r = sysid.fit(
        logs, "panda", w.spec, names=["kp_scale"], budget={"gens": 8, "polish": 0}, workers=0, start={"kp_scale": 1.0}
    )
    assert abs(r["params"]["kp_scale"] - 0.72) / 0.72 < 0.08, r["params"]
    assert r["err_after"] < r["err_before"]


def test_log_round_trip(tmp_path, panda):
    rec_log = Log(
        t=np.arange(5) * 0.01,
        q_des=np.ones((5, 9)),
        grip=np.zeros((5, 1)),
        q=np.ones((5, 9)),
        dq=np.zeros((5, 9)),
        ee_force=np.full((5, 1, 3), np.nan),
        obj_p=np.zeros((5, 1, 3)),
        obj_q=np.tile([1.0, 0, 0, 0], (5, 1, 1)),
        base=np.full((5, 7), np.nan),
        meta={"probe": "swing", "roles": ["right"], "objects": ["red_block"]},
    )
    p = rec_log.save(tmp_path / "a.npz")
    back = Log.load(p)
    assert (
        back.probe == "swing"
        and np.array_equal(back.q, rec_log.q)
        and np.isnan(back.ee_force).all()
        and (back.seg == -1).all()
    )
    from pps.calib.log import clean  # a simulated robot that crashed (MuJoCo reset): cut there

    rec_log.t = np.array([3.0, 3.01, 3.02, 0.0, 0.01])
    cut = clean(rec_log)
    assert len(cut) == 3 and "truncated" in cut.meta


def test_lerobot_episode_reader(tmp_path, panda):
    """A P1 LeRobot episode (parquet + meta/info.json): named state / action columns ("joint:<name>", "gripper:<role>")
    are mapped onto the plant's joints by name; the episode replays in the headless simulator."""
    pa = pytest.importorskip("pyarrow")
    import json

    import pyarrow.parquet as pq

    from pps.bridge.sim import joint_labels
    from pps.calib.log import from_lerobot

    w, rt, _, _ = panda
    labels = joint_labels(rt.plant.m, rt.plant.nq_r)  # joint1..7, finger_joint1, finger_joint2
    arm = [j for j in labels if not j.startswith("finger")]
    names = [f"joint:{j}" for j in arm] + ["gripper:right"]
    root = tmp_path / "ds"
    (root / "meta").mkdir(parents=True)
    (root / "data" / "chunk-000").mkdir(parents=True)
    (root / "meta" / "info.json").write_text(
        json.dumps({"features": {"observation.state": {"names": names}, "action": {"names": names}}})
    )
    q0 = np.array(rt.configuration.q, float)
    n = 30
    rows = [[float(q0[labels.index(j)]) + (0.002 * k if j == arm[0] else 0.0) for j in arm] for k in range(n)]
    tab = pa.table(
        {
            "observation.state": [r + [1.0] for r in rows],
            "action": [r + [0.0] for r in rows],
            "timestamp": [k / 30 + 5.0 for k in range(n)],
            "frame_index": list(range(n)),
            "observation.ee_force": [[0.0, 0.0, -0.1 * k] for k in range(n)],
        }
    )
    pq.write_table(tab, root / "data" / "chunk-000" / "episode_000000.parquet")
    lg = from_lerobot(
        root / "data" / "chunk-000" / "episode_000000.parquet",
        labels,
        roles=("right",),
        q_home=q0,
        meta={"robot": "panda", "scene": w.spec},
    )
    j1 = labels.index(arm[0])
    assert (
        len(lg) == n
        and lg.q.shape == (n, len(labels))
        and lg.t[0] == 0.0
        and lg.ee_force[3, 0, 2] == pytest.approx(-0.3)
    )
    assert lg.q_des[2, j1] == pytest.approx(q0[j1] + 0.004) and np.isnan(lg.q[:, labels.index("finger_joint1")]).all()
    r = sysid.PlantSim("panda", w.spec).rollout(lg, {})  # unrecorded fingers start at their command
    assert r["ok"] and np.isfinite(sysid.errors(lg, r)["q"])
