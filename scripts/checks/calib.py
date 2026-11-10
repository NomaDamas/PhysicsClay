"""P6 calibration in the server (docs/calibration.md): the robot is given "today's" physics (a slippery table, a stiffer
arm, heavier objects), probes it (state.calib shows each probe), the twin is reset to nominal, the fit recovers what the
robot was given from the probe logs alone, `apply` puts the fit into state.physics.params, and `reset` restores the
defaults. Also: other tasks are refused while probing.

Runs inside scripts/selftest.py (async def run(t)). Needs physics (the default)."""
import os
import time

SCENE = {"template": "table", "objects": [{"name": "red_block", "kind": "block", "color": "red", "pos": [0.42, -0.15]},
                                          {"name": "mug", "kind": "mug", "color": "red", "pos": [0.40, 0.18]}]}
TODAY = {"friction_scale": 0.6, "kp_scale": 1.3, "mass_scale": 1.5}


async def _until(t, cond, limit):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(0.25)
        if cond(t.state()):
            return True
    return False


def _calib(t):
    return t.state().get("calib") or {}


def _params(t):
    return (t.state().get("physics") or {}).get("params") or {}


async def _world(t, **kw):
    rev = (t.state().get("world") or {}).get("rev", 0)
    await t.send(type="world", id=f"world-{rev}", **kw)
    await _until(t, lambda s: (s.get("world") or {}).get("rev", 0) > rev, 25.0)


async def _physics(t, **kw):
    tid = f"phys-{time.time():.3f}"
    await t.send(type="physics", id=tid, **kw)
    await _until(t, lambda s: (s.get("task") or {}).get("id") == tid, 3.0)


async def run(t):
    await t.pump(0.3)
    if os.environ.get("PC_PHYSICS", "1") == "0" or (t.state().get("physics") or {}).get("mode") != "dynamic":
        await t.send(type="calib", op="probe", id="calib-kin")
        await t.pump(0.5)
        t.check("calib: refused without physics", _calib(t).get("status") == "failed", _calib(t).get("message", "")[:60])
        return
    await _world(t, op="set", robot="panda", scene=SCENE)
    await _physics(t, op="set", params=TODAY)
    t.check("calib: state.calib is published (idle)", _calib(t).get("status") in ("idle", None) or "status" in _calib(t), str(_calib(t))[:80])

    # 1. probe the robot as it is today
    await t.send(type="calib", op="probe", id="calib-probe")
    seen = set()

    def probing(s):
        c = s.get("calib") or {}
        for p in c.get("probes") or []:
            if p.get("status") == "running":
                seen.add(p["name"])
        return c.get("status") not in ("probing",)
    await _until(t, lambda s: (s.get("calib") or {}).get("status") == "probing", 5)
    res = await t.task("pick_place", 1.5, object="mug", target="center")
    t.check("calib: other tasks wait while probing", res.get("status") == "failed" and "calibrat" in res.get("message", "").lower(),
            res.get("message", "")[:70])
    ok = await _until(t, probing, 150)
    c = _calib(t)
    logs = [lg["probe"] for lg in c.get("logs") or []]
    t.check("calib: probe routine runs as a task and records every probe", ok and c.get("status") == "probed"
            and logs == ["swing", "press", "lift", "slide"] and seen >= {"swing", "lift"},
            f"{c.get('status')}: {c.get('message', '')[:70]} logs={logs} seen={sorted(seen)}")
    if c.get("status") != "probed":
        await _physics(t, op="reset")
        await _world(t, op="reset")
        return

    # 2. the twin forgets (nominal physics), then fits to the probe logs alone
    await _physics(t, op="reset")
    t.check("calib: physics back to nominal before the fit", _params(t).get("friction_scale") == 1.0, str(_params(t))[:80])
    t0 = time.time()
    await t.send(type="calib", op="fit", id="calib-fit", budget="quick")
    ok = await _until(t, lambda s: (s.get("calib") or {}).get("status") in ("fitted", "failed"), 300)
    c = _calib(t)
    fp = c.get("params") or {}
    errs = {k: abs(fp.get(k, 0) - v) / v for k, v in TODAY.items()}
    t.check("calib: fit recovers the robot's friction, stiffness and mass (±20 %)", ok and c.get("status") == "fitted"
            and all(e < 0.2 for e in errs.values()) and (c.get("fit") or {}).get("err_after", 9) < (c.get("fit") or {}).get("err_before", 0),
            f"{c.get('status')} in {time.time() - t0:.0f}s: " + ", ".join(f"{k} {fp.get(k, 0):.2f}" for k in TODAY) +
            f" latency {fp.get('latency_ms', 0):.0f} ms; {c.get('message', '')[:50]}")
    t.check("calib: per-parameter report vs defaults", len(c.get("per_param") or []) >= 4
            and all({"name", "value", "default"} <= set(p) for p in c.get("per_param") or []), str((c.get("per_param") or [])[:1])[:80])

    # 3. apply -> state.physics.params; reset -> defaults
    await t.send(type="calib", op="apply", id="calib-apply")
    await _until(t, lambda s: (s.get("calib") or {}).get("applied"), 5)
    p = _params(t)
    t.check("calib: apply changes state.physics.params to the fit", _calib(t).get("applied") and fp
            and all(abs(p.get(k, 0) - v) < 1e-3 for k, v in fp.items()) and abs(p.get("friction_scale", 1) - 1.0) > 0.1,
            ", ".join(f"{k} {p.get(k, 0):.3g}" for k in fp))
    await t.send(type="calib", op="reset", id="calib-reset")
    await _until(t, lambda s: (s.get("calib") or {}).get("status") == "idle", 5)
    p = _params(t)
    t.check("calib: reset restores the default physics", _calib(t).get("status") == "idle" and not _calib(t).get("applied")
            and all(p.get(k) == v for k, v in {"friction_scale": 1.0, "mass_scale": 1.0, "kp_scale": 1.0, "latency_ms": 0.0}.items()),
            str({k: p.get(k) for k in ("friction_scale", "mass_scale", "kp_scale", "latency_ms")}))
    await _world(t, op="reset")
