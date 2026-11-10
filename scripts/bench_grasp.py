"""Contact pick & place benchmark per robot body (docs/V1.md P0): physics on, no server, no rendering.

For every robot: N trials of "mug -> center" and "red block -> center" in the default table scene. Each trial starts
from a scene reset with the object nudged a little (±1.5 cm, ±15° yaw, seeded) so the trials differ, runs the real
pick_place task (planner + plant, contact grasp) for up to 45 s of sim time, and scores it:

    ok    the task finished "done", the object rests upright on the surface, out of the hand, within 3 cm of the spot
    else  the task's own message (slip, out of reach, fell over, ...) or what the scorer saw

    .venv/bin/python scripts/bench_grasp.py                 # every robot, 10 trials each
    .venv/bin/python scripts/bench_grasp.py xarm7 spot -n 3 # some robots, 3 trials
    .venv/bin/python scripts/bench_grasp.py -j 3            # three robots at a time (processes)

Time is sim time (the plant's clock), so a loaded machine changes the wall time only. Prints a table (markdown).
"""
import argparse
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("PC_PHYSICS", "1")

TASKS = (("mug", "mug", "center"), ("block", "red block", "center"))
DT = 1 / 120                    # one IK step of the server loop (4 per 30 fps frame)
LIMIT = 45.0                    # sim seconds per trial
OK_CM = 3.0


def _jitter(tk, name, rng):
    import numpy as np
    o = tk.objs[name]
    p = o["home"].copy()
    p[:2] += rng.uniform(-0.015, 0.015, 2)
    yaw = math.radians(rng.uniform(-15, 15))
    q0 = o["home_q"]
    qz = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    import mujoco
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, qz, q0)
    tk.set_obj_pose(name, p, q)


def trial(rt, tk, obj, target, rng, verbose=False):
    import numpy as np
    tk.home("reset", reset_scene=True)
    name = tk.obj_name(obj)
    if name is None:
        return {"ok": False, "why": f"no {obj} in the scene"}
    _jitter(tk, name, rng)
    pl = rt.plant
    t0 = t_start = pl.d.time
    while pl.d.time - t0 < 0.5:              # let it settle where it was put
        rt.step(DT)
        tk.tick()
    tk.pick_place("bench", obj, target, None, None, 4 / 3)
    st = dict(tk.task)
    if st["status"] != "running":
        if verbose:
            print(f"    {rt.profile.id} {obj:10s} NO  (not started) {st['message'][:240]}")
        return {"ok": False, "why": st["message"], "t": 0.0}
    t0 = pl.d.time
    palm = rt.path["palm"]
    ref, drift, wob = None, 0.0, 0.0           # in-hand stability while carried: drift (mm) and turn (deg) vs the hand
    trace, last = os.environ.get("PC_BENCH_TRACE"), -1.0     # (debugging: a timeline of the trial)
    while tk.task["status"] == "running" and pl.d.time - t0 < LIMIT:
        rt.step(DT)
        tk.tick()
        if trace and pl.d.time - last >= float(trace):
            last = pl.d.time
            role = tk.role(palm)
            tcp = rt.data.site_xpos[rt.palm_sid[palm]]
            ta, tb = pl.touching(role, name)
            print(f"      t={pl.d.time - t0:5.2f} i={rt.path['i'] if rt.path else '-'} ph={(tk.ph or {}).get('phase')} "
                  f"cmd={pl.grip[role]['cmd']} ctrl={pl.grip[role].get('ctrl', 0):.3f} obj-tcp={np.round((tk.mug_center(name) - tcp) * 100, 1)} "
                  f"lag={np.linalg.norm(tcp - rt.data.mocap_pos[rt.hands_mid[palm]]) * 100:.1f}cm touch={int(ta)}{int(tb)} "
                  f"held={tk.holding.get(palm)} obj={np.round(tk.mug_center(name)[:2] * 100, 1)} oc={sorted({pl._label(x) for x in pl.object_contacts(name)})}", flush=True)
        h = tk.held.get(palm)
        if h is not None and tk.ph is None and pl.d.time - h["t0"] > 0.3:
            R = rt.data.site_xmat[rt.palm_sid[palm]].reshape(3, 3)
            rel = R.T @ (tk.mug_center(name) - rt.data.site_xpos[rt.palm_sid[palm]])
            Ro = R.T @ tk.obj_mat(name)
            if ref is None:
                ref = (rel, Ro)
            drift = max(drift, float(np.linalg.norm(rel - ref[0])) * 1000)
            wob = max(wob, math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(ref[1].T @ Ro) - 1) / 2)))))
    dur = pl.d.time - t0
    fin = dict(tk.task)
    for _ in range(int(0.5 / DT)):           # it has to stay where it was left
        rt.step(DT)
        tk.tick()
    dest = getattr(tk, "bench_dest", None)
    c = tk.mug_center(name)
    tilt = tk.tilt(name)
    held = any(v == name for v in pl.grasped().values())
    from pps import tasks as T
    on = abs((c[2] - tk.objs[name]["half_h"]) - T.TABLE_TOP) < 0.02
    err = None if dest is None else float(np.hypot(*(c[:2] - dest[:2]))) * 100
    res = {"t": dur, "err": err, "tilt": tilt, "status": fin["status"], "msg": fin["message"], "sim": pl.d.time - t_start,
           "drift": drift, "wobble": wob}
    if fin["status"] == "running":
        res.update(ok=False, why=f"timed out after {LIMIT:.0f} s ({fin.get('step') or 'no step'})")
    elif fin["status"] != "done":
        res.update(ok=False, why=fin["message"])
    elif held:
        res.update(ok=False, why="still in the gripper")
    elif tilt > 25 or not on:
        res.update(ok=False, why=f"not resting upright on the surface (tilt {tilt:.0f}°, z {c[2]:.3f})")
    elif err is None or err > OK_CM:
        res.update(ok=False, why=f"placed {err:.1f} cm from the spot")
    else:
        res.update(ok=True, why="")
    if verbose:
        print(f"    {rt.profile.id} {obj:10s} {'ok ' if res['ok'] else 'NO '} t={dur:5.1f}s err={err if err is None else round(err, 1)} cm "
              f"tilt={tilt:.0f}° in-hand drift={drift:.0f} mm turn={wob:.0f}° {res['why'][:110]}", flush=True)
    return res


def bench_robot(rid, n, seed, verbose, which):
    import numpy as np
    from pps import tasks as T
    from pps.world import scene
    from pps.world.runtime import make_runtime
    t0 = time.time()
    w = scene.build(rid, None, physics=True)
    T.apply_surface(w.surface, w.regions)
    rt = make_runtime(w)
    tk = T.Tasks(rt, w.kinds)
    _orig = tk.pick_place

    def pick_place(tid, obj, target, *a, **k):  # remember where the task meant to put it (after free-spot shifting)
        _orig(tid, obj, target, *a, **k)
        pa = getattr(tk, "place_at", None)
        tk.bench_dest = None if pa is None else np.asarray(pa[1], float)
    tk.pick_place = pick_place
    out = {"robot": rid, "label": w.profile.label, "build_s": time.time() - t0, "tasks": {}}
    wall0 = time.time()
    sim = 0.0
    for ti, (key, obj, target) in enumerate(TASKS):
        if which and key not in which:
            continue
        rng = np.random.default_rng(seed * 100 + ti)
        rs = []
        if verbose:
            print(f"  {rid}: {obj} -> {target}", flush=True)
        only = os.environ.get("PC_BENCH_ONLY")         # replay one trial (debugging): its nudge, nothing else
        for k in range(n):
            if only is not None and k != int(only):
                rng.uniform(-1, 1, 2), rng.uniform(-1, 1)
                continue
            try:
                rs.append(trial(rt, tk, obj, target, rng, verbose))
                sim += rs[-1].get("sim", 0.0)
            except Exception as e:  # a crash is a failed trial, reported
                import traceback
                traceback.print_exc()
                rs.append({"ok": False, "why": f"error: {e}"})
        out["tasks"][key] = rs
    out["rtf"] = sim / max(time.time() - wall0, 1e-6)
    return out


def _summary(rs):
    ok = sum(r["ok"] for r in rs)
    errs = [r["err"] for r in rs if r.get("ok") and r.get("err") is not None]
    whys = {}
    for r in rs:
        if not r["ok"]:
            k = r["why"].split(":")[0][:48] if r["why"].startswith(("It slipped", "I can't", "Too heavy")) else r["why"][:60]
            whys[k] = whys.get(k, 0) + 1
    why = "; ".join(f"{k} ×{v}" if v > 1 else k for k, v in sorted(whys.items(), key=lambda kv: -kv[1]))
    return ok, len(rs), (sum(errs) / len(errs) if errs else None), why


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("robots", nargs="*", help="robot ids (default: every robot but h2)")
    ap.add_argument("-n", "--trials", type=int, default=10)
    ap.add_argument("-s", "--seed", type=int, default=0)
    ap.add_argument("-j", "--jobs", type=int, default=1)
    ap.add_argument("-t", "--task", action="append", choices=[k for k, _, _ in TASKS], help="only this task (repeatable)")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--json", action="store_true", help="also print one line 'BENCH_JSON {...}' (scripts/checks/grasp_bench.py)")
    a = ap.parse_args()
    from pps.world import profiles
    ids = a.robots or [r for r in profiles.PROFILES if r != "h2"]
    t0 = time.time()
    if a.jobs > 1 and len(ids) > 1:
        with ProcessPoolExecutor(max_workers=a.jobs) as ex:
            res = list(ex.map(bench_robot, ids, [a.trials] * len(ids), [a.seed] * len(ids), [a.verbose] * len(ids), [a.task] * len(ids)))
    else:
        res = [bench_robot(r, a.trials, a.seed, a.verbose, a.task) for r in ids]
    keys = [k for k, _, _ in TASKS if not a.task or k in a.task]
    print()
    print("| Robot | " + " | ".join(f"{k} → center (ok / err cm)" for k in keys) + " | Status | Failures | Loop RTF |")
    print("|---|" + "---|" * len(keys) + "---|---|---|")
    bad, summary = 0, {}
    for r in res:
        p = profiles.PROFILES[r["robot"]]
        cells, fails, reliable = [], [], True
        for k in keys:
            ok, n, err, why = _summary(r["tasks"].get(k, []))
            reliable &= ok >= math.ceil(0.9 * n)
            cells.append(f"**{ok}/{n}**" + (f" ({err:.1f})" if err is not None else ""))
            summary.setdefault(r["robot"], {})[k] = {"ok": ok, "n": n, "err": err, "why": why}
            if why:
                fails.append(f"{k}: {why}")
        exp = bool((p.physics or {}).get("experimental"))
        status = "experimental" if exp else ("ok" if reliable else "**below 9/10**")
        bad += (not exp and not reliable)
        print(f"| {r['robot']} | " + " | ".join(cells) + f" | {status} | {'; '.join(fails)[:240]} | {r['rtf']:.1f} |")
    print(f"\n{len(ids)} robots, {a.trials} trials per task, seed {a.seed}, {time.time() - t0:.0f} s wall")
    if a.json:
        import json
        print("BENCH_JSON " + json.dumps(summary), flush=True)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
