"""The generated-world scenarios per robot body (docs/worlds.md "What works per robot"): physics on, no server.

Each scenario builds its world (template + seed), runs the same task actions the server would (`open`, `close`,
`pick_from`, `place_in`, in order, each to "done" or "failed") with the planner and the plant stepping in sim time,
and checks the physical outcome after every step (how far a drawer opened, which container an object is in):

    kitchen   open the top drawer (>= 60 %), take the spoon, put it in the sink, close the drawer (< 10 %)
    lower     open the second drawer (>= 60 %) and close it again (a knob right under another one: a front grasp)
    bottom    open the bottom drawer (>= 60 %) and close it again
    dishwasher open the dishwasher door (>= 60 %; bottom-hinged) and close it
    warehouse pick the box out of the tote and put it in the empty tote
    assembly  a part rides the conveyor to the photo-eye, the arm picks it into the finished-parts bin
    lab       a tube from the yellow source rack into a free slot of the tube rack
    tending   open the CNC door, the part from the vise into the finished tray, close the door

    .venv/bin/python scripts/bench_worlds.py panda ur5e              # every scenario, seed 1
    .venv/bin/python scripts/bench_worlds.py ur5e -c kitchen -s 1 -s 2 -s 3 -v
    .venv/bin/python scripts/bench_worlds.py -j 3 panda ur5e xarm7   # robots in parallel (processes)

Prints a markdown table (robot x scenario: the seeds that passed, and the first failure's message).
"""
import argparse
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("PC_PHYSICS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")   # small IK matrices: BLAS threads only thrash a busy machine
os.environ.setdefault("OMP_NUM_THREADS", "1")

DT = 1 / 120                     # one IK step of the server loop (4 per 30 fps frame)
STEP_LIMIT = 90.0                # sim seconds per step

# scenario -> (template, [(action, args, check)]); check(env) -> why it isn't right, or None
SCENARIOS = {
    "kitchen": ("kitchen_full", [
        ("open", {"fixture": "the top drawer", "amount": 0.8}, lambda e: e.opened("drawer_1", 0.6)),
        ("pick_from", {"container": "drawer_1", "object": "spoon"}, lambda e: e.holds("spoon")),
        ("place_in", {"object": "spoon", "container": "sink"}, lambda e: e.inside("spoon", "sink")),
        ("close", {"fixture": "drawer_1"}, lambda e: e.closed("drawer_1"))]),
    "lower": ("kitchen_full", [
        ("open", {"fixture": "the second drawer", "amount": 0.8}, lambda e: e.opened("drawer_2", 0.6)),
        ("close", {"fixture": "drawer_2"}, lambda e: e.closed("drawer_2"))]),
    "bottom": ("kitchen_full", [
        ("open", {"fixture": "the bottom drawer", "amount": 0.8}, lambda e: e.opened("drawer_3", 0.6)),
        ("close", {"fixture": "drawer_3"}, lambda e: e.closed("drawer_3"))]),
    "dishwasher": ("kitchen_full", [
        ("open", {"fixture": "the dishwasher door", "amount": 0.8}, lambda e: e.opened("dishwasher_door", 0.6)),
        ("close", {"fixture": "dishwasher_door"}, lambda e: e.closed("dishwasher_door"))]),
    "warehouse": ("warehouse", [
        ("pick_from", {"container": "tote", "object": "box"}, lambda e: e.holds("box")),
        ("place_in", {"object": "box", "container": "the empty tote"}, lambda e: e.inside("box", "tote_out"))]),
    "assembly": ("assembly_line", [
        ("wait_part", {}, None),
        ("place_in", {"object": "$part", "container": "out_bin"}, lambda e: e.inside(e.vars["part"], "out_bin"))]),
    "lab": ("lab", [
        ("place_in", {"object": "$tube", "container": "the tube rack"}, lambda e: e.inside(e.vars["tube"], "rack_"))]),
    "tending": ("machine_tending", [
        ("open", {"fixture": "the CNC door", "amount": 0.8}, lambda e: e.opened("cnc_door", 0.6)),
        ("place_in", {"object": "part_in_vise", "container": "done_tray"}, lambda e: e.inside("part_in_vise", "done_tray")),
        ("close", {"fixture": "cnc_door"}, lambda e: e.closed("cnc_door"))]),
}


class Env:
    def __init__(self, rt, tk):
        self.rt, self.tk, self.vars = rt, tk, {}

    def part(self, name):
        return self.tk.fx.by_name[name]

    def opened(self, name, need):
        f = self.tk.fx.frac(self.part(name))
        return None if f >= need else f"{name} only {f * 100:.0f}% open"

    def closed(self, name):
        f = self.tk.fx.frac(self.part(name))
        return None if f < 0.10 else f"{name} still {f * 100:.0f}% open"

    def holds(self, obj):
        return None if obj in self.tk.holding.values() else f"not holding {obj} ({self.tk.holding})"

    def inside(self, obj, cont):
        c = self.tk.container_of(obj)
        name = (c or {}).get("name", "")
        ok = name.startswith(cont) if cont.endswith("_") else name == cont
        return None if ok else f"{obj} in {name or 'nothing'}, not {cont}"


def run_steps(rt, tk, steps, verbose=False, tag="", env_vars=None):
    """-> (ok, step index, message, seconds)"""
    pl = rt.plant
    env = Env(rt, tk)
    env.vars.update(env_vars or {})
    t_start = pl.d.time
    for i, (action, args, check) in enumerate(steps):
        args = {k: (env.vars.get(v[1:], v[1:]) if isinstance(v, str) and v.startswith("$") else v) for k, v in args.items()}
        t0 = pl.d.time
        if action == "wait_part":                 # the conveyor brings a part to the photo-eye
            belt = tk.fx.belts[0]
            while belt.get("held") is None and pl.d.time - t0 < 60:
                rt.step(DT)
                tk.tick()
            if belt.get("held") is None:
                return False, i, "no part reached the photo-eye in 60 s", pl.d.time - t_start
            env.vars["part"] = belt["held"]
            continue
        tid = f"w{i}"
        if action in ("open", "close"):
            tk.operate(tid, action, args.get("fixture", ""), args.get("amount"))
        elif action == "pick_from":
            tk.pick_from(tid, args.get("container", ""), args.get("object"))
        elif action == "place_in":
            tk.place_in(tid, args.get("object"), args.get("container", ""))
        while tk.task and tk.task.get("status") in ("running", "planning") and pl.d.time - t0 < STEP_LIMIT:
            rt.step(DT)
            tk.tick()
            if tk.search is not None:
                tk.search_tick(0.05)
        st = dict(tk.task or {})
        for _ in range(int(0.5 / DT)):            # what it did has to stay done
            rt.step(DT)
            tk.tick()
        msg = st.get("message", "")
        if verbose:
            print(f"    {tag} step {i + 1} {action} {args}: {st.get('status')} after {pl.d.time - t0:.1f} s: {msg[:160]}", flush=True)
        if st.get("status") == "running":
            return False, i, f"timed out after {STEP_LIMIT:.0f} s ({action})", pl.d.time - t_start
        if st.get("status") != "done":
            return False, i, msg, pl.d.time - t_start
        why = check(env) if check else None
        if why:
            return False, i, f"{action} said done ({msg[:80]}) but {why}", pl.d.time - t_start
    return True, len(steps), "", pl.d.time - t_start


def bench(rid, scenarios, seeds, verbose):
    from pps import tasks as T
    from pps.world import rooms, scene
    from pps.world.runtime import make_runtime
    out = []
    for sc in scenarios:
        tpl, steps = SCENARIOS[sc]
        if rid not in rooms.ROOMS[tpl]["robots"]:
            out.append({"robot": rid, "scenario": sc, "seed": None, "ok": None, "why": "doesn't fit this world"})
            continue
        for seed in seeds:
            t0 = time.time()
            try:
                w = scene.build(rid, {"template": tpl, "seed": seed}, physics=True)
                T.apply_surface(w.surface, w.regions)
                rt = make_runtime(w)
                tk = T.Tasks(rt, w.kinds)
                for _ in range(int(0.5 / DT)):
                    rt.step(DT)
                    tk.tick()
                ev = {}
                if sc == "lab":                  # the first tube in the source rack
                    ev["tube"] = next((n for n in tk.objs if str((tk.container_of(n) or {}).get("name", "")).startswith("source_rack")), "tube_1")
                ok, i, why, sim = run_steps(rt, tk, steps, verbose, f"{rid}/{sc}/{seed}", ev)
            except Exception as e:                # a crash is a failed run, reported
                import traceback
                traceback.print_exc()
                ok, i, why, sim = False, 0, f"error: {e}", 0.0
            r = {"robot": rid, "scenario": sc, "seed": seed, "ok": ok, "step": i, "why": why, "sim": sim, "wall": time.time() - t0}
            if verbose:
                print(f"  {rid} {sc} seed {seed}: {'ok' if ok else 'NO'} ({sim:.0f} s sim, {r['wall']:.0f} s wall) {why[:160]}", flush=True)
            out.append(r)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("robots", nargs="*", default=["panda", "ur5e"])
    ap.add_argument("-c", "--scenario", action="append", choices=list(SCENARIOS))
    ap.add_argument("-s", "--seed", action="append", type=int)
    ap.add_argument("-j", "--jobs", type=int, default=1)
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--json", action="store_true", help="also print one line 'WORLDS_JSON [...]'")
    a = ap.parse_args()
    scs = a.scenario or list(SCENARIOS)
    seeds = a.seed or [1]
    t0 = time.time()
    if a.jobs > 1 and len(a.robots) > 1:
        with ProcessPoolExecutor(max_workers=a.jobs) as ex:
            res = sum(ex.map(bench, a.robots, [scs] * len(a.robots), [seeds] * len(a.robots), [a.verbose] * len(a.robots)), [])
    else:
        res = sum((bench(r, scs, seeds, a.verbose) for r in a.robots), [])
    print()
    print("| Scenario | " + " | ".join(a.robots) + " |")
    print("|---|" + "---|" * len(a.robots))
    for sc in scs:
        cells = []
        for rid in a.robots:
            rs = [r for r in res if r["robot"] == rid and r["scenario"] == sc]
            if rs and rs[0]["ok"] is None:
                cells.append("doesn't fit")
                continue
            good = [str(r["seed"]) for r in rs if r["ok"]]
            bad = [r for r in rs if not r["ok"]]
            c = f"{len(good)}/{len(rs)}" + (f" (seeds {','.join(good)})" if good and bad else "")
            if bad:
                c += "; " + "; ".join(f"s{r['seed']} step {r['step'] + 1}: {r['why'][:90]}" for r in bad)
            cells.append(c)
        print(f"| {sc} | " + " | ".join(cells) + " |")
    print(f"\n{len(a.robots)} robots, seeds {seeds}, {time.time() - t0:.0f} s wall")
    if a.json:
        import json
        print("WORLDS_JSON " + json.dumps(res), flush=True)
    sys.exit(0 if all(r["ok"] is not False for r in res) else 1)


if __name__ == "__main__":
    main()
