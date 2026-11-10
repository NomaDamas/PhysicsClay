"""Replay one intent as many randomized episodes, headlessly, into one LeRobot dataset collection (v1 P1, pipeline A of
docs/ROADMAP_DATA.md).

    .venv/bin/python scripts/generate.py --seed seed.json --robots panda,ur5e --scenes table,kitchen --n 20 \\
        --randomize objects,lighting,camera --name cup_next_to_bowl

The seed is a task or a trajectory:
    task        {"action": "pick_place", "object": "red_cup", "target": "next to the bowl",
                 "language": ["put the red cup next to the bowl", "place the red cup beside the bowl"],   (optional)
                 "scene": {"objects": [{"name": "red_cup", "kind": "cup", "color": "red"}, ...]}}            (optional)
    trajectory  {"points": [[x, y, z] | {"p": [x, y, z]}, ...], "frame": "world" | "surface" | "object:<name>",
                 "role": "right", "label": "trace a wave over the table"}
                 ("surface": z is above the surface top; "object:<name>": points relative to that object)
--task '{"action": ...}' gives the seed inline.

It starts the server in-process-free mode as a subprocess (`python -m pps.app --fake`, offline: no camera, no LLM;
port --port, default 8907) or uses a running one (--server http://127.0.0.1:8905), switches robot and scene per
episode ({type: "world"}), records with {type: "dataset"} and runs the task over the same protocol as the UI, so every
check the robot makes (reach, collisions, zones, safety, physics) applies. Output: datasets/<name>/<robot>/ (one
LeRobot v2.1 dataset per robot body; scenes and variations are per-episode metadata), then a success table.

--scenes also takes the generated worlds (kitchen_full, warehouse, lab, ...: docs/worlds.md): every episode then gets a
new layout (a fresh world seed, kept in the episode's metadata) with the seed's objects added on its work surface.

Randomization (--randomize, comma list):
    objects    object positions anywhere in the reachable band of the surface, sizes x0.85-1.15, distractor colours
               (an object named with a colour in the seed keeps it: the words must stay true), 0-2 distractors
    lighting   light directions and intensity of the dataset cameras        (strength --lighting, default 0.6)
    camera     camera position / angle jitter of the dataset cameras        (metres --camera-jitter, default 0.02)
    physics    {type: "physics", op: "set", params: {friction_scale, mass_scale, damping_scale, kp_scale, latency_ms}} per
               episode when the server has P0 physics (state.physics); skipped otherwise. Drawn by P6's shared
               randomizer (pps/calib/randomize.py, the robot's ranges, seeded); the ranges go into each episode's metadata
"""
import argparse
import asyncio
import json
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from pps.calib.randomize import Randomizer  # noqa: E402
from pps.world import profiles  # noqa: E402
from pps.world import scene as world_scene

DISTRACTORS = ("block", "can", "fruit", "box", "bottle")
COLORS = ("red", "green", "blue", "yellow", "orange", "purple", "white", "pink")


# ------------------------------------------------------------------ seeds
def load_seed(a):
    if a.task:
        seed = json.loads(a.task)
    elif a.seed:
        seed = json.loads(Path(a.seed).read_text())
    else:
        seed = {"action": "pick_place", "object": "red_cup", "target": "next to the bowl"}
    if not isinstance(seed, dict):
        raise SystemExit("the seed must be a JSON object (a task or a trajectory)")
    if "traj" in seed and isinstance(seed["traj"], dict):
        seed = {**seed["traj"], **{k: v for k, v in seed.items() if k != "traj"}}
    kind = "traj" if seed.get("points") else "task"
    if kind == "task" and not seed.get("action"):
        raise SystemExit("a task seed needs an action (pick_place, reach, ...)")
    return kind, seed


def _obj_spec(word):
    """'red_cup' / 'the red cup' -> {"name", "kind", "color"?} with the colour only when the words name one."""
    w = re.sub(r"^(the|a|an)\s+", "", str(word or "").strip().lower()).replace(" ", "_")
    o = {"name": w, "kind": world_scene._kind({"name": w})}
    toks = w.split("_")
    col = next((c for c in world_scene.COLORS if c in toks), None)
    if col:
        o["color"] = col
    return o


def base_objects(kind, seed):
    """The seed's scene objects, or the ones its words name (the object to move + the reference in the target)."""
    if isinstance(seed.get("scene"), dict) and seed["scene"].get("objects"):
        return [dict(o, _named=True) for o in seed["scene"]["objects"]]
    if kind == "traj":
        f = str(seed.get("frame") or "")
        return [dict(_obj_spec(f.split(":", 1)[1]), _named=True)] if f.startswith("object:") else []
    out = []
    if seed.get("object"):
        out.append(dict(_obj_spec(seed["object"]), _named=True))
    tgt = str(seed.get("target") or "").lower()
    for k in sorted(list(world_scene.KINDS) + list(world_scene.KIND_ALIASES), key=len, reverse=True):
        if re.search(rf"\b{k}\b", tgt) and not any(o["kind"] == world_scene.KIND_ALIASES.get(k, k) for o in out[1:]):
            m = re.search(rf"\b(?:the\s+)?((?:\w+\s+)?{k})\b", tgt)
            words = m.group(1) if m else k
            o = _obj_spec(words if any(c in words.split() for c in world_scene.COLORS) else k)
            out.append(dict(o, movable=False, _named=True))
            break
    return out


def randomize_scene(template, objs, rng, rand_objects):
    """-> scene spec. Named objects keep their names (and named colours); positions spread over the reachable band."""
    s = world_scene.surface(template)
    near = s["x"][0]
    out, taken = [], []
    pool = [dict(o) for o in objs]
    if rand_objects:
        for _ in range(int(rng.integers(0, 3))):
            k = str(rng.choice([d for d in DISTRACTORS if d not in {world_scene._kind(o) for o in objs}]))
            pool.append({"name": f"{k}_{len(pool)}", "kind": k, "color": str(rng.choice(COLORS)), "movable": True})
    for o in pool:
        o = {k: v for k, v in o.items()}
        named = o.pop("_named", False)
        kind = world_scene._kind(o)
        if rand_objects:
            base = world_scene.KINDS[kind][0]
            sc = float(rng.uniform(0.85, 1.15))
            o["size"] = [round(base[0] * sc, 4), round(base[1] * sc, 4), round(base[2] * float(rng.uniform(0.9, 1.1)), 4)]
            if not o.get("color") or not named:
                o["color"] = str(rng.choice(COLORS))
            r = max(o["size"][0], o["size"][1]) / 2
            for _ in range(60):
                xy = [float(rng.uniform(near + 0.07, near + 0.21)), float(rng.uniform(-0.28, 0.28))]
                if all(np.hypot(xy[0] - p[0], xy[1] - p[1]) > r + q + 0.04 for p, q in taken):
                    break
            o["pos"] = [round(xy[0], 3), round(xy[1], 3)]
            taken.append((xy, r))
        out.append(o)
    return {"template": template, "objects": out}


def traj_points(seed, spec, top):
    """Trajectory seed -> world points for this scene (surface- or object-relative frames follow the scene)."""
    pts = [p.get("p") if isinstance(p, dict) else p for p in seed["points"]]
    pts = [np.asarray(p, float) for p in pts if p is not None and len(p) == 3]
    f = str(seed.get("frame") or "world")
    if f == "surface":
        return [p + np.array([0, 0, top]) for p in pts]
    if f.startswith("object:"):
        name = world_scene._slug(f.split(":", 1)[1])
        o = next((o for o in spec["objects"] if world_scene._slug(o["name"]) == name), None)
        if o is not None and o.get("pos"):
            c = np.array([o["pos"][0], o["pos"][1], top])
            return [p + c for p in pts]
    return pts


# ------------------------------------------------------------------ client
class Client:
    def __init__(self, base):
        self.base, self.st, self.ws, self.s = base, {}, None, None

    async def open(self, wait=120):
        import aiohttp
        self.s = aiohttp.ClientSession()
        for _ in range(int(wait * 2)):
            try:
                async with self.s.get(self.base + "/") as r:
                    if r.status == 200:
                        break
            except Exception:
                await asyncio.sleep(0.5)
        self.ws = await self.s.ws_connect(self.base + "/ws", heartbeat=20)

    async def close(self):
        if self.ws is not None:
            await self.ws.close()
        if self.s is not None:
            await self.s.close()

    async def send(self, **msg):
        await self.ws.send_str(json.dumps(msg))

    async def pump(self, sec):
        import aiohttp
        t0 = time.time()
        while time.time() - t0 < sec:
            m = await asyncio.wait_for(self.ws.receive(), 15)
            if m.type == aiohttp.WSMsgType.TEXT:
                self.st = json.loads(m.data)
            elif m.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                raise ConnectionError("server closed the connection")
        return self.st

    async def until(self, pred, limit, step=0.2):
        t0 = time.time()
        while time.time() - t0 < limit:
            await self.pump(step)
            try:
                v = pred(self.st)
            except Exception:
                v = None
            if v:
                return v
        return None

    def ds(self):
        return self.st.get("dataset") or {}


# ------------------------------------------------------------------ run
async def generate(a, kind, seed):
    rng = np.random.default_rng(a.rng_seed)
    rand = {x.strip() for x in a.randomize.split(",") if x.strip()}
    robots = [r.strip() for r in a.robots.split(",") if r.strip()]
    scenes = [s.strip() for s in a.scenes.split(",") if s.strip()]
    for r in robots:
        if profiles.get(r) is None:
            raise SystemExit(f"unknown robot '{r}'. Known: {', '.join(profiles.PROFILES)}")
    for s in scenes:
        if s not in world_scene.TEMPLATES and not world_scene.is_room(s):
            raise SystemExit(f"unknown scene '{s}'. Known: {', '.join(t['id'] for t in world_scene.templates())}")
    language = seed.get("language") or seed.get("utterance") or seed.get("label")
    language = [language] if isinstance(language, str) else list(language or [])
    objs = base_objects(kind, seed)
    c = Client(a.server or f"http://127.0.0.1:{a.port}")
    await c.open()
    tally = defaultdict(lambda: {"tries": 0, "episodes": 0, "success": 0, "refused": 0, "errors": 0})
    t_start = time.time()
    try:
        await c.pump(1.0)
        has_physics = "physics" in c.st
        if "physics" in rand and not has_physics:
            print("note: this server has no P0 physics (state.physics); physics randomization is skipped")
        await c.send(type="dataset", op="start", name=a.name, auto=True, fps=a.fps, size=a.size, id="gen-start")
        ok = await c.until(lambda s: (s.get("dataset") or {}).get("recording"), 10)
        if not ok:
            raise SystemExit(f"could not start recording: {(c.ds().get('reply') or {}).get('message')}")
        rnds = {}                                         # physics: P6's randomizer per robot (its ranges), seeded from --rng-seed
        rspec = {}
        if "lighting" in rand:
            rspec["lighting"] = a.lighting
        if "camera" in rand:
            rspec["camera"] = a.camera_jitter
        await c.send(type="dataset", op="randomize", seed=int(rng.integers(1 << 30)), id="gen-rand", **rspec)
        k = 0
        for robot in robots:
            for template in scenes:
                for i in range(a.n):
                    k += 1
                    key = (robot, template)
                    tally[key]["tries"] += 1
                    if world_scene.is_room(template):    # a generated home / site (docs/worlds.md): a new layout per episode
                        spec = {"template": template, "seed": int(rng.integers(1_000_000)),
                                "objects": [{k2: v for k2, v in o.items() if k2 not in ("pos", "_named")} for o in objs]}
                    else:
                        spec = randomize_scene(template, objs, rng, "objects" in rand)
                    wid = f"gen-w{k}"
                    await c.send(type="world", op="set", robot=robot, scene=spec, id=wid)
                    w = await c.until(lambda s: (s.get("task") or {}).get("id") == wid and s["task"]["status"] in ("done", "failed")
                                      and s["task"], 60, 0.3)
                    if not w or w["status"] != "done":
                        tally[key]["errors"] += 1
                        print(f"  [{k}] {robot}/{template}: world build failed: {(w or {}).get('message', 'timeout')}")
                        continue
                    world = c.st.get("world") or {}
                    spec_now = world.get("scene") or spec
                    top = float((world.get("surface") or {}).get("top", world_scene.surface(template)["top"]))
                    variation = {"generator": "scripts/generate.py", "seed": a.seed or "inline", "i": i, "robot": robot,
                                 "scene": template, "randomize": sorted(rand)}
                    if spec.get("seed") is not None:
                        variation["world_seed"] = spec["seed"]
                    if "physics" in rand and has_physics:
                        if robot not in rnds:
                            rnds[robot] = Randomizer(robot=robot, seed=int(rng.integers(1 << 30)))
                        rnd = rnds[robot]
                        params = rnd.physics(rnd.sample())
                        await c.send(**rnd.message(params, f"gen-p{k}"))
                        variation["physics"] = params
                        variation["physics_ranges"] = rnd.ranges()
                    await c.send(type="dataset", op="meta", data=variation, id=f"gen-m{k}")
                    words = str(rng.choice(language)) if language else None
                    n_before = c.ds().get("episodes", 0)
                    tid = f"gen{k}-{robot}-{template}"
                    if kind == "task":
                        msg = {kk: v for kk, v in seed.items() if kk not in ("language", "scene", "utterance", "label", "type", "id")}
                        msg.update(type="task", id=tid)
                        if words:
                            msg["utterance"] = words
                        await c.send(**msg)
                    else:
                        pts = traj_points(seed, spec_now, top)
                        await c.send(type="path3d", points=[[round(float(x), 4) for x in p] for p in pts],
                                     hand=seed.get("role") if seed.get("role") in ("left", "right") else "auto")
                        await c.pump(0.6)
                        await c.send(type="task", id=tid, action="follow_path", utterance=words or "follow the path")
                    t = await c.until(lambda s: (s.get("task") or {}).get("id") == tid and s["task"]["status"] not in ("planning",)
                                      and s["task"], 10)
                    if t and t["status"] == "running":
                        t = await c.until(lambda s: (s.get("task") or {}).get("id") != tid or s["task"]["status"] != "running",
                                          a.timeout, 0.3)
                        if t is None:
                            await c.send(type="task", id=f"{tid}-stop", action="stop")
                        got = await c.until(lambda s: (s.get("dataset") or {}).get("episodes", 0) > n_before, 8)
                        last = c.ds().get("last") or {}
                        if got:
                            tally[key]["episodes"] += 1
                            tally[key]["success"] += int(bool(last.get("success")))
                        print(f"  [{k}] {robot}/{template} #{i}: {'ok  ' if last.get('success') else 'FAIL'} "
                              f"{last.get('seconds', 0):5.1f} s  {last.get('task', '')[:60]}"
                              + ("" if last.get("success") else f"  ({(c.st.get('task') or {}).get('message', '')[:70]})"), flush=True)
                    else:
                        tally[key]["refused"] += 1
                        print(f"  [{k}] {robot}/{template} #{i}: refused  {((t or {}).get('message') or 'no answer')[:90]}", flush=True)
        await c.until(lambda s: (s.get("dataset") or {}).get("pending") == 0, 120, 0.5)
        roots = c.ds().get("datasets") or []
        await c.send(type="dataset", op="stop", id="gen-stop")
        await c.until(lambda s: not (s.get("dataset") or {}).get("recording"), 10)
        await c.pump(1.0)
        cost = c.ds().get("cost_ms") or {}
    finally:
        await c.close()
    dur = time.time() - t_start
    print(f"\nDataset '{a.name}' in {dur / 60:.1f} min · render {cost.get('render')} ms per sample (dataset thread)")
    print(f"{'robot':10s} {'scene':9s} {'tries':>5s} {'episodes':>8s} {'success':>8s} {'rate':>6s} {'refused':>7s} {'errors':>6s}")
    total = {"tries": 0, "episodes": 0, "success": 0, "refused": 0, "errors": 0}
    for (robot, template), v in tally.items():
        rate = v["success"] / v["tries"] if v["tries"] else 0.0
        print(f"{robot:10s} {template:9s} {v['tries']:5d} {v['episodes']:8d} {v['success']:8d} {rate:6.0%} {v['refused']:7d} {v['errors']:6d}")
        for kk in total:
            total[kk] += v[kk]
    rate = total["success"] / total["tries"] if total["tries"] else 0.0
    print(f"{'all':10s} {'':9s} {total['tries']:5d} {total['episodes']:8d} {total['success']:8d} {rate:6.0%} {total['refused']:7d} {total['errors']:6d}")
    print("success rate = successful episodes / attempts (refusals and failed builds count as attempts)")
    for r in roots:
        print("  ", r)
    return total


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", help="task or trajectory JSON file")
    ap.add_argument("--task", help="the seed inline, e.g. '{\"action\": \"pick_place\", \"object\": \"red_cup\", \"target\": \"next to the bowl\"}'")
    ap.add_argument("--robots", default="panda")
    ap.add_argument("--scenes", default="table")
    ap.add_argument("--n", type=int, default=5, help="episodes per robot x scene")
    ap.add_argument("--randomize", default="objects,lighting,camera", help="objects,lighting,camera,physics")
    ap.add_argument("--lighting", type=float, default=0.6, help="lighting randomization strength 0..1")
    ap.add_argument("--camera-jitter", type=float, default=0.02, help="camera position jitter, metres")
    ap.add_argument("--name", default=None, help="dataset name (datasets/<name>/<robot>/)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--size", type=int, default=256)
    ap.add_argument("--timeout", type=float, default=60.0, help="seconds per episode before it is stopped")
    ap.add_argument("--rng-seed", type=int, default=0)
    ap.add_argument("--port", type=int, default=8907)
    ap.add_argument("--server", help="use a running server (http://host:port) instead of starting one")
    a = ap.parse_args()
    a.name = a.name or time.strftime("gen_%Y%m%d_%H%M%S")
    kind, seed = load_seed(a)
    srv = None
    if not a.server:
        env = dict(os.environ, PC_BYOK="1")         # offline: never call a model with the server's keys
        srv = subprocess.Popen([str(ROOT / ".venv/bin/python"), "-m", "pps.app", "--fake", "--port", str(a.port),
                                "--https-port", str(a.port + 1)], cwd=ROOT, env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        total = asyncio.run(generate(a, kind, seed))
    finally:
        if srv is not None:
            srv.terminate()
            try:
                srv.wait(10)
            except subprocess.TimeoutExpired:
                srv.kill()
    sys.exit(0 if total["episodes"] else 1)


if __name__ == "__main__":
    main()
