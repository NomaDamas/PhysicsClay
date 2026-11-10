"""A fast subset of scripts/bench_grasp.py (v1 P0c): real contact pick & place, mug -> center and red block -> center,
two jittered trials each, on the bodies whose grippers needed per-model work (Stretch 3's spring-loaded parallel pads,
Spot's claw, TidyBot's 2F-85 with the whole arm kept clear) plus the Panda as a reference. The bench runs in its own
process (no server, sim time: the verdicts are the same on a loaded machine) while the self test keeps reading frames.

Runs inside scripts/selftest.py (async def run(t)); with physics off it only checks that it is skipped honestly."""
import asyncio
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ROBOTS = ("stretch3", "spot", "tidybot", "panda")
TRIALS = 2


async def run(t):
    if os.environ.get("PC_PHYSICS", "1") == "0" or ((t.state().get("physics") or {}).get("mode")) != "dynamic":
        return                                      # contact grasps need the plant
    env = dict(os.environ, PC_PHYSICS="1")
    proc = await asyncio.create_subprocess_exec(sys.executable, str(ROOT / "scripts" / "bench_grasp.py"), *ROBOTS, "-n", str(TRIALS),
                                                "-j", "2", "--json", cwd=str(ROOT), env=env, stdout=asyncio.subprocess.PIPE,
                                                stderr=asyncio.subprocess.STDOUT)
    out = []

    async def read():
        async for line in proc.stdout:
            out.append(line.decode(errors="replace"))
    reader = asyncio.ensure_future(read())
    while proc.returncode is None:                  # keep the websocket drained while the bench runs (a few minutes)
        await t.pump(1.0)
        try:
            await asyncio.wait_for(proc.wait(), 0.01)
        except asyncio.TimeoutError:
            pass
    await reader
    js = next((ln[len("BENCH_JSON "):] for ln in out if ln.startswith("BENCH_JSON ")), None)
    res = json.loads(js) if js else {}
    for rid in ROBOTS:
        r = res.get(rid) or {}
        cells = [f"{k} {v['ok']}/{v['n']}" + (f" ({v['err']:.1f} cm)" if v.get("err") is not None else "") + (f": {v['why'][:60]}" if v.get("why") else "")
                 for k, v in r.items()]
        t.check(f"grasp bench: {rid} picks and places the mug and the red block by contact ({TRIALS} trials each)",
                len(r) == 2 and all(v["ok"] == v["n"] for v in r.values()), "; ".join(cells) or "".join(out[-3:])[:160])
