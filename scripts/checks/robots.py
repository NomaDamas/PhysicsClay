"""Robot UX (UX 3): every robot has a system prompt (.md) and UI data (.json) with >= 3 examples and valid dock tools,
the browser can fetch that data over HTTP from the server, and the robot UI module is served.

Runs inside scripts/selftest.py (async def run(t)) or on its own against a running server:
    .venv/bin/python scripts/checks/robots.py [--base http://127.0.0.1:8921]      (files only when no server answers)"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROMPTS = ROOT / "pps" / "world" / "prompts"
IDS = ["g1", "panda", "ur5e", "xarm7", "iiwa14", "aloha", "apollo", "stretch3", "tidybot", "spot", "h2"]
KINDS = {"humanoid", "arm", "bimanual", "mobile_manipulator", "quadruped_arm"}
TOOLS = {"point", "zone", "goal", "show", "path", "handover", "drive"}
BASE_RULES = ["observe", "plan", "which one", "never invent", "language", "stop"]


def _load(i):
    try:
        return json.loads((PROMPTS / f"{i}.json").read_text())
    except (OSError, ValueError):
        return None


def file_checks(check):
    base = (PROMPTS / "base.md").read_text().lower() if (PROMPTS / "base.md").exists() else ""
    missing = [r for r in BASE_RULES if r not in base]
    check("robots: base.md has the shared rules", base and not missing, f"missing: {missing}" if missing else f"{len(base)} chars")
    bad = []
    for i in IDS:
        md = PROMPTS / f"{i}.md"
        j = _load(i)
        why = []
        if not md.exists() or len(md.read_text()) < 600:
            why.append("md missing/short")
        if not isinstance(j, dict):
            why.append("json missing/invalid")
        else:
            ex = j.get("examples")
            if not (isinstance(ex, list) and len(ex) >= 3 and all(isinstance(e, str) and e.strip() for e in ex)):
                why.append("examples < 3")
            tools = j.get("tools")
            if not (isinstance(tools, list) and tools and set(tools) <= TOOLS):
                why.append(f"tools {tools}")
            if not (j.get("label") and j.get("tagline") and j.get("kind") in KINDS):
                why.append("label/tagline/kind")
            if "handover" in (tools or []) and j.get("kind") != "bimanual":
                why.append("handover on a non-bimanual robot")
        if why:
            bad.append(f"{i}: {', '.join(why)}")
    check("robots: every id has .md + .json (>=3 examples, valid tools)", not bad, "; ".join(bad) or f"{len(IDS)} robots")
    want = {i: _load(i) for i in IDS}
    cat = ROOT / "web" / "robots" / "catalog.json"
    try:
        local = json.loads(cat.read_text())
    except (OSError, ValueError):
        local = None
    stale = [i for i in IDS if not local or local.get(i) != want[i]]
    check("robots: web/robots/catalog.json matches pps/world/prompts", not stale,
          f"stale: {stale} (run scripts/sync_robot_prompts.py)" if stale else "in sync")
    return want


async def run(t):
    want = file_checks(t.check)
    async with t.http.get(t.base + "/static/robots/catalog.json") as r:
        cat = await r.json(content_type=None) if r.status == 200 else None
    served = [i for i in IDS if cat and cat.get(i) == want[i]]
    t.check("robots: per-robot JSON reachable over HTTP", len(served) == len(IDS), f"{len(served)}/{len(IDS)} served from /static/robots/catalog.json")
    async with t.http.get(t.base + "/ui/modules.json") as r:
        mods = await r.json() if r.status == 200 else []
    t.check("robots: robot UI module is loaded", "/static/ui/robots.js" in mods, str(mods)[:120])


if __name__ == "__main__":
    import asyncio
    import sys
    from types import SimpleNamespace

    fails = []

    def check(name, ok, detail=""):
        print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)
        if not ok:
            fails.append(name)

    async def main():
        import aiohttp
        base = sys.argv[sys.argv.index("--base") + 1] if "--base" in sys.argv else None
        if not base:
            file_checks(check)
            return
        async with aiohttp.ClientSession() as s:
            await run(SimpleNamespace(check=check, http=s, base=base))

    asyncio.run(main())
    sys.exit(1 if fails else 0)
