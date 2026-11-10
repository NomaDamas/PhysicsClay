"""Demo packs (docs/packs.md): a ready scene for one story, plus what the robot should know about it.

A pack is `pps/world/packs/<id>.json` (+ an optional `<id>.md` appended to the agent's prompt while the pack is loaded):
```
{"id", "label", "summary", "robot": default body, "robots": [bodies that fit], "scene": a world scene spec (template,
 objects, tool, seams), "examples": [what to say], "setup": {"say": str, "rules": [{"text": "when I ..., ..."}],
 "watch": mode | null}, "demo": {synthetic demonstration for people without a camera}, "physics": "what needs P0"}
```
Load one with {"type": "world", "op": "pack", "name": "weld_cell"} (or the agent's load_pack tool, or the Scene tab).
The current body is kept when the pack lists it, else the pack's default body is used.
"""
import copy
import json
from pathlib import Path

DIR = Path(__file__).resolve().parent


def _load(p):
    try:
        d = json.loads(p.read_text())
    except (OSError, ValueError):
        return None
    d.setdefault("id", p.stem)
    return d


_CACHE = {}


def all_packs():
    """Every pack (read once: the files ship with the code; a copy each call so callers can't change them)."""
    if "packs" not in _CACHE:
        _CACHE["packs"] = {d["id"]: d for d in (_load(p) for p in sorted(DIR.glob("*.json"))) if d}
    return copy.deepcopy(_CACHE["packs"])


ALIASES = {"weld": "weld_cell", "welding": "weld_cell", "weld cell": "weld_cell", "welding cell": "weld_cell",
           "gym": "gym", "workout": "gym", "fitness": "gym", "workshop": "workshop", "workshop helper": "workshop",
           "tools": "workshop", "garage": "workshop"}


def find(name):
    packs = all_packs()
    k = str(name or "").strip().lower()
    k = ALIASES.get(k, k).replace(" ", "_")
    if k in packs:
        return packs[k]
    hits = [v for i, v in packs.items() if k and (k in i or i in k or k in str(v.get("label", "")).lower())]
    return hits[0] if len(hits) == 1 else None


def listing():
    return [{"id": d["id"], "label": d.get("label") or d["id"], "summary": d.get("summary") or "", "robot": d.get("robot"),
             "robots": d.get("robots") or [], "examples": (d.get("examples") or [])[:4], "icon": d.get("icon") or d["id"]}
            for d in all_packs().values()]


def target(name, current_robot=None):
    """Pack -> (robot id, scene spec). Raises ValueError with a sentence."""
    p = find(name)
    if p is None:
        raise ValueError(f"There is no demo pack '{name}'. Packs: {', '.join(all_packs())}.")
    robots = list(p.get("robots") or [p.get("robot")])
    robot = current_robot if current_robot in robots else (p.get("robot") or robots[0])
    spec = copy.deepcopy(p["scene"])
    spec["pack"] = p["id"]
    return robot, spec


def prompt(name):
    p = find(name)
    f = DIR / f"{p['id']}.md" if p else None
    return f.read_text() if f is not None and f.exists() else ""
