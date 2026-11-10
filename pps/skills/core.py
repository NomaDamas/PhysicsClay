"""Skill objects: kinds, parameters (defaults + bounds), JSON files, one-sentence descriptions and parameter edits.

A skill file (`skills/<name>.json`, user data; examples ship in `pps/skills/examples/`):
```
{"version": 1, "name": "my_weave", "kind": "weave_seam", "params": {...},
 "created": 1731234567.0, "source": {"demo": "p4_weave", "robot": "ur5e", "activity": "weave", "tool": "torch"},
 "measured": {...what the extractor read and how sure it is...}, "notes": ["the torch angle is a default"]}
```
Parameters are *style* relative to geometry: the seam, the region, the container or the target is chosen when the skill is
applied (geometry gives precision), the rest (how) comes from the person.
"""
import copy
import json
import math
import os
import re
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXAMPLES = Path(__file__).resolve().parent / "examples"
VERSION = 1

KINDS = {
    "weave_seam": {
        "label": "Weave along a seam", "needs": ("follow_path",), "trajectory": True,
        "defaults": {"seam": None, "travel_speed": 0.008, "work_angle": 90.0, "travel_angle": 10.0, "standoff": 0.015,
                     "weave": {"kind": "zigzag", "amp": 0.004, "freq": 2.0}, "direction": "auto"},
        "bounds": {"travel_speed": (0.003, 0.08), "work_angle": (45.0, 135.0), "travel_angle": (-30.0, 30.0),
                   "standoff": (0.005, 0.04), "weave.amp": (0.0, 0.03), "weave.freq": (0.2, 6.0)},
        "choices": {"weave.kind": ("zigzag", "circle", "figure8"), "direction": ("auto", "forward", "reverse")}},
    "wipe": {
        "label": "Wipe an area", "needs": ("follow_path",), "trajectory": True,
        "defaults": {"region": None, "force": 5.0, "pattern": "raster", "passes": 1, "spacing": 0.03, "speed": 0.08},
        "bounds": {"force": (0.0, 40.0), "passes": (1, 5), "spacing": (0.01, 0.10), "speed": (0.02, 0.25)},
        "choices": {"pattern": ("raster", "spiral")}},
    "stir": {
        "label": "Stir in a container", "needs": ("follow_path",), "trajectory": True,
        "defaults": {"container": None, "radius": 0.03, "freq": 0.8, "depth": 0.03, "turns": 3, "direction": "ccw"},
        "bounds": {"radius": (0.01, 0.12), "freq": (0.1, 3.0), "depth": (0.0, 0.15), "turns": (1, 20)},
        "choices": {"direction": ("ccw", "cw")}},
    "press": {
        "label": "Press on something", "needs": ("follow_path",), "trajectory": True,
        "defaults": {"target": None, "force": 10.0, "hold_s": 1.5},
        "bounds": {"force": (0.5, 60.0), "hold_s": (0.2, 10.0)}},
    "spot": {
        "label": "Spot a lift", "needs": ("reach",), "trajectory": False,
        "defaults": {"object": "hands", "follow": "hands", "assist_when": {"bar_velocity_below": 0.05, "for_s": 0.5},
                     "max_force": 40.0, "lift_cm": 6.0},
        "bounds": {"assist_when.bar_velocity_below": (0.005, 0.3), "assist_when.for_s": (0.1, 3.0), "max_force": (5.0, 150.0),
                   "lift_cm": (1.0, 15.0)},
        "choices": {"follow": ("hands",)}},
    "mirror_motion": {
        "label": "Whole-body motion", "needs": ("imitate_video",), "trajectory": False,
        "defaults": {"clip": None}, "bounds": {}},
}
INTS = ("passes", "turns")


class SkillError(ValueError):
    """A skill that can't be made, read or applied; the message is a sentence for the person."""


# ---------------------------------------------------------------------------- dotted parameter paths
def get_path(d, path):
    cur = d
    for k in path.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def set_path(d, path, value):
    keys = path.split(".")
    cur = d
    for k in keys[:-1]:
        if not isinstance(cur.get(k), dict):
            cur[k] = {}
        cur = cur[k]
    cur[keys[-1]] = value


def _merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _round(path, v):
    if path in INTS:
        return int(round(v))
    if path.endswith(("amp", "standoff", "radius", "depth", "spacing", "travel_speed", "speed", "bar_velocity_below")):
        return round(float(v), 4)
    return round(float(v), 3)


def clamp_params(kind, params):
    """Params with defaults filled in, numbers clamped to the kind's bounds, choices checked."""
    spec = KINDS[kind]
    p = _merge(spec["defaults"], params or {})
    for path, (lo, hi) in spec["bounds"].items():
        v = get_path(p, path)
        if v is None:
            continue
        try:
            x = float(v)
        except (TypeError, ValueError):
            raise SkillError(f"{path.replace('.', ' ')} must be a number") from None
        if not math.isfinite(x):
            raise SkillError(f"{path.replace('.', ' ')} must be finite")
        set_path(p, path, _round(path, min(hi, max(lo, x))))
    for path, opts in spec.get("choices", {}).items():
        v = get_path(p, path)
        if v is not None and v not in opts:
            raise SkillError(f"{path.replace('.', ' ')} must be one of {', '.join(opts)}")
    return p


# ---------------------------------------------------------------------------- the skill object
def safe_name(name, fallback="skill"):
    n = re.sub(r"[^a-z0-9_-]+", "_", str(name or "").strip().lower()).strip("_")[:40]
    return n or f"{fallback}_{time.strftime('%m%d_%H%M%S')}"


def make(kind, params=None, name=None, source=None, measured=None, notes=None):
    if kind not in KINDS:
        raise SkillError(f"I don't know the skill kind '{kind}'. Kinds: {', '.join(KINDS)}")
    return normalize({"version": VERSION, "name": safe_name(name, kind), "kind": kind, "params": params or {},
                      "created": round(time.time(), 1), "source": source or {}, "measured": measured or {},
                      "notes": list(notes or [])})


def normalize(sk):
    """Validate a skill dict (from a file, the agent or the browser). Raises SkillError."""
    if not isinstance(sk, dict):
        raise SkillError("a skill must be an object")
    kind = str(sk.get("kind") or "")
    if kind not in KINDS:
        raise SkillError(f"unknown skill kind '{kind}'")
    out = {"version": VERSION, "name": safe_name(sk.get("name"), kind), "kind": kind,
           "params": clamp_params(kind, sk.get("params") or {}), "created": float(sk.get("created") or time.time()),
           "source": dict(sk.get("source") or {}), "measured": dict(sk.get("measured") or {}),
           "notes": [str(n)[:160] for n in (sk.get("notes") or [])][:8]}
    if sk.get("example"):
        out["example"] = True
    return out


def needs(sk):
    return list(KINDS[sk["kind"]]["needs"])


def is_trajectory(sk):
    return bool(KINDS[sk["kind"]]["trajectory"])


# ---------------------------------------------------------------------------- describe (one sentence)
def _mm(x):
    v = float(x) * 1000
    return f"{v:.1f}".rstrip("0").rstrip(".") if v < 10 else f"{v:.0f}"


def _speed(v):
    v = float(v)
    return f"{_mm(v)} mm/s" if v < 0.05 else f"{v * 100:.0f} cm/s"


def _hz(f):
    return f"{float(f):.1f}".rstrip("0").rstrip(".")


def describe(sk):
    """'4 mm zigzag at 2 Hz, 5 mm/s, 15° push angle' — the style, in the person's words (no target)."""
    p, k = sk["params"], sk["kind"]
    if k == "weave_seam":
        w = p["weave"]
        a = float(p["travel_angle"])
        ang = "no push or drag" if abs(a) < 0.5 else f"{abs(a):.0f}° {'push' if a > 0 else 'drag'} angle"
        lean = float(p["work_angle"]) - 90.0
        work = "" if abs(lean) < 2 else f", leaning {abs(lean):.0f}° to the {'right' if lean > 0 else 'left'}"
        weave = "no weave" if float(w["amp"]) < 0.0005 else f"{_mm(w['amp'])} mm {w['kind']} at {_hz(w['freq'])} Hz"
        return f"{weave}, {_speed(p['travel_speed'])}, {ang}{work}"
    if k == "wipe":
        pat = "raster" if p["pattern"] == "raster" else "spiral"
        n = int(p["passes"])
        return f"{pat} wipe, strokes {float(p['spacing']) * 100:.0f} cm apart, {float(p['force']):g} N, {n} pass{'es' if n != 1 else ''}"
    if k == "stir":
        return (f"{int(p['turns'])} {'counter-clockwise' if p['direction'] == 'ccw' else 'clockwise'} turns of "
                f"{float(p['radius']) * 100:.0f} cm at {_hz(p['freq'])} Hz, {float(p['depth']) * 100:.0f} cm deep")
    if k == "press":
        return f"press with {float(p['force']):g} N for {float(p['hold_s']):g} s"
    if k == "spot":
        aw = p["assist_when"]
        what = "the bar" if p.get("object") not in (None, "hands") else "your hands"
        return (f"follow {what} under your hands and help when it slows below {float(aw['bar_velocity_below']) * 100:.0f} cm/s "
                f"for {float(aw['for_s']):g} s, up to {float(p['max_force']):g} N ({float(p['lift_cm']):g} cm lift)")
    if k == "mirror_motion":
        return f"replay the whole-body motion '{p.get('clip')}'"
    return k


def summary(sk, ok_here=None):
    """Compact listing entry (state, agent)."""
    out = {"name": sk["name"], "kind": sk["kind"], "label": KINDS[sk["kind"]]["label"], "describe": describe(sk),
           "needs": needs(sk), "example": bool(sk.get("example")), "created": sk.get("created"),
           "demo": (sk.get("source") or {}).get("demo")}
    if ok_here is not None:
        out["ok"] = bool(ok_here)
    return out


# ---------------------------------------------------------------------------- edits
def edit(sk, changes):
    """changes: {dotted path: value | {"mul": f} | {"add": d}} -> (new skill, [what changed, in words])."""
    if not isinstance(changes, dict) or not changes:
        raise SkillError("say what to change")
    out = copy.deepcopy(sk)
    p = out["params"]
    said = []
    bounds = KINDS[sk["kind"]]["bounds"]
    for path, ch in changes.items():
        path = str(path)
        old = get_path(p, path)
        if isinstance(ch, dict) and ("mul" in ch or "add" in ch):
            if not isinstance(old, (int, float)):
                raise SkillError(f"{path.replace('.', ' ')} isn't a number I can scale")
            new = float(old) * float(ch.get("mul", 1.0)) + float(ch.get("add", 0.0))
        else:
            new = ch
        if path not in bounds and path not in KINDS[sk["kind"]].get("choices", {}) and get_path(KINDS[sk["kind"]]["defaults"], path) is None \
                and path.split(".")[0] not in KINDS[sk["kind"]]["defaults"]:
            raise SkillError(f"a {sk['kind'].replace('_', ' ')} skill has no '{path}'")
        set_path(p, path, new)
        said.append(path)
    out["params"] = clamp_params(sk["kind"], p)
    words = []
    for path in said:
        a, b = get_path(sk["params"], path), get_path(out["params"], path)
        if a != b:
            words.append(_change_words(path, a, b))
    return normalize(out), words


def _fmt(path, v):
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        return str(v)
    if path.endswith(("amp", "standoff")):
        return f"{_mm(v)} mm"
    if path.endswith(("radius", "depth", "spacing")):
        return f"{v * 100:.1f}".rstrip("0").rstrip(".") + " cm"
    if path.endswith(("travel_speed", "speed", "bar_velocity_below")):
        return _speed(v)
    if path.endswith("freq"):
        return f"{_hz(v)} Hz"
    if path.endswith("angle"):
        return f"{v:.0f}°"
    if path.endswith("force"):
        return f"{v:g} N"
    if path.endswith(("hold_s", "for_s")):
        return f"{v:g} s"
    return f"{v:g}"


def _change_words(path, a, b):
    name = {"weave.amp": "weave width", "weave.freq": "weave frequency", "weave.kind": "weave", "travel_speed": "travel speed",
            "travel_angle": "travel angle", "work_angle": "work angle", "assist_when.bar_velocity_below": "help below",
            "assist_when.for_s": "after", "max_force": "max force", "lift_cm": "lift", "hold_s": "hold"}.get(path, path.replace("_", " "))
    return f"{name} {_fmt(path, a)} → {_fmt(path, b)}"


# ---------------------------------------------------------------------------- files
def skill_dir():
    return Path(os.environ.get("PC_SKILLS_DIR") or (ROOT / "skills"))


def save(sk, directory=None):
    sk = normalize(sk)
    d = Path(directory) if directory else skill_dir()
    d.mkdir(parents=True, exist_ok=True)
    path = d / f"{sk['name']}.json"
    body = {k: v for k, v in sk.items() if k != "example"}
    path.write_text(json.dumps(body, indent=1))
    return path


def _read(path, example=False):
    try:
        sk = normalize(json.loads(Path(path).read_text()))
    except (OSError, ValueError, SkillError):
        return None
    if example:
        sk["example"] = True
    return sk


def load(name, directory=None):
    """User skills first, then the shipped examples."""
    key = safe_name(name)
    d = Path(directory) if directory else skill_dir()
    for p, ex in ((d / f"{key}.json", False), (EXAMPLES / f"{key}.json", True)):
        if p.exists():
            return _read(p, ex)
    return None


def listing(directory=None):
    """Every skill (user ones override examples of the same name), newest user skills first."""
    d = Path(directory) if directory else skill_dir()
    out = {}
    for p in sorted(EXAMPLES.glob("*.json")):
        sk = _read(p, True)
        if sk:
            out[sk["name"]] = sk
    user = []
    if d.exists():
        for p in d.glob("*.json"):
            sk = _read(p)
            if sk:
                user.append(sk)
    for sk in sorted(user, key=lambda s: s.get("created") or 0):
        out[sk["name"]] = sk
    return sorted(out.values(), key=lambda s: (bool(s.get("example")), -(s.get("created") or 0)))


def delete(name, directory=None):
    d = Path(directory) if directory else skill_dir()
    p = d / f"{safe_name(name)}.json"
    if p.exists():
        p.unlink()
        return True
    return False
