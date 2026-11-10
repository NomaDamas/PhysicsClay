"""Scenes built on the fly: scene spec + robot profile -> compiled MuJoCo model (docs/WORLD.md §1).

    {"template": "table" | "kitchen" | "shelf" | "floor" | "gym",
     "objects": [{"name": "mug", "kind": "mug|block|bowl|cup|bottle|can|box|plate|fruit|vase|steel_plate|bench|barbell|
                                          wrench|screwdriver|brush", "color": "red",
                  "size": [sx, sy, sz] | null, "pos": [x, y] | "left" | "center" | ..., "movable": true,
                  "exact": true (keep pos and size as given: workpieces and fixtures), "mass": kg (for P0 physics)}],
     # optional (demo packs, pps/world/packs/): kept in the spec and in state.world.scene
     "pack": "weld_cell", "tool": {"kind": "torch", "tcp": 0.16}, "seams": {name: {"points": [[x, y, z], ...],
             "label": "the L seam", "between": [object, object]}}}

The arena (lights, floor, the template's furniture, objects, IK target ghosts, the robot_view camera) is written as
MJCF, the robot model is attached at its mount with mujoco.MjSpec, and the result is compiled in one go.
Movable objects are mocap bodies `obj_<name>` (carried kinematically by the task layer), fixed ones static bodies
`obs_<name>`; each has an invisible collision geom `<body>_collision` the task layer reads its size from.
The main work surface is always the geom named `table` (the G1 collision pairs and the validator refer to it).
With physics on (pps/physics.py) the same spec is compiled twice: the IK planner's model (objects stay mocap) and the
plant (movable objects become free bodies with mass and friction, grippers get actuators, legged bases a harness).
"""
import copy
import math
import re
import warnings
from dataclasses import dataclass, field

import mujoco
import numpy as np

from .. import physics as phys
from .profiles import DEFAULT_ROBOT, ROOT, RobotProfile, get

G1_DIR = ROOT / "assets" / "unitree_g1"
DEFAULT_XML = G1_DIR / "scene_pc.xml"

# work surfaces: top height, x / y extent (robot frame). The near edge is at x = 0.22 everywhere.
TEMPLATES = {
    "table": {"label": "Wooden table", "top": 0.74, "x": (0.22, 0.82), "y": (-0.50, 0.50)},
    "kitchen": {"label": "Kitchen counter", "top": 0.90, "x": (0.22, 0.84), "y": (-0.80, 0.80)},
    "shelf": {"label": "Shelf unit (work on the lower board)", "top": 0.80, "x": (0.22, 0.62), "y": (-0.45, 0.45)},
    "floor": {"label": "Floor mat", "top": 0.01, "x": (0.22, 0.92), "y": (-0.50, 0.50)},
    "gym": {"label": "Gym floor", "top": 0.01, "x": (0.22, 0.47), "y": (-0.60, 0.60)},    # the lifter stands close (spotting)
}

# kind -> (full size [sx, sy, sz] metres, default colour, collision shape)
KINDS = {
    "mug": ((0.13, 0.13, 0.142), "red", "cyl"),
    "block": ((0.052, 0.052, 0.052), "red", "box"),
    "box": ((0.12, 0.09, 0.08), "cardboard", "box"),
    "bowl": ((0.15, 0.15, 0.06), "white", "cyl"),
    "cup": ((0.07, 0.07, 0.10), "blue", "cyl"),
    "can": ((0.066, 0.066, 0.12), "silver", "cyl"),
    "bottle": ((0.066, 0.066, 0.22), "green", "cyl"),
    "plate": ((0.20, 0.20, 0.02), "white", "cyl"),
    "fruit": ((0.07, 0.07, 0.07), "orange", "cyl"),
    "vase": ((0.09, 0.09, 0.24), "blue", "cyl"),
    # demo packs (P4): workpieces, gym gear and tools in stands
    "steel_plate": ((0.10, 0.30, 0.008), "steel", "box"),
    "bench": ((0.30, 1.10, 0.43), "black", "bench"),
    "barbell": ((0.45, 1.30, 0.45), "black", "bar"),
    "wrench": ((0.06, 0.06, 0.20), "steel", "tool"),
    "screwdriver": ((0.06, 0.06, 0.20), "yellow", "tool"),
    "brush": ((0.06, 0.06, 0.20), "brown", "tool"),
}
SIZE_LIMITS = {"steel_plate": (0.004, 0.60), "bench": (0.02, 1.50), "barbell": (0.02, 1.60)}   # default (0.02, 0.40)
ROUND = ("cyl", "tool")          # footprint is a circle (sx = sy)
TOOLS = {"torch": {"label": "welding torch", "tcp": 0.16}}
KIND_ALIASES = {"cube": "block", "brick": "block", "glass": "cup", "tumbler": "cup", "dish": "plate", "apple": "fruit", "plate_steel": "steel_plate", "steel": "steel_plate", "workpiece": "steel_plate",
                "spanner": "wrench", "bar": "barbell", "weight_bench": "bench",
                "orange": "fruit", "banana": "fruit", "lemon": "fruit", "ball": "fruit", "carton": "box", "crate": "box",
                "jar": "can", "tin": "can", "soda": "can", "coffee mug": "mug", "flask": "bottle"}
COLORS = {
    "red": (0.92, 0.36, 0.33), "green": (0.30, 0.72, 0.45), "blue": (0.24, 0.55, 0.93), "yellow": (0.98, 0.82, 0.25),
    "orange": (0.98, 0.58, 0.20), "purple": (0.58, 0.40, 0.85), "pink": (0.96, 0.55, 0.72), "white": (0.94, 0.94, 0.92),
    "black": (0.15, 0.15, 0.17), "gray": (0.55, 0.56, 0.60), "grey": (0.55, 0.56, 0.60), "silver": (0.75, 0.77, 0.80),
    "brown": (0.55, 0.38, 0.24), "cardboard": (0.74, 0.58, 0.40), "steel": (0.60, 0.62, 0.66),
}

# the UX 2.0 world, as a spec (G1 + this spec loads scene_pc.xml unchanged)
DEFAULT_SCENE = {"template": "table", "objects": [
    {"name": "mug", "kind": "mug", "color": "red", "pos": [0.33, -0.18], "movable": True},
    {"name": "red_block", "kind": "block", "color": "red", "pos": [0.29, -0.34], "movable": True},
    {"name": "green_block", "kind": "block", "color": "green", "pos": [0.47, -0.12], "movable": True},
    {"name": "vase", "kind": "vase", "color": "blue", "pos": [0.33, 0.04], "movable": False},
    {"name": "blue_block", "kind": "block", "color": "blue", "size": [0.06, 0.06, 0.052], "pos": [0.44, 0.24], "movable": False},
]}


def surface(template):
    t = TEMPLATES.get(template) or TEMPLATES["table"]
    return {"template": template if template in TEMPLATES else "table", "top": t["top"], "x": tuple(t["x"]), "y": tuple(t["y"])}


def regions(s):
    """Named spots, relative to the surface's near edge (the table template gives the UX 2.0 values exactly)."""
    near, far = s["x"]
    return {"right": (round(near + 0.09, 3), -0.22), "left": (round(near + 0.09, 3), 0.22), "center": (round(near + 0.12, 3), 0.0),
            "front": (round(near + 0.05, 3), 0.0), "back": (round(far - 0.16, 3), 0.0)}


def templates():
    return [{"id": k, "label": v["label"], "top": v["top"]} for k, v in TEMPLATES.items()]


def _num(v, lo, hi):
    x = float(v)
    if not np.isfinite(x):
        raise ValueError("not finite")
    return float(np.clip(x, lo, hi))


def _norm_tool(t):
    """{"kind": "torch", "tcp": m} (or just "torch") -> canonical tool, or None."""
    if isinstance(t, str):
        t = {"kind": t}
    if not isinstance(t, dict) or str(t.get("kind") or "").lower() not in TOOLS:
        return None
    k = str(t["kind"]).lower()
    try:
        tcp = _num(t.get("tcp", TOOLS[k]["tcp"]), 0.05, 0.30)
    except (TypeError, ValueError):
        tcp = TOOLS[k]["tcp"]
    return {"kind": k, "tcp": round(tcp, 4)}


def _norm_seams(seams):
    """{name: {points: [[x, y, z], ...], label?, between?}} (or a list with names) -> canonical dict."""
    if isinstance(seams, list):
        seams = {str(x.get("name") or f"seam_{i + 1}"): x for i, x in enumerate(seams) if isinstance(x, dict)}
    if not isinstance(seams, dict):
        return {}
    out = {}
    for name, sm in list(seams.items())[:8]:
        if not isinstance(sm, dict):
            continue
        try:
            pts = [[round(_num(c, -5.0, 5.0), 4) for c in p_[:3]] for p_ in (sm.get("points") or [])[:40] if len(p_) >= 3]
        except (TypeError, ValueError):
            continue
        if len(pts) < 2:
            continue
        key = _slug(name) or f"seam_{len(out) + 1}"
        out[key] = {"points": pts, "label": str(sm.get("label") or key.replace("_", " "))[:40],
                    "between": [_slug(b) for b in (sm.get("between") or [])][:4]}
    return out


# ------------------------------------------------------------------ spec normalisation
def _slug(s):
    return re.sub(r"_+", "_", re.sub(r"[^a-z0-9_]", "_", str(s or "").strip().lower())).strip("_")[:24]


def _kind(o):
    k = str(o.get("kind") or "").strip().lower()
    if k in KINDS:
        return k
    if k in KIND_ALIASES:
        return KIND_ALIASES[k]
    name = str(o.get("name") or "").lower()
    for w in sorted(list(KINDS) + list(KIND_ALIASES), key=len, reverse=True):
        if w in name:
            return KIND_ALIASES.get(w, w)
    return "block"


def _color(o, kind):
    c = str(o.get("color") or "").strip().lower()
    if c in COLORS:
        return c
    name = str(o.get("name") or "").lower()
    for w in COLORS:
        if w in name.split("_") or name.startswith(w + " "):
            return w
    return KINDS[kind][1]


def normalize(spec, profile=None):
    """-> (canonical spec with numeric positions, notes). Unknown kinds/colours fall back sensibly, positions are
    clamped onto the surface and spread so objects don't overlap."""
    spec = spec or {}
    tpl = spec.get("template") if spec.get("template") in TEMPLATES else "table"
    s = surface(tpl)
    reg = regions(s)
    notes, out, taken = [], [], []
    names = set()
    for raw in list(spec.get("objects") or [])[:16]:
        if not isinstance(raw, dict):
            continue
        kind = _kind(raw)
        color = _color(raw, kind)
        size = raw.get("size")
        lo, hi = SIZE_LIMITS.get(kind, (0.02, 0.40))
        if isinstance(size, (list, tuple)) and len(size) == 3 and all(isinstance(v, (int, float)) and v > 0 for v in size):
            size = [float(np.clip(v, lo, hi)) for v in size]
        else:
            size = list(KINDS[kind][0])
        if KINDS[kind][2] in ROUND:
            size[1] = size[0]
        exact = bool(raw.get("exact"))
        base = _slug(raw.get("name")) or (f"{color}_{kind}" if raw.get("color") else kind)
        name, k = base, 2
        while name in names:
            name, k = f"{base}_{k}", k + 1
        names.add(name)
        r = max(size[0], size[1]) / 2
        pos = raw.get("pos")
        xy = None
        if isinstance(pos, (list, tuple)) and len(pos) >= 2 and all(isinstance(v, (int, float)) for v in pos[:2]):
            xy = [float(pos[0]), float(pos[1])]
        elif isinstance(pos, str):
            key = pos.strip().lower()
            hit = next((v for kname, v in reg.items() if kname in key), None)
            if hit is not None:
                xy = list(hit)
                if "left" in key and "back" in key or "right" in key and "back" in key:
                    xy[0] = reg["back"][0]
        m = 0.03 + r
        if xy is not None and exact:                    # workpieces and fixtures: exactly where the pack put them
            pass
        elif xy is not None:
            cx = float(np.clip(xy[0], s["x"][0] + m, s["x"][1] - m))
            cy = float(np.clip(xy[1], s["y"][0] + m, s["y"][1] - m))
            if abs(cx - xy[0]) > 1e-6 or abs(cy - xy[1]) > 1e-6:
                notes.append(f"{name} moved onto the {s['template']}")
            xy = [cx, cy]
            if any(np.hypot(xy[0] - p[0], xy[1] - p[1]) < r + q + 0.01 for p, q in taken):
                xy = _free_xy(s, r, taken, prefer=xy)
                if xy is None:                          # never stack objects silently
                    notes.append(f"no room for {name}")
                    continue
        else:
            xy = _free_xy(s, r, taken)
            if xy is None:
                notes.append(f"no room for {name}")
                continue
        if not exact:
            taken.append((xy, r))
        o = {"name": name, "kind": kind, "color": color, "size": [round(v, 4) for v in size],
             "pos": [round(xy[0], 4 if exact else 3), round(xy[1], 4 if exact else 3)], "movable": bool(raw.get("movable", True))}
        if exact:
            o["exact"] = True
        if isinstance(raw.get("mass"), (int, float)) and raw["mass"] > 0:
            o["mass"] = round(float(raw["mass"]), 3)
        out.append(o)
    spec_out = {"template": tpl, "objects": out}
    if spec.get("pack"):
        spec_out["pack"] = _slug(spec["pack"])
    tool = _norm_tool(spec.get("tool"))
    if tool:
        spec_out["tool"] = tool
    seams = _norm_seams(spec.get("seams"))
    if seams:
        spec_out["seams"] = seams
    return spec_out, notes


def _free_xy(s, r, taken, prefer=None):
    """A free spot in the comfortable band in front of the robot (near the near edge, spread left/right)."""
    near = s["x"][0]
    cands = []
    for dx in (0.10, 0.16, 0.06, 0.22, 0.28, 0.34):
        for dy in (0.0, -0.14, 0.14, -0.26, 0.26, -0.36, 0.36, -0.07, 0.07):
            x, y = near + dx, dy
            if not (s["x"][0] + r + 0.03 < x < s["x"][1] - r - 0.03 and s["y"][0] + r + 0.03 < y < s["y"][1] - r - 0.03):
                continue
            if all(np.hypot(x - p[0], y - p[1]) > r + q + 0.03 for p, q in taken):
                score = 0.0 if prefer is None else np.hypot(x - prefer[0], y - prefer[1])
                cands.append((score, len(cands), [x, y]))
    if not cands:                                       # small or off-centre surfaces: a fine grid over the whole top
        cy = (s["y"][0] + s["y"][1]) / 2
        for x in np.arange(s["x"][0] + r + 0.035, s["x"][1] - r - 0.03, 0.02):
            for y in np.arange(s["y"][0] + r + 0.035, s["y"][1] - r - 0.03, 0.02):
                if all(np.hypot(x - p[0], y - p[1]) > r + q + 0.03 for p, q in taken):
                    ref = prefer if prefer is not None else (near + 0.10, cy)
                    cands.append((float(np.hypot(x - ref[0], y - ref[1])), len(cands), [float(x), float(y)]))
        if not cands:
            return None
        return min(cands)[2]
    return min(cands)[2] if prefer is not None else cands[0][2]


def is_default(robot_id, spec):
    return robot_id == DEFAULT_ROBOT and _canon(spec) == _canon(DEFAULT_SCENE)


def _canon(spec):
    s, _ = normalize(spec)
    return s


# ------------------------------------------------------------------ MJCF pieces
def _rgba(color, a=1.0):
    r, g, b = COLORS.get(color, COLORS["gray"])
    return f"{r} {g} {b} {a}"


def _f(*v):
    return " ".join(f"{float(x):.5g}" for x in v)


def _object_xml(o, top):
    kind, (sx, sy, sz) = o["kind"], o["size"]
    x, y = o["pos"]
    movable = o["movable"]
    body = ("obj_" if movable else "obs_") + o["name"]
    rgba = _rgba(o["color"])
    vis = 'contype="0" conaffinity="0" group="2"'
    col = f'name="{body}_collision" rgba="0 0 0 0" contype="1" conaffinity="1" group="3"'
    mocap = ' mocap="true"' if movable else ""
    if kind == "mug":
        s = sx / 0.13
        h = 0.068 * s
        geoms = (f'<geom type="mesh" mesh="pc_mug" material="pc_mug" {vis}/>' if abs(s - 1) < 1e-3 else
                 f'<geom type="cylinder" size="{_f(0.06 * s, 0.065 * s)}" rgba="{rgba}" {vis}/>')
        geoms += f'<geom {col} type="cylinder" size="{_f(0.065 * s, 0.071 * s)}" pos="{_f(0, -0.008 * s, 0.003 * s)}"/>'
        return f'<body name="{body}" pos="{_f(x, y, top + h)}"{mocap}>{geoms}</body>'
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    shape = KINDS[kind][2]
    if shape in ("bench", "bar", "tool"):
        geoms = _gear_geoms(kind, shape, (hx, hy, hz), rgba, vis)
        if shape == "tool":
            geoms += f'<geom {col} type="cylinder" size="{_f(hx, hz)}"/>'
        else:
            geoms += f'<geom {col} type="box" size="{_f(hx, hy, hz)}"/>'
        return f'<body name="{body}" pos="{_f(x, y, top + hz)}"{mocap}>{geoms}</body>'
    if shape == "box":
        geoms = f'<geom type="box" size="{_f(hx, hy, hz)}" rgba="{rgba}" {vis}/>'
        geoms += f'<geom {col} type="box" size="{_f(hx, hy, hz)}"/>'
    else:
        r = hx
        if kind == "fruit":
            geoms = f'<geom type="sphere" size="{_f(r)}" rgba="{rgba}" {vis}/>'
            geoms += f'<geom type="cylinder" size="{_f(0.004, 0.012)}" pos="{_f(0, 0, r + 0.008)}" rgba="0.35 0.25 0.15 1" {vis}/>'
        elif kind == "bottle":
            geoms = f'<geom type="cylinder" size="{_f(r, hz * 0.68)}" pos="{_f(0, 0, -hz * 0.32)}" rgba="{rgba}" {vis}/>'
            geoms += f'<geom type="cylinder" size="{_f(r * 0.4, hz * 0.32)}" pos="{_f(0, 0, hz * 0.68)}" rgba="{rgba}" {vis}/>'
        elif kind == "bowl":
            geoms = f'<geom type="cylinder" size="{_f(r, hz)}" rgba="{rgba}" {vis}/>'
            geoms += f'<geom type="cylinder" size="{_f(r * 0.86, 0.002)}" pos="{_f(0, 0, hz)}" rgba="{_rgba("gray", 1)}" {vis}/>'
        elif kind in ("cup", "mug"):
            geoms = f'<geom type="cylinder" size="{_f(r, hz)}" rgba="{rgba}" {vis}/>'
            geoms += f'<geom type="cylinder" size="{_f(r * 0.85, 0.002)}" pos="{_f(0, 0, hz)}" rgba="0.2 0.2 0.22 1" {vis}/>'
        else:
            geoms = f'<geom type="cylinder" size="{_f(r, hz)}" rgba="{rgba}" {vis}/>'
        geoms += f'<geom {col} type="cylinder" size="{_f(r, hz)}"/>'
    return f'<body name="{body}" pos="{_f(x, y, top + hz)}"{mocap}>{geoms}</body>'


def _gear_geoms(kind, shape, half, rgba, vis):
    """Visual geoms of the pack objects (body frame at the object's centre)."""
    hx, hy, hz = half
    steel, dark = _rgba("steel"), "0.13 0.13 0.15 1"
    if shape == "bench":                              # padded top on two legs
        g = f'<geom type="box" size="{_f(hx, hy, 0.035)}" pos="{_f(0, 0, hz - 0.035)}" rgba="{rgba}" {vis}/>'
        for ly in (-hy + 0.10, hy - 0.10):
            g += f'<geom type="box" size="{_f(hx * 0.75, 0.025, hz - 0.035)}" pos="{_f(0, ly, -0.035)}" rgba="{steel}" {vis}/>'
        return g
    if shape == "bar":                                # a bar along y with two plates
        g = f'<geom type="cylinder" fromto="{_f(0, -hy, 0, 0, hy, 0)}" size="0.0125" rgba="{steel}" {vis}/>'
        for sgn in (-1, 1):
            yc = sgn * (hy - 0.16)
            g += f'<geom type="cylinder" fromto="{_f(0, yc - 0.025, 0, 0, yc + 0.025, 0)}" size="{_f(hx)}" rgba="{rgba}" {vis}/>'
            g += f'<geom type="cylinder" fromto="{_f(0, yc - sgn * 0.045, 0, 0, yc - sgn * 0.03, 0)}" size="0.025" rgba="{steel}" {vis}/>'
        return g
    base = f'<geom type="box" size="{_f(hx * 0.9, hx * 0.9, 0.01)}" pos="{_f(0, 0, -hz + 0.01)}" rgba="{dark}" {vis}/>'
    if kind == "wrench":                              # standing in its stand: handle, open-end head on top
        return (base + f'<geom type="box" size="{_f(0.004, 0.011, hz * 0.72)}" pos="{_f(0, 0, -0.02)}" rgba="{rgba}" {vis}/>'
                + f'<geom type="box" size="{_f(0.005, 0.022, 0.012)}" pos="{_f(0, 0, hz * 0.72)}" rgba="{rgba}" {vis}/>'
                + "".join(f'<geom type="box" size="{_f(0.005, 0.006, 0.016)}" pos="{_f(0, sg * 0.016, hz * 0.72 + 0.026)}" rgba="{rgba}" {vis}/>'
                          for sg in (-1, 1)))
    if kind == "screwdriver":                         # handle up, shaft down into the stand
        return (base + f'<geom type="cylinder" size="0.0035 {_f(hz * 0.4)}" pos="{_f(0, 0, -hz * 0.45)}" rgba="{steel}" {vis}/>'
                + f'<geom type="capsule" size="0.015 {_f(hz * 0.42)}" pos="{_f(0, 0, hz * 0.45)}" rgba="{rgba}" {vis}/>')
    return (base + f'<geom type="cylinder" size="0.009 {_f(hz * 0.62)}" pos="{_f(0, 0, -0.03)}" rgba="{rgba}" {vis}/>'      # brush
            + f'<geom type="box" size="{_f(0.012, 0.026, 0.028)}" pos="{_f(0, 0, hz * 0.62)}" rgba="{rgba}" {vis}/>'
            + f'<geom type="box" size="{_f(0.010, 0.024, 0.018)}" pos="{_f(0, 0, hz * 0.62 + 0.044)}" rgba="{dark}" {vis}/>')


def _template_xml(s):
    t, top = s["template"], s["top"]
    (x0, x1), (y0, y1) = s["x"], s["y"]
    cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    nc = 'contype="0" conaffinity="0"'
    if t == "table":
        g = f'<geom name="table" type="box" size="{_f(hx, hy, 0.02)}" pos="{_f(cx, cy, top - 0.02)}" material="wood"/>'
        for lx in (x0 + 0.03, x1 - 0.03):
            for ly in (y0 + 0.05, y1 - 0.05):
                g += f'<geom type="box" size=".025 .025 {_f((top - 0.04) / 2)}" pos="{_f(lx, ly, (top - 0.04) / 2)}" material="wood" {nc}/>'
        return g
    if t == "kitchen":
        return (f'<geom name="table" type="box" size="{_f(hx + 0.01, hy, 0.02)}" pos="{_f(cx, cy, top - 0.02)}" rgba="0.86 0.85 0.82 1"/>'
                f'<geom name="cabinet" type="box" size="{_f(hx - 0.02, hy - 0.02, (top - 0.04) / 2)}" pos="{_f(cx + 0.02, cy, (top - 0.04) / 2)}" rgba="0.95 0.95 0.96 1" {nc}/>'
                f'<geom name="kick" type="box" size="{_f(0.005, hy - 0.04, 0.05)}" pos="{_f(x0 + 0.06, cy, 0.05)}" rgba="0.3 0.3 0.33 1" {nc}/>'
                f'<geom name="backsplash" type="box" size="{_f(0.01, hy, 0.30)}" pos="{_f(x1 + 0.02, cy, top + 0.30)}" rgba="0.80 0.86 0.90 1"/>')
    if t == "shelf":
        g = f'<geom name="table" type="box" size="{_f(hx, hy, 0.015)}" pos="{_f(cx, cy, top - 0.015)}" material="wood"/>'
        g += f'<geom name="shelf_upper" type="box" size="{_f(hx * 0.6, hy, 0.015)}" pos="{_f(x1 - hx * 0.6, cy, top + 0.42)}" material="wood"/>'
        g += f'<geom name="shelf_bottom" type="box" size="{_f(hx, hy, 0.015)}" pos="{_f(cx, cy, 0.12)}" material="wood" {nc}/>'
        g += f'<geom name="shelf_back" type="box" size="{_f(0.01, hy, (top + 0.5) / 2)}" pos="{_f(x1 + 0.01, cy, (top + 0.5) / 2)}" material="wood"/>'
        for side, yy in (("right", y0 - 0.01), ("left", y1 + 0.01)):
            g += f'<geom name="shelf_{side}" type="box" size="{_f(hx, 0.01, (top + 0.5) / 2)}" pos="{_f(cx, yy, (top + 0.5) / 2)}" material="wood"/>'
        return g
    if t == "gym":                                   # rubber lifting floor (the work surface) on a darker gym floor
        return (f'<geom name="table" type="box" size="{_f(hx, hy, top / 2)}" pos="{_f(cx, cy, top / 2)}" rgba="0.20 0.21 0.24 1"/>'
                f'<geom name="gym_floor" type="box" size="1.6 1.4 0.004" pos="0.6 0 0.002" rgba="0.33 0.34 0.38 1" {nc}/>')
    # floor mat
    return f'<geom name="table" type="box" size="{_f(hx, hy, top / 2)}" pos="{_f(cx, cy, top / 2)}" rgba="0.62 0.70 0.82 1"/>'


def _target_xml(name, size=0.035, rgba=".3 .9 .4 .35"):
    return (f'<body name="{name}" pos="0.5 0 .5" quat="0 1 0 0" mocap="true">'
            f'<geom type="box" size="{_f(size, size, size)}" contype="0" conaffinity="0" group="1" rgba="{rgba}"/>'
            f'<site type="sphere" size="0.01" rgba="0 0 1 1" group="1"/></body>')


def _arena_xml(profile, spec, s):
    top = s["top"]
    cam = profile.camera or {}
    if profile.kind == "humanoid":                    # a head camera, as on the G1 (its head stays where it is)
        cam_pos, cam_ax = [0.13, 0.0, 1.40], [0, -1, 0, 0.845, 0, 0.536]
    elif profile.kind == "bimanual":                  # above the rig, looking at the table
        cam_pos, cam_ax = [0.13, 0.0, top + 0.66], [0, -1, 0, 0.845, 0, 0.536]
    else:                                             # arms: a camera mounted above the base, clear of the arm
        cam_pos, cam_ax = [0.05, 0.0, top + 1.05], [0, -1, 0, 0.866, 0, 0.5]
    cam_pos, cam_ax = cam.get("pos") or cam_pos, cam.get("xyaxes") or cam_ax
    parts = [_template_xml(s)]
    m = profile.mount
    if m.get("on") == "surface" and m.get("pedestal") and top > 0.05:
        ph = top + float(m.get("z", 0.0))                 # a riser lifts the arm above the surface
        parts.append(f'<geom name="pedestal" type="cylinder" size="{_f(0.085, ph / 2)}" pos="{_f(m["x"], m["y"], ph / 2)}" '
                     f'rgba="0.42 0.44 0.48 1" contype="0" conaffinity="0"/>')
    targets = []
    if profile.base == "legged" and profile.feet:
        targets.append('<body name="com_target" pos="0.5 0 .5" mocap="true"><geom type="sphere" size=".06" contype="0" '
                       'conaffinity="0" group="1" rgba=".6 .3 .3 0"/><site type="sphere" size="0.01" rgba="0 0 1 1" group="1"/></body>')
    for role in profile.ee:
        targets.append(_target_xml(f"{role}_palm_target"))
    for foot, _ in profile.feet:
        targets.append(_target_xml(f"{foot}_target", 0.03, ".6 .3 .3 .15"))
    mug = (G1_DIR / "objects" / "mug").as_posix()
    wood = (G1_DIR / "assets" / "wood.png").as_posix()
    arena = f"""<mujoco model="PhysicsClay {profile.id} at the {s['template']}">
  <statistic center="0.3 0 .8" extent="1" meansize="0.08"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.25 0.25 0.27" specular="0 0 0"/>
    <global azimuth="150" elevation="-15" offwidth="1920" offheight="1080"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.985 0.985 0.99" rgb2="0.90 0.91 0.94" width="512" height="512"/>
    <texture name="pc_grid" type="2d" builtin="checker" rgb1=".93 .93 .95" rgb2=".86 .87 .9" width="512" height="512" mark="edge" markrgb=".78 .79 .83"/>
    <material name="pc_grid" texture="pc_grid" texrepeat="1 1" texuniform="true"/>
    <texture type="2d" name="wood" file="{wood}"/>
    <material name="wood" texture="wood"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" directional="true"/>
    <camera name="robot_view" pos="{_f(*cam_pos)}" xyaxes="{_f(*cam_ax)}" fovy="{_f(cam.get('fovy', 72))}"/>
    <light pos="1.5 -1 2.5" dir="-0.5 0.3 -1" diffuse="0.4 0.4 0.4"/>
    <geom name="floor" size="3 3 0.01" type="plane" material="pc_grid"/>
    {''.join(parts)}
  </worldbody>
</mujoco>"""
    props = f"""<mujoco model="props">
  <asset>
    <texture type="2d" name="pc_mug" file="{mug}/texture.png"/>
    <material name="pc_mug" texture="pc_mug" specular="0.5" shininess="0.5"/>
    <mesh name="pc_mug" file="{mug}/model.obj"/>
  </asset>
  <worldbody>
    {''.join(_object_xml(o, top) for o in spec["objects"])}
    {''.join(targets)}
  </worldbody>
</mujoco>"""
    return arena, props


# ------------------------------------------------------------------ build
@dataclass
class World:
    profile: RobotProfile
    spec: dict
    surface: dict
    regions: dict
    model: object
    home_q: np.ndarray | None = None        # None: use the model's keyframe (default world)
    legacy: bool = False                     # the UX 2.0 scene_pc.xml world
    robot_bodies: set = field(default_factory=set)
    notes: list = field(default_factory=list)
    tool: dict | None = None                 # a tool mounted on the end effector (demo packs: the welding torch)
    ik_model: object = None                  # the IK planner's model (physics); None: the same as `model` (kinematic)
    physics: bool = False

    @property
    def kinds(self):
        return {o["name"]: o["kind"] for o in self.spec["objects"]}

    @property
    def robot_roots(self):
        m = self.model
        return {int(m.body_rootid[b]) for b in self.robot_bodies}


def _yaw_quat(deg):
    a = math.radians(deg) / 2
    return np.array([math.cos(a), 0.0, 0.0, math.sin(a)])


def _quat_mul(a, b):
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(a, float), np.asarray(b, float))
    return out


def _compile(sp, profile, kinds, physics, masses=None):
    """-> (plant model, planner model). Kinematic: one model for both. masses: {object: kg} given by the scene spec."""
    if not physics:
        m = sp.compile()
        return m, m
    layout, keys = phys.joint_layout(sp), [np.array(k.qpos, float) for k in sp.keys]
    phys.add_grippers(sp, profile)
    ik = sp.compile()
    for i, q in enumerate(keys):                  # gripper joints were added mid-tree: keyframes follow by joint name
        if i < ik.nkey and len(q) == sum(7 if t == mujoco.mjtJoint.mjJNT_FREE else 1 for _, t in layout):
            ik.key_qpos[i] = phys.home_overrides(profile, ik, phys.remap_q(layout, q, ik))
    phys.plant_spec(sp, profile, kinds, masses)
    return sp.compile(), ik


def build(robot_id=None, spec=None, physics=None):
    """robot id + scene spec -> World (compiled model + home pose). Raises ValueError with a readable message."""
    profile = get(robot_id)
    if profile is None:
        raise ValueError(f"unknown robot '{robot_id}'")
    physics = phys.enabled() if physics is None else bool(physics)
    spec_n, notes = normalize(DEFAULT_SCENE if spec is None else spec, profile)
    s = surface(spec_n["template"])
    kinds = {o["name"]: o["kind"] for o in spec_n["objects"]}
    masses = {o["name"]: o["mass"] for o in spec_n["objects"] if o.get("mass")}     # e.g. the gym's 20 kg barbell
    if is_default(profile.id, spec_n):
        if physics:
            m, ik = _compile(mujoco.MjSpec.from_file(DEFAULT_XML.as_posix()), profile, kinds, True, masses)
        else:
            m = ik = mujoco.MjModel.from_xml_path(DEFAULT_XML.as_posix())
        w = World(profile, spec_n, s, regions(s), m, None, True, notes=notes, ik_model=ik, physics=physics)
        w.robot_bodies = _robot_bodies(m)
        return w
    xml = (ROOT / profile.model_xml)
    if not xml.exists():
        raise ValueError(f"the {profile.label} model is not on disk ({profile.model_xml})")
    child = mujoco.MjSpec.from_file(xml.as_posix())
    tool = None
    if spec_n.get("tool"):
        profile, tool, note = mount_tool(profile, child, spec_n["tool"])
        if note:
            notes.append(note)
    arena_xml, props_xml = _arena_xml(profile, spec_n, s)
    arena = mujoco.MjSpec.from_string(arena_xml)
    key = _home_key(profile, child)
    for k in list(child.keys):
        child.delete(k)
    mnt = profile.mount
    mz = s["top"] + float(mnt.get("z", 0.0)) if mnt.get("on") == "surface" else 0.0
    mpos = np.array([mnt.get("x", 0.0), mnt.get("y", 0.0), mz])
    yaw = float(mnt.get("yaw", 0.0))
    free = [j for j in child.joints if j.type == mujoco.mjtJoint.mjJNT_FREE]
    if free and profile.planar_base:              # e.g. Stretch: the free joint becomes a driven slide (+ yaw)
        fj = free[0]
        body = fj.parent
        if key is not None:
            body.pos, body.quat = key[:3].copy(), key[3:7].copy()
        child.delete(fj)
        for ax in profile.planar_base:
            if ax == "yaw":
                body.add_joint(name="base_yaw", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
            else:
                body.add_joint(name=f"base_{ax}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0] if ax == "x" else [0, 1, 0])
        if key is not None:
            key = np.concatenate([np.zeros(len(profile.planar_base)), key[7:]])
        free = []
    for k in ("integrator", "timestep", "impratio", "cone", "noslip_iterations"):   # no attach conflicts to report
        setattr(arena.option, k, getattr(child.option, k))
    layout = phys.joint_layout(child)             # the home key's layout (joints may be added later: grippers)
    prev = mujoco.get_mju_user_warning()
    mujoco.set_mju_user_warning(lambda msg: None)   # attach warnings (empty prefix) would otherwise land in MUJOCO_LOG.TXT
    try:
        frame = arena.worldbody.add_frame(pos=mpos.tolist(), quat=_yaw_quat(yaw).tolist())
        arena.attach(child, frame=frame, prefix="")
        # objects and IK targets after the robot: the robot must be body 1 (mink's ComTask uses subtree_com[1])
        arena.attach(mujoco.MjSpec.from_string(props_xml), frame=arena.worldbody.add_frame(), prefix="")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            m, ik = _compile(arena, profile, kinds, physics, masses)
    except Exception as e:
        raise ValueError(f"could not build the scene: {e}") from e
    finally:
        mujoco.set_mju_user_warning(prev)
    if key is not None and len(key) == ik.nq:
        q = np.array(key, float)
    elif key is not None and len(key) == sum(7 if t == mujoco.mjtJoint.mjJNT_FREE else 1 for _, t in layout):
        q = phys.remap_q(layout, key, ik)
    else:
        q = ik.qpos0.copy()
    if physics:
        q = phys.home_overrides(profile, ik, q)
    if free:                                      # floating base: place it at the mount
        adr = ik.jnt_qposadr[ik.joint(free[0].name).id] if free[0].name else 0
        qy = _yaw_quat(yaw)
        p = q[adr:adr + 3].copy()
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, qy)
        q[adr:adr + 3] = mpos + R.reshape(3, 3) @ np.array([p[0], p[1], 0.0]) + np.array([0, 0, p[2]])
        q[adr + 3:adr + 7] = _quat_mul(qy, q[adr + 3:adr + 7])
    w = World(profile, spec_n, s, regions(s), m, q, False, notes=notes, tool=tool, ik_model=ik, physics=physics)
    w.robot_bodies = _robot_bodies(m)
    return w


def mount_tool(profile, child, tool):
    """Attach a tool (visual + collision geoms) to a single arm's end-effector body and make its tip the end effector
    (the TCP: IK, paths and checks then work on the torch tip). -> (profile copy, tool info | None, note | None)."""
    if profile.kind not in ("arm", "mobile_manipulator") or len(profile.ee) != 1:
        return profile, None, f"the {TOOLS[tool['kind']]['label']} mounts on single arms only, so the {profile.label} works without it"
    role, site_name = next(iter(profile.ee.items()))
    try:
        site = child.site(site_name)
    except Exception:
        return profile, None, "the tool could not be mounted"
    body = site.parent
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(site.quat, float))
    a = R.reshape(3, 3) @ np.asarray(profile.ee_axis, float)
    a = a / np.linalg.norm(a)
    p0 = np.asarray(site.pos, float)
    L = float(tool["tcp"])

    def capsule(r, s0, s1, rgba, collide=False, kind=mujoco.mjtGeom.mjGEOM_CAPSULE):
        g = body.add_geom()
        g.type = kind
        g.fromto = np.concatenate([p0 + a * s0, p0 + a * s1])
        g.size = [r, 0, 0]
        if collide:
            g.name, g.rgba, g.group, g.contype, g.conaffinity = "torch_collision", [0, 0, 0, 0], 3, 1, 1
        else:
            g.rgba, g.group, g.contype, g.conaffinity = rgba, 2, 0, 0
    capsule(0.022, 0.0, 0.035, [0.10, 0.10, 0.12, 1])                      # mount
    capsule(0.015, 0.035, L - 0.055, [0.16, 0.17, 0.19, 1])                # gun neck
    capsule(0.011, L - 0.055, L - 0.012, [0.78, 0.47, 0.22, 1], kind=mujoco.mjtGeom.mjGEOM_CYLINDER)   # copper nozzle
    capsule(0.0018, L - 0.012, L - 0.002, [0.70, 0.70, 0.72, 1], kind=mujoco.mjtGeom.mjGEOM_CYLINDER)  # wire stick-out
    capsule(0.018, 0.01, L - 0.035, None, collide=True)                    # collision: the gun, not the last 3.5 cm
    tcp = body.add_site()
    tcp.name, tcp.pos, tcp.quat = "torch_tcp", p0 + a * L, np.asarray(site.quat, float)
    tcp.size, tcp.rgba, tcp.group = [0.003, 0, 0], [1.0, 0.45, 0.1, 1], 4
    info = {"kind": tool["kind"], "label": TOOLS[tool["kind"]]["label"], "tcp": L, "site": "torch_tcp", "mount": site_name}
    import dataclasses
    # physics: the torch replaces the gripper (no jaws are built or driven on it; the TCP stays the torch tip)
    ph = {k: v for k, v in (profile.physics or {}).items() if k != "grip"}
    return dataclasses.replace(profile, ee={role: "torch_tcp"}, tool=info, physics=ph), info, None


def _home_key(profile, child):
    src = child
    if profile.home_xml:
        src = mujoco.MjSpec.from_file((ROOT / profile.home_xml).as_posix())
    if profile.home_key:
        for k in src.keys:
            if k.name == profile.home_key:
                return np.array(k.qpos, float)
    return None


def _robot_bodies(m):
    """Bodies that belong to the robot: everything except the world, the objects, fixtures and the IK target ghosts."""
    out = set()
    for b in range(1, m.nbody):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        root = m.body_rootid[b]
        rn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, root) or ""
        if m.body_mocapid[root] >= 0 or rn.startswith(("obj_", "obs_", "pc_harness")) or rn.endswith("_target"):
            continue
        if rn in ("mug", "vase", "red_block", "green_block", "blue_block"):
            continue
        out.add(b)
    return out
