"""Articulated fixtures (issue #48): parametric MJCF builders for furniture and machines with real joints.

A fixture is `{"name", "kind", "pos": [x, y, z] (centre of its footprint on the floor, or on what it stands on),
"yaw": degrees, "size": [w (along its local y), d (local x, depth), h], ...kind parameters}`. Local frame: the front
faces -x (toward the robot when yaw = 0), y is to the left seen from the front, z is up.

Static parts are world geoms named `fx_<fixture>_<part>`. Moving parts (drawers, doors, lids, the rack of a dishwasher,
the sliding door of a CNC enclosure, conveyor slats, rollers) are bodies `fx_<fixture>_<part>`:
  * the IK planner keeps them as mocap bodies (synced from the plant when a task plans around them),
  * the plant (pps/physics.py) gives them their joint (slide or hinge) with damping, friction, armature and limits,
    excludes their contacts with the static world (they are held by their joint), and drives conveyors.
Handles are knobs (a stem and a head the jaws close on) with a site `fx_<fixture>_<part>_handle` at the grasp point.

build(fixture) -> Built(static_xml, bodies_xml, parts, containers, surfaces): `parts` are the joints (what opens,
how far, where the handle is), `containers` the volumes things go in (a drawer, a sink, a tote, a rack slot),
`surfaces` extra flat tops (shelves, a side table) for placing.
"""
import math
from dataclasses import dataclass, field

import numpy as np

KNOB_R, KNOB_HEAD, KNOB_OUT = 0.007, 0.016, 0.050     # stem radius, head radius, how far the head stands off the front
T = 0.018                                             # panel thickness

COL = {"white": "0.95 0.95 0.94 1", "cream": "0.93 0.90 0.82 1", "oak": "0.72 0.55 0.36 1", "walnut": "0.45 0.31 0.21 1",
       "sage": "0.60 0.68 0.58 1", "navy": "0.20 0.27 0.40 1", "grey": "0.55 0.57 0.60 1", "charcoal": "0.24 0.25 0.28 1",
       "steel": "0.66 0.68 0.72 1", "dark": "0.14 0.14 0.16 1", "black": "0.08 0.08 0.09 1", "granite": "0.36 0.36 0.38 1",
       "marble": "0.90 0.90 0.88 1", "butcher": "0.78 0.60 0.40 1", "blue": "0.20 0.42 0.75 1", "yellow": "0.95 0.78 0.15 1",
       "orange": "0.95 0.50 0.15 1", "green": "0.22 0.55 0.35 1", "red": "0.80 0.20 0.18 1", "glass": "0.70 0.85 0.95 0.30",
       "rubber": "0.18 0.18 0.20 1", "safety": "0.98 0.80 0.10 1", "cardboard": "0.74 0.58 0.40 1", "pallet": "0.80 0.66 0.45 1",
       "esd": "0.20 0.40 0.62 1", "labwhite": "0.92 0.93 0.95 1", "teal": "0.15 0.55 0.58 1", "beige": "0.84 0.78 0.68 1"}


def rgba(c):
    return COL.get(c, c if isinstance(c, str) and len(c.split()) == 4 else COL["grey"])


def _f(*v):
    return " ".join(f"{float(x):.5g}" for x in v)


def _quat_z(a):
    return np.array([math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)])


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


Q_X90 = np.array([math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0])    # cylinder axis z -> -y ... (about x)
Q_Y90 = np.array([math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0])    # cylinder axis z -> x


@dataclass
class Built:
    static: list = field(default_factory=list)       # world geom XML
    bodies: list = field(default_factory=list)       # moving part body XML (mocap; the plant adds the joints)
    parts: list = field(default_factory=list)        # joint meta (see Fb.part)
    containers: list = field(default_factory=list)   # {"name", "label", "fixture", "body" | None, "c", "h", "yaw"}
    surfaces: list = field(default_factory=list)     # {"name", "z", "c" (world xy), "h" (half xy), "yaw"}
    belts: list = field(default_factory=list)        # conveyors: {"name", "kind", "parts", "speed", "axis", "stop_at", ...}


class Fb:
    """Builds one fixture in its local frame (front at -x, y left, z up) and writes world MJCF."""

    def __init__(self, fx, out):
        self.fx, self.out = fx, out
        self.name = fx["name"]
        self.P = np.array(fx.get("pos", (0, 0, 0)), float)
        self.a = math.radians(float(fx.get("yaw", 0.0)))
        c, s = math.cos(self.a), math.sin(self.a)
        self.R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        self.q = _quat_z(self.a)
        self.n = 0

    def w(self, p):
        return self.P + self.R @ np.asarray(p, float)

    def wdir(self, v):
        return self.R @ np.asarray(v, float)

    def _gname(self, part):
        self.n += 1
        return f"fx_{self.name}_{part or 'g'}{self.n}"

    # ---- static geoms (world frame)
    def box(self, part, c, h, color, collide=True, q=None):
        q = self.q if q is None else _qmul(self.q, q)
        cc = 'contype="1" conaffinity="1"' if collide else 'contype="0" conaffinity="0"'
        self.out.static.append(f'<geom name="{self._gname(part)}" type="box" size="{_f(*h)}" pos="{_f(*self.w(c))}" '
                               f'quat="{_f(*q)}" rgba="{rgba(color)}" {cc} group="0"/>')

    def cyl(self, part, c, r, hh, color, collide=True, q=None, typ="cylinder"):
        q = self.q if q is None else _qmul(self.q, q)
        cc = 'contype="1" conaffinity="1"' if collide else 'contype="0" conaffinity="0"'
        self.out.static.append(f'<geom name="{self._gname(part)}" type="{typ}" size="{_f(r, hh)}" pos="{_f(*self.w(c))}" '
                               f'quat="{_f(*q)}" rgba="{rgba(color)}" {cc} group="0"/>')

    def named(self, gname, c, h, color):
        """A static box with an exact name (the main work surface must be called `table`)."""
        self.out.static.append(f'<geom name="{gname}" type="box" size="{_f(*h)}" pos="{_f(*self.w(c))}" quat="{_f(*self.q)}" '
                               f'rgba="{rgba(color)}" contype="1" conaffinity="1" group="0"/>')

    # ---- moving parts
    def part(self, pname, joint, anchor, axis, rng, geoms, handle=None, label=None, damping=4.0, friction=0.6,
             armature=0.05, stiffness=0.0, springref=0.0, container=None, kind=None, act=None, name=None):
        """A moving part: body at `anchor` (local), joint 'slide' | 'hinge' along / about `axis` (local), range [lo, hi]
        (lo = closed). geoms: [(type, size, pos (local), quat (local) | None, colour, collide)]. handle: grasp point
        (local) of the knob and its outward direction (local). container: interior box (local centre, half)."""
        body = f"fx_{self.name}_{pname}"
        an = np.asarray(anchor, float)
        xml = [f'<body name="{body}" pos="{_f(*self.w(an))}" quat="{_f(*self.q)}" mocap="true">']
        for k, (typ, size, pos, q, color, collide) in enumerate(geoms):
            cc = 'contype="1" conaffinity="1"' if collide else 'contype="0" conaffinity="0"'
            qq = f' quat="{_f(*q)}"' if q is not None else ""
            gn = f"{body}_floor{k}" if collide == "support" or kind in ("slat", "roller") else f"{body}_{k}"   # floors: not obstacles
            xml.append(f'<geom name="{gn}" type="{typ}" size="{_f(*size)}" pos="{_f(*(np.asarray(pos, float) - an))}"{qq} '
                       f'rgba="{rgba(color)}" {cc} group="0"/>')
        hd = None
        if handle is not None:
            hp, hdir = np.asarray(handle[0], float), np.asarray(handle[1], float)
            xml.append(f'<site name="{body}_handle" pos="{_f(*(hp - an))}" size="0.008" rgba="1 0.5 0.1 0" group="5"/>')
            hd = {"site": f"{body}_handle", "local": (hp - an).round(5).tolist(), "out": hdir.tolist()}
        xml.append("</body>")
        self.out.bodies.append("".join(xml))
        meta = {"name": name or (pname if pname.startswith(self.name) else f"{self.name}_{pname}"), "fixture": self.name,
                "kind": kind or self.fx["kind"], "body": body, "joint": f"{body}_joint", "type": joint,
                "axis": [float(x) for x in axis], "range": [float(rng[0]), float(rng[1])], "damping": damping,
                "friction": friction, "armature": armature, "stiffness": stiffness, "springref": springref,
                "handle": hd, "label": label or pname.replace("_", " "), "act": act}
        self.out.parts.append(meta)
        if container is not None:
            cc, hh = container
            self.out.containers.append({"name": meta["name"], "label": meta["label"], "fixture": self.name, "body": body,
                                        "c": (np.asarray(cc, float) - an).round(5).tolist(), "h": [float(x) for x in hh],
                                        "yaw": 0.0, "kind": meta["kind"]})
        return meta

    def container(self, cname, c, h, label=None, kind=None):
        """A static interior volume (sink basin, tote, rack slot): world centre, half extents in the fixture's frame."""
        self.out.containers.append({"name": cname, "label": label or cname.replace("_", " "), "fixture": self.name, "body": None,
                                    "c": self.w(c).round(5).tolist(), "h": [float(x) for x in h], "yaw": round(self.a, 6),
                                    "kind": kind or self.fx["kind"]})

    def surface(self, sname, c, h, label=None):
        self.out.surfaces.append({"name": sname, "label": label or sname.replace("_", " "), "z": float(self.w(c)[2]),
                                  "c": self.w(c)[:2].round(5).tolist(), "h": [float(h[0]), float(h[1])], "yaw": round(self.a, 6)})


# ------------------------------------------------------------------ geometry helpers for parts (local frame)
def g_box(c, h, color, collide=True, q=None):
    return ("box", h, c, q, color, collide)


def g_cyl(c, r, hh, color, collide=True, q=None):
    return ("cylinder", (r, hh), c, q, color, collide)


def knob(front_x, y, z, color="steel", out=(-1, 0, 0)):
    """A knob on a front panel at x = front_x (local), standing off toward `out`: stem + head geoms, handle point."""
    o = np.asarray(out, float)
    base = np.array([front_x, y, z])
    stem_c = base + o * (KNOB_OUT * 0.5)
    head_c = base + o * (KNOB_OUT - 0.008)
    q = Q_Y90 if abs(o[0]) > 0.5 else Q_X90 if abs(o[1]) > 0.5 else None
    geoms = [g_cyl(stem_c, KNOB_R, KNOB_OUT * 0.5, color, True, q), g_cyl(head_c, KNOB_HEAD, 0.008, color, True, q)]
    return geoms, (head_c, o)


def drawer_geoms(w, d, hf, front_x, z0, color_front, color_box="white", knob_color="steel"):
    """A drawer: front panel (w x hf at x = front_x, bottom at z0), a box behind it, a knob. -> (geoms, handle, interior)."""
    g = [g_box((front_x + T / 2, 0, z0 + hf / 2), (T / 2, w / 2, hf / 2), color_front)]
    bw, bd, bh = w - 0.04, d - 0.06, max(0.05, hf - 0.05)
    x0 = front_x + T
    zb = z0 + 0.022
    g += [g_box((x0 + bd / 2, 0, zb + 0.006), (bd / 2, bw / 2, 0.006), color_box, "support"),           # floor
          g_box((x0 + bd - 0.006, 0, zb + bh / 2), (0.006, bw / 2, bh / 2), color_box),                 # back
          g_box((x0 + bd / 2, bw / 2 - 0.006, zb + bh / 2), (bd / 2, 0.006, bh / 2), color_box),        # sides
          g_box((x0 + bd / 2, -bw / 2 + 0.006, zb + bh / 2), (bd / 2, 0.006, bh / 2), color_box)]
    kg, handle = knob(front_x, 0.0, z0 + hf * (0.5 if hf < 0.2 else 0.78), knob_color)
    interior = ((x0 + bd / 2, 0, zb + 0.012 + (bh - 0.012) / 2), (bd / 2 - 0.012, bw / 2 - 0.012, (bh - 0.012) / 2))
    return g + kg, handle, interior


# ------------------------------------------------------------------ home fixtures
def base_cabinet(b, fx):
    """Kitchen / storage base cabinet: carcass + a front of drawers (`drawers`: list of front heights, top first) and / or
    doors (`doors`: 0, 1 or 2 below the drawers; `hinge`: left | right for one door). No top (a countertop goes on it)."""
    w, d, h = fx["size"]
    kick = fx.get("kick", 0.09)
    body, front = fx.get("color", "white"), fx.get("front", "white")
    kc = fx.get("knob", "steel")
    b.box("side", (0, w / 2 - T / 2, h / 2), (d / 2, T / 2, h / 2), body)
    b.box("side", (0, -w / 2 + T / 2, h / 2), (d / 2, T / 2, h / 2), body)
    b.box("back", (d / 2 - T / 2, 0, h / 2), (T / 2, w / 2 - T, h / 2), body)
    b.box("bottom", (0, 0, kick + T / 2), (d / 2 - T, w / 2 - T, T / 2), body)
    b.box("kick", (-d / 2 + 0.05, 0, kick / 2), (0.01, w / 2 - T, kick / 2), "charcoal", collide=False)
    b.box("rail", (-d / 2 + 0.03, 0, h - 0.01), (0.03, w / 2 - T, 0.01), body)
    fx_x = -d / 2                                         # front plane (local x)
    z = h - 0.004
    names = fx.get("drawer_names") or []
    for i, hf in enumerate(fx.get("drawers") or []):
        z -= hf
        pname = f"drawer_{i + 1}"
        geoms, handle, interior = drawer_geoms(w - 0.006, d, hf - 0.004, fx_x, z + 0.002, front, knob_color=kc)
        travel = round(min(0.36, (d - 0.06) * 0.78), 3)
        lab = fx.get("drawer_labels", [None] * 9)[i] if i < len(fx.get("drawer_labels") or []) else None
        b.part(pname, "slide", (fx_x, 0, z), (-1, 0, 0), (0.0, travel), geoms, handle, label=lab, damping=6.0, friction=0.8,
               armature=0.2, container=interior, kind="drawer", name=names[i] if i < len(names) else None)
    nd = int(fx.get("doors", 0))
    if nd:
        top = z - 0.004
        hd = top - kick - 0.004
        dw = (w - 0.006) / nd
        for k in range(nd):
            yc = (-w / 2 + 0.003) + dw * (k + 0.5)
            left_hinge = (nd == 2 and k == 1) or (nd == 1 and fx.get("hinge", "left") == "left")
            hy = yc + dw / 2 if left_hinge else yc - dw / 2                  # hinge edge (local y)
            ky = yc - dw / 2 + 0.05 if left_hinge else yc + dw / 2 - 0.05     # knob near the free edge
            an = (fx_x, hy, kick + 0.002)
            geoms = [g_box((fx_x + T / 2, yc, kick + 0.002 + hd / 2), (T / 2, dw / 2 - 0.002, hd / 2), front)]
            kg, handle = knob(fx_x, ky, kick + hd - 0.07, kc)
            pn = f"door_{'left' if left_hinge else 'right'}" if nd == 2 else "door"
            dn = (fx.get("door_names") or [])[k] if k < len(fx.get("door_names") or []) else None
            dl = (fx.get("door_labels") or [])[k] if k < len(fx.get("door_labels") or []) else None
            b.part(pn, "hinge", an, (0, 0, -1) if left_hinge else (0, 0, 1), (0.0, math.radians(100)), geoms + kg, handle,
                   label=dl, damping=0.6, friction=0.25, armature=0.02, kind="door", name=dn)
        b.box("shelf", (0.01, 0, kick + hd * 0.55), (d / 2 - T - 0.02, w / 2 - T, 0.009), body)
        b.container(f"{b.name}_inside", (0.02, 0, kick + T + (hd * 0.55 - T) / 2), (d / 2 - 0.06, w / 2 - 0.04, (hd * 0.55 - T) / 2 - 0.01),
                    label=f"inside the {b.name.replace('_', ' ')}", kind="cabinet")


def countertop(b, fx):
    """A slab of `size` at its height (pos z = the floor), with an optional rectangular cut-out `hole`:
    {"c": [x, y] local, "h": [hx, hy]} (for a sink). The piece named by `main` (default 'table') becomes that geom name."""
    w, d, th = fx["size"]
    top = fx["top"]
    color = fx.get("color", "granite")
    hole = fx.get("hole")
    z = top - th / 2
    if not hole:
        if fx.get("main"):
            b.named(fx["main"], (0, 0, z), (d / 2, w / 2, th / 2), color)
        else:
            b.box("top", (0, 0, z), (d / 2, w / 2, th / 2), color)
        return
    (hx0, hy0), (hhx, hhy) = hole["c"], hole["h"]
    # four pieces around the hole: front strip, back strip, left and right blocks
    if hx0 - hhx > -d / 2 + 0.005:
        b.box("top", ((-d / 2 + hx0 - hhx) / 2, hy0, z), ((hx0 - hhx + d / 2) / 2, hhy, th / 2), color)
    if hx0 + hhx < d / 2 - 0.005:
        b.box("top", ((hx0 + hhx + d / 2) / 2, hy0, z), ((d / 2 - hx0 - hhx) / 2, hhy, th / 2), color)
    yl0, yl1 = hy0 + hhy, w / 2
    yr0, yr1 = -w / 2, hy0 - hhy
    for (y0, y1) in ((yl0, yl1), (yr0, yr1)):
        if y1 - y0 < 0.005:
            continue
        main = fx.get("main") and (y0 <= fx.get("main_y", 0.0) <= y1)
        if main:
            b.named(fx["main"], (0, (y0 + y1) / 2, z), (d / 2, (y1 - y0) / 2, th / 2), color)
        else:
            b.box("top", (0, (y0 + y1) / 2, z), (d / 2, (y1 - y0) / 2, th / 2), color)


def sink(b, fx):
    """A sink basin hanging under a countertop cut-out: `size` = [w, d, depth] of the basin, `top` = the counter top."""
    w, d, depth = fx["size"]
    top = fx["top"]
    c = fx.get("color", "steel")
    t = 0.008
    zb = top - depth
    b.box("floor", (0, 0, zb + t / 2), (d / 2, w / 2, t / 2), c)
    b.box("wall", (-d / 2 + t / 2, 0, zb + depth / 2), (t / 2, w / 2, depth / 2), c)
    b.box("wall", (d / 2 - t / 2, 0, zb + depth / 2), (t / 2, w / 2, depth / 2), c)
    b.box("wall", (0, w / 2 - t / 2, zb + depth / 2), (d / 2, t / 2, depth / 2), c)
    b.box("wall", (0, -w / 2 + t / 2, zb + depth / 2), (d / 2, t / 2, depth / 2), c)
    b.cyl("drain", (0.02, 0, zb + t + 0.001), 0.03, 0.001, "dark", collide=False)
    # faucet at the back
    fx_x = d / 2 + 0.04
    b.cyl("faucet", (fx_x, 0, top + 0.13), 0.014, 0.13, "steel")
    b.cyl("faucet", (fx_x - 0.08, 0, top + 0.255), 0.012, 0.08, "steel", q=Q_Y90)
    b.cyl("faucet", (fx_x - 0.15, 0, top + 0.235), 0.012, 0.02, "steel", collide=False)
    b.container(b.name, (0, 0, zb + t + (depth - t) / 2), (d / 2 - t - 0.01, w / 2 - t - 0.01, (depth - t) / 2), label="the sink",
                kind="sink")


def stove(b, fx):
    """A cooktop set into the counter (`size` = w, d of the glass), four burners and front knobs."""
    w, d = fx["size"][:2]
    top = fx["top"]
    b.box("glass", (0, 0, top + 0.003), (d / 2, w / 2, 0.004), "black")
    for i, (px, py) in enumerate(((-0.1, -0.13), (-0.1, 0.13), (0.1, -0.13), (0.1, 0.13))):
        r = 0.09 if i % 3 == 0 else 0.07
        b.cyl("burner", (px * d / 0.5, py * w / 0.6, top + 0.0075), r, 0.0008, "0.55 0.18 0.15 1", collide=False)
    b.surface(b.name, (0, 0, top + 0.007), (d / 2 - 0.03, w / 2 - 0.03), label="the stove")


def upper_cabinet(b, fx):
    """Wall cabinet (pos z = its bottom), two doors with knobs low on the free edges, one shelf inside."""
    w, d, h = fx["size"]
    body, front = fx.get("color", "white"), fx.get("front", "white")
    z0 = 0.0
    b.box("side", (0, w / 2 - T / 2, z0 + h / 2), (d / 2, T / 2, h / 2), body)
    b.box("side", (0, -w / 2 + T / 2, z0 + h / 2), (d / 2, T / 2, h / 2), body)
    b.box("back", (d / 2 - T / 2, 0, z0 + h / 2), (T / 2, w / 2 - T, h / 2), body)
    b.box("bottom", (0, 0, z0 + T / 2), (d / 2, w / 2 - T, T / 2), body)
    b.box("top", (0, 0, z0 + h - T / 2), (d / 2, w / 2 - T, T / 2), body)
    b.box("shelf", (0.01, 0, z0 + h * 0.5), (d / 2 - T - 0.01, w / 2 - T, 0.008), body)
    fx_x = -d / 2
    nd = 2 if w > 0.55 else 1
    dw = (w - 0.006) / nd
    for k in range(nd):
        yc = (-w / 2 + 0.003) + dw * (k + 0.5)
        left_hinge = (nd == 2 and k == 1) or nd == 1
        hy = yc + dw / 2 if left_hinge else yc - dw / 2
        ky = yc - dw / 2 + 0.05 if left_hinge else yc + dw / 2 - 0.05
        geoms = [g_box((fx_x + T / 2 - T, yc, z0 + h / 2), (T / 2, dw / 2 - 0.002, h / 2 - 0.002), front)]
        kg, handle = knob(fx_x - T, ky, z0 + 0.07, fx.get("knob", "steel"))
        b.part(f"door_{'left' if left_hinge else 'right'}" if nd == 2 else "door", "hinge", (fx_x - T, hy, z0),
               (0, 0, -1) if left_hinge else (0, 0, 1), (0.0, math.radians(100)), geoms + kg, handle,
               label=f"{'left' if left_hinge else 'right'} door of the wall cabinet" if nd == 2 else "wall cabinet door",
               damping=0.5, friction=0.2, armature=0.02, kind="door")
    b.container(f"{b.name}_shelf", (0.0, 0, z0 + h * 0.5 + 0.008 + (h * 0.5 - T - 0.02) / 2), (d / 2 - 0.05, w / 2 - 0.04, (h * 0.5 - T - 0.02) / 2),
                label="the wall cabinet shelf", kind="cabinet")
    b.container(f"{b.name}_inside", (0.0, 0, z0 + T + (h * 0.5 - T) / 2), (d / 2 - 0.05, w / 2 - 0.04, (h * 0.5 - T - 0.02) / 2),
                label="inside the wall cabinet", kind="cabinet")


def fridge(b, fx):
    """Tall fridge: a body, a door (hinge on `hinge` side) with door shelves, shelves inside."""
    w, d, h = fx["size"]
    c = fx.get("color", "steel")
    t = 0.03
    b.box("side", (0, w / 2 - t / 2, h / 2), (d / 2, t / 2, h / 2), c)
    b.box("side", (0, -w / 2 + t / 2, h / 2), (d / 2, t / 2, h / 2), c)
    b.box("back", (d / 2 - t / 2, 0, h / 2), (t / 2, w / 2 - t, h / 2), c)
    b.box("bottom", (0, 0, 0.06), (d / 2, w / 2 - t, 0.06), c)
    b.box("top", (0, 0, h - t / 2), (d / 2, w / 2 - t, t / 2), c)
    b.box("inner", (0.0, 0, h / 2), (d / 2 - t - 0.01, w / 2 - t, 0.004), "0.92 0.95 0.97 1", collide=False)
    for zz in (0.55, 0.85, 1.15, 1.42):
        if zz < h - 0.1:
            b.box("shelf", (0.02, 0, zz), (d / 2 - t - 0.03, w / 2 - t, 0.005), "glass")
    b.surface(f"{b.name}_shelf", (0.02, 0, 0.855), (d / 2 - t - 0.06, w / 2 - t - 0.03), label="the fridge shelf")
    left = fx.get("hinge", "left") == "left"
    fx_x = -d / 2
    hy = w / 2 if left else -w / 2
    dy = -1 if left else 1                                   # from the hinge toward the free edge
    geoms = [g_box((fx_x - 0.025, hy + dy * w / 2, h / 2), (0.025, w / 2 - 0.002, h / 2 - 0.004), c)]
    for zz in (0.5, 0.85, 1.2):                              # door bins
        geoms.append(g_box((fx_x - 0.025 + 0.07, hy + dy * w / 2, zz), (0.045, w / 2 - 0.05, 0.004), "glass"))
        geoms.append(g_box((fx_x - 0.025 + 0.112, hy + dy * w / 2, zz + 0.04), (0.004, w / 2 - 0.05, 0.04), "glass"))
    kz = fx.get("knob_z", 1.05)
    kg, handle = knob(fx_x - 0.05, hy + dy * (w - 0.06), kz, "steel")
    geoms.append(g_box((fx_x - 0.05 - 0.004, hy + dy * (w - 0.06), kz), (0.004, 0.012, 0.18), "steel", False))
    b.part("door", "hinge", (fx_x, hy, 0.0), (0, 0, -1) if left else (0, 0, 1), (0.0, math.radians(105)), geoms + kg, handle,
           label="the fridge door", damping=2.0, friction=0.6, armature=0.2, kind="door")
    b.container(b.name, (0.02, 0, 0.855 + 0.15), (d / 2 - t - 0.06, w / 2 - t - 0.03, 0.14), label="the fridge", kind="fridge")


def microwave(b, fx):
    """Microwave on a counter (pos z = the counter top): box, side-hinged door (hinge on the left), turntable inside."""
    w, d, h = fx["size"]
    c = fx.get("color", "charcoal")
    t = 0.015
    b.box("bottom", (0, 0, t / 2), (d / 2, w / 2, t / 2), c)
    b.box("top", (0, 0, h - t / 2), (d / 2, w / 2, t / 2), c)
    b.box("back", (d / 2 - t / 2, 0, h / 2), (t / 2, w / 2, h / 2), c)
    b.box("side", (0, w / 2 - t / 2, h / 2), (d / 2, t / 2, h / 2), c)
    b.box("panel", (0, -w / 2 + 0.06, h / 2), (d / 2, 0.06, h / 2), c)                  # control panel on the right
    b.box("keys", (-d / 2 - 0.001, -w / 2 + 0.06, h * 0.55), (0.002, 0.04, h * 0.3), "0.3 0.32 0.35 1", collide=False)
    b.cyl("plate", (0.0, 0.06, t + 0.004), min(d, w - 0.12) / 2 - 0.03, 0.004, "glass")
    iw = w - 0.12 - t
    fx_x = -d / 2
    hy = w / 2                                                # hinge on the left
    geoms = [g_box((fx_x - 0.012, hy - iw / 2 - 0.004, h / 2), (0.012, iw / 2, h / 2 - 0.004), "0.18 0.18 0.2 1"),
             g_box((fx_x - 0.025, hy - iw / 2 - 0.004, h / 2), (0.002, iw / 2 - 0.04, h / 2 - 0.04), "0.25 0.32 0.36 0.6", False)]
    kg, handle = knob(fx_x - 0.024, hy - iw + 0.025, h * 0.5, "steel")
    b.part("door", "hinge", (fx_x, hy, 0.0), (0, 0, -1), (0.0, math.radians(110)), geoms + kg, handle, label="the microwave door",
           damping=0.3, friction=0.15, armature=0.01, kind="door")
    b.container(b.name, (0.0, 0.06, t + 0.012 + (h - 2 * t - 0.02) / 2), (d / 2 - 0.04, iw / 2 - 0.03, (h - 2 * t - 0.03) / 2),
                label="the microwave", kind="microwave")


def dishwasher(b, fx):
    """Under-counter dishwasher: a bottom-hinged door (spring balanced) and a rack drawer behind it."""
    w, d, h = fx["size"]
    c = fx.get("color", "steel")
    b.box("side", (0, w / 2 - 0.01, h / 2), (d / 2, 0.01, h / 2), "0.5 0.52 0.55 1")
    b.box("side", (0, -w / 2 + 0.01, h / 2), (d / 2, 0.01, h / 2), "0.5 0.52 0.55 1")
    b.box("back", (d / 2 - 0.01, 0, h / 2), (0.01, w / 2 - 0.02, h / 2), "0.5 0.52 0.55 1")
    b.box("bottom", (0, 0, 0.09), (d / 2, w / 2 - 0.02, 0.01), "0.5 0.52 0.55 1")
    b.box("top", (0, 0, h - 0.01), (d / 2, w / 2 - 0.02, 0.01), "0.5 0.52 0.55 1")
    fx_x = -d / 2
    hd = h - 0.10
    geoms = [g_box((fx_x + 0.012, 0, 0.10 + hd / 2), (0.012, w / 2 - 0.004, hd / 2), c),
             g_box((fx_x - 0.002, 0, 0.10 + hd - 0.05), (0.003, w / 2 - 0.02, 0.03), "dark", False)]
    kg, handle = knob(fx_x, 0.0, 0.10 + hd - 0.09, "steel")
    b.part("door", "hinge", (fx_x, 0, 0.10), (0, -1, 0), (0.0, math.radians(88)), geoms + kg, handle, label="the dishwasher door",
           damping=3.0, friction=0.5, armature=0.1, stiffness=6.0, kind="door")
    rd = d - 0.10
    rz = 0.16
    rack = [g_box((fx_x + 0.05 + rd / 2, 0, rz), (rd / 2, w / 2 - 0.05, 0.004), "0.75 0.77 0.8 1", "support")]
    for yy in np.linspace(-w / 2 + 0.07, w / 2 - 0.07, 7):
        rack.append(g_box((fx_x + 0.05 + rd / 2, yy, rz + 0.05), (rd / 2 - 0.02, 0.002, 0.05), "0.75 0.77 0.8 1", False))
    rack.append(g_box((fx_x + 0.05 + rd - 0.004, 0, rz + 0.05), (0.004, w / 2 - 0.05, 0.05), "0.75 0.77 0.8 1"))
    rack.append(g_box((fx_x + 0.05 + 0.004, 0, rz + 0.03), (0.004, w / 2 - 0.05, 0.03), "0.75 0.77 0.8 1"))
    kg2, h2 = knob(fx_x + 0.05, 0.0, rz + 0.04, "0.75 0.77 0.8 1")
    b.part("rack", "slide", (fx_x + 0.05, 0, rz), (-1, 0, 0), (0.0, round(rd * 0.8, 3)), rack + kg2, h2, label="the dishwasher rack",
           damping=4.0, friction=0.5, armature=0.1, container=((fx_x + 0.05 + rd / 2, 0, rz + 0.06), (rd / 2 - 0.03, w / 2 - 0.07, 0.05)),
           kind="rack")


def shelving(b, fx):
    """Open shelving (pos z = the floor or the wall bracket height): `levels` boards, side panels."""
    w, d, h = fx["size"]
    c = fx.get("color", "oak")
    levels = int(fx.get("levels", 3))
    b.box("side", (0, w / 2 - T / 2, h / 2), (d / 2, T / 2, h / 2), c)
    b.box("side", (0, -w / 2 + T / 2, h / 2), (d / 2, T / 2, h / 2), c)
    if fx.get("back", True):
        b.box("back", (d / 2 - 0.005, 0, h / 2), (0.005, w / 2 - T, h / 2), c)
    for k in range(levels):
        z = 0.02 + k * (h - 0.04) / max(1, levels - 1) if levels > 1 else 0.02
        b.box("board", (0, 0, z), (d / 2, w / 2 - T, 0.012), c)
        if k < levels - 1:
            gap = (h - 0.04) / max(1, levels - 1)
            b.container(f"{b.name}_{k + 1}", (0, 0, z + 0.012 + gap / 2 - 0.01), (d / 2 - 0.03, w / 2 - T - 0.02, gap / 2 - 0.03),
                        label=f"the {_ord(k + 1)} shelf of the {b.name.replace('_', ' ')}", kind="shelf")
            b.surface(f"{b.name}_{k + 1}", (0, 0, z + 0.012), (d / 2 - 0.03, w / 2 - T - 0.02), label=f"the {_ord(k + 1)} shelf")


def _ord(k):
    return {1: "bottom", 2: "second", 3: "third", 4: "fourth", 5: "fifth"}.get(k, f"{k}th")


def table(b, fx):
    """A table (dining, coffee, work, lab): top at `top`, four legs (or a frame). `main`: name the top geom 'table'."""
    w, d = fx["size"][:2]
    top = fx["top"]
    th = fx.get("thick", 0.035)
    c, lc = fx.get("color", "oak"), fx.get("legs", fx.get("color", "oak"))
    if fx.get("main"):
        b.named(fx["main"], (0, 0, top - th / 2), (d / 2, w / 2, th / 2), c)
    else:
        b.box("top", (0, 0, top - th / 2), (d / 2, w / 2, th / 2), c)
        b.surface(b.name, (0, 0, top), (d / 2 - 0.03, w / 2 - 0.03), label=f"the {b.name.replace('_', ' ')}")
    lh = (top - th) / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            b.box("leg", (sx * (d / 2 - 0.04), sy * (w / 2 - 0.04), lh), (0.022, 0.022, lh), lc, collide=False)
    if fx.get("shelf"):
        b.box("shelf", (0, 0, 0.18), (d / 2 - 0.04, w / 2 - 0.04, 0.012), lc)
    if fx.get("mat"):
        b.box("mat", (0, 0, top + 0.001), (d / 2 - 0.05, w / 2 - 0.08, 0.0015), fx["mat"], collide=False)


def chair(b, fx):
    s = 0.44
    c = fx.get("color", "oak")
    b.box("seat", (0, 0, 0.45), (s / 2, s / 2, 0.02), c)
    b.box("back", (s / 2 - 0.02, 0, 0.70), (0.02, s / 2, 0.22), c)
    for sx in (-1, 1):
        for sy in (-1, 1):
            b.box("leg", (sx * (s / 2 - 0.03), sy * (s / 2 - 0.03), 0.215), (0.018, 0.018, 0.215), c, collide=False)


def sofa(b, fx):
    w, d, h = fx["size"]
    c = fx.get("color", "navy")
    b.box("base", (0, 0, 0.21), (d / 2, w / 2, 0.21), c)
    b.box("back", (d / 2 - 0.10, 0, 0.58), (0.10, w / 2, 0.20), c)
    for s in (-1, 1):
        b.box("arm", (0, s * (w / 2 - 0.09), 0.52), (d / 2, 0.09, 0.12), c)
    for k in (-1, 1):
        b.box("cushion", (-0.06, k * (w / 4 - 0.05), 0.46), (d / 2 - 0.16, w / 4 - 0.08, 0.04), "0.28 0.36 0.52 1")
    b.surface(b.name, (-0.06, 0, 0.50), (d / 2 - 0.2, w / 2 - 0.25), label="the sofa")


def tv_unit(b, fx):
    """Low media cabinet with two drawers and a TV on top."""
    w, d, h = fx["size"]
    c, front = fx.get("color", "walnut"), fx.get("front", "walnut")
    b.box("body", (d / 2 * 0.15, 0, h / 2), (d / 2 * 0.85, w / 2, h / 2 - 0.02), c)
    b.box("top", (0, 0, h - 0.012), (d / 2, w / 2, 0.012), c)
    b.box("plinth", (0, 0, 0.02), (d / 2 - 0.02, w / 2 - 0.02, 0.02), "charcoal")
    fx_x = -d / 2
    dw = w / 2 - 0.01
    for k, nm in enumerate(("drawer_left", "drawer_right")):
        yc = (k * 2 - 1) * (dw / 2 + 0.005)
        geoms, handle, interior = drawer_geoms(dw - 0.006, d * 0.85, h - 0.08, fx_x, 0.045, front)
        geoms = [(t, s, (p[0], p[1] + yc, p[2]), q, cc, col) for (t, s, p, q, cc, col) in geoms]
        handle = (np.asarray(handle[0]) + np.array([0, yc, 0]), handle[1])
        interior = ((interior[0][0], interior[0][1] + yc, interior[0][2]), interior[1])
        b.part(nm, "slide", (fx_x, yc, 0.045), (-1, 0, 0), (0.0, round(d * 0.6, 3)), geoms, handle,
               label=f"the {'right' if k == 0 else 'left'} drawer of the TV unit", damping=5.0, friction=0.6, armature=0.1,
               container=interior, kind="drawer")
    b.box("tv", (d / 4, 0, h + 0.36), (0.025, min(0.62, w / 2 - 0.05), 0.34), "black")
    b.box("tvstand", (d / 4, 0, h + 0.02), (0.08, 0.15, 0.01), "black")
    b.surface(b.name, (0, 0, h), (d / 2 - 0.04, w / 2 - 0.05), label="the TV unit")


def washer(b, fx):
    """Front-loading washer or dryer: a box with a round door (hinge on the left) on the drum opening."""
    w, d, h = fx["size"]
    c = fx.get("color", "white")
    t = 0.02
    b.box("top", (0, 0, h - t / 2), (d / 2, w / 2, t / 2), c)
    b.box("side", (0, w / 2 - t / 2, h / 2), (d / 2, t / 2, h / 2), c)
    b.box("side", (0, -w / 2 + t / 2, h / 2), (d / 2, t / 2, h / 2), c)
    b.box("back", (d / 2 - t / 2, 0, h / 2), (t / 2, w / 2, h / 2), c)
    b.box("bottom", (0, 0, 0.1), (d / 2, w / 2, 0.1), c)
    zc = h * 0.48
    r = min(w, h) * 0.30
    # front panel with a square opening around the drum (four strips)
    b.box("front", (-d / 2 + t / 2, 0, (zc + r + h) / 2), (t / 2, w / 2, (h - zc - r) / 2), c)
    b.box("front", (-d / 2 + t / 2, 0, (zc - r + 0.2) / 2), (t / 2, w / 2, (zc - r - 0.2) / 2), c)
    b.box("front", (-d / 2 + t / 2, (r + w / 2) / 2, zc), (t / 2, (w / 2 - r) / 2, r), c)
    b.box("front", (-d / 2 + t / 2, -(r + w / 2) / 2, zc), (t / 2, (w / 2 - r) / 2, r), c)
    b.cyl("drum", (0.03, 0, zc), r * 0.98, d / 2 - 0.06, "0.7 0.72 0.75 1", collide=False, q=Q_Y90)
    b.box("drumfloor", (0.0, 0, zc - r + 0.02), (d / 2 - t - 0.04, r * 0.8, 0.01), "0.7 0.72 0.75 1")
    b.box("panel", (-d / 2 - 0.001, 0, h - 0.06), (0.002, w / 2 - 0.03, 0.04), "0.3 0.32 0.35 1", collide=False)
    fx_x = -d / 2
    hy = r + 0.02
    geoms = [g_cyl((fx_x - 0.02, hy - r - 0.02, zc), r + 0.025, 0.02, "0.82 0.84 0.86 1", True, Q_Y90),
             g_cyl((fx_x - 0.042, hy - r - 0.02, zc), r * 0.75, 0.002, "0.3 0.4 0.5 0.6", False, Q_Y90)]
    kg, handle = knob(fx_x - 0.04, hy - 2 * r - 0.02, zc, "steel")
    b.part("door", "hinge", (fx_x, hy, zc), (0, 0, -1), (0.0, math.radians(100)), geoms + kg, handle,
           label=f"the {fx.get('label', b.name.replace('_', ' '))} door", damping=0.8, friction=0.3, armature=0.03, kind="door")
    b.container(b.name, (0.03, 0, zc - r * 0.35), (d / 2 - 0.1, r * 0.75, r * 0.5), label=f"the {fx.get('label', 'washer')}", kind="drum")


def bin_(b, fx):
    """A hollow open bin (tote, laundry basket, carton, parts bin): floor + four walls (pos z = what it stands on)."""
    w, d, h = fx["size"]
    c = fx.get("color", "blue")
    t = fx.get("wall", 0.008)
    b.box("floor", (0, 0, t / 2), (d / 2, w / 2, t / 2), c)
    b.box("wall", (-d / 2 + t / 2, 0, h / 2), (t / 2, w / 2, h / 2), c)
    b.box("wall", (d / 2 - t / 2, 0, h / 2), (t / 2, w / 2, h / 2), c)
    b.box("wall", (0, w / 2 - t / 2, h / 2), (d / 2 - t, t / 2, h / 2), c)
    b.box("wall", (0, -w / 2 + t / 2, h / 2), (d / 2 - t, t / 2, h / 2), c)
    if fx.get("flaps"):                                  # an open carton: flaps folded out
        for s in (-1, 1):
            b.box("flap", (s * (d / 2 + 0.04), 0, h - 0.002), (0.04, w / 2, 0.002), c, collide=False)
    if fx.get("rim"):
        b.box("rim", (-d / 2 - 0.004, 0, h - 0.01), (0.004, w / 2, 0.01), fx["rim"], collide=False)
    b.container(b.name, (0, 0, t + (h - t) / 2), (d / 2 - t - 0.008, w / 2 - t - 0.008, (h - t) / 2),
                label=fx.get("label") or f"the {b.name.replace('_', ' ')}", kind=fx.get("ckind", "bin"))


# ------------------------------------------------------------------ industrial fixtures
def conveyor(b, fx):
    """Belt conveyor along local y: slats on slide joints driven at `speed` (m/s, + = toward +y); a slat that leaves the
    downstream end wraps to the upstream end. `stop_at` (local y): a photo-eye that stops the belt while a part sits in
    front of it (the station). The bed under the slats is the `main` geom when given (the work surface)."""
    L, W, top = fx["size"][0], fx["size"][1], fx["top"]
    pitch = fx.get("pitch", 0.075)
    n = max(4, int(round(L / pitch)))
    L = n * pitch
    st = 0.012                                            # slat thickness
    zb = top - st
    frame = fx.get("frame", "grey")
    if fx.get("main"):
        b.named(fx["main"], (0, 0, zb - 0.01), (W / 2, L / 2, 0.01), "dark")
    else:
        b.box("bed", (0, 0, zb - 0.01), (W / 2, L / 2, 0.01), "dark")
    for s in (-1, 1):                                     # side rails (guides) above the belt
        b.box("rail", (s * (W / 2 + 0.012), 0, top + 0.02), (0.012, L / 2 + 0.05, 0.035), frame)
    for yy in (-L / 2 + 0.05, L / 2 - 0.05):
        for s in (-1, 1):
            b.box("leg", (s * (W / 2 - 0.02), yy, (zb - 0.02) / 2), (0.025, 0.025, (zb - 0.02) / 2), frame, collide=False)
    for yy in (-L / 2 - 0.02, L / 2 + 0.02):
        b.cyl("drum", (0, yy, zb - 0.025), 0.035, W / 2 - 0.01, frame, collide=False, q=Q_X90)
    parts = []
    belt_c = fx.get("belt", "rubber")
    for i in range(n):
        y0 = -L / 2 + pitch * (i + 0.5)
        geoms = [g_box((0, y0, top - st / 2), (W / 2 - 0.004, pitch / 2 - 0.0015, st / 2), belt_c if i % 2 else "0.22 0.22 0.24 1")]
        m = b.part(f"slat{i}", "slide", (0, y0, top - st / 2), (0, 1, 0), (-10.0, 10.0), geoms, None, label="belt", damping=0.0,
                   friction=0.0, armature=0.0, kind="slat", act={"type": "velocity", "kv": 400.0, "force": 120.0})
        m["wrap"] = {"lo": -L / 2, "hi": L / 2, "y0": y0, "len": L}
        parts.append(m["joint"])
    stop = fx.get("stop_at")
    if stop is not None:                                  # photo-eye post and the end stop
        b.box("eye", (W / 2 + 0.05, stop, top + 0.06), (0.012, 0.012, 0.07), "safety", collide=False)
        b.box("eye", (-W / 2 - 0.05, stop, top + 0.06), (0.012, 0.012, 0.07), "safety", collide=False)
    end = L / 2 if fx.get("speed", 0.08) >= 0 else -L / 2
    b.box("endstop", (0, end - math.copysign(0.01, end), top + 0.03), (W / 2, 0.01, 0.03), frame)
    b.out.belts.append({"name": b.name, "kind": "conveyor", "joints": parts, "speed": float(fx.get("speed", 0.08)),
                        "stop_at": None if stop is None else [float(x) for x in b.w((0, stop, top))], "axis": b.wdir((0, 1, 0)).tolist(),
                        "width": W, "top": top, "center": b.w((0, 0, top)).tolist(), "length": L, "label": fx.get("label", "the conveyor")})


def roller_conveyor(b, fx):
    """Driven roller conveyor along local y (rollers on hinges, velocity servos): boxes ride on the turning rollers."""
    L, W, top = fx["size"][0], fx["size"][1], fx["top"]
    r = 0.025
    pitch = fx.get("pitch", 0.075)
    n = max(3, int(L / pitch))
    frame = fx.get("frame", "grey")
    for s in (-1, 1):
        b.box("rail", (s * (W / 2 + 0.01), 0, top - r), (0.01, L / 2, 0.045), frame)
        for yy in (-L / 2 + 0.05, L / 2 - 0.05):
            b.box("leg", (s * (W / 2 - 0.02), yy, (top - r - 0.04) / 2), (0.02, 0.02, (top - r - 0.04) / 2), frame, collide=False)
    joints = []
    for i in range(n):
        y = -L / 2 + pitch * (i + 0.5)
        m = b.part(f"roller{i}", "hinge", (0, y, top - r), (-1, 0, 0), (-1e4, 1e4),
                   [g_cyl((0, y, top - r), r, W / 2 - 0.004, "steel", True, Q_Y90)], None, label="roller", damping=0.0,
                   friction=0.0, armature=0.0, kind="roller", act={"type": "velocity", "kv": 4.0, "force": 6.0})
        joints.append(m["joint"])
    b.out.belts.append({"name": b.name, "kind": "rollers", "joints": joints, "speed": float(fx.get("speed", 0.1)), "radius": r,
                        "stop_at": None, "axis": b.wdir((0, 1, 0)).tolist(), "width": W, "top": top, "center": b.w((0, 0, top)).tolist(),
                        "length": L, "label": fx.get("label", "the roller conveyor")})
    b.surface(b.name, (0, 0, top), (W / 2 - 0.02, L / 2 - 0.05), label=fx.get("label", "the roller conveyor"))


def rack(b, fx):
    """Warehouse pallet / shelf rack: uprights, beams, decks at `levels` heights; bays are containers on the decks."""
    w, d, h = fx["size"]
    levels = fx.get("levels", [0.15, 0.75, 1.35])
    bays = int(fx.get("bays", 2))
    up, beam = "blue", "orange"
    for sy in np.linspace(-w / 2, w / 2, bays + 1):
        for sx in (-d / 2 + 0.04, d / 2 - 0.04):
            b.box("upright", (sx, sy, h / 2), (0.035, 0.035, h / 2), up)
    bw = w / bays
    for lv, z in enumerate(levels):
        b.box("beam", (-d / 2 + 0.04, 0, z - 0.04), (0.03, w / 2, 0.04), beam, collide=False)
        b.box("deck", (0, 0, z), (d / 2 - 0.04, w / 2 - 0.03, 0.012), "grey")
        for k in range(bays):
            yc = -w / 2 + bw * (k + 0.5)
            b.surface(f"{b.name}_l{lv + 1}_b{k + 1}", (0, yc, z + 0.012), (d / 2 - 0.08, bw / 2 - 0.06),
                      label=f"level {lv + 1}, bay {k + 1} of the {b.name.replace('_', ' ')}")


def pallet(b, fx):
    """A wooden pallet on the floor (1.2 x 0.8 m class)."""
    w, d = fx["size"][:2]
    c = "pallet"
    for yy in (-w / 2 + 0.05, 0, w / 2 - 0.05):
        b.box("block", (0, yy, 0.05), (d / 2, 0.045, 0.05), c)
    for k in range(7):
        x = -d / 2 + 0.05 + k * (d - 0.1) / 6
        b.box("board", (x, 0, 0.11), (0.045, w / 2, 0.011), c)
    b.surface(b.name, (0, 0, 0.122), (d / 2 - 0.05, w / 2 - 0.05), label=f"the {b.name.replace('_', ' ')}")


def cnc(b, fx):
    """Bench-top CNC mill: an enclosure (pos z = the bench top) with a front door that slides to the side, a vise with
    fixed and moving jaws inside, the spindle overhead. The vise opening is a container (the part sits in it)."""
    w, d, h = fx["size"]
    c = fx.get("color", "0.30 0.34 0.38 1")
    t = 0.012
    b.box("base", (0, 0, 0.04), (d / 2, w / 2, 0.04), c)
    b.box("top", (0, 0, h - t / 2), (d / 2, w / 2, t / 2), c)
    b.box("back", (d / 2 - t / 2, 0, h / 2), (t / 2, w / 2, h / 2), c)
    b.box("side", (0, w / 2 - t / 2, h / 2), (d / 2, t / 2, h / 2), c)
    b.box("side", (0, -w / 2 + t / 2, h / 2), (d / 2, t / 2, h / 2), c)
    b.box("lintel", (-d / 2 + t / 2, 0, h - 0.06), (t / 2, w / 2, 0.06), c)
    b.box("window", (0, w / 2 - t - 0.001, h * 0.6), (d / 2 - 0.05, 0.001, h * 0.25), "glass", collide=False)
    b.box("spindle", (0.06, 0, h - 0.12), (0.05, 0.05, 0.06), "dark", collide=False)
    b.cyl("tool", (0.06, 0, h - 0.21), 0.006, 0.03, "steel", collide=False)
    b.box("table", (0.02, 0, 0.09), (d / 2 - 0.08, w / 2 - 0.08, 0.01), "steel")
    vx, vz = 0.04, 0.10
    b.box("vise", (vx, 0, vz + 0.015), (0.06, 0.085, 0.015), "0.2 0.35 0.6 1")
    b.box("jaw", (vx + 0.045, 0, vz + 0.055), (0.012, 0.07, 0.025), "steel")
    b.box("jaw", (vx - 0.045, 0, vz + 0.055), (0.012, 0.07, 0.025), "steel")
    b.container(f"{b.name}_vise", (vx, 0, vz + 0.03 + 0.03), (0.032, 0.065, 0.03), label="the vise", kind="vise")
    # the door: slides toward +y (behind the right... left wall), panel with a window and a knob near its free edge
    dw = w - 0.04
    fx_x = -d / 2 - 0.01
    geoms = [g_box((fx_x - 0.006, 0, h / 2 + 0.02), (0.006, dw / 2, h / 2 - 0.08), "safety"),
             g_box((fx_x - 0.013, 0, h / 2 + 0.04), (0.001, dw / 2 - 0.05, h / 2 - 0.14), "glass", False)]
    kg, handle = knob(fx_x - 0.012, -dw / 2 + 0.05, h * 0.5, "dark")
    b.part("door", "slide", (fx_x, 0, 0.0), (0, 1, 0), (0.0, round(dw * 0.85, 3)), geoms + kg, handle, label="the CNC door",
           damping=8.0, friction=1.5, armature=0.3, kind="door")


def tube_rack(b, fx):
    """Test-tube rack on a bench (pos z = the bench top): a base, a top plate with `n` square slots (built from bars so
    tubes really drop in) and one container per slot (`<name>_1` ...)."""
    n = int(fx.get("n", 6))
    pitch = fx.get("pitch", 0.040)
    slot = fx.get("slot", 0.032)
    h = fx.get("h", 0.060)
    c = fx.get("color", "teal")
    L = n * pitch
    b.box("base", (0, 0, 0.004), (0.035, L / 2 + 0.01, 0.004), c)
    tp = 0.004
    zt = h - tp
    bar_w = (pitch - slot) / 2
    for k in range(n + 1):                                       # bars across between slots
        y = -L / 2 + k * pitch
        b.box("bar", (0, y, zt), (0.035, bar_w + 0.0005, tp), c)
    for s in (-1, 1):                                            # front / back strips of the top plate
        b.box("strip", (s * (slot / 2 + 0.008), 0, zt), (0.008, L / 2 + bar_w, tp), c)
    for s in (-1, 1):                                            # end walls
        b.box("end", (0, s * (L / 2 + 0.008), h / 2), (0.035, 0.004, h / 2), c)
    for k in range(n):
        y = -L / 2 + pitch * (k + 0.5)
        b.container(f"{b.name}_{k + 1}", (0, y, 0.008 + 0.045), (slot / 2 - 0.004, slot / 2 - 0.004, 0.045),
                    label=f"slot {k + 1} of {fx.get('label') or 'the ' + b.name.replace('_', ' ')}", kind="slot")


def centrifuge(b, fx):
    """Bench-top centrifuge (pos z = the bench top): a round body, a rotor bowl, and a lid hinged at the back."""
    r = fx.get("r", 0.14)
    h = fx.get("h", 0.20)
    c = fx.get("color", "labwhite")
    b.box("body", (0, 0, h / 2 - 0.02), (r, r, h / 2 - 0.02), c)
    b.cyl("bowl", (0, 0, h - 0.035), r * 0.75, 0.015, "0.4 0.42 0.45 1", collide=False)
    b.box("panel", (-r - 0.002, 0, h * 0.4), (0.002, r * 0.6, h * 0.2), "0.25 0.3 0.35 1", collide=False)
    an = (r - 0.01, 0, h - 0.04)
    geoms = [g_box((0, 0, h - 0.02), (r, r, 0.018), c), g_cyl((0, 0, h + 0.0), r * 0.55, 0.002, "glass", False)]
    kg, handle = knob(-r, 0, h - 0.025, "dark")
    b.part("lid", "hinge", an, (0, 1, 0), (0.0, math.radians(95)), geoms + kg, handle, label="the centrifuge lid",
           damping=0.4, friction=0.1, armature=0.01, kind="lid")


def parts_tray(b, fx):
    """A compartment tray (pos z = the bench top): `nx` x `ny` cells, each a container."""
    w, d = fx["size"][:2]
    h = fx.get("h", 0.035)
    nx, ny = int(fx.get("nx", 2)), int(fx.get("ny", 3))
    c = fx.get("color", "0.25 0.3 0.4 1")
    t = 0.004
    b.box("floor", (0, 0, t / 2), (d / 2, w / 2, t / 2), c)
    for i in range(nx + 1):
        x = -d / 2 + i * d / nx
        b.box("div", (x, 0, h / 2), (t / 2, w / 2, h / 2), c)
    for j in range(ny + 1):
        y = -w / 2 + j * w / ny
        b.box("div", (0, y, h / 2), (d / 2, t / 2, h / 2), c)
    for i in range(nx):
        for j in range(ny):
            cx, cy = -d / 2 + d / nx * (i + 0.5), -w / 2 + w / ny * (j + 0.5)
            b.container(f"{b.name}_{i * ny + j + 1}", (cx, cy, t + (h - t) / 2), (d / nx / 2 - t - 0.003, w / ny / 2 - t - 0.003, (h - t) / 2),
                        label=f"compartment {i * ny + j + 1} of {fx.get('label') or 'the ' + b.name.replace('_', ' ')}", kind="tray")


def scale(b, fx):
    """A packing scale (pos z = the table top): base, steel platform, display."""
    w, d = fx["size"][:2]
    b.box("base", (0, 0, 0.015), (d / 2, w / 2, 0.015), "dark")
    b.box("platform", (0, 0, 0.034), (d / 2 - 0.01, w / 2 - 0.01, 0.004), "steel")
    b.box("display", (-d / 2 - 0.03, 0, 0.05), (0.02, 0.06, 0.03), "0.15 0.2 0.18 1", collide=False)
    b.surface(b.name, (0, 0, 0.038), (d / 2 - 0.02, w / 2 - 0.02), label="the scale")


def static_box(b, fx):
    """Any plain block (a wall, a machine body, a monitor, a lamp base): size w, d, h, colour."""
    w, d, h = fx["size"]
    b.box(fx.get("part", "box"), (0, 0, h / 2), (d / 2, w / 2, h / 2), fx.get("color", "grey"), collide=fx.get("collide", True))


def floor_patch(b, fx):
    """A coloured floor area (rug, tiles) or, with `lines`, a rectangle of floor marking (safety lines): no contact."""
    w, d = fx["size"][:2]
    c = fx.get("color", "beige")
    if fx.get("lines"):
        t = 0.05
        for s in (-1, 1):
            b.box("line", (s * (d / 2 - t / 2), 0, 0.0025), (t / 2, w / 2, 0.0005), c, collide=False)
            b.box("line", (0, s * (w / 2 - t / 2), 0.0025), (d / 2, t / 2, 0.0005), c, collide=False)
        return
    b.box("floor", (0, 0, 0.001), (d / 2, w / 2, 0.001), c, collide=False)


BUILDERS = {"base_cabinet": base_cabinet, "countertop": countertop, "sink": sink, "stove": stove, "upper_cabinet": upper_cabinet,
            "fridge": fridge, "microwave": microwave, "dishwasher": dishwasher, "shelving": shelving, "table": table,
            "chair": chair, "sofa": sofa, "tv_unit": tv_unit, "washer": washer, "bin": bin_, "conveyor": conveyor,
            "roller_conveyor": roller_conveyor, "rack": rack, "pallet": pallet, "cnc": cnc, "tube_rack": tube_rack,
            "centrifuge": centrifuge, "parts_tray": parts_tray, "scale": scale, "box": static_box, "floor": floor_patch}
KINDS = tuple(BUILDERS)


def build(fixtures):
    """[fixture spec] -> Built (all of them together)."""
    out = Built()
    for fx in fixtures:
        fn = BUILDERS.get(fx.get("kind"))
        if fn is None:
            continue
        fn(Fb(fx, out), fx)
    return out


def normalize(fixtures):
    """Keep well-formed fixture specs only (known kind, a name, numbers that are numbers). -> list."""
    out, names = [], set()
    for fx in list(fixtures or [])[:80]:
        if not isinstance(fx, dict) or fx.get("kind") not in BUILDERS:
            continue
        name = "".join(ch for ch in str(fx.get("name") or fx["kind"]).lower().replace(" ", "_") if ch.isalnum() or ch == "_")[:28]
        base, k = name or fx["kind"], 2
        while name in names or not name:
            name, k = f"{base}_{k}", k + 1
        names.add(name)
        try:
            pos = [float(v) for v in (fx.get("pos") or (0, 0, 0))][:3]
            size = [float(v) for v in (fx.get("size") or (0.5, 0.5, 0.5))][:3]
        except (TypeError, ValueError):
            continue
        if len(pos) < 3:
            pos = pos + [0.0] * (3 - len(pos))
        if len(size) < 3:
            size = size + [0.0] * (3 - len(size))
        if not all(np.isfinite(pos)) or not all(np.isfinite(size)) or any(s <= 0 for s in size[:2]) or size[2] < 0:
            continue
        o = dict(fx, name=name, pos=[round(v, 4) for v in pos], size=[round(min(v, 6.0), 4) for v in size],
                 yaw=round(float(fx.get("yaw", 0.0) or 0.0), 3))
        out.append(o)
    return out


# ------------------------------------------------------------------ runtime: joint state, conveyors, handles
class FixtureRuntime:
    """What the fixtures are doing in a running world: each part's joint (plant) or mocap pose (kinematic), how open it
    is, the conveyors (wrap slats, photo-eye stop), and locks (a check can jam a drawer to prove honest failure)."""

    def __init__(self, model, data, built, physics=True):
        import mujoco
        self.mj = mujoco
        self.m, self.d = model, data
        self.physics = physics
        self.parts, self.by_name = [], {}
        for p in built.parts:
            q = dict(p)
            q["bid"] = model.body(p["body"]).id
            q["mid"] = int(model.body_mocapid[q["bid"]])
            q["sid"] = model.site(p["handle"]["site"]).id if p.get("handle") else None
            if physics and q["mid"] < 0:
                j = model.joint(p["joint"]).id
                q["jid"], q["qadr"], q["dadr"] = j, int(model.jnt_qposadr[j]), int(model.jnt_dofadr[j])
                q["aid"] = model.actuator(f"{p['joint']}_act").id if p.get("act") else None
            else:
                q["jid"] = q["qadr"] = q["dadr"] = q["aid"] = None
            q["home_pos"] = data.mocap_pos[q["mid"]].copy() if q["mid"] >= 0 else data.xpos[q["bid"]].copy()
            q["home_quat"] = data.mocap_quat[q["mid"]].copy() if q["mid"] >= 0 else data.xquat[q["bid"]].copy()
            q["kq"] = 0.0                                   # kinematic joint value
            self.parts.append(q)
            if p.get("kind") not in ("slat", "roller"):
                self.by_name[q["name"]] = q
        self.containers = list(built.containers)
        self.surfaces = list(built.surfaces)
        self.belts = []
        for bl in built.belts:
            b = dict(bl)
            b["running"] = abs(b["speed"]) > 1e-6
            b["held"] = False
            b["parts"] = [p for p in self.parts if p["joint"] in bl["joints"]]
            self.belts.append(b)
        self.locked = {}
        self.objects = None                                 # set by the runtime: {name: (qadr) } for the photo-eye

    # ---- joint values
    def q(self, p):
        if p["qadr"] is not None:
            return float(self.d.qpos[p["qadr"]])
        return float(p["kq"])

    def frac(self, p):
        lo, hi = p["range"]
        return float(np.clip((self.q(p) - lo) / (hi - lo), 0.0, 1.0)) if hi > lo else 0.0

    def set_q(self, p, v):
        """Kinematic mode (or a reset): put the part at joint value v."""
        lo, hi = p["range"]
        v = float(np.clip(v, lo, hi))
        if p["qadr"] is not None:
            self.d.qpos[p["qadr"]] = v
            self.d.qvel[p["dadr"]] = 0.0
            return
        p["kq"] = v
        pos, quat = self.pose_at(p, v)
        self.d.mocap_pos[p["mid"]], self.d.mocap_quat[p["mid"]] = pos, quat

    def pose_at(self, p, v):
        """World pose (pos, quat) of the part's body at joint value v (from its closed pose)."""
        R0 = np.zeros(9)
        self.mj.mju_quat2Mat(R0, p["home_quat"])
        R0 = R0.reshape(3, 3)
        ax = R0 @ np.asarray(p["axis"], float)
        if p["type"] == "slide":
            return p["home_pos"] + ax * v, p["home_quat"].copy()
        dq = np.zeros(4)
        self.mj.mju_axisAngle2Quat(dq, ax, v)
        q = np.zeros(4)
        self.mj.mju_mulQuat(q, dq, p["home_quat"])
        return p["home_pos"].copy(), q

    def handle_at(self, p, v):
        """World position of the part's handle at joint value v, and the knob's outward direction there."""
        pos, quat = self.pose_at(p, v)
        R = np.zeros(9)
        self.mj.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        h = p["handle"]
        return pos + R @ np.asarray(h["local"], float), R @ np.asarray(h["out"], float)

    def handle_now(self, p):
        if p["sid"] is None:
            return None, None
        R = self.d.site_xmat[p["sid"]].reshape(3, 3)
        return self.d.site_xpos[p["sid"]].copy(), R @ np.asarray(p["handle"]["out"], float)

    def geoms(self, p):
        return [g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] == p["bid"]]

    # ---- containers
    def box(self, c):
        """A container's current world box: (centre, half, rotation)."""
        if c["body"] is None:
            a = c["yaw"]
            R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
            return np.asarray(c["c"], float), np.asarray(c["h"], float), R
        b = self.m.body(c["body"]).id
        if self.m.body_mocapid[b] >= 0:
            mid = self.m.body_mocapid[b]
            R = np.zeros(9)
            self.mj.mju_quat2Mat(R, self.d.mocap_quat[mid])
            R, p = R.reshape(3, 3), self.d.mocap_pos[mid]
        else:
            R, p = self.d.xmat[b].reshape(3, 3), self.d.xpos[b]
        return p + R @ np.asarray(c["c"], float), np.asarray(c["h"], float), R.copy()

    def inside(self, c, x, margin=0.02):
        cc, h, R = self.box(c)
        loc = R.T @ (np.asarray(x, float) - cc)
        return bool(np.all(np.abs(loc) <= h + margin))

    def container_of(self, x):
        best = None
        for c in self.containers:
            if self.inside(c, x):
                cc, h, _ = self.box(c)
                vol = float(np.prod(h))
                if best is None or vol < best[0]:
                    best = (vol, c)
        return None if best is None else best[1]

    # ---- conveyors
    def belt_tick(self, objects_pos):
        """Physics: wrap slats, drive the belt, stop it while a part is at the photo-eye. objects_pos: {name: xyz}."""
        d = self.d
        for b in self.belts:
            held = None
            if b.get("stop_at") is not None and objects_pos:
                eye, ax = np.asarray(b["stop_at"]), np.asarray(b["axis"])
                for n, x in objects_pos.items():
                    rel = np.asarray(x) - eye
                    along = float(np.dot(rel, ax)) * (1 if b["speed"] >= 0 else -1)
                    side = rel - ax * float(np.dot(rel, ax))
                    if -0.03 < along < 0.25 and np.hypot(side[0], side[1]) < b["width"] / 2 + 0.02 and abs(rel[2]) < 0.15:
                        held = n
                        break
            b["held"] = held
            v = b["speed"] if (b["running"] and held is None) else 0.0
            for p in b["parts"]:
                if p["aid"] is None:
                    continue
                if b["kind"] == "rollers":
                    d.ctrl[p["aid"]] = v / b["radius"]
                else:
                    d.ctrl[p["aid"]] = v
                    w = p.get("wrap")
                    if w is not None:
                        y = w["y0"] + d.qpos[p["qadr"]]
                        if y > w["hi"] + 1e-4:
                            d.qpos[p["qadr"]] -= w["len"]
                        elif y < w["lo"] - 1e-4:
                            d.qpos[p["qadr"]] += w["len"]

    def lock(self, p, on):
        """Jam a part (a check proves the robot reports a blocked drawer honestly): a huge joint friction."""
        if p["jid"] is None:
            self.locked[p["name"]] = bool(on)
            return
        dof = p["dadr"]
        if on:
            self.locked[p["name"]] = float(self.m.dof_frictionloss[dof])
            self.m.dof_frictionloss[dof] = 1e4
        elif p["name"] in self.locked:
            self.m.dof_frictionloss[dof] = self.locked.pop(p["name"])

    def state(self):
        out = []
        for p in self.by_name.values():
            out.append({"name": p["name"], "kind": p["kind"], "label": p["label"], "open": round(self.frac(p), 3),
                        "joint": p["joint"], "type": p["type"], "fixture": p["fixture"], **({"locked": True} if p["name"] in self.locked else {})})
        for b in self.belts:
            out.append({"name": b["name"], "kind": b["kind"], "label": b["label"], "running": bool(b["running"]),
                        "speed": b["speed"], "holding": b["held"]})
        return out
