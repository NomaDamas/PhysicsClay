"""Procedural, seeded worlds (issue #48): homes and industrial sites built from fixtures (pps/world/fixtures.py) and
object kinds (pps/world/objects.py), with clutter dropped in and settled by physics.

    generate(template, seed) -> scene spec {"template", "seed", "surface": {top, x, y, label}, "fixtures": [...],
                                            "objects": [{..., "pos": [x, y, z], "quat": [w, x, y, z], "exact": true,
                                                         "in": container | None}], "room": {"label", "group", ...}}

The robot stands where it always does (pps/world/profiles.py `mount`): the station's work surface has its near edge at
x = 0.22 and is centred on y = 0, so arms are bolted at the station, humanoids stand at the counter and mobile bases
have the floor in front of it to drive on. Everything else (sizes, heights, colours, which side the sink is on, what
is in which drawer, how the clutter fell) changes with the seed; the same seed always gives the same world.
"""
import copy
import functools
import math
import random

import mujoco
import numpy as np

from . import fixtures as F
from . import objects as O

HOME, INDUSTRIAL = "home", "industrial"
ALL_BODIES = ("g1", "panda", "ur5e", "xarm7", "iiwa14", "aloha", "apollo", "stretch3", "tidybot", "spot", "h2")
TABLETOP = ("g1", "panda", "ur5e", "xarm7", "iiwa14", "aloha", "apollo", "stretch3", "tidybot", "spot", "h2")
NO_RIG = tuple(b for b in ALL_BODIES if b != "aloha")       # the ALOHA rig sits on the surface: not on a busy counter


class Gen:
    """Collects fixtures, exact objects and clutter drops for one world."""

    def __init__(self, template, seed):
        self.template, self.seed = template, int(seed)
        self.r = random.Random(f"{template}:{self.seed}")
        self.fx, self.objs, self.drops = [], [], []
        self.surface = None
        self.names = set()

    def u(self, a, b):
        return self.r.uniform(a, b)

    def pick(self, seq):
        return self.r.choice(list(seq))

    def fixture(self, kind, name, pos, size, yaw=0.0, **kw):
        f = dict(kind=kind, name=name, pos=[round(float(v), 4) for v in pos], size=[round(float(v), 4) for v in size],
                 yaw=float(yaw), **kw)
        self.fx.append(f)
        return f

    def _name(self, base):
        n, k = base, 2
        while n in self.names:
            n, k = f"{base}_{k}", k + 1
        self.names.add(n)
        return n

    def obj(self, kind, pos, yaw=0.0, name=None, color=None, size=None, inside=None, movable=True, mass=None):
        """An object placed exactly (pos = its centre); no settling."""
        k = O.KINDS.get(kind)
        from .scene import KINDS as SK
        sz = list(size or (k["size"] if k else SK[kind][0]))
        o = {"name": self._name(name or kind), "kind": kind, "color": color or (k["color"] if k else SK[kind][1]),
             "size": [round(v, 4) for v in sz], "pos": [round(float(v), 4) for v in pos],
             "quat": [round(math.cos(yaw / 2), 6), 0.0, 0.0, round(math.sin(yaw / 2), 6)], "movable": movable, "exact": True}
        if inside:
            o["in"] = inside
        if mass:
            o["mass"] = mass
        self.objs.append(o)
        return o

    def drop(self, region, items, slots=None, yaw=None, jitter=0.02):
        """Clutter: items [(kind, name | None, colour | None)] dropped into `region` (a container name, or a surface box
        {"c": [x, y], "h": [hx, hy], "z": top, "yaw"}) and settled by physics. slots: local (x, y) per item; yaw: local
        yaw per item (radians) or None for random."""
        self.drops.append({"region": region, "items": list(items), "slots": slots, "yaw": yaw, "jitter": jitter})


# ------------------------------------------------------------------ settling (headless physics)
def _obj_body_xml(o, pos, quat):
    kind = o["kind"]
    hx, hy, hz = (v / 2 for v in o["size"])
    k = O.KINDS.get(kind)
    from .. import physics as P
    from .scene import KINDS as SK
    mass = o.get("mass") or (k["mass"] if k else P.OBJ.get(kind, (0.2, 0.8))[0])
    mu = k["mu"] if k else P.OBJ.get(kind, (0.2, 0.8))[1]
    shape = k["shape"] if k else ("cyl" if SK.get(kind, ((0, 0, 0), "", "box"))[2] in ("cyl", "tool") else "box")
    parts = O.collision(kind, (hx, hy, hz))
    kw = f'contype="1" conaffinity="1" condim="4" friction="{mu} 0.01 0.0001"'
    if parts:
        g = ""
        for typ, size, p, _q in parts:
            g += f'<geom type="{typ}" size="{" ".join(f"{s:.5g}" for s in size)}" pos="{" ".join(f"{v:.5g}" for v in p)}" {kw}/>'
    elif shape == "cyl":
        g = f'<geom type="cylinder" size="{hx:.5g} {hz:.5g}" {kw}/>'
    else:
        g = f'<geom type="box" size="{hx:.5g} {hy:.5g} {hz:.5g}" {kw}/>'
    inertia = [mass * (hy * hy + hz * hz) / 3, mass * (hx * hx + hz * hz) / 3, mass * (hx * hx + hy * hy) / 3]
    return (f'<body name="o_{o["name"]}" pos="{" ".join(f"{v:.6g}" for v in pos)}" quat="{" ".join(f"{v:.6g}" for v in quat)}">'
            f'<freejoint/><inertial pos="0 0 0" mass="{mass:.4g}" diaginertia="{inertia[0]:.4g} {inertia[1]:.4g} {inertia[2]:.4g}"/>{g}</body>')


def settle(static_xml, bodies_xml, placed, seconds=1.2, walls=None):
    """Drop `placed` [(object spec, pos, quat)] onto the given fixture geometry and simulate until it rests.
    walls: [(centre, half)] invisible boxes that keep things in a surface region while they fall. -> [(pos, quat, v)]"""
    extra = "".join(f'<geom type="box" size="{h[0]:.4g} {h[1]:.4g} {h[2]:.4g}" pos="{c[0]:.4g} {c[1]:.4g} {c[2]:.4g}" rgba="0 0 0 0"/>'
                    for c, h in (walls or ()))
    objs = "".join(_obj_body_xml(o, p, q) for o, p, q in placed)
    xml = (f'<mujoco><option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>'
           f'<worldbody><geom name="floor" type="plane" size="4 4 0.01"/>{"".join(static_xml)}{"".join(bodies_xml)}{extra}{objs}'
           f'</worldbody></mujoco>')
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    n = int(seconds / 0.002)
    for i in range(n):
        mujoco.mj_step(m, d)
        if i > 250 and i % 50 == 0 and float(np.max(np.abs(d.qvel))) < 0.01:
            break
    out = []
    for o, _, _ in placed:
        b = m.body(f"o_{o['name']}").id
        out.append((d.xpos[b].copy(), d.xquat[b].copy(), float(np.linalg.norm(d.cvel[b][3:]))))
    return out


def _settle_drops(g, built):
    """Run every clutter drop of a generator against its fixture's geometry; adds the settled objects to g.objs."""
    cont = {c["name"]: c for c in built.containers}
    per_fx = {}                                                     # fixture -> (static xml, bodies xml)
    for s in built.static:
        name = s.split('name="fx_', 1)[1].split('"', 1)[0] if 'name="fx_' in s else ""
        per_fx.setdefault(_fx_of(name, g), ([], []))[0].append(s)
    for b in built.bodies:
        name = b.split('name="fx_', 1)[1].split('"', 1)[0]
        per_fx.setdefault(_fx_of(name, g), ([], []))[1].append(b)
    for dr in g.drops:
        reg = dr["region"]
        walls = []
        if isinstance(reg, str):
            c = cont.get(reg)
            if c is None:
                continue
            cc, h, R, z_floor = _cbox(c, built)
            static, bodies = per_fx.get(c["fixture"], ([], []))
            inside = reg
        else:
            a = float(reg.get("yaw", 0.0))
            R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
            hz = 0.15
            cc = np.array([reg["c"][0], reg["c"][1], reg["z"] + hz])
            h = np.array([reg["h"][0], reg["h"][1], hz])
            z_floor = reg["z"]
            static = [f'<geom type="box" size="{h[0] + 0.3:.4g} {h[1] + 0.3:.4g} 0.01" pos="{cc[0]:.4g} {cc[1]:.4g} {reg["z"] - 0.01:.4g}"/>']
            static += reg.get("extra") or []
            bodies = []
            for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):     # surface regions are axis-aligned (yaw 0)
                wc = cc + np.array([sx * (h[0] + 0.01), sy * (h[1] + 0.01), 0.0])
                walls.append((wc, (0.01, h[1] + 0.02, 0.3) if sx else (h[0] + 0.02, 0.01, 0.3)))
            inside = reg.get("name")
        placed, zs = [], z_floor + 0.004
        for i, it in enumerate(dr["items"]):
            kind, name, color, size = (list(it) + [None, None, None])[:4]
            k = O.KINDS.get(kind)
            from .scene import KINDS as SK
            size = list(size or (k["size"] if k else SK[kind][0]))
            o = {"name": g._name(name or kind), "kind": kind, "color": color or (k["color"] if k else SK[kind][1]),
                 "size": [round(v, 4) for v in size], "movable": True, "exact": True}
            if inside:
                o["in"] = inside
            r_xy = max(size[0], size[1]) / 2
            dz_slot = 0.0
            if dr["slots"] and i < len(dr["slots"]):
                lx, ly = dr["slots"][i][:2]
                dz_slot = dr["slots"][i][2] if len(dr["slots"][i]) > 2 else 0.0
                lx += g.u(-dr["jitter"], dr["jitter"])
                ly += g.u(-dr["jitter"], dr["jitter"])
            else:
                lx = g.u(-max(0.0, h[0] - r_xy), max(0.0, h[0] - r_xy))
                ly = g.u(-max(0.0, h[1] - r_xy), max(0.0, h[1] - r_xy))
            yl = dr["yaw"][i] if isinstance(dr["yaw"], (list, tuple)) and i < len(dr["yaw"]) else (dr["yaw"] if isinstance(dr["yaw"], (int, float)) else g.u(-math.pi, math.pi))
            yl += g.u(-0.15, 0.15)
            a0 = math.atan2(R[1, 0], R[0, 0])
            p = cc + R @ np.array([lx, ly, 0.0])
            p[2] = zs + size[2] / 2 + 0.002
            zs += (size[2] + 0.004) if dr["slots"] is None else 0.0
            if dr["slots"] is not None:
                p[2] = z_floor + size[2] / 2 + 0.003 + dz_slot
            yaw = a0 + yl
            placed.append((o, p, np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])))
        if not placed:
            continue
        res = settle(static, bodies, placed, walls=walls)
        for (o, _, _), (p, q, v) in zip(placed, res):
            loc = R.T @ (p - cc)
            if p[2] < z_floor - 0.03 or np.any(np.abs(loc[:2]) > np.asarray(h[:2]) + 0.04) or v > 0.2:
                continue                                         # fell out or still moving: leave it out
            o["pos"] = [round(float(x), 4) for x in p]
            o["quat"] = [round(float(x), 5) for x in q]
            g.objs.append(o)


def _fx_of(geom_name, g):
    best = ""
    for f in g.fx:
        if geom_name.startswith(f["name"] + "_") and len(f["name"]) > len(best):
            best = f["name"]
    return best


def _cbox(c, built):
    """Container world box at the closed pose: (centre, half, R, floor z)."""
    if c["body"] is None:
        a = c["yaw"]
        R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
        cc = np.asarray(c["c"], float)
    else:
        xml = next(b for b in built.bodies if f'name="{c["body"]}"' in b)
        head = xml.split(">", 1)[0]
        pos = np.array([float(v) for v in head.split('pos="', 1)[1].split('"', 1)[0].split()])
        q = np.array([float(v) for v in head.split('quat="', 1)[1].split('"', 1)[0].split()])
        Rm = np.zeros(9)
        mujoco.mju_quat2Mat(Rm, q)
        R = Rm.reshape(3, 3)
        cc = pos + R @ np.asarray(c["c"], float)
    h = np.asarray(c["h"], float)
    return cc, h, R, float(cc[2] - h[2])


# ------------------------------------------------------------------ homes
def kitchen_full(g):
    r = g
    top = round(r.pick((0.88, 0.90, 0.92)), 3)
    d = round(r.u(0.60, 0.66), 3)
    s = r.pick((-1, 1))                                   # sink side (+1 = the robot's left)
    x0 = 0.22
    xc = x0 + d / 2
    cab_h = top - 0.04
    body = r.pick(("white", "cream", "grey"))
    front = r.pick(("white", "sage", "navy", "oak", "walnut", "charcoal"))
    counter = r.pick(("granite", "marble", "butcher", "charcoal"))
    knob = r.pick(("steel", "black", "0.80 0.65 0.30 1"))
    sink_c = round(s * r.u(0.44, 0.50), 3)
    sink_w = round(r.u(0.36, 0.42), 3)
    sink_d = round(min(d - 0.20, r.u(0.34, 0.40)), 3)
    edge = abs(sink_c) - sink_w / 2                        # the inner edge of the sink cut-out
    # base run: (y0, y1, what) from the robot outward on each side
    cab = lambda name, y0, y1, **kw: g.fixture("base_cabinet", name, (xc, (y0 + y1) / 2, 0.0), (abs(y1 - y0), d, cab_h),
                                               color=body, front=front, knob=knob, **kw)
    ds = -s                                               # the drawer side
    cab("center_cabinet", -0.17, 0.17, doors=1, hinge="left" if ds > 0 else "right", door_names=["cabinet_door"],
        door_labels=["the cabinet door in front of me"])
    hs = [0.15, round(r.u(0.20, 0.24), 3)]
    hs.append(round(cab_h - 0.10 - sum(hs) - 0.01, 3))
    cab("drawer_stack", ds * 0.17, ds * 0.57, drawers=hs, drawer_names=["drawer_1", "drawer_2", "drawer_3"],
        drawer_labels=["the top drawer", "the second drawer", "the bottom drawer"])
    cab("sink_cabinet", s * 0.17, s * 0.79, doors=2, door_names=["sink_door_" + ("left" if s < 0 else "right"), "sink_door_" + ("right" if s < 0 else "left")],
        door_labels=["a sink cabinet door", "a sink cabinet door"])
    g.fixture("dishwasher", "dishwasher", (xc, s * 1.09, 0.0), (0.60, d, cab_h), color=r.pick(("steel", "white", "black")))
    cab("stove_cabinet", ds * 0.57, ds * 1.17, drawers=[0.20, round(cab_h - 0.30, 3)], drawer_names=["pot_drawer", "deep_drawer"],
        drawer_labels=["the drawer under the stove", "the deep drawer under the stove"])
    y_lo, y_hi = sorted((s * 1.39, ds * 1.17))
    g.fixture("countertop", "counter", (xc, (y_lo + y_hi) / 2, 0.0), (y_hi - y_lo, d + 0.02, 0.04), top=top, color=counter,
              hole={"c": [-(d + 0.02) / 2 + 0.06 + sink_d / 2 - 0.01, sink_c - (y_lo + y_hi) / 2], "h": [sink_d / 2, sink_w / 2]},
              main="table", main_y=-(y_lo + y_hi) / 2)
    g.fixture("sink", "sink", (x0 + 0.06 + sink_d / 2 - 0.01 + 0.01, sink_c, 0.0), (sink_w, sink_d, round(r.u(0.16, 0.20), 3)), top=top)
    g.fixture("stove", "stove", (xc + 0.02, ds * 0.87, 0.0), (0.58, 0.50), top=top)
    fy = ds * (1.17 + 0.36)
    g.fixture("fridge", "fridge", (x0 + 0.34, fy, 0.0), (0.70, 0.68, round(r.u(1.72, 1.85), 3)), color=r.pick(("steel", "white", "black")),
              hinge="left" if ds < 0 else "right")
    back = x0 + d + 0.01
    g.fixture("box", "back_wall", (back + 0.03, 0.0, 0.0), (4.4, 0.06, 2.4), color="0.93 0.92 0.89 1", part="wall")
    g.fixture("box", "splash", (back - 0.006, 0.0, top), (2.8, 0.012, 0.45), color=r.pick(("0.80 0.86 0.90 1", "0.92 0.90 0.86 1", "0.75 0.80 0.78 1")), part="tiles", collide=False)
    ud = 0.33
    uz = round(top + r.u(0.48, 0.55), 3)
    g.fixture("upper_cabinet", "wall_cabinet", (back - ud / 2, ds * 0.80, uz), (0.80, ud, 0.70), color=body, front=front, knob=knob)
    g.fixture("upper_cabinet", "wall_cabinet_2", (back - ud / 2, s * 0.85, uz), (0.70, ud, 0.70), color=body, front=front, knob=knob)
    g.fixture("shelving", "open_shelf", (back - 0.13, -s * 0.05 + s * 0.10, uz + 0.05), (0.60, 0.24, 0.62), levels=3, color="oak", back=False)
    if r.r.random() < 0.85:
        g.fixture("microwave", "microwave", (back - 0.21, s * 1.13, top), (0.48, 0.36, 0.28), color=r.pick(("charcoal", "white", "steel")))
    g.fixture("floor", "floor_tiles", (0.6, 0.0, 0.0), (4.4, 2.6, 0), color=r.pick(("0.86 0.84 0.80 1", "0.72 0.66 0.58 1", "0.80 0.80 0.82 1")))
    g.surface = {"top": top, "x": [x0, round(x0 + d, 3)], "y": [-round(min(0.60, edge - 0.03), 3), round(min(0.60, edge - 0.03), 3)],
                 "label": "Kitchen counter"}
    # what is where: utensils in the top drawer near its front (spaced so a gripper fits between them, handles toward
    # the robot), cooking tools in the second
    fr = -((d - 0.06) / 2 - 0.012)                          # the drawer interior's front, local to its centre
    g.drop("drawer_1", [("spoon", "spoon", None), ("fork", "fork", None), ("knife", "knife", None)],
           slots=[(fr + 0.105 + r.u(-0.01, 0.01), 0.0), (fr + 0.11, -0.12), (fr + 0.12, 0.12)], yaw=[0.0, 0.0, 0.0], jitter=0.006)
    g.drop("drawer_2", [("spatula", None, None), ("ladle", None, None), ("whisk", None, None)],
           slots=[(fr + 0.16, -0.11), (fr + 0.16, 0.0), (fr + 0.15, 0.11)], yaw=[0.0, 0.0, 0.0], jitter=0.01)
    g.drop("drawer_3", [("pot", None, None), ("jar", None, "orange")], slots=[(-0.05, -0.08), (0.08, 0.09)], jitter=0.01)
    g.drop("sink", [(r.pick(("plate", "bowl")), None, "white"), ("sponge", None, None)], slots=[(0.03, 0.04), (-0.08, -0.10)], jitter=0.015)
    g.drop("pot_drawer", [("cutting_board", None, None), ("towel", "dish_towel", r.pick(("blue", "red", "white")))],
           slots=[(0.0, -0.05), (0.02, 0.10)], yaw=[0.0, 0.0], jitter=0.01)
    back_band = {"c": [x0 + d - 0.17, ds * 0.32], "h": [0.09, 0.22], "z": top}
    items = [("cup", None, r.pick(("blue", "red", "yellow"))), ("bottle", None, r.pick(("green", "blue"))), ("apple", None, None),
             ("glass", None, None), ("bowl", None, "white"), ("banana", None, None), ("jar", None, r.pick(("orange", "red")))]
    r.r.shuffle(items)
    k = 3 + r.r.randint(0, 2)
    g.drop(back_band, items[:k], slots=[(r.u(-0.05, 0.05), -0.16 + 0.16 * i) for i in range(k)], jitter=0.01)
    # a mug and a fruit bowl on the counter near the robot (things to move around)
    g.obj("mug", (x0 + r.u(0.12, 0.18), ds * r.u(0.08, 0.16), top + 0.068), yaw=r.u(-1, 1), color=r.pick(("red", "white", "blue")))
    g.obj("fruit", (x0 + r.u(0.30, 0.36), -ds * r.u(0.02, 0.10), top + 0.035), color=r.pick(("orange", "red", "green")), name="orange")


def dining(g):
    r = g
    top = round(r.u(0.73, 0.77), 3)
    w = round(r.u(1.4, 1.8), 3)
    d = round(r.u(0.80, 0.95), 3)
    x0 = 0.22
    wood = r.pick(("oak", "walnut", "0.85 0.80 0.72 1", "white"))
    g.fixture("table", "dining_table", (x0 + d / 2, 0.0, 0.0), (w, d, top), top=top, color=wood, main="table")
    for k, (cx, cy, yaw) in enumerate([(x0 + d + 0.30, -0.35, 0), (x0 + d + 0.30, 0.35, 0), (x0 + d / 2, w / 2 + 0.32, -90),
                                       (x0 + d / 2, -w / 2 - 0.32, 90)]):
        g.fixture("chair", f"chair_{k + 1}", (cx, cy, 0.0), (0.44, 0.44, 0.9), yaw=yaw, color=r.pick(("oak", "walnut", "charcoal")))
    sb_w = round(r.u(1.2, 1.6), 3)
    side = r.pick((-1, 1))
    g.fixture("base_cabinet", "sideboard", (x0 + 0.20, side * (w / 2 + 0.85), 0.0), (sb_w, 0.45, 0.82), yaw=-90 * side,
              color=wood, front=wood, drawers=[0.16, 0.16], doors=2, drawer_names=["sideboard_drawer_1", "sideboard_drawer_2"],
              drawer_labels=["the top drawer of the sideboard", "the second drawer of the sideboard"],
              door_names=["sideboard_door_a", "sideboard_door_b"], knob="steel")
    g.fixture("box", "sideboard_top", (x0 + 0.20, side * (w / 2 + 0.85), 0.82), (sb_w + 0.02, 0.47, 0.03), yaw=-90 * side, color=wood, part="top")
    g.fixture("floor", "rug", (x0 + d / 2, 0.0, 0.0), (w + 1.0, d + 1.2, 0), color=r.pick(("0.70 0.30 0.28 1", "0.35 0.45 0.60 1", "0.78 0.72 0.60 1")))
    g.fixture("box", "back_wall", (x0 + d + 1.2, 0.0, 0.0), (4.0, 0.06, 2.4), color="0.92 0.90 0.86 1", part="wall")
    g.surface = {"top": top, "x": [x0, round(x0 + d, 3)], "y": [-round(w / 2, 3), round(w / 2, 3)], "label": "Dining table"}
    # place settings along the far side and in front of the robot, a vase and a bottle in the middle
    for k, cy in enumerate((-0.32, 0.32)):
        px = x0 + d - 0.22
        g.obj("plate", (px, cy, top + 0.01), color="white", name=f"plate_{k + 1}")
        g.obj("fork", (px, cy - 0.15, top + 0.009), yaw=0.0, name=f"fork_{k + 1}")
        g.obj("knife", (px, cy + 0.15, top + 0.009), yaw=0.0, name=f"knife_{k + 1}")
        g.obj("glass", (px - 0.13, cy + 0.17, top + 0.055), name=f"glass_{k + 1}")
    g.obj("plate", (x0 + 0.17, r.u(-0.30, -0.20), top + 0.01), color="white", name="plate_3")
    g.obj("spoon", (x0 + 0.13, r.u(0.10, 0.18), top + 0.009), yaw=r.u(-0.4, 0.4), name="spoon")
    g.obj("cup", (x0 + 0.20, r.u(0.24, 0.32), top + 0.05), color=r.pick(("blue", "red", "yellow")), name="cup")
    g.obj("vase", (x0 + d / 2 + 0.05, r.u(-0.06, 0.06), top + 0.12), color=r.pick(("blue", "white", "green")), movable=False, name="vase")
    g.drop({"c": [x0 + d / 2, 0.0], "h": [0.10, 0.45], "z": top}, [("bottle", None, "green"), ("bowl", "fruit_bowl", "white"),
                                                                ("apple", None, None), ("banana", None, None)],
           slots=[(0.0, -0.32), (0.0, 0.30), (r.u(-0.04, 0.04), -0.16), (0.0, 0.14)], jitter=0.01)
    sb_reg = "sideboard_drawer_1"
    g.drop(sb_reg, [("spoon", "teaspoon", None), ("fork", "dessert_fork", None)], slots=[(-0.06, -0.06), (-0.04, 0.07)], yaw=[0.0, 0.0])


def living_room(g):
    r = g
    top = round(r.u(0.40, 0.46), 3)
    w = round(r.u(0.90, 1.15), 3)
    d = round(r.u(0.50, 0.60), 3)
    x0 = 0.22
    wood = r.pick(("oak", "walnut", "white", "charcoal"))
    g.fixture("table", "coffee_table", (x0 + d / 2, 0.0, 0.0), (w, d, top), top=top, color=wood, main="table", shelf=True, thick=0.04)
    g.fixture("sofa", "sofa", (x0 + d + 0.75, 0.0, 0.0), (round(r.u(1.9, 2.3), 3), 0.90, 0.80), color=r.pick(("navy", "sage", "0.55 0.45 0.40 1", "grey")))
    side = r.pick((-1, 1))
    g.fixture("tv_unit", "tv_unit", (x0 + 0.35, side * 1.55, 0.0), (1.4, 0.42, 0.50), yaw=-90 * side, color=wood, front=wood)
    g.fixture("shelving", "bookcase", (x0 + 1.0, -side * 1.6, 0.0), (0.80, 0.30, 1.6), yaw=90 * side, levels=5, color=wood)
    g.fixture("floor", "rug", (x0 + 0.5, 0.0, 0.0), (2.4, 1.8, 0), color=r.pick(("0.62 0.36 0.30 1", "0.40 0.48 0.58 1", "0.82 0.78 0.68 1")))
    g.fixture("floor", "wood_floor", (0.4, 0.0, 0.0), (4.4, 4.0, 0), color="0.66 0.50 0.36 1")
    g.surface = {"top": top, "x": [x0, round(x0 + d, 3)], "y": [-round(w / 2, 3), round(w / 2, 3)], "label": "Coffee table"}
    g.drop({"c": [x0 + d / 2, 0.0], "h": [d / 2 - 0.08, w / 2 - 0.10], "z": top},
           [("remote", None, None), ("book", None, r.pick(("red", "blue", "green"))), ("mug", None, r.pick(("white", "red"))),
            ("cup", None, "yellow"), ("book", "magazine", "white")],
           slots=[(-0.10, -0.25), (0.10, 0.28), (0.05, -0.05), (-0.12, 0.12), (0.10, 0.30)], jitter=0.02)
    # floor clutter for mobile robots: two bands beside the robot, clear of where it stands and drives
    toys = [("ball", None, r.pick(("red", "orange", "blue"))), ("toy_car", None, None), ("block", None, "yellow"), ("block", None, "green"),
            ("shoe", None, "brown"), ("book", "kids_book", "yellow"), ("cushion", None, None), ("shoe", "shoe_2", "brown")]
    r.r.shuffle(toys)
    g.drop({"c": [0.35, 0.95], "h": [0.45, 0.22], "z": 0.0}, toys[:4])
    g.drop({"c": [0.35, -0.95], "h": [0.45, 0.22], "z": 0.0}, toys[4:7])
    g.drop("tv_unit_drawer_left", [("remote", "spare_remote", None), ("book", "manual", "white")], slots=[(0.0, -0.05), (0.0, 0.08)])
    g.drop("bookcase_3", [("book", "novel", "red"), ("book", "atlas", "blue")], slots=[(0.0, -0.15), (0.0, 0.15)], yaw=[0.0, 0.0])


def laundry(g):
    r = g
    top = round(r.u(0.90, 0.94), 3)
    d = 0.66
    x0 = 0.22
    xc = x0 + d / 2
    s = r.pick((-1, 1))
    g.fixture("washer", "washer", (xc, s * 0.35, 0.0), (0.60, d - 0.02, top - 0.05), color="white", label="washer")
    g.fixture("washer", "dryer", (xc, -s * 0.35, 0.0), (0.60, d - 0.02, top - 0.05), color=r.pick(("white", "grey")), label="dryer")
    g.fixture("countertop", "counter", (xc, 0.0, 0.0), (1.6, d + 0.02, 0.035), top=top, color=r.pick(("white", "oak", "granite")), main="table")
    back = x0 + d + 0.01
    g.fixture("box", "back_wall", (back + 0.03, 0.0, 0.0), (3.6, 0.06, 2.4), color="0.90 0.92 0.93 1", part="wall")
    g.fixture("shelving", "wall_shelf", (back - 0.13, 0.0, top + 0.45), (1.2, 0.26, 0.50), levels=2, color="white", back=False)
    g.fixture("bin", "basket", (x0 + 0.34, -s * 0.18, top), (0.42, 0.32, 0.22), color=r.pick(("0.85 0.80 0.70 1", "0.60 0.70 0.85 1", "white")),
              label="the laundry basket", ckind="basket")
    g.fixture("bin", "floor_basket", (-0.05, s * 0.85, 0.0), (0.45, 0.35, 0.30), color="0.85 0.80 0.70 1", label="the basket on the floor", ckind="basket")
    g.fixture("floor", "floor_tiles", (0.5, 0.0, 0.0), (3.6, 3.0, 0), color="0.82 0.82 0.84 1")
    g.surface = {"top": top, "x": [x0, round(x0 + d, 3)], "y": [-0.6, 0.6], "label": "Laundry counter"}
    cols = ["blue", "white", "red", "yellow", "green", "purple"]
    r.r.shuffle(cols)
    g.drop("basket", [("towel", f"towel_{i + 1}", cols[i]) for i in range(3)], yaw=[0.0, 0.3, -0.2])
    g.drop("washer", [("towel", "wet_towel", cols[3])])
    g.drop("floor_basket", [("towel", "towel_4", cols[4]), ("cushion", "pillowcase", "white")])
    g.drop("wall_shelf_1", [("detergent", None, r.pick(("blue", "orange"))), ("detergent", "softener", "pink"), ("bottle", "stain_remover", "white")],
           slots=[(0.0, -0.35), (0.0, 0.0), (0.0, 0.35)], yaw=[0.0, 0.0, 0.0])
    g.obj("towel", (x0 + 0.16, s * 0.25, top + 0.012), yaw=0.0, color=cols[5], name="folded_towel")


# ------------------------------------------------------------------ industrial
def assembly_line(g):
    r = g
    top = round(r.u(0.80, 0.86), 3)
    W = 0.32
    L = 2.25
    x0 = 0.22
    sp = round(r.pick((-1, 1)) * r.u(0.06, 0.09), 3)          # belt speed and direction (+ = toward the robot's left)
    eye = round(math.copysign(0.02, sp), 3)
    g.fixture("conveyor", "conveyor", (x0 + 0.02 + W / 2, 0.0, 0.0), (L, W, 0), top=top, speed=sp, stop_at=eye, main="table",
              frame=r.pick(("grey", "0.25 0.45 0.65 1", "0.35 0.55 0.35 1")), label="the conveyor")
    s = r.pick((-1, 1))
    g.fixture("box", "bin_stand", (0.12, s * 0.38, 0.0), (0.26, 0.20, top - 0.12), color="grey", part="stand", collide=True)
    g.fixture("bin", "out_bin", (0.12, s * 0.38, top - 0.12), (0.26, 0.20, 0.10), color=r.pick(("blue", "orange", "green")),
              label="the finished-parts bin", ckind="bin")
    g.fixture("table", "assembly_bench", (x0 + 0.40, -s * 1.15, 0.0), (0.9, 0.6, 0.9), top=0.9, color="0.75 0.77 0.80 1", legs="grey", mat="esd")
    g.fixture("parts_tray", "bench_tray", (x0 + 0.40, -s * 1.15, 0.9), (0.30, 0.20, 0.04), nx=2, ny=3)
    g.fixture("box", "guard", (x0 + 0.95, 0.0, 0.0), (2.4, 0.04, 1.2), color="0.95 0.80 0.10 0.45", part="fence", collide=False)
    g.fixture("box", "control_box", (x0 + 0.02 + W + 0.12, -math.copysign(1.0, sp) * 0.95, top - 0.2), (0.25, 0.15, 0.35), color="0.85 0.86 0.88 1", part="panel")
    g.fixture("floor", "floor_lines", (0.4, 0.0, 0.0), (2.8, 1.1, 0), color="0.98 0.82 0.12 1")
    g.fixture("floor", "floor_grey", (0.4, 0.0, 0.0), (4.4, 3.6, 0), color="0.55 0.57 0.58 1")
    g.surface = {"top": top, "x": [x0, round(x0 + 0.02 + W + 0.02, 3)], "y": [-0.55, 0.55], "label": "Conveyor station"}
    # parts upstream on the belt: they ride to the photo-eye one at a time
    up = -math.copysign(1.0, sp)
    kinds = [("blank", None, None), ("gear", None, None), ("bracket", None, None), ("part", None, None), ("blank", "blank_2", None)]
    r.r.shuffle(kinds)
    xs = x0 + 0.02 + W / 2
    for k, (kind, name, _color) in enumerate(kinds[:3]):
        size = O.KINDS[kind]["size"]
        g.obj(kind, (xs + r.u(-0.03, 0.03), up * (0.38 + 0.30 * k), top + size[2] / 2 + 0.001), yaw=r.u(-0.3, 0.3), name=name)
    g.drop("bench_tray_1", [("bolt", None, None), ("bolt", "bolt_2", None)])
    g.drop("bench_tray_2", [("nut", None, None), ("nut", "nut_2", None), ("nut", "nut_3", None)])
    g.drop("bench_tray_4", [("screw", None, None), ("screw", "screw_2", None)])
    g.obj("screwdriver_flat", (x0 + 0.30, -s * 1.15 + 0.22, 0.9 + 0.016), yaw=0.3, name="screwdriver")


def warehouse(g):
    r = g
    top = round(r.u(0.74, 0.80), 3)
    x0 = 0.22
    d = 0.62
    s = r.pick((-1, 1))
    g.fixture("table", "pick_table", (x0 + d / 2, 0.0, 0.0), (1.2, d, top), top=top, color="0.70 0.72 0.75 1", legs="grey", main="table")
    tc = r.pick(("blue", "orange", "grey", "green"))
    g.fixture("bin", "tote", (x0 + 0.27, s * 0.27, top), (0.46, 0.36, 0.15), color=tc, label="the tote", ckind="tote")
    g.fixture("bin", "tote_out", (x0 + 0.27, -s * 0.27, top), (0.46, 0.36, 0.15), color=r.pick(("blue", "orange", "grey")), label="the empty tote", ckind="tote")
    for k, (cx, cy, yaw) in enumerate(((x0 + d + 0.75, 0.0, 0), (0.6, 1.55, -90), (0.6, -1.55, 90))):
        g.fixture("rack", f"rack_{k + 1}", (cx, cy, 0.0), (2.2, 0.80, 2.0), yaw=yaw, levels=[0.12, 0.75, 1.40], bays=2)
    g.fixture("pallet", "pallet", (0.35, s * 0.95, 0.0), (1.0, 0.8, 0))
    g.fixture("floor", "aisle_lines", (0.4, 0.0, 0.0), (2.6, 1.6, 0), color="0.98 0.82 0.12 1")
    g.fixture("floor", "concrete", (0.6, 0.0, 0.0), (4.4, 4.4, 0), color="0.62 0.62 0.60 1")
    g.surface = {"top": top, "x": [x0, round(x0 + d, 3)], "y": [-0.6, 0.6], "label": "Pick table"}
    small = [0.07, 0.055, 0.05]
    items = [("box", "small_box", None, small), ("pouch", None, r.pick(("white", "blue"))), ("block", "foam_block", "yellow"),
             ("tape", None, None), ("carton", "small_carton", None, [0.10, 0.065, 0.05]), ("block", "foam_block_2", "orange"),
             ("box", "small_box_2", None, small)]
    r.r.shuffle(items)
    corners = [(-0.095, -0.14), (0.095, -0.14), (-0.095, 0.14), (0.095, 0.14)]
    r.r.shuffle(corners)                                    # one layer, the box in the middle (a hand fits around it)
    g.drop("tote", items[:3] + [("box", "box", None, [0.065, 0.055, 0.05])], slots=corners[:3] + [(0.0, 0.0)], jitter=0.012)
    stack = [("box", f"case_{i + 1}", None) for i in range(6)]
    g.drop({"c": [0.35, s * 0.95], "h": [0.30, 0.42], "z": 0.122}, stack,
           slots=[(-0.07, -0.06), (0.07, -0.06), (-0.07, 0.06), (0.07, 0.06), (0.0, -0.05, 0.082), (0.0, 0.06, 0.082)], yaw=[0.0] * 6, jitter=0.004)
    deck = 0.75 + 0.012
    g.drop({"c": [x0 + d + 0.75, 0.55], "h": [0.30, 0.40], "z": deck}, [("carton", f"stock_{i + 1}", None) for i in range(3)],
           slots=[(0, -0.25), (0, 0.0), (0, 0.25)])


def machine_tending(g):
    r = g
    top = round(r.u(0.84, 0.90), 3)
    x0 = 0.22
    d = 0.66
    s = r.pick((-1, 1))
    g.fixture("table", "station", (x0 + d / 2, 0.0, 0.0), (1.4, d, top), top=top, color="0.62 0.64 0.68 1", legs="grey", main="table")
    g.fixture("cnc", "cnc", (x0 + 0.36, s * 0.38, top), (0.46, 0.40, 0.58), color=r.pick(("0.30 0.34 0.38 1", "0.85 0.86 0.88 1")),
              slide=s)
    g.fixture("bin", "raw_tray", (x0 + 0.20, -s * 0.18, top), (0.26, 0.20, 0.05), color="steel", label="the raw blanks tray", ckind="tray")
    g.fixture("bin", "done_tray", (x0 + 0.20, -s * 0.44, top), (0.26, 0.20, 0.05), color="green", label="the finished parts tray", ckind="tray")
    g.fixture("box", "chip_bin", (x0 + 0.45, s * 1.0, 0.0), (0.4, 0.4, 0.5), color="0.35 0.38 0.42 1", part="bin")
    g.fixture("box", "tool_cabinet", (x0 + 0.5, -s * 1.15, 0.0), (0.6, 0.45, 1.2), color="0.75 0.15 0.15 1", part="cabinet")
    g.fixture("floor", "floor_lines", (0.4, 0.0, 0.0), (2.4, 1.6, 0), color="0.98 0.82 0.12 1")
    g.fixture("floor", "concrete", (0.5, 0.0, 0.0), (4.4, 4.0, 0), color="0.58 0.60 0.60 1")
    g.surface = {"top": top, "x": [x0, round(x0 + d, 3)], "y": [-0.6, 0.6], "label": "Machine-tending station"}
    vise_c = np.array([x0 + 0.36 + 0.04, s * 0.38, top + 0.10 + 0.03 + 0.02])
    g.obj("blank", tuple(vise_c), yaw=math.pi / 2, name="part_in_vise", color="steel", size=[0.07, 0.05, 0.04])
    g.drop("raw_tray", [("blank", None, None), ("blank", "blank_2", None)], slots=[(0.0, -0.05), (0.0, 0.05)], yaw=[0.0, 0.0])


def electronics_bench(g):
    r = g
    top = round(r.u(0.74, 0.78), 3)
    x0 = 0.22
    d = 0.70
    s = r.pick((-1, 1))
    g.fixture("table", "bench", (x0 + d / 2, 0.0, 0.0), (1.4, d, top), top=top, color="0.88 0.86 0.80 1", legs="grey", main="table", mat="esd")
    g.fixture("parts_tray", "tray", (x0 + 0.24, s * 0.30, top), (0.30, 0.20, 0.04), nx=2, ny=3, color=r.pick(("0.25 0.3 0.4 1", "0.6 0.15 0.15 1")))
    g.fixture("box", "monitor", (x0 + d - 0.12, 0.0, top), (0.60, 0.05, 0.40), color="black", part="screen", collide=True)
    g.fixture("box", "multimeter", (x0 + 0.45, -s * 0.40, top), (0.10, 0.16, 0.05), color="safety", part="meter")
    g.fixture("box", "soldering_station", (x0 + 0.52, s * 0.52, top), (0.14, 0.16, 0.10), color="0.2 0.4 0.75 1", part="iron")
    g.fixture("shelving", "parts_shelf", (x0 + d + 0.12, 0.0, top), (1.2, 0.25, 0.60), levels=3, color="grey")
    g.fixture("floor", "floor", (0.5, 0.0, 0.0), (3.6, 3.0, 0), color="0.40 0.44 0.50 1")
    g.surface = {"top": top, "x": [x0, round(x0 + d, 3)], "y": [-0.6, 0.6], "label": "Electronics bench"}
    g.drop("tray_1", [("screw", None, None), ("screw", "screw_2", None), ("screw", "screw_3", None)])
    g.drop("tray_2", [("nut", None, None), ("nut", "nut_2", None)])
    g.drop("tray_3", [("bolt", None, None)])
    g.drop("tray_5", [("chip_reel", None, None)])
    g.obj("pcb", (x0 + 0.17, r.u(-0.08, 0.04), top + 0.009), yaw=r.u(-0.3, 0.3), name="pcb")
    g.obj("pcb", (x0 + 0.30, -s * 0.15, top + 0.009), yaw=r.u(-0.3, 0.3), name="pcb_2", color="blue")
    g.obj("screwdriver_flat", (x0 + 0.14, -s * r.u(0.22, 0.30), top + 0.016), yaw=r.u(-0.2, 0.2), name="screwdriver")
    g.obj("pliers", (x0 + 0.24, -s * 0.46, top + 0.009), yaw=0.2, name="pliers")


def lab(g):
    r = g
    top = round(r.u(0.88, 0.92), 3)
    x0 = 0.22
    d = 0.66
    s = r.pick((-1, 1))
    g.fixture("table", "lab_bench", (x0 + d / 2, 0.0, 0.0), (1.5, d, top), top=top, color="labwhite", legs="grey", main="table")
    g.fixture("tube_rack", "rack", (x0 + 0.22, s * 0.16, top), (0.3, 0.07, 0.06), n=6, label="the tube rack", color=r.pick(("teal", "0.85 0.35 0.35 1", "0.3 0.5 0.85 1")))
    g.fixture("tube_rack", "source_rack", (x0 + 0.22, -s * 0.22, top), (0.3, 0.07, 0.06), n=6, label="the yellow source rack", color="0.95 0.75 0.20 1")
    g.fixture("centrifuge", "centrifuge", (x0 + 0.50, -s * 0.57, top), (0.3, 0.3, 0.2))
    g.fixture("shelving", "reagent_shelf", (x0 + d - 0.10, 0.0, top + 0.20), (1.3, 0.22, 0.6), levels=3, color="labwhite", back=True)
    g.fixture("box", "fume_hood", (x0 + 0.40, s * 1.2, 0.0), (0.9, 0.7, 2.0), color="0.85 0.87 0.9 1", part="hood")
    g.fixture("floor", "floor", (0.5, 0.0, 0.0), (3.8, 3.2, 0), color="0.86 0.88 0.90 1")
    g.surface = {"top": top, "x": [x0, round(x0 + d, 3)], "y": [-0.65, 0.65], "label": "Lab bench"}
    caps = ["0.85 0.25 0.25 1", "0.25 0.45 0.85 1", "0.3 0.7 0.35 1", "0.95 0.75 0.2 1"]
    n = 2 + r.r.randint(0, 1)
    slots = list(range(6))
    r.r.shuffle(slots)
    for i in range(n):
        g.drop(f"source_rack_{slots[i] + 1}", [("tube", f"tube_{i + 1}", None)], slots=[(0.0, 0.0)], yaw=[0.0], jitter=0.0)
    g.drop("rack_2", [("tube", "tube_done", None)], slots=[(0.0, 0.0)], yaw=[0.0], jitter=0.0)
    g.obj("pipette", (x0 + 0.16, r.u(-0.05, 0.05), top + 0.016), yaw=r.u(-0.2, 0.2), name="pipette")
    g.obj("sample_plate", (x0 + 0.40, s * 0.45, top + 0.008), yaw=0.0, name="sample_plate")
    g.obj("sample_plate", (x0 + 0.40, s * 0.45, top + 0.025), yaw=0.05, name="sample_plate_2", color="0.9 0.95 1 1")
    g.drop("reagent_shelf_1", [("beaker", None, None), ("flask_small", "flask", None), ("bottle", "reagent", "orange")],
           slots=[(0.0, -0.4), (0.0, 0.0), (0.0, 0.4)], yaw=[0.0, 0.0, 0.0])
    _ = caps


def packing(g):
    r = g
    top = round(r.u(0.78, 0.84), 3)
    x0 = 0.22
    d = 0.66
    s = r.pick((-1, 1))
    g.fixture("table", "packing_table", (x0 + d / 2, 0.0, 0.0), (1.5, d, top), top=top, color="0.80 0.66 0.48 1", legs="grey", main="table")
    g.fixture("bin", "carton", (x0 + 0.30, s * 0.28, top), (0.36, 0.28, 0.20), color="cardboard", label="the open carton", ckind="carton",
              wall=0.005, flaps=True)
    g.fixture("bin", "product_tote", (x0 + 0.26, -s * 0.30, top), (0.36, 0.28, 0.12), color=r.pick(("blue", "grey", "green")), label="the product tote", ckind="tote")
    g.fixture("scale", "scale", (x0 + 0.48, -s * 0.02, top), (0.26, 0.24, 0.04))
    g.fixture("box", "tape_dispenser", (x0 + 0.55, s * 0.60, top), (0.10, 0.18, 0.10), color="0.75 0.15 0.15 1", part="dispenser")
    g.fixture("box", "flat_cartons", (x0 + d - 0.10, -s * 0.55, top), (0.45, 0.30, 0.08), color="cardboard", part="stack")
    g.fixture("roller_conveyor", "infeed", (x0 + d + 0.38, 0.0, 0.0), (1.8, 0.45, 0), top=0.78, speed=-s * 0.12, label="the infeed rollers")
    g.fixture("floor", "floor", (0.5, 0.0, 0.0), (3.8, 3.2, 0), color="0.70 0.70 0.68 1")
    g.surface = {"top": top, "x": [x0, round(x0 + d, 3)], "y": [-0.65, 0.65], "label": "Packing table"}
    items = [("pouch", None, "white"), ("carton", "item_box", None), ("tape", None, None), ("bottle", None, r.pick(("green", "blue"))),
             ("can", None, "silver"), ("book", None, "red")]
    r.r.shuffle(items)
    g.drop("product_tote", items[:4])
    g.drop({"c": [x0 + d + 0.38, s * 0.6], "h": [0.12, 0.25], "z": 0.78}, [("carton", "inbound_1", None), ("carton", "inbound_2", None)],
           slots=[(0.0, -0.12), (0.0, 0.14)], jitter=0.01)


ROOMS = {
    "kitchen_full": {"label": "Full kitchen", "group": HOME, "gen": kitchen_full, "robots": NO_RIG,
                     "summary": "Counter run with drawers and doors, a sink, a stove, a fridge, a microwave, wall cabinets and clutter."},
    "dining": {"label": "Dining room", "group": HOME, "gen": dining, "robots": TABLETOP,
               "summary": "A set table with plates, cutlery and glasses, chairs and a sideboard with drawers."},
    "living_room": {"label": "Living room", "group": HOME, "gen": living_room, "robots": TABLETOP,
                    "summary": "Coffee table, sofa, TV unit with drawers, bookcase and toys on the floor for mobile robots."},
    "laundry": {"label": "Laundry", "group": HOME, "gen": laundry, "robots": NO_RIG,
                "summary": "Washer and dryer with doors under a counter, laundry baskets with towels, detergent on a shelf."},
    "assembly_line": {"label": "Assembly line", "group": INDUSTRIAL, "gen": assembly_line, "robots": NO_RIG,
                      "summary": "A running belt conveyor that stops parts at the photo-eye, a parts bin and an assembly bench."},
    "warehouse": {"label": "Warehouse", "group": INDUSTRIAL, "gen": warehouse, "robots": TABLETOP,
                  "summary": "Bin picking from a tote at a pick table, racks with bays, a pallet of stacked cases."},
    "machine_tending": {"label": "Machine tending", "group": INDUSTRIAL, "gen": machine_tending, "robots": NO_RIG,
                        "summary": "A CNC enclosure with a sliding door and a part in the vise, raw and finished trays."},
    "electronics_bench": {"label": "Electronics bench", "group": INDUSTRIAL, "gen": electronics_bench, "robots": TABLETOP,
                          "summary": "An ESD bench with PCBs, a parts tray of screws and nuts, a screwdriver and pliers."},
    "lab": {"label": "Lab bench", "group": INDUSTRIAL, "gen": lab, "robots": TABLETOP,
            "summary": "Test-tube racks with tubes, a centrifuge with a hinged lid, a pipette and sample plates."},
    "packing": {"label": "Packing station", "group": INDUSTRIAL, "gen": packing, "robots": TABLETOP,
                "summary": "An open carton, a tote of products, a scale and a roller conveyor bringing boxes."},
}


def listing():
    return [{"id": k, "label": v["label"], "group": v["group"], "summary": v["summary"], "robots": list(v["robots"])} for k, v in ROOMS.items()]


@functools.lru_cache(maxsize=48)
def _generate(template, seed):
    g = Gen(template, seed)
    ROOMS[template]["gen"](g)
    g.fx = F.normalize(g.fx)
    built = F.build(g.fx)
    _settle_drops(g, built)
    room = ROOMS[template]
    return {"template": template, "seed": int(seed), "surface": g.surface, "fixtures": g.fx, "objects": g.objs,
            "room": {"label": room["label"], "group": room["group"]}}


def generate(template, seed=0):
    """(template, seed) -> scene spec (a copy: callers may change it)."""
    if template not in ROOMS:
        raise ValueError(f"unknown world template '{template}'")
    try:
        seed = int(seed) % 1_000_000
    except (TypeError, ValueError):
        seed = 0
    return copy.deepcopy(_generate(template, seed))
