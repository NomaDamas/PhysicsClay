"""Object kinds beyond the basic table set (issue #48): utensils, tableware, household things, tools, industrial parts,
lab items. Each is a compound of primitives sized like the real thing, with a mass and a friction, so a parallel
gripper can actually hold it (utensil handles are thick enough to pinch, flat things have a raised grip).

    KINDS[kind] = {"size": full (sx, sy, sz) m, "color", "shape": "box" | "cyl" (the bounding collision the task layer
                   reads), "cls": grasp class ("box" | "round" | "long" | "flat"), "mass": kg, "mu": sliding friction,
                   "lim": (min, max) side length}

Long things lie along their local x (handle toward -x); `grip` is where along x a parallel gripper holds them (as a
fraction of the half length from the centre) and `grip_w` how wide they are there. The body frame is the bounding
box's centre. Visual geoms are group 2 (no contact); `collision(kind, half)` lists the plant's contact primitives
(group 3): the planner and the task layer keep the bounding box / cylinder named `<body>_collision`.
"""
import math

# kind: size (sx, sy, sz), colour, bounding shape, grasp class, mass kg, friction, size limits, grip (fraction, width m)
_K = {
    # utensils (lying flat, handle toward -x)
    "spoon": ((0.170, 0.036, 0.018), "silver", "box", "long", 0.035, 0.7, (0.008, 0.40), (-0.30, 0.014)),
    "fork": ((0.180, 0.028, 0.018), "silver", "box", "long", 0.035, 0.7, (0.008, 0.40), (-0.30, 0.014)),
    "knife": ((0.210, 0.024, 0.018), "silver", "box", "long", 0.050, 0.7, (0.008, 0.40), (-0.45, 0.016)),
    "spatula": ((0.290, 0.070, 0.022), "black", "box", "long", 0.080, 0.8, (0.008, 0.40), (-0.40, 0.018)),
    "ladle": ((0.290, 0.085, 0.050), "silver", "box", "long", 0.120, 0.7, (0.008, 0.40), (-0.45, 0.016)),
    "whisk": ((0.260, 0.060, 0.050), "silver", "box", "long", 0.090, 0.7, (0.008, 0.40), (-0.50, 0.018)),
    # tableware and kitchen things
    "glass": ((0.068, 0.068, 0.110), "white", "cyl", "round", 0.20, 0.8, (0.02, 0.40), None),
    "pot": ((0.200, 0.200, 0.120), "steel", "cyl", "round", 1.10, 0.8, (0.04, 0.40), None),
    "cutting_board": ((0.340, 0.230, 0.020), "brown", "box", "flat", 0.60, 0.8, (0.008, 0.45), None),
    "sponge": ((0.100, 0.068, 0.035), "yellow", "box", "box", 0.02, 1.1, (0.01, 0.40), None),
    "towel": ((0.280, 0.200, 0.024), "blue", "box", "flat", 0.15, 1.2, (0.008, 0.45), None),
    "banana": ((0.190, 0.040, 0.040), "yellow", "box", "long", 0.12, 0.9, (0.02, 0.40), (0.0, 0.036)),
    "apple": ((0.075, 0.075, 0.075), "red", "cyl", "round", 0.18, 0.9, (0.02, 0.40), None),
    "jar": ((0.075, 0.075, 0.110), "orange", "cyl", "round", 0.35, 0.8, (0.02, 0.40), None),
    "detergent": ((0.120, 0.075, 0.240), "blue", "box", "box", 0.80, 0.8, (0.02, 0.40), None),
    # living room
    "book": ((0.220, 0.150, 0.030), "red", "box", "flat", 0.40, 0.8, (0.008, 0.40), None),
    "remote": ((0.170, 0.050, 0.022), "black", "box", "long", 0.12, 0.9, (0.008, 0.40), (0.0, 0.050)),
    "toy_car": ((0.110, 0.055, 0.045), "red", "box", "box", 0.10, 0.9, (0.02, 0.40), None),
    "ball": ((0.070, 0.070, 0.070), "orange", "cyl", "round", 0.06, 1.0, (0.02, 0.40), None),
    "shoe": ((0.270, 0.100, 0.100), "brown", "box", "box", 0.35, 1.0, (0.02, 0.40), None),
    "cushion": ((0.400, 0.400, 0.120), "purple", "box", "flat", 0.40, 1.2, (0.02, 0.50), None),
    # tools (lying on the bench)
    "hammer": ((0.300, 0.100, 0.034), "orange", "box", "long", 0.55, 0.8, (0.01, 0.40), (-0.45, 0.026)),
    "wrench_flat": ((0.200, 0.045, 0.012), "steel", "box", "long", 0.25, 0.7, (0.008, 0.40), (-0.25, 0.016)),
    "screwdriver_flat": ((0.190, 0.030, 0.030), "yellow", "box", "long", 0.10, 0.8, (0.01, 0.40), (-0.40, 0.028)),
    "pliers": ((0.180, 0.050, 0.016), "red", "box", "long", 0.20, 0.8, (0.008, 0.40), (-0.35, 0.020)),
    # industrial parts
    "bolt": ((0.070, 0.022, 0.022), "steel", "box", "long", 0.06, 0.6, (0.01, 0.40), (0.15, 0.014)),
    "nut": ((0.030, 0.030, 0.016), "steel", "cyl", "round", 0.02, 0.6, (0.008, 0.40), None),
    "screw": ((0.040, 0.012, 0.012), "steel", "box", "long", 0.01, 0.6, (0.008, 0.40), (0.25, 0.010)),
    "bracket": ((0.090, 0.050, 0.050), "steel", "box", "box", 0.15, 0.6, (0.01, 0.40), None),
    "gear": ((0.080, 0.080, 0.022), "steel", "cyl", "round", 0.25, 0.6, (0.01, 0.40), None),
    "blank": ((0.080, 0.060, 0.040), "steel", "box", "box", 0.55, 0.6, (0.01, 0.40), None),
    "part": ((0.070, 0.070, 0.050), "silver", "cyl", "round", 0.30, 0.6, (0.01, 0.40), None),
    "pcb": ((0.100, 0.070, 0.016), "green", "box", "flat", 0.05, 0.8, (0.008, 0.40), None),
    "chip_reel": ((0.090, 0.090, 0.020), "black", "cyl", "round", 0.08, 0.8, (0.008, 0.40), None),
    # lab
    "tube": ((0.018, 0.018, 0.105), "white", "cyl", "round", 0.02, 0.8, (0.012, 0.40), None),
    "sample_plate": ((0.128, 0.086, 0.016), "white", "box", "flat", 0.05, 0.8, (0.008, 0.40), None),
    "pipette": ((0.200, 0.032, 0.032), "white", "box", "long", 0.08, 0.8, (0.01, 0.40), (-0.10, 0.028)),
    "beaker": ((0.070, 0.070, 0.090), "white", "cyl", "round", 0.12, 0.8, (0.02, 0.40), None),
    "flask_small": ((0.060, 0.060, 0.120), "white", "cyl", "round", 0.10, 0.8, (0.02, 0.40), None),
    # packing / warehouse
    "carton": ((0.160, 0.120, 0.100), "cardboard", "box", "box", 0.25, 0.8, (0.02, 0.40), None),
    "pouch": ((0.140, 0.100, 0.030), "white", "box", "flat", 0.15, 1.0, (0.008, 0.40), None),
    "tape": ((0.090, 0.090, 0.048), "brown", "cyl", "round", 0.18, 0.8, (0.02, 0.40), None),
}

KINDS = {k: {"size": v[0], "color": v[1], "shape": v[2], "cls": v[3], "mass": v[4], "mu": v[5], "lim": v[6],
             "grip": v[7]} for k, v in _K.items()}
ALIASES = {"teaspoon": "spoon", "tablespoon": "spoon", "fork": "fork", "butter knife": "knife", "turner": "spatula",
           "ladle": "ladle", "wine glass": "glass", "water glass": "glass", "saucepan": "pot", "pan": "pot",
           "chopping board": "cutting_board", "board": "cutting_board", "dish towel": "towel", "cloth": "towel",
           "rag": "towel", "remote control": "remote", "toy": "toy_car", "car": "toy_car", "sneaker": "shoe",
           "pillow": "cushion", "spanner_flat": "wrench_flat", "washer_nut": "nut", "hex nut": "nut", "circuit board": "pcb",
           "board_pcb": "pcb", "test tube": "tube", "vial": "tube", "well plate": "sample_plate", "plate96": "sample_plate",
           "microplate": "sample_plate", "parcel": "carton", "package": "carton", "mailer": "pouch", "bag": "pouch",
           "sprocket": "gear", "billet": "blank", "workpiece_blank": "blank", "detergent bottle": "detergent",
           "soap": "detergent", "flask": "flask_small"}
# the basic kinds (pps/world/scene.py) by grasp class
BASIC_CLASS = {"block": "box", "box": "box", "plate": "flat", "mug": "mug", "bowl": "round", "cup": "round", "can": "round",
               "bottle": "round", "fruit": "round", "vase": "round", "steel_plate": "flat"}


def grasp_class(kind):
    """box | round | long | flat | mug (how a parallel gripper holds it; tasks.grasp_plan)."""
    if kind in KINDS:
        return KINDS[kind]["cls"]
    return BASIC_CLASS.get(kind, "box")


def grip(kind, half):
    """Long things: (x offset of the grip point from the centre, width there). None for the others."""
    g = (KINDS.get(kind) or {}).get("grip")
    if not g:
        return None
    return g[0] * half[0], g[1]


# ------------------------------------------------------------------ MJCF
def _f(*v):
    return " ".join(f"{float(x):.5g}" for x in v)


def _q(a):
    """Quaternion (w x y z) of a turn about z by a radians."""
    return f"{math.cos(a / 2):.6g} 0 0 {math.sin(a / 2):.6g}"


def _qy(a):
    return f"{math.cos(a / 2):.6g} 0 {math.sin(a / 2):.6g} 0"


VIS = 'contype="0" conaffinity="0" group="2"'


def _b(pos, half, rgba, extra=""):
    return f'<geom type="box" size="{_f(*half)}" pos="{_f(*pos)}" rgba="{rgba}" {VIS}{extra}/>'


def _c(pos, r, hh, rgba, extra=""):
    return f'<geom type="cylinder" size="{_f(r, hh)}" pos="{_f(*pos)}" rgba="{rgba}" {VIS}{extra}/>'


def _s(pos, r, rgba):
    return f'<geom type="sphere" size="{_f(r)}" pos="{_f(*pos)}" rgba="{rgba}" {VIS}/>'


def _e(pos, half, rgba):
    return f'<geom type="ellipsoid" size="{_f(*half)}" pos="{_f(*pos)}" rgba="{rgba}" {VIS}/>'


def _cx(p0, p1, r, rgba, typ="cylinder"):
    return f'<geom type="{typ}" fromto="{_f(*p0, *p1)}" size="{_f(r)}" rgba="{rgba}" {VIS}/>'


DARK, STEEL, WHITE = "0.13 0.13 0.15 1", "0.62 0.64 0.68 1", "0.94 0.94 0.92 1"


def visual(kind, half, rgba):
    """Visual geoms (group 2) of a kind, body frame at the bounding box's centre."""
    hx, hy, hz = half
    if kind in ("spoon", "ladle"):
        bl = hx * (0.32 if kind == "spoon" else 0.30)           # bowl length (half)
        g = _cx((-hx, 0, -hz + 0.010), (hx - 2 * bl, 0, -hz + 0.006), 0.0045 if kind == "spoon" else 0.006, rgba, "capsule")
        g += _e((hx - bl, 0, -hz + min(hz, 0.012) * 0.6), (bl, hy * 0.95, min(hz, 0.012) * 0.55), rgba)
        if kind == "ladle":
            g += _e((hx - bl, 0, -hz + bl * 0.7), (bl, hy * 0.95, bl * 0.7), rgba)
        return g
    if kind == "fork":
        hl = hx * 0.62
        g = _cx((-hx, 0, -hz + 0.010), (-hx + 2 * hl, 0, -hz + 0.006), 0.0045, rgba, "capsule")
        g += _b((hx - hx * 0.38 + 0.006, 0, -hz + 0.004), (hx * 0.38 - 0.006, hy * 0.6, 0.0018), rgba)
        for k in range(4):
            y = (k - 1.5) * hy * 0.42
            g += _b((hx - 0.018, y, -hz + 0.004), (0.018, 0.0012, 0.0015), rgba)
        return g
    if kind == "knife":
        g = _b((-hx * 0.55, 0, -hz + 0.008), (hx * 0.45, 0.008, 0.007), DARK)
        g += _b((hx * 0.45, 0, -hz + 0.010), (hx * 0.55, hy * 0.75, 0.0012), STEEL)
        return g
    if kind in ("spatula", "whisk"):
        g = _cx((-hx, 0, -hz + 0.012), (0.0, 0, -hz + 0.008), 0.009, DARK, "capsule")
        if kind == "spatula":
            g += _b((hx * 0.5, 0, -hz + 0.004), (hx * 0.5, hy, 0.0025), rgba)
        else:
            g += _e((hx * 0.5, 0, 0.0), (hx * 0.5, hy, hz), "0.75 0.77 0.80 0.55")
        return g
    if kind in ("hammer",):
        g = _cx((-hx, 0, 0), (hx - 0.02, 0, 0), 0.013, "0.55 0.38 0.24 1", "capsule")
        g += _b((hx - 0.02, 0, 0), (0.016, hy, 0.016), STEEL)
        return g
    if kind == "wrench_flat":
        g = _b((0, 0, 0), (hx * 0.78, 0.009, hz), rgba)
        g += _b((hx * 0.85, 0, 0), (hx * 0.15, hy, hz), rgba)
        g += _b((hx * 0.95, 0, 0), (hx * 0.06, hy * 0.35, hz + 0.0005), DARK)
        return g
    if kind == "screwdriver_flat":
        g = _cx((-hx, 0, 0), (-hx * 0.2, 0, 0), hy, rgba, "capsule")
        g += _cx((-hx * 0.2, 0, 0), (hx, 0, 0), 0.0035, STEEL)
        return g
    if kind == "pliers":
        g = ""
        for s in (-1, 1):
            g += _b((-hx * 0.35, s * hy * 0.45, 0), (hx * 0.62, 0.006, hz), rgba)
        g += _b((hx * 0.6, 0, 0), (hx * 0.4, hy * 0.35, hz * 0.8), STEEL)
        return g
    if kind == "remote":
        g = _b((0, 0, 0), (hx, hy, hz), rgba)
        for k in range(6):
            g += _b((-hx * 0.5 + k * hx * 0.22, 0, hz), (0.006, hy * 0.5, 0.0015), "0.5 0.52 0.56 1")
        return g
    if kind == "banana":
        return _cx((-hx, 0, -hz * 0.3), (0, 0, hz * 0.2), hy, rgba, "capsule") + _cx((0, 0, hz * 0.2), (hx, 0, -hz * 0.3), hy, rgba, "capsule")
    if kind == "bolt":
        g = _cx((-hx, 0, 0), (-hx + 0.008, 0, 0), hy, rgba)
        g += _cx((-hx + 0.008, 0, 0), (hx, 0, 0), hy * 0.42, STEEL)
        return g
    if kind == "screw":
        return _cx((-hx, 0, 0), (-hx + 0.004, 0, 0), hy, rgba) + _cx((-hx + 0.004, 0, 0), (hx, 0, 0), hy * 0.35, STEEL)
    if kind == "nut":
        return _c((0, 0, 0), hx, hz, rgba) + _c((0, 0, 0), hx * 0.45, hz + 0.0005, DARK)
    if kind == "gear":
        g = _c((0, 0, 0), hx * 0.86, hz, rgba) + _c((0, 0, 0), hx * 0.25, hz + 0.0005, DARK)
        for k in range(12):
            a = 2 * math.pi * k / 12
            g += f'<geom type="box" size="{_f(hx * 0.08, hx * 0.06, hz)}" pos="{_f(hx * 0.9 * math.cos(a), hx * 0.9 * math.sin(a), 0)}" quat="{_q(a)}" rgba="{rgba}" {VIS}/>'
        return g
    if kind == "bracket":
        t = 0.004
        return _b((0, 0, -hz + t), (hx, hy, t), rgba) + _b((-hx + t, 0, 0), (t, hy, hz), rgba)
    if kind == "part":
        return _c((0, 0, -hz * 0.3), hx, hz * 0.7, rgba) + _c((0, 0, hz * 0.7), hx * 0.55, hz * 0.3, STEEL)
    if kind == "pcb":
        g = _b((0, 0, -hz + 0.0008), (hx, hy, 0.0008), "0.10 0.45 0.22 1")
        g += _b((hx * 0.1, 0, -hz + 0.006), (hx * 0.3, hy * 0.35, 0.004), DARK)
        g += _b((-hx * 0.6, hy * 0.5, -hz + 0.005), (0.006, 0.006, 0.004), "0.8 0.8 0.82 1")
        g += _c((-hx * 0.6, -hy * 0.4, -hz + 0.008), 0.006, 0.007, "0.15 0.3 0.7 1")
        return g
    if kind == "chip_reel":
        return _c((0, 0, 0), hx, hz, rgba) + _c((0, 0, 0), hx * 0.3, hz + 0.0005, WHITE)
    if kind == "tube":
        g = _c((0, 0, -hz * 0.12), hx, hz * 0.88, "0.86 0.92 0.96 0.75")
        g += _c((0, 0, -hz * 0.45), hx * 0.8, hz * 0.4, rgba)
        g += _c((0, 0, hz * 0.88), hx * 1.1, hz * 0.12, "0.85 0.25 0.25 1")
        return g
    if kind == "sample_plate":
        g = _b((0, 0, 0), (hx, hy, hz), rgba)
        for i in range(6):
            for j in range(4):
                g += _c((-hx * 0.75 + i * hx * 0.3, -hy * 0.6 + j * hy * 0.4, hz), 0.004, 0.0008, "0.6 0.6 0.65 1")
        return g
    if kind == "pipette":
        g = _cx((-hx, 0, 0), (hx * 0.2, 0, 0), hy, rgba, "capsule")
        g += _cx((hx * 0.2, 0, 0), (hx, 0, 0), hy * 0.25, "0.85 0.85 0.88 1")
        return g
    if kind in ("beaker", "glass", "flask_small", "jar"):
        a = "0.80 0.88 0.95 0.55" if kind != "jar" else rgba
        g = _c((0, 0, 0), hx, hz, a)
        if kind == "jar":
            g += _c((0, 0, hz), hx * 0.9, 0.006, "0.3 0.3 0.32 1")
        elif kind == "beaker":
            g += _c((0, 0, -hz * 0.4), hx * 0.93, hz * 0.55, "0.40 0.65 0.95 0.6")
        return g
    if kind == "pot":
        g = _c((0, 0, 0), hx * 0.92, hz, rgba) + _c((0, 0, hz), hx * 0.86, 0.002, DARK)
        for s in (-1, 1):
            g += _b((s * hx * 0.98, 0, hz * 0.6), (hx * 0.08, 0.02, 0.006), DARK)
        return g
    if kind == "cutting_board":
        return _b((0, 0, 0), (hx, hy, hz), rgba) + _b((-hx * 0.85, 0, hz), (0.012, hy * 0.25, 0.0008), "0.35 0.24 0.15 1")
    if kind == "towel":
        return _b((0, 0, -hz * 0.25), (hx, hy, hz * 0.75), rgba) + _b((hx * 0.1, 0, hz * 0.6), (hx * 0.85, hy * 0.98, hz * 0.4), rgba)
    if kind == "book":
        return _b((0, 0, 0), (hx, hy, hz), rgba) + _b((0.004, 0, 0), (hx - 0.004, hy - 0.006, hz * 0.85), WHITE)
    if kind == "detergent":
        return (_b((0, 0, -hz * 0.1), (hx, hy, hz * 0.9), rgba) + _c((hx * 0.4, 0, hz * 0.88), hx * 0.25, hz * 0.12, WHITE)
                + _b((-hx * 0.5, 0, hz * 0.55), (hx * 0.12, hy * 0.6, hz * 0.2), rgba))
    if kind == "toy_car":
        g = _b((0, 0, -hz * 0.2), (hx, hy, hz * 0.45), rgba) + _b((-hx * 0.1, 0, hz * 0.5), (hx * 0.5, hy * 0.85, hz * 0.3), "0.6 0.8 0.95 1")
        for sx in (-1, 1):
            for sy in (-1, 1):
                g += f'<geom type="cylinder" size="{_f(hz * 0.4, 0.006)}" pos="{_f(sx * hx * 0.6, sy * hy * 0.95, -hz * 0.55)}" quat="0.7071 0.7071 0 0" rgba="{DARK}" {VIS}/>'
        return g
    if kind == "shoe":
        return (_b((0, 0, -hz * 0.75), (hx, hy, hz * 0.25), DARK) + _e((-hx * 0.25, 0, -hz * 0.2), (hx * 0.7, hy * 0.95, hz * 0.6), rgba)
                + _e((hx * 0.55, 0, -hz * 0.45), (hx * 0.42, hy * 0.9, hz * 0.32), rgba))
    if kind == "cushion":
        return _e((0, 0, 0), (hx, hy, hz), rgba)
    if kind in ("ball", "apple"):
        g = _s((0, 0, 0), hx, rgba)
        if kind == "apple":
            g += _c((0, 0, hx + 0.006), 0.003, 0.008, "0.35 0.25 0.15 1")
        return g
    if kind == "tape":
        return _c((0, 0, 0), hx, hz, rgba) + _c((0, 0, 0), hx * 0.55, hz + 0.0005, "0.9 0.85 0.75 1")
    if kind == "carton":
        return _b((0, 0, 0), (hx, hy, hz), rgba) + _b((0, 0, hz), (0.02, hy, 0.0006), "0.85 0.75 0.55 1")
    if kind == "pouch":
        return _b((0, 0, 0), (hx, hy, hz), rgba) + _b((hx * 0.8, 0, 0), (hx * 0.2, hy, hz * 0.6), rgba)
    if kind == "sponge":
        return _b((0, 0, -hz * 0.25), (hx, hy, hz * 0.75), rgba) + _b((0, 0, hz * 0.75), (hx, hy, hz * 0.25), "0.30 0.62 0.36 1")
    if kind == "blank":
        return _b((0, 0, 0), (hx, hy, hz), rgba)
    return _b((0, 0, 0), (hx, hy, hz), rgba)


def collision(kind, half):
    """The plant's contact primitives for a kind: [(type, size, pos, quat | None)] in the body frame, or None to keep the
    bounding box / cylinder. Long things get a handle the jaws can close on (raised off the floor where it lies)."""
    hx, hy, hz = half
    if kind in ("spoon", "fork", "knife"):
        hl = hx * 0.62
        return [("box", (hl, 0.007, hz * 0.5), (-hx + hl, 0, 0), None),          # handle: 14 mm wide, raised
                ("box", (hx - hl, hy, hz * 0.35), (hx - (hx - hl), 0, -hz * 0.65 + 0.0005), None)]   # bowl / tines / blade
    if kind in ("spatula", "ladle", "whisk"):
        return [("box", (hx * 0.5, 0.009, hz * 0.5 if kind != "ladle" else 0.009), (-hx * 0.5, 0, -hz + (hz * 0.5 if kind != "ladle" else 0.009)), None),
                ("box", (hx * 0.5, hy, hz if kind != "spatula" else 0.004), (hx * 0.5, 0, 0 if kind != "spatula" else -hz + 0.004), None)]
    if kind == "hammer":
        return [("box", (hx - 0.02, 0.013, 0.013), (-0.02, 0, 0), None), ("box", (0.016, hy, 0.016), (hx - 0.02, 0, 0), None)]
    if kind in ("wrench_flat", "pliers"):
        return [("box", (hx * 0.8, 0.010, hz), (-hx * 0.2, 0, 0), None), ("box", (hx * 0.2, hy, hz), (hx * 0.8, 0, 0), None)]
    if kind == "screwdriver_flat":
        return [("box", (hx * 0.4, hy, hz), (-hx * 0.6, 0, 0), None), ("box", (hx * 0.6, 0.004, 0.004), (hx * 0.4, 0, 0), None)]
    if kind == "bolt":
        return [("box", (0.004, hy, hz), (-hx + 0.004, 0, 0), None), ("box", (hx - 0.004, hy * 0.45, hz * 0.45), (0.004, 0, 0), None)]
    if kind == "banana":
        return [("box", (hx, hy * 0.9, hz * 0.8), (0, 0, -hz * 0.2), None)]
    if kind == "bracket":
        t = 0.004
        return [("box", (hx, hy, t), (0, 0, -hz + t), None), ("box", (t, hy, hz), (-hx + t, 0, 0), None)]
    if kind == "pipette":
        return [("box", (hx * 0.6, hy * 0.9, hz * 0.9), (-hx * 0.4, 0, 0), None), ("box", (hx * 0.4, hy * 0.3, hz * 0.3), (hx * 0.6, 0, 0), None)]
    if kind in ("ball", "apple"):
        return [("sphere", (hx,), (0, 0, 0), None)]
    return None
