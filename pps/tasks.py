"""Intent -> grounded robot task.  Every task is compiled to a world-space hand path, pre-checked with IK
(the physics verdict), then executed; the robot reports in one sentence what it will do or why it can't."""
import json
import os
import re
import sys
import time

import mujoco
import numpy as np

from .manip import FixtureTasks
from .robot_runtime import HANDS, ROOT, slerp
from .world import objects as wobj

TABLE_TOP = 0.74
TABLE_X = (0.22, 0.82)
TABLE_Y = (-0.50, 0.50)
REGIONS = {  # robot frame: x forward, y left
    "right": (0.31, -0.22), "left": (0.31, 0.22), "center": (0.34, 0.0),
    "front": (0.27, 0.0), "back": (0.66, 0.0),
}
HOVER = 0.08
PLACE_GAP = 0.004        # physics: the object's bottom this far above the surface when the hand stops to let go
CRIT_TOL = 0.015         # physics: the plan must bring the jaws this close to the grasp and the drop point
RELEASE_OPEN = 0.35      # physics: letting go, the jaws open this fraction of their travel past where they came free
LIFT = 0.08              # physics: lift this far off the surface (higher only to clear what it passes over)
HANDOVER_Z = 0.14        # two-arm handover: the hands meet this far above the surface ...
HANDOVER_END = 0.022     # ... each gripper holds the object this far in from its end ...
HANDOVER_GAP = 0.16      # ... and the two grippers stay at least this far apart
W_TURN = 1.0            # physics: rad/s, how fast the hand turns along a path (a wrist flipping at full speed jerks things)
# physics side grasps (humanoid palms, Stretch): (heading: from the shoulder / straight ahead / in between, pitch down)
SIDE_WAYS = ((0, 0.0), (1, 0.0), (2, 0.0), (1, 0.6), (0, 0.6), (1, 1.0))
SURFACE = "table"        # what the work surface is called (pps.world templates: counter, shelf, floor mat)
SKILL_DIR = ROOT / "recordings" / "skills"
ROBOT_BODIES_ROOT = "pelvis"
LEGACY_OBJS = (("mug", "mug_collision", "cyl"), ("red_block", "red_block_collision", "box"), ("green_block", "green_block_collision", "box"))
LEGACY_OBSTACLES = (("vase", "vase_collision"), ("blue_block", "blue_block_collision"))
LEGACY_KEYS = {"mug": ("mug", "cup", "컵", "머그"), "red_block": ("red", "빨간"), "green_block": ("green", "초록", "녹색"), "blue_block": ("blue", "파란"),
               "vase": ("vase", "꽃병", "화병")}
# words people use -> what they mean (object kinds and colours of scenes built on the fly)
WORDS = {"cup": ("cup", "컵", "잔"), "mug": ("mug", "머그", "컵"), "block": ("block", "cube", "블록", "큐브"), "box": ("box", "상자", "박스"),
         "bowl": ("bowl", "그릇"), "bottle": ("bottle", "병"), "can": ("can", "캔"), "plate": ("plate", "dish", "접시"),
         "fruit": ("fruit", "apple", "과일", "사과"), "vase": ("vase", "꽃병", "화병"),
         "red": ("red", "빨간"), "green": ("green", "초록", "녹색"), "blue": ("blue", "파란"), "yellow": ("yellow", "노란"),
         "orange": ("orange", "주황"), "purple": ("purple", "보라"), "white": ("white", "흰", "하얀"), "black": ("black", "검은"),
         "pink": ("pink", "분홍")}


def apply_surface(surface, regions):
    """The world changed (pps.world): move the work surface. Feature modules imported TABLE_TOP & co. by value, so the
    new values are pushed into every pps.* module that holds them (REGIONS is updated in place)."""
    global TABLE_TOP, TABLE_X, TABLE_Y, SURFACE
    TABLE_TOP, TABLE_X, TABLE_Y = float(surface["top"]), tuple(surface["x"]), tuple(surface["y"])
    SURFACE = {"kitchen": "counter", "shelf": "shelf", "floor": "floor mat"}.get(surface.get("template"), "table")
    REGIONS.clear()
    REGIONS.update({k: tuple(v) for k, v in regions.items()})
    me = sys.modules[__name__]
    for mod in list(sys.modules.values()):
        if mod is me or not getattr(mod, "__name__", "").startswith("pps."):
            continue
        for k, v in (("TABLE_TOP", TABLE_TOP), ("TABLE_X", TABLE_X), ("TABLE_Y", TABLE_Y)):
            if k in vars(mod):
                setattr(mod, k, v)


def densify(pts, step=0.035):
    out = [np.asarray(pts[0], float)]
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


def _drain(gen):
    """Run a resumable step (a generator that yields between slices of work) to the end -> its return value."""
    while True:
        try:
            next(gen)
        except StopIteration as e:
            return e.value


def _seg_dist(p, a, b):
    """Distance from point p to the segment a-b (any dimension)."""
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    s = float(np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) or 1.0), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + s * ab)))


def hand_disc(hang, foot, n=8):
    """A hand as points relative to its TCP: the TCP, and a ring of radius `foot` `hang` m below it (a gripper whose
    shape isn't known: the kinematic point hand, the unit tests)."""
    ring = [[foot * np.cos(a), foot * np.sin(a), -hang] for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]
    return np.array([[0.0, 0.0, 0.0], [0.0, 0.0, -hang]] + ring)


def path_object_hits(pts, bodies, hand, label="my hand", carry=None, margin=0.01):
    """Per path point: why the hand (or what it carries) would run into an object there, or None.

    hand: points of the hand relative to its TCP (world axes; Tasks.hand_points: the corners of the gripper's collision
    geoms as it is held, or hand_disc). bodies: [(centre xyz, radius, half height, name)] as upright cylinders. carry:
    (name, radius, half height, how far its centre hangs below the TCP) of what the hand holds. The wording matches the
    P2 trajectory checks (pps/traj.py): 'my hand would hit the mug'."""
    P = np.asarray(pts, float).reshape(-1, 3)
    H = np.asarray(hand, float).reshape(-1, 3)
    out = [None] * len(P)
    for c, r, h, n in bodies:
        top = float(c[2]) + h
        Q = P[:, None, :] + H[None, :, :]                                         # every hand point at every path point
        inside = (np.hypot(Q[..., 0] - c[0], Q[..., 1] - c[1]) < r + margin) & (Q[..., 2] < top + margin)
        hit = inside.any(axis=1)
        if carry:
            hit_c = (np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) < r + carry[1] + margin) & (P[:, 2] - carry[3] - carry[2] < top + margin)
        for i in range(len(P)):
            if out[i] is None and hit[i]:
                out[i] = f"{label} would hit {n}"
            elif out[i] is None and carry and hit_c[i]:
                out[i] = f"{carry[0]} would hit {n}"
    return out


def lift_path(pts, bodies, hand, margin=0.05, room=0.03):
    """The path raised over the bodies it would run into, `margin` over their tops within `room` of their outlines, with
    45° ramps up and down (1 cm lower per cm further out), unchanged elsewhere (the hand turns a little along a path, so
    both are generous). hand and bodies as in path_object_hits. -> (points, names of the bodies it now goes over)."""
    P = np.array(pts, float).reshape(-1, 3)
    H = np.asarray(hand, float).reshape(-1, 3)
    need = np.full(len(P), -np.inf)
    names = []
    for c, r, h, n in bodies:
        reach = r + room
        Q = P[:, None, :2] + H[None, :, :2]
        out = np.maximum(np.hypot(Q[..., 0] - c[0], Q[..., 1] - c[1]) - reach, 0.0)    # how far outside its footprint
        z = (float(c[2]) + h + margin - H[None, :, 2] - out).max(axis=1)                # TCP height each point needs
        if not np.any(P[:, 2] < z - 1e-9):
            continue
        names.append(n)
        need = np.maximum(need, z)
    P[:, 2] = np.maximum(P[:, 2], need)
    return P, names


class Tasks(FixtureTasks):
    def __init__(self, rt, kinds=None):
        """kinds: {object name: kind} from the world spec (scenes built on the fly), used to understand 'the cup'."""
        self.rt = rt
        m, d = rt.model, rt.data
        self.hands = list(getattr(rt, "hands", HANDS))
        self.kinds = dict(kinds or {})
        self.plant = getattr(rt, "plant", None)        # physics (pps/physics.py): objects are free bodies, grasps by contact
        self.objs = {}
        self.legacy = False
        for name, geom, kind in LEGACY_OBJS:          # the UX 2.0 scene (scene_pc.xml)
            try:
                b, g = m.body(name).id, m.geom(geom).id
            except KeyError:
                continue
            self.legacy = True
            size = m.geom_size[g]
            self.objs[name] = self._obj(b, g, float(size[0]), float(size[1] if kind == "cyl" else size[2]))
        self.obstacles = {}
        for name, geom in LEGACY_OBSTACLES:
            try:
                vg = m.geom(geom).id
            except KeyError:
                continue
            sz = m.geom_size[vg]
            box = m.geom_type[vg] == mujoco.mjtGeom.mjGEOM_BOX
            self.obstacles[name] = {"pos": d.geom_xpos[vg].copy(), "r": float(max(sz[0], sz[1]) if box else sz[0]),
                                    "h": float(sz[2] if box else sz[1])}
        for b in range(1, m.nbody):                   # scenes built on the fly: obj_<name> (mocap) / obs_<name> (fixed)
            bname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if not bname.startswith(("obj_", "obs_")):
                continue
            try:
                g = m.geom(f"{bname}_collision").id
            except KeyError:
                continue
            sz = m.geom_size[g]
            box = m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
            r, hh = float(max(sz[0], sz[1]) if box else sz[0]), float(sz[2] if box else sz[1])
            if bname.startswith("obj_"):
                self.objs[bname[4:]] = self._obj(b, g, r, hh)
            else:
                self.obstacles[bname[4:]] = {"pos": d.geom_xpos[g].copy(), "r": r, "h": hh}
        self.cur = "mug" if ("mug" in self.objs or not self.objs) else next(iter(self.objs))
        self.home_palm = {h: d.site_xpos[rt.palm_sid[h]].copy() for h in self.hands}
        self.home_quat = {h: d.mocap_quat[rt.hands_mid[h]].copy() for h in self.hands}
        self.vase = self.obstacles.get("vase") or (next(iter(self.obstacles.values())) if self.obstacles else None)
        self.keepout = None      # keep-out zones hook (pps/features/zones.py): .at(p, r), .along(pts, events, palm), .route(kps, ev, palm)
        roots = getattr(rt, "robot_roots", None)
        self.robot_roots = set(roots) if roots else {m.body(ROBOT_BODIES_ROOT).id}
        self.robot_root = min(self.robot_roots)
        self.markers = []        # [{"id": "A", "pos": np.array}]
        self.next_marker = 0
        self.holding = {h: None for h in reversed(self.hands)}
        self.grasp_off = None
        self.ph = None           # physics grasp/release in progress: {"phase", "palm", "obj", "t0"}
        self.held = {}           # physics: palm -> what the slip monitor knows about the object in that hand
        self.place_at = None     # physics: where the current pick & place should leave the object (checked at the end)
        self.seq, self.seq_tid = [], None   # the rest of a multi-step task (two-arm handover): run as each path ends
        self.search = None       # a time-sliced pick search in progress: (task id, generator); search_tick() advances it
        self.task = None         # {"id","action","status","message","step"}
        self.events = {}         # waypoint index -> event name
        self.fired = set()
        self.kind = None         # "task" | "path"
        self.on_done = None
        self.fixture_init()      # issue #48: drawers, doors, containers, conveyors (pps/manip.py)
        os.makedirs(SKILL_DIR, exist_ok=True)

    # ------------------------------------------------------------------ geometry
    # current object (the one being handled); the mug_* names are kept for the original single-object code paths
    @property
    def mug_mid(self): return self.objs[self.cur]["mid"]
    @property
    def mug_home(self): return self.objs[self.cur]["home"]
    @property
    def mug_center_off(self): return self.objs[self.cur]["off"]
    @property
    def mug_half_h(self): return self.objs[self.cur]["half_h"]
    @property
    def mug_r(self): return self.objs[self.cur]["r"]

    def _obj(self, b, g, r, half_h):
        """A movable object: a mocap body (kinematic) or a free body (physics); its pose always goes through obj_pos."""
        m, d = self.rt.model, self.rt.data
        mid = int(m.body_mocapid[b])
        pos, quat = (d.mocap_pos[mid], d.mocap_quat[mid]) if mid >= 0 else (d.xpos[b], d.xquat[b])
        return {"bid": int(b), "mid": mid, "geom": int(g), "home": pos.copy(), "home_q": quat.copy(), "off": m.geom_pos[g].copy(), "r": r, "half_h": half_h}

    def obj_pos(self, name):
        """World position of a movable object's body (the one helper every reader uses)."""
        o, d = self.objs[name], self.rt.data
        return d.mocap_pos[o["mid"]].copy() if o["mid"] >= 0 else d.xpos[o["bid"]].copy()

    def obj_mat(self, name):
        o, d = self.objs[name], self.rt.data
        if o["mid"] >= 0:
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, d.mocap_quat[o["mid"]])
            return R.reshape(3, 3)
        return d.xmat[o["bid"]].reshape(3, 3).copy()

    def set_obj_pose(self, name, pos, quat=None):
        """Teleport an object (scene reset, the person holding it): mocap, or the free joint at rest (physics)."""
        o, d = self.objs[name], self.rt.data
        if o["mid"] >= 0:
            d.mocap_pos[o["mid"]] = pos
            if quat is not None:
                d.mocap_quat[o["mid"]] = quat
        elif self.plant is not None:
            self.plant.set_object(name, pos, quat)

    def mug_center(self, name=None):
        name = name or self.cur
        return self.obj_pos(name) + self.obj_mat(name) @ self.objs[name]["off"]

    def limb(self, side, what="hand"):
        """'right hand' / 'left arm' on two-armed bodies, 'gripper' / 'arm' on single arms."""
        if len(self.hands) > 1:
            return f"{side} {what}"
        return "gripper" if what == "hand" else "arm"

    def hand_for(self, p):
        """Which end effector handles a point: by side for two-armed bodies, the only one otherwise."""
        if len(self.hands) == 1:
            return self.hands[0]
        return "right_palm" if p[1] <= 0 else "left_palm"

    def picker(self, name):
        """The hand that will carry an object: the one holding it, else the one on its side (as pick_place chooses)."""
        held = next((h for h, o in self.holding.items() if o == name), None)
        return held or self.hand_for(self.mug_center(name))

    def keys_for(self, name):
        """Words that name an object or obstacle."""
        if self.legacy and name in LEGACY_KEYS:
            return LEGACY_KEYS[name]
        toks = [w for w in name.split("_") if w and not w.isdigit()]
        keys = [name.replace("_", " ")] + toks
        for w in toks + [self.kinds.get(name, "")]:
            keys += list(WORDS.get(w, ()))
        return tuple(dict.fromkeys(k for k in keys if k))

    def _match(self, text):
        """-> (movable object | None, fixed obstacle | None) that best fits the words (scenes built on the fly)."""
        t = str(text or "").lower().strip()
        if not t:
            return (self.cur if self.cur in self.objs else next(iter(self.objs), None)), None
        best, bscore, fixed = None, 0, False
        for pool, is_fixed in ((self.objs, False), (self.obstacles, True)):
            for n in pool:
                score = 10 if (n.replace("_", " ") in t or n in t) else 0
                kind_words = WORDS.get(self.kinds.get(n, ""), ())
                for k in self.keys_for(n)[1:]:
                    if k in t:
                        score += 3 if k in kind_words else 2
                if score > bscore:
                    best, bscore, fixed = n, score, is_fixed
        if best is None:
            return None, None
        return (None, best) if fixed else (best, None)

    def movable_list(self):
        names = [self.pretty(n) for n in self.objs]
        if not names:
            return "nothing here"
        return names[0] if len(names) == 1 else ", ".join(names[:-1]) + " or " + names[-1]

    def obj_name(self, text):
        if not self.legacy:
            return self._match(text)[0]
        t = str(text or "").lower()
        if any(k in t for k in ("blue", "파란", "vase", "꽃병", "화병")):
            return None
        if any(k in t for k in ("green", "초록", "녹색")):
            return "green_block" if "green_block" in self.objs else None
        if any(k in t for k in ("block", "cube", "red", "블록", "빨간", "큐브")):
            return "red_block" if "red_block" in self.objs else None
        if any(k in t for k in ("mug", "cup", "컵", "머그")) or not t:
            return "mug"
        return None

    @staticmethod
    def pretty(name):
        return {"mug": "the mug", "red_block": "the red block", "green_block": "the green block", "blue_block": "the blue block",
                "vase": "the vase"}.get(name, ("the " + str(name).replace("_", " ")) if name else name)

    def raycast(self, cam, u, v, aspect):
        """cam = (pos, forward, up, fovy_deg). Returns world hit point or None (ignores the robot and ghosts)."""
        m, d = self.rt.model, self.rt.data
        pos, fwd, up, fovy = cam
        right = np.cross(fwd, up)
        t = np.tan(np.radians(fovy) / 2)
        vec = fwd + right * (2 * u - 1) * t * aspect + up * (1 - 2 * v) * t
        vec = vec / np.linalg.norm(vec)
        pnt = np.array(pos, float)
        gg = np.array([1, 0, 1, 0, 0, 0], np.uint8)
        gid = np.zeros(1, np.int32)
        exclude = -1
        for _ in range(8):
            dist = mujoco.mj_ray(m, d, pnt, vec, gg, 1, exclude, gid)
            if dist < 0:
                return None
            hit = pnt + vec * dist
            body = m.geom_bodyid[gid[0]]
            if m.body_rootid[body] in self.robot_roots:
                pnt, exclude = hit + vec * 1e-3, body
                continue
            return hit
        return None

    @staticmethod
    def project(cam, p, aspect):
        pos, fwd, up, fovy = cam
        right = np.cross(fwd, up)
        rel = np.asarray(p, float) - pos
        z = float(np.dot(rel, fwd))
        if z <= 1e-3:
            return None
        t = np.tan(np.radians(fovy) / 2)
        x, y = np.dot(rel, right) / (z * t * aspect), np.dot(rel, up) / (z * t)
        u, v = (x + 1) / 2, (1 - y) / 2
        if not (-0.2 <= u <= 1.2 and -0.2 <= v <= 1.2):
            return None
        return [round(float(u), 4), round(float(v), 4)]

    def set_marker(self, mid, p):
        self.markers = [mk for mk in self.markers if mk["id"] != mid] + [{"id": mid, "pos": np.asarray(p, float)}]
        return mid

    def add_marker(self, p):
        mid = "ABCDEF"[self.next_marker % 6]
        self.next_marker += 1
        self.markers = [mk for mk in self.markers if mk["id"] != mid] + [{"id": mid, "pos": np.asarray(p, float)}]
        return mid

    # ------------------------------------------------------------------ targets
    def resolve(self, target=None, target_uv=None, cam=None, aspect=4 / 3):
        """-> (table point xyz, label) or (None, reason)."""
        if target_uv is not None and cam is not None:
            hit = self.raycast(cam, float(target_uv[0]), float(target_uv[1]), aspect)
            if hit is None:
                return None, "that spot is not on anything I can see"
            mid = self.add_marker(hit)
            return hit, f"marker {mid}"
        if target:
            t = str(target).strip().lower()
            if t in ("there", "here", "that", "over there", "that spot", "where i'm pointing", "where i am pointing",
                     "저기", "여기", "거기", "저기에", "여기에", "거기에", "p", "marker p"):
                pick = next((mk for mk in self.markers if mk["id"] == "P"), None) or (self.markers[-1] if self.markers else None)
                if pick is None:
                    return None, "point at the spot first (or tap it on my view), then say 'there'"
                return pick["pos"].copy(), f"the spot you {'pointed at' if pick['id'] == 'P' else 'marked'} ({pick['id']})"
            for mk in self.markers:
                if t in (mk["id"].lower(), f"marker {mk['id'].lower()}", f"point {mk['id'].lower()}"):
                    return mk["pos"].copy(), f"marker {mk['id']}"
            for name, (x, y) in REGIONS.items():
                if name in t:
                    return np.array([x, y, TABLE_TOP]), f"the {name} of the {SURFACE}"
            near = any(k in t for k in ("next to", "beside", "near", "옆"))
            order = list(self.objs) + list(self.obstacles)
            if not self.legacy:   # best match first, so 'next to the red cup' doesn't stop at another cup
                best = [x for x in self._match(t) if x]
                order = best + [n for n in order if n not in best]
            for oname in order:
                keys = self.keys_for(oname)
                if any(k in t for k in keys):
                    c = self.mug_center(oname) if oname in self.objs else self.obstacles[oname]["pos"]
                    if near:
                        others = [self.mug_center(n) for n in self.objs if n != oname and n != self.cur] + [o["pos"] for o in self.obstacles.values() if o is not self.obstacles.get(oname)]
                        best, bscore = None, -1.0
                        for dx, dy in ((0, 0.12), (0, -0.12), (-0.09, 0.0), (0.11, 0.0), (-0.08, 0.1), (-0.08, -0.1)):
                            cand = np.array([c[0] + dx, c[1] + dy, TABLE_TOP])
                            if not self.on_table(cand):
                                continue
                            clear = min([np.linalg.norm(cand[:2] - np.asarray(o)[:2]) for o in others] or [1.0])
                            score = min(clear, 0.2) - 0.3 * max(0.0, cand[0] - 0.36)
                            if score > bscore:
                                best, bscore = cand, score
                        if best is not None:
                            return best, f"next to {self.pretty(oname)}"
                    return np.array([c[0], c[1], TABLE_TOP]), self.pretty(oname)
            return None, f"I don't know where '{target}' is. Tap it on my view or use left, right, center, front or back"
        return None, "no target given"

    def free_spot(self, dest, moving):
        """dest is occupied by an obstacle or another object? -> nearest free point on the table, plus who was in the way."""
        return _drain(self._free_spot(dest, moving))

    def _free_spot(self, dest, moving):
        """free_spot, resumable: yields between the (physics) reach tests of its candidates."""
        r_me = self.objs[moving]["r"] if moving in self.objs else 0.04
        jaws = None
        if self.plant is not None and self.plant.grip:   # physics: the hand that sets it down has to fit there too
            r_me = max(r_me, min(g.get("depth", 0.0) for g in self.plant.grip.values()) + 0.01)
            room = min(g.get("room", 0.0) for g in self.plant.grip.values())
            prof = getattr(self.rt, "profile", None)
            if room and moving in self.objs and prof is not None and (prof.physics or {}).get("grasp", prof.grasp) == "side":
                sh = self.rt.retargeter.robot_shoulders[self.picker(moving)]   # side jaws close level, across the heading
                hd = self.mug_center(moving)[:2] - np.asarray(sh, float)[:2]  # they took it with (kept to set it down):
                hd = hd / (np.linalg.norm(hd) or 1.0)                         # they reach `room` past its sides as they
                jaws = (np.array([-hd[1], hd[0]]), self._half_w(moving) + room, 0.025)   # open to let go
        others = [(n, o["pos"], o["r"]) for n, o in self.obstacles.items()] + \
                 [(n, self.mug_center(n), o["r"]) for n, o in self.objs.items() if n != moving]

        def blocker_at(p):
            for n, c, r in others:
                dv = np.asarray(c, float)[:2] - np.asarray(p, float)[:2]
                if np.linalg.norm(dv) < r + r_me + 0.01:
                    return n
                if jaws is not None:
                    a, hl, hw = jaws
                    da, db = abs(float(np.dot(dv, a))), abs(float(a[0] * dv[1] - a[1] * dv[0]))
                    if np.hypot(max(da - hl, 0.0), max(db - hw, 0.0)) < r + 0.005:
                        return n
            return self.keepout.at(p, r_me) if self.keepout else None

        who = blocker_at(dest)
        if who is None:
            return dest, None
        cands = []
        for rad in (0.08, 0.11, 0.14, 0.17):
            for ang in np.linspace(0, 2 * np.pi, 16, endpoint=False):
                cand = np.array([dest[0] + rad * np.cos(ang), dest[1] + rad * np.sin(ang), dest[2]])
                if self.on_table(cand) and blocker_at(cand) is None:
                    score = rad + 0.5 * max(0.0, cand[0] - 0.33) + 0.5 * max(0.0, abs(cand[1]) - 0.24)   # near, reachable
                    if self.plant is not None and moving in self.objs:   # physics: not over a fixed obstacle (the vase is
                        a = self.mug_center(moving)[:2]                   # taller than a carry clears) when there's another way
                        if any(_seg_dist(o["pos"][:2], a, cand[:2]) < o["r"] + self.objs[moving]["r"] + 0.01 for o in self.obstacles.values()
                               if o["pos"][2] + o["h"] > TABLE_TOP + 0.12):
                            score += 0.5
                    cands.append((score, len(cands), cand))
            if cands and self.plant is None:
                break
        if not cands:
            return None, who
        cands.sort(key=lambda c: c[:2])
        if self.plant is not None and moving in self.objs:   # physics: the nearest spot the hand can really set it down on
            hh, hand = self.objs[moving]["half_h"], self.picker(moving)
            frm = None if self.holding.get(hand) == moving else self.mug_center(moving)
            if frm is not None and np.linalg.norm(frm - np.asarray(self.rt.retargeter.robot_shoulders[hand])) > self._reach_bound(hand):
                return cands[0][2], who           # (it can't even be picked up: no use testing where it would go)
            for _, _, cand in cands[:8]:
                self.rt.set_obstacles((moving,))
                p = np.array([cand[0], cand[1], TABLE_TOP + hh])
                if self._reach_err(p, hand, frm=frm) < CRIT_TOL:
                    return cand, who
                yield
        return cands[0][2], who

    def _reach_err(self, p, palm, iters=50, frm=None):
        """Physics: how close (m) the planner gets the hand to p from where it is, the way a place goes (over the spot, then
        down), holding the place orientation: the one it holds now when it carries something (frm None and a load in the
        hand), else side grasps keep the heading they took the object with (in from the shoulder toward frm, the object's
        position now; toward p without one), and from above: as now, or turned 90° about the vertical, whichever fits."""
        rt = self.rt
        task = rt.hand_tasks[rt.hands.index(palm)]
        saved = task.transform_target_to_world
        q0 = rt.data.mocap_quat[rt.hands_mid[palm]].copy()
        quats = [q0]
        prof = getattr(rt, "profile", None)
        style = (prof.physics or {}).get("grasp", prof.grasp) if prof is not None else "side"
        g = self.plant.grip.get(self.role(palm)) if self.plant is not None else None
        if style == "side" and g is not None and (frm is not None or not self.holding.get(palm)):
            sh = rt.retargeter.robot_shoulders[palm]            # side grasps: in from the shoulder, jaws level
            src = p if frm is None else frm
            n = np.array([src[0] - sh[0], src[1] - sh[1], 0.0])
            n /= np.linalg.norm(n) or 1.0
            a = np.cross([0.0, 0.0, 1.0], n)
            L = np.column_stack([g["axis"], g["approach"], np.cross(g["axis"], g["approach"])])
            W = np.column_stack([a, n, np.cross(a, n)])
            if g.get("up") is not None and (W @ L.T @ g["up"])[2] < 0:
                W = np.column_stack([-a, n, np.cross(-a, n)])
            q0 = np.zeros(4)
            mujoco.mju_mat2Quat(q0, (W @ L.T).flatten())
            quats = [q0]
        if getattr(prof, "ee_down", False) and style == "top":
            q90 = np.zeros(4)
            mujoco.mju_mulQuat(q90, np.array([np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)]), q0)
            quats.append(q90)
        rt.orient(True)
        best = np.inf
        cur = rt.data.site_xpos[rt.palm_sid[palm]].copy()
        above = np.asarray(p, float) + np.array([0.0, 0.0, 0.08])
        for q in quats:                           # the way a place goes: from the hand, over the spot, down (as the plan)
            pts, qs, _ = self._dense([cur, above, np.asarray(p, float)], [q, q, q])
            rt.precheck(pts, palm, quats=qs)
            best = min(best, float(rt.last_precheck_dists[-1]))
            if best < CRIT_TOL:
                break
        rt.orient(False)
        if saved is not None:
            task.set_target(saved)
        return best

    def on_table(self, p):
        return TABLE_X[0] + 0.03 < p[0] < TABLE_X[1] - 0.03 and TABLE_Y[0] + 0.03 < p[1] < TABLE_Y[1] - 0.03

    # ------------------------------------------------------------------ lifecycle
    def set_task(self, tid, action, status, message, step=""):
        self.task = {"id": tid, "action": action, "status": status, "message": message, "step": step}

    def _launch(self, tid, action, keypoints, palm, events, kind, ok_msg, quats=None, loose=None):
        """Densify, pre-check, then start following. events: {keypoint_index: name}. quats: a hand orientation per keypoint.
        loose: keypoint index ranges (k0, k1) whose inner waypoints are transit (a looser reach check)."""
        rt = self.rt
        if quats is not None and len(quats) != len(keypoints):
            quats = None                          # a feature changed the keypoints (detours): keep the orientation
        if quats is not None and self.plant is not None:   # physics: the path starts with the hand as it really is
            q0 = np.zeros(4)
            mujoco.mju_mat2Quat(q0, rt.data.site_xmat[rt.palm_sid[palm]])
            quats = [q0] + list(quats[1:])
        pts, qs, idx_map = self._dense(keypoints, quats)
        if self.plant is not None:                # physics: plan from where the arm really is (it may still be moving)
            rt.configuration.update(self.plant.measured_q(rt.configuration.model))
            events = dict(events)                 # ... and stop above a grasp until it has lined up, then go in slowly
            for k, nm in list(events.items()):   # (a drawer / door knob too: the arm trails the plan, and cutting the
                if nm in ("grasp", "grasp_fx") and k > 0 and (k - 1) not in events:   # corner puts the fingers on the
                    events[k - 1] = "hover"                                           # counter's edge)
        self.set_task(tid, action, "planning", "Checking reach for every waypoint.")
        side = "right" if palm.startswith("right") else "left"

        def hits(ok, hit):
            ok = [o and not (lo <= j <= hi) for j, o in enumerate(ok) for lo, hi in [hit[1]]]
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            rt.path = {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side}
            self.kind = None
            self.fail_reason = "vase"
            over = "the vase" if self.legacy else self.pretty(self.hit_obstacle)
            self.set_task(tid, action, "failed",
                          f"I can't carry it that way: {self.pretty(self.cur)} would hit {self.pretty(self.hit_obstacle)} ({hit[0]:.0f} cm too low). "
                          f"Draw a route over {over} in 3D, then say 'carry it along my path'.")
            return False

        if self.plant is not None:                # physics: the planner keeps clear of obstacles, so say what's in the way first
            hit = self.carry_hits_vase(pts, {idx_map[k]: n for k, n in events.items()}, palm)
            if hit is not None:
                return hits([True] * len(pts), hit)
        rt.orient(qs is not None)                 # physics: grasp paths hold the hand orientation firmly
        ok = rt.precheck(pts, palm, quats=qs)
        dists = rt.last_precheck_dists
        if self.plant is not None:                # the first waypoint is where the hand is
            ok[0], dists[0] = True, 0.0
        for k0, k1 in loose or ():                # transit segments only need passing near (2x the reach tolerance)
            for j in range(idx_map[k0] + 1, idx_map[k1]):
                ok[j] = dists[j] < 2 * rt.validator.reach_tol
            if self.plant is not None and k1 == len(keypoints) - 1 and k0 > 0:
                ok[-1] = True                     # physics: going home after the work: wherever it ends up near home is fine
        crit = [idx_map[k] for k, nm in events.items() if nm in ("grasp", "release")] if self.plant is not None else []
        # dropping into a container (place_in) needs the object inside it, not exactly on a spot: the release may be a few cm
        # off (the container's walls keep the hand back); the check after the release says whether it ended up inside
        tol = {idx_map[k]: (0.05 if nm == "release" and action == "place_in" else CRIT_TOL) for k, nm in events.items()}
        short = [j for j in crit if dists[j] > tol.get(j, CRIT_TOL)]
        if short and all(ok):                     # physics: the jaws must get exactly there, not just near
            j = max(short, key=lambda i: dists[i])
            for i in short:
                ok[i] = False
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            rt.path = {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side}
            rt.path_msg = None
            self.kind = None
            where = "the grasp" if events.get(next(k for k in events if idx_map[k] == j)) == "grasp" else "the spot"
            self.set_task(tid, action, "failed", f"I can't: my {self.limb(side, 'arm')} can't get my {self.limb(side)} exactly to "
                                                 f"{where} ({dists[j] * 100:.1f} cm off: it is at the edge of my reach or something is in the way).")
            return False
        if not all(ok):
            worst = int(np.argmax(dists))
            short = (dists[worst] - rt.validator.reach_tol) * 100 + rt.validator.reach_tol * 100
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            rt.path = {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side}
            rt.path_msg = None
            self.kind = None
            msg = f"I can't: {sum(1 for o in ok if not o)} of {len(ok)} waypoints are out of my {self.limb(side, 'arm')}'s reach, worst by {short:.0f} cm."
            self.set_task(tid, action, "failed", msg)
            return False
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        hit = self.carry_hits_vase(pts, {idx_map[k]: n for k, n in events.items()}, palm)
        if hit is not None:
            return hits(ok, hit)
        zs = self.keepout.along(pts, {idx_map[k]: n for k, n in events.items()}, palm) if self.keepout else None
        if zs and any(zs):
            zone, who = next(z for z in zs if z)
            ok = [o and not z for o, z in zip(ok, zs)]
            rt.path = {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side}
            self.kind, self.fail_reason = None, "zone"
            self.set_task(tid, action, "failed", (f"I can't: my {self.limb(side)} would enter {zone}." if who == "hand" else
                          f"I can't carry it that way: {who} would pass through {zone}.") + " Draw a route around it in 3D, or remove the zone.")
            return False
        rt.path = {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side}
        if self.plant is not None:                # physics: the last segment into a grasp / a release is slow
            rt.path["slow"] = [(idx_map[k - 1], idx_map[k]) for k, nm in events.items() if nm in ("grasp", "release", "press", "grasp_fx") and k > 0]
        rt.path["slow"] = rt.path.get("slow", []) + self.fx_slow(events, idx_map)   # a drawer / door moves slowly
        gk = next((idx_map[k] for k, nm in sorted(events.items()) if nm == "grasp"), None)
        if self.plant is not None and gk is not None and self.cur in self.objs:   # the grasp, in the object's own frame
            rt.path["anchor"] = (gk, self.cur, self.obj_mat(self.cur).T @ (pts[gk] - self.obj_pos(self.cur)))
        if qs is not None:
            rt.path["q"] = qs
            if self.plant is not None:            # physics: the hand turns no faster than W_TURN (slower along that segment)
                seg_t = np.maximum(seg / max(rt.follow_speed, 1e-3), 1e-6)
                ang = np.array([2 * np.arccos(min(1.0, abs(float(np.dot(qs[j], qs[j + 1]))))) for j in range(len(seg))])
                rt.path["sf"] = np.clip(seg_t / np.maximum(ang / W_TURN, 1e-6), 0.15, 1.0)
        rt.path_msg = None
        self.events = {idx_map[k]: name for k, name in events.items()}
        self.fired = set()
        self.kind = kind
        rt.engaged = False
        rt.mode, rt.follow_s, rt.follow_hold, self.ph = "FOLLOW", 0.0, False, None
        rt.recorder.start()
        self.set_task(tid, action, "running", ok_msg)
        return True

    def carry_hits_vase(self, pts, events, palm):
        """While the mug is in hand (grasp..release), does the carried mug sweep through the vase? -> (cm short, (i0,i1)) | None"""
        if self.vase is None and (self.plant is None or len(self.objs) < 2):
            return None
        held_now = self.holding.get(palm) == self.cur
        g = next((i for i, n in sorted(events.items()) if n == "grasp"), None)
        r = next((i for i, n in sorted(events.items()) if n == "release"), len(pts) - 1)
        start = 0 if held_now else g
        if start is None:
            return None
        off = self.mug_center() - self.rt.data.site_xpos[self.rt.palm_sid[palm]] if held_now else -getattr(self, "cur_off", np.array([-(self.mug_r + 0.015), 0.0, 0.01]))
        worst, span, who = 0.0, None, None
        bodies = [(ob["pos"], ob["r"], ob["h"], n) for n, ob in self.obstacles.items()]
        if self.plant is not None:                # physics: the other objects are just as solid; neighbours of the spots it
            ends = [pts[start] + off, pts[min(r, len(pts) - 1)] + off]      # leaves from / is set down at are passed vertically
            bodies += [(self.mug_center(n), o["r"], o["half_h"], n) for n, o in self.objs.items() if n != self.cur and
                       all(np.linalg.norm((self.mug_center(n) - e)[:2]) > o["r"] + self.mug_r + 0.01 for e in ends)]
        for vp, vr, vh, oname in bodies:
            for j in range(start + 1, r):
                c = pts[j] + off
                if np.linalg.norm(c[:2] - vp[:2]) < vr + self.mug_r + 0.01:
                    short = (vp[2] + vh) - (c[2] - self.mug_half_h) + 0.01
                    if short > 0:
                        if short > worst:
                            worst, who = short, oname
                        span = (j, j) if span is None else (min(span[0], j), max(span[1], j))
        self.hit_obstacle = who
        return None if span is None else (worst * 100, span)

    def _bodies(self, skip=()):
        """Everything on the surface a carried object could bump into: (centre xyz, radius, half height, name)."""
        out = [(self.mug_center(n), self._radius(n), o["half_h"], n) for n, o in self.objs.items() if n not in skip]
        return out + [(ob["pos"], ob["r"], ob["h"], n) for n, ob in self.obstacles.items() if n not in skip]

    def _half_w(self, name):
        """Half the width a gripper closes across: a box's smaller half side, a cylinder's radius, the mug's body."""
        o, m = self.objs[name], self.rt.model
        if (self.plant is not None and self.plant.objects.get(name, {}).get("kind") == "mug") or name == "mug":
            from .physics import MUG
            return MUG["R"] * o["r"] / 0.065
        if m.geom_type[o["geom"]] == mujoco.mjtGeom.mjGEOM_BOX:
            return float(min(m.geom_size[o["geom"]][:2]))
        return o["r"]

    def _radius(self, name):
        """Horizontal radius of a movable object's outline (a box's corners stick out past its half width)."""
        o, m = self.objs[name], self.rt.model
        if m.geom_type[o["geom"]] == mujoco.mjtGeom.mjGEOM_BOX:
            return float(np.hypot(*m.geom_size[o["geom"]][:2]))
        return o["r"]

    def _place_yaw(self, name, place_c, off, q_grip, palm):
        """Physics, top grasps: a turn of the hand about the vertical (0 or ±90°) for setting `name` down at place_c so the
        open jaws and the hand (its footprint across / along the jaws) keep clear of what stands next to the spot."""
        g = self.plant.grip.get(self.role(palm))
        if g is None:
            return 0.0
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, q_grip)
        a0 = R.reshape(3, 3) @ g["axis"]
        a0 = np.array([a0[0], a0[1]]) / (np.hypot(a0[0], a0[1]) or 1.0)
        span, depth = max(g.get("span", 0.0), g["width"] / 2 + 0.01), max(g.get("depth", 0.0), 0.015)
        bodies = [(c[:2], r) for c, r, _, n in self._bodies(skip=(name,))]
        out = []
        for yaw in (0.0, np.pi / 2, -np.pi / 2):
            cz, sz = np.cos(yaw), np.sin(yaw)
            a = np.array([cz * a0[0] - sz * a0[1], sz * a0[0] + cz * a0[1]])
            b = np.array([-a[1], a[0]])
            o = np.array([cz * off[0] - sz * off[1], sz * off[0] + cz * off[1]])
            p = place_c[:2] + o
            clear = min([float(np.linalg.norm(np.maximum(np.abs([np.dot(c - p, a), np.dot(c - p, b)]) - [span, depth], 0.0))) - r
                         for c, r in bodies] or [1.0])
            out.append((clear + (0.02 if yaw == 0.0 else 0.0), yaw))   # keep the grasp's own heading unless turning helps
        return [y for _, y in sorted(out, key=lambda s: -s[0])]

    def _carry_z(self, name, a, b, off, margin=0.03):
        """Physics: the hand height (TCP z) at which `name`, held with TCP offset `off` from its centre, clears the top of
        the other movable objects it passes over on the straight way from hand point a to b (with `margin`); -inf when
        nothing is near. Fixed obstacles (the vase) are not flown over on its own: carrying into one is refused and the
        person draws a route over it (UX 2.0), and the planner keeps the hand clear of them."""
        r, hh = self._radius(name), self.objs[name]["half_h"]
        ca, cb = (np.asarray(a, float) - off)[:2], (np.asarray(b, float) - off)[:2]
        ab = cb - ca
        need = -np.inf
        for c, rb, hb, n in self._bodies(skip=(name,)):
            if n in self.obstacles:
                continue
            t = float(np.clip(np.dot(c[:2] - ca, ab) / (float(np.dot(ab, ab)) or 1.0), 0.0, 1.0))
            if np.linalg.norm(c[:2] - (ca + t * ab)) < rb + r + 0.008:
                need = max(need, float(c[2]) + hb + margin + hh + float(off[2]))
        return need

    def _pre_dz(self, name, palm, g):
        """Physics, from above: how far over the grasp the hand stops before going down: 9 cm, or more for a long gripper
        whose open fingertips would otherwise skim the object's top on the way there (Spot's claw reaches 7 cm past its
        TCP and holds the mug 2 cm below the rim)."""
        top = self.mug_center(name)[2] + self.objs[name]["half_h"]
        return max(0.09, top + self._hang(palm, side=False) + 0.025 - float(g[2]))

    def _hang(self, palm, side=True):
        """Physics: how far the open hand reaches below its TCP (side grasps: the pads' half height; from above: the
        fingertips past the TCP)."""
        g = self.plant.grip.get(self.role(palm)) if self.plant is not None else None
        if g is None:
            return 0.03
        return float(g.get("pad_h", 0.018)) + 0.01 if side else float(g["tip"])

    def _clear_z(self, a, b, palm, side, placed=None, margin=0.03, skip=()):
        """Physics: the TCP height at which the open hand, going straight from a to b, passes over every movable object near
        that line (`placed`: (name, centre) of one just set down, counted where it now stands; `skip`: ones it goes to)
        with `margin`; -inf if none. Fixed obstacles (the vase) are not flown over: the planner keeps the hand clear of them."""
        hang, need = self._hang(palm, side), -np.inf
        g = self.plant.grip.get(self.role(palm)) if self.plant is not None else None
        foot = max(float(g.get("span", 0.0)), float(g["width"]) / 2) if g else 0.05
        bodies = [(c, r, hh, n) for c, r, hh, n in self._bodies(skip=tuple(self.obstacles) + tuple(skip))
                  if not placed or n != placed[0]]
        if placed:
            o = self.objs[placed[0]]
            bodies.append((np.array([placed[1][0], placed[1][1], TABLE_TOP + o["half_h"]]), self._radius(placed[0]), o["half_h"], placed[0]))
        a2, b2 = np.asarray(a, float)[:2], np.asarray(b, float)[:2]
        for c, r, hh, _n in bodies:
            c2 = np.asarray(c, float)[:2]
            if _seg_dist(c2, a2, b2) >= r + foot + 0.01:
                continue
            if placed and side and np.dot(c2 - a2, b2 - a2) <= 0:   # a level side gripper that backed off beside it
                continue                                              # and only moves away from it: nothing to go over
            need = max(need, float(c[2]) + hh + hang + margin)
        return need

    def tick(self):
        """Call after every rt.step: fire waypoint events, carry the mug, finish tasks."""
        rt, d = self.rt, self.rt.data
        if self.search is not None and time.time() - getattr(self, "_search_t", 0.0) > 0.05:
            self.search_tick()                    # (a slice of a pick search, about once a frame)
            self._search_t = time.time()
        if rt.path is not None and self.kind in ("task", "path") and (rt.mode == "FOLLOW" or self.plant is not None):
            i = rt.path["i"]                      # (physics: also an event on the last waypoint, fired as the path ends)
            for idx, name in sorted(self.events.items()):
                if idx <= i and idx not in self.fired:
                    self.fired.add(idx)
                    self._event(name, rt.path["palm"])
        if self.plant is not None:
            self._phys_tick()
        else:
            for palm, obj in self.holding.items():   # (both hands on it, a handover: it rides the one that took it last)
                if obj in self.objs and self.grasp_off is not None and getattr(self, "grasp_palm", palm) in (palm, None):
                    self.set_obj_pose(obj, d.site_xpos[rt.palm_sid[palm]] + self.grasp_off)
            self._fx_kin_tick()                   # kinematic: a held drawer / door follows the hand
        active = (self.kind in ("task", "path") and (rt.mode == "FOLLOW" or self.ph is not None)) \
            or (self.kind == "skill" and rt.mode == "PREVIEW") or (self.kind == "drive" and getattr(rt, "driving", False))
        if self.seq and (not self.task or self.task.get("id") != self.seq_tid or self.task["status"] != "running"):
            self.seq = []                         # stopped, failed or replaced by another command
        if self.kind and not active and self.seq:
            self.kind = None                      # a multi-step task (two-arm handover): on to its next step
            self.seq.pop(0)()
            return
        if self.kind and not active:
            rt.orient(False)                      # the next reach / mirroring moves the hand freely again
            self._free_obstacles()
            if self.task and self.task["status"] == "running":
                act = self.task.get("action")
                bad = (self._placed_badly() if act in ("pick_place", "handover", "place_in") else
                       self._fx_short() if act in ("open", "close") else None)
                if bad:
                    self.set_task(self.task["id"], self.task["action"], "failed", bad)
                else:
                    self.set_task(self.task["id"], self.task["action"], "done", self.done_msg or "Done.")
            self.kind = None

    # ------------------------------------------------------------------ physics: grasp by contact, slip, weight
    @staticmethod
    def role(palm):
        return palm[:-5] if palm.endswith("_palm") else palm

    def _hold_path(self, on):
        self.rt.follow_hold = bool(on)

    def _fail_phys(self, palm, msg, let_go=True):
        """The physical world said no: stop where we are, report honestly."""
        rt, pl = self.rt, self.plant
        obj = self.holding.get(palm) or (self.ph or {}).get("obj")
        if let_go:
            if palm in self.holding:
                self.holding[palm] = None
            self.held.pop(palm, None)
            pl.grip_cmd(self.role(palm), "open")
        self.ph, self.place_at = None, None
        self._fx_cleanup()
        self._hold_path(False)
        rt.mode = "MIRROR"
        rt.hold_here()
        rt.orient(False)
        self._free_obstacles()
        self.kind = None
        if obj in self.objs:
            self.cur = obj
        if self.task:
            self.set_task(self.task["id"], self.task["action"], "failed", msg)

    def _free_obstacles(self):
        """Physics: between tasks the planner keeps clear of fixtures only. Objects are obstacles while a pick / press plans
        around them (set_obstacles there, with fresh poses); a drawn path, a trajectory, mirroring or teleop may close the
        gripper on any of them, and a stale pose would otherwise keep the hand away from where an object used to be."""
        if self.plant is not None:
            self.rt.set_obstacles(tuple(self.plant.objects))

    def _manual_grip(self, now):
        """Grippers driven outside a task (rt.set_gripper: P2 trajectory grip events, P3 mirroring, VR teleop): after a
        close the task layer holds what both jaws touch, and lets go of it on open or once the jaws lose it. A task that
        commands the gripper itself takes over (its command differs from the manual one)."""
        man = getattr(self.rt, "grip_manual", None)
        if not man or self.ph is not None:
            return
        for role, cmd in list(man.items()):
            palm, g = f"{role}_palm", self.plant.grip.get(role)
            if g is None or palm not in self.holding or g["cmd"] != cmd:
                man.pop(role, None)
                continue
            held = self.holding.get(palm)
            if cmd == "open":
                if held is not None:
                    self.holding[palm] = None
                    self.held.pop(palm, None)
                    self.cur = held
                man.pop(role, None)
            elif held is None:
                obj = self.plant.grasped().get(role)
                if obj is not None and obj not in self.holding.values():
                    self.holding[palm], self.cur = obj, obj
            elif palm not in self.held:                 # held by a manual close: drop it from holding once it's gone
                lost = self._manual_lost = getattr(self, "_manual_lost", {})
                if all(self.plant.touching(role, held)):
                    lost[palm] = None
                elif lost.get(palm) is None:
                    lost[palm] = now
                elif now - lost[palm] > 0.3:
                    self.holding[palm] = None
                    lost[palm] = None

    def _phys_tick(self):
        rt, pl = self.rt, self.plant
        now = float(pl.d.time)
        ph = self.ph
        s = float(np.clip(getattr(rt, "safety_scale", 1.0), 0.0, 1.0))
        last, self._t_phys = getattr(self, "_t_phys", now), now
        if ph is not None and s < 1.0 and now > last:   # P5: near a person the phase clocks run at the safe speed
            for k in ("t0", "t_last", "on", "still"):    # (elapsed-time timers; "tp" samples the hand's speed: real time)
                if ph.get(k) is not None:
                    ph[k] += (now - last) * (1.0 - s)
        self._manual_grip(now)
        if s <= 0.0:                                # protective stop: no grasp / release / press decisions (the gripper keeps
            for palm, h in list(self.held.items()):  # its command); only an object that really falls out is reported
                self._slip_check(palm, h, now)
            return
        if ph is not None and ph["phase"] in ("settle", "close", "arrive", "open", "hover", "aim"):
            palm, obj, role = ph["palm"], ph["obj"], self.role(ph["palm"])
            tcp = rt.data.site_xpos[rt.palm_sid[palm]]
            if ph["phase"] in ("hover", "aim"):    # above the grasp / the drop spot: line up, then go down slowly
                off = float(np.linalg.norm(tcp - rt.data.mocap_pos[rt.hands_mid[palm]]))
                v = 0.0 if ph.get("p") is None else float(np.linalg.norm(tcp - ph["p"])) / max(now - ph["tp"], 1e-4)
                ph["p"], ph["tp"] = tcp.copy(), now
                if (off < 0.004 and v < 0.02) or now - ph["t0"] > 1.5:
                    if ph["phase"] == "aim":
                        self._aim(palm)
                    else:
                        self._regrasp_aim()
                    self.ph = None
                    self._hold_path(False)
            elif ph["phase"] == "arrive":          # at the drop point? (the plant lags the plan; it may be stuck)
                off = float(np.linalg.norm(tcp - rt.data.mocap_pos[rt.hands_mid[palm]]))
                v = 0.0 if ph.get("p") is None else float(np.linalg.norm(tcp - ph["p"])) / max(now - ph["tp"], 1e-4)
                ph["p"], ph["tp"] = tcp.copy(), now
                ph["still"] = (ph.get("still") or now) if (off < 0.005 and v < 0.02) else None
                sup = obj in pl.objects and pl.support(obj, role) > 0.5 * pl.weight(obj)
                if sup or (ph["still"] is not None and now - ph["still"] > 0.1) or (now - ph["t0"] > 2.5 and off < 0.06):
                    pl.grip_cmd(role, "open", to=pl.grip[role].get("open_to"))   # resting on the surface, or the hand has
                    # stopped just above it (as wide as the grasp opened: a claw inside a mug would swing into its far wall)
                    self.held.pop(palm, None)       # letting go on purpose: no more slip watch
                    ph["phase"], ph["t0"] = "open", now
                elif now - ph["t0"] > 2.5:
                    kg = pl.grip_load(role, obj) / 9.81
                    pay = float((rt.profile.physics or {}).get("payload", 3.0)) if getattr(rt, "profile", None) else 3.0
                    if kg > pay * 1.05:               # it hangs in the jaws but the arm can't carry it
                        self._fail_phys(palm, f"Too heavy: {self.pretty(obj)} weighs about {kg:.1f} kg and I can lift {pay:g} kg, "
                                              "so I let it back down.")
                    else:
                        self._fail_phys(palm, f"My arm couldn't follow that path with {self.pretty(obj)} in hand: it stopped "
                                              f"{off * 100:.0f} cm short of where I'd put it down, so I'm still holding it.", let_go=False)
                    return
            elif ph["phase"] == "settle":          # let the arm arrive before closing (the plant lags the plan)
                tgt = rt.data.mocap_pos[rt.hands_mid[palm]]
                if np.linalg.norm(tcp - tgt) < 0.012 or now - ph["t0"] > 1.2:
                    pl.grip_cmd(role, "close")
                    ph["phase"], ph["t0"] = "close", now
            elif ph["phase"] == "close":
                ta, tb = pl.touching(role, obj)
                if (ta and tb and now - ph["t0"] > 0.3) or now - ph["t0"] > 0.9:
                    R = rt.data.site_xmat[rt.palm_sid[palm]].reshape(3, 3)
                    self.holding[palm] = obj        # an attempt: the slip monitor decides whether it really holds
                    self.held[palm] = {"obj": obj, "rel": R.T @ (self.obj_pos(obj) - tcp), "t0": now, "lost": None,
                                       "free_t": None, "weighed": False, "z0": float(self.obj_pos(obj)[2]), "contact": bool(ta and tb)}
                    self.ph = None
                    self._hold_path(False)
                    if self.task:
                        self.task["step"] = "grasped"
            elif ph["phase"] == "open":           # wait until the jaws are off it (they open over ~0.8 s)
                g, touch = pl.grip[role], any(pl.touching(role, obj))
                ph["off"] = None if touch else (ph.get("off") or (now, g.get("ctrl")))
                if ph.get("free") is None and ph["off"] is not None and now - ph["off"][0] > 0.08:
                    ph["free"] = ph["off"][1]     # the jaws let go here: open a little further, not all the way (a wide
                if g["cmd"] == "open" and ph.get("free") is not None and g.get("ctrl") is not None and (   # gripper opening
                        abs(g["ctrl"] - ph["free"]) > RELEASE_OPEN * abs(g["open"] - g["close"]) or pl.hand_bump(role, obj)):
                    pl.grip_cmd(role, "hold")      # fully next to a neighbour would push it, or jam against it)
                if (now - ph["t0"] > 0.35 and not touch) or now - ph["t0"] > 1.2:
                    self.holding[palm] = None
                    self.held.pop(palm, None)
                    self.cur = obj
                    self.ph = None
                    self._hold_path(False)
                    if self.task:
                        self.task["step"] = "placed"
        for palm, h in list(self.held.items()):
            self._slip_check(palm, h, now)
        if ph is not None and ph["phase"] == "press":
            self._press_tick(now)
        if self.ph is not None and str(self.ph.get("phase", "")).startswith("fx_"):
            self._fx_tick(now)                    # a drawer / door in hand (pps/manip.py)

    def _slip_check(self, palm, h, now):
        rt, pl = self.rt, self.plant
        obj, role = h["obj"], self.role(palm)
        tcp = rt.data.site_xpos[rt.palm_sid[palm]]
        R = rt.data.site_xmat[rt.palm_sid[palm]].reshape(3, 3)
        drift = float(np.linalg.norm(self.obj_pos(obj) - (tcp + R @ h["rel"])))
        ta, tb = pl.touching(role, obj)
        h["lost"] = None if (ta and tb) else (h["lost"] if h["lost"] is not None else now)
        what = self.pretty(obj)
        F = pl.params["grip_force"]
        if drift > 0.05 or (h["lost"] is not None and now - h["lost"] > 0.3):
            lifted = float(self.obj_pos(obj)[2]) - h["z0"] > 0.02
            why = (f"It slipped: {what} fell out of my {self.limb(role)}." if lifted else
                   f"It slipped: {what} didn't come up with my {self.limb(role)}") + ("" if lifted else
                   (f" (grip force {F:.0f} N)." if F < 5 else " (I couldn't get a firm grip)."))
            self._fail_phys(palm, why)
            return
        others = [g for g in pl.object_contacts(obj) if g not in pl.grip[role]["pads"][0] + pl.grip[role]["pads"][1]
                  and self.rt.model.geom_bodyid[g] not in pl.ee_bodies[role]]
        if others:
            h["free_t"] = None
        elif h["free_t"] is None:
            h["free_t"] = now
        if not h["weighed"] and h["free_t"] is not None and now - h["free_t"] > 0.2:
            h["weighed"] = True                     # lifted clear: what the wrist feels is the object's weight
            kg = pl.grip_load(role, obj) / 9.81
            pay = float((getattr(rt, "profile", None) and rt.profile.physics or {}).get("payload", 3.0))
            if kg > pay * 1.05:
                self._fail_phys(palm, f"Too heavy: {what} weighs about {kg:.1f} kg and I can lift {pay:g} kg, so I let it back down.")

    def _regrasp_aim(self):
        """Physics, above a grasp: if the object has moved since the plan (nudged, or drooping in the other hand during a
        handover, or bumped by the arm on its way), the way down follows it, by up to 8 cm (the robot looks before it
        reaches in)."""
        path = self.rt.path
        if not path or path.get("anchor") is None:
            return
        gk, name, local = path["anchor"]
        i = int(path["i"])
        if name not in self.objs or gk <= i:
            return
        pts = np.array(path["pts"], float)
        delta = self.obj_pos(name) + self.obj_mat(name) @ local - pts[gk]
        if not 1e-4 < float(np.linalg.norm(delta)) < 0.08:
            return
        w = np.zeros(len(pts))
        w[i + 1:gk + 1] = 1.0
        if gk + 1 < len(pts):
            w[gk + 1:] = np.linspace(1.0, 0.0, len(pts) - gk)[1:]
        pts += w[:, None] * delta[None, :]
        path["pts"], path["cum"] = pts, np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])

    def _aim(self, palm):
        """Physics, above the spot: where does the object really sit in the hand? It can shift a little while the jaws close
        or on the way. The rest of the way down is moved so that the object lands on the spot, not the planned grasp point
        (a real robot does this with its wrist camera / an in-hand pose estimate)."""
        rt, obj = self.rt, self.holding.get(palm)
        path = rt.path
        if obj is None or obj not in self.objs or path is None or palm not in self.held:
            return
        rel = next((i for i, n in sorted(self.events.items()) if n == "release"), None)
        i = int(path["i"])
        if rel is None or rel <= i:
            return
        tcp = rt.data.site_xpos[rt.palm_sid[palm]]
        R = rt.data.site_xmat[rt.palm_sid[palm]].reshape(3, 3)
        inhand = R.T @ (self.mug_center(obj) - tcp)              # where it sits, in the hand's frame
        if path.get("q") is not None:
            Rr = np.zeros(9)
            mujoco.mju_quat2Mat(Rr, path["q"][rel])
            Rr = Rr.reshape(3, 3)
        else:
            Rr = R
        off = -(Rr @ inhand)                                     # TCP - object centre when the hand lets go
        delta = off - np.asarray(self.cur_off, float)
        n = float(np.linalg.norm(delta))
        if n < 1e-4:
            return
        if n > 0.04:                                             # more than that is a slip (the monitor's business)
            delta *= 0.04 / n
        pts = np.array(path["pts"], float)
        w = np.zeros(len(pts))
        w[i + 1:rel + 1] = 1.0
        if rel + 1 < len(pts):                                   # fade out on the way home
            w[rel + 1:] = np.linspace(1.0, 0.0, len(pts) - rel)[1:]
        pts += w[:, None] * delta[None, :]
        path["pts"], path["cum"] = pts, np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        self.cur_off = np.asarray(self.cur_off, float) + delta

    def _press_tick(self, now):
        """Admittance outer loop on the hand target along the approach axis: track `force` (N), never exceed the impedance
        force limit, hold for hold_s, then carry on along the path (retreat)."""
        rt, pl, ph = self.rt, self.plant, self.ph
        palm, mid = ph["palm"], self.rt.hands_mid[ph["palm"]]
        down = np.array([0.0, 0.0, -1.0])
        f = float(-np.dot(pl.ee_force_raw[self.role(palm)], down))      # what the surface pushes back with (unfiltered)
        ph["fmax"] = max(ph["fmax"], f)
        lim = pl.params["force_limit"] if pl.params["impedance"] else float("inf")
        want = min(ph["force"], 0.8 * lim)
        dt = max(now - ph["t_last"], 0.0)
        ph["t_last"] = now
        vmax = 0.01
        if f < 0.5:                               # not touching yet: a compliant arm trails its target; the target waits
            hz = rt.data.site_xpos[rt.palm_sid[palm]][2]           # for the hand (a trailing spring lands hard), and the
            if hz - rt.data.mocap_pos[mid][2] > 0.005:             # last centimetre goes slower
                vmax = 0.0
            elif hz - ph.get("contact_z", -1.0) < 0.01:
                vmax = 0.004
        if f > 0.9 * lim:                         # near the limit: back off at once
            v = -0.03
        else:                                     # admittance: a guarded move toward the surface until the force is right
            v = float(np.clip(0.002 * (want - f), -0.03, vmax))
        rt.data.mocap_pos[mid] = rt.data.mocap_pos[mid] + down * v * dt
        if abs(f - want) < max(0.25 * want, 1.0):
            ph["fs"].append(f)
            ph["on"] = ph["on"] if ph["on"] is not None else now
        if ph["on"] is not None and now - ph["on"] >= ph["hold"]:
            fs = ph["fs"] or [f]
            self.press_result = {"mean": float(np.mean(fs)), "max": ph["fmax"], "want": ph["force"], "limit": lim}
            capped = ph["force"] > want
            self.done_msg = (f"Pressed {ph['label']} with {np.mean(fs):.1f} N for {ph['hold']:.1f} s (peak {ph['fmax']:.1f} N"
                             + (f", capped by the {lim:.0f} N impedance limit" if capped else "") + ").")
            self.ph = None
            rt.mode = "FOLLOW"                    # retreat along the rest of the path
            self._hold_path(False)
        elif now - ph["t0"] > 8.0:
            self._fail_phys(palm, f"I couldn't press {ph['label']}: no firm contact after 8 s (peak {ph['fmax']:.1f} N).", let_go=False)

    def press(self, tid, target, target_uv, cam, aspect, force=10.0, hold_s=1.5):
        """Press down on a spot (or an object's top) with a force (N) for hold_s seconds: compliance, needs physics."""
        rt = self.rt
        if self.plant is None:
            self.set_task(tid, "press", "failed", "Pressing needs contact forces: physics is off (kinematic mode).")
            return
        dest, label = self.resolve(target, target_uv, cam, aspect)
        if dest is None:
            self.set_task(tid, "press", "failed", label[0].upper() + label[1:] + ".")
            return
        try:
            force, hold_s = float(np.clip(float(force), 0.5, 200.0)), float(np.clip(float(hold_s), 0.2, 10.0))
        except (TypeError, ValueError):
            self.set_task(tid, "press", "failed", "Say how hard (force in N) and how long (hold_s).")
            return
        for n_, o in self.objs.items():           # on top of an object when the spot is on one
            cc = self.mug_center(n_)
            if np.hypot(*(cc[:2] - dest[:2])) < o["r"]:
                dest = np.array([cc[0], cc[1], cc[2] + o["half_h"]])
        palm = self.hand_for(dest)
        role = self.role(palm)
        g = self.plant.grip.get(role)
        tip = g["tip"] if g else 0.0
        if g:
            self.plant.grip_cmd(role, "close")    # press with closed jaws
        cur = rt.data.site_xpos[rt.palm_sid[palm]].copy()
        home = self.home_palm[palm]
        start = dest + np.array([0, 0, tip + 0.03])
        kps = [cur]
        if home[2] < TABLE_TOP + 0.06 and home[0] < TABLE_X[0] + 0.05 and cur[2] < TABLE_TOP + 0.06:
            kps.append(np.array([cur[0], cur[1], TABLE_TOP + 0.10]))
        kps += [dest + np.array([0, 0, 0.10]), start]
        ev = {len(kps) - 1: "press"}
        kps += [dest + np.array([0, 0, 0.10]), home]
        self.press_spec = {"force": force, "hold": hold_s, "label": label, "contact_z": float(dest[2] + tip)}
        self.done_msg = f"Pressed {label}."
        rt.set_obstacles(())
        self._launch(tid, "press", kps, palm, ev, "task", f"Pressing {label} with {force:.0f} N for {hold_s:.1f} s.",
                     loose=[(0, len(kps) - 3)])

    def tilt(self, name):
        """How far (deg) an object is from standing: a cube resting on any face is standing, anything else on its base."""
        R = self.obj_mat(name)
        m, g = self.rt.model, self.objs[name]["geom"]
        hs = m.geom_size[g]
        cube = m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and max(hs) < 1.25 * min(hs)
        up = np.max(np.abs(R[2, :])) if cube else R[2, 2]
        return float(np.degrees(np.arccos(np.clip(up, -1, 1))))

    def _placed_badly(self):
        """Physics: after a pick & place, is the object resting where it should, upright? -> reason it isn't, or None.
        Into a container (place_in): is it inside it?"""
        chk, self.place_check = getattr(self, "place_check", None), None
        if chk is not None and self.place_at is not None and self.fx is not None:
            name, cont = self.place_at[0], chk
            self.place_at = None
            if name in self.objs and not self.fx.inside(cont, self.mug_center(name), margin=0.03):
                return f"I let go of {self.pretty(name)} but it didn't end up in {cont['label']}."
            return None
        if self.plant is None or self.place_at is None:
            return None
        name, dest = self.place_at
        self.place_at = None
        if name not in self.objs:
            return None
        c = self.mug_center(name)
        err = float(np.hypot(*(c[:2] - dest[:2])))
        tilt = self.tilt(name)
        self.place_err = (name, err, tilt)
        if self.kinds.get(name) in wobj.KINDS and wobj.grasp_class(self.kinds[name]) in ("long", "flat"):
            tilt = 0.0                                # lying things lie: their tilt isn't a fall
            self.place_err = (name, err, tilt)
        if tilt > 35:
            return f"I let go of {self.pretty(name)} but it fell over ({tilt:.0f}°)."
        if err > 0.06:
            return f"I let go of {self.pretty(name)} but it ended up {err * 100:.0f} cm from the spot."
        if self.done_msg and self.done_msg.endswith("."):        # what really happened, measured
            self.done_msg = self.done_msg[:-1] + f", {err * 100:.1f} cm from the spot."
        return None

    def _event(self, name, palm):
        d = self.rt.data
        if name in ("grasp_fx", "release_fx"):            # a drawer / door handle (pps/manip.py)
            self._fx_event(name, palm)
            return
        if self.plant is not None and name == "press":     # compliance: the force loop takes over the hand target
            now = float(self.plant.d.time)
            self.ph = {"phase": "press", "palm": palm, "t0": now, "t_last": now, "fmax": 0.0, "fs": [], "on": None,
                       **getattr(self, "press_spec", {"force": 10.0, "hold": 1.5, "label": "there"})}
            self.rt.mode = "PRESS"
            self._hold_path(True)
            return
        if self.plant is not None and name == "clear":
            self.rt.set_obstacles(())
            return
        if self.plant is not None and name in ("aim", "hover"):   # stop over the spot until the arm has caught up
            self.ph = {"phase": name, "palm": palm, "obj": self.holding.get(palm) or self.cur, "t0": float(self.plant.d.time)}
            self._hold_path(True)
            return
        if self.plant is not None and name in ("grasp", "release"):
            now = float(self.plant.d.time)
            if name == "grasp":
                self.ph = {"phase": "settle", "palm": palm, "obj": self.cur, "t0": now}
            else:                                 # arrive first, then let go
                obj = self.holding.get(palm) or self.cur
                self.ph = {"phase": "arrive", "palm": palm, "obj": obj, "t0": now, "p": None, "slow_t": None}
            self._hold_path(True)
            return
        if name == "let_go":                      # kinematic two-arm handover: the giving hand is off it (the other has it)
            self.holding[palm] = None
            return
        if name == "grasp":
            self.holding[palm] = self.cur
            self.grasp_off = self.obj_pos(self.cur) - d.site_xpos[self.rt.palm_sid[palm]]
            self.grasp_palm = palm
            if self.task:
                self.task["step"] = "grasped"
        elif name == "release":
            if self.holding.get(palm) in self.objs:
                self.cur = self.holding[palm]
            self.holding[palm] = None
            self.grasp_off = None
            p = self.obj_pos(self.cur)
            c = p + self.mug_center_off
            rz = getattr(self, "release_z", None)          # place_in (kinematic): the container's floor, not the table
            p[2] = (rz if rz is not None else TABLE_TOP if self.on_table(c) else 0.0) + self.mug_half_h - self.mug_center_off[2]
            self.release_z = None
            self.set_obj_pose(self.cur, p)
            if self.task:
                self.task["step"] = "placed"

    # ------------------------------------------------------------------ actions
    def pick_place(self, tid, obj, target, target_uv, cam, aspect, via_pts=None, grasp=None, sliced=False, dest=None, hold=False,
                   action="pick_place"):
        """dest: (xyz, label) given by the caller (place_in: a spot inside a container, xyz[2] its floor); hold: pick it
        up and keep it in the hand (pick_from). Generated worlds set place_z / transit_z (pps/manip.py). sliced: the
        physics search runs a slice per server tick (search_tick)."""
        self.cur_action = action
        caps = getattr(self.rt, "capabilities", None)
        if caps is not None and "pick_place" not in caps:
            prof = getattr(self.rt, "profile", None)
            self.set_task(tid, "pick_place", "failed", "I can't pick things up with this body. " + (getattr(prof, "note", "") or ""))
            return
        grasp = grasp or getattr(getattr(self.rt, "profile", None), "grasp", "side")
        name = self.obj_name(obj)
        if name is None and not self.legacy:
            fixed = self._match(obj)[1]
            if fixed:
                self.set_task(tid, "pick_place", "failed", f"{self.pretty(fixed).capitalize()} is fixed in this scene; I can move {self.movable_list()}.")
            else:
                self.set_task(tid, "pick_place", "failed", f"I don't see '{obj}'. I can move {self.movable_list()}.")
            return
        if name is None:
            if any(k in str(obj).lower() for k in ("blue", "파란", "vase", "꽃병", "화병")):
                self.set_task(tid, "pick_place", "failed", f"{self.pretty('blue_block' if 'blu' in str(obj).lower() or '파란' in str(obj) else 'vase').capitalize()} is fixed in this scene; I can move the mug or the red block.")
            else:
                self.set_task(tid, "pick_place", "failed", f"I don't see '{obj}'. I can move the mug or the red block.")
            return
        held_other = next((o for o in self.holding.values() if o and o != name), None)
        if held_other:
            self.set_task(tid, "pick_place", "failed", f"I'm still holding {self.pretty(held_other)}.")
            return
        self.cur = name
        given = dest
        via = [np.asarray(v, float) for v in via_pts] if via_pts else None
        if given is not None or hold:                 # place_in / pick_from: the caller worked out where
            dest, label = (None, given[1] if given else "") if hold else (np.asarray(given[0], float), given[1])
        elif via is None and target and str(target).strip().lower() in ("path", "my path", "along my path", "the path", "route", "경로", "경로대로", "내 경로"):
            if self.rt.path is None or len(self.rt.path["pts"]) < 2:
                self.set_task(tid, "pick_place", "failed", "There is no route yet. Draw one on my view or in 3D first.")
                return
            via = [np.array(p) for p in self.rt.path["pts"][::2]] + [np.array(self.rt.path["pts"][-1])]
            end = via[-1]
            dest, label = np.array([end[0], end[1], TABLE_TOP]), "the end of your route"
        if given is None and not hold and not (target and str(target).strip().lower() in ("path", "my path", "along my path", "the path", "route", "경로", "경로대로", "내 경로")):
            dest, label = self.resolve(target, target_uv, cam, aspect)
        if dest is None and not hold:
            self.set_task(tid, "pick_place", "failed", label[0].upper() + label[1:] + ".")
            return
        if given is None and not hold and not self.on_table(dest):
            self.set_task(tid, "pick_place", "failed", f"I can't place it there: {label} is not on the {SURFACE}.")
            return
        fixed = given is not None or hold          # place_in / pick_from: the caller worked out where
        if self.plant is None:
            if via is None and not fixed:
                free, blocker = self.free_spot(dest, name)
                if free is None:
                    self.set_task(tid, "pick_place", "failed", f"{label[0].upper() + label[1:]} is taken by {self.pretty(blocker)} and there is no free spot nearby.")
                    return
                if blocker:
                    dest, label = free, f"{label} (beside {self.pretty(blocker)}, which is there)"
            held = next((h for h, o in self.holding.items() if o == name), None)
            c = self.mug_center()
            palm = held or self.hand_for(c)
            self._pick_plan(tid, name, palm, held, c, dest, label, via, via_pts, grasp, None, self.rt.data.mocap_quat[self.rt.hands_mid[palm]].copy())
            return
        search = self._pick_all(tid, name, dest, label, via, via_pts, grasp, fixed)
        if not sliced:
            _drain(search)
            return
        self.search = (tid, search)               # the server loop: one way in per tick, so a long search (a refusal tries
        self.set_task(tid, "pick_place", "planning", f"Working out how to pick up {self.pretty(name)}.")   # them all)
        self.search_tick()
        self._search_t = time.time()

    def search_tick(self, budget=0.25):
        """Advance a time-sliced pick search (pick_place(..., sliced=True)): ways in until about `budget` s of wall time
        is used, so the loop never stalls for a whole search. Dropped if another command took over."""
        if self.search is None:
            return
        tid, gen = self.search
        if not self.task or self.task.get("id") != tid or self.task.get("status") != "planning":
            gen.close()
            self.search = None
            return
        t0 = time.time()
        while True:
            try:
                next(gen)
            except StopIteration:
                self.search = None
                return
            if time.time() - t0 > budget:
                self.set_task(tid, "pick_place", "planning", self.task.get("message") if self.task.get("status") == "planning"
                              else f"Working out how to pick up {self.pretty(self.cur)}.")
                self.rt.path = None
                return

    def _reach_bound(self, palm):
        """A hand's reach can't exceed its arm stretched out: the sum of the link lengths from the shoulder to the hand
        (an upper bound, plus room for a waist or legs leaning; mobile bases drive, so no bound)."""
        cache = self.__dict__.setdefault("_bounds", {})
        if palm in cache:
            return cache[palm]
        rt = self.rt
        prof = getattr(rt, "profile", None)
        R = np.inf
        if prof is None or prof.base != "mobile":
            m, d = rt.configuration.model, rt.ik_data
            sh = np.asarray(rt.retargeter.robot_shoulders[palm], float)
            sid = rt.palm_sid[palm]
            b = m.site_bodyid[sid]
            L = float(np.linalg.norm(d.site_xpos[sid] - d.xpos[b]))
            while b != 0:
                pb = m.body_parentid[b]
                L += float(np.linalg.norm(d.xpos[b] - d.xpos[pb]))
                if np.linalg.norm(d.xpos[pb] - sh) < 0.03:
                    R = L + (0.15 if prof is not None and prof.base == "legged" else 0.10)
                    break
                b = pb
        cache[palm] = R
        return R

    def _pick_all(self, tid, name, dest, label, via, via_pts, grasp, fixed=False):
        """Physics pick & place after the words are resolved, resumable: where it can go (free spot), then the ways in."""
        if via is None and not fixed:
            free, blocker = yield from self._free_spot(dest, name)
            if free is None:
                self.set_task(tid, "pick_place", "failed", f"{label[0].upper() + label[1:]} is taken by {self.pretty(blocker)} and there is no free spot nearby.")
                return
            if blocker:
                dest, label = free, f"{label} (beside {self.pretty(blocker)}, which is there)"
        held = next((h for h, o in self.holding.items() if o == name), None)
        self.cur = name
        c = self.mug_center()
        palm = held or self.hand_for(c)
        grasp = (self.rt.profile.physics or {}).get("grasp", grasp) if getattr(self.rt, "profile", None) else grasp
        if self.fx is not None:                   # a generated world: what's near the pick and the drop (drawer walls too);
            # things in the same bin / drawer as the one being picked are not kept away from (the fingers go in among them)
            co = tuple(self.contents(self.container_of(name), skip=name)) if self.container_of(name) is not None else ()
            self.rt.set_obstacles((name,) + co, near=[c] + ([dest] if dest is not None else []), fixtures=True)
        else:
            self.rt.set_obstacles((name,))        # the planner keeps the gripper clear of everything else on the table
        palms = [palm] if held else [palm] + [h for h in self.hands if h != palm]   # two hands: the other one may get there
        yield from self._pick_search(tid, name, palms, held, c, dest, label, via, via_pts, grasp)

    def _pick_search(self, tid, name, palms, held, c, dest, label, via, via_pts, grasp):
        """The ways in, one by one (a generator: it yields after each failed one). Hands whose arm can't stretch to the
        object or the spot at all are out at once; if no hand can, the main way is planned once for the message, the red
        waypoints and the nearest spot it can reach. All failing: the first (main) way's verdict stands."""
        d = self.rt.data
        sh = self.rt.retargeter.robot_shoulders
        pz = TABLE_TOP if getattr(self, "place_z", None) is None else self.place_z
        place_p = None if dest is None else np.array([dest[0], dest[1], pz + self.mug_half_h])   # (pick_from: no spot)
        far = lambda p, q: float(np.linalg.norm(np.asarray(q, float) - np.asarray(sh[p], float))) > self._reach_bound(p)
        able = [p for p in palms if not ((held != p and far(p, c)) or (via is None and place_p is not None and far(p, place_p)))]
        tries = len(SIDE_WAYS) + 2 if able else 1
        palms = able or palms[:1]
        stop, first = False, None
        marks, nm = list(self.markers), self.next_marker   # a failed first way may offer a closer spot as a marker
        quats = {p: d.mocap_quat[self.rt.hands_mid[p]].copy() for p in palms}
        try:
            for k, p in enumerate(palms):
                mid = self.rt.hands_mid[p]
                q_now = quats[p]
                for variant in range(tries):      # physics: a few ways in (approach direction / jaw yaw); first feasible wins
                    d.mocap_quat[mid] = q_now
                    off, n, why = self.grasp_plan(name, p, grasp, held, variant)
                    if off is None:
                        if why is not None and k == 0 and not held:
                            self.set_task(tid, "pick_place", "failed", why)
                            return
                        break
                    if self._pick_plan(tid, name, p, held, c, dest, label, via, via_pts, grasp, (off, n), q_now, last=first is None):
                        self.markers, self.next_marker = marks, nm   # (another way got there after all)
                        quats = {}
                        return
                    if first is None:             # all failing: report the first (main) way and its suggestion
                        first = (dict(self.task), self.rt.path)
                    if getattr(self, "fail_reason", "reach") != "reach" or held:
                        stop = True
                        break
                    d.mocap_quat[mid] = q_now
                    yield
                d.mocap_quat[mid] = q_now
                if stop:
                    break
            if first is not None and not stop:
                self.task, self.rt.path = first
            self.fail_reason = "reach"
        finally:
            for p, q in quats.items():            # dropped half way (another command): the hands' targets as they were
                d.mocap_quat[self.rt.hands_mid[p]] = q

    def _pick_plan(self, tid, name, palm, held, c, dest, label, via, via_pts, grasp, plan, q_now, last=True, yaw=None, lifts=True):
        """Keypoints of a pick & place (approach, grasp, carry, release, retreat) -> _launch. plan: (offset, approach) with
        physics, None for the kinematic grasp. Physics, from above: the hand may turn about the vertical to set it down
        (yaw; tried in the order that leaves the hand most room). lifts: the way in and the way home go over the other
        objects (planned again without, if those waypoints are what the arm can't reach). -> launched?"""
        d = self.rt.data
        mid = self.rt.hands_mid[palm]
        q_grip = d.mocap_quat[mid].copy()          # grasp_plan sets the orientation the grasp needs
        if yaw is None and plan is not None and plan[1][2] <= -0.7 and via is None and dest is not None:
            pz = TABLE_TOP if getattr(self, "place_z", None) is None else self.place_z
            place_c = np.array([dest[0], dest[1], pz + self.mug_half_h])
            first = None
            yaws = self._place_yaw(name, place_c, plan[0], q_grip, palm)
            if np.hypot(*plan[0][:2]) >= 0.02:    # held off-centre (rim, an end): turning swings it, so only if it must
                yaws = [0.0] + [y for y in yaws if y]
            for k, y in enumerate(yaws):
                d.mocap_quat[mid] = q_grip
                if self._pick_plan(tid, name, palm, held, c, dest, label, via, via_pts, grasp, plan, q_now, last=last and k == 0, yaw=y):
                    return True
                first = first or (dict(self.task), self.rt.path)
                msg = self.task.get("message", "")
                miss = re.search(r"(\d+) of \d+ waypoints .* worst by (\d+) cm", msg)
                if getattr(self, "fail_reason", "reach") != "reach" or not (
                        "exactly to the spot" in msg or (miss and (int(miss[1]) == 1 or int(miss[2]) <= 3))):
                    break     # (a turned hand may fit, or get past one awkward waypoint; it won't make up for a far spot)
            self.task, self.rt.path = first
            return False
        cur = d.site_xpos[self.rt.palm_sid[palm]].copy()
        if plan is None:
            off, n = (np.array([0.0, 0.0, self.mug_half_h + 0.035]) if grasp == "top" else
                      np.array([-(self.mug_r + 0.015), 0.0, 0.01])), None
            if held is not None:                  # already in the hand: where it really sits (after a two-arm handover
                off = d.site_xpos[self.rt.palm_sid[palm]] - self.mug_center(name)   # it's held by an end)
        else:
            off, n = plan
        hold = dest is None                        # pick_from: pick it up and keep it
        tz = getattr(self, "transit_z", None)      # generated worlds: carry over the counter / out of the drawer
        up = (lambda p, dz: np.array([p[0], p[1], max(p[2] + dz, tz)])) if tz is not None else (lambda p, dz: p + np.array([0, 0, dz]))
        pz = TABLE_TOP if getattr(self, "place_z", None) is None else self.place_z
        self.release_z = getattr(self, "place_z", None)    # kinematic mode sets the object down there at the release
        place_c = None if hold else np.array([dest[0], dest[1], pz + self.mug_half_h])
        drop = getattr(self, "place_drop", 0.0) or 0.0
        sidegrasp = n is not None and n[2] > -0.7
        nh = n if sidegrasp else None             # side grasps come in (and back out) along the approach axis
        q_place, off_g, nh_p = q_grip, off, nh
        if yaw:                                   # physics: turn the hand about the vertical so it fits where it goes
            qz = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
            q_place = np.zeros(4)
            mujoco.mju_mulQuat(q_place, qz, q_grip)
            rz = lambda v: np.array([np.cos(yaw) * v[0] - np.sin(yaw) * v[1], np.sin(yaw) * v[0] + np.cos(yaw) * v[1], v[2]])
            off = rz(off)
            nh_p = rz(nh) if nh is not None else None
        self.cur_off = off
        place_hand = None if hold else place_c + off + (np.array([0, 0, PLACE_GAP + drop]) if n is not None else 0.0)   # let go just above
        kps, ev, lifted = [cur], {}, False
        home = self.home_palm[palm]
        over = n is not None and home[2] < TABLE_TOP + 0.06 and home[0] < TABLE_X[0] + 0.05   # hands start below the edge
        if over and cur[2] < TABLE_TOP + 0.06:    # physics: up over the table's edge first (a straight line would cut it)
            kps.append(np.array([cur[0], cur[1], TABLE_TOP + 0.10]))
        if held is None:
            g = c + off_g
            if n is None:
                pre = g + (np.array([0, 0, 0.09]) if np.array_equal(off[:2], [0, 0]) else np.array([-0.08, 0, 0.07]))
            else:                                 # physics: come in along the approach axis with the jaws open
                pre = g - nh * 0.10 + np.array([0, 0, 0.03]) if sidegrasp else g + np.array([0, 0, self._pre_dz(name, palm, g)])
                a0 = kps[-1]                      # ... over the other objects on the way there for a body whose arm trails
                # its plan by several cm in transit (profile physics.transit_over: TidyBot), not skimming past them (the
                over_in = lifts and bool((getattr(getattr(self.rt, "profile", None), "physics", None) or {}).get("transit_over"))
                zl = self._clear_z(a0, pre, palm, sidegrasp, skip=(name,)) if over_in else -np.inf   # arm trails the plan)
                if zl > min(a0[2], pre[2]) + 0.005:
                    kps += [np.array([a0[0], a0[1], zl])] if a0[2] < zl else []
                    kps += [np.array([pre[0], pre[1], zl])] if pre[2] < zl else []
                    lifted = True
            az = getattr(self, "approach_z", None)  # generated worlds: over a machine's walls to get above it first
            if az is not None and az > pre[2] + 0.01:
                kps.append(np.array([pre[0], pre[1], az]))
            kps += [pre, g]
            ev[len(kps) - 1] = "grasp"
            kps += [up(g, LIFT if n is not None else 0.12)]
        if hold:                                  # pick_from: lifted clear, held there
            if n is not None and held is None:
                self.plant.grip_cmd(self.role(palm), "open")
            quats = None if n is None else [q_now] + [q_grip] * (len(kps) - 1)
            what = self.pretty(name)
            self.done_msg = f"I'm holding {what}" + (f", taken from {label}." if label else ".")
            gk = [k for k, e in ev.items() if e == "grasp"]
            loose = ([(0, gk[0] - 1)] if gk and gk[0] > 1 else []) if n is not None else None
            side = "right" if palm.startswith("right") else "left"
            ok = self._launch(tid, getattr(self, "cur_action", "pick_from"), kps, palm, ev, "task",
                              f"Every waypoint is reachable. Taking {what}" + (f" from {label}" if label else "") + f" with my {self.limb(side)}.",
                              quats=quats, loose=loose)
            self.fail_reason = "reach" if ok else getattr(self, "fail_reason", "reach")
            return ok
        if via is not None:
            kps += [np.array(v) for v in via]
            if via_pts is None:
                label = "the end of your route"
        above = up(place_hand, LIFT if n is not None else 0.12)
        if n is not None and via is None:         # physics: carry it high enough to clear everything it passes over
            a = kps[-1] if held is None else cur
            zc = self._carry_z(name, a, above, off)
            if zc > min(a[2], above[2]):
                zt = max(zc, a[2], above[2])
                if held is None:
                    kps[-1] = np.array([a[0], a[1], zt])
                elif zt > cur[2] + 0.01:
                    kps.append(np.array([cur[0], cur[1], zt]))
                above = np.array([above[0], above[1], zt])
        ia = len(kps)                             # from here on the hand has the place orientation
        kps += [above, place_hand]
        if n is not None:                         # physics: look where it really sits in the hand before setting it down
            ev[len(kps) - 2] = "aim"
        ev[len(kps) - 1] = "release"
        if n is None:
            retreat = place_hand + np.array([-0.10, 0, 0.07])
        else:                                     # back off along the approach axis before going home
            retreat = place_hand - nh_p * 0.10 + np.array([0, 0, 0.03]) if sidegrasp else up(place_hand, 0.10)
            self.place_at = (name, place_c)
            if held is None:
                self.plant.grip_cmd(self.role(palm), "open", to=getattr(self, "grasp_open", None))
        kps += [retreat]
        if n is not None:                         # physics: backed off: from here the hand keeps clear of what it set down
            ev[len(kps) - 1] = "clear"
            nxt = np.array([home[0], home[1], TABLE_TOP + 0.10]) if over else home
            zl = self._clear_z(retreat, nxt, palm, sidegrasp, placed=(name, place_c)) if lifts else -np.inf
            if zl > min(retreat[2], nxt[2]) + 0.005:   # ... and goes over it (and its neighbours) on the way home, down
                kps += [np.array([retreat[0], retreat[1], zl])] if retreat[2] < zl else []   # only once past them
                kps += [np.array([nxt[0], nxt[1], zl])] if nxt[2] < zl else []
                lifted = True
        kps += ([np.array([home[0], home[1], TABLE_TOP + 0.10])] if over else []) + [home]
        quats = None
        if n is not None:                         # the hand turns to the grasp orientation, keeps it, and turns back home
            # (back over the table's edge with the home orientation already)
            quats = [q_now] + [q_grip] * (ia - 1) + [q_place] * (len(kps) - ia - 1 - int(over)) + [self.home_quat[palm]] * (1 + int(over))
        if via is None and self.keepout:   # keep-out zones: detour around or over them (adds e.g. ", going around zone A")
            old = list(kps)
            kps, ev, note = self.keepout.route(kps, ev, palm)
            label += note
            if quats is not None and len(kps) != len(old):     # detour points take the orientation of the point before
                qmap, j = [], 0
                for p in kps:
                    if j < len(old) and np.allclose(p, old[j]):
                        j += 1
                    qmap.append(quats[max(0, j - 1)])
                quats = qmap
        side = "right" if palm.startswith("right") else "left"
        what = self.pretty(name)
        into = getattr(self, "cur_action", "pick_place") == "place_in"
        self.done_msg = f"Put {what} in {label}." if into else f"Placed {what} on {label}."
        loose = None
        if n is not None:                         # physics: getting there and going home are transit, not the work
            gk = [k for k, e in ev.items() if e == "grasp"]
            rk = [k for k, e in ev.items() if e == "release"]
            loose = ([(0, gk[0] - 1)] if gk and gk[0] > 1 else []) + ([(rk[-1], len(kps) - 1)] if rk else [])
        ok = self._launch(tid, getattr(self, "cur_action", "pick_place"), kps, palm, ev, "task",
                          "Every waypoint is reachable. " + (f"Putting {what} in {label} with my {self.limb(side)}." if into else
                                                             f"Putting {what} on {label} with my {self.limb(side)}." if held else
                                                             f"Picking up {what} with my {self.limb(side)} and placing it on {label}."),
                          quats=quats, loose=loose)
        if not ok:
            self.place_at = None
        if not ok and lifted and getattr(self, "fail_reason", "reach") == "reach":   # the waypoints over the others are
            self.fail_reason = "reach"                                                 # what it can't do: the plain way
            return self._pick_plan(tid, name, palm, held, c, dest, label, via, via_pts, grasp, plan, q_now, last=last, yaw=yaw, lifts=False)
        if not ok and last and getattr(self, "fail_reason", "reach") == "reach" and via is None and not into:
            self.suggest(tid, dest, palm, off)
        reason = getattr(self, "fail_reason", "reach")
        self.fail_reason = "reach"
        if not ok and reason != "reach":
            self.fail_reason = reason             # the caller stops trying other grasps (vase, zone)
        return ok

    def grasp_waypoints(self, name, palm, cur):
        """Approach, grasp and lift keypoints after `cur` (features that build their own tasks: pick & hold, handover).
        Physics: the first way in the planner can trace, with an orientation per keypoint. -> (kps, quats | None, index of
        the grasp in [cur] + kps)."""
        c, o = self.mug_center(name), self.objs[name]
        if self.plant is None:                    # the UX 2.0 side grasp (the object rides the hand)
            off = np.array([-(o["r"] + 0.015), 0.0, 0.01])
            self.cur_off = off
            g = c + off
            return [g + np.array([-0.08, 0, 0.07]), g, g + np.array([0, 0, 0.14])], None, 2
        rt, d = self.rt, self.rt.data
        mid = rt.hands_mid[palm]
        q_now = d.mocap_quat[mid].copy()
        style = (rt.profile.physics or {}).get("grasp", rt.profile.grasp)
        rt.set_obstacles((name,))
        self.plant.grip_cmd(self.role(palm), "open")
        home, best = self.home_palm[palm], None
        lift = [np.array([cur[0], cur[1], TABLE_TOP + 0.10])] if (home[2] < TABLE_TOP + 0.06 and home[0] < TABLE_X[0] + 0.05
                                                                and cur[2] < TABLE_TOP + 0.06) else []
        for variant in range(len(SIDE_WAYS) + 2):
            d.mocap_quat[mid] = q_now
            off, n, why = self.grasp_plan(name, palm, style, None, variant)
            if off is None:
                break
            q_g = d.mocap_quat[mid].copy()
            g = c + off
            pre = g - n * 0.10 + np.array([0, 0, 0.03]) if n[2] > -0.7 else g + np.array([0, 0, self._pre_dz(name, palm, g)])
            kps = lift + [pre, g, g + np.array([0, 0, 0.12])]
            quats = [q_now] + [q_g] * len(kps)
            pts, qs, _ = self._dense([cur] + kps, quats)
            plan = (kps, quats, len(kps) - 1, off)
            rt.orient(True)
            if all(rt.precheck(pts, palm, quats=qs)):
                best = plan
                break
            best = best or plan
        d.mocap_quat[mid] = q_now
        if best is None:
            return [c + np.array([0, 0, 0.09]), c, c + np.array([0, 0, 0.12])], None, 2
        self.cur_off = best[3]
        return best[0], best[1], best[2]

    @staticmethod
    def _dense(keypoints, quats=None):
        """Keypoints -> densified points (+ slerped orientations) and the index of every keypoint in them."""
        idx_map, pts, qs = [], [], []
        for k, kp in enumerate(keypoints):
            if k == 0:
                pts.append(np.asarray(kp, float))
                idx_map.append(0)
                if quats is not None:
                    qs.append(np.asarray(quats[0], float))
                continue
            seg = densify([pts[-1], kp])[1:]
            pts.extend(seg)
            if quats is not None:
                qs += [slerp(quats[k - 1], quats[k], (j + 1) / len(seg)) for j in range(len(seg))]
            idx_map.append(len(pts) - 1)
        return np.array(pts), (np.array(qs) if quats is not None else None), idx_map

    def grasp_plan(self, name, palm, style, held, variant=0):
        """Physics: where the gripper's TCP goes relative to the object centre, and from which direction.
        -> (offset, approach direction, None) or (None, None, why not); variant 0, 1, 2 are different ways in
        (side: from the shoulder / straight ahead / in between; top: jaw yaw 0 / +90 / -90 degrees).
        Sets the hand target's orientation."""
        rt, pl = self.rt, self.plant
        role = self.role(palm)
        g = pl.grip.get(role)
        what = self.pretty(name)
        self.grasp_open = None                    # how far the jaws open for this grasp (None: all the way)
        if g is None:
            return None, None, f"I have no gripper on my {self.limb(role)}: I can't pick up {what}."
        sid, mid = rt.palm_sid[palm], rt.hands_mid[palm]
        tcp = rt.data.site_xpos[sid].copy()
        c = self.mug_center(name)
        if held:                                  # already in hand: keep the grip it has
            if variant:
                return None, None, None
            R = rt.data.site_xmat[sid].reshape(3, 3)
            return tcp - c, R @ g["approach"], None
        Rt = np.zeros(9)
        mujoco.mju_quat2Mat(Rt, rt.data.mocap_quat[mid])
        Rt = Rt.reshape(3, 3)
        Ro = self.obj_mat(name)
        kind = pl.objects.get(name, {}).get("kind", "block")
        cls = wobj.grasp_class(kind)              # box | round | long | flat | mug (pps/world/objects.py)
        boxy = kind in ("block", "box") or (kind in wobj.KINDS and wobj.KINDS[kind]["shape"] == "box" and cls in ("box", "flat"))
        if kind not in wobj.KINDS and cls not in ("mug",):
            cls = "box" if kind in ("block", "box") else "round"   # the basic kinds keep their old handling
        o = self.objs[name]
        top_z, w = c[2] + o["half_h"], g["width"]
        sup = self.support_z(name)                # what it rests on: the table, a drawer's floor, a shelf (generated worlds)
        if self.fx is not None:
            top_z = sup + 2 * self.vertical_half(name)
        if style == "side" and variant >= len(SIDE_WAYS) and (kind in ("block", "box", "can", "cup", "bottle", "fruit") or
                                                             (cls in ("box", "round", "long", "flat") and kind in wobj.KINDS)) \
                and variant - len(SIDE_WAYS) < 2:  # small things: from above as a last resort (doesn't sweep its neighbours)
            sh = rt.retargeter.robot_shoulders[palm]
            h = np.array([c[0] - sh[0], c[1] - sh[1], 0.0])
            h = h / (np.linalg.norm(h) or 1.0)
            a = np.cross([0.0, 0.0, 1.0], h) if variant == len(SIDE_WAYS) else h
            n = np.array([0.0, 0.0, -1.0])
            if boxy:          # jaws square to the block's faces
                psi = np.arctan2(Ro[1, 0], Ro[0, 0])
                ang = np.arctan2(a[1], a[0])
                ang = psi + np.round((ang - psi) / (np.pi / 2)) * np.pi / 2
                a = np.array([np.cos(ang), np.sin(ang), 0.0])
            elif cls == "long":                   # across the handle
                u = Ro[:, 0]
                a = np.cross([0.0, 0.0, 1.0], [u[0], u[1], 0.0])
                a = a / (np.linalg.norm(a) or 1.0)
            L = np.column_stack([g["axis"], g["approach"], np.cross(g["axis"], g["approach"])])
            W = np.column_stack([a, n, np.cross(a, n)])
            mujoco.mju_mat2Quat(rt.data.mocap_quat[mid], (W @ L.T).flatten())
            z = max(top_z - min(0.025, 0.8 * o["half_h"]), sup + g["tip"] + 0.006)
            style = "top"
        elif style == "side":                     # humanoid hands / Stretch: from the robot's side, jaws closing horizontally
            if variant >= len(SIDE_WAYS):
                return None, None, None
            sh = rt.retargeter.robot_shoulders[palm]
            n = np.array([c[0] - sh[0], c[1] - sh[1], 0.0])
            n = n / (np.linalg.norm(n) or 1.0)
            heading, pitch = SIDE_WAYS[variant]
            n = [n, np.array([1.0, 0, 0]), n + np.array([1.0, 0, 0])][heading]
            n = n / (np.linalg.norm(n) or 1.0)
            a = np.cross([0.0, 0.0, 1.0], n)
            if np.dot(a, Rt @ g["axis"]) < 0:
                a = -a
            n = np.cos(pitch) * n + np.array([0, 0, -np.sin(pitch)])     # pitched down about the (level) jaw axis
            L = np.column_stack([g["axis"], g["approach"], np.cross(g["axis"], g["approach"])])
            W = np.column_stack([a, n, np.cross(a, n)])
            if g.get("up") is not None and (W @ L.T @ g["up"])[2] < 0:   # a hand with a top side (Stretch: the camera)
                a = -a                                                    # keeps it up: the jaws are symmetric
                W = np.column_stack([a, n, np.cross(a, n)])
            mujoco.mju_mat2Quat(rt.data.mocap_quat[mid], (W @ L.T).flatten())
            z = max(c[2], sup + g["tip"] * np.sin(pitch) + g.get("pad_h", 0.018) * np.cos(pitch) + 0.006)
        else:
            if variant >= 3 or (cls == "long" and variant >= 2):
                return None, None, None
            n = np.array([0.0, 0.0, -1.0])
            a = Rt @ g["axis"]
            dzs = [0.0, np.pi / 2, -np.pi / 2]
            phi = np.arctan2(a[1], a[0])
            if boxy:   # square up the jaws with the block's faces (smallest wrist turn)
                psi = np.arctan2(Ro[1, 0], Ro[0, 0])
                k = np.round((phi - psi) / (np.pi / 2))
                dzs = [d + psi + k * np.pi / 2 - phi for d in dzs]
            elif kind == "mug":                   # jaws across the handle's direction: they hold the round body only
                psi = np.arctan2(-Ro[1, 1], -Ro[0, 1]) + np.pi / 2
                dzs = [d + psi + np.round((phi - psi) / np.pi) * np.pi - phi for d in dzs]
            elif cls == "long":                   # across the handle (either way round: the jaws are symmetric)
                psi = np.arctan2(Ro[1, 0], Ro[0, 0]) + np.pi / 2
                base = psi + np.round((phi - psi) / np.pi) * np.pi - phi
                dzs = [base, base + np.pi]
            if self.fx is not None and cls not in ("long", "mug"):     # a busy world: the side with room for the fingers first
                hs0 = pl.m.geom_size[o["geom"]]

                def span(dz):                     # how wide the object is between the jaws turned by dz
                    if not boxy:
                        return 2 * o["r"]
                    aa = np.array([np.cos(phi + dz), np.sin(phi + dz), 0.0])
                    return 2 * (abs(np.dot(aa, Ro[:, 0])) * hs0[0] + abs(np.dot(aa, Ro[:, 1])) * hs0[1])
                dzs = self.jaw_order(name, c, phi, dzs, w, span)
            dz = dzs[variant]
            if dz:
                qz = np.array([np.cos(dz / 2), 0, 0, np.sin(dz / 2)])
                q = np.zeros(4)
                mujoco.mju_mulQuat(q, qz, rt.data.mocap_quat[mid].copy())
                rt.data.mocap_quat[mid] = q
                a = np.array([np.cos(dz) * a[0] - np.sin(dz) * a[1], np.sin(dz) * a[0] + np.cos(dz) * a[1], a[2]])
            z = max(top_z - min(0.025, 0.8 * o["half_h"]), sup + g["tip"] + 0.006)
        a = np.array([a[0], a[1], 0.0]) / (np.hypot(a[0], a[1]) or 1.0)
        if boxy:
            hs = pl.m.geom_size[o["geom"]]
            ext = 2 * (abs(np.dot(a, Ro[:, 0])) * hs[0] + abs(np.dot(a, Ro[:, 1])) * hs[1])
        else:
            ext = 2 * o["r"]
        p = np.array([c[0], c[1], z])
        if cls == "long":                         # held at its grip (a spoon's handle), as wide as it is there
            hs = pl.m.geom_size[o["geom"]]
            gp = wobj.grip(kind, hs)
            if gp is not None:
                gx = c + Ro @ np.array([gp[0], 0.0, 0.0])
                p, ext = np.array([gx[0], gx[1], z]), gp[1]
        if kind == "mug":                         # its real outline along the jaws: the body and, maybe, the handle
            from .physics import MUG
            s = o["r"] / 0.065
            body = self.obj_pos(name) + Ro @ np.array([0, MUG["cy"] * s, 0])
            handle = Ro @ np.array([0, -1.0, 0])
            tip = float(np.dot(handle, a)) * (MUG["R"] + 0.031) * s
            lo, hi = min(-MUG["R"] * s, tip), max(MUG["R"] * s, tip)
            mid = body + a * (lo + hi) / 2
            p, ext = np.array([mid[0], mid[1], z]), hi - lo
        if kind == "mug" and style != "side" and ext > w - 0.012:     # too wide for the jaws: hold it by the rim
            sh = rt.retargeter.robot_shoulders[palm]
            side = -np.sign(np.dot(a, handle)) if abs(np.dot(a, handle)) > 0.5 else \
                (1.0 if np.linalg.norm(body[:2] + a[:2] * 0.04 - sh[:2]) < np.linalg.norm(body[:2] - a[:2] * 0.04 - sh[:2]) else -1.0)
            if g.get("jaw") is not None and side > 0:   # one fixed jaw: it goes outside the wall, the finger inside (a
                q = np.zeros(4)                          # bulky jaw inside the mug hooks it on the way up): the same rim,
                hm = rt.hands_mid[palm]                  # hand turned 180°
                mujoco.mju_mulQuat(q, np.array([0.0, 0.0, 0.0, 1.0]), rt.data.mocap_quat[hm].copy())
                rt.data.mocap_quat[hm] = q
                a, side = -a, -side
            rim = body + side * a * (MUG["R"] - MUG["t"] / 2) * s
            p = np.array([rim[0], rim[1], self.obj_pos(name)[2] + MUG["z1"] * s - 0.02])
            ext = MUG["t"] * s
        elif g.get("wide_open") is not None and style != "side" and ext > 0.7 * w:   # nearly fills a claw whose finger
            self.grasp_open = g["wide_open"]      # swings down in an arc (Spot): open it wider, so it comes down outside
            if g.get("deep") is not None:         # the far side instead of landing on the top, and hold lower on the body
                p[2] = max(top_z - min(float(g["deep"]), 0.8 * o["half_h"]), TABLE_TOP + g["tip"] + 0.006)
        if ext > w - 0.006:
            return None, None, (f"{what.capitalize()} is {ext * 100:.0f} cm wide and my gripper opens {w * 100:.0f} cm, "
                                "so I can't get a grip on it.")
        if g.get("jaw") is not None:              # one fixed jaw (Spot): its face goes just beside the object, the finger
            p = p - a * (ext / 2 + g["jaw"] + 0.004)   # closes from the other side and presses it against the jaw
        return p - c, n, None

    # ------------------------------------------------------------------ two arms: pass an object from one hand to the other
    def _top_quat(self, palm, a):
        """Hand orientation for a grasp from above with the jaws closing along the horizontal direction a (physics)."""
        g = self.plant.grip[self.role(palm)]
        a = np.array([a[0], a[1], 0.0]) / (np.hypot(a[0], a[1]) or 1.0)
        n = np.array([0.0, 0.0, -1.0])
        L = np.column_stack([g["axis"], g["approach"], np.cross(g["axis"], g["approach"])])
        W = np.column_stack([a, n, np.cross(a, n)])
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, (W @ L.T).flatten())
        return q

    def _ends(self, name, giver):
        """The object's long horizontal axis u (pointing away from the giver's end), its half length and width."""
        o, m = self.objs[name], self.rt.model
        g, R = o["geom"], self.obj_mat(name)
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            hs = m.geom_size[g]
            k = 0 if hs[0] >= hs[1] else 1
            u, half, width = R[:, k].copy(), float(hs[k]), float(2 * hs[1 - k])
        else:
            u, half, width = np.array([1.0, 0.0, 0.0]), float(o["r"]), float(2 * o["r"])
        u = np.array([u[0], u[1], 0.0]) / (np.hypot(u[0], u[1]) or 1.0)
        sh = self.rt.retargeter.robot_shoulders[giver]
        if np.dot(u[:2], sh[:2] - self.mug_center(name)[:2]) > 0:
            u = -u                                # the giver holds the end nearest to it
        return u, half, width

    def handover_plan(self, obj=None, to=None):
        """Two-arm bodies (ALOHA): who gives, who takes, where the hands meet. -> (plan, None) | (None, why not)."""
        caps = set(getattr(self.rt, "capabilities", ()) or ())
        if len(self.hands) < 2 or not ({"bimanual", "handover"} & caps):
            return None, "I have one arm: I can't pass things from one hand to the other. Set it down and pick it up again."
        held_any = next((o for o in self.holding.values() if o), None)
        name = (self.obj_name(obj) if obj and str(obj).strip().lower() not in ("it", "this", "that") else None) or held_any
        if name is None or name not in self.objs:
            return None, f"I don't see '{obj}'. I can pass {self.movable_list()}."
        held = next((h for h, o in self.holding.items() if o == name), None)
        to = str(to or "").strip().lower().split()[0] if to else ""
        if to in ("left", "right"):
            receiver = f"{to}_palm"
            giver = held or next(h for h in self.hands if h != receiver)
        else:
            giver = held or self.hand_for(self.mug_center(name))
            receiver = next(h for h in self.hands if h != giver)
        if giver == receiver:
            return None, f"{self.pretty(name).capitalize()} is already in my {self.limb(receiver[:-5])}."
        u, half, width = self._ends(name, giver)
        what = self.pretty(name)
        if self.plant is not None:
            w = min(self.plant.grip[self.role(p)]["width"] for p in (giver, receiver))
            if width > w - 0.006:
                return None, f"{what.capitalize()} is {width * 100:.0f} cm wide and my grippers open {w * 100:.0f} cm."
        d = half - HANDOVER_END
        if d < HANDOVER_GAP / 2:
            return None, (f"{what.capitalize()} is {2 * half * 100:.0f} cm long: too short for both my grippers at once (they need "
                          f"about {(HANDOVER_GAP + 2 * HANDOVER_END) * 100:.0f} cm). I can set it down in the middle and pick it up "
                          "with my other hand instead.")
        sg, sr = self.rt.retargeter.robot_shoulders[giver], self.rt.retargeter.robot_shoulders[receiver]
        um = np.array([sr[0] - sg[0], sr[1] - sg[1], 0.0])
        um /= np.linalg.norm(um) or 1.0
        meet = np.array([(sg[0] + sr[0]) / 2, (sg[1] + sr[1]) / 2, TABLE_TOP + HANDOVER_Z])
        return {"name": name, "giver": giver, "receiver": receiver, "held": held, "u": u, "um": um, "d": d, "meet": meet}, None

    def _grip_z(self, name, c, palm):
        """TCP height for a grasp from above (as grasp_plan's)."""
        o = self.objs[name]
        tip = self.plant.grip[self.role(palm)]["tip"] if self.plant is not None else 0.02
        return max(c[2] + o["half_h"] - min(0.025, 0.8 * o["half_h"]), TABLE_TOP + tip + 0.006) - c[2]

    def handover_arms(self, tid, obj=None, to=None, target=None, cam=None, aspect=4 / 3):
        """Pass an object between the two hands: the giving arm picks it (by the end nearest it), both arms meet in the
        middle, the receiving gripper closes on the free end, the hold is checked, the giving gripper opens and backs off,
        then the receiver puts it on `target` (or keeps it). Physics: held by contact all the way, no gluing."""
        plan, why = self.handover_plan(obj, to)
        if plan is None:
            self.set_task(tid, "handover", "failed", why)
            return
        name, giver, receiver, d = plan["name"], plan["giver"], plan["receiver"], plan["d"]
        what, gs, rs = self.pretty(name), giver[:-5], receiver[:-5]
        rt = self.rt
        self.cur = name
        rt.set_obstacles((name,))
        self.seq, self.seq_tid = [], tid
        phys = self.plant is not None
        if phys:
            self.plant.grip_cmd(self.role(receiver), "open")

        def present():                            # 1. the giver picks it up and holds it out in the middle
            c, u, um, meet = self.mug_center(name), plan["u"], plan["um"], plan["meet"]
            cur = rt.data.site_xpos[rt.palm_sid[giver]].copy()
            mid = rt.hands_mid[giver]
            q_now = rt.data.mocap_quat[mid].copy()
            th = float(np.arctan2(um[1], um[0]) - np.arctan2(u[1], u[0]))
            Rz = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
            qz = np.array([np.cos(th / 2), 0, 0, np.sin(th / 2)])
            if plan["held"]:
                off = cur - c
                q1 = q_now
                kps, ev = [cur, cur + np.array([0, 0, LIFT])], {}
            else:
                g = c - u * d
                g[2] = c[2] + self._grip_z(name, c, giver)
                off = g - c
                q1 = self._top_quat(giver, np.cross([0, 0, 1.0], u)) if phys else q_now
                if phys:
                    self.plant.grip_cmd(self.role(giver), "open")
                kps, ev = [cur, g + np.array([0, 0, 0.09]), g], {2: "grasp"}
                kps.append(g + np.array([0, 0, LIFT]))
            qm = np.zeros(4)
            mujoco.mju_mulQuat(qm, qz, q1)
            kps.append(meet + Rz @ off)
            quats = ([q_now] + [q1] * (len(kps) - 2) + [qm]) if phys else None
            self.done_msg = f"Holding {what} out in the middle."
            ok = self._launch(tid, "handover", kps, giver, ev, "task",
                              f"Passing {what} from my {gs} hand to my {rs} hand: picking it up and holding it out in the middle.",
                              quats=quats, loose=[(0, 1)] if len(kps) > 3 else None)
            if not ok:
                self.seq = []

        def take():                               # 2. the receiver closes on the free end
            if self.holding.get(giver) != name:
                self.set_task(tid, "handover", "failed", f"I lost {what} on the way: it isn't in my {gs} hand any more.")
                self.seq = []
                return
            c = self.mug_center(name)
            u, _, _ = self._ends(name, giver)     # where it really is now (a long thing held by one end droops a little)
            R = self.obj_mat(name)
            k = int(np.argmax([abs(np.dot(R[:, j], u)) for j in range(3)]))
            u3 = R[:, k] * np.sign(np.dot(R[:, k], u))
            end = c + u3 * d
            r = np.array([end[0], end[1], end[2] + self._grip_z(name, end, receiver)])
            cur = rt.data.site_xpos[rt.palm_sid[receiver]].copy()
            q_now = rt.data.mocap_quat[rt.hands_mid[receiver]].copy()
            q2 = self._top_quat(receiver, np.cross([0, 0, 1.0], u)) if phys else q_now
            kps = [cur, r + np.array([0, 0, 0.06]), r] + ([] if phys else [r + np.array([0, 0, 0.005])])
            self.done_msg = f"Took {what} with my {rs} hand."           # (kinematic: an event fires passing a waypoint,
                                                                         # so the grasp isn't the last one)
            ok = self._launch(tid, "handover", kps, receiver, {2: "grasp"}, "task",
                              f"My {rs} hand takes the free end of {what}.", quats=[q_now, q2, q2] if phys else None, loose=[(0, 1)])
            if not ok:
                self.seq = []

        def let_go():                             # 3. checked: the receiver holds it; the giver opens and backs off
            if phys:
                ta, tb = self.plant.touching(self.role(receiver), name)
                if not (ta and tb) or self.holding.get(receiver) != name:
                    self.holding[receiver] = None
                    self.held.pop(receiver, None)
                    self.plant.grip_cmd(self.role(receiver), "open")
                    self.set_task(tid, "handover", "failed", f"My {rs} hand couldn't get a firm grip on {what}, so my {gs} hand keeps it.")
                    self.seq = []
                    return
            cur = rt.data.site_xpos[rt.palm_sid[giver]].copy()
            home = self.home_palm[giver]
            q_now = rt.data.mocap_quat[rt.hands_mid[giver]].copy()
            if phys:
                kps, ev = [cur, cur + np.array([0, 0, 0.003]), cur + np.array([0, 0, LIFT]), home], {1: "release"}
                quats = [q_now, q_now, q_now, self.home_quat[giver]]
            else:                                 # kinematic: it rides the receiving hand; the giver lets go a moment later
                kps, ev, quats = [cur, cur + np.array([0, 0, 0.05]), cur + np.array([0, 0, LIFT]), home], {1: "let_go"}, None
            self.done_msg = (f"Passed {what} from my {gs} hand to my {rs} hand." +
                             ("" if target else f" It's in my {rs} hand."))
            ok = self._launch(tid, "handover", kps, giver, ev, "task", f"My {rs} hand has {what}; my {gs} hand lets go.",
                              quats=quats, loose=[(1, len(kps) - 1)])
            if not ok:
                self.seq = []

        def place():                              # 4. the receiver puts it where it goes
            self.pick_place(tid, name, target, None, cam, aspect)
            if self.task and self.task.get("status") == "running":
                self.task["action"] = "handover"
                self.done_msg = f"Passed {what} from my {gs} hand to my {rs} hand. " + (self.done_msg or "")

        self.seq = [take, let_go] + ([place] if target else [])
        present()

    def suggest(self, tid, dest, palm, off):
        """Infeasible target: walk it toward the robot until the hand can place there, and offer that spot as a marker."""
        sh = self.rt.retargeter.robot_shoulders[palm]
        for k in range(1, 10):
            cand = np.array(dest, float)
            direction = np.array([sh[0] - cand[0], sh[1] - cand[1]])
            n = np.linalg.norm(direction)
            if n < 1e-6:
                break
            cand[:2] += direction / n * 0.04 * k
            if not self.on_table(cand):
                break
            hand = np.array([cand[0], cand[1], TABLE_TOP + self.mug_half_h]) + off
            okp = self.rt.precheck(np.array([hand + np.array([0, 0, 0.12]), hand]), palm)
            if all(okp):
                mid = self.add_marker(np.array([cand[0], cand[1], TABLE_TOP]))
                self.task["message"] += f" The closest spot I can reach is marker {mid}, {4 * k} cm nearer. Say 'put it on {mid}'."
                return

    def reach(self, tid, target, target_uv, cam, aspect):
        dest, label = self.resolve(target, target_uv, cam, aspect)
        if dest is None:
            self.set_task(tid, "reach", "failed", label[0].upper() + label[1:] + ".")
            return
        palm = self.hand_for(dest)
        cur = self.rt.data.site_xpos[self.rt.palm_sid[palm]].copy()
        self.done_msg = f"My hand is over {label}."
        self._launch(tid, "reach", [cur, np.array([dest[0], dest[1], dest[2] + HOVER + 0.04])], palm, {}, "task",
                     f"Reaching over {label}.")

    # ------------------------------------------------------------------ paths vs the objects on the table (physics)
    def hand_points(self, palm):
        """The hand as points relative to its TCP (world axes), as it is held now: with physics the corners of the
        gripper's collision geoms (a drawn path keeps the hand's orientation), else a 3 cm point hand."""
        pl = self.plant
        role = self.role(palm)
        if pl is None or role not in pl.ee_bodies:
            return hand_disc(0.02, 0.03)
        m, d = pl.m, pl.d
        tcp = d.site_xpos[self.rt.palm_sid[palm]]
        out = [np.zeros(3)]
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
        hand, b = set(pl.ee_bodies[role]), m.site_bodyid[self.rt.palm_sid[palm]]
        for _ in range(2):                        # and the wrist links it hangs from (the G1's rubber hand)
            b = m.body_parentid[b]
            if b in pl.robot_bodies:
                hand.add(b)
        for g in range(m.ngeom):
            if m.geom_bodyid[g] not in hand or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                continue
            c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
            out += list(d.geom_xpos[g] + (c + h * corners) @ d.geom_xmat[g].reshape(3, 3).T - tcp)
        return np.array(out)

    def grip_reach(self, palm, footprint=False):
        """The gripper the way the pick planner sees it, whatever the hand's orientation now: its reach below the TCP
        (open fingertips from above, the pads of a side gripper), straight down or, with `footprint`, over the jaws'
        footprint (hand_disc). P2 trajectories (their own wrist orientation) and the red waypoints of a drawn path."""
        g = self.plant.grip.get(self.role(palm)) if self.plant is not None else None
        if g is None:
            return hand_disc(0.0, 0.0)
        prof = getattr(self.rt, "profile", None)
        side = prof is not None and (prof.physics or {}).get("grasp", prof.grasp) == "side"
        foot = float(np.clip(max(g.get("span", 0.0), g.get("depth", 0.0)), 0.02, 0.06)) if footprint else 0.0
        return hand_disc(self._hang(palm, side), foot, n=8 if footprint else 1)

    def path_hits(self, pts, palm, skip=(), carrying=None, hand=None, margin=0.01):
        """Per point: why the hand (or what it carries) would hit an object there, or None (path_object_hits with this
        hand as it is held, or `hand` points, and the objects where they stand; `skip`: ones the path is meant to reach)."""
        held = carrying[0] if carrying else None
        bodies = [(c, r, h, self.pretty(n)) for c, r, h, n in self._bodies(skip=tuple(skip) + ((held,) if held else ()))]
        carry = (self.pretty(held), self._radius(held), self.objs[held]["half_h"], float(carrying[1])) if held else None
        hand = self.hand_points(palm) if hand is None else hand
        return path_object_hits(pts, bodies, hand, f"my {self.limb(self.role(palm))}", carry, margin=margin)

    def lift_over(self, pts, palm, skip=(), carrying=None):
        """The path raised over the movable objects it would run into (lift_path); fixed obstacles (the vase) are not
        flown over. -> (points, names lifted over)."""
        hand = self.hand_points(palm)
        if carrying:                              # what it carries hangs below the hand
            held = carrying[0]
            r, hh = self._radius(held), self.objs[held]["half_h"]
            hand = np.vstack([hand, hand_disc(float(carrying[1]) + hh, r)])
            skip = tuple(skip) + (held,)
        return lift_path(pts, self._bodies(skip=tuple(self.obstacles) + tuple(skip)), hand)

    def carrying(self, palm):
        """(object in this hand, how far its centre hangs below the TCP) or None."""
        held = self.holding.get(palm)
        if held not in self.objs:
            return None
        return held, max(0.0, float(self.rt.data.site_xpos[self.rt.palm_sid[palm]][2] - self.mug_center(held)[2]))

    def _names(self, names):
        names = [self.pretty(n) for n in names]
        return names[0] if len(names) == 1 else ", ".join(names[:-1]) + " and " + names[-1]

    def draw(self, uvs, cam, aspect, hand=None):
        hits = [self.raycast(cam, u, v, aspect) for u, v in uvs]
        hits = [h for h in hits if h is not None]
        if len(hits) < 2:
            self.rt.path_msg = "The stroke missed the table. Draw over the table or objects."
            return
        self.set_path_world([h + np.array([0, 0, HOVER]) for h in hits], hand)

    def set_path_world(self, pts, hand=None):
        """World-space waypoints (from a stroke or the 3D editor) -> densified path, pre-checked per waypoint."""
        pts = [np.asarray(p, float) for p in pts]
        if len(pts) < 2:
            self.rt.path_msg = "A path needs at least two waypoints."
            return
        palm = {"left": "left_palm", "right": "right_palm"}.get(hand)
        if palm not in self.hands:
            palm = self.hand_for([0.0, float(np.mean([p[1] for p in pts]))])
        pts = densify(pts, 0.04)
        side = "right" if palm.startswith("right") else "left"
        carry, hits, over = self.carrying(palm), None, []
        if self.plant is not None:                # physics: the hand would run into something: over the movable ones, if
            if any(self.path_hits(pts, palm, carrying=carry)):     # it can reach that high (the vase is not flown over,
                lp, over = self.lift_over(pts, palm, carrying=carry)   # as with a carry); the hand as held: lifts are
                if over and all(self.rt.precheck(lp, palm)):        # generous (higher is harmless), red waypoints need
                    pts = lp                                        # a real overlap (a false refusal is not harmless:
                else:                                               # the corners of mesh bounding boxes overstate a hand)
                    over = []
            hits = self.path_hits(pts, palm, carrying=carry, margin=0.0)
        ok = self.rt.precheck(pts, palm)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.rt.path = {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side}
        self.rt.path["src"] = "draw"     # P2 hook: pps/features/traj.py adopts drawn / 3D-editor paths as trajectories
        bad = sum(1 for o in ok if not o)
        lim = self.limb(side).capitalize()
        lifted = f", lifted over {self._names(over)}" if over else ""
        self.rt.path_msg = (f"{lim} path: {len(pts)} waypoints, all reachable{lifted}. Say 'go' or press F."
                            if bad == 0 else f"{lim} path: {bad}/{len(pts)} waypoints out of reach (red){lifted}.")
        if hits and any(hits):                    # what it would still run into: red, with why
            self.rt.path["ok"] = [o and not h for o, h in zip(self.rt.path["ok"], hits)]
            why = next(h for h in hits if h)
            self.rt.path["why"] = f"Part of that path runs into something: {why} (red). Draw it around, or over it in 3D."
            self.rt.path["hits"] = hits
            self.rt.path_msg = f"{lim} path: {sum(1 for h in hits if h)}/{len(pts)} waypoints where {why} (red)."
        if over:
            self.rt.path["lifted"] = list(over)
        zs = self.keepout.along(pts, {}, palm) if self.keepout else None
        if zs and any(zs):   # waypoints inside a keep-out zone are not ok either
            self.rt.path["ok"] = [o and not z for o, z in zip(ok, zs)]
            self.rt.path["why"] = f"Part of that path goes through {next(z for z in zs if z)[0]} (red). Redraw it around."
            self.rt.path_msg = f"{self.limb(side).capitalize()} path: {sum(1 for z in zs if z)}/{len(pts)} waypoints inside {next(z for z in zs if z)[0]} (red)."

    def follow_path(self, tid):
        rt = self.rt
        if rt.path is None:
            self.set_task(tid, "follow_path", "failed", "There is no path yet. Draw one on my view.")
            return
        if not all(rt.path["ok"]):
            self.set_task(tid, "follow_path", "failed", rt.path.get("why") or "Part of that path is out of my reach (red). Redraw it closer.")
            return
        if self.plant is not None:                # physics: get to the route's start first (over the table's edge, not into it,
            palm = rt.path["palm"]                # and over the objects on the way), with every object but what the hand
            cur = rt.data.site_xpos[rt.palm_sid[palm]].copy()   # holds an obstacle for the planner
            rt.set_obstacles(tuple(o for o in self.holding.values() if o))
            pts = np.asarray(rt.path["pts"], float)
            if np.linalg.norm(pts[0] - cur) > 0.03:
                lead = [cur] + ([np.array([cur[0], cur[1], TABLE_TOP + 0.10])] if cur[2] < TABLE_TOP + 0.06 and cur[0] < TABLE_X[0] + 0.05 else [])
                head = densify(lead + [pts[0]])
                head = self.lift_over(head, palm, carrying=self.carrying(palm))[0][:-1]
                pts = np.vstack([head, pts])
                rt.path["pts"], rt.path["ok"] = pts, [True] * len(head) + list(rt.path["ok"])
                rt.path["cum"] = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        self.events, self.fired, self.kind = {}, set(), "path"
        self._free_obstacles()                    # physics: the path may close the gripper on any object (P2 grip points)
        rt.orient(bool(rt.path.get("orient")))    # physics: a trajectory's grasps hold the hand's orientation firmly
        rt.engaged, rt.mode, rt.follow_s, rt.path["i"], rt.follow_hold = False, "FOLLOW", 0.0, 0, False
        rt.recorder.start()
        self.done_msg = "Finished the path."
        self.set_task(tid, "follow_path", "running", f"Following your path with my {self.limb(rt.path['hand'])}.")

    def follow_me(self, tid, enabled):
        rt = self.rt
        if enabled:
            rt.orient(False)                      # mirroring moves the hands freely
            rt.path, self.kind = None, None
            rt.retargeter.reset()
            rt.engaged, rt.hold = True, False
            rt.recorder.start()
            rt.mode = "REC"
            self.set_task(tid, "follow_me", "done", "Following you. Move your arms; I'll record it so you can name it.")
        else:
            rt.engaged = False
            rt.mode = "MIRROR"
            n = len(rt.recorder.frames)
            self.set_task(tid, "follow_me", "done", f"Stopped following. I recorded {rt.recorder.duration:.1f} s ({n} frames)."
                          + (" Say 'learn this as' and a name." if n else ""))

    def learn_skill(self, tid, name):
        rt = self.rt
        name = "".join(ch for ch in str(name or "").strip().lower().replace(" ", "_") if ch.isalnum() or ch in "_-")[:32]
        if not name:
            self.set_task(tid, "learn_skill", "failed", "Give the skill a name.")
            return
        if rt.mode == "REC":
            rt.engaged, rt.mode = False, "MIRROR"
        frames = [{"t": f["t"], "targets": f["targets"], "body": f.get("body")} for f in rt.recorder.frames if f.get("targets")]
        if len(frames) < 5:
            self.set_task(tid, "learn_skill", "failed", "I have nothing to learn yet. Say 'follow me', show the motion, then name it.")
            return
        rep = rt.recorder.report()
        with open(SKILL_DIR / f"{name}.json", "w") as f:
            json.dump({"name": name, "robot": "unitree_g1" if getattr(rt, "profile_id", "g1") == "g1" else rt.profile_id, "created": time.time(), "frames": frames, "report": rep}, f)
        self.set_task(tid, "learn_skill", "done",
                      f"Learned '{name}': {frames[-1]['t']:.1f} s, {rep['feasible_fraction'] * 100:.0f}% feasible.")

    def skills(self):
        return sorted(p[:-5] for p in os.listdir(SKILL_DIR) if p.endswith(".json"))

    def play_skill(self, tid, name):
        rt = self.rt
        key = str(name or "").strip().lower().replace(" ", "_")
        path = SKILL_DIR / f"{key}.json"
        if not path.exists():
            self.set_task(tid, "play_skill", "failed", f"I don't know '{name}'. I know: {', '.join(self.skills()) or 'nothing yet'}.")
            return
        data = json.load(open(path))
        rt.recorder.frames = [{"t": f["t"], "targets": f["targets"], "body": f.get("body"), "human": None, "grip": {}, "context": {},
                               "qpos": [], "flags": {}, "residuals": {}} for f in data["frames"]]
        rt.engaged, rt.path, self.kind = False, None, "skill"
        rt.mode, rt.preview_t, rt.preview_idx = "PREVIEW", 0.0, 0
        self.done_msg = f"Finished '{key}'."
        self.set_task(tid, "play_skill", "running", f"Playing '{key}' at x{rt.speed_scale:.1f} speed.")

    def drive(self, tid, dx=0.0, dy=0.0, dyaw=0.0):
        """Mobile bases (drive_base): move the base by (dx, dy) m in the world frame and dyaw degrees."""
        rt = self.rt
        if "drive_base" not in getattr(rt, "capabilities", ()) or not hasattr(rt, "start_drive"):
            self.set_task(tid, "drive", "failed", f"The {getattr(rt, 'label', 'robot')} has no base to drive.")
            return
        try:
            dx, dy, dyaw = float(dx or 0), float(dy or 0), float(dyaw or 0)
        except (TypeError, ValueError):
            self.set_task(tid, "drive", "failed", "Say how far: dx / dy in metres, dyaw in degrees.")
            return
        if self.kind:
            self.set_task(tid, "drive", "failed", "I'm still busy with the last task. Say stop first.")
            return
        ok, msg = rt.start_drive(dx, dy, dyaw)
        if not ok:
            self.set_task(tid, "drive", "failed", msg)
            return
        self.kind, self.done_msg = "drive", msg.replace("Driving", "Drove", 1).replace("Turning", "Turned", 1).replace("turning", "turned", 1)
        self.set_task(tid, "drive", "running", msg)

    def stop(self, tid):
        rt = self.rt
        rt.engaged = False
        rt.mode = "MIRROR"
        if getattr(rt, "drive_goal", None) is not None:
            rt.drive_goal = None
            rt.base_task.set_cost(np.zeros(rt.configuration.model.nv))
        if self.task and self.task.get("status") in ("running", "planning") and self.task.get("id") != tid:
            self.task["status"], self.task["message"] = "failed", "Stopped by you before it finished."
            self.stopped_task = dict(self.task)
        self.kind = None
        self._fx_cleanup()
        if self.plant is not None:                # stop where the arm *is*, not where the plan was heading
            self.ph, self.place_at, rt.follow_hold = None, None, False
            rt.orient(False)
            rt.hold_here()
            self._free_obstacles()
        self.set_task(tid, "stop", "done", "Stopped. Holding this pose.")

    def home(self, tid, reset_scene=False):
        rt = self.rt
        rt.home()
        rt.retargeter.reset()
        rt.engaged, rt.mode, rt.path, self.kind = False, "MIRROR", None, None
        self._fx_cleanup()
        if self.plant is not None:
            self.ph, self.place_at, rt.follow_hold = None, None, False
            rt.orient(False)
            self._free_obstacles()
        if reset_scene:
            self.holding = {h: None for h in self.holding}
            self.grasp_off = None
            if self.plant is not None:            # a reset of the simulation: robot and objects back, at rest
                self.held = {}
                for r in self.plant.grip:
                    self.plant.grip_cmd(r, "open")
                self.plant.reset(rt.configuration.q)
                self.plant.settle(0.3)
            else:
                for n, o in self.objs.items():
                    self.set_obj_pose(n, o["home"], o["home_q"])
            self.fixture_reset()
            self.markers, self.next_marker = [], 0
        else:
            for palm, obj in self.holding.items():
                if obj and self.plant is not None:     # going home with something in hand: let go of it
                    self.plant.grip_cmd(self.role(palm), "open")
                    self.holding[palm] = None
                    self.held.pop(palm, None)
                elif obj:
                    self._event("release", palm)
        self.set_task(tid, "reset_scene" if reset_scene else "home", "done",
                      "Scene reset." if reset_scene else "Back to my start pose.")


# ---------------------------------------------------------------------- skill editing
PART_KEYS = {
    "left_arm": ["left_palm"], "right_arm": ["right_palm"], "arms": ["left_palm", "right_palm"],
    "legs": ["left_foot", "right_foot", "com"], "body": ["left_palm", "right_palm", "left_foot", "right_foot", "com", "yaw"],
}


def edit_skill(tasks, tid, name, t0=None, t1=None, part="body", speed=None, amplitude=None):
    """Edit a time range of one body part: retime (speed) and/or scale motion around the home pose (amplitude)."""
    key = str(name or "").strip().lower().replace(" ", "_")
    path = SKILL_DIR / f"{key}.json"
    if not path.exists():
        tasks.set_task(tid, "edit_skill", "failed", f"I don't know a skill called '{name}'.")
        return
    data = json.load(open(path))
    frames = data["frames"]
    dur = frames[-1]["t"] if frames else 0.0
    t0 = 0.0 if t0 is None else max(0.0, float(t0))
    t1 = dur if t1 is None else min(dur, float(t1))
    if t1 - t0 < 0.1:
        tasks.set_task(tid, "edit_skill", "failed", "Pick a time range longer than 0.1 s.")
        return
    part = part if part in PART_KEYS else "body"
    keys = PART_KEYS[part]
    rt = tasks.rt
    homes = {"left_palm": tasks.home_palm["left_palm"], "right_palm": tasks.home_palm["right_palm"],
             "left_foot": rt.body_home["feet"]["left_foot"], "right_foot": rt.body_home["feet"]["right_foot"],
             "com": rt.body_home["com"]}

    def ramp(t):  # smooth 0..1..0 weight across the range, 0.2 s edges
        e = min(0.2, (t1 - t0) / 3)
        if t < t0 or t > t1:
            return 0.0
        return float(min(1.0, (t - t0) / e if e else 1.0, (t1 - t) / e if e else 1.0))

    changes = []
    if amplitude:
        a = float(np.clip(amplitude, 0.3, 2.5))
        for f in frames:
            w = ramp(f["t"])
            if w <= 0:
                continue
            k = 1 + (a - 1) * w
            for palm in ("left_palm", "right_palm"):
                if palm in keys and palm in f["targets"]:
                    shoulder = rt.retargeter.robot_shoulders[palm]
                    f["targets"][palm] = (shoulder + (np.asarray(f["targets"][palm]) - shoulder) * k).tolist()
            b = f.get("body")
            if b:
                for kk in ("left_foot", "right_foot", "com"):
                    if kk in keys:
                        b[kk] = (homes[kk] + (np.asarray(b[kk]) - homes[kk]) * k).tolist()
                if "yaw" in keys:
                    b["yaw"] = float(np.clip(b["yaw"] * k, -0.9, 0.9))
        changes.append(f"{'bigger' if a > 1 else 'smaller'} x{a:.1f}")
    if speed:
        sp = float(np.clip(speed, 0.25, 4.0))
        new_t, acc, prev_t = [], 0.0, None
        for f in frames:
            if prev_t is None:
                new_t.append(f["t"]); prev_t = f["t"]; continue
            dt = f["t"] - prev_t
            mid = (f["t"] + prev_t) / 2
            acc += dt / sp if t0 <= mid <= t1 else dt
            new_t.append(frames[0]["t"] + acc)
            prev_t = f["t"]
        for f, t in zip(frames, new_t):
            f["t"] = round(t, 4)
        changes.append(f"{'faster' if sp > 1 else 'slower'} x{sp:.1f}")
    if not changes:
        tasks.set_task(tid, "edit_skill", "failed", "Say faster, slower, bigger or smaller.")
        return
    data["frames"] = frames
    data.setdefault("edits", []).append({"t0": t0, "t1": t1, "part": part, "speed": speed, "amplitude": amplitude})
    json.dump(data, open(path, "w"))
    part_txt = part.replace("_", " ")
    tasks.set_task(tid, "edit_skill", "done",
                   f"Edited '{key}': {part_txt} from {t0:.1f} to {t1:.1f} s, {' and '.join(changes)}. New length {frames[-1]['t']:.1f} s.")


def skills_meta(tasks):
    out = []
    for name in tasks.skills():
        try:
            fr = json.load(open(SKILL_DIR / f"{name}.json"))["frames"]
            out.append({"name": name, "duration": round(fr[-1]["t"], 2) if fr else 0.0})
        except Exception:
            continue
    return out
