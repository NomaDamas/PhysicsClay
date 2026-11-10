"""Intent -> grounded robot task.  Every task is compiled to a world-space hand path, pre-checked with IK
(the physics verdict), then executed; the robot reports in one sentence what it will do or why it can't."""
import json
import os
import sys
import time

import mujoco
import numpy as np

from .robot_runtime import HANDS, ROOT, slerp

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
W_TURN = 1.0             # physics: rad/s, how fast the hand turns along a path (a wrist flipping at full speed jerks things)
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


class Tasks:
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
        self.task = None         # {"id","action","status","message","step"}
        self.events = {}         # waypoint index -> event name
        self.fired = set()
        self.kind = None         # "task" | "path"
        self.on_done = None
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
        r_me = self.objs[moving]["r"] if moving in self.objs else 0.04
        if self.plant is not None and self.plant.grip:   # physics: the hand that sets it down has to fit there too
            r_me = max(r_me, min(g.get("depth", 0.0) for g in self.plant.grip.values()) + 0.01)
        others = [(n, o["pos"], o["r"]) for n, o in self.obstacles.items()] + \
                 [(n, self.mug_center(n), o["r"]) for n, o in self.objs.items() if n != moving]

        def blocker_at(p):
            for n, c, r in others:
                if np.linalg.norm(np.asarray(p)[:2] - np.asarray(c)[:2]) < r + r_me + 0.01:
                    return n
            return self.keepout.at(p, r_me) if self.keepout else None

        who = blocker_at(dest)
        if who is None:
            return dest, None
        best = None
        for rad in (0.08, 0.11, 0.14, 0.17):
            for ang in np.linspace(0, 2 * np.pi, 16, endpoint=False):
                cand = np.array([dest[0] + rad * np.cos(ang), dest[1] + rad * np.sin(ang), dest[2]])
                if self.on_table(cand) and blocker_at(cand) is None:
                    score = rad + 0.5 * max(0.0, cand[0] - 0.33) + 0.5 * max(0.0, abs(cand[1]) - 0.24)   # near, reachable
                    if best is None or score < best[0]:
                        best = (score, cand)
            if best is not None:
                break
        return (None if best is None else best[1]), who

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
            rt.configuration.update(self.plant.d.qpos[:self.plant.nq_r].copy())
            events = dict(events)                 # ... and stop above a grasp until it has lined up, then go in slowly
            for k, nm in list(events.items()):
                if nm == "grasp" and k > 0 and (k - 1) not in events:
                    events[k - 1] = "hover"
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
        crit = [idx_map[k] for k, nm in events.items() if nm in ("grasp", "release")] if self.plant is not None else []
        short = [j for j in crit if dists[j] > CRIT_TOL]
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
            rt.path["slow"] = [(idx_map[k - 1], idx_map[k]) for k, nm in events.items() if nm in ("grasp", "release", "press") and k > 0]
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
        everything it passes over on the straight way from hand point a to b (with `margin`); -inf when nothing is near."""
        r, hh = self._radius(name), self.objs[name]["half_h"]
        ca, cb = (np.asarray(a, float) - off)[:2], (np.asarray(b, float) - off)[:2]
        ab = cb - ca
        need = -np.inf
        for c, rb, hb, _ in self._bodies(skip=(name,)):
            t = float(np.clip(np.dot(c[:2] - ca, ab) / (float(np.dot(ab, ab)) or 1.0), 0.0, 1.0))
            if np.linalg.norm(c[:2] - (ca + t * ab)) < rb + r + 0.02:
                need = max(need, float(c[2]) + hb + margin + hh + float(off[2]))
        return need

    def tick(self):
        """Call after every rt.step: fire waypoint events, carry the mug, finish tasks."""
        rt, d = self.rt, self.rt.data
        if rt.mode == "FOLLOW" and rt.path is not None and self.kind:
            i = rt.path["i"]
            for idx, name in sorted(self.events.items()):
                if idx <= i and idx not in self.fired:
                    self.fired.add(idx)
                    self._event(name, rt.path["palm"])
        if self.plant is not None:
            self._phys_tick()
        else:
            for palm, obj in self.holding.items():
                if obj in self.objs and self.grasp_off is not None:
                    self.set_obj_pose(obj, d.site_xpos[rt.palm_sid[palm]] + self.grasp_off)
        active = (self.kind in ("task", "path") and (rt.mode == "FOLLOW" or (self.ph or {}).get("phase") == "press")) \
            or (self.kind == "skill" and rt.mode == "PREVIEW") or (self.kind == "drive" and getattr(rt, "driving", False))
        if self.kind and not active:
            rt.orient(False)                      # the next reach / mirroring moves the hand freely again
            if self.task and self.task["status"] == "running":
                bad = self._placed_badly() if self.task.get("action") == "pick_place" else None
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
        self._hold_path(False)
        rt.mode = "MIRROR"
        rt.hold_here()
        rt.orient(False)
        self.kind = None
        if obj in self.objs:
            self.cur = obj
        if self.task:
            self.set_task(self.task["id"], self.task["action"], "failed", msg)

    def _phys_tick(self):
        rt, pl = self.rt, self.plant
        now = float(pl.d.time)
        ph = self.ph
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
                    self.ph = None
                    self._hold_path(False)
            elif ph["phase"] == "arrive":          # at the drop point? (the plant lags the plan; it may be stuck)
                off = float(np.linalg.norm(tcp - rt.data.mocap_pos[rt.hands_mid[palm]]))
                v = 0.0 if ph.get("p") is None else float(np.linalg.norm(tcp - ph["p"])) / max(now - ph["tp"], 1e-4)
                ph["p"], ph["tp"] = tcp.copy(), now
                ph["still"] = (ph.get("still") or now) if (off < 0.005 and v < 0.02) else None
                sup = obj in pl.objects and pl.support(obj, role) > 0.5 * pl.weight(obj)
                if sup or (ph["still"] is not None and now - ph["still"] > 0.1) or (now - ph["t0"] > 2.5 and off < 0.06):
                    pl.grip_cmd(role, "open")      # resting on the surface, or the hand has stopped just above it
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
                if (now - ph["t0"] > 0.35 and not any(pl.touching(role, obj))) or now - ph["t0"] > 1.2:
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
        if f > 0.9 * lim:                         # near the limit: back off at once
            v = -0.03
        else:                                     # admittance: a guarded move toward the surface until the force is right
            v = float(np.clip(0.002 * (want - f), -0.03, 0.01))
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
        self.press_spec = {"force": force, "hold": hold_s, "label": label}
        self.done_msg = f"Pressed {label}."
        rt.set_obstacles(())
        self._launch(tid, "press", kps, palm, ev, "task", f"Pressing {label} with {force:.0f} N for {hold_s:.1f} s.",
                     loose=[(0, len(kps) - 3)])

    def _placed_badly(self):
        """Physics: after a pick & place, is the object resting where it should, upright? -> reason it isn't, or None."""
        if self.plant is None or self.place_at is None:
            return None
        name, dest = self.place_at
        self.place_at = None
        if name not in self.objs:
            return None
        c = self.mug_center(name)
        err = float(np.hypot(*(c[:2] - dest[:2])))
        tilt = float(np.degrees(np.arccos(np.clip(self.obj_mat(name)[2, 2], -1, 1))))
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
        if self.plant is not None and name == "press":     # compliance: the force loop takes over the hand target
            now = float(self.plant.d.time)
            self.ph = {"phase": "press", "palm": palm, "t0": now, "t_last": now, "fmax": 0.0, "fs": [], "on": None,
                       **getattr(self, "press_spec", {"force": 10.0, "hold": 1.5, "label": "there"})}
            self.rt.mode = "PRESS"
            self._hold_path(True)
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
        if name == "grasp":
            self.holding[palm] = self.cur
            self.grasp_off = self.obj_pos(self.cur) - d.site_xpos[self.rt.palm_sid[palm]]
            if self.task:
                self.task["step"] = "grasped"
        elif name == "release":
            if self.holding.get(palm) in self.objs:
                self.cur = self.holding[palm]
            self.holding[palm] = None
            self.grasp_off = None
            p = self.obj_pos(self.cur)
            c = p + self.mug_center_off
            p[2] = (TABLE_TOP if self.on_table(c) else 0.0) + self.mug_half_h - self.mug_center_off[2]
            self.set_obj_pose(self.cur, p)
            if self.task:
                self.task["step"] = "placed"

    # ------------------------------------------------------------------ actions
    def pick_place(self, tid, obj, target, target_uv, cam, aspect, via_pts=None, grasp=None):
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
        via = [np.asarray(v, float) for v in via_pts] if via_pts else None
        if via is None and target and str(target).strip().lower() in ("path", "my path", "along my path", "the path", "route", "경로", "경로대로", "내 경로"):
            if self.rt.path is None or len(self.rt.path["pts"]) < 2:
                self.set_task(tid, "pick_place", "failed", "There is no route yet. Draw one on my view or in 3D first.")
                return
            via = [np.array(p) for p in self.rt.path["pts"][::2]] + [np.array(self.rt.path["pts"][-1])]
            end = via[-1]
            dest, label = np.array([end[0], end[1], TABLE_TOP]), "the end of your route"
        if not (target and str(target).strip().lower() in ("path", "my path", "along my path", "the path", "route", "경로", "경로대로", "내 경로")):
            dest, label = self.resolve(target, target_uv, cam, aspect)
        if dest is None:
            self.set_task(tid, "pick_place", "failed", label[0].upper() + label[1:] + ".")
            return
        if not self.on_table(dest):
            self.set_task(tid, "pick_place", "failed", f"I can't place it there: {label} is not on the {SURFACE}.")
            return
        if via is None:
            free, blocker = self.free_spot(dest, name)
            if free is None:
                self.set_task(tid, "pick_place", "failed", f"{label[0].upper() + label[1:]} is taken by {self.pretty(blocker)} and there is no free spot nearby.")
                return
            if blocker:
                dest, label = free, f"{label} (beside {self.pretty(blocker)}, which is there)"
        held = next((h for h, o in self.holding.items() if o == name), None)
        c = self.mug_center()
        palm = held or self.hand_for(c)
        d = self.rt.data
        mid = self.rt.hands_mid[palm]
        q_now = d.mocap_quat[mid].copy()
        if self.plant is None:
            self._pick_plan(tid, name, palm, held, c, dest, label, via, via_pts, grasp, None, q_now)
            return
        grasp = (self.rt.profile.physics or {}).get("grasp", grasp) if getattr(self.rt, "profile", None) else grasp
        self.rt.set_obstacles((name,))            # the planner keeps the gripper clear of everything else on the table
        palms = [palm] if held else [palm] + [h for h in self.hands if h != palm]   # two hands: the other one may get there
        stop, first = False, None
        for k, p in enumerate(palms):
            mid = self.rt.hands_mid[p]
            q_now = d.mocap_quat[mid].copy()
            for variant in range(len(SIDE_WAYS) + 2):  # physics: a few ways in (approach direction / jaw yaw); first feasible wins
                d.mocap_quat[mid] = q_now
                off, n, why = self.grasp_plan(name, p, grasp, held, variant)
                if off is None:
                    if why is not None and k == 0 and not held:
                        self.set_task(tid, "pick_place", "failed", why)
                        return
                    break
                if self._pick_plan(tid, name, p, held, c, dest, label, via, via_pts, grasp, (off, n), q_now, last=first is None):
                    return
                if first is None:                 # all failing: report the first (main) way and its suggestion
                    first = (dict(self.task), self.rt.path)
                if getattr(self, "fail_reason", "reach") != "reach" or held:
                    stop = True
                    break
            d.mocap_quat[mid] = q_now
            if stop:
                break
        if first is not None and not stop:
            self.task, self.rt.path = first
        self.fail_reason = "reach"

    def _pick_plan(self, tid, name, palm, held, c, dest, label, via, via_pts, grasp, plan, q_now, last=True, yaw=None):
        """Keypoints of a pick & place (approach, grasp, carry, release, retreat) -> _launch. plan: (offset, approach) with
        physics, None for the kinematic grasp. Physics, from above: the hand may turn about the vertical to set it down
        (yaw; tried in the order that leaves the hand most room). -> launched?"""
        d = self.rt.data
        mid = self.rt.hands_mid[palm]
        q_grip = d.mocap_quat[mid].copy()          # grasp_plan sets the orientation the grasp needs
        if yaw is None and plan is not None and plan[1][2] <= -0.7 and via is None:
            place_c = np.array([dest[0], dest[1], TABLE_TOP + self.mug_half_h])
            first = None
            for k, y in enumerate(self._place_yaw(name, place_c, plan[0], q_grip, palm)):
                d.mocap_quat[mid] = q_grip
                if self._pick_plan(tid, name, palm, held, c, dest, label, via, via_pts, grasp, plan, q_now, last=last and k == 0, yaw=y):
                    return True
                first = first or (dict(self.task), self.rt.path)
                if getattr(self, "fail_reason", "reach") != "reach" or "exactly to the spot" not in self.task.get("message", ""):
                    break                         # only a hand that doesn't fit at the spot is helped by turning it
            self.task, self.rt.path = first
            return False
        cur = d.site_xpos[self.rt.palm_sid[palm]].copy()
        if plan is None:
            off, n = (np.array([0.0, 0.0, self.mug_half_h + 0.035]) if grasp == "top" else
                      np.array([-(self.mug_r + 0.015), 0.0, 0.01])), None
        else:
            off, n = plan
        place_c = np.array([dest[0], dest[1], TABLE_TOP + self.mug_half_h])
        sidegrasp = n is not None and n[2] > -0.7
        nh = n if sidegrasp else None             # side grasps come in (and back out) along the approach axis
        q_place, off_g = q_grip, off
        if yaw:                                   # physics, from above: turn the jaws so the hand fits where it goes
            qz = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
            q_place = np.zeros(4)
            mujoco.mju_mulQuat(q_place, qz, q_grip)
            off = np.array([np.cos(yaw) * off[0] - np.sin(yaw) * off[1], np.sin(yaw) * off[0] + np.cos(yaw) * off[1], off[2]])
        self.cur_off = off
        place_hand = place_c + off + (np.array([0, 0, PLACE_GAP]) if n is not None else 0.0)   # let go just above the surface
        kps, ev = [cur], {}
        home = self.home_palm[palm]
        over = n is not None and home[2] < TABLE_TOP + 0.06 and home[0] < TABLE_X[0] + 0.05   # hands start below the edge
        if over and cur[2] < TABLE_TOP + 0.06:    # physics: up over the table's edge first (a straight line would cut it)
            kps.append(np.array([cur[0], cur[1], TABLE_TOP + 0.10]))
        if held is None:
            g = c + off_g
            if n is None:
                pre = g + (np.array([0, 0, 0.09]) if np.array_equal(off[:2], [0, 0]) else np.array([-0.08, 0, 0.07]))
            else:                                 # physics: come in along the approach axis with the jaws open
                pre = g - nh * 0.10 + np.array([0, 0, 0.03]) if sidegrasp else g + np.array([0, 0, 0.09])
            kps += [pre, g]
            ev[len(kps) - 1] = "grasp"
            kps += [g + np.array([0, 0, 0.12])]
        if via is not None:
            kps += [np.array(v) for v in via]
            if via_pts is None:
                label = "the end of your route"
        above = place_hand + np.array([0, 0, 0.12])
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
            retreat = place_hand - nh * 0.10 + np.array([0, 0, 0.03]) if sidegrasp else place_hand + np.array([0, 0, 0.10])
            self.place_at = (name, place_c)
            if held is None:
                self.plant.grip_cmd(self.role(palm), "open")
        kps += [retreat] + ([np.array([home[0], home[1], TABLE_TOP + 0.10])] if over else []) + [home]
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
        self.done_msg = f"Placed {what} on {label}."
        loose = None
        if n is not None:                         # physics: getting there and going home are transit, not the work
            gk = [k for k, e in ev.items() if e == "grasp"]
            rk = [k for k, e in ev.items() if e == "release"]
            loose = ([(0, gk[0] - 1)] if gk and gk[0] > 1 else []) + ([(rk[-1], len(kps) - 1)] if rk else [])
        ok = self._launch(tid, "pick_place", kps, palm, ev, "task",
                          f"Every waypoint is reachable. Picking up {what} with my {self.limb(side)} and placing it on {label}.",
                          quats=quats, loose=loose)
        if not ok:
            self.place_at = None
        if not ok and last and getattr(self, "fail_reason", "reach") == "reach" and via is None:
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
            pre = g - n * 0.10 + np.array([0, 0, 0.03]) if n[2] > -0.7 else g + np.array([0, 0, 0.09])
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
        o = self.objs[name]
        top_z, w = c[2] + o["half_h"], g["width"]
        if style == "side" and variant >= len(SIDE_WAYS) and kind in ("block", "box", "can", "cup", "bottle", "fruit") \
                and variant - len(SIDE_WAYS) < 2:  # small things: from above as a last resort (doesn't sweep its neighbours)
            sh = rt.retargeter.robot_shoulders[palm]
            h = np.array([c[0] - sh[0], c[1] - sh[1], 0.0])
            h = h / (np.linalg.norm(h) or 1.0)
            a = np.cross([0.0, 0.0, 1.0], h) if variant == len(SIDE_WAYS) else h
            n = np.array([0.0, 0.0, -1.0])
            if kind in ("block", "box"):          # jaws square to the block's faces
                psi = np.arctan2(Ro[1, 0], Ro[0, 0])
                ang = np.arctan2(a[1], a[0])
                ang = psi + np.round((ang - psi) / (np.pi / 2)) * np.pi / 2
                a = np.array([np.cos(ang), np.sin(ang), 0.0])
            L = np.column_stack([g["axis"], g["approach"], np.cross(g["axis"], g["approach"])])
            W = np.column_stack([a, n, np.cross(a, n)])
            mujoco.mju_mat2Quat(rt.data.mocap_quat[mid], (W @ L.T).flatten())
            z = max(top_z - min(0.025, 0.8 * o["half_h"]), TABLE_TOP + g["tip"] + 0.006)
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
            mujoco.mju_mat2Quat(rt.data.mocap_quat[mid], (W @ L.T).flatten())
            z = max(c[2], TABLE_TOP + g["tip"] * np.sin(pitch) + g.get("pad_h", 0.018) * np.cos(pitch) + 0.006)
        else:
            if variant >= 3:
                return None, None, None
            n = np.array([0.0, 0.0, -1.0])
            a = Rt @ g["axis"]
            dz = [0.0, np.pi / 2, -np.pi / 2][variant]
            if kind in ("block", "box"):          # square up the jaws with the block's faces (smallest wrist turn)
                phi, psi = np.arctan2(a[1], a[0]), np.arctan2(Ro[1, 0], Ro[0, 0])
                k = np.round((phi - psi) / (np.pi / 2))
                dz += psi + k * np.pi / 2 - phi
            elif kind == "mug":                   # jaws across the handle's direction: they hold the round body only
                phi, psi = np.arctan2(a[1], a[0]), np.arctan2(-Ro[1, 1], -Ro[0, 1]) + np.pi / 2
                dz += psi + np.round((phi - psi) / np.pi) * np.pi - phi
            if dz:
                qz = np.array([np.cos(dz / 2), 0, 0, np.sin(dz / 2)])
                q = np.zeros(4)
                mujoco.mju_mulQuat(q, qz, rt.data.mocap_quat[mid].copy())
                rt.data.mocap_quat[mid] = q
                a = np.array([np.cos(dz) * a[0] - np.sin(dz) * a[1], np.sin(dz) * a[0] + np.cos(dz) * a[1], a[2]])
            z = max(top_z - min(0.025, 0.8 * o["half_h"]), TABLE_TOP + g["tip"] + 0.006)
        a = np.array([a[0], a[1], 0.0]) / (np.hypot(a[0], a[1]) or 1.0)
        if kind in ("block", "box"):
            hs = pl.m.geom_size[o["geom"]]
            ext = 2 * (abs(np.dot(a, Ro[:, 0])) * hs[0] + abs(np.dot(a, Ro[:, 1])) * hs[1])
        else:
            ext = 2 * o["r"]
        p = np.array([c[0], c[1], z])
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
            rim = body + side * a * (MUG["R"] - MUG["t"] / 2) * s
            p = np.array([rim[0], rim[1], self.obj_pos(name)[2] + MUG["z1"] * s - 0.02])
            ext = MUG["t"] * s
        if ext > w - 0.006:
            return None, None, (f"{what.capitalize()} is {ext * 100:.0f} cm wide and my gripper opens {w * 100:.0f} cm, "
                                "so I can't get a grip on it.")
        return p - c, n, None

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
        ok = self.rt.precheck(pts, palm)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        side = "right" if palm.startswith("right") else "left"
        self.rt.path = {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side}
        bad = sum(1 for o in ok if not o)
        self.rt.path_msg = (f"{self.limb(side).capitalize()} path: {len(pts)} waypoints, all reachable. Say 'go' or press F."
                            if bad == 0 else f"{self.limb(side).capitalize()} path: {bad}/{len(pts)} waypoints out of reach (red).")
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
        self.events, self.fired, self.kind = {}, set(), "path"
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
        if self.plant is not None:                # stop where the arm *is*, not where the plan was heading
            self.ph, self.place_at, rt.follow_hold = None, None, False
            rt.orient(False)
            rt.hold_here()
        self.set_task(tid, "stop", "done", "Stopped. Holding this pose.")

    def home(self, tid, reset_scene=False):
        rt = self.rt
        rt.home()
        rt.retargeter.reset()
        rt.engaged, rt.mode, rt.path, self.kind = False, "MIRROR", None, None
        if self.plant is not None:
            self.ph, self.place_at, rt.follow_hold = None, None, False
            rt.orient(False)
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
