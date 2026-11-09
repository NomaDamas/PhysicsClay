"""Intent -> grounded robot task.  Every task is compiled to a world-space hand path, pre-checked with IK
(the physics verdict), then executed; the robot reports in one sentence what it will do or why it can't."""
import json
import os
import sys
import time

import mujoco
import numpy as np

from .robot_runtime import HANDS, ROOT

TABLE_TOP = 0.74
TABLE_X = (0.22, 0.82)
TABLE_Y = (-0.50, 0.50)
REGIONS = {  # robot frame: x forward, y left
    "right": (0.31, -0.22), "left": (0.31, 0.22), "center": (0.34, 0.0),
    "front": (0.27, 0.0), "back": (0.66, 0.0),
}
HOVER = 0.08
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
        self.objs = {}
        self.legacy = False
        for name, geom, kind in LEGACY_OBJS:          # the UX 2.0 scene (scene_pc.xml)
            try:
                mid, g = m.body(name).mocapid[0], m.geom(geom).id
            except KeyError:
                continue
            self.legacy = True
            size = m.geom_size[g]
            self.objs[name] = {"mid": mid, "home": d.mocap_pos[mid].copy(), "off": m.geom_pos[g].copy(),
                               "r": float(size[0]), "half_h": float(size[1] if kind == "cyl" else size[2])}
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
            if bname.startswith("obj_") and m.body_mocapid[b] >= 0:
                mid = m.body_mocapid[b]
                self.objs[bname[4:]] = {"mid": mid, "home": d.mocap_pos[mid].copy(), "off": m.geom_pos[g].copy(), "r": r, "half_h": hh}
            else:
                self.obstacles[bname[4:]] = {"pos": d.geom_xpos[g].copy(), "r": r, "h": hh}
        self.cur = "mug" if ("mug" in self.objs or not self.objs) else next(iter(self.objs))
        self.home_palm = {h: d.site_xpos[rt.palm_sid[h]].copy() for h in self.hands}
        self.vase = self.obstacles.get("vase") or (next(iter(self.obstacles.values())) if self.obstacles else None)
        self.keepout = None      # keep-out zones hook (pps/features/zones.py): .at(p, r), .along(pts, events, palm), .route(kps, ev, palm)
        roots = getattr(rt, "robot_roots", None)
        self.robot_roots = set(roots) if roots else {m.body(ROBOT_BODIES_ROOT).id}
        self.robot_root = min(self.robot_roots)
        self.markers = []        # [{"id": "A", "pos": np.array}]
        self.next_marker = 0
        self.holding = {h: None for h in reversed(self.hands)}
        self.grasp_off = None
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

    def mug_center(self, name=None):
        o = self.objs[name or self.cur]
        return self.rt.data.mocap_pos[o["mid"]] + o["off"]

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

    def _launch(self, tid, action, keypoints, palm, events, kind, ok_msg):
        """Densify, pre-check, then start following. events: {keypoint_index: name}."""
        rt = self.rt
        idx_map, pts = [], []
        for k, kp in enumerate(keypoints):
            if k == 0:
                pts.append(np.asarray(kp, float)); idx_map.append(0); continue
            seg = densify([pts[-1], kp])[1:]
            pts.extend(seg)
            idx_map.append(len(pts) - 1)
        pts = np.array(pts)
        self.set_task(tid, action, "planning", "Checking reach for every waypoint.")
        ok = rt.precheck(pts, palm)
        dists = rt.last_precheck_dists
        side = "right" if palm.startswith("right") else "left"
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
            ok = [o and not (lo <= j <= hi) for j, o in enumerate(ok) for lo, hi in [hit[1]]]
            rt.path = {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side}
            self.kind = None
            self.fail_reason = "vase"
            over = "the vase" if self.legacy else self.pretty(self.hit_obstacle)
            self.set_task(tid, action, "failed",
                          f"I can't carry it that way: {self.pretty(self.cur)} would hit {self.pretty(self.hit_obstacle)} ({hit[0]:.0f} cm too low). "
                          f"Draw a route over {over} in 3D, then say 'carry it along my path'.")
            return False
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
        rt.path_msg = None
        self.events = {idx_map[k]: name for k, name in events.items()}
        self.fired = set()
        self.kind = kind
        rt.engaged = False
        rt.mode, rt.follow_s = "FOLLOW", 0.0
        rt.recorder.start()
        self.set_task(tid, action, "running", ok_msg)
        return True

    def carry_hits_vase(self, pts, events, palm):
        """While the mug is in hand (grasp..release), does the carried mug sweep through the vase? -> (cm short, (i0,i1)) | None"""
        if self.vase is None:
            return None
        held_now = self.holding.get(palm) == self.cur
        g = next((i for i, n in sorted(events.items()) if n == "grasp"), None)
        r = next((i for i, n in sorted(events.items()) if n == "release"), len(pts) - 1)
        start = 0 if held_now else g
        if start is None:
            return None
        off = self.mug_center() - self.rt.data.site_xpos[self.rt.palm_sid[palm]] if held_now else -getattr(self, "cur_off", np.array([-(self.mug_r + 0.015), 0.0, 0.01]))
        worst, span, who = 0.0, None, None
        for oname, ob in self.obstacles.items():
            vp, vr, vh = ob["pos"], ob["r"], ob["h"]
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

    def tick(self):
        """Call after every rt.step: fire waypoint events, carry the mug, finish tasks."""
        rt, d = self.rt, self.rt.data
        if rt.mode == "FOLLOW" and rt.path is not None and self.kind:
            i = rt.path["i"]
            for idx, name in sorted(self.events.items()):
                if idx <= i and idx not in self.fired:
                    self.fired.add(idx)
                    self._event(name, rt.path["palm"])
        for palm, obj in self.holding.items():
            if obj in self.objs and self.grasp_off is not None:
                d.mocap_pos[self.objs[obj]["mid"]] = d.site_xpos[rt.palm_sid[palm]] + self.grasp_off
        active = (self.kind in ("task", "path") and rt.mode == "FOLLOW") or (self.kind == "skill" and rt.mode == "PREVIEW") \
            or (self.kind == "drive" and getattr(rt, "driving", False))
        if self.kind and not active:
            if self.task and self.task["status"] == "running":
                self.set_task(self.task["id"], self.task["action"], "done", self.done_msg or "Done.")
            self.kind = None

    def _event(self, name, palm):
        d = self.rt.data
        if name == "grasp":
            self.holding[palm] = self.cur
            self.grasp_off = d.mocap_pos[self.mug_mid] - d.site_xpos[self.rt.palm_sid[palm]]
            if self.task:
                self.task["step"] = "grasped"
        elif name == "release":
            if self.holding.get(palm) in self.objs:
                self.cur = self.holding[palm]
            self.holding[palm] = None
            self.grasp_off = None
            p = d.mocap_pos[self.mug_mid].copy()
            c = p + self.mug_center_off
            p[2] = (TABLE_TOP if self.on_table(c) else 0.0) + self.mug_half_h - self.mug_center_off[2]
            d.mocap_pos[self.mug_mid] = p
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
        cur = d.site_xpos[self.rt.palm_sid[palm]].copy()
        off = np.array([0.0, 0.0, self.mug_half_h + 0.035]) if grasp == "top" else np.array([-(self.mug_r + 0.015), 0.0, 0.01])
        self.cur_off = off
        place_c = np.array([dest[0], dest[1], TABLE_TOP + self.mug_half_h])
        place_hand = place_c + off
        kps, ev = [cur], {}
        if held is None:
            grasp = c + off
            kps += [grasp + (np.array([0, 0, 0.09]) if np.array_equal(off[:2], [0, 0]) else np.array([-0.08, 0, 0.07])), grasp]
            ev[len(kps) - 1] = "grasp"
            kps += [grasp + np.array([0, 0, 0.12])]
        if via is not None:
            kps += [np.array(v) for v in via]
            if via_pts is None:
                label = "the end of your route"
        kps += [place_hand + np.array([0, 0, 0.12]), place_hand]
        ev[len(kps) - 1] = "release"
        kps += [place_hand + np.array([-0.10, 0, 0.07]), self.home_palm[palm]]
        if via is None and self.keepout:   # keep-out zones: detour around or over them (adds e.g. ", going around zone A")
            kps, ev, note = self.keepout.route(kps, ev, palm)
            label += note
        side = "right" if palm.startswith("right") else "left"
        what = self.pretty(name)
        self.done_msg = f"Placed {what} on {label}."
        ok = self._launch(tid, "pick_place", kps, palm, ev, "task",
                          f"Every waypoint is reachable. Picking up {what} with my {self.limb(side)} and placing it on {label}.")
        if not ok and getattr(self, "fail_reason", "reach") == "reach" and via is None:
            self.suggest(tid, dest, palm, off)
        self.fail_reason = "reach"

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
        rt.engaged, rt.mode, rt.follow_s, rt.path["i"] = False, "FOLLOW", 0.0, 0
        rt.recorder.start()
        self.done_msg = "Finished the path."
        self.set_task(tid, "follow_path", "running", f"Following your path with my {self.limb(rt.path['hand'])}.")

    def follow_me(self, tid, enabled):
        rt = self.rt
        if enabled:
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
            rt.base_task.set_cost(np.zeros(rt.model.nv))
        if self.task and self.task.get("status") in ("running", "planning") and self.task.get("id") != tid:
            self.task["status"], self.task["message"] = "failed", "Stopped by you before it finished."
            self.stopped_task = dict(self.task)
        self.kind = None
        self.set_task(tid, "stop", "done", "Stopped. Holding this pose.")

    def home(self, tid, reset_scene=False):
        rt = self.rt
        rt.home()
        rt.retargeter.reset()
        rt.engaged, rt.mode, rt.path, self.kind = False, "MIRROR", None, None
        if reset_scene:
            self.holding = {h: None for h in self.holding}
            self.grasp_off = None
            for o in self.objs.values():
                rt.data.mocap_pos[o["mid"]] = o["home"]
            self.markers, self.next_marker = [], 0
        else:
            for palm, obj in self.holding.items():
                if obj:
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
