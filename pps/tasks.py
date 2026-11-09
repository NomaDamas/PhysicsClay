"""Intent -> grounded robot task.  Every task is compiled to a world-space hand path, pre-checked with IK
(the physics verdict), then executed; the robot reports in one sentence what it will do or why it can't."""
import json
import os
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
SKILL_DIR = ROOT / "recordings" / "skills"
ROBOT_BODIES_ROOT = "pelvis"


def densify(pts, step=0.035):
    out = [np.asarray(pts[0], float)]
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


class Tasks:
    def __init__(self, rt):
        self.rt = rt
        m, d = rt.model, rt.data
        self.mug_mid = m.body("mug").mocapid[0]
        self.mug_home = d.mocap_pos[self.mug_mid].copy()
        g = m.geom("mug_collision").id
        self.mug_center_off = m.geom_pos[g].copy()          # body frame offset to the mug's centre
        self.mug_half_h = float(m.geom_size[g][1])
        self.mug_r = float(m.geom_size[g][0])
        self.home_palm = {h: d.site_xpos[rt.palm_sid[h]].copy() for h in HANDS}
        self.robot_root = m.body(ROBOT_BODIES_ROOT).id
        self.markers = []        # [{"id": "A", "pos": np.array}]
        self.next_marker = 0
        self.holding = {"left_palm": None, "right_palm": None}
        self.grasp_off = None
        self.task = None         # {"id","action","status","message","step"}
        self.events = {}         # waypoint index -> event name
        self.fired = set()
        self.kind = None         # "task" | "path"
        self.on_done = None
        os.makedirs(SKILL_DIR, exist_ok=True)

    # ------------------------------------------------------------------ geometry
    def mug_center(self):
        return self.rt.data.mocap_pos[self.mug_mid] + self.mug_center_off

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
            if m.body_rootid[body] == self.robot_root:
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
            for mk in self.markers:
                if t in (mk["id"].lower(), f"marker {mk['id'].lower()}", f"point {mk['id'].lower()}"):
                    return mk["pos"].copy(), f"marker {mk['id']}"
            for name, (x, y) in REGIONS.items():
                if name in t:
                    return np.array([x, y, TABLE_TOP]), f"the {name} of the table"
            if "mug" in t:
                c = self.mug_center()
                return np.array([c[0], c[1], TABLE_TOP]), "the mug"
            return None, f"I don't know where '{target}' is. Tap it on my view or use left, right, center, front or back"
        return None, "no target given"

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
            msg = f"I can't: {sum(1 for o in ok if not o)} of {len(ok)} waypoints are out of my {side} arm's reach, worst by {short:.0f} cm."
            self.set_task(tid, action, "failed", msg)
            return False
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
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
            if obj == "mug" and self.grasp_off is not None:
                d.mocap_pos[self.mug_mid] = d.site_xpos[rt.palm_sid[palm]] + self.grasp_off
        active = (self.kind in ("task", "path") and rt.mode == "FOLLOW") or (self.kind == "skill" and rt.mode == "PREVIEW")
        if self.kind and not active:
            if self.task and self.task["status"] == "running":
                self.set_task(self.task["id"], self.task["action"], "done", self.done_msg or "Done.")
            self.kind = None

    def _event(self, name, palm):
        d = self.rt.data
        if name == "grasp":
            self.holding[palm] = "mug"
            self.grasp_off = d.mocap_pos[self.mug_mid] - d.site_xpos[self.rt.palm_sid[palm]]
            if self.task:
                self.task["step"] = "grasped"
        elif name == "release":
            self.holding[palm] = None
            self.grasp_off = None
            p = d.mocap_pos[self.mug_mid].copy()
            c = p + self.mug_center_off
            p[2] = (TABLE_TOP if self.on_table(c) else 0.0) + self.mug_half_h - self.mug_center_off[2]
            d.mocap_pos[self.mug_mid] = p
            if self.task:
                self.task["step"] = "placed"

    # ------------------------------------------------------------------ actions
    def pick_place(self, tid, obj, target, target_uv, cam, aspect):
        if obj and "mug" not in str(obj).lower() and "cup" not in str(obj).lower():
            self.set_task(tid, "pick_place", "failed", f"I only see a mug on the table, not '{obj}'.")
            return
        dest, label = self.resolve(target, target_uv, cam, aspect)
        if dest is None:
            self.set_task(tid, "pick_place", "failed", label[0].upper() + label[1:] + ".")
            return
        if not self.on_table(dest):
            self.set_task(tid, "pick_place", "failed", f"I can't place it there: {label} is not on the table.")
            return
        held = next((h for h, o in self.holding.items() if o == "mug"), None)
        c = self.mug_center()
        palm = held or ("right_palm" if c[1] <= 0 else "left_palm")
        d = self.rt.data
        cur = d.site_xpos[self.rt.palm_sid[palm]].copy()
        off = np.array([-(self.mug_r + 0.015), 0.0, 0.01])
        place_c = np.array([dest[0], dest[1], TABLE_TOP + self.mug_half_h])
        place_hand = place_c + off
        kps, ev = [cur], {}
        if held is None:
            grasp = c + off
            kps += [grasp + np.array([-0.08, 0, 0.07]), grasp]
            ev[len(kps) - 1] = "grasp"
            kps += [grasp + np.array([0, 0, 0.12])]
        kps += [place_hand + np.array([0, 0, 0.12]), place_hand]
        ev[len(kps) - 1] = "release"
        kps += [place_hand + np.array([-0.10, 0, 0.07]), self.home_palm[palm]]
        side = "right" if palm.startswith("right") else "left"
        self.done_msg = f"Placed the mug on {label}."
        self._launch(tid, "pick_place", kps, palm, ev, "task",
                     f"Every waypoint is reachable. Picking up the mug with my {side} hand and placing it on {label}.")

    def reach(self, tid, target, target_uv, cam, aspect):
        dest, label = self.resolve(target, target_uv, cam, aspect)
        if dest is None:
            self.set_task(tid, "reach", "failed", label[0].upper() + label[1:] + ".")
            return
        palm = "right_palm" if dest[1] <= 0 else "left_palm"
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
        pts = [h + np.array([0, 0, HOVER]) for h in hits]
        palm = {"left": "left_palm", "right": "right_palm"}.get(hand) or ("right_palm" if np.mean([p[1] for p in pts]) <= 0 else "left_palm")
        pts = densify(pts, 0.04)
        ok = self.rt.precheck(pts, palm)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        side = "right" if palm.startswith("right") else "left"
        self.rt.path = {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side}
        bad = sum(1 for o in ok if not o)
        self.rt.path_msg = (f"{side.capitalize()} hand path: {len(pts)} waypoints, all reachable. Say 'go' or press F."
                            if bad == 0 else f"{side.capitalize()} hand path: {bad}/{len(pts)} waypoints out of reach (red).")

    def follow_path(self, tid):
        rt = self.rt
        if rt.path is None:
            self.set_task(tid, "follow_path", "failed", "There is no path yet. Draw one on my view.")
            return
        if not all(rt.path["ok"]):
            self.set_task(tid, "follow_path", "failed", "Part of that path is out of my reach (red). Redraw it closer.")
            return
        self.events, self.fired, self.kind = {}, set(), "path"
        rt.engaged, rt.mode, rt.follow_s, rt.path["i"] = False, "FOLLOW", 0.0, 0
        rt.recorder.start()
        self.done_msg = "Finished the path."
        self.set_task(tid, "follow_path", "running", f"Following your path with my {rt.path['hand']} hand.")

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
        frames = [{"t": f["t"], "targets": f["targets"]} for f in rt.recorder.frames if f.get("targets")]
        if len(frames) < 5:
            self.set_task(tid, "learn_skill", "failed", "I have nothing to learn yet. Say 'follow me', show the motion, then name it.")
            return
        rep = rt.recorder.report()
        with open(SKILL_DIR / f"{name}.json", "w") as f:
            json.dump({"name": name, "robot": "unitree_g1", "created": time.time(), "frames": frames, "report": rep}, f)
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
        rt.recorder.frames = [{"t": f["t"], "targets": f["targets"], "human": None, "grip": {}, "context": {},
                               "qpos": [], "flags": {}, "residuals": {}} for f in data["frames"]]
        rt.engaged, rt.path, self.kind = False, None, "skill"
        rt.mode, rt.preview_t, rt.preview_idx = "PREVIEW", 0.0, 0
        self.done_msg = f"Finished '{key}'."
        self.set_task(tid, "play_skill", "running", f"Playing '{key}' at x{rt.speed_scale:.1f} speed.")

    def stop(self, tid):
        rt = self.rt
        rt.engaged = False
        rt.mode = "MIRROR"
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
            rt.data.mocap_pos[self.mug_mid] = self.mug_home
            self.markers, self.next_marker = [], 0
        else:
            for palm, obj in self.holding.items():
                if obj:
                    self._event("release", palm)
        self.set_task(tid, "reset_scene" if reset_scene else "home", "done",
                      "Scene reset." if reset_scene else "Back to my start pose.")
