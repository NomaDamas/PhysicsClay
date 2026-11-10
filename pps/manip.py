"""Working with fixtures and containers (issue #48), mixed into pps.tasks.Tasks.

Task actions (a `{"type": "task", "action": ...}` message, docs/PROTOCOL.md "v9 · Worlds"):
  * `open {fixture, amount (0..1, default 0.8)}` / `close {fixture}`: grasp the handle (a knob) from above, then move it
    along the joint (a drawer's slide, a door's arc) slowly and compliantly (the plant's impedance mode), let go and back
    off. Honest failures: no such part, out of reach (the whole motion is pre-checked), the jaws can't close on the knob,
    they slip off it, or it doesn't move (blocked: the hand falls behind the plan) -> it says how far it got.
  * `pick_from {container, object}`: take the object out of a drawer / sink / tote / shelf / rack and hold it, lifted
    clear (the drawer or door in front of it must be open).
  * `place_in {object, container}`: put the object (held, or picked first) into a container: a free spot inside, let
    go just above what is there, checked afterwards (is it inside?).
In kinematic mode (no physics) a grasped drawer / door follows the hand and what is inside rides along.
"""
import math
import re

import mujoco
import numpy as np

STOP = {"the", "a", "an", "open", "close", "shut", "please", "of", "to", "my", "your", "it", "up", "in", "into", "from", "on",
        "out", "and", "then", "take", "put", "pull", "push", "this", "that", "one", "with", "at", "me", "for", "door's", "all", "way"}
ORDINAL = {"first": "top", "1st": "top", "top": "top", "upper": "top", "second": "second", "2nd": "second", "middle": "second",
           "third": "bottom", "3rd": "bottom", "bottom": "bottom", "lowest": "bottom", "lower": "bottom", "last": "bottom"}
SYN = {"fridge": ("fridge", "refrigerator"), "microwave": ("microwave",), "dishwasher": ("dishwasher",), "cnc": ("cnc", "mill", "machine"),
       "centrifuge": ("centrifuge",), "washer": ("washer", "washing"), "dryer": ("dryer",), "sink": ("sink", "basin"),
       "tote": ("tote", "bin", "box"), "rack": ("rack",), "drawer": ("drawer", "drawers"), "door": ("door", "doors"), "lid": ("lid",),
       "cabinet": ("cabinet", "cupboard"), "shelf": ("shelf", "shelves"), "tray": ("tray",), "basket": ("basket",),
       "carton": ("carton", "box"), "vise": ("vise", "vice"), "conveyor": ("conveyor", "belt", "line")}


def _words(s):
    return [w for w in re.findall(r"[a-z0-9]+", str(s or "").lower().replace("_", " ")) if w not in STOP]


class FixtureTasks:
    """Fixture and container actions for pps.tasks.Tasks (uses its _launch / pick_place / grasp machinery)."""

    def fixture_init(self):
        self.fx = getattr(self.rt, "fixtures", None)
        self.place_z = self.transit_z = self.place_check = self.approach_z = None
        self.place_drop = 0.0
        self.fxop = None
        self.fx_speed = 0.30                      # x the follow speed (0.3 m/s): a drawer / door moves at ~9 cm/s

    # ------------------------------------------------------------------ finding things
    def _tokens(self, item):
        t = set(_words(item.get("name"))) | set(_words(item.get("label"))) | set(_words(item.get("fixture"))) | {item.get("kind", "")}
        for k, syn in SYN.items():
            if k in t or any(s in t for s in syn):
                t |= set(syn) | {k}
        for w, o in ORDINAL.items():
            if o in t:
                t.add(w)
        return t

    def _best(self, text, items, what):
        words = _words(text)
        if not items:
            return None, f"There is no {what} here."
        if not words:
            return (items[0], None) if len(items) == 1 else (None, f"Which {what}: " + ", ".join(i["label"] for i in items[:6]) + "?")
        kinds = {k for k in ("drawer", "door", "lid", "rack") if any(w in SYN.get(k, (k,)) for w in words)}
        scored = []
        for it in items:
            if kinds and it.get("kind") not in kinds and not (it.get("kind") == "rack" and "rack" in kinds):
                if not (kinds & {"door"} and it.get("kind") == "lid"):
                    continue
            toks = self._tokens(it)
            name = str(it.get("name", "")).lower()
            score = 10.0 if words == re.findall(r"[a-z0-9]+", name.replace("_", " ")) else 0.0   # its exact name ('tote_out')
            for w in words:
                if w in toks:
                    score += 3.0 if w in ORDINAL or w in kinds else 2.0
                elif w.isdigit() and w in name.split("_"):
                    score += 4.0
            if score > 0:
                scored.append((score, it))
        if not scored:
            return None, f"I don't see '{text}'. There is " + ", ".join(i["label"] for i in items[:8]) + "."
        scored.sort(key=lambda s: -s[0])
        top = [it for s, it in scored if s >= scored[0][0] - 1e-6]
        if len(top) > 1:
            return None, "Which one: " + ", ".join(it["label"] for it in top[:6]) + "?"
        return top[0], None

    def find_part(self, text):
        """A drawer / door / lid by what people call it ('the top drawer', 'fridge door', 'drawer_2')."""
        if self.fx is None or not self.fx.by_name:
            return None, "There is nothing here that opens."
        parts = [p for p in self.fx.by_name.values() if p.get("handle")]
        return self._best(text, parts, "drawer or door")

    def find_container(self, text, for_object=None):
        """A container by name ('the sink', 'tote', 'the rack'); a group (a rack's slots, a tray's compartments, a shelf's
        levels) picks the first free one near the robot."""
        if self.fx is None or not self.fx.containers:
            return None, "There are no containers here."
        cs = self.fx.containers
        exact = next((c for c in cs if c["name"] == str(text or "").strip().lower().replace(" ", "_")), None)
        if exact is not None:
            return exact, None
        words = set(_words(text))
        groups = {}
        for c in cs:
            if c.get("kind") in ("slot", "tray"):
                groups.setdefault(c["fixture"], []).append(c)
        # a rack / tray named as a whole ("the rack", "the tube rack", "the parts tray"): its free member nearest the robot
        generic = {"tube", "tubes", "test", "slot", "slots", "free", "empty", "compartment", "parts", "next", "a"}
        for fx_name, members in sorted(groups.items(), key=lambda kv: set(_words(kv[0])) != words):   # exact names first
            fw = set(_words(fx_name)) | set(SYN.get(fx_name.split("_")[0], ()))
            if words and (fw & words) and words <= fw | generic:
                free = [m for m in members if not self.contents(m, skip=for_object)]
                pool = sorted(free or members, key=lambda m: float(np.linalg.norm(self.fx.box(m)[0][:2])))
                return pool[0], None
        return self._best(text, cs, "container")

    def contents(self, c, skip=None):
        return [n for n in self.objs if n != skip and self.fx.inside(c, self.mug_center(n), margin=0.01)]

    def container_of(self, name):
        if self.fx is None or name not in self.objs:
            return None
        return self.fx.container_of(self.mug_center(name))

    def guard_of(self, c):
        """The drawer / door that has to be open to get at a container (None: it's open to the sky)."""
        if self.fx is None:
            return None
        if c.get("body"):                         # inside a drawer / rack: the part itself
            p = next((q for q in self.fx.by_name.values() if q["body"] == c["body"]), None)
            return p if p is not None and p.get("handle") else None
        doors = [q for q in self.fx.by_name.values() if q["fixture"] == c["fixture"] and q["kind"] in ("door", "lid")]
        return doors[0] if doors else None

    # ------------------------------------------------------------------ geometry
    def vertical_half(self, name):
        o, m = self.objs[name], self.rt.model
        R = self.obj_mat(name)
        s = m.geom_size[o["geom"]]
        if m.geom_type[o["geom"]] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            c = abs(float(R[2, 2]))
            return c * float(s[1]) + math.sqrt(max(0.0, 1 - c * c)) * float(s[0])
        return float(abs(R[2, 0]) * s[0] + abs(R[2, 1]) * s[1] + abs(R[2, 2]) * s[2])

    def support_z(self, name):
        """The height the object rests on (the table, a drawer's floor, a shelf): its bottom in a generated world."""
        from . import tasks as T
        if self.fx is None:
            return T.TABLE_TOP
        return float(self.mug_center(name)[2] - self.vertical_half(name))

    def jaw_order(self, name, c, phi, dzs, w, span=None):
        """Jaw directions ordered by fit (the object must be narrower than the jaws open: span(dz)) and room for the
        fingers (other objects, the container's walls); the smallest wrist turn first when there is room either way."""
        if span is not None:
            fit = [dz for dz in dzs if span(dz) <= w - 0.006]
            if fit and len(fit) < len(dzs):
                rest = [dz for dz in dzs if dz not in fit]
                return self.jaw_order(name, c, phi, fit, w) + rest
        cont = self.container_of(name)
        others = [(self.mug_center(n), self.objs[n]["r"]) for n in self.objs if n != name]

        def room(dz):
            a = np.array([math.cos(phi + dz), math.sin(phi + dz), 0.0])
            worst = 1.0
            for s in (-1, 1):
                f = np.asarray(c, float) + s * a * (w / 2 + 0.012)
                for p, r in others:
                    if abs(p[2] - c[2]) < 0.12:
                        worst = min(worst, float(np.hypot(*(f[:2] - p[:2]))) - r)
                if cont is not None and cont.get("kind") in ("drawer", "sink", "bin", "tote", "basket", "carton", "cabinet", "fridge", "rack", "drum", "vise", "tray"):
                    cc, h, R = self.fx.box(cont)
                    loc = R.T @ (f - cc)
                    worst = min(worst, float(min(h[0] - abs(loc[0]), h[1] - abs(loc[1]))) + 0.01)
            return worst
        rooms = [room(dz) for dz in dzs]
        if min(rooms) > 0.03:
            return list(dzs)
        return [dz for _, _, dz in sorted(zip(rooms, range(len(dzs)), dzs), key=lambda t: (-round(t[0], 3), t[1]))]

    def fx_slow(self, events, idx_map):
        g = next((k for k, n in events.items() if n == "grasp_fx"), None)
        r = next((k for k, n in events.items() if n == "release_fx"), None)
        if g is None or r is None:
            return []
        return [(idx_map[g], idx_map[r], self.fx_speed)]

    def _transit(self, a, b, carried=None, hang=0.0, extra=()):
        """A carry height over everything near the straight line a -> b (objects, a container's rim) and the surface."""
        from . import tasks as T
        z = T.TABLE_TOP + 0.10
        a2, b2 = np.asarray(a, float)[:2], np.asarray(b, float)[:2]
        ab = b2 - a2
        L2 = float(np.dot(ab, ab)) or 1e-9
        for n in self.objs:
            if n == carried:
                continue
            p = self.mug_center(n)
            t = float(np.clip(np.dot(p[:2] - a2, ab) / L2, 0.0, 1.0))
            if np.linalg.norm(p[:2] - (a2 + t * ab)) < self.objs[n]["r"] + 0.10:
                z = max(z, float(p[2] + self.vertical_half(n)) + 0.05)
        z = max(z, self._fixture_tops(a2, b2) + 0.05)
        for e in extra:
            z = max(z, float(e))
        return z + hang

    def _fixture_tops(self, a2, b2, margin=0.10):
        """The highest static furniture top near the line a -> b that a hand can pass over (a machine's enclosure, a
        microwave, a tote's rim): not the counter itself (the surface) and not walls / fridges / racks (too tall)."""
        from . import tasks as T
        m, d = self.rt.model, self.rt.data
        if getattr(self, "_tops", None) is None:              # static: (centre xy, half xy, top z) of each fx_ box, once
            tops = []
            for g in range(m.ngeom):
                if m.geom_bodyid[g] != 0 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                    continue
                if not (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("fx_"):
                    continue
                R, s = d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
                ext = np.abs(R) @ s
                tops.append((d.geom_xpos[g][:2].copy(), ext[:2].copy(), float(d.geom_xpos[g][2] + ext[2]), float(d.geom_xpos[g][2] - ext[2])))
            self._tops = tops
        best = -1.0
        ab = b2 - a2
        L2 = float(np.dot(ab, ab)) or 1e-9
        for c, h, top, bottom in self._tops:
            if not (T.TABLE_TOP + 0.02 < top < T.TABLE_TOP + 0.75) or bottom > T.TABLE_TOP + 0.10:   # hanging cabinets: pass under
                continue
            t = float(np.clip(np.dot(c - a2, ab) / L2, 0.0, 1.0))
            q = a2 + t * ab                                     # the line's closest point to the box centre, then to the box
            if np.all(np.abs(q - c) <= h + margin):
                best = max(best, top)
        return best

    # ------------------------------------------------------------------ actions
    def operate(self, tid, action, what, amount=None):
        """open / close a drawer, door or lid by its handle (see the module docstring)."""
        act = "close" if action == "close" else "open"
        caps = getattr(self.rt, "capabilities", None)
        if caps is not None and "pick_place" not in caps:
            prof = getattr(self.rt, "profile", None)
            self.set_task(tid, act, "failed", f"I can't {act} things with this body: it has no gripper I can drive. " + (getattr(prof, "note", "") or ""))
            return
        p, why = self.find_part(what)
        if p is None:
            self.set_task(tid, act, "failed", why)
            return
        held = next((o for o in self.holding.values() if o), None)
        if held:
            self.set_task(tid, act, "failed", f"I'm holding {self.pretty(held)}. Put it down first, then I'll {act} {p['label']}.")
            return
        fx, rt = self.fx, self.rt
        lo, hi = p["range"]
        if act == "open":
            try:
                amt = 0.8 if amount in (None, "") else float(amount)
            except (TypeError, ValueError):
                amt = 0.8
            amt = float(np.clip(amt / 100.0 if amt > 1.0 else amt, 0.1, 1.0))
        else:
            amt = 0.0
        q0, goal = fx.q(p), lo + amt * (hi - lo)
        if abs(q0 - goal) < 0.04 * (hi - lo):
            self.set_task(tid, act, "done", f"{p['label'][:1].upper() + p['label'][1:]} is already {'open' if act == 'open' else 'closed'}"
                          + (f" ({fx.frac(p) * 100:.0f}%)." if act == "open" else "."))
            return
        h0, out0 = fx.handle_now(p) if self.plant is not None else fx.handle_at(p, q0)
        above = self._over_knob(p, h0)
        if above:                                 # the hand comes from above: a knob right under another one is out
            self.set_task(tid, act, "failed", f"I can't get my fingers around the handle of {p['label']}: {above} is right above it "
                          "and my gripper only grasps handles from above (a robot that reaches in from the front could).")
            return
        palm = self.hand_for(h0)
        role = self.role(palm)
        g = self.plant.grip.get(role) if self.plant is not None else None
        if self.plant is not None and g is None:
            self.set_task(tid, act, "failed", f"I have no gripper on my {self.limb(role)} to hold {p['label']}'s handle.")
            return
        tip = g["tip"] if g else 0.0
        axis = g["axis"] if g else np.array([0, 1.0, 0])
        appr = g["approach"] if g else np.array([0, 0, 1.0])
        L = np.column_stack([axis, appr, np.cross(axis, appr)])
        R0 = self._part_R(p, q0)

        def hand(qv):                             # the TCP and the hand orientation with the knob at joint value qv
            hp, out = fx.handle_at(p, qv)
            dR = self._part_R(p, qv) @ R0.T
            o = dR @ out0
            a = np.cross([0.0, 0.0, 1.0], [o[0], o[1], 0.0])
            if np.linalg.norm(a) < 1e-6:          # a knob pointing up / down (a lid turned over): keep the start's jaw axis
                a = dR @ np.cross([0.0, 0.0, 1.0], [out0[0], out0[1], 0.0])
            a = a / (np.linalg.norm(a) or 1.0)
            n = np.array([0.0, 0.0, -1.0])
            W = np.column_stack([a, n, np.cross(a, n)])
            qq = np.zeros(4)
            mujoco.mju_mat2Quat(qq, (W @ L.T).flatten())
            return hp + np.array([0, 0, tip - 0.010]), qq

        n_s = max(2, int(abs(goal - q0) / (0.02 if p["type"] == "slide" else math.radians(5))) + 1)
        samples = [hand(q0 + (goal - q0) * k / n_s) for k in range(n_s + 1)]
        g0, qg = samples[0]
        cur = rt.data.site_xpos[rt.palm_sid[palm]].copy()
        q_now = rt.data.mocap_quat[rt.hands_mid[palm]].copy()
        home = self.home_palm[palm]
        kps, quats = [cur], [q_now]
        from . import tasks as T
        if home[2] < T.TABLE_TOP + 0.06 and home[0] < T.TABLE_X[0] + 0.05 and cur[2] < T.TABLE_TOP + 0.06 and g0[2] > T.TABLE_TOP:
            kps.append(np.array([cur[0], cur[1], T.TABLE_TOP + 0.10]))
            quats.append(q_now)
        o0 = np.array([out0[0], out0[1], 0.0]) / (np.hypot(out0[0], out0[1]) or 1.0)
        tall = g0[2] > T.TABLE_TOP + 0.05            # a machine / appliance on the surface: come from the side it faces
        far0 = g0 + o0 * 0.18 + np.array([0, 0, 0.10])
        if tall:
            kps.append(far0)
            quats.append(qg)
        kps += [g0 + np.array([0, 0, 0.10]), g0]
        quats += [qg, qg]
        ev = {len(kps) - 1: "grasp_fx"}
        for hp, qq in samples[1:]:
            kps.append(hp)
            quats.append(qq)
        ev[len(kps) - 1] = "release_fx"
        ge, qe = samples[-1]
        kps.append(ge + np.array([0, 0, 0.10]))
        quats.append(qe)
        if tall:                                    # back out the way it came, not across the machine
            kps += [ge + o0 * 0.18 + np.array([0, 0, 0.10]), far0]
            quats += [qe, qe]
        kps.append(home)
        quats.append(self.home_quat[palm])
        if self.plant is not None:
            rt.set_obstacles((p["name"],), near=[h0, samples[-1][0]], radius=0.25, fixtures=True)
            self.plant.grip_cmd(role, "open")
        self.fxop = {"part": p, "goal": goal, "amount": amt, "palm": palm, "action": act, "q0": q0, "attached": False}
        lab = p["label"]
        verb = "Opening" if act == "open" else "Closing"
        self.done_msg = f"{'Opened' if act == 'open' else 'Closed'} {lab}."
        gk = [k for k, e in ev.items() if e == "grasp_fx"][0]
        ok = self._launch(tid, act, kps, palm, ev, "task", f"{verb} {lab}" + (f" to {amt * 100:.0f}%." if act == "open" else "."),
                          quats=quats, loose=[(0, gk - 1)] if gk > 1 else None)
        if not ok:
            self.fxop = None
            if self.task and self.task.get("status") == "failed" and "reach" in self.task.get("message", ""):
                self.task["message"] = self.task["message"].rstrip(".") + f" ({lab} is out of reach for this robot)."

    def _over_knob(self, p, h):
        """What sits right above a knob (another drawer's knob, a shelf) within a hand's height: its label, or None."""
        m, d = self.rt.model, self.rt.data
        gid = np.zeros(1, np.int32)
        for dx, dy in ((0.0, 0.0), (0.012, 0.0), (-0.012, 0.0)):
            start = np.array([h[0] + dx, h[1] + dy, h[2] + 0.025])
            dist = mujoco.mj_ray(m, d, start, np.array([0.0, 0.0, 1.0]), np.array([1, 0, 0, 0, 0, 0], np.uint8), 1, p["bid"], gid)
            if 0 <= dist < 0.16:
                b = m.geom_bodyid[gid[0]]
                q = next((x for x in self.fx.by_name.values() if x["bid"] == b), None)
                if q is not None:
                    return q["label"]
                if b == 0 or (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith("fx_"):
                    return "the " + (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid[0]) or "furniture").replace("fx_", "").split("_")[0]
        return None

    def _part_R(self, p, qv):
        _, quat = self.fx.pose_at(p, qv)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        return R.reshape(3, 3)

    def pick_from(self, tid, container, obj=None):
        """Take an object out of a container and hold it, lifted clear."""
        c = None
        if container:
            c, why = self.find_container(container)
            if c is None:
                self.set_task(tid, "pick_from", "failed", why)
                return
        if c is not None and obj and c.get("kind") in ("slot", "tray"):   # 'the rack': the slot the object is in
            want = self.obj_name(obj)
            sib = next((s for s in self.fx.containers if s["fixture"] == c["fixture"] and want in self.contents(s)), None) if want else None
            c = sib or c
        inside = self.contents(c) if c is not None else list(self.objs)
        name = None
        if obj:
            name = self._match_in(obj, inside) or self.obj_name(obj)
            if name is None:
                self.set_task(tid, "pick_from", "failed", f"I don't see '{obj}'" + (f" in {c['label']}" if c else "") + ".")
                return
        elif inside:
            name = min(inside, key=lambda n: (-self.mug_center(n)[2], float(np.linalg.norm(self.mug_center(n)[:2]))))
        if name is None:
            self.set_task(tid, "pick_from", "failed", f"{c['label'][:1].upper() + c['label'][1:]} is empty.")
            return
        if c is not None and name not in inside:
            where = self.container_of(name)
            have = ", ".join(self.pretty(n) for n in inside[:5]) or "nothing"
            self.set_task(tid, "pick_from", "failed", f"{self.pretty(name)[:1].upper() + self.pretty(name)[1:]} isn't in {c['label']}"
                          + (f" (it's in {where['label']})" if where else "") + f". In {c['label']} there is {have}.")
            return
        c = c or self.container_of(name)
        if c is not None and not self._reachable_guard(tid, "pick_from", c):
            return
        if next((h for h, o in self.holding.items() if o == name), None):
            self.set_task(tid, "pick_from", "done", f"I'm already holding {self.pretty(name)}.")
            return
        from . import tasks as T
        top = self.fx.box(c)[0][2] + self.fx.box(c)[1][2] if c is not None else T.TABLE_TOP
        hang = 2 * self.vertical_half(name) + 0.02
        o2 = self.mug_center(name)[:2]
        self.transit_z = max(T.TABLE_TOP + 0.10, top + 0.04, self._fixture_tops(o2, o2) + 0.05) + hang
        self.place_z, self.place_check, self.place_drop = None, None, 0.0
        self.approach_z = self._approach(name)
        try:
            self.pick_place(tid, name, None, None, None, None, dest=(None, c["label"] if c else ""), hold=True, action="pick_from")
        finally:
            self.transit_z = self.approach_z = None

    def _approach(self, name):
        """How high the hand has to come in to get over a machine's walls on the way to `name` (None: nothing in the way)."""
        rt = self.rt
        hand = rt.data.site_xpos[rt.palm_sid[self.hand_for(self.mug_center(name))]][:2]
        tops = self._fixture_tops(np.asarray(hand, float), self.mug_center(name)[:2], margin=0.08)
        return None if tops < 0 else tops + 0.09

    def place_in(self, tid, obj, container):
        """Put an object (held, or picked first) into a container."""
        c, why = self.find_container(container, for_object=None)
        if c is None:
            self.set_task(tid, "place_in", "failed", why)
            return
        held = next((o for o in self.holding.values() if o), None)
        name = (self.obj_name(obj) if obj else None) or held
        if name is None:
            self.set_task(tid, "place_in", "failed", f"What should I put in {c['label']}? I can move {self.movable_list()}.")
            return
        if c.get("kind") == "slot":                # a rack: the free slot (a named slot that is taken gets the next free one)
            c2, _ = self.find_container(c["fixture"], for_object=name) if self.contents(c, skip=name) else (c, None)
            c = c2 or c
        if not self._reachable_guard(tid, "place_in", c):
            return
        src = None if name == held else self.container_of(name)
        if src is not None and not self._reachable_guard(tid, "place_in", src):   # it has to come out of something closed
            return
        cc, h, R = self.fx.box(c)
        spot, sup = self._spot_in(c, name)
        if spot is None:
            self.set_task(tid, "place_in", "failed", f"{c['label'][:1].upper() + c['label'][1:]} is full: there is no room for {self.pretty(name)}.")
            return
        from . import tasks as T
        hang = 2 * self.vertical_half(name) + 0.02
        start = self.mug_center(name)
        rim = cc[2] + h[2]
        self.transit_z = self._transit(start, spot, carried=name, hang=hang, extra=(rim + 0.05, T.TABLE_TOP + 0.06))
        self.place_z, self.place_check = sup, c
        from .world import objects as wobj
        self.place_drop = 0.004 if c.get("kind") == "slot" else 0.02 if wobj.grasp_class(self.kinds.get(name, "")) in ("long", "flat") else 0.012
        if name != held:                          # picked on the way: over a machine's walls into it, and lifted out of it
            self.approach_z = self._approach(name)
            self.transit_z = max(self.transit_z, self._fixture_tops(start[:2], start[:2]) + 0.05 + hang)
        try:
            self.pick_place(tid, name, None, None, None, None, dest=(np.array([spot[0], spot[1], sup]), c["label"]), action="place_in")
        finally:
            self.transit_z, self.place_drop, self.approach_z = None, 0.0, None
            if not self.task or self.task.get("status") not in ("running", "planning"):
                self.place_check = None
            self.place_z = None

    def _match_in(self, text, names):
        """The best match for `text` among `names` (objects in one container)."""
        best, bs = None, 0
        t = str(text or "").lower()
        for n in names:
            s = 10 if n.replace("_", " ") in t or n in t else 0
            s += sum(2 for k in self.keys_for(n)[1:] if k in t)
            s += 3 if self.kinds.get(n, "") and self.kinds[n].replace("_", " ") in t else 0
            if s > bs:
                best, bs = n, s
        return best

    def _reachable_guard(self, tid, action, c):
        """The drawer / door in front of a container has to be open. -> ok?"""
        p = self.guard_of(c)
        if p is None:
            return True
        need = 0.5 if p["kind"] == "drawer" else 0.6
        if self.fx.frac(p) < need:
            self.set_task(tid, action, "failed", f"{p['label'][:1].upper() + p['label'][1:]} is {'closed' if self.fx.frac(p) < 0.05 else 'only ' + format(self.fx.frac(p) * 100, '.0f') + '% open'}: "
                          f"open it first (I need it at least {need * 100:.0f}% open to get in).")
            return False
        return True

    def _spot_in(self, c, name):
        """A free spot inside a container for `name` and the height it will rest on: -> (xyz, support z) | (None, None)."""
        cc, h, R = self.fx.box(c)
        r = self.objs[name]["r"] if name in self.objs else 0.03
        others = [(self.mug_center(n), self.objs[n]["r"]) for n in self.contents(c, skip=name)]
        best = None
        mx, my = max(0.0, h[0] - min(r, h[0]) * 0.8), max(0.0, h[1] - min(r, h[1]) * 0.8)
        for lx in np.linspace(-mx, mx, 5) if mx > 0.005 else [0.0]:
            for ly in np.linspace(-my, my, 5) if my > 0.005 else [0.0]:
                p = cc + R @ np.array([lx, ly, 0.0])
                clear = min([float(np.hypot(*(p[:2] - q[:2]))) - rq - r for q, rq in others] or [1.0])
                score = min(clear, 0.15) - 0.15 * math.hypot(lx / (h[0] or 1), ly / (h[1] or 1)) - 0.05 * float(np.linalg.norm(p[:2]))
                if best is None or score > best[0]:
                    best = (score, p, clear)
        if best is None:
            return None, None
        _, p, clear = best
        floor = float(cc[2] - h[2])
        sup = floor
        if self.plant is not None and name in self.objs and (clear < 0.0 or self.objs[name]["r"] > min(h[0], h[1])):
            # on top of what is there (a full sink), under its whole footprint (a long spoon reaches past the spot)
            Ro, m = self.obj_mat(name), self.rt.model
            hs = m.geom_size[self.objs[name]["geom"]]
            hx, hy = (hs[0], hs[0]) if m.geom_type[self.objs[name]["geom"]] == mujoco.mjtGeom.mjGEOM_CYLINDER else (hs[0], hs[1])
            pts = [p] + [p + Ro @ np.array([sx * hx * 0.9, sy * hy * 0.9, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
            sup = max(self._ray_down(q, floor, skip=name) for q in pts)
        return p, sup

    def _ray_down(self, p, floor, skip=None):
        m, d = self.rt.model, self.rt.data
        start = np.array([p[0], p[1], floor + 0.4])
        gid = np.zeros(1, np.int32)
        excl = self.objs[skip]["bid"] if skip in self.objs else -1
        dist = mujoco.mj_ray(m, d, start, np.array([0.0, 0.0, -1.0]), np.array([0, 0, 0, 1, 0, 0], np.uint8), 1, excl, gid)
        if dist < 0 or m.body_rootid[m.geom_bodyid[gid[0]]] in getattr(self, "robot_roots", ()):
            return floor
        return max(floor, float(start[2] - dist))

    # ------------------------------------------------------------------ running: physics
    def _fx_event(self, name, palm):
        op = self.fxop
        if op is None:
            return
        rt = self.rt
        if self.plant is None:                    # kinematic: the part follows the hand
            if name == "grasp_fx":
                op["attached"] = True
                op["tcp0"] = rt.data.site_xpos[rt.palm_sid[palm]].copy()
                op["qa"] = self.fx.q(op["part"])
                op["riders"] = self._riders(op["part"])
            else:
                op["attached"] = False
                self._fx_done_msg()
            return
        now = float(self.plant.d.time)
        if name == "grasp_fx":
            self.ph = {"phase": "fx_settle", "palm": palm, "t0": now}
        else:
            self.ph = {"phase": "fx_open", "palm": palm, "t0": now}
            self.plant.grip_cmd(self.role(palm), "open")
            self._fx_impedance(False)
        self._hold_path(True)

    def _pads_touch(self, role, p):
        pl = self.plant
        g = pl.grip.get(role)
        if g is None:
            return False, False
        geoms = set(self.fx.geoms(p))
        a, b = set(g["pads"][0]), set(g["pads"][1])
        ta = tb = False
        d = pl.d
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in geoms or c.geom2 in geoms:
                o = c.geom2 if c.geom1 in geoms else c.geom1
                ta |= o in a
                tb |= o in b
        return ta, tb

    def _fx_impedance(self, on):
        """Compliance while a drawer / door is in hand: the plant's impedance mode (restored after)."""
        op, pl = self.fxop, self.plant
        if pl is None or op is None:
            return
        if on and "imp0" not in op:
            op["imp0"] = bool(pl.params["impedance"])
            pl.set_params({"impedance": True})
        elif not on and "imp0" in op:
            pl.set_params({"impedance": op.pop("imp0")})

    def _fx_tick(self, now):
        rt, pl, ph, op = self.rt, self.plant, self.ph, self.fxop
        if op is None:
            self.ph = None
            self._hold_path(False)
            return
        palm = ph["palm"]
        role = self.role(palm)
        p = op["part"]
        tcp = rt.data.site_xpos[rt.palm_sid[palm]]
        tgt = rt.data.mocap_pos[rt.hands_mid[palm]]
        lab = p["label"]
        if ph["phase"] == "fx_settle":
            if np.linalg.norm(tcp - tgt) < 0.012 or now - ph["t0"] > 1.2:
                pl.grip_cmd(role, "close")
                ph["phase"], ph["t0"] = "fx_close", now
        elif ph["phase"] == "fx_close":
            ta, tb = self._pads_touch(role, p)
            if (ta and tb and now - ph["t0"] > 0.35) or now - ph["t0"] > 1.0:
                if not (ta and tb):
                    self._fail_phys(palm, f"I couldn't get a grip on the handle of {lab}: my fingers closed without holding it.")
                    return
                self._fx_impedance(True)
                op["q_grasp"] = self.fx.q(p)
                self.ph = {"phase": "fx_move", "palm": palm, "t0": now, "lost": None, "lag": None}
                self._hold_path(False)
        elif ph["phase"] == "fx_move":
            ta, tb = self._pads_touch(role, p)
            ph["lost"] = None if (ta or tb) else (ph["lost"] if ph["lost"] is not None else now)
            lag = float(np.linalg.norm(tcp - tgt))
            ph["lag"] = None if lag < 0.045 else (ph["lag"] if ph["lag"] is not None else now)
            frac = self.fx.frac(p) * 100
            moved = abs(self.fx.q(p) - op.get("q_grasp", op["q0"])) > 0.01 * (p["range"][1] - p["range"][0]) + 1e-4
            if ph["lost"] is not None and now - ph["lost"] > 0.4:
                if moved:
                    self._fail_phys(palm, f"My gripper slipped off the handle of {lab}; it's {frac:.0f}% open now.")
                else:                             # pulled hard enough to slip off without moving it at all: it's jammed
                    self._fail_phys(palm, f"{lab[:1].upper() + lab[1:]} won't {op['action']}: it's stuck at {frac:.0f}% open (it "
                                          "didn't move when I pulled, and my gripper slipped off the handle).")
            elif ph["lag"] is not None and now - ph["lag"] > 0.9:
                self._fail_phys(palm, f"{lab[:1].upper() + lab[1:]} won't {op['action']} any further: it's stuck at {frac:.0f}% open"
                                + ("" if moved else " (it didn't move when I pulled)") + ". I let go of the handle.")
        elif ph["phase"] == "fx_open":
            ta, tb = self._pads_touch(role, p)
            if (now - ph["t0"] > 0.4 and not (ta or tb)) or now - ph["t0"] > 1.2:
                self._fx_done_msg()
                self.ph = None
                self._hold_path(False)

    def _fx_done_msg(self):
        op = self.fxop
        if op is None:
            return
        p = op["part"]
        f = self.fx.frac(p)
        op["final"] = f
        if op["action"] == "open":
            self.done_msg = f"Opened {p['label']} to {f * 100:.0f}%."
        else:
            self.done_msg = f"Closed {p['label']}." if f < 0.05 else f"Pushed {p['label']} to {f * 100:.0f}% open."

    def _fx_short(self):
        """At the end of an open / close: did it get there? -> why not, or None."""
        op, self.fxop = self.fxop, None
        if op is None:
            return None
        self._fx_impedance(False)
        p = op["part"]
        f = self.fx.frac(p)
        if abs(f - op["amount"]) > 0.15 and not (op["action"] == "open" and f > op["amount"]):
            return f"{p['label'][:1].upper() + p['label'][1:]} only got to {f * 100:.0f}% open (I wanted {op['amount'] * 100:.0f}%)."
        return None

    def _fx_cleanup(self):
        """A task ended early (failed, stopped, sent home): compliance back to what it was, nothing in hand."""
        if getattr(self, "fxop", None) is not None:
            self._fx_impedance(False)
        self.fxop = None

    def fixture_reset(self):
        """Scene reset: every part closed (the plant's reset already did it with physics)."""
        if self.fx is None:
            return
        self.fxop = None
        if self.plant is None:
            for p in self.fx.parts:
                if p["kind"] not in ("slat", "roller"):
                    self.fx.set_q(p, p["range"][0])

    # ------------------------------------------------------------------ running: kinematic
    def _riders(self, p):
        """Objects in a part's container (they ride along with it in kinematic mode)."""
        cs = [c for c in self.fx.containers if c.get("body") == p["body"]]
        out = {}
        if not cs:
            return out
        pos, quat = self.fx.pose_at(p, self.fx.q(p))
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        for n in self.objs:
            if any(self.fx.inside(c, self.mug_center(n)) for c in cs):
                out[n] = (R.T @ (self.obj_pos(n) - pos), self.obj_mat(n))
        return out

    def _fx_kin_tick(self):
        op = getattr(self, "fxop", None)
        if op is None or not op.get("attached") or self.plant is not None:
            return
        rt, fx, p = self.rt, self.fx, op["part"]
        tcp = rt.data.site_xpos[rt.palm_sid[op["palm"]]].copy()
        pos0, quat0 = fx.pose_at(p, op["qa"])
        R0 = np.zeros(9)
        mujoco.mju_quat2Mat(R0, p["home_quat"])
        ax = R0.reshape(3, 3) @ np.asarray(p["axis"], float)
        if p["type"] == "slide":
            q = op["qa"] + float(np.dot(tcp - op["tcp0"], ax))
        else:
            a0, a1 = op["tcp0"] - pos0, tcp - pos0
            a0, a1 = a0 - ax * np.dot(a0, ax), a1 - ax * np.dot(a1, ax)
            q = op["qa"] + math.atan2(float(np.dot(np.cross(a0, a1), ax)), float(np.dot(a0, a1)))
        fx.set_q(p, q)
        pos, quat = fx.pose_at(p, fx.q(p))
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        for n, (rel, _) in op.get("riders", {}).items():
            self.set_obj_pose(n, pos + R @ rel)
