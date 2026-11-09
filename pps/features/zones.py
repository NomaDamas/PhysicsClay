"""Keep-out zones (UX 2.0 #D): "not here". Paint a rectangle on the table in 3D, or say "stay away from the vase".

A zone is a volume standing on the table top: {id, shape: "rect"|"circle", center: [x, y, z_table], size: [w, d],
height, label, name, near, source}. Planning respects them through the tasks.keepout hook:
  at(p, r)                 -> label of the zone a table spot (with radius r) falls in, or None   (free_spot)
  along(pts, events, palm) -> per waypoint None | (label, who): the hand or the carried object would enter a zone
  route(kps, ev, palm)     -> (kps, ev, note): pick_place detours around or over zones on the way

Messages: {type:"zone", op:"add", shape?, center?, size?, rect?: [x0, y0, x1, y1], r?, height?, near?, label?, source?}
          {type:"zone", op:"remove", id} · {type:"zone", op:"clear"}
State: state.zones = [zone, ...]."""
import numpy as np

from .. import ext
from ..tasks import TABLE_TOP, densify

HAND_R = 0.025          # palm radius for the hand-in-zone test
DEFAULT_H = 0.30        # painted zone height above the table top
NEAR_MARGIN = 0.06      # "stay away from X": this much clearance around X
ALIASES = {"mug": ("mug", "cup", "컵", "머그"), "red_block": ("red", "빨간"), "blue_block": ("blue", "파란"),
           "vase": ("vase", "flower", "꽃병", "화병")}


def _zones(ctx):
    return ctx.store.setdefault("zones", [])


def _next_id(ctx):
    used = {z["id"] for z in _zones(ctx)}
    for k in range(26):
        c = chr(65 + k)
        if c not in used:
            return c
    return None


def inside(z, p, r=0.0, bottom=None):
    """Does a body at p (xyz) with xy radius r and lowest point `bottom` overlap zone z?"""
    top = TABLE_TOP + z["height"]
    if (p[2] if bottom is None else bottom) >= top:
        return False
    dx, dy = p[0] - z["center"][0], p[1] - z["center"][1]
    if z["shape"] == "circle":
        return float(np.hypot(dx, dy)) < z["size"][0] / 2 + r
    return abs(dx) < z["size"][0] / 2 + r and abs(dy) < z["size"][1] / 2 + r


class Keepout:
    """The tasks.keepout hook. Every method is a no-op while there are no zones."""

    def __init__(self, ctx):
        self.ctx = ctx

    @property
    def zones(self):
        return _zones(self.ctx)

    def at(self, p, r=0.0):
        """An object of radius r set down at p, and the hand placing it from the robot's side, stay out of every zone?"""
        p = np.asarray(p, float)
        hand = p + np.array([-(r + 0.015), 0.0, 0.0])
        for z in self.zones:
            if inside(z, p, r, TABLE_TOP) or inside(z, hand, HAND_R, TABLE_TOP):
                return z["label"]
        return None

    # --------------------------------------------------------------- carried object
    def _carry(self, events, palm, n):
        """-> (start, end, offset hand->object centre, r, half_h) for the stretch the current object is in this hand."""
        t = self.ctx.tasks
        held_now = t.holding.get(palm) == t.cur
        g = next((i for i, nm in sorted(events.items()) if nm == "grasp"), None)
        rel = next((i for i, nm in sorted(events.items()) if nm == "release"), n - 1)
        start = 0 if held_now else g
        if start is None:
            return None
        off = (t.mug_center() - t.rt.data.site_xpos[t.rt.palm_sid[palm]]) if held_now else \
            -getattr(t, "cur_off", np.array([-(t.mug_r + 0.015), 0.0, 0.01]))
        return start, rel, off, t.mug_r, t.mug_half_h

    def _hit(self, p, carry=None):
        """-> (label, who) if the hand at p (or the object it carries) enters a zone."""
        for z in self.zones:
            if inside(z, p, HAND_R, p[2] - HAND_R):
                return z["label"], "hand"
            if carry is not None:
                off, r, hh = carry
                c = p + off
                if inside(z, c, r, c[2] - hh):
                    return z["label"], self.ctx.tasks.pretty(self.ctx.tasks.cur)
        return None

    def along(self, pts, events, palm):
        if not self.zones:
            return None
        pts = np.asarray(pts, float)
        cr = self._carry(events, palm, len(pts))
        out = [None]                     # the hand may always leave where it is now
        for j in range(1, len(pts)):
            carry = (cr[2], cr[3], cr[4]) if cr and cr[0] < j <= cr[1] else None
            out.append(self._hit(pts[j], carry))
        return out

    # --------------------------------------------------------------- detours for pick_place
    def _clear(self, poly, carry, fixed=False):
        """No zone on the way (fixed=True: and the carried object clears the fixed obstacles, e.g. the vase, too)."""
        pts = densify(poly, 0.02)
        return all(self._hit(p, carry) is None and not (fixed and self._bumps(p, carry)) for p in pts[1:])

    def _bumps(self, p, carry):
        if carry is None:
            return False
        off, r, hh = carry
        c = p + off
        return any(np.linalg.norm(c[:2] - o["pos"][:2]) < o["r"] + r + 0.01 and c[2] - hh < o["pos"][2] + o["h"] + 0.01
                   for o in self.ctx.tasks.obstacles.values())

    def route(self, kps, ev, palm):
        if not self.zones:
            return kps, ev, ""
        cr = self._carry(ev, palm, len(kps))
        out, new_ev, notes = [np.asarray(kps[0], float)], {}, []
        if 0 in ev:
            new_ev[0] = ev[0]
        for i in range(len(kps) - 1):
            a, b = np.asarray(kps[i], float), np.asarray(kps[i + 1], float)
            carry = (cr[2], cr[3], cr[4]) if cr and cr[0] <= i and i + 1 <= cr[1] else None
            vias = []
            if not self._clear([a, b], carry):
                vias, note = self._detour(out, a, b, palm, carry)
                if note:
                    notes.append(note)
            out += vias + [b]
            if i + 1 in ev:
                new_ev[len(out) - 1] = ev[i + 1]
        if not notes:
            return kps, ev, ""
        return out, new_ev, ", " + " and ".join(dict.fromkeys(notes))

    def _detour(self, prefix, a, b, palm, carry):
        """Shortest reachable way around (either side) or over the zones between a and b -> (vias, note)."""
        seg = densify([a, b], 0.02)
        hits = [k for k, p in enumerate(seg) if self._hit(p, carry)]
        if not hits or hits[0] == 0 or hits[-1] == len(seg) - 1:
            return [], None             # a or b itself is in a zone: no detour helps; the launch check names it
        names = list(dict.fromkeys(self._hit(seg[k], carry)[0] for k in hits))
        ab = b - a
        L = float(np.linalg.norm(ab[:2]))
        cands = []
        if L > 1e-6:
            nrm = np.array([-ab[1], ab[0], 0.0]) / L
            t0 = max(0.0, hits[0] / (len(seg) - 1) - 0.08 / L)
            t1 = min(1.0, hits[-1] / (len(seg) - 1) + 0.08 / L)
            zmid = max(a[2], b[2])
            for s in (1.0, -1.0):
                for d in np.arange(0.06, 0.46, 0.04):
                    v1, v2 = a + ab * t0 + nrm * s * d, a + ab * t1 + nrm * s * d
                    v1[2] = v2[2] = zmid
                    if self._clear([a, v1, v2, b], carry, fixed=True):
                        cands.append((2 * d, [v1, v2], "around"))
                        break
        top = max(TABLE_TOP + z["height"] for z in self.zones if z["label"] in names)
        need = top + 0.04 if carry is None else top + 0.03 + carry[2] - carry[0][2]   # hand z so the object clears the top
        if need > max(a[2], b[2]) - 1e-3:
            v1, v2 = np.array([a[0], a[1], need]), np.array([b[0], b[1], need])
            if self._clear([a, v1, v2, b], carry, fixed=True):
                cands.append((2 * (need - max(a[2], b[2])) + 0.02, [v1, v2], "over"))
        rt = self.ctx.tasks.rt
        for _, vias, how in sorted(cands, key=lambda c: c[0])[:4]:
            pre = densify(prefix, 0.035) if len(prefix) > 1 else np.asarray(prefix, float)
            ok = rt.precheck(densify(list(prefix) + vias + [b], 0.035), palm)
            if all(ok[len(pre) - 1:]):
                return vias, f"going {how} {' and '.join(names)}"
        return [], None


def _keepout(ctx):
    if not isinstance(getattr(ctx.tasks, "keepout", None), Keepout):
        ctx.tasks.keepout = Keepout(ctx)
    return ctx.tasks.keepout


def _near(ctx, text):
    """'the vase' / 'marker A' / 'left' -> (centre xyz, radius, height, name) or (None, reason)."""
    t = ctx.tasks
    low = str(text or "").strip().lower()
    for name, keys in ALIASES.items():
        if any(k in low for k in keys):
            if name in t.objs:
                c = t.mug_center(name)
                r, top = t.objs[name]["r"], c[2] + t.objs[name]["half_h"]
            elif name in t.obstacles:
                ob = t.obstacles[name]
                c, r, top = ob["pos"], ob["r"], ob["pos"][2] + ob["h"]
            else:
                continue
            return (np.array([c[0], c[1], TABLE_TOP]), r + NEAR_MARGIN, max(0.12, top - TABLE_TOP + NEAR_MARGIN), t.pretty(name)), None
    p, label = t.resolve(text)
    if p is None:
        return None, f"I don't know where '{text}' is, so I can't keep out of it."
    return (np.array([p[0], p[1], TABLE_TOP]), 0.08, DEFAULT_H, label), None


def _bare(name):
    return name[4:] if name.startswith("the ") else name


def _recheck_path(ctx):
    """A drawn route turns red (or green again) when zones change."""
    rt, t = ctx.tasks.rt, ctx.tasks
    if rt.path is not None and rt.path_msg and rt.mode != "FOLLOW" and not t.kind:
        t.set_path_world(list(rt.path["pts"]), rt.path.get("hand"))


@ext.handler("zone")
def on_zone(ctx, msg):
    zones = _zones(ctx)
    _keepout(ctx)
    op = msg.get("op", "add")
    say = (lambda text: ctx.brain.utter(text, "robot")) if msg.get("source") in ("voice", "text") else (lambda text: None)
    if op == "clear":
        n = len(zones)
        zones.clear()
        say("Cleared all keep-out zones." if n else "There were no keep-out zones.")
    elif op == "remove":
        zid = str(msg.get("id", "")).strip().upper()
        z = next((z for z in zones if z["id"] == zid or (msg.get("near") and z.get("near") and str(msg["near"]).lower() in z["near"].lower())), None)
        if z is not None:
            zones[:] = [x for x in zones if x is not z]
            say(f"Removed {z['label']}.")
        else:
            say("I couldn't find that keep-out zone.")
    elif op == "add":
        zid = _next_id(ctx)
        if zid is None:
            say("That's too many keep-out zones. Remove one first.")
            return
        h = float(np.clip(float(msg.get("height") or DEFAULT_H), 0.03, 0.8))
        near = None
        if msg.get("near"):
            got, why = _near(ctx, msg["near"])
            if got is None:
                say(why)
                return
            c, r, h, near = got
            if any(z.get("near") == near for z in zones):
                say(f"I'm already staying away from {near}.")
                return
            z = {"shape": "circle", "center": c, "size": [2 * r, 2 * r]}
        elif msg.get("rect") is not None:
            x0, y0, x1, y1 = (float(v) for v in msg["rect"][:4])
            z = {"shape": "rect", "center": np.array([(x0 + x1) / 2, (y0 + y1) / 2, TABLE_TOP]), "size": [abs(x1 - x0), abs(y1 - y0)]}
        else:
            c = [float(v) for v in (msg.get("center") or [])[:2]]
            if len(c) < 2:
                return
            shape = "circle" if msg.get("shape") == "circle" else "rect"
            size = [2 * float(msg["r"])] * 2 if shape == "circle" and msg.get("r") is not None else [float(v) for v in (msg.get("size") or [0.1, 0.1])[:2]]
            if shape == "circle":
                size = [size[0], size[0]]
            z = {"shape": shape, "center": np.array([c[0], c[1], TABLE_TOP]), "size": size}
        z["size"] = [float(np.clip(abs(s), 0.02, 1.2)) for s in z["size"]]
        z["center"] = [float(z["center"][0]), float(z["center"][1]), TABLE_TOP]
        z.update(id=zid, height=h, near=near, source=str(msg.get("source") or "paint"),
                 label=str(msg.get("label") or (f"the {_bare(near)} keep-out zone" if near else f"keep-out zone {zid}")),
                 name=_bare(near).capitalize() if near else f"Zone {zid}")
        zones.append(z)
        say(f"Okay, I'll stay away from {near}. I put a keep-out zone around it and will plan around it."
            if near else f"Okay, I'll keep out of {z['label']}.")
    _recheck_path(ctx)


@ext.state
def zones_state(ctx):
    return {"zones": [{"id": z["id"], "shape": z["shape"], "center": [round(float(v), 3) for v in z["center"]],
                       "size": [round(s, 3) for s in z["size"]], "height": round(z["height"], 3), "label": z["label"],
                       "name": z["name"], "near": z["near"], "source": z["source"]} for z in _zones(ctx)]}
