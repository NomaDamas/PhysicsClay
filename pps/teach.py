"""Teach by Showing: the user demonstrates a task (camera) while explaining it (voice/text).

The robot does not copy the motion. It turns the demonstration + words into a task spec
    {goal, steps: [pick_place(object, target)], constraints: [avoid / keep_still / upright / slow / grasp_side / grasp_top]}
asks back to confirm, practices the spec in simulation with its own body (IK + physics checks), and accepts corrections.
With a Gemini key the browser's Live session proposes the spec (set_spec); offline, the rule-based extractor below does.
"""
import json
import re
import time

import numpy as np

from .tasks import SKILL_DIR

OBJ = {
    "mug": ("mug", "cup", "컵", "머그", "커피"),
    "red_block": ("red block", "red", "block", "cube", "빨간", "블록", "큐브"),
    "blue_block": ("blue", "파란"),
    "vase": ("vase", "꽃병", "화병"),
}
TARGET = {
    "left": ("left", "왼쪽", "좌측"), "right": ("right", "오른쪽", "우측"), "center": ("center", "middle", "가운데", "중앙"),
    "front": ("front", "closer", "near me", "앞", "가까이"), "back": ("back", "far", "뒤", "멀리"),
    "there": ("there", "here", "저기", "여기", "거기"),
}
SPLIT = re.compile(r"\b(?:and then|then|after that|afterwards)\b|그 ?다음|그리고 나서|다음에|그리고|[.;!?]", re.I)
NEG = ("don't", "dont", "do not", "never", "without", "avoid", "말고", "하지 마", "지 마", "피해", "건드리지", "닿지", "안 돼", "안돼")


def _has(t, words):
    return any(w in t for w in words)


def _objects_in(t):
    found = []
    for name, words in OBJ.items():
        idx = min([t.find(w) for w in words if w in t] or [-1])
        if idx >= 0:
            found.append((idx, name))
    # "red block" also matches "block"; "blue block" must not count as red
    if any(n == "blue_block" for _, n in found) and not _has(t, ("red", "빨간")):
        found = [(i, n) for i, n in found if n != "red_block" or _has(t.replace("blue block", ""), ("block", "블록", "cube"))]
    return [n for _, n in sorted(found)]


def _target_in(t, objs):
    for oname in objs:
        for w in OBJ[oname]:
            for near in ("next to the ", "next to ", "beside the ", "near the "):
                if near + w in t:
                    return f"next to {oname.replace('_', ' ')}"
            if w + " 옆" in t or w + "옆" in t:
                return f"next to {oname.replace('_', ' ')}"
    for name, words in TARGET.items():
        if _has(t, words):
            return name
    m = re.search(r"\b(?:on|to|at)\s+(?:marker\s+)?([a-f])\b", t)
    if m:
        return m.group(1).upper()
    return None


def constraints_from(t):
    out = []
    for clause in re.split(r"[.,;!?]|\bbut\b|\band\b|하지만|근데|그리고", t):
        c = clause.strip()
        if not c:
            continue
        negs = [c.find(w) for w in NEG + ("keep", "leave") if w in c]
        start = min(negs) if negs else 0
        objs = _objects_in(c[start:]) if negs and not any(w in c for w in ("그대로", "두고", "건드리지", "피해", "닿지", "움직이지", "옮기지")) else _objects_in(c)
        if _has(c, NEG) or _has(c, ("keep", "leave", "그대로", "두고")):
            for o in objs:
                if _has(c, ("move", "옮기", "움직", "keep", "leave", "그대로", "두고")) and o != "vase":
                    out.append({"type": "keep_still", "object": o})
                elif _has(c, ("touch", "hit", "avoid", "건드", "닿", "피해", "부딪")) or o == "vase":
                    out.append({"type": "avoid", "object": o})
        if _has(c, ("upright", "spill", "tilt", "level", "쏟", "기울", "똑바로", "흘리")):
            out.append({"type": "upright"})
        if _has(c, ("slow", "careful", "gently", "천천히", "조심", "살살")):
            out.append({"type": "slow"})
        if _has(c, ("from the side", "side grip", "옆에서", "옆을 잡", "옆으로 잡")):
            out.append({"type": "grasp_side"})
        if _has(c, ("from the top", "from above", "위에서", "위를 잡")):
            out.append({"type": "grasp_top"})
    uniq, seen = [], set()
    for c in out:
        k = (c["type"], c.get("object"))
        if k not in seen:
            seen.add(k)
            uniq.append(c)
    return uniq


def describe(spec):
    pretty = {"mug": "the mug", "red_block": "the red block", "blue_block": "the blue block", "vase": "the vase"}
    tgt = lambda t: {"left": "the left of the table", "right": "the right of the table", "center": "the middle of the table",
                     "front": "the front of the table", "back": "the back of the table",
                     "there": "the spot you showed me"}.get(t, f"marker {t}" if t and len(t) == 1 else ("the spot " + t.replace("_", " ") if t and t.startswith("next to") else (t or "?")))
    steps = [f"{i + 1}) pick up {pretty.get(s['object'], s['object'])} and put it on {tgt(s['target'])}" for i, s in enumerate(spec["steps"])]
    cons = []
    for c in spec["constraints"]:
        if c["type"] == "avoid":
            cons.append(f"stay clear of {pretty.get(c.get('object'), c.get('object'))}")
        elif c["type"] == "keep_still":
            cons.append(f"leave {pretty.get(c.get('object'), c.get('object'))} where it is")
        elif c["type"] == "upright":
            cons.append("keep it upright")
        elif c["type"] == "slow":
            cons.append("move slowly")
        elif c["type"] == "grasp_top":
            cons.append("grab it from the top")
        elif c["type"] == "grasp_side":
            cons.append("grab it from the side")
    return "; ".join(steps) + (". I'll " + ", ".join(cons) if cons else "")


class Teacher:
    def __init__(self, tasks):
        self.tasks = tasks
        self.phase = "idle"
        self.t0 = 0.0
        self.utterances = []
        self.hand_uv = {"l": [], "r": []}
        self.grip_prev = {"l": 0, "r": 0}
        self.grasps = 0
        self.objects_seen = set()
        self.spec = None
        self.question = ""
        self.step_i = -1
        self.step_tid = None
        self.auto_route_tried = False

    # ------------------------------------------------------------------ recording
    def start(self):
        self.phase, self.t0 = "recording", time.time()
        self.utterances, self.hand_uv, self.grasps, self.objects_seen = [], {"l": [], "r": []}, 0, set()
        self.spec, self.question, self.step_i = None, "", -1
        self.tasks.home("teach-home")

    def observe(self, pts, grip, scene_labels):
        if self.phase != "recording":
            return
        if pts:
            from .perceive import H, W
            for s, i in (("l", 15), ("r", 16)):
                x, y = pts[i]
                self.hand_uv[s].append([round(1 - x / W, 4), round(y / H, 4)])
                del self.hand_uv[s][:-300]
        for s in ("l", "r"):
            g = int(grip.get(s, 0))
            if g and not self.grip_prev[s]:
                self.grasps += 1
            self.grip_prev[s] = g
        self.objects_seen |= set(scene_labels or [])

    def utter(self, text):
        text = str(text or "").strip()
        if text:
            self.utterances.append(text)
            if self.phase in ("confirm", "done") and not self.utterances_are_teaching:
                pass

    utterances_are_teaching = True

    def moved_side(self):
        best, side = 0.0, None
        for s in ("l", "r"):
            path = self.hand_uv[s]
            if len(path) < 6:
                continue
            u = np.array([p[0] for p in path])
            span = float(u.max() - u.min())
            if span > best:
                best = span
                d = float(np.median(u[-5:]) - np.median(u[:5]))
                side = "right" if d > 0.08 else "left" if d < -0.08 else None
        return side

    def active_hand_uv(self):
        spans = {s: (np.ptp([p[0] for p in v]) + np.ptp([p[1] for p in v])) if len(v) > 3 else 0 for s, v in self.hand_uv.items()}
        return self.hand_uv[max(spans, key=spans.get)] if spans else []

    # ------------------------------------------------------------------ understanding
    def stop(self):
        if self.phase != "recording":
            return
        text = ". ".join(u.strip().rstrip(".") for u in self.utterances).lower()
        moved = self.moved_side()
        steps = []
        for clause in SPLIT.split(text) if text else []:
            c = clause.strip()
            if not c or (_has(c, NEG) and not _has(c, ("put", "move", "place", "옮", "놔", "올려"))):
                continue
            objs = [o for o in _objects_in(c) if o in self.tasks.objs]
            if not objs:
                continue
            tgt = _target_in(c, _objects_in(c)) or moved or "there"
            if tgt == "there" and moved:
                tgt = moved
            if tgt.startswith("next to") and objs and objs[-1] != objs[0]:
                objs = objs[:1]
            steps.append({"action": "pick_place", "object": objs[0], "target": tgt})
        if not steps:
            obj = "mug" if ("cup" in self.objects_seen or "mug" in text or not text) else (_objects_in(text) or ["mug"])[0]
            steps = [{"action": "pick_place", "object": obj if obj in self.tasks.objs else "mug", "target": moved or "left"}]
        cons = constraints_from(text)
        for i, s in enumerate(steps):
            s.update({"id": f"s{i + 1}", "status": "pending", "message": ""})
        self.spec = {"goal": text[:160] or "the task you showed me", "steps": steps, "constraints": cons}
        self._ask()

    def set_spec(self, spec):
        steps = []
        for i, s in enumerate(spec.get("steps") or []):
            obj = self.tasks.obj_name(s.get("object")) or "mug"
            steps.append({"id": f"s{i + 1}", "action": "pick_place", "object": obj, "target": s.get("target") or "there",
                          "status": "pending", "message": ""})
        cons = []
        for c in spec.get("constraints") or []:
            if c.get("type") in ("avoid", "keep_still", "upright", "slow", "grasp_side", "grasp_top"):
                o = c.get("object")
                o = next((k for k, w in OBJ.items() if o and any(x in str(o).lower() for x in w)), o)
                cons.append({"type": c["type"], **({"object": o} if o else {})})
        self.spec = {"goal": spec.get("goal") or "the task you showed me", "steps": steps or
                     [{"id": "s1", "action": "pick_place", "object": "mug", "target": "left", "status": "pending", "message": ""}],
                     "constraints": cons}
        self._ask()

    def _ask(self):
        self.phase = "confirm"
        self.question = f"Let me check: {describe(self.spec)}. Is that right? Say yes and I'll practice it in simulation."
        self.tasks.set_task("teach", "teach", "done", self.question)

    def correct(self, text):
        if not self.spec:
            return
        t = str(text or "").lower()
        if _has(t, ("yes", "yeah", "right", "correct", "do it", "go", "맞아", "응", "네", "좋아", "그래")) and not _has(t, ("no", "not", "아니")):
            return self.practice()
        objs = [o for o in _objects_in(t) if o in self.tasks.objs]
        tgt = _target_in(t, _objects_in(t))
        new_cons = constraints_from(t)
        changed = []
        if _has(t, ("also", "and then", "then", "그다음", "그리고", "또")) and objs and tgt:
            self.spec["steps"].append({"id": f"s{len(self.spec['steps']) + 1}", "action": "pick_place", "object": objs[0],
                                       "target": tgt, "status": "pending", "message": ""})
            changed.append("added a step")
        else:
            step = self.spec["steps"][-1]
            for s in self.spec["steps"]:
                if objs and s["object"] == objs[0]:
                    step = s
            if tgt and not new_cons:
                step["target"] = tgt
                changed.append("changed the target")
            if objs and _has(t, ("instead", "not the", "말고", "대신")) and objs[-1] != step["object"]:
                step["object"] = objs[-1]
                changed.append("changed the object")
        for c in new_cons:
            keyset = {(x["type"], x.get("object")) for x in self.spec["constraints"]}
            if c["type"] in ("grasp_top", "grasp_side"):
                self.spec["constraints"] = [x for x in self.spec["constraints"] if x["type"] not in ("grasp_top", "grasp_side")]
            if (c["type"], c.get("object")) not in keyset:
                self.spec["constraints"].append(c)
                changed.append("added a constraint")
        for s in self.spec["steps"]:
            s["status"], s["message"] = "pending", ""
        self._ask()
        if changed:
            self.question = "Updated (" + ", ".join(dict.fromkeys(changed)) + "). " + self.question
            self.tasks.set_task("teach", "teach", "done", self.question)

    # ------------------------------------------------------------------ practice in sim
    def practice(self, spec=None):
        if spec is not None:
            self.spec = spec
        if not self.spec:
            return
        self.tasks.home("teach-home", reset_scene=True)
        rt = self.tasks.rt
        rt.speed_scale = 0.6 if any(c["type"] == "slow" for c in self.spec["constraints"]) else 1.0
        for s in self.spec["steps"]:
            s["status"], s["message"] = "pending", ""
        self.phase, self.step_i, self.step_tid = "practice", -1, None
        self._next()

    def _grasp(self):
        return "top" if any(c["type"] == "grasp_top" for c in self.spec["constraints"]) else "side"

    def _next(self):
        self.step_i += 1
        if self.step_i >= len(self.spec["steps"]):
            self.phase = "done"
            n = len(self.spec["steps"])
            self.question = f"Practice passed: {n} step{'s' if n > 1 else ''} done in simulation. Say 'save it as' and a name."
            self.tasks.set_task("teach", "teach", "done", self.question)
            return
        s = self.spec["steps"][self.step_i]
        for c in self.spec["constraints"]:
            if c["type"] == "keep_still" and c.get("object") == s["object"]:
                s["status"], s["message"] = "failed", f"This step moves {s['object'].replace('_', ' ')}, but you told me to leave it."
                return self._fail(s)
        self.auto_route_tried = False
        self._run_step(s)

    def _run_step(self, s, via=None):
        self.step_tid = f"{s['id']}-{int(time.time() * 1000) % 100000}"
        s["status"] = "running"
        self.tasks.pick_place(self.step_tid, s["object"], s["target"], None, None, 4 / 3, via_pts=via, grasp=self._grasp())

    def tick(self):
        if self.phase != "practice" or self.step_i < 0 or self.step_i >= len(self.spec["steps"]):
            return
        task = self.tasks.task
        st = getattr(self.tasks, "stopped_task", None)
        if st and st.get("id") == self.step_tid:
            task, self.tasks.stopped_task = st, None
        elif not task or task.get("id") != self.step_tid:
            return
        s = self.spec["steps"][self.step_i]
        s["message"] = task["message"]
        if task["status"] == "done":
            s["status"] = "done"
            self._next()
        elif task["status"] == "failed":
            hit = getattr(self.tasks, "hit_obstacle", None)
            if "would hit" in task["message"] and not self.auto_route_tried:
                # the constraint says "stay clear": plan a route over the obstacle myself
                self.auto_route_tried = True
                via = self.route_over(s)
                if via:
                    s["message"] = f"The straight carry hits {str(hit).replace('_', ' ')}; replanning a route over it."
                    return self._run_step(s, via)
            s["status"] = "failed"
            self._fail(s)

    def route_over(self, s):
        tk = self.tasks
        ob = tk.obstacles.get(getattr(tk, "hit_obstacle", None) or "vase")
        if ob is None:
            return None
        dest, _ = tk.resolve(s["target"])
        if dest is None:
            return None
        c = tk.mug_center(s["object"]) if s["object"] in tk.objs else None
        if c is None:
            return None
        top = ob["pos"][2] + ob["h"]
        z = top + tk.objs[s["object"]]["half_h"] * 2 + 0.06
        a, b = np.array([c[0], c[1]]), np.array([dest[0], dest[1]])
        off = -getattr(tk, "cur_off", np.array([-(tk.objs[s["object"]]["r"] + 0.015), 0, 0.01]))
        return [np.array([*(a + (b - a) * f), z]) - off for f in (0.25, 0.5, 0.75)]

    def _fail(self, s):
        self.phase = "confirm"
        hint = ""
        if "out of my" in s["message"]:
            side = "right" if "right arm" in s["message"] else "left"
            hint = f" Try a spot on my {side} side or the middle, e.g. 'put the {s['object'].replace('_', ' ')} in the middle'."
        elif "would hit" in s["message"]:
            hint = " Draw a route over it in 3D, or tell me another spot."
        self.question = f"Step {s['id'][1:]} failed: {s['message']}{hint} Tell me how to fix it, or show me again."
        self.tasks.set_task("teach", "teach", "failed", self.question)

    # ------------------------------------------------------------------ save
    def save(self, name):
        if not self.spec:
            return "Nothing to save yet."
        key = "".join(ch for ch in str(name or "task").strip().lower().replace(" ", "_") if ch.isalnum() or ch in "_-")[:32] or "task"
        with open(SKILL_DIR / f"{key}.json", "w") as f:
            json.dump({"name": key, "type": "task", "spec": self.spec, "created": time.time()}, f)
        msg = f"Saved '{key}'. Say 'do {key}' and I'll run it again."
        self.tasks.set_task("teach", "teach", "done", msg)
        return msg

    def state(self):
        return {"phase": self.phase, "seconds": round(time.time() - self.t0, 1) if self.phase == "recording" else 0,
                "utterances": self.utterances[-8:], "spec": self.spec, "question": self.question,
                "evidence": {"hand_uv": self.active_hand_uv()[-120:] if self.phase == "recording" else [],
                             "grasps": self.grasps, "moved": self.moved_side(), "objects_seen": sorted(self.objects_seen)}}
