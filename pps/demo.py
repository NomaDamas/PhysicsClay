"""Demo autopilot: a scripted, mock-driven run of the REAL system so the demo video always works.

Story: text alone fails -> a spatial prompt (3D route, pointing) succeeds -> teach by showing (show & tell, ask back,
practice, correct, save). Every robot line is the system's own output (task / teach messages); only the user's
lines and captions are scripted.
"""
import time

import numpy as np

ROUTE = [[0.28, -0.12, 1.04], [0.29, -0.02, 1.14], [0.29, 0.10, 1.14], [0.29, 0.20, 1.02]]


class Demo:
    def __init__(self, tasks, teacher, brain, rt):
        self.tasks, self.teacher, self.brain, self.rt = tasks, teacher, brain, rt
        self.steps, self.i, self.name, self.t_wait, self.cond = [], 0, None, 0.0, None

    # ------------------------------------------------------------------ script building blocks
    def cap(self, text):
        return ("do", lambda: self.brain.utter(text, "caption"), text)

    def user(self, text):
        return ("do", lambda: self.brain.utter(text, "user"), text)

    def robot_says_task(self):
        return ("do", lambda: self.brain.utter(self.tasks.task["message"] if self.tasks.task else "", "robot"), "robot")

    def wait(self, s):
        return ("wait", s, "")

    def until_task_done(self, timeout=30):
        return ("until", lambda: self.tasks.task and self.tasks.task.get("status") in ("done", "failed"), timeout)

    def until_teach(self, phases, timeout=60):
        return ("until", lambda: self.teacher.phase in phases, timeout)

    def act(self, fn, title=""):
        return ("do", fn, title)

    def script(self, name):
        tk, te = self.tasks, self.teacher
        text_vs_spatial = [
            self.act(lambda: tk.home("demo", reset_scene=True), "reset"),
            self.cap("Text only"), self.wait(1.2),
            self.user("Move the mug to the left, carefully."), self.wait(0.8),
            self.act(lambda: tk.pick_place("demo-t1", "mug", "left", None, None, 4 / 3), "text command"),
            self.wait(1.0), self.robot_says_task(), self.wait(5.5),
            self.cap("You can't put everything you think into words."), self.wait(3.2),
            self.cap("Show the path instead: lift it over the vase in 3D"), self.wait(1.5),
            self.act(lambda: tk.set_path_world([np.array(p) for p in ROUTE], "right"), "3D route"),
            self.wait(1.2), self.act(lambda: self.brain.utter(self.rt.path_msg or "Route checked.", "robot")), self.wait(3.0),
            self.user("Carry it along my path."), self.wait(0.6),
            self.act(lambda: tk.pick_place("demo-t2", "mug", "along my path", None, None, 4 / 3), "carry"),
            self.wait(0.8), self.robot_says_task(), self.until_task_done(), self.robot_says_task(), self.wait(2.5),
            self.cap("Point at it, and say “there”"), self.wait(1.5),
            self.act(lambda: (tk.set_marker("P", np.array([0.33, -0.20, 0.74])),
                              self.brain.utter("You're pointing at the right side of the table. I marked it P.", "vision")), "point"),
            self.wait(2.8), self.user("Put the red block there."), self.wait(0.6),
            self.act(lambda: tk.pick_place("demo-t3", "red block", "there", None, None, 4 / 3), "deictic"),
            self.wait(0.8), self.robot_says_task(), self.until_task_done(), self.robot_says_task(), self.wait(2.5),
        ]
        teach = [
            self.act(lambda: tk.home("demo", reset_scene=True), "reset"),
            self.cap("Teach by showing — like teaching a friend"), self.wait(2.0),
            self.act(lambda: (te.start(), self.brain.utter("I'm watching. Show me and tell me what matters.", "robot")), "start teaching"),
            self.wait(2.5),
            self.user("I'm moving the mug to the middle."), self.act(lambda: te.utter("I'm moving the mug to the middle.")), self.wait(2.2),
            self.user("Don't touch the vase."), self.act(lambda: te.utter("Don't touch the vase.")), self.wait(1.8),
            self.user("Then the red block goes on the right. Slowly."),
            self.act(lambda: te.utter("Then the red block goes on the right. Slowly.")), self.wait(2.2),
            self.act(lambda: te.stop(), "understand"), self.wait(0.5),
            self.act(lambda: self.brain.utter(te.question, "robot")), self.wait(7.5),
            self.user("No — the red block goes on the left."), self.wait(0.6),
            self.act(lambda: te.correct("No, the red block goes on the left."), "correct"), self.wait(0.6),
            self.act(lambda: self.brain.utter(te.question, "robot")), self.wait(6.0),
            self.user("Yes, practice it."), self.wait(0.5),
            self.act(lambda: te.correct("yes"), "practice"), self.until_teach(("done", "confirm"), 70),
            self.act(lambda: self.brain.utter(te.question, "robot")), self.wait(5.0),
            self.user("Save it as tidy table."), self.act(lambda: te.save("tidy table")), self.wait(0.6),
            self.robot_says_task(), self.wait(3.0),
        ]
        outro = [self.cap("New Generation of interactive prompt for Physical AI"), self.wait(4.0)]
        return {"text_vs_spatial": text_vs_spatial + outro, "teach": teach + outro,
                "full": text_vs_spatial + teach + outro}.get(name)

    # ------------------------------------------------------------------ runtime
    def start(self, name):
        steps = self.script(name)
        if steps is None:
            return
        self.steps, self.i, self.name, self.t_wait, self.cond = steps, 0, name, 0.0, None

    def stop(self):
        self.steps, self.name = [], None

    def tick(self):
        if not self.steps:
            return
        now = time.time()
        while self.i < len(self.steps):
            if self.t_wait and now < self.t_wait:
                return
            if self.cond is not None:
                fn, deadline = self.cond
                if not fn() and now < deadline:
                    return
                self.cond = None
            self.t_wait = 0.0
            kind, a, b = self.steps[self.i]
            self.i += 1
            if kind == "do":
                a()
            elif kind == "wait":
                self.t_wait = now + a
                return
            elif kind == "until":
                self.cond = (a, now + b)
                return
        self.steps, self.name = [], None

    def state(self):
        if not self.steps:
            return None
        titles = [s[2] for s in self.steps[: self.i] if s[0] == "do" and s[2]]
        return {"name": self.name, "step": self.i, "total": len(self.steps), "title": titles[-1] if titles else ""}
