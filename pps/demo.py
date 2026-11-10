"""Demo autopilot: a scripted, mock-driven run of the REAL system so the demo video always works.

Story: text alone fails -> a spatial prompt (3D route, pointing) succeeds -> teach by showing (show & tell, ask back,
practice, correct, save). Every robot line is the system's own output (task / teach messages); only the user's
lines and captions are scripted.
"""
import time
from pathlib import Path

import numpy as np

ROOT_VIDEOS = Path(__file__).resolve().parents[1] / "recordings" / "demo_videos"
ROUTE = [[0.28, -0.12, 1.04], [0.29, -0.02, 1.14], [0.29, 0.10, 1.14], [0.29, 0.20, 1.02]]


class Demo:
    def __init__(self, tasks, teacher, brain, rt, push=None, video_active=None, er=None):
        self.er = er or (lambda: None)
        self.tasks, self.teacher, self.brain, self.rt = tasks, teacher, brain, rt
        self.push = push or (lambda c: None)
        self.video_active = video_active or (lambda: False)
        self.steps, self.i, self.name, self.t_wait, self.cond = [], 0, None, 0.0, None
        self.loop_name = self.mock0 = None

    # ------------------------------------------------------------------ script building blocks
    def utter(self, text, kind):
        """Every line of a script (captions, the narrator, the scripted user and robot lines) is marked as the demo's, so
        the brain tells it from a real one (pps/mock.py): a demo line never covers a real line, and a real line stops or
        holds the demo (pps/features/agent.py, app.py)."""
        self.brain.utter(text, kind, src="demo")

    def cap(self, text):
        return ("do", lambda: self.utter(text, "caption"), text)

    def user(self, text):
        return ("do", lambda: self.utter(text, "user"), text)

    def robot_says_task(self):
        return ("do", lambda: self.utter(self.tasks.task["message"] if self.tasks.task else "", "robot"), "robot")

    def wait(self, s):
        return ("wait", s, "")

    def until(self, fn, timeout=30):
        return ("until", fn, timeout)

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
            self.wait(1.2), self.act(lambda: self.utter(self.rt.path_msg or "Route checked.", "robot")), self.wait(3.0),
            self.user("Carry it along my path."), self.wait(0.6),
            self.act(lambda: tk.pick_place("demo-t2", "mug", "along my path", None, None, 4 / 3), "carry"),
            self.wait(0.8), self.robot_says_task(), self.until_task_done(), self.robot_says_task(), self.wait(2.5),
            self.cap("Point at it, and say “there”"), self.wait(1.5),
            self.act(lambda: (tk.set_marker("P", np.array([0.33, -0.20, 0.74])),
                              self.utter("You're pointing at the right side of the table. I marked it P.", "vision")), "point"),
            self.wait(2.8), self.user("Put the red block there."), self.wait(0.6),
            self.act(lambda: tk.pick_place("demo-t3", "red block", "there", None, None, 4 / 3), "deictic"),
            self.wait(0.8), self.robot_says_task(), self.until_task_done(), self.robot_says_task(), self.wait(2.5),
        ]
        teach = [
            self.act(lambda: tk.home("demo", reset_scene=True), "reset"),
            self.cap("Teach by showing — like teaching a friend"), self.wait(2.0),
            self.act(lambda: (te.start(), self.utter("I'm watching. Show me and tell me what matters.", "robot")), "start teaching"),
            self.wait(2.5),
            self.user("I'm moving the mug to the middle."), self.act(lambda: te.utter("I'm moving the mug to the middle.")), self.wait(2.2),
            self.user("Don't touch the vase."), self.act(lambda: te.utter("Don't touch the vase.")), self.wait(1.8),
            self.user("Then the red block goes on the right. Slowly."),
            self.act(lambda: te.utter("Then the red block goes on the right. Slowly.")), self.wait(2.2),
            self.act(lambda: te.stop(), "understand"), self.wait(0.5),
            self.act(lambda: self.utter(te.question, "robot")), self.wait(7.5),
            self.user("No — the red block goes on the left."), self.wait(0.6),
            self.act(lambda: te.correct("No, the red block goes on the left."), "correct"), self.wait(0.6),
            self.act(lambda: self.utter(te.question, "robot")), self.wait(6.0),
            self.user("Yes, practice it."), self.wait(0.5),
            self.act(lambda: te.correct("yes"), "practice"), self.until_teach(("done", "confirm"), 70),
            self.act(lambda: self.utter(te.question, "robot")), self.wait(5.0),
            self.user("Save it as tidy table."), self.act(lambda: te.save("tidy table")), self.wait(0.6),
            self.robot_says_task(), self.wait(3.0),
        ]
        vids = ROOT_VIDEOS
        video = [
            self.cap("Some motions you can't describe. Show a video instead."), self.wait(2.5),
            self.user("Copy this."),
            self.act(lambda: self.push({"type": "source", "path": str(vids / "gamst_reaction.mp4"), "name": "gamst_reaction", "seconds": 11}), "video 1"),
            self.wait(1.5), self.until(lambda: not self.video_active(), 30),
            self.act(lambda: self.utter("Got it. I learned that motion from your video and saved it as a skill.", "robot")), self.wait(3.0),
            self.cap("A dance — the whole body, not just the arms"), self.wait(1.5),
            self.act(lambda: self.push({"type": "source", "path": str(vids / "dance_clip.mp4"), "name": "dance_clip", "seconds": 16}), "video 2"),
            self.wait(1.5), self.until(lambda: not self.video_active(), 40),
            self.act(lambda: self.utter("Learned the dance too: arms, torso twist and steps, checked for balance.", "robot")), self.wait(3.0),
        ]
        outro = [self.cap("New Generation of interactive prompt for Physical AI"), self.wait(4.0)]
        pitch = [
            self.act(lambda: tk.home("demo", reset_scene=True), "reset"),
            self.cap("Text only"), self.wait(0.8),
            self.user("Move the mug to the left, carefully."), self.wait(0.6),
            self.act(lambda: tk.pick_place("demo-t1", "mug", "left", None, None, 4 / 3), "text command"),
            self.wait(0.8), self.robot_says_task(), self.wait(5.0),
            self.cap("Show the path instead: over the vase, in 3D"), self.wait(1.0),
            self.act(lambda: tk.set_path_world([np.array(p) for p in ROUTE], "right"), "3D route"), self.wait(1.5),
            self.user("Carry it along my path."), self.wait(0.4),
            self.act(lambda: tk.pick_place("demo-t2", "mug", "along my path", None, None, 4 / 3), "carry"),
            self.wait(0.6), self.robot_says_task(), self.until_task_done(), self.robot_says_task(), self.wait(2.0),
            self.cap("Point, and say “there”"), self.wait(0.8),
            self.act(lambda: (tk.set_marker("P", np.array([0.33, -0.20, 0.74])),
                              self.utter("You're pointing at the right side. I marked it P.", "vision")), "point"),
            self.wait(2.6), self.user("Put the red block there."), self.wait(0.4),
            self.act(lambda: tk.pick_place("demo-t3", "red block", "there", None, None, 4 / 3), "deictic"),
            self.until_task_done(), self.robot_says_task(), self.wait(2.0),
            self.act(lambda: tk.home("demo", reset_scene=True), "reset"),
            self.cap("Teach it like a friend"), self.wait(0.8),
            self.act(lambda: (te.start(), self.utter("I'm watching. Show me and tell me what matters.", "robot")), "start teaching"),
            self.wait(2.6),
            self.user("Mug to the middle. Don't touch the vase. Then the red block on the right."),
            self.act(lambda: [te.utter(x) for x in ("Mug to the middle.", "Don't touch the vase.", "Then the red block on the right.")]),
            self.wait(4.5), self.act(lambda: te.stop(), "understand"), self.wait(0.4),
            self.act(lambda: self.utter(te.question, "robot")), self.wait(8.5),
            self.user("Yes, practice it."), self.wait(0.3),
            self.act(lambda: te.correct("yes"), "practice"), self.until_teach(("done", "confirm"), 60),
            self.act(lambda: self.utter("Practice passed. Both steps done in simulation, the vase untouched.", "robot")), self.wait(3.5),
        ]
        def er_line():
            e = self.er() or {}
            labels = [p["label"] for p in (e.get("points") or [])][:4]
            return ("Gemini Robotics-ER sees: " + ", ".join(labels) + ".") if labels else "I can see you and the scene in real time."
        keyfeatures = [
            self.act(lambda: tk.home("demo", reset_scene=True), "reset"),
            self.act(lambda: self.utter("Key feature one: video to robot. Show a video, and the robot performs it.", "narrator"), "kf1"),
            self.cap("① Video → Robot"), self.wait(2.0),
            self.act(lambda: self.push({"type": "source", "path": str(ROOT_VIDEOS / "dance_clip.mp4"), "name": "dance_clip", "seconds": 8}), "video"),
            self.wait(1.0), self.act(lambda: self.push({"type": "spatial", "target": "camera", "query": "Point to the people and their hands."}), "er"),
            self.until(lambda: not self.video_active(), 25),
            self.act(lambda: self.utter("Learned that dance from the video, whole body, checked for balance.", "robot")), self.wait(2.4),
            self.act(lambda: self.utter("Key feature two: real-time trajectory lining and pointing, with voice.", "narrator"), "kf2"),
            self.cap("② Trajectory lining + pointing, with voice"), self.wait(2.6),
            self.user("Move the mug to the left."), self.wait(1.2),
            self.act(lambda: tk.pick_place("demo-t1", "mug", "left", None, None, 4 / 3), "text command"),
            self.act(lambda: self.utter("I can't: the mug would hit the vase, thirteen centimeters too low.", "robot")), self.wait(3.2),
            self.act(lambda: tk.set_path_world([np.array(p) for p in ROUTE], "right"), "3D route"),
            self.user("Here's the path, over the vase. Carry it along my line."), self.wait(1.8),
            self.act(lambda: tk.pick_place("demo-t2", "mug", "along my path", None, None, 4 / 3), "carry"),
            self.act(lambda: self.utter("Every waypoint checks out. Carrying it along your line.", "robot")),
            self.until_task_done(), self.wait(0.5),
            self.act(lambda: (tk.set_marker("P", np.array([0.33, -0.20, 0.74])), self.utter("Put the red block there.", "user")), "point"),
            self.cap("Pointing: “there”"), self.wait(1.4),
            self.act(lambda: tk.pick_place("demo-t3", "red block", "there", None, None, 4 / 3), "deictic"),
            self.act(lambda: self.utter("You pointed at the right side. Placing the red block there.", "robot")),
            self.until_task_done(), self.wait(1.0),
            self.act(lambda: self.utter("Key feature three: real-time video understanding. Your camera is part of the prompt.", "narrator"), "kf3"),
            self.cap("③ Real-time video understanding"), self.wait(3.6),
            self.act(lambda: self.utter(er_line(), "robot")), self.wait(3.0),
        ]
        teach_short = teach[1:]
        return {"full": keyfeatures + teach_short + outro, "keyfeatures": keyfeatures + outro, "pitch": pitch + outro, "text_vs_spatial": text_vs_spatial + outro, "teach": teach + outro, "video": video + outro,
                "full_old": text_vs_spatial + video + teach + outro}.get(name)

    # ------------------------------------------------------------------ runtime
    def start(self, name):
        self.loop_name = "full" if name == "loop" else None
        if name == "loop":
            name = "full"
        steps = self.script(name)
        if steps is None:
            return
        if not self.steps:                        # the demo turns the mock brain on; what it was comes back afterwards
            self.mock0 = bool(getattr(self.brain, "enabled", False))
        self.brain.enabled = True
        self.steps, self.i, self.name, self.t_wait, self.cond = steps, 0, name, 0.0, None

    def stop(self):
        self._end()

    def _end(self):
        """Stopped or finished: nothing of the demo keeps running (the mock brain is back to what it was before)."""
        self.steps, self.name, self.loop_name = [], None, None
        if self.mock0 is not None:
            self.brain.enabled = self.mock0
            self.mock0 = None

    def tick(self):
        if not self.steps:
            return
        now = time.time()
        fresh = getattr(self.brain, "real_fresh", None)
        if fresh is not None and fresh():         # a real line (the agent, the robot, vision) was just said: the script
            return                                # waits, so its next line can't cover that one before it's seen and heard
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
                before = self.brain._sid
                a()
                if self.brain._sid != before:      # a line was spoken: give it its own frame so the next one can't overwrite it
                    self.t_wait = now + 0.35
                    return
            elif kind == "wait":
                self.t_wait = now + a
                return
            elif kind == "until":
                self.cond = (a, now + b)
                return
        if getattr(self, "loop_name", None):          # endless mode: start over
            self.start("loop")
            self.t_wait = time.time() + 2.0
            return
        self._end()

    def state(self):
        if not self.steps:
            return None
        titles = [s[2] for s in self.steps[: self.i] if s[0] == "do" and s[2]]
        return {"name": self.name, "step": self.i, "total": len(self.steps), "title": titles[-1] if titles else ""}
