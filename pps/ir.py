"""Physical Intent IR recorder.

The IR is robot-agnostic intent (where the hands should be, when, with what grip) plus the
constraints the compiler must respect.  trajectory.json is the robot-specific compiled motion,
report.json is the validator's verdict.  Same IR, different robot -> different trajectory.
"""
import json
import os
import time


class IntentRecorder:
    def __init__(self, robot_name, joint_names, mirror):
        self.robot = robot_name
        self.joint_names = list(joint_names)
        self.mirror = mirror
        self.frames = []
        self.t0 = None
        self.min_speed_scale = 1.0

    def start(self):
        self.frames = []
        self.t0 = time.time()
        self.min_speed_scale = 1.0

    def add(self, lm, targets, grip, qpos, flags, residuals, speed_scale, context=None):
        t = time.time() - self.t0
        self.min_speed_scale = min(self.min_speed_scale, speed_scale)
        self.frames.append({
            "t": round(t, 3),
            "human": lm,
            "targets": {k: [round(float(x), 4) for x in v] for k, v in targets.items()},
            "grip": grip,
            "context": context or {},
            "qpos": [round(float(x), 4) for x in qpos],
            "flags": dict(flags),
            "residuals": {k: round(float(v), 4) for k, v in residuals.items()},
        })

    @property
    def duration(self):
        return self.frames[-1]["t"] if self.frames else 0.0

    def intent_ir(self):
        keyframes, last = [], -1.0
        for f in self.frames:
            if f["t"] - last >= 0.1:
                keyframes.append({
                    "time_sec": f["t"],
                    "targets": [{"body_part": k, "reference_frame": "world", "position": v}
                                for k, v in f["targets"].items()],
                    "grip": f["grip"],
                    "holding": (f.get("context") or {}).get("holding", {}),
                })
                last = f["t"]
        return {
            "schema": "physical-intent-ir/0.1",
            "source": "human_demonstration_webcam",
            "robot_hint": self.robot,
            "mirror": self.mirror,
            "duration_sec": self.duration,
            "speed_scale": round(self.min_speed_scale, 3),
            "scene_objects": sorted({o for f in self.frames for o in (f.get("context") or {}).get("scene", [])}),
            "keyframes": keyframes,
            "constraints": {"feet_contact": True, "maintain_balance": "com_fixed",
                            "self_collision": "avoid", "joint_limits": True},
            "success": {"position_tolerance_m": 0.06},
        }

    def trajectory(self):
        return {"robot": self.robot, "joint_names": self.joint_names,
                "frames": [{"t": f["t"], "qpos": f["qpos"]} for f in self.frames]}

    def report(self):
        n = len(self.frames)
        counts, worst = {}, 0.0
        for f in self.frames:
            for v in f["flags"].values():
                counts[v] = counts.get(v, 0) + 1
            if f["residuals"]:
                worst = max(worst, max(f["residuals"].values()))
        feasible = sum(1 for f in self.frames if not f["flags"])
        frac = feasible / n if n else 0.0
        return {"frames": n, "duration_sec": self.duration, "feasible_fraction": round(frac, 3),
                "flag_frames": counts, "worst_residual_m": round(worst, 4),
                "speed_scale": round(self.min_speed_scale, 3),
                "verdict": "FEASIBLE" if n and frac > 0.9 else "NEEDS_CORRECTION"}

    def save(self, root="recordings"):
        d = os.path.join(root, time.strftime("%Y%m%d-%H%M%S"))
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "intent.json"), "w") as f:
            json.dump(self.intent_ir(), f, indent=1)
        with open(os.path.join(d, "trajectory.json"), "w") as f:
            json.dump(self.trajectory(), f)
        with open(os.path.join(d, "report.json"), "w") as f:
            json.dump(self.report(), f, indent=1)
        return d
