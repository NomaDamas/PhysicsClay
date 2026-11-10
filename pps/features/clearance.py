"""v1 P0c: P2 trajectories are checked with the gripper's real size, not only its TCP point.

pps/features/traj.py checks every sample of a trajectory (reach, the surface, the objects, keep-out zones) with the
hand as a point. With physics the hand is a real gripper that reaches below its TCP (open fingertips from above, the
pads of a side gripper) over a footprint, so a path whose TCP clears the mug's top by 2 cm still drives the fingers
through it and knocks it over. This adds a check through traj.register_checker: the samples where the gripper itself
would hit an object (tasks.path_object_hits with tasks.hand_extent), worded like traj.py's own reasons ('my gripper
would hit the mug'), so they turn red, the plan card is blocked and the issue says where. Objects the path is meant
to reach are not obstacles: what a grip event closes on (traj.graspable), what the hand already holds, and what a key
point sits on (a skill working on a plate, stirring in a bowl). Drawn strokes are checked in tasks.set_path_world.

No-op without P2 (no pps/features/traj.py) or in the kinematic fallback (the hand is a point there)."""
import numpy as np

try:
    from . import traj as _traj
except ImportError:                     # a branch without P2 trajectories
    _traj = None


def _palm(rt, dense):
    if len(rt.hands) == 1:
        return rt.hands[0]
    p = dense["p"][0]
    return min(rt.hands, key=lambda h: float(np.linalg.norm(rt.data.mocap_pos[rt.hands_mid[h]] - p)))


def gripper_clearance(ctx, pts, qs, info):
    tasks, rt = ctx.tasks, ctx.rt
    if getattr(tasks, "plant", None) is None or not len(pts):
        return [None] * len(pts)
    dense = info["dense"]
    palm = _palm(rt, dense)
    keys = [dense["p"][k] for k in dense["key"]]
    meant = {o for o in tasks.holding.values() if o}
    for n in list(tasks.objs) + list(tasks.obstacles):            # what a key point sits on, or a grip closes on
        o = tasks.objs.get(n) or tasks.obstacles[n]
        c = tasks.mug_center(n) if n in tasks.objs else np.asarray(o["pos"], float)
        r, hh = (tasks._radius(n), o["half_h"]) if n in tasks.objs else (o["r"], o["h"])
        if any(np.hypot(k[0] - c[0], k[1] - c[1]) < r and k[2] < c[2] + hh + 0.06 for k in keys):
            meant.add(n)
    for _t, kind, val, pi in dense["events"]:
        if kind == "grip" and val >= 0.5 and pi < len(keys):
            for n in tasks.objs:
                if _traj.graspable(keys[pi], tasks.mug_center(n), tasks.objs[n]["r"], tasks.objs[n]["half_h"]):
                    meant.add(n)
    return tasks.path_hits(pts, palm, skip=tuple(meant))


if _traj is not None:
    _traj.register_checker(gripper_clearance)
