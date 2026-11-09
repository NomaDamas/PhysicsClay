"""Robot-independent intent: compile the same hand path for a different robot (UR5e arm) and report reachability."""
from pathlib import Path

import mink
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
UR5E = ROOT / "vendor" / "mink" / "examples" / "universal_robots_ur5e" / "ur5e.xml"
BASE = np.array([0.02, -0.0, 0.74])     # arm bolted to the near edge of the same table, where the G1 stands


def compile_ur5e(points_world, tol=0.03):
    if not UR5E.exists():
        return {"ok": False, "message": "UR5e model not found (vendor/mink missing)."}
    m = mujoco.MjModel.from_xml_path(str(UR5E))
    cfg = mink.Configuration(m)
    cfg.update_from_keyframe("home")
    task = mink.FrameTask("attachment_site", "site", position_cost=1.0, orientation_cost=0.0, lm_damping=1e-3)
    post = mink.PostureTask(m, cost=1e-3)
    post.set_target_from_configuration(cfg)
    lim = [mink.ConfigurationLimit(m)]
    sid = m.site("attachment_site").id
    dists = []
    for p in points_world:
        local = np.asarray(p, float) - BASE
        task.set_target(mink.SE3.from_translation(local))
        for _ in range(40):
            v = mink.solve_ik(cfg, [task, post], 0.05, "daqp", damping=1e-3, limits=lim)
            cfg.integrate_inplace(v, 0.05)
        dists.append(float(np.linalg.norm(cfg.data.site_xpos[sid] - local)))
    n, bad = len(dists), sum(1 for x in dists if x > tol)
    worst = max(dists) if dists else 0.0
    if n == 0:
        return {"ok": False, "message": "Nothing to compile yet. Run a task or record a motion first."}
    if bad == 0:
        msg = f"Same intent compiled for a UR5e arm: all {n} waypoints reachable (worst error {worst * 100:.1f} cm)."
    else:
        msg = f"Same intent on a UR5e arm: {bad} of {n} waypoints out of reach, worst by {worst * 100:.0f} cm."
    return {"ok": bad == 0, "message": msg, "n": n, "bad": bad}
