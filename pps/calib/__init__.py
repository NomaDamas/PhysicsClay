"""P6 real-to-sim calibration (docs/calibration.md, docs/V1.md P6).

The trajectory is the ground truth: a robot in a real home should reproduce what was planned (joints, the objects it
moves, the forces it feels). Homes differ (table friction, object weight, a worn joint, a slow network), so:

* `params`     the parameter vector (friction, mass, damping, servo stiffness, latency, ...), applied to a live plant;
* `randomize`  domain randomization over those parameters (P1 dataset generation imports it);
* `probes`     a short probe routine (swing, press, lift, push) run through a RobotDriver (pps/bridge);
* `log`        the rollout log format (.npz) and a reader for P1 LeRobot episodes;
* `sysid`      system identification: replay the logged commands in a headless copy of the world and fit the
               parameters with CMA-ES so the replay matches.

`scripts/calibrate.py --demo` proves it without hardware against a "hidden real" robot (the sim with secret
parameters); `{type: "calib", op: "probe" | "fit" | "apply" | "reset"}` runs it in the server (pps/features/calib.py).
"""
