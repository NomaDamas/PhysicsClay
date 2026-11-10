# Real-to-sim calibration (v1 · P6)

**The trajectory is the ground truth.** A robot in someone's home should do what was planned: the hand path, the joint
motion, *and* what happens to the objects it touches and the forces it feels (a perfect hand path can still slide a cup
on a slick table). Every home differs: the table's material, how heavy things are, a worn joint, a slow network link.
So before trusting the simulator for a site, we make the simulator match the robot there:

1. **Calibrate the simulator to the robot** (system identification): run a short probe routine, record what the robot
   did, and fit the plant parameters (friction, object mass, joint damping, servo stiffness, latency) so a replay of
   the same commands in simulation reproduces the recording.
2. **Train with domain randomization** over those parameters (P1 dataset generation), narrowed around the site's
   calibrated profile when there is one.
3. **Adapt per site** by re-running the probes: once per home, and again when something changes.

No hardware is claimed. The proof is a *hidden real* robot: the simulator with secret parameters and noisy sensors,
which the calibration only sees through its probe logs (below). Everything that commands a robot goes through a
`RobotDriver` (`pps/bridge/`), so the same probes and fit run on hardware once a driver exists.

## Files

| file | what |
|---|---|
| `pps/calib/params.py` | the parameter vector: defaults, search ranges, apply / restore on a live plant or a bare MjModel |
| `pps/calib/randomize.py` | domain randomization (seeded, uniform / log-uniform), lighting / texture hooks; shared with P1 |
| `pps/calib/probes.py` | the probe routine (swing, press, lift, slide), its runner, a task recorder, a headless robot helper |
| `pps/calib/log.py` | the rollout log (`.npz`) and a reader for P1 LeRobot episodes |
| `pps/calib/sysid.py` | headless replay (`PlantSim`), the objective, CMA-ES, the staged fit, held-out tracking |
| `pps/bridge/` | `RobotDriver` protocol, `SimDriver` (the plant), `LeRobotDriver` / `ROS2Driver` stubs |
| `pps/features/calib.py`, `web/ui/calib.js` | the server feature (`{type: "calib"}`) and its panel |
| `pps/agent.py` | agent tool `calibrate_here` |
| `scripts/calibrate.py` | `--demo` (hidden-real proof), `--probe`, `--fit` |
| `tests/test_calib.py`, `scripts/checks/calib.py` | headless tests (~5 s) and the server check |

## The parameters

All are P0 plant parameters (`pps/physics.py`), so a fitted or sampled vector is applied to a running world without
rebuilding it, exactly like `{type: "physics", op: "set", params}`.

| name | what it scales | default | search range | space |
|---|---|---|---|---|
| `friction_scale` | sliding friction of every geom (object–surface, finger pads) | 1 | 0.4 – 2.0 | log |
| `mass_scale` | mass and inertia of the movable objects | 1 | 0.4 – 2.5 | log |
| `damping_scale` | viscous damping of the joints | 1 | 0.4 – 2.5 | log |
| `kp_scale` | position-servo stiffness (the servo's damping scales with √kp) | 1 | 0.5 – 2.0 | log |
| `latency_ms` | delay between a command and the servos acting on it | 0 | 0 – 80 | linear |
| `armature_scale` (optional) | rotor inertia reflected at the robot's joints | 1 | 0.5 – 2.0 | log |
| `frictionloss_scale` (optional) | Coulomb friction of the robot's joints | 1 | 0.2 – 3.0 | log |

A scale parameter only exists on a robot whose model has something to scale (`params.active(plant)`): the Panda's
joints have damping and armature but no Coulomb friction, the UR5e's have no damping, a scene without movable objects
has no mass or object friction to fit. The default fit set is the five core parameters that act on the plant.

**Latency** is now a real delay buffer in the plant: the servo setpoint is stored every 2 ms substep and read back
`latency_ms` later, linearly interpolated between substeps, so the delay is continuous (P0 delayed whole server ticks,
which quantized it to ~8 ms and could not be fitted).

## The probe routine

| probe | what the robot does | excites | robot time |
|---|---|---|---|
| **swing** | three brisk joint moves (±0.3 rad on the base, shoulder and elbow joints, trapezoid 1.5 rad/s, 10 rad/s²), each held 0.35 s | servo stiffness, damping, latency (the transient and its timing) | ~4.5 s |
| **press** | closed jaws over a free spot of the table, a guarded move down until the wrist feels 1.5 N, then on at 3 mm/s to 8 N, hold 1 s | static stiffness under load (force vs commanded depth) | ~10 s |
| **lift** | grasp an object (the task layer's own grasp planner), lift 10 cm, hold still 1.2 s, put it back | object mass (the wrist feels the weight) | ~14 s |
| **slide** | grasp an object, set it down until the table carries its weight + 3 N, slide it 10 cm, let go | friction (the wrist feels μ·N horizontally and N vertically) | ~15 s |

Each probe is planned in the current world from what the robot sees: it picks a free spot, a graspable object with
room to slide, a joint that can swing clear of the table, and checks every hand path with the planner (IK pre-check)
first; a probe that can't run here is skipped with a reason. Forces stay small: the press is force-limited by closed
loop on the measured wrist force (it stops loading at 8 N), the slide presses with ~3 N.

*Why "slide while holding" instead of a free push:* pushing a light block with open contact (jaws closed, block free)
jammed it into the table or tipped it in this contact model (elliptic cones, stiff pads, 2 ms steps), sometimes until
the simulation diverged. Holding the object while sliding it keeps the contact well-posed and gives the friction
coefficient directly from the wrist force.

## Log format

One log per probe (or per task), one row per control tick: the state measured at `t[k]` and the command sent right
after reading it. `.npz` keys (N rows, nq robot qpos entries, nv robot dofs, R gripper roles, O objects):

| key | shape | meaning |
|---|---|---|
| `t` | (N,) | seconds on the robot's clock; command k is held until `t[k+1]` |
| `q_des` | (N, nq) | commanded joint positions (the action) |
| `grip` | (N, R) | gripper command per role, 0 open … 1 close |
| `q`, `dq` | (N, nq), (N, nv) | measured joints (NaN when not measured) |
| `ee_force` | (N, R, 3) | wrist force on the hand, world frame, N (NaN: no force sensor) |
| `obj_p`, `obj_q` | (N, O, 3), (N, O, 4) | object positions / orientations (NaN: not tracked) |
| `base` | (N, 7) | planned pose of a welded floating base (humanoids), else NaN |
| `seg` | (N,) | probe segment index of the row (−1: unknown) |
| `meta` | JSON | `robot`, `scene`, `probe`, `roles`, `objects`, `joints`, `source` |

`log.from_lerobot(path, nq)` reads a P1 LeRobot episode (`observation.state`, `action`, `timestamp`, optional
`observation.ee_force`, `observation.objects`). A dataset without object poses or forces still calibrates the joint
dynamics (stiffness, damping, latency); mass and friction need the force or the object channel.

## System identification

`PlantSim` builds a headless copy of the world (the same `scene.build` + `Plant`, no server, no IK) and replays a log:
it starts from the measured state of the log's first 0.2 s (robot at rest, objects where they were seen, then 0.15 s
to settle), and feeds the logged commands with exactly the logged substep counts. With the true parameters the replay
reproduces the recording to the sensor noise floor (tested with variable server-like tick lengths too).

**Objective.** For a parameter vector θ and logs ℓ:

    J(θ) = Σ_ℓ [ w_q · mean_k ‖q_sim(k) − q(k)‖² / σ_q²
               + w_p · mean_k ‖p_sim(k) − p(k)‖² / σ_p²
               + w_f · Σ_s (n_s / n) ‖f̄_sim(s) − f̄(s)‖² / σ_f² ]

q are the robot's joint positions, p the object positions, f the wrist force; σ_q = 0.01 rad, σ_p = 0.01 m,
σ_f = 1 N put the channels on comparable scales, w = 1. The force is compared as its **mean over each probe segment**
s (the hold, the press, the slide…; 0.5 s windows for logs without segments), weighted by the segment's length n_s:
contact forces chatter (stick-slip) differently in two equally good simulations, their averages don't. With per-sample
forces the friction minimum was buried in that chatter; with segment means it is sharp. A 1-D scan of the force term on
a slide log of a hidden robot with friction 0.7:

| friction_scale | 0.5 | 0.6 | 0.65 | **0.7** | 0.75 | 0.8 | 1.0 |
|---|---|---|---|---|---|---|---|
| per-sample force (0.1 s smoothing) | 0.18 | **0.13** | 0.14 | 0.16 | 0.16 | 0.15 | 0.25 |
| segment means (used) | 0.032 | 0.0072 | 0.0019 | **0.0014** | 0.0028 | 0.0076 | 0.088 |

The same scan of the joint term over the swing and press logs (hidden kp 0.8, latency 30 ms) is sharp on its own:
J = 4.9 / 1.1 / **0.18** / 0.86 / 2.5 for kp 0.6 / 0.7 / **0.8** / 0.9 / 1.0, and 8.2 / 2.2 / **0.18** / 2.2 / 8.3 for
latency 0 / 15 / **30** / 45 / 60 ms. Missing channels (NaN) drop out.

**Optimizer.** CMA-ES — `cma` (pycma) when it is installed, otherwise the small (μ/μ_w, λ)-CMA-ES in `sysid.py`
(Hansen's tutorial version, box-constrained by clipping plus a quadratic penalty) — in a normalized space (log space for
the scale parameters). The fit runs in **stages** that follow what each probe excites, each later stage keeping the
earlier results fixed:

| stage | parameters | logs |
|---|---|---|
| dynamics | kp, damping, latency (+ armature, joint friction if asked) | swing, press |
| mass | mass_scale | lift |
| friction | friction_scale | slide |

Generations per stage: `quick` 10, `normal` 18, `thorough` 30, ×(1 + (d−1)/2) for a d-parameter stage (the 3-D dynamics
stage gets twice as many); population 4 + ⌊3 ln d⌋ (6 for one parameter); start at the defaults with σ = 0.3. A joint
"polish" of all parameters on all logs is only in `thorough`: in a trial it moved a well-fitted mass by 10 %, because in
a joint fit the weak channels (mass, friction) are outvoted by the joints' noise floor. Rollouts run in at most **2 worker processes** (spawned, one headless
world each); the server waits on them from a background thread so the robot loop keeps its frame rate.

## Proof without hardware: the hidden-real demo

    .venv/bin/python scripts/calibrate.py --demo --seeds 5

Per seed: the hidden robot is a Panda on a table with a block and a mug whose plant parameters are drawn with
`Randomizer(seed)` from friction 0.5–1.6, mass 0.5–2.0, damping 0.6–1.7, stiffness 0.6–1.5, latency 0–60 ms; its
sensors are noisy (joints σ 0.5 mrad, wrist force σ 0.1 N, object tracking σ 1.5 mm). The probes run on it; a nominal
simulator (all defaults) is calibrated from the four probe logs only. Then a **held-out task** the probes never did —
the task layer's own pick & place of the mug to the centre of the table — runs on the hidden robot, and its commands are
replayed open loop in the nominal and in the calibrated simulator. Tolerance: ±15 % for the scale parameters, ±10 ms
for latency.

Per seed (true → fitted):

| seed | friction_scale | mass_scale | damping_scale | kp_scale | latency_ms | held-out joints (°) nominal → calibrated | hand path (mm) | object path (mm) |
|---|---|---|---|---|---|---|---|---|
| 0 | 1.05 → 1.06 | 0.727 → 0.725 | 0.626 → 0.626 | 0.609 → 0.61 | 48.8 → 48.9 | 2.90 → 0.10 | 10.7 → 0.9 | 5.2 → 2.6 |
| 1 | 0.907 → 0.898 | 1.87 → 1.85 | 0.697 → 0.702 | 1.43 → 1.43 | 18.7 → 18.5 ¹ | 0.61 → 0.16 | 4.4 → 1.0 | 116.4 → 113.9 |
| 2 | 0.678 → 0.683 | 0.756 → 0.75 | 1.4 → 1.39 | 0.653 → 0.646 | 36 → 35.9 | 4.22 → 0.09 | 22.7 → 0.7 | 7.2 → 2.7 |
| 3 | 0.552 → 0.548 | 0.694 → 0.68 | 1.38 → 1.36 | 1.02 → 1.01 | 5.65 → 5.58 | 1.06 → 0.09 | 5.8 → 0.7 | 4.0 → 2.6 |
| 4 | 1.5 → 1.49 | 1.02 → 1.02 | 1.66 → 1.64 | 0.646 → 0.631 | 36.4 → 35.2 | 4.96 → 0.10 | 28.0 → 0.8 | 8.1 → 2.6 |

Median over seeds:

| parameter | median error | worst | within tolerance |
|---|---|---|---|
| friction_scale | 0.9 % | 1.3 % | 5/5 (±15 %) |
| mass_scale | 0.7 % | 2.0 % | 5/5 (±15 %) |
| damping_scale | 0.9 % | 1.3 % | 5/5 (±15 %) |
| kp_scale | 1.0 % | 2.4 % | 5/5 (±15 %) |
| latency_ms | 0.1 ms | 1.2 ms | 5/5 (±10 ms) |

Held-out pick & place on the hidden robot, replayed open loop (median over seeds):

| simulator | joints RMS (°) | hand path RMS (mm) | object path RMS (mm) | object final (mm) | wrist force RMS (N) |
|---|---|---|---|---|---|
| nominal (before) | 2.905 | 10.73 | 7.21 | 3.74 | 0.789 |
| calibrated (after) | 0.099 | 0.80 | 2.64 | 0.57 | 0.039 |
| true parameters (noise floor) | 0.091 | 0.69 | 2.64 | 0.61 | 0.037 |

Runtime per seed (median): probes 3 s (44 s of robot time), fit 222 s (453 rollouts, 2 processes), total 227 s.
Held-out task outcome on the hidden robot: done, failed, done, done, done.

¹ Seed 1: the hidden robot's own simulation blew up 3.8 s into the held-out pick & place (the heavy mug slipped out
of the jaws and MuJoCo diverged, then reset), so its held-out log is cut there and its object path is meaningless for
every simulator, the true one included. Its parameters were still recovered from the probes.

What it means:

* **Parameters**: every parameter of every seed is within 2.4 % (latency within 1.2 ms). The targets were ±15 % and
  ±10 ms, so they are met with a wide margin. An earlier run of the same demo (same seeds, before the last refinements)
  gave the same picture: medians ≤ 1 %, worst 3.9 %.
* **The held-out task**: with the nominal simulator, replaying the pick & place commands misses the real robot's
  joints by 2.9° RMS, the hand by 11 mm and the wrist force by 0.8 N. With the calibrated simulator the errors drop to
  0.10°, 0.8 mm and 0.04 N. That is essentially the floor set by the sensor noise, which is what replaying with the
  true parameters gives (0.09°, 0.7 mm, 0.04 N). The object path error (2.6 mm) is the 1.5 mm tracking noise in 3D.
* **Runtime** (Apple-silicon laptop, other jobs running): about 45 s of robot time for the probes. Fitting takes
  about 3–4 min per seed at the `normal` budget (453 rollouts in 2 processes). The `quick` budget the server uses takes
  about 2 min and lands within a few percent (server check below).
* **Honesty**: the hidden robot differs from the nominal model only in the five parameters being fitted. The demo
  shows that the probes and the fit can identify them through noise, not that a real robot would match this well
  (see the limitations).

Reproduce: `scripts/calibrate.py --demo --seeds 5 --save-logs` (add `--retrack <json>` to recompute the held-out
numbers from the saved logs, `--report <json>` to print the tables again).

## In the server

`{type: "calib", op: "probe" | "fit" | "apply" | "reset" | "run" | "stop"}` (`docs/PROTOCOL.md` v7). `probe` runs the
routine on the robot as a task (the plan card shows "Calibrating", the step is the probe running; other tasks are
refused until it is done, `stop` ends it); the probe runner drives the plant through a `SimDriver`, taking over the
runtime's plant step while it runs (the IK keeps planning). `fit` replays the logs headlessly in two worker processes
(`quick` budget by default: about two minutes on a laptop next to the running server); `apply` writes the fit
into the plant (`state.physics.params`) and saves the profile to `recordings/calib/<robot>.json`; `reset` restores the
defaults. `state.calib` carries the status, each probe's progress, the fit progress, the fitted values against the
defaults (`per_param`) and the error before / after.

The panel (`web/ui/calib.js`, side slot) is one line with a "Calibrate here" button while idle; while probing it shows
the probes as chips, while fitting a thin progress bar, and after the fit a table of the values against the defaults
(a dot on the parameter's range, the tick at the default) and the fit error before → after. Saying or typing
"calibrate" runs it; with the LLM agent on, the agent's `calibrate_here` tool does ("this table is slippery,
recalibrate" → probe + fit + apply → "Calibrated: friction ×0.60 (slippery), mass ×1.50, damping as modelled,
stiffness ×1.33, latency 1 ms; joint tracking error X° → Y°." with the probe logs' worst joint error before / after).

The server check (`scripts/checks/calib.py`) gives the robot "today's" physics (friction 0.6, stiffness 1.3, mass 1.5),
probes it, resets the twin to nominal, fits from the logs alone (quick budget), and checks the fit recovers the three
within 20 %, that `apply` moves `state.physics.params` to the fit and `reset` restores the defaults. In the full self
test it recovered friction 0.60, stiffness 1.33, mass 1.50 (damping 1.01, latency 1 ms; truth 1.0 and 0) with a
120 s fit while the server kept running.

## Domain randomization (shared with P1)

```python
from pps.calib.randomize import Randomizer
rnd = Randomizer(seed=7)                    # core five, the search ranges above (per-robot overrides in params.ROBOTS)
p = rnd.sample()                            # {"friction_scale": 0.83, "mass_scale": 1.41, "damping_scale": ..., ...}
ws.send(rnd.message(p))                     # {"type": "physics", "op": "set", "params": p}   (scripts/generate.py)
rnd.apply(plant, p)                         # or straight onto a headless plant
site = rnd.around(profile["params"], rel=0.2, abs_ms=10)   # "this home ± 20 %" instead of every home
rnd.on("lighting", fn).on("texture", fn)    # rendering hooks (P1 owns rendering): draws come from the same generator
```

Scale parameters are drawn log-uniformly (×0.5 and ×2 are equally likely), latency uniformly; the same seed gives the
same sequence. `rnd.ranges()` describes the ranges for a dataset card.

## How a real deployment would use it

1. Write the robot's `RobotDriver` (joint order of the profile's planner model, `read_state` with the wrist force if
   the arm has a sensor or a torque estimate, object poses from perception) and check it with `pps.bridge.conforms`.
2. **Once per home** (and whenever the table, the objects or the robot change): clear a spot on the table, put one
   graspable object down, run the probes (about 45 s of robot time) and the fit (one to three minutes). Keep the
   result as the site's profile (`recordings/calib/<robot>.json`: robot, scene, parameters, fit report).
3. Plan and check tasks in the calibrated simulator; generate training data with randomization narrowed around the
   profile (`Randomizer.around`), plus the broad ranges for robustness.
4. Re-probe when tracking drifts (the replay error of ordinary tasks, `sysid.tracking`, is a cheap monitor: log a task
   with `TaskRecorder` and compare it with its replay).

## What is not addressed (honestly)

* **No hardware was used.** The hidden robot is the same simulator with other parameters, so the model class is exact
  (no unmodelled dynamics, no backlash, no cable forces, no sensor bias); a real robot leaves a larger residual and the
  recovered parameters absorb some of the mismatch. The demo's sensor noise is simple Gaussian noise.
* **Temperature and humidity are not modelled explicitly.** They show up as shifts in friction (a humid table, a cold
  rubber pad), damping (cold grease) and latency (a busy controller); re-probing picks them up, and randomization covers
  them in training.
* **One friction scale and one mass scale for the whole scene**: per-surface and per-object values need one probe per
  surface / object (lift each object to weigh it). The pads' friction scales with the table's.
* **Contact stiffness** (solref / solimp) is not a parameter; the press calibrates servo stiffness under load with the
  contact model as is.
* **Free pushes** are not used (see the probe table); the slide needs a gripper and a graspable object, the lift too.
  Without them only swing and press run (stiffness, damping, latency).
* **Latency** is a pure delay; jitter is not modelled. Damping and stiffness trade off a little in the swing (the press
  breaks the tie, since it is static).
* **Robots.** Recovery was evaluated on the Panda (the demo and the server check). Headless, the UR5e and the G1 run all
  four probes and replay them exactly (the G1 raises its hand first to swing clear of the table); ALOHA skips the swing
  (its arms start too low). The G1's slide replays less exactly (0.5° with identical parameters: its two-pad grasp
  is chaotic), so expect looser fits on humanoids. Mobile bases were not tried.
* **The contact model can blow up.** When a grasped object slips out of the jaws (heavy mug, low friction), MuJoCo
  occasionally diverges and resets; a replay that diverges is scored up to that point, and a recording whose clock
  jumps back (the simulated "robot" crashed) is cut there. Both are reported, not hidden.
