# Skills from demonstration (v1 P4)

Some intent can't be said in words: *watch me weld this seam*, *wipe it like this*, *spot me*. PhysicsClay watches the
person (P3 `learn`), pulls out **how** they did it as a few parameters, and applies that style to **where** the scene
says it goes. The seam, the area, the container and the target always come from the scene's geometry, never from the
person's hand. *Geometry gives precision, the person gives style.*

```
watch learn ─stop─▶ recordings/demos/<name>.json ─▶ pps/skills.rank  (which kind? ask when two fit)
                                                  └▶ pps/skills.extract  (style parameters)  ─▶ skills/<name>.json
                                                         │
             scene geometry (seams, objects) ──▶ pps/skills.compile ─▶ P2 trajectory (source "skill")
                                                         │                   ─▶ traj propose: ghost + plan card
                                                         │                      "I'll weld the L seam: 5 mm zigzag at
                                                         │                       2 Hz, 10 mm/s, 10° push angle. Right?"
               "wider", "slower", "6 mm" ─▶ edit params ─┘ (recompile, re-propose)          "go" ─▶ run
```

## Kinds

| kind | parameters (style) | applied to (geometry) | compiles to |
|---|---|---|---|
| `weave_seam` | `travel_speed` m/s, `weave {kind: zigzag\|circle\|figure8, amp m, freq Hz}`, `travel_angle` ° (+ push, − drag), `work_angle` ° (90 = square to the plate), `standoff` m, `direction` | a seam polyline from the scene (`spec.seams`), a plate (its seam, else its long edge), or `[[x, y, z], …]` | trajectory: approach, arc on (hold 0.3 s), the seam with the weave as a P2 `pattern` across it, arc off (hold 0.4 s), retreat; torch angles as per-point `tilt` / `roll` |
| `wipe` | `force` N, `pattern` raster\|spiral, `passes`, `spacing` m, `speed` | an object's top face (its footprint), a region, or `[x0, y0, x1, y1]` | trajectory: strokes along the long side (or a spiral), force from the first contact to the release |
| `stir` | `radius`, `freq`, `depth`, `turns`, `direction` | a container (bowl, pot, cup …), radius capped inside its wall | trajectory in the container's frame (it follows the container) |
| `press` | `force` N, `hold_s` | an object's top, a marker or a point | trajectory: above → touch with `force` and `hold` → release; with P0 physics and no preview, P0's own `press` task |
| `spot` | `assist_when {bar_velocity_below m/s, for_s s}`, `max_force` N, `lift_cm`, `object` (the bar) | the person's hands (the bar is their midpoint) | the watch `assist` behaviour with these parameters, and P0 impedance (`force_limit = max_force`) when physics is on |
| `mirror_motion` | `clip` | — | `play_skill` of a recorded whole-body motion (the existing video → skill path); humanoids only |

Parameters are clamped to sane bounds (`pps/skills/core.py` `KINDS`). Skill files live in `skills/` (user data,
git-ignored; `PC_SKILLS_DIR` overrides it). Examples ship in `pps/skills/examples/` (a 4 mm MIG weave, a raster wipe, a
slow stir, a 10 N press, a bench spot); editing an example saves your own copy (`my_<name>`).

## Extraction (`pps/skills/extract.py`, `pps/human/periodic.py`)

Everything is in the robot's frame: the person faces the robot, so their motion is mirrored (`demo.mirror_vec`).
Speeds, widths and frequencies don't change under the mirror; directions (stir sense, lean side) do.

* **Weave.** The longest stretch of the demonstration that passes the weave gates (2 s windows, below) is analysed:
  1. *Travel*: a least-squares line `p(t) = p0 + v t` gives the travel speed and direction (a zero-mean weave doesn't
     bias it; the path's PCA axis is reported too). `r2` = how much of the motion along `v` the straight travel explains.
  2. *Lateral offset*: the residual's principal axis across the travel.
  3. *Frequency*: the peak of a Hann-windowed, zero-padded FFT of the lateral offset (0.5–6 Hz), refined by parabolic
     interpolation.
  4. *Width*: the offset is band-limited to the fundamental and the odd harmonics that stand out of the noise floor,
     interpolated 8× (the peaks of a fast weave fall between frames), split into cycles at the fundamental's zero
     crossings; `amp` is the median half peak-to-peak. When the 3rd harmonic says "triangle" and the frame rate cut
     the higher harmonics off, the missing part of the series is added back.
  5. *Shape*: what the hand does along the travel at f (in quadrature: circle) or at 2f (figure 8), else zigzag.
  6. *Smoothing*: co-presence smooths every joint with an EMA (α 0.5 per server tick), which narrows a fast weave by
     5–25 %. The demo records it (`smoothing: {ema, dt}`) and the extractor divides out the gain measured on the
     demo's own (uneven) frame times.
  7. *Torch angles*: from the palm orientation (a torch in a fist leaves it along the palm's y axis, tip down):
     travel angle = lean along the travel (+ push), work angle = 90 − lean to the left. When the hand doesn't point a
     torch down steadily (pose-only hands rarely do), the defaults (10° push, square) are used and the robot says so.
  8. *Standoff* can't be seen (the tip is beyond the hand): 15 mm, correctable ("closer").
* **Wipe.** Strokes run along whichever principal axis of the horizontal path turns back most often (2 cm hysteresis);
  the other axis is how far they step (`spacing`). A path that winds around its centre with a growing radius is a spiral.
* **Stir.** A Kasa circle fit of the horizontal path (radius, spread), turns and frequency from the unwrapped angle.
* **Press.** The hand comes down (≥ 3 cm) and stops in every direction for ≥ 0.5 s: `hold_s`.
* **Spot.** A lift (squat, press, curl) with reps: the bar is the hands' midpoint; "help below" = 25 % of the
  concentric bar speed (2–15 cm/s), for 0.5 s.

**Which kind?** `rank()` scores each kind from the activity, the held tool (torch, sponge, spoon, barbell…) and the
scene (seams, containers). When the best two are within 0.12 the robot asks with the candidates card ("Was that 1, a
weave along a seam or 2, wiping?"); the answer resumes the extraction.

**Accuracy** (`tests/test_skills.py`, sine weaves 4–15 mm at 1.2–4.5 Hz, 5–30 mm/s, 0.5–1 mm noise, 30 fps, three
seeds each): width, frequency and travel speed within 15 % (frequency and speed within ~1 %, width typically within
5 %). Through the live server (synthetic person → co-presence → person model → demo file): 5 mm at 2 Hz and 2 cm/s
came back as 4.4–4.7 mm, 2.00 Hz, 20.0 mm/s. A triangular weave whose 3rd harmonic is buried in noise above ~1 mm reads
up to 19 % narrow (its fundamental); a square-ish weave with long dwells at the edges is underestimated more.

### The weave false positive (real-camera finding, PR #40)

MediaPipe's world landmarks make a still, hanging hand jitter 1–3 cm at ~0.9 Hz, which the old rule (≥ 4 mm, ≥ 4
zero crossings) called a weave. A weave is now an oscillation about a path that **travels**, checked by gates on every
window (`pps/human/periodic.py WEAVE`): travel ≥ 3 mm/s, straightness r² ≥ 0.55, ≥ 1.2 cm travelled, 1–5 Hz, 2.5 mm–6 cm
wide, spectral peak ratio ≥ 0.3 and autocorrelation at one period ≥ 0.3. On synthetic jitter (`synth.frames("jitter")`,
quasi-periodic at 0.9 Hz, 1–3 cm, 12 seeds × amplitudes) the old rule said "weave" 12/12, the new one 0/12; the welder's
5 mm weave still reads as one. A wave at shoulder height (wrist ≥ shoulder − 8 cm, swinging in place) is now a wave, and
a forearm swinging across the body is no longer a curl.

## Messages and state

```
{type:"skill", op:"extract", demo?, kind?, name?, target?, propose?=true}  learn's stop does this by itself
{type:"skill", op:"apply", name, target?, overrides?, preview?=true, task_id?}
{type:"skill", op:"edit", name?, changes? {dotted.param: value | {mul} | {add}}, text? ("a bit wider")}
{type:"skill", op:"list" | "delete" (name) | "bead_clear" | "pack_demo"}
```
Every op answers in `state.skillbook.reply = {id, op, ok, message, skill?, notes?, plan?, question?, changed?}`.
`source: "agent"` keeps the robot from saying the sentence the agent will say.

```
state.skillbook = {skills: [{name, kind, label, describe, needs, ok (this robot can), fits (examples: this scene),
                              example, demo}],
                   current: {name, kind, describe, target, plan, phase: proposed|running|done|blocked|dropped},
                   reply, rev, seams: [{name, label, points}], pack: {id, label, examples, physics, demo} | null,
                   beads: [{id, seam, r, live, pts}], spot: {name, max_force, impedance} | null, bar: {held, pos} | null}
state.plan.skill = {name, kind, describe, label, target}   while a skill is proposed (the web shows quick corrections)
```
The trajectory carries `meta.skill` (name, kind, params, target; for welds the seam polyline, standoff and the arc span)
and `meta.work_on` (objects the tool may touch or enter: the P2 object check skips them). P2 points gained an optional
`roll` (degrees, lean about the travel direction: a weld's work angle).

**The bead.** While a `weave_seam` trajectory runs, the end effector site's actual position (the torch tip, so the
weave and any tracking error are in it) is appended to a bead whenever it is on the seam (within the weave width + 8 mm,
below standoff + 12 mm). The web draws a growing tube (hot at the tip), the robot view capsules. Beads belong to the
world they were made in. There is no arc or melt physics.

## Agent

Tools: `learn_skill_from_demo(demo?, kind?, name?, target?)`, `list_skills()`, `apply_skill(name, target?, overrides?)`,
`edit_skill_params(name?, text?, changes?)`, `load_pack(name)`. The context carries `learned_skills`, `skill_on_screen`,
`seams` and `demo_pack`; `base.md` teaches "confirm in one sentence, corrections edit parameters"; a loaded pack adds its
own prompt (`pps/world/packs/<id>.md`).

## Browser

`web/ui/skills.js`: the bead tubes and dashed seam guides in the 3D stage (layer `skills`), quick corrections under the
plan card (Wider · Narrower · Slower · Faster · More push · Closer …), the pack card (what to say, what needs physics,
"No camera? Play a demo"), and offline voice ("wider", "a bit slower", "6 mm", "now the L seam") when the agent is off.
The timeline's Skills strip lists parametric skills for every body that can run them; play proposes them on this scene.
The Scene tab of "Robot & scene" has the demo packs.

## What needs physics (verified at integration with P0)

Developed on the kinematic base. With P0: `press` uses P0's admittance `press` task (run without preview) or the
trajectory's `force` through `rt.set_force`; `wipe` force likewise; `spot` sends `{type:"physics", op:"set", params:
{impedance: true, force_limit: max_force}}` on start and `impedance: false` when assist ends, and the assist force
request stays in `state.watch.assist.request`; the gym barbell is a 20 kg free body (it only follows the lifter's hands
in kinematic mode); the torch is a collision capsule on the wrist (its last 3.5 cm are visual only).

## Limitations

* The torch angle rarely comes from a webcam (pose-only hands); defaults are used and said. Standoff is a default.
* No arc, melt or penetration model: the bead is the tip's path, not a weld.
* Wipe force and stir depth can't be seen: defaults, correctable by voice.
* Seams come from the scene spec (the weld cell lists them); plate edges are used for a single plate. No seam finding
  from images.
* Spotting a bench press is tested with a standing synthetic lifter (the synthetic person can't lie on the bench); a
  real bench press seen by a webcam is untested.
* Thresholds are tuned on synthetic people; the gates were designed against the real-camera jitter numbers in PR #40,
  not re-measured on footage here.
