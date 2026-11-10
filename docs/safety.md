# Safety around people

> **This is not a certified safety system.** PhysicsClay is a simulator. The person is a pose estimate from a webcam,
> a phone or a headset, and that estimate can be centimetres off, lag behind, or miss you. Nothing here meets
> ISO 10218, ISO/TS 15066 or ISO 13849, and nothing here may be used as the safety function of a real robot. Real
> robots need certified sensors, a safety-rated controller, a risk assessment, and an emergency stop you can reach.

What it is: an honest model of **speed and separation monitoring** (SSM) and a small **contact policy**, so that the
way the robot behaves around people can be designed, tested and recorded in the simulator, and so that the agent and
the UI learn to treat safety as something that can't be argued with.

## What the robot does

* **It slows down** when any part of you comes within about half a metre of any part of it.
* **It stops** (a protective stop) before it would touch you. Near your head, neck or body it stops sooner.
* A running task **pauses** with a plain message ("Paused for safety: you're 9 cm from my gripper") and **goes on by
  itself** once you've stepped back and stayed clear for a second. The speed then ramps back up.
* While it is stopped it won't start anything new, and it won't plan a reach or a pick whose target is inside your
  body zone.
* In `assist` and `mirror` modes, and while handing you something, your **hands and forearms** may come right up to its gripper, at
  a crawl. **Touching** is allowed only when the end-effector force is measured (P0 physics) and stays under the cap.
  Without a force reading the robot stops at contact.
* It never touches your head, neck or body, in any mode.

The UI only shows this: a faint outline of your safety capsules in the 3D view (amber while the robot slows down, red
when it has stopped), and one calm line above the plan card and in the plan chip. There is no switch to turn it off,
in the app or in the config.

## How it works

**Geometry.** You are 17 capsules built from the 33 pose landmarks in the robot's world frame (`state.human.pose`):
head, neck, torso (spine, shoulder line, hip line), upper arms, forearms, hands, thighs, shins and feet. The robot is
one capsule per geom of its bodies: collision geoms where a body has them, otherwise the bounding box of its visual
meshes. This works for every robot profile (arms, two-arm rigs, humanoids, mobile bases, legged bodies) and costs
well under a millisecond per frame. The gripper, wrist and forearm links, and anything the robot holds, count as the
end effector. Distances are surface to surface.

**Separation.** For every robot link and body part the monitor computes the protective separation distance

```
S_p = v_h * (T_r + T_s)  +  v_r * T_r  +  v_r * T_s / 2  +  C
```

* `v_h`: your speed toward the robot (measured from the pose; moving away costs nothing), capped at 1.6 m/s;
* `v_r`: the speed of that robot link (measured);
* `T_r`: reaction time (0.1 s); `T_s`: braking time (0.2 s), so `v_r * T_s / 2` is the braking distance;
* `C`: the intrusion and uncertainty margin: `d_stop` = 0.12 m for the body, `d_stop_critical` = 0.15 m for the
  head, neck and torso, `d_touch` = 0 m for your hands and forearms against the end effector while cooperating.

That is the ISO/TS 15066 formula with the robot's braking modelled as a constant deceleration and the pose error
folded into `C`. The speed scale for one pair at distance `d` is

```
1                                              if d >= d_slow + (S_p - C)
min_scale + (1 - min_scale) * (d - S_p) / band  between              (band = d_slow + (S_p - C) - S_p)
0  (protective stop)                           if d <  S_p
```

with `d_slow` = 0.5 m and `min_scale` = 0.2. The slow band moves outward by exactly the distance the robot and you
could cover while it reacts and brakes, so a fast robot slows earlier. The most restrictive pair wins.

**No chatter.** Slowing down is immediate; speeding up is limited to 1.0 of scale per second. A stop is released only
when every pair is beyond its stop distance plus 5 cm (hysteresis) for a full second (`resume_s`).

**Why these numbers.** A webcam pose is good to a few centimetres when you face the camera and worse when you turn,
so 12 cm keeps a margin around the body. The head and torso get 15 cm because contact there is never acceptable.
Half a metre is about an arm's reach across a table, where a person starts to share the robot's space. The person
stands about 35 cm beyond the table's far edge in the default scene, so arms working on the table run at full speed
until you lean or reach in. At rest, every robot is outside the stop distance of a person standing there; most are
outside the slow band too, but the UR5e and xArm7 (about 42 cm away) and the Panda (its raised home pose is 24 cm from
your chest) start a little slowed until their hand moves down to the table.

## Contact policy

| Your part            | Robot part      | Mode                          | Force reading      | Result                                                  |
|----------------------|-----------------|-------------------------------|--------------------|---------------------------------------------------------|
| head, neck, torso    | any             | any                           | any                | stop at `S_p` with C = 15 cm (reason `critical`)        |
| upper arm, leg       | any             | any                           | any                | stop at `S_p` with C = 12 cm (`body`)                   |
| hand, forearm        | arm or body     | any                           | any                | stop at `S_p` with C = 12 cm (`body`)                   |
| hand, forearm        | end effector    | normal                        | any                | stop at `S_p` with C = 12 cm (`body`)                   |
| hand, forearm        | end effector    | assist, mirror or handover    | none               | at most `coop_scale` (0.3) near you; stop at contact (`no_force`) |
| hand, forearm        | end effector    | assist, mirror or handover    | under 40 N         | contact allowed, at most `coop_scale`                   |
| hand, forearm        | end effector    | assist, mirror or handover    | over 40 N          | stop, **latched** (`force`): confirm after you're clear |

The 40 N cap (`f_contact_max`) is well below the ISO/TS 15066 quasi-static limits for the hand (140 N) and forearm
(160 N). Separation stops resume by themselves; a force-limit stop needs `{type: "safety", op: "resume"}` (the
"I'm clear" button), and even then only once you are clear.

## Configuration

Server side only, never from the app:

1. defaults in `pps/safety.py` (`Config`);
2. the robot profile's `safety` dict (`pps/world/profiles.py`), for bodies that need more room;
3. a JSON file: `python -m pps.app --safety-config safety.json` (or `PC_SAFETY_CONFIG`).

Every value is clamped to a floor (for example `d_stop` ≥ 5 cm, `min_scale` ≥ 0.05, `f_contact_max` ≤ 140 N), and
unknown keys such as `"enabled": false` are ignored.

## For developers

* `pps/safety.py` is pure and unit tested (`tests/test_safety.py`: capsule maths, the SSM formula, the speed curve
  with hysteresis, the contact table, the config floors, robot capsules for every profile).
* `pps/features/safety.py` runs it every frame and sets `rt.safety_scale`; both runtimes scale their motion by it in
  `step()` and hold still at 0.
* Planners (P2 trajectories) ask `tasks.safety.check_point(p, role)` → `{ok, d, part, region, why}` for each point.
* P0 physics can provide the end-effector force as `rt.ee_force()` (or `ctx.store["physics"]["ee_force"]`):
  `{role: [fx, fy, fz]}`. Without it, cooperative contact is never allowed.
* `scripts/checks/safety.py` drives a synthetic person against the server: leaning in slows the arm, reaching into
  its path stops it and pauses the task, stepping back resumes it, a target in the torso is refused, assist lets the
  hands close but the head still stops it, and "disable" messages change nothing.

## Limitations

* **Not certified, not redundant, not real time.** One process, one pose estimate, roughly 20 to 30 checks a second.
* **The pose can be wrong.** Occlusion, a second person, loose clothing, someone out of frame: the robot only knows
  about one tracked person and only what the camera sees. If the camera loses you while you are close, the last pose
  is kept for 1.5 s, then the robot assumes you have gone.
* **Capsules are approximations.** Fingers, tools you hold and long hair are not modelled; robot mesh boxes are
  conservative in some places and loose in others.
* **The kinematic simulator stops instantly.** The braking term models a real robot's stopping distance; the sim
  itself halts in one tick. P0 physics makes this more realistic but not certified.
* **No power and force limiting yet** beyond the end-effector force cap: transient contact energy, pinch points
  between links, and pressure (N/cm²) are not computed.
* A handover now stops 10 cm short of your hand so the robot never has to touch you; you close the gap.
