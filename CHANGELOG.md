# Changelog

All notable changes to PhysicsClay. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versions follow [Semantic Versioning](https://semver.org/). PR numbers refer to
[NomaDamas/PhysicsClay](https://github.com/NomaDamas/PhysicsClay/pulls). The design and the measured results behind each
area are in [docs/V1.md](docs/V1.md).

## [1.0.0] - 2026-11-10

The first open-source release. The hackathon build proved the interaction model (say, point and show; the robot
proposes, you correct). v1 makes it physical, makes the robot watch the person, makes every task a dataset, and
packages it so people can install, extend and trust it.

### Physics
- Real dynamics are the default: mink IK plans, a MuJoCo plant steps at 2 ms and tracks the plan with position servos
  (gravity compensation, effort limits, velocity feed-forward). `PC_PHYSICS=0` / `--kinematic` keeps the old
  kinematic mode (#42).
- Objects are free bodies with mass, inertia and friction; grasps hold by contact only. Slips ("It slipped"), too-heavy
  loads and placements are measured and reported honestly, with the placement error in centimetres (#42, #51).
- Compliance: impedance mode, a force-limited `press` task and an admittance loop along the approach axis (#42).
- `state.physics` (mode, rtf, grasps, wrist forces, contacts, objects) and `{type: "physics"}` to change friction,
  mass, damping, stiffness, latency and gravity at run time (#42).
- One clock: when the plant caps its substeps on a busy machine, the planner slows with it (`rtf < 1`) (#42).

### Grasping on every robot
- Reliable contact pick & place on ten bodies, measured by `scripts/bench_grasp.py` (10 trials per task, objects
  nudged each trial): G1, Panda, UR5e, xArm7, iiwa14, ALOHA, Apollo, Stretch 3, TidyBot and Spot all at 9–10/10;
  the Unitree H2 stays experimental (its hands aren't driven) (#51, #54).
- Built grippers where a model has none (parallel jaws on UR5e and iiwa14, two-pad palm grippers on humanoids);
  Stretch 3's spring-loaded pads, Spot's claw and TidyBot's Robotiq 2F-85 made to grasp for real (#51, #54).
- The planner keeps the whole arm clear of objects, tries several ways in, re-aims at where the object really is,
  searches in slices so refusals never stall the loop, and lifts the hand over objects on the way (#51, #54).
- ALOHA arm-to-arm handover of long objects (#51).
- Drawn paths respect objects: lifted over a movable object, red with a reason into a fixed one (#54).

### Datasets
- LeRobot v2.1 recording from the app (**Record**): joint states, actions (`q_des`), front and wrist video at 30 Hz,
  the user's words as the task, success labels, the person's pose and wrist forces (#44).
- `scripts/generate.py`: one intent replayed as many randomized episodes across robots, scenes, generated rooms,
  lighting, cameras, physics parameters and real objects from the photo library (#44, #47, #52, #53).

### Trajectories and VR
- One trajectory object for drawn, spoken, demonstrated, keyframed and VR paths: height, wrist orientation, speed,
  force, grip and tolerance per point, object-relative frames, weave patterns; checked per point (#43).
- Desktop authoring: ghost preview, draggable handles, height and wrist gizmos, keyframes, voice edits ("higher
  there") through the agent's `make_trajectory` / `edit_trajectory` (#43).
- VR / MR on Meta Quest (WebXR): draw in the air, move handles by hand, two-corner passthrough alignment, room scan to
  scene, live teleop of the robot's hand; tested with a mocked `navigator.xr` (#45).

### Watching the person
- The person in the robot's world: 33-point body, hands (pinch, open), held tool, activity, phase, reps, speed,
  fatigue, optional VLM summary (#39).
- Watch modes: mirror, learn, respond (standing rules, also by voice), assist and coach; agent tools to set them (#39).
- Real-camera end-to-end check with recorded clips as a fake webcam (#40).

### Skills and demo packs
- Parametric skills from a demonstration: `weave_seam`, `wipe`, `stir`, `press`, `spot`, `mirror_motion`, applied to
  the scene's geometry and corrected by voice ("a bit wider") (#46).
- Demo packs: weld cell (torch TCP, seams, a growing bead), gym (coach and spot with a 20 kg barbell), workshop helper
  (hand me the wrench) (#46).

### Safety around people
- Speed and separation monitoring between the robot's links and the person's body capsules: slow down, protective
  stop, automatic resume; contact allowed only on hands and forearms in assist and handovers, force-capped; cannot be
  turned off. Not a certified safety system (#41).

### Calibration
- Probe routine (swing, press, lift, slide), system identification (CMA-ES) of friction, mass, damping, stiffness and
  latency, domain randomization shared with datasets, and a `RobotDriver` interface with a sim driver and LeRobot /
  ROS 2 stubs; proven against a hidden simulated robot (#47).

### Worlds
- Ten generated rooms, every seed a different layout: kitchen, dining room, living room, laundry, assembly line with a
  running conveyor, warehouse, CNC machine tending, electronics bench, lab, packing station (#52).
- Drawers, doors and lids on real joints, opened by their handles in impedance mode; `open`, `close`, `pick_from`,
  `place_in` and multi-step `sequence` plan cards; jammed drawers reported as stuck (#52).

### Real photos → simulation
- A photo of an object becomes a graspable, physics-ready object with metric size, mass and friction (primitive fits
  offline; TripoSR, fal.ai TRELLIS or Meshy optional); a photo of a table becomes the scene's layout; an object
  library for scenes and datasets (#53).

### User experience
- Smooth proportional zoom, a usable gestures guide, natural voice per language (OpenAI TTS or the best system voice),
  a gear menu with System / Light / Dark themes, a camera and phone camera that show you (QR code to `/phone`), an
  in-app guide in English and Korean, and a design pass after the unslop method (#49).
- The co-presence mannequin no longer hides the work area in the default 3D view, and the skills strip is compact on
  phones (this release).

### Languages
- Server messages (tasks, plans, safety, watch, skills, worlds, recon, calibration, the mock brain…) in English and
  Korean through `pps/i18n.py` and a catalog of about 1,800 messages (`pps/locale/`), following Settings → Language
  (System / 한국어 / English); the UI's own labels follow it too (`web/ui/i18n-ko.js`, about 800 strings); the
  offline parser understands Korean commands such as "빨간 컵을 그릇 옆으로 옮겨줘" (this release).

### Running
- `make run` serves http on 8780 and https on 8781 by default (self-signed certificate), so phones and Quest work
  without extra flags, and prints the phone address (this release).

### Open source
- Apache-2.0 license and a NOTICE with every third-party model, asset, library and service; `pyproject.toml` with
  extras (`vision`, `data`, `recon`, `dev`, opt-in AGPL `yolo`); Makefile; ruff; a pytest unit suite; CI (lint, unit
  tests, the offline self test on headless Linux, gitleaks); contributing guide, code of conduct, security policy,
  issue and PR templates; architecture docs (#38).
- Documentation: a rewritten README in English and Korean, a docs index, this changelog, a link checker for the docs,
  and a v1 status table in `docs/V1.md` (this release).

## [0.x] - Hackathon build (2026-11-09)

Built for YC RFS Fall 2026, *New Operating System for the Physical World*: "a new generation of interactive prompt for
Physical AI".

- Video → robot motion: a person's video retargeted to the Unitree G1's whole body, balance-checked, saved as a skill.
- Routes drawn in 3D and pointing with real-time voice (Gemini Live, Robotics-ER); physics-checked refusals with the
  nearest alternative; teach by showing; cross-robot compile to a UR5e.
- UX 2.0 (#16 with #1, #9–#15): the robot proposes and you correct (plan card and ghost), "which one?" candidates,
  keep-out zones, goal from a photo or a 3D arrangement, the teach timeline, gestures and the nudge pad.
- UX 3 (#26 with #22–#25): many robot bodies and scenes built on the fly, an LLM agent brain, per-robot prompts and UI,
  and co-presence (the person in the robot's world).

[1.0.0]: https://github.com/NomaDamas/PhysicsClay/compare/main...v1/integration
