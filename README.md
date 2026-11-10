# PhysicsClay

**Instruct robots the way you instruct people: say it, point at it, show it, draw it.
The robot watches you, proposes a plan, and acts with real physics.**

English · [한국어](README.ko.md) · [Docs](docs/README.md) · [Changelog](CHANGELOG.md)

PhysicsClay is an open-source web app for giving tasks to simulated robots. You talk to it, point at the table, show
it a motion on camera, or draw a route in 3D (or in VR). It turns that into a concrete plan for the robot body you
picked, checks every waypoint against a MuJoCo model, shows you the plan and a ghost of the motion, and runs it only
when you say "go". It runs locally in your browser with a Python server (MuJoCo + [mink](https://github.com/kevinzakka/mink)
+ aiohttp + three.js), works without any API keys, and speaks English and Korean.

The core idea:

* **Say · point · show.** Words carry *what* and *why*; a point carries *where*; a demonstration carries *how*.
  Text alone can't say "exactly there, around the vase, slowly", and some motions can't be described at all.
* **The robot proposes, you correct.** Nothing moves before you see the plan. If something is wrong you fix only that
  part: point at a new spot, drag a waypoint, say "higher there". If the robot can't do it, it says why ("13 cm too
  low") and offers the nearest thing it can do.
* **Real physics.** Robots are simulated dynamically at 2 ms: grasps hold by contact, objects have mass and friction,
  slips and too-heavy loads fail honestly. Nothing is teleported into the hand.
* **It watches you.** Your body, hands and the tool you hold are in the robot's world. It can mirror you, learn a
  demonstration, follow standing rules ("when I hold my hand out, give me the wrench"), assist, or coach, and it slows
  down and stops around you.
* **A data engine for robot learning.** Every task can be recorded as a LeRobot dataset episode, and one intent can be
  replayed as many randomized episodes across robots, rooms and physics parameters.

## Screenshots

These are from earlier builds (UX 2.0 and the hackathon); the layout is the same today.

| The robot proposes: plan card + ghost | "Which one?" when your words fit several objects |
|---|---|
| ![Plan card with a proposed pick and place](docs/ux2/ux2-propose.png) | ![The robot asks which block](docs/ux2/ux2-which-one.png) |
| **Keep-out zone: "stay away from the vase"** | **Phone layout** |
| ![A keep-out zone around the vase](docs/ux2/ux2-zone.png) | ![The app on a phone](docs/ux2/shell-phone.png) |
| **A route drawn in 3D carries the mug over the vase** | **Teach by showing: the robot reads back what it understood** |
| ![A drawn route over the vase](docs/readme/carry.jpg) | ![Teach by showing](docs/readme/teach.jpg) |

Still to capture for this README (contributions welcome, our own footage only): a physics pick & place on the Panda;
a drawer opened in a generated kitchen; Watch → Coach in the gym pack; a path drawn in the air on a Quest; a photo
turned into a graspable object; the Korean UI.

## Quickstart (60 seconds)

You need macOS or Linux with [uv](https://docs.astral.sh/uv/), git and curl. Datasets also need `ffmpeg`.

```bash
git clone https://github.com/NomaDamas/PhysicsClay && cd PhysicsClay
make setup        # = ./scripts/setup.sh: .venv, Python deps, robot models, MediaPipe models
make run          # http://127.0.0.1:8780 here, https://<LAN IP>:8781 for phones and Quest
```

Open **http://127.0.0.1:8780** in Chrome, Edge or Safari, type "put the mug on the right" and press **Run** on the
plan card. Press **?** in the top bar for the in-app guide.

* **Keys are optional.** Without any, an offline rule parser (English and Korean) and the browser's speech recognition
  take your commands, and `make run ARGS="--mock"` adds a scripted offline brain. For Gemini Live (voice + vision),
  paste a Gemini key in **Settings → AI** (stored only in your browser). Server keys go in `.env` (git-ignored):
  `GEMINI_API_KEY`, `OPENAI_API_KEY` (robot voice), and `LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL` for the LLM agent
  (any OpenAI-compatible Responses API). On a `--byok` server each visitor enters their own keys in Settings.
* **Phones and Quest.** `make run` also serves https on 8781 with a self-signed certificate (created in `certs/` on
  first start), because phones only share their camera and microphone over https. Accept the warning once. The
  terminal prints the phone address (`https://<LAN IP>:8781/phone`); in the app, **Use my phone** shows it as a QR code.
* **Options.** `make run HTTPS=0` (http only), `make run PORT=9000 HTTPS_PORT=9001`, `ARGS="--webcam"` (this computer's
  camera on the server), `ARGS="--kinematic"` (no physics), `ARGS="--byok"` (a public server: visitors bring their own
  keys, see [SECURITY.md](SECURITY.md)).

Full user guide: [docs/USAGE.md](docs/USAGE.md) ([한국어](docs/USAGE.ko.md)).

## Feature tour

### Robots

Eleven bodies from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (via mink's examples) and
Unitree. Switch from the robot pill in the top bar (or ask the agent). The same intent compiles for each body.

| Robot | Kind | Gripper (physics) | Contact pick & place, mug · block (10 trials each) | Status |
|---|---|---|---|---|
| Unitree G1 | humanoid | two-pad palm gripper (built, 15 cm) | 10/10 · 10/10 | ok |
| Franka Emika Panda | arm | own fingers | 10/10 · 10/10 | ok |
| Universal Robots UR5e | arm | parallel jaws (built, 14 cm) | 10/10 · 10/10 | ok |
| uFactory xArm7 | arm | own gripper | 10/10 · 10/10 | ok |
| KUKA LBR iiwa 14 | arm | parallel jaws (built, 14 cm) | 10/10 · 10/10 | ok |
| ALOHA | two arms | own fingers; two-arm handover | 10/10 · 9/10 | ok |
| Apptronik Apollo | humanoid | two-pad palm gripper (built) | 10/10 · 10/10 | ok |
| Hello Robot Stretch 3 | mobile manipulator | spring-loaded parallel pads | 10/10 · 10/10 | ok |
| Stanford TidyBot | mobile manipulator | own Robotiq 2F-85 | 10/10 · 10/10 | ok |
| Boston Dynamics Spot | quadruped + arm | own claw | 10/10 · 10/10 | ok |
| Unitree H2 | humanoid | hands not driven | no pick | experimental (reach, paths) |

Numbers are from `scripts/bench_grasp.py -n 10` (seed 0; the method and the history are in [docs/V1.md](docs/V1.md)).
Humanoids and Spot stand on a harness that follows the planned base: no walking or dynamic balance. Mobile bases drive,
with the wheels abstracted. `PC_PHYSICS=0` (or `--kinematic`) is the old kinematic mode: IK poses the robot and a
picked object rides the hand.

### Worlds

A wooden table, a kitchen counter, a shelf, a floor mat and a gym floor, plus **generated rooms**: four homes
(kitchen, dining room, living room, laundry) and six industrial sites (assembly line with a running conveyor,
warehouse, CNC machine tending, electronics bench, lab, packing station). Every seed is a different layout; drawers and
doors are on real joints and the robot opens them by their handles. Multi-step requests ("take the spoon out of the top
drawer and put it in the sink") become one plan card. Guide: [docs/worlds.md](docs/worlds.md).

### Real photos → simulated objects and spaces

Photograph one object and it becomes a physics-ready object with its real size, mass and friction that robots can
grasp; photograph your table and the scene is rebuilt with the same layout. Works offline with simple shapes; better
depth, recognition and meshes with optional models or keys. Guide: [docs/RECON.md](docs/RECON.md).

### Watching the person

With your camera (laptop, phone or Quest) the robot tracks your body, hands and the tool you hold, and reads what you
are doing (activity, phase, reps). Modes: **mirror** (copies you live), **learn** (records a demonstration it can
repeat), **respond** (standing rules), **assist** (force-limited help when you stall), **coach** (counts reps, one cue
per rep). See [Watch modes](docs/USAGE.md#watch-modes).

### Trajectories and VR

Paths are full 3D trajectories: height, wrist orientation, speed, force, grip and tolerance per point, shown as a ghost
gripper with handles you can drag, turn and edit by voice ("higher there", "slower near the vase"). Draw them from words
(agent), keyframes, a stroke in the robot's view, a demonstration, or in the air with a Meta Quest (VR or passthrough MR,
with live teleop). Guides: [Trajectories](docs/USAGE.md#trajectories), [docs/vr.md](docs/vr.md).

### Skills and demo packs

A demonstration becomes a parametric skill relative to the scene: `weave_seam`, `wipe`, `stir`, `press`, `spot`,
`mirror_motion`. "Geometry gives precision, the person gives style": the seam comes from the scene, the weave from your
hand. Three packs set up a story in one click: a **weld cell**, a **gym** and a **workshop helper**. Guides:
[docs/skills.md](docs/skills.md), [docs/packs.md](docs/packs.md).

### Safety around people

Speed and separation monitoring in the spirit of ISO/TS 15066: the robot slows within about half a metre of you, makes
a protective stop before contact (sooner near your head and torso), resumes when you step back, and caps force where
contact is allowed (hands and forearms, in assist and handovers). It can't be turned off. It is **not** a certified
safety system. Guide: [docs/safety.md](docs/safety.md).

### Datasets

Press **Record** and every task becomes a LeRobot v2.1 episode (joint states, actions, front and wrist video, your
words, success labels, your pose). `scripts/generate.py` replays one intent as many randomized episodes across robots,
rooms, lighting and physics. Guide: [docs/datasets.md](docs/datasets.md).

### Real-to-sim calibration

Probe motions (swing, press, lift, slide) and system identification fit friction, mass, damping, stiffness and latency
so the simulation matches a robot; the same ranges drive domain randomization. A `RobotDriver` interface (sim driver
plus LeRobot / ROS 2 stubs) is where real hardware would plug in. Guide: [docs/calibration.md](docs/calibration.md).

### Languages

English and Korean throughout: the offline parser understands both, the LLM agent and Gemini answer in the language you
use, the in-app guide has both, and the server's messages (tasks, plans, safety, watch, skills, worlds, calibration…)
and the UI's own labels follow **Settings → Language** (System / 한국어 / English). See
[Voice and language](docs/USAGE.md#voice-and-language).

## Architecture

One Python process: an aiohttp server and a simulation worker thread. The browser sends what you say, point at and
show over a WebSocket; the worker grounds it in a MuJoCo world, plans with mink IK, runs it on the physics plant and
streams state and video back; three.js draws the shared 3D stage.

```
browser (web/)                                     server (pps/, one process)
  voice · text · camera · pointing · 3D · VR  ──ws──▶  features (pps/features/*): plan, candidates, zones, goal,
  Gemini Live / Robotics-ER (optional)                   copresence, watch, safety, traj, skills, fixtures, recon …
  plan card · ghost · 3D stage (three.js)    ◀──ws──   tasks (pps/tasks.py, pps/manip.py) → IK planner (mink)
                                                         → physics plant (pps/physics.py, MuJoCo mj_step)
  LLM agent (pps/agent.py) acts like a browser ──────▶   → state + frames (localized by pps/i18n.py) → browser
```

Details: [docs/architecture.md](docs/architecture.md) · wire protocol: [docs/PROTOCOL.md](docs/PROTOCOL.md) · the v1
workstream contract and "as built" notes: [docs/V1.md](docs/V1.md).

## Extending

Most additions never touch `pps/app.py` or `web/index.html`:

* **A robot**: a `RobotProfile` in `pps/world/profiles.py` plus its prompts ([how](CONTRIBUTING.md#add-a-robot)).
* **A scene template or a generated room**: `pps/world/scene.py` ([how](CONTRIBUTING.md#add-a-scene-template)),
  `pps/world/rooms.py` and `pps/world/fixtures.py` ([how](docs/worlds.md#adding-your-own)); a demo pack in
  `pps/world/packs/` ([how](docs/packs.md#adding-a-pack)).
* **A feature**: `pps/features/<name>.py` (decorators for messages, tasks, state, ticks, overlays) and
  `web/ui/<name>.js` on the `PCUI` registry ([how](CONTRIBUTING.md#add-a-feature-server--ui)).
* **A check**: `scripts/checks/<name>.py` with `async def run(t)`; every feature has one
  ([how](CONTRIBUTING.md#add-a-check)).
* **A language**: the server's messages go through `pps/i18n.py`; the catalog is `pps/locale/ko_*.py` (English
  template → Korean), checked by `tests/test_i18n.py` ([how it works](docs/PROTOCOL.md#v17--language-of-server-messages)).

## Testing

```bash
make lint                       # ruff
make test                       # pytest unit suite: pure functions, no server, seconds
node --test tests/*.mjs         # browser-side pure modules (XR alignment, QR)
make selftest                   # end-to-end: an isolated server driven over the real protocol by scripts/checks/*
```

* `PC_SELFTEST_OFFLINE=1 make selftest` runs without keys or network, as CI does; `PC_SELFTEST_CHECKS=physics,paths`
  runs only those checks; `PC_SELFTEST_PORT=8791` picks the port. `PC_PHYSICS=0 make selftest` tests the kinematic mode.
* `scripts/e2e_ui.py`: the UI in headless Chrome with real input events (zoom, theme contrast, help, voice, camera,
  phone camera). `scripts/e2e_camera.py`: the real-camera path with your own clips as a fake webcam.
* `scripts/bench_grasp.py -n 10`: the contact pick & place benchmark per robot. `scripts/calibrate.py --demo`: the
  hidden-robot calibration demo.
* `scripts/check_links.py`: relative links in the docs (also a unit test).

## Limitations

PhysicsClay is a simulator and a research tool. Honestly:

* **Simulation only.** No real robot has been driven by it; the hardware bridge is an interface with stubs. Calibration
  was proven against a "hidden" simulated robot, not hardware.
* **Not a safety system.** The person is a pose estimate from one camera; the safety layer models the idea of speed
  and separation monitoring and must never guard a real robot.
* **No walking.** Humanoids and Spot stand on a harness that follows the planned base; mobile bases slide (no wheel
  contact). The Unitree H2's hands aren't driven.
* **Reactive IK, not a motion planner.** Rarely a hand wedges between objects or a reachable spot is refused; the
  robot says so instead of trying something unsafe. The G1's side grasp places reliably only in a band in front of it.
* **Simplified world.** Objects are rigid (no cloth, liquids or deformables); no welding arc or melt (the bead is the
  torch path); handles are grasped from above; photo reconstruction from one view guesses the hidden side.
* **Sim-to-real gap.** Rendered images, primitive objects and joint-target actions are far from a real robot's data.
* **VR** was tested with an emulator and a mocked `navigator.xr`; a real Quest session is a manual test.
* **Language.** Server messages that aren't in the catalog yet stay in English.

Each guide ends with its own limitations section.

## License and credits

PhysicsClay is licensed under [Apache-2.0](LICENSE). Third-party models, assets and libraries keep their own licenses,
listed in [NOTICE](NOTICE): robot models from MuJoCo Menagerie and the vendors (one, TidyBot, has no license file
upstream; check before redistributing), the Google Scanned Objects mug (CC BY 4.0), MediaPipe and Depth Anything V2
Small models (Apache-2.0). The optional `yolo` extra (Ultralytics) is **AGPL-3.0** and is not installed by default.

Built on [MuJoCo](https://github.com/google-deepmind/mujoco), [mink](https://github.com/kevinzakka/mink),
[MediaPipe](https://github.com/google-ai-edge/mediapipe), [three.js](https://threejs.org), OpenCV and aiohttp. The
HUD and path-drawing interaction were inspired by [CozyClay](https://github.com/HaD0Yun/CozyClay) (no code included).
The interface's design pass followed the [unslop](https://heydamir.design/unslop) method by Damir Sokolovsky
(CC BY 4.0; no unslop images or code included).

Contributing: [CONTRIBUTING.md](CONTRIBUTING.md) · [Code of Conduct](CODE_OF_CONDUCT.md) · [Security](SECURITY.md)

## History

PhysicsClay started at a hackathon (YC RFS Fall 2026, *New Operating System for the Physical World*) as "a new
generation of interactive prompt for Physical AI": video → robot motion, routes drawn in 3D with voice, and real-time
video understanding on a Unitree G1. v1 added real physics, more robots and worlds, watching the person, skills,
safety, datasets and calibration ([CHANGELOG.md](CHANGELOG.md)).

* Hackathon demo video (2 min): [docs/hackathon/demo/physicsclay_demo.mp4](docs/hackathon/demo/physicsclay_demo.mp4)
* Hackathon live demo: https://physicsclay.jinminseong.com and its narrated pitch
  https://physicsclay.jinminseong.com/static/present.html (a demo deployment; it may not run the latest version)
* Pitch script and stills: [docs/hackathon/](docs/hackathon/README.md); the UX 2.0 design: [docs/UX2.md](docs/UX2.md)
