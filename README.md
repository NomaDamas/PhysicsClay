# PhysicsClay

**New Generation of interactive prompt for Physical AI — turning human intent into spatial instructions that robots can understand and execute.**

Some tasks are nearly impossible to give a robot in text alone. You can't put everything you're thinking into words:
where exactly, which path, how high, what not to touch. And some motions you can't describe at all — a Taekwondo kick,
a sprinter's stride, a fighter's stance: showing beats describing. Physical AI needs its own prompt, a spatial one.

**Live:** https://physicsclay.jinminseong.com (phone camera + mic work over HTTPS) · pitch: `/static/present.html` ·
browser physics lab: `/static/lab.html` · before/after samples: `web/samples/`

Text alone cannot carry a physical instruction. PhysicsClay combines four channels and grounds them in physics:

| Channel | What it carries | How |
|---|---|---|
| Voice (realtime) | what to do, constraints ("put the mug on the left, slowly") | Gemini Live, native audio, tool calls |
| Your camera | gestures and actions, understood semantically | frames streamed to Gemini Live; MediaPipe pose for follow mode |
| Robot's view | where: tap a spot, draw a path on what the robot sees | MuJoCo raycast to 3D table points; Gemini Robotics-ER pointing for "next to the mug" |
| Physics | whether the robot can actually do it, and why not | IK pre-check of every waypoint on the real Unitree G1 model, joint limits, self-collision, speed |

Imitation is a mode, not the default: say "follow me", show a motion, then "learn this as wave" and later "do wave".

## Run
```bash
./scripts/setup.sh                 # venv + mujoco + mink + mediapipe + models
./scripts/web.sh                   # http://127.0.0.1:8777  (webcam)
./scripts/web.sh --https           # + https://<lan-ip>:8443 so a phone can use its mic (accept the cert warning)
./scripts/web.sh --fake            # no camera
```
Open Settings in the page and paste a Gemini API key (stored only in the browser). Without a key the page falls back to
browser speech recognition and a rule parser.

## Architecture
```
browser: Gemini Live (voice + your camera) --tool calls--> intents --WebSocket--> server
         Robotics-ER pointing on the robot view                                    |
server:  intent -> target (region / marker / raycast) -> hand path -> IK pre-check per waypoint
         -> refuse with a reason, or execute (kinematic grasp) -> verdict back to Gemini, which says it out loud
```
Protocol: `docs/PROTOCOL.md`. Assets: Unitree G1 (mujoco_menagerie), Google Scanned Objects mug (CC-BY 4.0).
Open source: MuJoCo, mink, MediaPipe, OpenCV, aiohttp; HUD patterns from CozyClay.

## Roadmap
1. Today: teach by showing (camera + voice + pointing + 3D routes) on a simulated Unitree G1 and a UR5e, physics-checked.
2. **VR / XR teaching**: stand inside the robot's space with a headset, grab the 3D route and the objects with your hands,
   and demonstrate at true scale; the same task spec and physics check carry over.
3. Real robots: send approved task specs to hardware (sim-checked first), learn from corrections across robots.
