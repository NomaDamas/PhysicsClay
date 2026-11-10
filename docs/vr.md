# VR / MR on Meta Quest (WebXR)

Draw a robot's trajectory in the air at 1:1 scale, grab the ghost grippers and turn them with your wrist, see the
robot on your real table through passthrough, turn your room into the scene, and drive the robot's hand live. There is
no app to install: it is the same page, opened in the Quest browser.

Code: `web/xr.js` (WebXR glue), `web/xr-core.js` (the pure part: alignment, gestures, strokes, the input → message
pipeline, room → scene), `pps/features/teleop.py` (live teleoperation). Trajectories are the P2 object
(`docs/PROTOCOL.md` → v9); teleop and the VR additions are in `docs/PROTOCOL.md` → v10.

## Open it on a Quest

1. Start the server with HTTPS (WebXR needs a secure page):
   ```
   .venv/bin/python -m pps.app --https            # add --fake for no camera
   ```
   It prints a `Quest :` line, `https://<your-computer's-LAN-IP>:8443` (`--https-port` to change it).
2. Put the Quest on the same Wi-Fi as the computer. Open the Quest browser and go to that address.
3. The certificate is self-signed: choose **Advanced → Proceed to … (unsafe)** once.
4. In the 3D stage, next to **3D / Robot eye**, press **Enter VR** or **Enter MR** (they only appear when the browser
   supports them). Allow hand tracking and spatial data when the Quest asks.

Alternative without the certificate warning: connect the Quest over USB (developer mode) and run
`adb reverse tcp:8777 tcp:8777`, then open `http://localhost:8777` in the Quest browser (localhost counts as secure).

Hands work best (Settings → Movement tracking → Hand and body tracking). Controllers work too. For **MR room scans**, do
Space Setup first (Settings → Physical space → Space setup) and mark your desk / table.

## VR and MR

* **VR** (`immersive-vr`): the table appears in front of you at a comfortable height for how you stand or sit
  (about 60 cm below your eyes). **Recenter** (menu) puts it in front of you again.
* **MR** (`immersive-ar`, passthrough): the robot stands on your real table. The first time, the page asks you to
  **pinch your table's near-left corner, then its near-right corner** (the edge in front of you). That fixes the
  robot's table to yours: rotation, position and height, centred if your table is wider or narrower than the robot's
  (the panel tells you both widths). The sim's floor is hidden and its furniture drawn as a faint ghost, so you see your
  own. The calibration is saved in this browser (`localStorage`, key `pc.xr.mr`) together with a spatial anchor when
  the Quest grants `anchors`, so next time the robot comes back to the same spot even if you start somewhere else.
  **Calibrate** (menu) does it again.
* **Side** (menu): *facing* (default) puts the robot across the table from you, like a colleague; *robot's* puts you on
  its side, looking over its shoulder (its left is your left: the natural view for driving it).

## Gestures

| | Hands | Controllers |
|---|---|---|
| Draw a path | pinch (thumb + index) and move; release to finish | hold the trigger and move |
| Add a keyframe | a quick pinch in empty space (the first two connect into a path; later ones are inserted where they fit) | a quick trigger pull |
| Move a point | pinch on a ghost handle (within 3.5 cm) and move | trigger on a handle |
| Turn the wrist there | twist your hand while holding the handle (more than 10°) | twist the controller |
| Drive the robot's hand | make a fist (middle, ring and little finger curled); open the hand to let go | hold the grip button |
| Close its gripper while driving | pinch index and thumb together, keeping the other three fingers curled | trigger (analog) |
| Menu | turn your left palm toward you; poke a button with your right index finger | it sits on the left controller; point the right ray and pull the trigger |

The menu also has a fixed spot on your left by the table's edge, so it is always reachable.

**Menu:** Go · Undo · Clear · Exit / Draw · Keys · Drive · Scan / Recenter (VR) or Calibrate (MR) · Side · Speed · Talk /
Use my room (in Scan) · Record (when the server has P1 datasets).

* **Draw**: pinch-and-move draws; quick pinches add keyframes; handles can be grabbed.
* **Keys**: every pinch drops a keyframe (no strokes); handles can be grabbed.
* **Drive**: live teleoperation (below).
* **Scan**: shows what the room scan found; **Use my room** sends it.
* **Speed**: *robot's* (default speed), *as drawn* (the path moves as fast as you drew it), *pinch* (a firmer pinch or
  trigger pull = faster, 5 to 30 cm/s).
* **Talk**: the page's existing microphone button (Gemini Live or browser speech), so "higher there" or "go" still work.

A floating panel by the table says what to do in the current mode, whether the path is fine (✓) or why not (✕, the
server's reason, e.g. "Out of my reach by 12 cm between points 5 and 6"), what you are holding, and the teleop rate and
round-trip time. The path itself is the same ghost the desktop draws: a blue tube (red where it fails a check), a
ghost gripper at each point and numbered handles.

## What it sends

* A stroke: resampled every 1.5 cm (at most 60 points), smoothed (moving average), in world coordinates, then
  `{type:"traj", op:"propose", traj:{source:"vr", frame:"world", role, label:"drawn in VR", points:[{p, speed?}]}}`.
  The role is the hand on the path's side for two-armed robots. Nothing moves until **Go** (or "go").
* A moved handle: `{type:"traj", op:"edit", index, world:true, point:{p, q?}}` (`q` when you turned your wrist more than
  10°: the turn you made, applied to the point's orientation).
* A keyframe: `{op:"edit", insert:{after, p}}` (or a 2-point `propose` with `source:"keyframes"`).
* Undo: the previous trajectory again as a `propose` (or `clear`). Go: `{op:"run"}`. Clear: `{op:"clear"}`.
* Drive: `{type:"teleop", op:"start", role, src:"vr"}`, then `{type:"teleop", role, p, q, grip, t}` at up to 30 Hz while
  the fist / grip is held, `{op:"stop"}` when you leave Drive mode.
* Use my room: `{type:"world", op:"set", scene}`.

## Live teleop (`pps/features/teleop.py`)

Your hand is a clutch: when you make a fist, the robot's hand target is anchored where it is, and from then on it moves
by exactly as much as your hand moves (and turns as your wrist turns), in the shared space. Across the table your right
hand drives its left hand (a mirror, like P3's mirror mode); on its side, right drives right. The server:

* clamps every target to the arm's reach (a sphere around the shoulder) and above the work surface;
* moves the target at most 1 m/s and 3 rad/s, both multiplied by P5's `rt.safety_scale`;
* ignores commands during a protective stop (`rt.safety_scale <= 0` or `tasks.safety.stopped`) and targets inside the
  person's body zone (`tasks.safety.check_point`): VR cannot bypass safety;
* holds still when commands stop (they are absolute targets), and ends teleop after 3 s without one;
* drives the gripper with `rt.set_gripper` when the runtime has one (P0), else the kinematic grasp (closing next to an
  object holds it);
* leaves the hand alone while a task or path runs (`held: "busy …"`); starting teleop stops it.

`state.teleop = {active, role, src, rate, lag_ms, stale, echo, held, clamped, scale, hands: {right: {p, q, ee, grip,
holding}}, reply, events}`. P1 datasets can record these sessions; when the server publishes `state.dataset`, the menu
shows **Record** and the panel shows "● Recording a demo" while it records.

## Room scan → scene

With `plane-detection` / `mesh-detection` (Quest, after Space Setup):

* the table is the plane or mesh labelled *table* / *desk*, else the largest horizontal plane 40–130 cm above the floor;
* its height picks the template: under 25 cm *floor*, from 84 cm *kitchen*, otherwise *table*;
* if MR isn't calibrated, the robot's table is lined up with the edge of your table nearest to you;
* things standing on the table (meshes and planes within its footprint) become fixed boxes (`movable:false`, at most 10,
  sizes clamped to the scene spec's 2–40 cm);
* the current movable objects (the mug, blocks) are kept;
* walls, floor and furniture away from the table are previewed (grey) but not sent: the scene spec only describes the
  work surface (docs/WORLD.md).

Boxes that will be sent are blue in the preview. **Use my room** sends `{type:"world", op:"set", scene}`; the server
rebuilds the world and the 3D stage reloads (the robot stays).

## Testing

### Emulated (no headset): `scripts/checks/xr.py` (part of `scripts/selftest.py`)

* `node --test tests/xr_core.test.mjs`: the XR ↔ sim transform and its matrix, two-point calibration (both sides,
  random poses), VR recentring, re-basing on a moved anchor, pinch / trigger / fist hysteresis, hand metrics, stroke
  smoothing / 1.5 cm resampling / 60-point cap / speed from the hand or pinch, handle picking, keyframe insertion,
  the input → message pipeline (stroke → `propose`, grab + twist → `edit` with `p` and `q`, tap → `insert`, undo, go,
  clear, UI hover, lost tracking, teleop clutch at ≤ 30 Hz, calibration taps) and room → scene spec.
* Headless Chrome with a mocked `navigator.xr` (`scripts/checks/xr_mock.js`: XRSystem, XRSession, frames with two eye
  views, 25-joint hands, planes, meshes, anchors), driving the real page and server:
  the buttons appear only when supported; Enter VR starts the session through three.js and renders; VR recentring; a
  synthetic pinch stroke becomes a `propose` the server accepts; grabbing a handle, lifting it 5 cm and twisting sends
  `edit` (the point moved, the wrist turned); a tap inserts a keyframe; Undo restores the path; a fist drives the robot's
  hand through `teleop` (it moves 6 cm); leaving Drive stops teleop; exiting restores the desktop view; Enter MR, the
  two-corner calibration (centred, saved, passthrough look); a room scan becomes a scene the server builds; a session
  without the optional features still starts and says what is missing; no page errors.
  (Software GL draws the full G1 at under 1 frame/s, so the check hides its high-poly link meshes after the first frames.)
* `scripts/checks/teleop.py` + `tests/test_teleop.py`: start, a 30 Hz stream moves the hand, rate / lag / echo, reach
  and surface clamps, grasp → carry → release the mug, stop, busy while a path runs; the rate limit, P5 stop / slow-down
  / body zone and the physical gripper with stand-ins (so they hold with or without P5 merged).

### Manual, on a real Quest (not covered by emulation)

1. Same Wi-Fi, `https://<mac-ip>:8443`, accept the certificate; **Enter VR** and **Enter MR** buttons are shown.
2. Enter VR: the permission prompts appear; the table is in front of you at a comfortable height, standing and sitting
   (Recenter after you sit down). The frame rate is smooth (the G1 is ~350k triangles; if it stutters, try the Panda).
3. Hands: pinch-and-move draws an orange line; on release the blue / red ghost appears with the server's reason on the
   panel; draw something out of reach and read the reason.
4. Grab a handle, move it 5 cm and twist your wrist: the handle follows, the ghost gripper turns after release.
5. A quick pinch in empty space adds a keyframe; Undo removes it; Go runs the path; Clear.
6. Turn your left palm toward you: the menu appears there; poke buttons with your right index finger; the left hand
   holding the menu doesn't draw.
7. Controllers: trigger draws / grabs, grip drives, the right ray + trigger presses menu buttons.
8. Drive: make a fist and move; the robot's hand follows smoothly (watch rate ≈ 30/s and the round-trip time on the
   panel); pinching index and thumb next to the mug picks it up, opening them puts it down; stepping very close stops it
   when P5 safety is merged.
9. Enter MR: pinch the near-left then near-right corner of your real table; the sim table lies on yours (edges
   within ~1–2 cm); exit and enter again from another spot: the robot comes back to your table (persistent anchor), or
   the panel suggests Calibrate.
10. Scan (after Space Setup): the preview boxes sit on your real furniture; **Use my room** rebuilds the scene with your
    monitor / boxes as obstacles; the template matches your table height.
11. Talk: the menu's Talk button starts the page's microphone; "make it higher there" edits the path.
12. Exit (menu or the Quest button): the desktop page is back to normal.

## Limitations

* The scene spec only describes the work surface: walls and furniture away from the table are shown, not simulated;
  boxes on the table are axis-aligned (no yaw) and at most 40 cm; the table's real size isn't copied (the template's
  size is used, centred on your table).
* Without the `anchors` feature the saved MR calibration is only right if the Quest's floor origin is the same as last
  time (it usually is not): press Calibrate.
* `hit-test` is requested but not used yet (calibration uses your fingertip, accurate to about a centimetre).
* Hand-tracking pinch can drop out when your hand is edge-on to the cameras; a dropped pinch ends the stroke.
* Teleop maps your hand's motion relative to where you clutched (not absolute positions), and orientation as a world
  turn; there is no force feedback. P1 recording depends on the P1 workstream (`state.dataset`).
* Rendering cost: the G1's meshes are ~350k triangles; on Quest 2 the frame rate may drop (manual test 2).
