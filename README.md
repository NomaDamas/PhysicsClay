# PhysicsClay

**A new way to prompt Physical AI: turning human intent into spatial instructions that robots can understand and execute.**

Show it. Shape it. Simulate it. Programming a robot shouldn't require programming every joint.
Demonstrate a motion in front of a webcam, shape the ghost targets in 3D, and the robot tells you
what it cannot do (reach, joint limits, self-collision, speed) before anything moves.

Built at c0mpiled-17 Seoul (YC RFS: New Operating System for the Physical World), 2026-11-09.

## Run
```bash
./scripts/setup.sh      # venv + mujoco + mink + mediapipe + pose/object models
./scripts/web.sh --objects            # browser HUD at http://127.0.0.1:8777 (webcam + real G1 sim)
./scripts/web.sh --video dance.mp4    # Show: drive the robot from any video, e.g. a dance clip
./scripts/run.sh        # alternative: native MuJoCo viewer (mjpython) + OpenCV HUD, ghosts draggable in 3D
# other intent sources (run the HUD process by hand, runtime via scripts/run.sh in another tab):
.venv/bin/python -m pps.perceive --video dance.mp4          # Show: any video, e.g. a dance clip
.venv/bin/python -m pps.perceive --objects                  # Scene: object context, what each hand holds
```
In the browser: stand back about 2 m so your shoulders and hands are in frame. `space` engages the clutch and the
G1 mirrors you. Draw a path on the camera view with the mouse: it is lifted into your body plane, compiled to the
robot, and every waypoint is pre-checked (green reachable, red not). Drag a waypoint to edit, set "reach forward"
for depth, press `F` and the robot follows it at a safe speed. `R` record, `P` preview, `A` approve and export.

Keys in the native HUD window: `space` clutch · `R` record · `P` preview · `A` approve/export · `M` mirror · `H` home · `/` command (slower, faster, mirror off, hold, go, reset) · `Q` quit.
With the clutch paused, double-click a ghost target in the viewer and Ctrl + right-drag to shape the pose.

## Architecture
Multimodal intent (demonstrate, show a video, scene objects, typed modifiers, drag 3D ghosts) ->
robot-agnostic Physical Intent IR -> per-robot grounding (retarget + IK) -> physics validation
(reach, joint limits, self-collision, speed) -> preview, approve, export.
```
perceive.py (python)                        robot_runtime.py (mjpython)
webcam -> MediaPipe Pose -> landmarks  --UDP-->  retarget -> mink IK -> validate -> read-back
HUD: mode, feasibility, ROBOT SAYS     <--UDP--  MuJoCo viewer: ghost targets, red joints
```
Physical Intent IR (`intent.json`) is robot-agnostic; `trajectory.json` is the compiled motion for the
chosen robot; `report.json` is the validator verdict. Open source used: MuJoCo, mink, MediaPipe, OpenCV.
