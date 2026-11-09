# Physical Prompt Studio

**Show it. Shape it. Simulate it.** Programming a robot shouldn't require programming every joint.
Demonstrate a motion in front of a webcam, shape the ghost targets in 3D, and the robot tells you
what it cannot do (reach, joint limits, self-collision, speed) before anything moves.

Built at c0mpiled-17 Seoul (YC RFS: New Operating System for the Physical World), 2026-11-09.

## Run
```bash
./scripts/setup.sh      # venv + mujoco + mink + mediapipe + Unitree G1 model + pose model
./scripts/run.sh        # robot runtime (mjpython, MuJoCo viewer) + perception HUD (webcam)
```
Keys in the HUD window: `space` clutch · `R` record · `P` preview · `A` approve/export · `M` mirror · `H` home · `/` command (slower, faster, mirror off, hold, go, reset) · `Q` quit.
With the clutch paused, double-click a ghost target in the viewer and Ctrl + right-drag to shape the pose.

## Architecture
```
perceive.py (python)                        robot_runtime.py (mjpython)
webcam -> MediaPipe Pose -> landmarks  --UDP-->  retarget -> mink IK -> validate -> read-back
HUD: mode, feasibility, ROBOT SAYS     <--UDP--  MuJoCo viewer: ghost targets, red joints
```
Physical Intent IR (`intent.json`) is robot-agnostic; `trajectory.json` is the compiled motion for the
chosen robot; `report.json` is the validator verdict. Open source used: MuJoCo, mink, MediaPipe, OpenCV.
