#!/usr/bin/env bash
# Launch robot runtime (mjpython, MuJoCo viewer) + perception HUD (python, webcam).
cd "$(dirname "$0")/.."
.venv/bin/mjpython -m pps.robot_runtime "$@" &
ROBOT=$!
trap 'kill $ROBOT 2>/dev/null' EXIT
sleep 2
.venv/bin/python -m pps.perceive
