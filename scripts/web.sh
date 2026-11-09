#!/usr/bin/env bash
# PhysicsClay browser HUD: one process (camera + pose + IK + sim render + web server).
cd "$(dirname "$0")/.."
( sleep 3; open "http://127.0.0.1:8777" 2>/dev/null ) &
.venv/bin/python -m pps.app "$@"
