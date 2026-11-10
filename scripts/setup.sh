#!/usr/bin/env bash
# One-shot environment setup: .venv + Python deps (pyproject.toml) + robot models (mink examples) + MediaPipe models.
#
#   ./scripts/setup.sh                                  core + [vision] (MediaPipe) + [dev] (ruff, pytest)
#   PC_EXTRAS=vision,data,dev ./scripts/setup.sh        choose extras: vision, data (pyarrow), dev,
#                                                       yolo (Ultralytics YOLO11/SAM 2.1, AGPL-3.0: see NOTICE)
#   PC_PYTHON=3.11 ./scripts/setup.sh                   another Python for a new .venv (default 3.10)
#
# Safe to re-run: an existing .venv, vendor/mink and models are kept.
set -euo pipefail
cd "$(dirname "$0")/.."
PY=${PC_PYTHON:-3.10}
EXTRAS=${PC_EXTRAS:-vision,dev}
MINK_REF=5f800f0a369d05bab88b5e5582cf95122d3b82a5   # kevinzakka/mink main, Nov 2026: the robot models we tested
MP=https://storage.googleapis.com/mediapipe-models

command -v uv >/dev/null || { echo "uv is required: https://docs.astral.sh/uv/getting-started/installation/"; exit 1; }

echo "[1/5] venv"
[ -x .venv/bin/python ] || uv venv .venv --python "$PY"

echo "[2/5] python deps (extras: ${EXTRAS:-none})"
uv pip install --python .venv/bin/python -e ".${EXTRAS:+[$EXTRAS]}"

mkdir -p vendor models
echo "[3/5] robot models (mink examples @ ${MINK_REF:0:7})"
if [ ! -d vendor/mink ]; then
  git init -q vendor/mink
  git -C vendor/mink remote add origin https://github.com/kevinzakka/mink
  git -C vendor/mink fetch -q --depth 1 origin "$MINK_REF"
  git -C vendor/mink checkout -q FETCH_HEAD
fi

echo "[4/5] mediapipe pose model"
[ -f models/pose_landmarker_full.task ] || curl -fsSL -o models/pose_landmarker_full.task \
  "$MP/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task"
echo "[5/5] mediapipe object model"
[ -f models/efficientdet_lite0.tflite ] || curl -fsSL -o models/efficientdet_lite0.tflite \
  "$MP/object_detector/efficientdet_lite0/float16/1/efficientdet_lite0.tflite"

.venv/bin/python -c "import mujoco, mink, cv2, aiohttp; from importlib.metadata import version as v; print('IMPORT OK  mujoco', v('mujoco'), ' mink', v('mink'), ' opencv', v('opencv-python'))"
echo "SETUP DONE   run: make run   (or ./scripts/web.sh)   test: make test"
