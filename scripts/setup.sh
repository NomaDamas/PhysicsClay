#!/usr/bin/env bash
# One-shot environment setup: venv + deps + G1 model (via mink examples) + MediaPipe pose model.
set -uo pipefail
cd "$(dirname "$0")/.."
echo "[1/4] venv"; uv venv .venv --python 3.10
echo "[2/4] pip"; uv pip install --python .venv/bin/python mujoco mink mediapipe opencv-python "numpy<2" imageio imageio-ffmpeg daqp qpsolvers
mkdir -p vendor models
echo "[3/4] mink examples (G1 model included)"; [ -d vendor/mink ] || git clone --depth 1 https://github.com/kevinzakka/mink vendor/mink
echo "[4/5] mediapipe pose model"; [ -f models/pose_landmarker_full.task ] || curl -sL -o models/pose_landmarker_full.task https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task
echo "[5/5] mediapipe object model"; [ -f models/efficientdet_lite0.tflite ] || curl -sL -o models/efficientdet_lite0.tflite https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/float16/1/efficientdet_lite0.tflite
ls -la models vendor/mink/examples/unitree_g1 | head -20
.venv/bin/python -c "import mujoco, mink, mediapipe, cv2; print('IMPORT OK', mujoco.__version__, mink.__version__, mediapipe.__version__, cv2.__version__)"
echo "SETUP DONE"
