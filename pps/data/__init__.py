"""Robot-learning datasets (v1 P1): every task the robot runs can become a training episode.

    lerobot.py   LeRobot v2.1 dataset writer (parquet + h264 mp4 + meta/*.json[l]); no lerobot import needed
    adapter.py   what a frame is for the current body: joint state, action, gripper, end-effector force
    recorder.py  fixed-rate sampler + offscreen cameras + episode lifecycle (one task = one episode)

The server side lives in pps/features/dataset.py ({type: "dataset"} messages, state.dataset); randomized generation in
scripts/generate.py; the format and how to train on it in docs/datasets.md.
"""
