"""pps/protocol.py: the UDP JSON link between the perception process and the robot runtime."""

import socket
import time

from pps import protocol


def _free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_landmark_indices_are_mediapipe_pose_indices():
    idx = list(protocol.LANDMARKS.values())
    assert len(idx) == len(set(idx))
    assert all(0 <= i <= 32 for i in idx)  # MediaPipe Pose has 33 landmarks
    for side in "lr":
        for part in ("shoulder", "elbow", "wrist", "hip", "knee", "ankle"):
            assert f"{side}_{part}" in protocol.LANDMARKS
    # left landmarks have odd indices, right ones even (MediaPipe convention)
    assert all(i % 2 == 1 for k, i in protocol.LANDMARKS.items() if k.startswith("l_"))
    assert all(i % 2 == 0 for k, i in protocol.LANDMARKS.items() if k.startswith("r_"))


def test_default_addresses_are_loopback():
    for host, port in (protocol.PERCEIVE_TO_ROBOT, protocol.ROBOT_TO_PERCEIVE):
        assert host == "127.0.0.1" and 1024 < port < 65536
    assert protocol.PERCEIVE_TO_ROBOT != protocol.ROBOT_TO_PERCEIVE


def test_send_receive_round_trip_skips_garbage():
    addr = ("127.0.0.1", _free_port())
    recv = protocol.make_receiver(addr)
    send = protocol.make_sender(addr)
    assert recv() == []
    msgs = [
        {"t": 1.0, "engaged": True, "lm": None, "grip": {"l": 0, "r": 1}, "key": None, "cmd": "R"},
        {"mode": "FOLLOW", "messages": ["héllo"], "feasibility": 0.5},
    ]
    send(msgs[0])
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as raw:
        raw.sendto(b"not json{", addr)
    send(msgs[1])
    got = []
    for _ in range(50):
        got += recv()
        if len(got) >= 2:
            break
        time.sleep(0.01)
    assert got == msgs
