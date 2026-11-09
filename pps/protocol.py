"""UDP JSON protocol between the perception process (python) and the robot runtime (mjpython).

perceive -> robot  (PERCEIVE_TO_ROBOT, ~30 Hz)
    {"t": float, "engaged": bool, "lm": {name: [x, y, z]} | null, "grip": {"l": 0|1, "r": 0|1},
     "key": "R"|"P"|"A"|"M"|"H"|null, "cmd": str|null}
    lm is in MediaPipe pose-world coordinates: metres, origin at hip centre,
    x = image right, y = image down, z = depth away from camera.  Robot-agnostic.

robot -> perceive  (ROBOT_TO_PERCEIVE, ~30 Hz)
    {"mode": str, "engaged": bool, "feasibility": float, "messages": [str], "flags": {name: type},
     "speed_scale": float, "rec_frames": int, "preview_t": float, "saved": str|null, "ik_hz": float}
"""
import json
import socket

PERCEIVE_TO_ROBOT = ("127.0.0.1", 5005)
ROBOT_TO_PERCEIVE = ("127.0.0.1", 5006)

# MediaPipe Pose landmark indices we ship over the wire.
LANDMARKS = {
    "nose": 0,
    "l_shoulder": 11, "r_shoulder": 12,
    "l_elbow": 13, "r_elbow": 14,
    "l_wrist": 15, "r_wrist": 16,
    "l_pinky": 17, "r_pinky": 18,
    "l_index": 19, "r_index": 20,
    "l_thumb": 21, "r_thumb": 22,
    "l_hip": 23, "r_hip": 24,
}


def make_sender(addr):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(obj):
        sock.sendto(json.dumps(obj).encode("utf-8"), addr)

    return send


def make_receiver(addr):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(addr)
    sock.setblocking(False)

    def recv_all():
        """Drain the socket; return list of decoded messages (oldest first)."""
        out = []
        while True:
            try:
                data, _ = sock.recvfrom(65536)
            except BlockingIOError:
                break
            except OSError:
                break
            try:
                out.append(json.loads(data.decode("utf-8")))
            except ValueError:
                continue
        return out

    return recv_all
