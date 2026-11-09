"""Build the ≤120 s demo video automatically: narrated slides + before/after motion clips + a screen capture of the REAL app
running the demo autopilot, with OpenAI TTS for every spoken line.

Usage: .venv/bin/python scripts/make_video.py      (server must be running on :8777)
Output: recordings/video/physicsclay_demo.mp4
"""
import asyncio
import base64
import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import aiohttp
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "recordings" / "video"
OUT.mkdir(parents=True, exist_ok=True)
TMP = OUT / "parts"
TMP.mkdir(exist_ok=True)
APP = "http://127.0.0.1:8777"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
W, H, FPS = 1920, 1080, 25
import imageio_ffmpeg  # noqa: E402

FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()
ONE_LINER = "New Generation of interactive prompt for Physical AI"


def env(key):
    for line in (ROOT / ".env").read_text().splitlines():
        if line.startswith(key + "="):
            return line.split("=", 1)[1].strip()
    return os.environ.get(key)


def tts(text, voice, name, instructions=None):
    path = TMP / f"{name}.mp3"
    if path.exists():
        return path
    body = {"model": "gpt-4o-mini-tts", "voice": voice, "input": text, "response_format": "mp3"}
    if instructions:
        body["instructions"] = instructions
    req = urllib.request.Request("https://api.openai.com/v1/audio/speech", data=json.dumps(body).encode(),
                                 headers={"Authorization": "Bearer " + env("OPENAI_API_KEY"), "Content-Type": "application/json"})
    import ssl
    import certifi
    with urllib.request.urlopen(req, timeout=60, context=ssl.create_default_context(cafile=certifi.where())) as r:
        path.write_bytes(r.read())
    return path


def duration(path):
    out = subprocess.run([FFMPEG, "-i", str(path)], capture_output=True, text=True).stderr
    for line in out.splitlines():
        if "Duration:" in line:
            h, m, s = line.split("Duration:")[1].split(",")[0].strip().split(":")
            return int(h) * 3600 + int(m) * 60 + float(s)
    return 2.0


def shot(url, path):
    subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--hide-scrollbars", f"--window-size={W},{H}",
                    "--virtual-time-budget=4000", f"--screenshot={path}", url], capture_output=True, cwd="/tmp", timeout=60)
    return cv2.resize(cv2.imread(str(path)), (W, H))


NARRATOR = "Speak like a confident startup founder giving a crisp demo: warm, clear, energetic, not rushed."


class Timeline:
    def __init__(self):
        self.writer = None
        self.t = 0.0
        self.audio = []          # (start_s, path)
        self.path = TMP / "video_noaudio.mp4"
        import imageio.v2 as imageio
        self.writer = imageio.get_writer(str(self.path), fps=FPS, codec="libx264", quality=8, macro_block_size=1)

    def frame(self, img_bgr):
        self.writer.append_data(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
        self.t += 1.0 / FPS

    def hold(self, img, seconds):
        for _ in range(int(round(seconds * FPS))):
            self.frame(img)

    def say(self, path, at=None):
        self.audio.append((self.t if at is None else at, path))

    def close(self):
        self.writer.close()


def caption(img, text):
    out = img.copy()
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.1, 2)
    x, y = (W - tw) // 2, H - 70
    cv2.rectangle(out, (x - 24, y - th - 22), (x + tw + 24, y + 20), (31, 29, 29), -1)
    cv2.putText(out, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (247, 245, 245), 2, cv2.LINE_AA)
    return out


def clip_frames(path, max_s):
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    frames, n = [], 0
    while n < max_s * fps:
        ok, f = cap.read()
        if not ok:
            break
        n += 1
        if int(n * FPS / fps) > len(frames):
            h, w = f.shape[:2]
            s = min(W / w, H / h)
            canvas = np.full((H, W, 3), (253, 251, 251), np.uint8)
            r = cv2.resize(f, (int(w * s) // 2 * 2, int(h * s) // 2 * 2))
            y0, x0 = (H - r.shape[0]) // 2, (W - r.shape[1]) // 2
            canvas[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
            frames.append(canvas)
    return frames


# --------------------------------------------------------------------------- live app capture via Chrome DevTools
async def capture_app(tl, demo_name="pitch", limit_s=95):
    port = 9333
    proc = subprocess.Popen([CHROME, "--headless=new", "--disable-gpu", "--enable-unsafe-swiftshader", "--hide-scrollbars",
                             f"--window-size={W},{H}", f"--remote-debugging-port={port}", "--autoplay-policy=no-user-gesture-required",
                             "--mute-audio", f"{APP}/?nospeak=1"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd="/tmp")
    try:
        async with aiohttp.ClientSession() as s:
            for _ in range(60):
                try:
                    async with s.get(f"http://127.0.0.1:{port}/json") as r:
                        tabs = await r.json()
                    page = next(t for t in tabs if t.get("type") == "page")
                    break
                except Exception:
                    await asyncio.sleep(0.5)
            cdp = await s.ws_connect(page["webSocketDebuggerUrl"], max_msg_size=0)
            app = await s.ws_connect(APP.replace("http", "ws") + "/ws")
            await asyncio.sleep(4.0)                          # let the page connect and render
            mid = [0]

            async def call(method, params=None):
                mid[0] += 1
                await cdp.send_str(json.dumps({"id": mid[0], "method": method, "params": params or {}}))

            await call("Page.startScreencast", {"format": "jpeg", "quality": 82, "maxWidth": W, "maxHeight": H, "everyNthFrame": 1})
            await app.send_str(json.dumps({"type": "demo", "name": demo_name}))
            t0 = time.time()
            start_t = tl.t
            last = None
            seen = None
            next_frame_t = 0.0
            done_at = None

            async def pump_app():
                nonlocal seen, done_at
                while True:
                    m = await app.receive()
                    if m.type != aiohttp.WSMsgType.TEXT:
                        if m.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                            return
                        continue
                    st = json.loads(m.data)
                    sy = st.get("say")
                    if sy and sy["id"] != seen:
                        seen = sy["id"]
                        if sy["kind"] in ("robot", "vision", "user") and sy["text"].strip():
                            voice = {"robot": "coral", "vision": "coral", "user": "ash"}[sy["kind"]]
                            ins = ("A friendly, natural humanoid robot assistant; calm and precise." if sy["kind"] != "user"
                                   else "A person casually instructing a robot, natural and relaxed.")
                            p = tts(sy["text"], voice, f"live_{sy['id']}_{sy['kind']}", ins)
                            tl.say(p, at=start_t + (time.time() - t0))
                    if st.get("demo") is None and time.time() - t0 > 5 and done_at is None:
                        done_at = time.time()

            task = asyncio.create_task(pump_app())
            while True:
                m = await cdp.receive()
                if m.type != aiohttp.WSMsgType.TEXT:
                    break
                ev = json.loads(m.data)
                if ev.get("method") == "Page.screencastFrame":
                    p = ev["params"]
                    await call("Page.screencastFrameAck", {"sessionId": p["sessionId"]})
                    img = cv2.imdecode(np.frombuffer(base64.b64decode(p["data"]), np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        last = cv2.resize(img, (W, H))
                el = time.time() - t0
                while last is not None and next_frame_t <= el:
                    tl.frame(last)
                    next_frame_t += 1.0 / FPS
                if (done_at and time.time() - done_at > 1.5) or el > limit_s:
                    break
            # flush to wall-clock time
            el = time.time() - t0
            while last is not None and next_frame_t <= el:
                tl.frame(last)
                next_frame_t += 1.0 / FPS
            task.cancel()
            await app.close()
            await cdp.close()
    finally:
        proc.terminate()


def main():
    tl = Timeline()
    pres = APP + "/static/present.html?go=0&slide="
    # 1) title
    img = shot(pres + "1", TMP / "s1.png")
    a = tts("This is PhysicsClay: a new generation of interactive prompt for Physical AI. "
            "It turns human intent into spatial instructions that robots can understand and execute.", "alloy", "n1", NARRATOR)
    tl.say(a); tl.hold(img, duration(a) + 0.3)
    # 2) text is not enough
    img = shot(pres + "2", TMP / "s2.png")
    a = tts("Some tasks are nearly impossible to give a robot in text alone. "
            "You can't put everything you think into words: where exactly, which path, what not to touch.", "alloy", "n2", NARRATOR)
    tl.say(a); tl.hold(img, duration(a) + 0.3)
    # 3) motions you can't describe: before / after clips
    a = tts("And some motions you simply can't describe. A Taekwondo kick, a sprinter's stride, a dance. "
            "So you show a video, and the robot learns it: seated, like the person on screen, or as a dance duet.", "alloy", "n3", NARRATOR)
    tl.say(a)
    clips = []
    for name, secs in (("gamst_seated", 9.0), ("dance_duet", 12.0)):
        f = ROOT / "recordings" / "samples" / f"{name}_before_after.mp4"
        if f.exists():
            clips += clip_frames(f, secs)
    for c in clips:
        tl.frame(c)
    # 4) the live app, driven by the demo autopilot (the real system)
    card = np.full((H, W, 3), (253, 251, 251), np.uint8)
    cv2.putText(card, "Live: the real system, simulated Unitree G1", (420, 520), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (31, 29, 29), 3, cv2.LINE_AA)
    a = tts("Here's the live system.", "alloy", "n4", NARRATOR)
    tl.say(a); tl.hold(card, duration(a) + 0.2)
    asyncio.run(capture_app(tl, "pitch"))
    # 5) business + close
    img = shot(pres + "6", TMP / "s6.png")
    a = tts("It's free to try in the browser, and robot teams pay per robot they teach.", "alloy", "n6", NARRATOR)
    tl.say(a); tl.hold(img, duration(a) + 0.3)
    img = shot(pres + "7", TMP / "s7.png")
    a = tts("What if teaching a robot were as natural as teaching a friend? PhysicsClay.", "alloy", "n7", NARRATOR)
    tl.say(a); tl.hold(img, duration(a) + 1.0)
    tl.close()
    total = tl.t
    # mix audio clips at their start times
    inputs, filters = [], []
    for i, (start, path) in enumerate(tl.audio):
        inputs += ["-i", str(path)]
        filters.append(f"[{i + 1}:a]adelay={int(start * 1000)}|{int(start * 1000)}[a{i}]")
    mix = "".join(f"[a{i}]" for i in range(len(tl.audio)))
    filters.append(f"{mix}amix=inputs={len(tl.audio)}:normalize=0:dropout_transition=0,volume=1.0[aout]")
    out = OUT / "physicsclay_demo.mp4"
    cmd = [FFMPEG, "-y", "-i", str(tl.path)] + inputs + ["-filter_complex", ";".join(filters), "-map", "0:v", "-map", "[aout]",
                                                          "-c:v", "copy", "-c:a", "aac", "-b:a", "160k", "-t", f"{total:.2f}", str(out)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(json.dumps({"out": str(out), "seconds": round(total, 1), "clips": len(tl.audio), "ffmpeg_ok": r.returncode == 0,
                      "err": r.stderr[-300:] if r.returncode else ""}))


if __name__ == "__main__":
    main()
