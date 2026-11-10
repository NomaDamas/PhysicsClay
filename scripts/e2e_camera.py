"""Real-camera end-to-end check (issue #37): a clip of a real person plays as the browser's webcam.

The whole real path, nothing synthetic: headless Chrome with a fake camera device that plays your clip
(--use-file-for-fake-video-capture) -> the app page -> the camera turned on through the Settings checkbox
("Use this device's camera", off by default) -> JPEG frames over the websocket -> the server's MediaPipe Pose ->
co-presence (pointing, open hand) and the person model / Watch modes (activity, reps; MediaPipe Hands runs in the
page while a Watch mode is on) -> state. A second websocket reads `state.human`, `state.copresence` and
`state.watch` and the check asserts on them.

Not part of the default self test: it needs Google Chrome, ffmpeg and clips of a person. Nothing is downloaded and
no footage ships with the repo; point it at your own clips (any format ffmpeg reads):

    PC_CAM_CLIP=me.mov                 any clip of a person: visible, source "phone", 33 world landmarks
    PC_CAM_POINT=point.mov             pointing at the table in front of the camera: a pointing target at least once
    PC_CAM_HAND=hand.mov               an open hand held out toward the camera: hand_open true at least once
    PC_CAM_SQUAT=squat.mov@2+12        3 squats: >= 2 reps counted in Watch "Coach" mode
    .venv/bin/python scripts/e2e_camera.py

`path@start+seconds` trims a longer clip (`@12` = from 12 s, `@12+20` = 20 s from 12 s). Without any of these
variables the check prints SKIP and exits 0.

Recording your own clips (phone or laptop, 5-10 s each, landscape, the whole upper body in view, 1-2 m away, as if
the camera were the robot across the table):
  * point:  stand facing the camera, point with a straight arm at a spot on a table between you and the camera,
            hold it still for ~1 s, then point somewhere else and hold again;
  * hand:   hold one hand out toward the camera, palm up / fingers spread, as if to receive something; hold ~2 s;
  * squat:  full body in view (feet too), 3 slow bodyweight squats facing the camera (or at 45 degrees).
One clip can do all three; then set the same path in each variable. The clip is converted to a 640x360 15 fps
y4m in a temp dir (deleted afterwards); Chrome loops it.

Assertions are deliberately tolerant (real MediaPipe output is noisy): "at least once" / ">= 2 reps" over the clip
played up to two times. Every run prints what it actually observed (frames visible, landmark count, how often
hand_open flipped, pointing armed/target, activity histogram, reps) so thresholds can be judged on real footage.

Env: PC_E2E_PORT (8937), PC_E2E_CDP (9336), PC_CHROME (Chrome binary), PC_E2E_HEADFUL=1 to watch it run,
PC_E2E_TRACE=1 to print the person state (activity, phase, reps, hip height, hand_open, pointing) 4x a second.
Exit status: 0 pass or skip, 1 any failure.
"""
import asyncio
import collections
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import aiohttp

ROOT = Path(__file__).resolve().parents[1]
PORT = int(os.environ.get("PC_E2E_PORT", "8937"))
CDP_PORT = int(os.environ.get("PC_E2E_CDP", "9336"))
BASE = f"http://127.0.0.1:{PORT}"
CHROME = os.environ.get("PC_CHROME") or next((p for p in (
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    shutil.which("google-chrome") or "", shutil.which("google-chrome-stable") or "",
    shutil.which("chromium") or "", shutil.which("chromium-browser") or "") if p and Path(p).exists()), None)
KINDS = (("PC_CAM_CLIP", "any"), ("PC_CAM_POINT", "point"), ("PC_CAM_HAND", "hand"), ("PC_CAM_SQUAT", "squat"))
WATCH_MODE = {"any": None, "point": None, "hand": "respond", "squat": "coach"}   # respond: runs browser Hands, does nothing
TRACE = os.environ.get("PC_E2E_TRACE") == "1"         # print the person state 4x a second
results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)


def ffmpeg_exe():
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


def parse_clip(spec):
    """'path[@start[+seconds]]' -> (path, start, seconds|None)."""
    path, start, dur = spec, 0.0, None
    if "@" in spec and not Path(spec).exists():
        path, rng = spec.rsplit("@", 1)
        a, _, b = rng.partition("+")
        start, dur = float(a or 0), (float(b) if b else None)
    return Path(path).expanduser(), start, dur


def to_y4m(src, start, dur, out):
    """-> seconds of video written (640x360, 15 fps, letterboxed)."""
    cmd = [ffmpeg_exe(), "-v", "error", "-y", "-ss", str(start), "-i", str(src)]
    cmd += ["-t", str(dur if dur else 12.0)]          # cap: y4m is raw video (~5 MB/s at this size)
    cmd += ["-vf", "scale=640:360:force_original_aspect_ratio=decrease,pad=640:360:(ow-iw)/2:(oh-ih)/2,fps=15",
            "-pix_fmt", "yuv420p", "-an", str(out)]
    subprocess.run(cmd, check=True)
    frames = (out.stat().st_size - 64) / (640 * 360 * 1.5 + 6)
    return max(1.0, frames / 15.0)


class CDP:
    def __init__(self, ws):
        self.ws, self.mid, self.pending, self.console = ws, 0, {}, []
        self.task = asyncio.create_task(self._pump())

    async def _pump(self):
        async for m in self.ws:
            if m.type != aiohttp.WSMsgType.TEXT:
                continue
            ev = json.loads(m.data)
            if "id" in ev and ev["id"] in self.pending:
                self.pending.pop(ev["id"]).set_result(ev)
            elif ev.get("method") == "Runtime.consoleAPICalled" and ev["params"].get("type") in ("error", "warning"):
                self.console.append(" ".join(str(a.get("value", a.get("description", "")))[:160] for a in ev["params"]["args"]))
            elif ev.get("method") == "Runtime.exceptionThrown":
                self.console.append("exception: " + ev["params"]["exceptionDetails"].get("text", "")[:160])

    async def call(self, method, params=None):
        self.mid += 1
        fut = asyncio.get_running_loop().create_future()
        self.pending[self.mid] = fut
        await self.ws.send_str(json.dumps({"id": self.mid, "method": method, "params": params or {}}))
        return await asyncio.wait_for(fut, 20)

    async def js(self, expr):
        r = await self.call("Runtime.evaluate", {"expression": expr, "returnByValue": True, "awaitPromise": True})
        return (r.get("result") or {}).get("result", {}).get("value")


async def run_clip(s, kind, y4m, seconds, profile):
    """Launch Chrome on the clip, turn the camera on through the UI, watch the server state. -> observations."""
    args = [CHROME, "--no-first-run", "--no-default-browser-check", "--disable-extensions", "--mute-audio",
            f"--user-data-dir={profile}", f"--remote-debugging-port={CDP_PORT}", "--window-size=1280,800",
            "--use-fake-ui-for-media-stream", "--use-fake-device-for-media-stream",
            f"--use-file-for-fake-video-capture={y4m}", "--autoplay-policy=no-user-gesture-required",
            "--enable-unsafe-swiftshader", f"{BASE}/?nospeak=1"]
    if os.environ.get("PC_E2E_HEADFUL") != "1":
        args[1:1] = ["--headless=new", "--hide-scrollbars"]
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=tempfile.gettempdir())
    obs = collections.defaultdict(int)
    obs.update(acts=collections.Counter(), src=collections.Counter(), hand_src=collections.Counter(), pose_len=set(),
               reps_max=0, open_flips=0, cues=[], targets=collections.Counter(), cam_status="", console=[])
    try:
        page = None
        for _ in range(60):
            try:
                async with s.get(f"http://127.0.0.1:{CDP_PORT}/json") as r:
                    page = next(t for t in await r.json() if t.get("type") == "page" and BASE in t.get("url", ""))
                break
            except Exception:
                await asyncio.sleep(0.5)
        if page is None:
            raise RuntimeError("Chrome did not open the app (is the debugging port free?)")
        cdp = CDP(await s.ws_connect(page["webSocketDebuggerUrl"], max_msg_size=0))
        await cdp.call("Runtime.enable")
        for _ in range(60):                                  # the page connected to the server
            if await cdp.js("!!(window.PC && window.PC.state && window.PC.state.mode)"):
                break
            await asyncio.sleep(0.5)
        mode = WATCH_MODE[kind]
        if mode:                                             # the Watch card's mode chip, as a person would click it
            await cdp.js(f"(document.querySelector('.pcw-chip[data-mode=\"{mode}\"]') || {{click(){{}}}}).click()")
        # Settings -> Camera -> "Use this device's camera" (off by default)
        await cdp.js("(() => { const c = document.getElementById('setPhoneCam'); if (!c.checked) c.click(); return c.checked; })()")
        app = await s.ws_connect(BASE + "/ws", heartbeat=None)
        st, prev_open, t_first = {}, None, None
        t0 = time.time()
        limit = 6.0 + min(2.0 * seconds, 40.0)               # warm-up (pose model loads on the first frame) + the clip twice
        while time.time() - t0 < limit:
            m = await asyncio.wait_for(app.receive(), 10)
            if m.type != aiohttp.WSMsgType.TEXT:
                if m.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                    break
                continue
            st = json.loads(m.data)
            h, cp, w = st.get("human") or {}, st.get("copresence") or {}, st.get("watch") or {}
            obs["states"] += 1
            obs["src"][st.get("source")] += 1
            if not h.get("visible") or h.get("src") != "phone":   # the synthetic no-camera dancer doesn't count
                continue
            if t_first is None:
                t_first = time.time() - t0
            obs["visible"] += 1
            obs["pose_len"].add(len(h.get("pose") or []))
            if TRACE and time.time() - obs["trace_t"] >= 0.25:
                obs["trace_t"] = time.time()
                pz = h.get("pose") or []
                hip = (pz[23][2] + pz[24][2]) / 2 - (pz[27][2] + pz[28][2]) / 2 if len(pz) == 33 and all(pz[i] for i in (23, 24, 27, 28)) else None
                print(f"      t={time.time() - t0:5.1f} {h.get('activity'):>7} {str(h.get('phase')):>6} reps={h.get('reps')} "
                      f"hip_h={hip if hip is None else round(hip, 3)} open={cp.get('hand_open')} offer={cp.get('offer')} "
                      f"point={'armed' if (cp.get('pointing') or {}).get('armed') else ('yes' if cp.get('pointing') else '-')}", flush=True)
            obs["acts"][h.get("activity")] += 1
            obs["reps_max"] = max(obs["reps_max"], h.get("reps") or 0)
            for hd in (h.get("hands") or {}).values():
                obs["hand_src"][hd.get("src")] += 1
                if hd.get("src") == "hands" and hd.get("open"):
                    obs["hands_open"] += 1
            ho = cp.get("hand_open") or {}
            is_open = any(bool(v) for v in ho.values())
            obs["hand_open"] += is_open
            if prev_open is not None and is_open != prev_open:
                obs["open_flips"] += 1
            prev_open = is_open
            obs["offer"] += bool(cp.get("offer"))
            p = cp.get("pointing")
            if p:
                obs["pointing"] += 1
                obs["armed"] += bool(p.get("armed"))
                obs["hit"] += p.get("hit") is not None
                if p.get("target"):
                    obs["targets"][p["target"]] += 1
            obs["marker_p"] = obs["marker_p"] or any(mk.get("id") == "P" for mk in st.get("markers") or [])
            last = (w.get("coach") or {}).get("last") or {}
            if last.get("text") and last["text"] not in obs["cues"]:
                obs["cues"].append(last["text"])
        obs["t_first"] = t_first
        obs["watch_mode"] = (st.get("watch") or {}).get("mode")
        obs["cam_status"] = await cdp.js("(() => { const v = window.PC && window.PC.camVideo; "
                                         "return v ? `streaming ${v.videoWidth}x${v.videoHeight}` : 'no camera video'; })()")
        obs["hands_model"] = await cdp.js("!!document.querySelector('.pcw-live') && document.querySelector('.pcw-live').textContent")
        obs["console"] = cdp.console[-5:]
        await app.close()
        cdp.task.cancel()
        await cdp.ws.close()
    finally:
        proc.terminate()
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
    return obs


def report(kind, name, o):
    span = f"{o['visible']}/{o['states']} states with a person"
    print(f"      observed [{name}]: {span}, first after {o['t_first'] if o['t_first'] is None else round(o['t_first'], 1)} s; "
          f"camera: {o['cam_status']!r}; Watch card: {o.get('hands_model')!r}; source {dict(o['src'])}; pose lengths {sorted(o['pose_len'])}", flush=True)
    print(f"      hand_open {o['hand_open']} states, {o['open_flips']} flips; offer {o['offer']}; browser Hands "
          f"{dict(o['hand_src'])} (open {o['hands_open']}); pointing {o['pointing']} (armed {o['armed']}, table hit "
          f"{o['hit']}, targets {dict(o['targets'])}, marker P {bool(o['marker_p'])})", flush=True)
    print(f"      activity {dict(o['acts'])}; reps max {o['reps_max']}; watch mode {o['watch_mode']}; coach cues {o['cues'][:4]}",
          flush=True)
    if o["console"]:
        print(f"      page console: {o['console']}", flush=True)


async def main(clips):
    tmp = Path(tempfile.mkdtemp(prefix="pc_e2e_cam_"))
    srv = subprocess.Popen([str(ROOT / ".venv/bin/python"), "-m", "pps.app", "--fake", "--port", str(PORT)], cwd=ROOT,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        async with aiohttp.ClientSession() as s:
            for _ in range(120):
                try:
                    async with s.get(BASE + "/") as r:
                        if r.status == 200:
                            break
                except Exception:
                    pass
                await asyncio.sleep(0.5)
            else:
                check("server starts", False, f"no answer on {BASE}")
                return
            for i, (kind, spec) in enumerate(clips):
                path, start, dur = parse_clip(spec)
                name = f"{kind}:{path.name}"
                if not path.exists():
                    check(f"{name} exists", False, str(path))
                    continue
                y4m = tmp / f"clip{i}.y4m"
                seconds = to_y4m(path, start, dur, y4m)
                profile = tmp / f"chrome{i}"
                try:
                    o = await run_clip(s, kind, y4m, seconds, profile)
                except Exception as e:
                    check(f"{name} ran", False, repr(e)[:160])
                    continue
                finally:
                    y4m.unlink(missing_ok=True)
                    shutil.rmtree(profile, ignore_errors=True)
                report(kind, name, o)
                check(f"{name}: browser camera -> person visible (source phone)",
                      o["visible"] >= 5 and o["src"].get("phone", 0) >= 5, f"{o['visible']} states, source {dict(o['src'])}")
                check(f"{name}: pose has 33 world landmarks", o["pose_len"] == {33}, str(sorted(o["pose_len"])))
                if kind == "point":
                    check(f"{name}: pointing target set at least once", o["armed"] >= 1 and (o["targets"] or o["hit"]),
                          f"pointing {o['pointing']}, armed {o['armed']}, hit {o['hit']}, targets {dict(o['targets'])}")
                if kind == "hand":
                    check(f"{name}: hand_open true at least once", o["hand_open"] >= 1 or o["hands_open"] >= 1,
                          f"pose hand_open {o['hand_open']} ({o['open_flips']} flips), browser Hands open {o['hands_open']}")
                if kind == "squat":
                    check(f"{name}: >= 2 reps in coach mode", o["reps_max"] >= 2 and o["watch_mode"] == "coach",
                          f"reps {o['reps_max']}, mode {o['watch_mode']}, activity {dict(o['acts'])}")
                # back to a clean server between clips: watch off, the camera gone
                async with s.ws_connect(BASE + "/ws") as ws:
                    await ws.send_str(json.dumps({"type": "watch", "op": "mode", "mode": "off"}))
                    await asyncio.sleep(1.5)
    finally:
        srv.terminate()
        try:
            srv.wait(10)
        except subprocess.TimeoutExpired:
            srv.kill()
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    clips = [(kind, os.environ[var]) for var, kind in KINDS if os.environ.get(var)]
    if not clips:
        print("SKIP  e2e camera: no clip. Set PC_CAM_CLIP / PC_CAM_POINT / PC_CAM_HAND / PC_CAM_SQUAT "
              "(see the docstring of scripts/e2e_camera.py for how to record them).")
        sys.exit(0)
    if not CHROME:
        print("SKIP  e2e camera: Google Chrome / Chromium not found (set PC_CHROME).")
        sys.exit(0)
    if not ffmpeg_exe():
        print("SKIP  e2e camera: ffmpeg not found.")
        sys.exit(0)
    asyncio.run(main(clips))
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} passed")
    sys.exit(1 if bad else 0)
