"""Browser UX checks in headless Chrome over CDP (the parts of the UI a protocol self test can't see).

    .venv/bin/python scripts/e2e_ui.py

Checks, with real input events (Input.dispatchMouseEvent) and real layout:
  * zoom: mouse-notch and trackpad wheel deltas zoom the 3D stage smoothly and proportionally at devicePixelRatio
    0.9 and 2 (three r160 OrbitControls jumped straight to min / max distance below DPR 1), flings ease to the limits,
    nothing snaps back; after the robot proposes a path, the wheel still zooms away from the handles and only a wheel
    turn right on a handle changes its height;
  * gestures guide: opens fully inside the viewport at 1440×900, 1280×720 and 390×844 (More menu on phones), says the
    camera is needed and offers to turn it on;
  * theme: Light and Dark apply to the page, the 3D stage and dialogs; key text passes WCAG AA (4.5:1) in both;
  * help: the ? button opens the guide (14 sections), the 한국어 toggle switches it, the first-run tip dismisses;
  * voice: per-line language (Hangul -> ko-KR), text cleaned for speech, sentence chunks, best-voice ranking, a quiet
    fallback without a working OpenAI key, Settings → Voice pickers;
  * camera: "Turn on camera" streams this browser's (fake) camera: the server's source becomes "phone", the self-view
    plays, "Camera is off" goes away; with PC_E2E_UI_Y4M=<a y4m of a person> the person also appears across the
    table (state.copresence.visible + joints);
  * phone camera: /phone in a second tab (phone user agent, fake camera) streams and the laptop says
    "Phone connected · N fps"; the "Use my phone" dialog shows a QR code and the https /phone address;
  * no errors in the page console.

Skips cleanly (exit 0) when Google Chrome is not installed. Starts its own server (--fake --https) on PC_E2E_UI_PORT
(8953, https +1) unless PC_E2E_UI_BASE points at a running one. CDP port PC_E2E_UI_CDP (9351).
PC_E2E_UI_HEADFUL=1 shows the browser; PC_E2E_UI_SHOTS=dir saves screenshots (1440×900 and 390×844, both themes).
"""
import asyncio
import base64
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
PORT = int(os.environ.get("PC_E2E_UI_PORT", "8953"))
BASE = os.environ.get("PC_E2E_UI_BASE") or f"http://127.0.0.1:{PORT}"
CDP_PORT = int(os.environ.get("PC_E2E_UI_CDP", "9351"))
Y4M = os.environ.get("PC_E2E_UI_Y4M")
SHOTS = os.environ.get("PC_E2E_UI_SHOTS")
CHROME = os.environ.get("PC_CHROME") or next((p for p in (
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    shutil.which("google-chrome") or "", shutil.which("google-chrome-stable") or "",
    shutil.which("chromium") or "", shutil.which("chromium-browser") or "") if p and Path(p).exists()), None)
IPHONE_UA = ("Mozilla/5.0 (iPhone; CPU iPhone OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) "
             "Version/17.5 Mobile/15E148 Safari/604.1")
results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)


class CDP:
    def __init__(self, ws):
        self.ws, self.mid, self.pending, self.errors = ws, 0, {}, []
        self.task = asyncio.create_task(self._pump())

    async def _pump(self):
        async for m in self.ws:
            if m.type != aiohttp.WSMsgType.TEXT:
                continue
            ev = json.loads(m.data)
            if "id" in ev and ev["id"] in self.pending:
                self.pending.pop(ev["id"]).set_result(ev)
            elif ev.get("method") == "Runtime.consoleAPICalled" and ev["params"].get("type") == "error":
                self.errors.append(" ".join(str(a.get("value", a.get("description", "")))[:200] for a in ev["params"]["args"]))
            elif ev.get("method") == "Runtime.exceptionThrown":
                d = ev["params"]["exceptionDetails"]
                self.errors.append("exception: " + str((d.get("exception") or {}).get("description", d.get("text", "")))[:300])
            elif ev.get("method") == "Log.entryAdded" and ev["params"]["entry"].get("level") == "error":
                e = ev["params"]["entry"]
                self.errors.append(f"{e.get('source')}: {e.get('text', '')[:160]} {e.get('url', '')[:80]}")

    async def call(self, method, params=None, timeout=30):
        self.mid += 1
        fut = asyncio.get_running_loop().create_future()
        self.pending[self.mid] = fut
        await self.ws.send_str(json.dumps({"id": self.mid, "method": method, "params": params or {}}))
        r = await asyncio.wait_for(fut, timeout)
        if "error" in r:
            raise RuntimeError(f"{method}: {r['error']}")
        return r.get("result", {})

    async def js(self, expr):
        r = await self.call("Runtime.evaluate", {"expression": expr, "returnByValue": True, "awaitPromise": True})
        if r.get("exceptionDetails"):
            raise RuntimeError("js: " + str(r["exceptionDetails"].get("exception", {}).get("description", r["exceptionDetails"]))[:300])
        return (r.get("result") or {}).get("value")

    async def size(self, w, h, dpr=1, mobile=False):
        await self.call("Emulation.setDeviceMetricsOverride", {"width": w, "height": h, "deviceScaleFactor": dpr, "mobile": mobile})

    async def wheel(self, x, y, dy, ctrl=False):
        p = {"type": "mouseWheel", "x": x, "y": y, "deltaX": 0, "deltaY": dy}
        if ctrl:
            p["modifiers"] = 2
        await self.call("Input.dispatchMouseEvent", p)

    async def move(self, x, y):
        await self.call("Input.dispatchMouseEvent", {"type": "mouseMoved", "x": x, "y": y})

    async def shot(self, name):
        if not SHOTS:
            return
        Path(SHOTS).mkdir(parents=True, exist_ok=True)
        r = await self.call("Page.captureScreenshot", {"format": "png"})
        (Path(SHOTS) / f"{name}.png").write_bytes(base64.b64decode(r["data"]))


async def until(cdp, expr, timeout=20.0, every=0.25):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            v = await cdp.js(expr)
            if v:
                return v
        except Exception:
            pass
        await asyncio.sleep(every)
    return None


async def open_tab(s, url):
    async with s.put(f"http://127.0.0.1:{CDP_PORT}/json/new?{url}") as r:
        t = await r.json()
    cdp = CDP(await s.ws_connect(t["webSocketDebuggerUrl"], max_msg_size=0))
    for d in ("Runtime", "Page", "Log"):
        await cdp.call(f"{d}.enable")
    return cdp, t["id"]


async def load(cdp, w=1440, h=900, dpr=1, mobile=False, query="nospeak=1&tip=0"):
    await cdp.size(w, h, dpr, mobile)
    await cdp.call("Page.navigate", {"url": f"{BASE}/?{query}"})
    ok = await until(cdp, "!!(window.PC3D && window.PC3D.sceneReady && window.PC && window.PC.state && window.PC.state.mode && document.getElementById('gesturesBtn'))", 60)
    await asyncio.sleep(0.8)
    return ok


DIST = "window.PC3D.distance"


async def wheel_series(cdp, x, y, deltas, wait=0.05, settle=0.6, ctrl=False):
    out = []
    for d in deltas:
        await cdp.wheel(x, y, d, ctrl)
        await asyncio.sleep(wait)
        out.append(await cdp.js(DIST))
    await asyncio.sleep(settle)
    return out, await cdp.js(DIST)


async def canvas_center(cdp, dx=0, dy=0):
    r = json.loads(await cdp.js("JSON.stringify(document.querySelector('#view3d canvas').getBoundingClientRect())"))
    return r["x"] + r["width"] / 2 + dx, r["y"] + r["height"] / 2 + dy, r


# ---------------------------------------------------------------------------------------------- checks
async def zoom_checks(cdp):
    for dpr in (0.9, 2):
        await load(cdp, dpr=dpr)
        await cdp.js("window.PC.send({type: 'traj', op: 'clear'})")
        await asyncio.sleep(0.4)
        x, y, _ = await canvas_center(cdp)
        await cdp.move(x, y)
        lim = await cdp.js("JSON.stringify(window.PC3D.zoomLimits)")
        lo, hi = json.loads(lim)["min"], json.loads(lim)["max"]
        d0 = await cdp.js(DIST)
        seq, d1 = await wheel_series(cdp, x, y, [-100] * 6)
        steps = [b / a for a, b in zip([d0] + seq[:-1], seq) if a]
        check(f"zoom DPR {dpr}: 6 mouse notches zoom in step by step", all(0.75 < r < 0.995 for r in steps) and lo * 1.05 < d1 < d0 * 0.9,
              f"{d0:.2f} -> {d1:.2f}, steps {min(steps):.3f}..{max(steps):.3f}")
        seq, d2 = await wheel_series(cdp, x, y, [100] * 6)
        check(f"zoom DPR {dpr}: and back out to where it started", abs(d2 - d0) < 0.02 * d0, f"{d1:.2f} -> {d2:.2f} (start {d0:.2f})")
        seq, d3 = await wheel_series(cdp, x, y, [-4, -6, -8, -10, -8, -6] * 2, wait=0.016)
        check(f"zoom DPR {dpr}: trackpad deltas zoom a little, continuously", 0.6 * d0 < d3 < 0.995 * d0 and len({round(v, 4) for v in seq}) >= 8,
              f"{d0:.2f} -> {d3:.2f} over 12 events")
        seq, d4 = await wheel_series(cdp, x, y, [-800] * 10, wait=0.016)
        check(f"zoom DPR {dpr}: a fling eases to the near limit, never past it", abs(d4 - lo) < 1e-3 and len({round(v, 3) for v in seq}) >= 4 and min(seq) >= lo - 1e-6,
              f"-> {d4:.3f} via {len({round(v, 3) for v in seq})} distinct steps (min {lo})")
        seq, d5 = await wheel_series(cdp, x, y, [1500] * 12, wait=0.016)
        await asyncio.sleep(1.0)
        d6 = await cdp.js(DIST)
        check(f"zoom DPR {dpr}: far limit, and no snapping back afterwards", abs(d5 - hi) < 1e-3 and abs(d6 - d5) < 1e-6, f"-> {d5:.3f}, 1 s later {d6:.3f}")
    # after a command: the robot proposes a path (P2 trajectory from words)
    await cdp.js("window.PC3D.zoomBy(0.45)")
    await asyncio.sleep(0.6)
    spec = {"label": "over the vase to the left", "keypoints": [{"rel": "above", "of": "mug", "dz": 0.08}, {"region": "left", "dz": 0.08}]}
    await cdp.js(f"window.PC.send({json.dumps({'type': 'traj', 'op': 'propose', 'id': 'e2e-ui', 'spec': spec, 'task_id': 'e2e-ui-plan'})})")
    keys = await until(cdp, "(window.PC.state.traj && window.PC.state.traj.ghost && window.PC.state.traj.ghost.keys || []).length >= 2 && JSON.stringify(window.PC.state.traj.ghost.keys)", 15)
    await asyncio.sleep(1.2)
    keys = json.loads(await cdp.js("JSON.stringify(window.PC.state.traj.ghost.keys)")) if keys else []
    check("after a proposed path: it is on screen", len(keys) >= 2, f"{len(keys)} handles")
    if len(keys) >= 2:
        x, y, _ = await canvas_center(cdp)
        scr = [json.loads(await cdp.js(f"JSON.stringify(window.PC3D.project({k[0]},{k[1]},{k[2]}))")) for k in keys]
        hx, hy = scr[1]
        await cdp.move(hx + 16, hy + 12)
        d0 = await cdp.js(DIST)
        _, d1 = await wheel_series(cdp, hx + 16, hy + 12, [-100] * 3)
        check("after a proposed path: the wheel next to a handle zooms", d1 < 0.9 * d0, f"{d0:.2f} -> {d1:.2f}")
        await cdp.move(x, y)
        _, d2 = await wheel_series(cdp, x, y, [-100] * 2)
        check("after a proposed path: the wheel in the middle zooms", d2 < 0.95 * d1, f"{d1:.2f} -> {d2:.2f}")
        scr = [json.loads(await cdp.js(f"JSON.stringify(window.PC3D.project({k[0]},{k[1]},{k[2]}))")) for k in keys]
        i = len(scr) // 2                                  # a mid-path waypoint (the ends are tied to the object / place)
        hx, hy = scr[i]
        z0 = keys[i][2]
        await cdp.move(hx, hy)
        await asyncio.sleep(0.2)
        under = await cdp.js(f"(() => {{ const e = document.elementFromPoint({hx}, {hy}); return e ? (e.id || e.className || e.tagName) : null; }})()")
        _, d3 = await wheel_series(cdp, hx, hy, [-100] * 2, settle=0.5)
        await until(cdp, "(window.PC.state.traj.reply || {}).op === 'edit'", 6)      # the server re-checks the edited path
        k1 = json.loads(await cdp.js("JSON.stringify(window.PC.state.traj.ghost.keys)"))
        up = [j for j in range(min(len(k1), len(keys))) if k1[j][2] > keys[j][2] + 0.004]   # the handle under the pointer
        check("after a proposed path: the wheel right on a handle raises it, the view stays", abs(d3 - d2) < 1e-6 and len(up) == 1,
              f"handle {up} z {[round(keys[j][2], 3) for j in up]} -> {[round(k1[j][2], 3) for j in up]} (aimed at {i}, z {z0:.3f}, under the pointer: {under}), distance {d2:.3f} -> {d3:.3f}"
              + ("" if up else " reply " + str(await cdp.js("JSON.stringify(window.PC.state.traj.reply)"))[:160]))
    await cdp.js("window.PC.send({type: 'traj', op: 'clear'})")


async def gestures_checks(cdp):
    for w, h, mobile in ((1440, 900, False), (1280, 720, False), (390, 844, True)):
        await load(cdp, w, h, 2 if mobile else 1, mobile)
        if mobile:
            await cdp.js("document.getElementById('moreMenu').open = true")
            await asyncio.sleep(0.2)
            await cdp.js("document.querySelector(\".more-menu [data-act='gestures']\").click()")
        else:
            await cdp.js("document.getElementById('gesturesBtn').click()")
        await asyncio.sleep(0.5)
        r = json.loads(await cdp.js("JSON.stringify(document.getElementById('pcgCard').getBoundingClientRect())"))
        txt = await cdp.js("document.getElementById('pcgCard').innerText")
        inside = r["width"] > 200 and r["left"] >= 0 and r["top"] >= 0 and r["right"] <= w + 0.5 and r["bottom"] <= h + 0.5
        dock = json.loads(await cdp.js("JSON.stringify(document.getElementById('dock').getBoundingClientRect())"))
        clear = not mobile or r["bottom"] <= dock["top"] + 0.5
        check(f"gestures guide {w}×{h}: fully on screen, not clipped", inside and clear and "Stop" in txt and "Go on" in txt,
              f"{r['left']:.0f},{r['top']:.0f} {r['width']:.0f}×{r['height']:.0f}")
        check(f"gestures guide {w}×{h}: says the camera is needed and offers to turn it on",
              "camera" in txt.lower() and await cdp.js("!document.querySelector('#pcgCard .pcg-cam-on').hidden"), txt.split("\n")[1][:60] if "\n" in txt else txt[:60])
        await cdp.js("window.PCGestures.close()")


CONTRAST = r"""(() => {
  const parse = (c) => { const m = c.match(/rgba?\(([^)]+)\)/); if (!m) return null; const p = m[1].split(/[ ,/]+/).filter(Boolean).map(Number); return [p[0], p[1], p[2], p.length > 3 ? p[3] : 1]; };
  const lum = (c) => { const f = (v) => { v /= 255; return v <= 0.03928 ? v / 12.92 : Math.pow((v + 0.055) / 1.055, 2.4); }; return 0.2126 * f(c[0]) + 0.7152 * f(c[1]) + 0.0722 * f(c[2]); };
  const blend = (top, bot) => [0, 1, 2].map((i) => top[i] * top[3] + bot[i] * (1 - top[3])).concat(1);
  function bg(el) { const stack = []; for (let e = el; e; e = e.parentElement) { const c = parse(getComputedStyle(e).backgroundColor); if (c && c[3] > 0) { stack.push(c); if (c[3] >= 1) break; } }
    let out = parse(getComputedStyle(document.body).backgroundColor) || [255, 255, 255, 1]; for (let i = stack.length - 1; i >= 0; i--) out = blend(stack[i], out); return out; }
  const sel = ['.pi-title', '.pi-sub', '.pi-ex', '.col-title', '#says li', '.seg-btn[aria-pressed="true"]', '.cmd-label', '.feed-empty', '.dock .pcui-tool', '#planChip', '.brand .name', '.plan-chip', '.stop-btn'];
  const out = [];
  for (const s of sel) { const el = document.querySelector(s); if (!el || !el.offsetParent) continue; const cs = getComputedStyle(el); const fg = parse(cs.color); if (!fg) continue;
    const b = bg(el), f = blend(fg, b), L1 = lum(f), L2 = lum(b); out.push([s, Math.round(((Math.max(L1, L2) + 0.05) / (Math.min(L1, L2) + 0.05)) * 100) / 100]); }
  return JSON.stringify(out);
})()"""


async def theme_checks(cdp):
    await load(cdp)
    for theme in ("light", "dark"):
        await cdp.js(f"window.PCTheme.set('{theme}')")
        await asyncio.sleep(0.5)
        attr = await cdp.js("document.documentElement.dataset.theme")
        body = await cdp.js("getComputedStyle(document.body).backgroundColor")
        stage = await cdp.js("window.PC3D.scene && '#' + window.PC3D.scene.background.getHexString()")
        dark_bg = body.startswith("rgb(15, 16, 19")
        check(f"theme {theme}: applied to the page and the 3D stage", attr == theme and (dark_bg == (theme == "dark"))
              and (stage == "#16171b") == (theme == "dark"), f"{body} stage {stage}")
        pairs = json.loads(await cdp.js(CONTRAST))
        low = [p for p in pairs if p[1] < 4.5]
        check(f"theme {theme}: text contrast ≥ 4.5:1 (WCAG AA)", len(pairs) >= 8 and not low, f"{len(pairs)} checked, min {min(p[1] for p in pairs) if pairs else '-'}{' low: ' + str(low) if low else ''}")
        await cdp.shot(f"e2e_ui_desk_{theme}")
    await cdp.js("window.PCTheme.set('system')")
    stored = await cdp.js("localStorage.getItem('pc.theme')")
    check("theme: the choice is remembered (localStorage pc.theme)", stored == "system", str(stored))


async def help_checks(cdp):
    await load(cdp, query="nospeak=1")
    await cdp.js("localStorage.removeItem('pc.tipSeen'); localStorage.setItem('pc.helpLang', 'en')")
    await load(cdp, query="nospeak=1")
    tip = await cdp.js("!document.getElementById('tipStrip').hidden && document.getElementById('tipStrip').innerText")
    check("first-run tip strip is shown", bool(tip), str(tip)[:70])
    await cdp.js("document.getElementById('helpBtn').click()")
    await asyncio.sleep(0.4)
    n = await cdp.js("document.getElementById('helpDlg').open && document.querySelectorAll('#helpDlg .help-sec').length")
    t_en = await cdp.js("document.getElementById('helpTitle').textContent")
    await cdp.js("document.querySelector('#helpDlg [data-lang=\"ko\"]').click()")
    await asyncio.sleep(0.2)
    t_ko = await cdp.js("document.getElementById('helpTitle').textContent")
    check("help: the ? button opens the guide with every section", n == 14, f"{n} sections, '{t_en}'")
    check("help: 한국어 toggle", any("가" <= ch <= "힣" for ch in (t_ko or "")), str(t_ko))
    r = json.loads(await cdp.js("JSON.stringify(document.getElementById('helpDlg').getBoundingClientRect())"))
    check("help: the sheet fits the window", r["top"] >= 0 and r["bottom"] <= 900.5 and r["right"] <= 1440.5, f"{r['width']:.0f}×{r['height']:.0f}")
    await cdp.js("document.querySelector('#helpDlg [data-lang=\"en\"]').click(); document.getElementById('helpDlg').close()")
    gone = await cdp.js("document.getElementById('tipStrip').hidden && localStorage.getItem('pc.tipSeen') === '1'")
    check("first-run tip strip goes away once the guide was opened, and stays away", bool(gone), "")
    await cdp.shot("e2e_ui_help")


VOICE = r"""(() => {
  const V = window.PCVoice, fake = (name, lang, extra) => Object.assign({ name, lang, voiceURI: name, localService: true, default: false }, extra || {});
  const ko = [fake("Yuna", "ko-KR"), fake("Yuna (Premium)", "ko-KR"), fake("Google 한국어", "ko-KR", { localService: false })];
  const en = [fake("Albert", "en-US"), fake("Fred", "en-US"), fake("Samantha", "en-US"), fake("Ava (Enhanced)", "en-US"), fake("Daniel", "en-GB")];
  const best = (vs, l) => vs.slice().sort((a, b) => V.voiceScore(b, l) - V.voiceScore(a, l))[0].name;
  const long = "This is a long sentence about the robot's plan. ".repeat(12);
  return JSON.stringify({
    lang: [V.speechLang("안녕하세요, 머그를 옮길게요."), V.speechLang("Put the mug on the right.")],
    clean: V.speechText("**Done** → I moved the `mug` 🎉 (speed ×0.7). See [docs](https://x.y/z)."),
    chunks: V.speechChunks(long).map((c) => c.length),
    ko: best(ko, "ko-KR"), en: best(en, "en-US"), novelty: V.voiceScore(en[0], "en-US"), wrongLang: V.voiceScore(ko[0], "en-US"),
  });
})()"""


async def voice_checks(cdp):
    await load(cdp)
    v = json.loads(await cdp.js(VOICE))
    check("voice: the language is detected per line (Hangul -> ko-KR)", v["lang"] == ["ko-KR", "en-US"], str(v["lang"]))
    c = v["clean"]
    check("voice: markdown, emoji, arrows, code and links are not read out", not any(x in c for x in "*`→🎉[]()") and "https" not in c
          and "times 0.7" in c and "Done" in c and "mug" in c, repr(c))
    check("voice: long text is spoken sentence by sentence (≤ 180 chars a chunk)", len(v["chunks"]) > 2 and max(v["chunks"]) <= 181, str(v["chunks"][:6]))
    check("voice: the best voice wins (Premium / Enhanced / Google), novelty voices never", v["ko"] in ("Yuna (Premium)", "Google 한국어")
          and v["en"] == "Ava (Enhanced)" and v["novelty"] < 0 and v["wrongLang"] < 0, f"ko {v['ko']}, en {v['en']}")
    before = len(cdp.errors)
    await cdp.js("window.PC.speak('안녕하세요. 음성 테스트입니다.')")
    await asyncio.sleep(2.5)
    tts = await cdp.js("window.PCVoice.ttsOK")
    errs = cdp.errors[before:]
    check("voice: without a working OpenAI key it falls back quietly (no console errors)", not errs, f"server voice in use: {tts}; {errs[:2]}")
    await cdp.js("document.getElementById('settingsBtn').click()")
    await asyncio.sleep(0.4)
    sel = await cdp.js("document.querySelectorAll('#voicePickers select').length")
    note = await cdp.js("document.getElementById('voiceNote').textContent")
    check("voice: Settings → Voice has a voice per language and a Test button", sel == 2 and await cdp.js("document.querySelectorAll('[data-voice-test]').length") == 2, str(note)[:80])
    await cdp.js("document.getElementById('settings').close()")


async def camera_checks(cdp, s):
    await load(cdp, query="nospeak=1&tip=0")
    await cdp.js("localStorage.removeItem('pc.phoneCam')")
    off = await cdp.js("!document.getElementById('camEmpty').hidden && document.getElementById('camEmpty').innerText")
    check("camera is off by default, and the panel says how to turn it on", bool(off) and "Turn on camera" in off,
          str(off)[:80].replace("\n", " | ") if off else await cdp.js("JSON.stringify({source: window.PC.state.source, on: window.PC.camera.on, status: window.PC.camera.status})"))
    await cdp.js("document.getElementById('camBtnM').click()")
    src = await until(cdp, "window.PC.state.source === 'phone' && window.PC.camera.on", 15)
    playing = await until(cdp, "(() => { const v = document.getElementById('camSelf'); return !v.hidden && v.videoWidth > 0 && !v.paused; })()", 8)
    gone = await cdp.js("document.getElementById('camEmpty').hidden")
    check("“Turn on camera”: one click streams this browser's camera to the robot", bool(src), str(await cdp.js("window.PC.state.source")))
    check("…you see yourself right away (self-view plays), the off card is gone", bool(playing) and gone, "")
    chip = await until(cdp, "(() => { const c = document.getElementById('phoneCamChip'); return !c.hidden && c.textContent; })()", 5)
    check("…the camera chip says it is on, with its fps", bool(chip) and "fps" in str(chip), str(chip))
    if Y4M:
        vis = await until(cdp, "(() => { const c = window.PC.state.copresence || {}; return c.visible && c.joints && Object.keys(c.joints).length > 8; })()", 20)
        mesh = await cdp.js("(() => { const g = window.PC3D.scene.getObjectByName('layer:copresence'); let n = 0; if (g) g.traverse((o) => { if (o.isMesh && o.visible) n++; }); return n; })()")
        check("…and you appear across the table (co-presence avatar in the 3D stage)", bool(vis) and mesh > 8, f"{mesh} visible avatar meshes")
        await cdp.shot("e2e_ui_camera_on")
    await cdp.js("document.getElementById('camOffBtn').click()")
    back = await until(cdp, "!document.getElementById('camEmpty').hidden && !window.PC.camera.on", 6)
    check("turning the camera off brings the off card back", bool(back), "")
    # the phone: /phone in a second tab with a phone user agent and the fake camera
    await cdp.js("window.PC.camera.phone()")
    await asyncio.sleep(1.0)
    url = await cdp.js("document.getElementById('phoneUrl').textContent")
    qr = await cdp.js("!!document.querySelector('#phoneQr svg path')")
    check("“Use my phone”: a QR code and the https /phone address", qr and str(url).startswith("https://") and str(url).endswith("/phone"), str(url))
    https = str(url).replace(str(url).split("/phone")[0].split("://")[1].split(":")[0], "127.0.0.1", 1) if url else None
    ph, tid = await open_tab(s, "about:blank")
    try:
        await ph.call("Network.enable")
        await ph.call("Network.setUserAgentOverride", {"userAgent": IPHONE_UA})
        await ph.size(390, 844, 2, True)
        await ph.call("Page.navigate", {"url": https})
        await until(ph, "document.readyState === 'complete' && !!document.getElementById('startBtn')", 15)
        await asyncio.sleep(2.0)                           # it starts by itself when the camera permission is already granted
        await ph.js("(() => { if (document.getElementById('live').hidden) document.getElementById('startBtn').click(); return true; })()")
        streaming = await until(ph, "!document.getElementById('live').hidden", 10)
        check("/phone: Start camera streams (fake camera, phone user agent)", bool(streaming), str(await ph.js("document.getElementById('statusMain').textContent")))
        await cdp.call("Page.bringToFront")              # a background tab doesn't render (no animation frames)
        live = await until(cdp, "(() => { const p = document.getElementById('phoneStatus'); return p.dataset.state === 'live' && p.innerText; })()", 20)
        check("the phone streams: the laptop says “Phone connected · N fps”", bool(live) and "fps" in str(live), str(live)[:80])
        chip = await cdp.js("document.getElementById('phoneCamChip').textContent")
        check("…and the camera chip says so too", "Phone connected" in str(chip), str(chip))
        if ph.errors:
            check("phone page console has no errors", False, "; ".join(ph.errors[:3]))
        await cdp.shot("e2e_ui_phone_dialog")
    finally:
        await cdp.js("document.getElementById('phoneDlg').close()")
        try:
            async with s.get(f"http://127.0.0.1:{CDP_PORT}/json/close/{tid}"):
                pass
        except Exception:
            pass


async def screenshots(cdp):
    if not SHOTS:
        return
    for theme in ("light", "dark"):
        for name, w, h, dpr, mob in (("desk", 1440, 900, 1, False), ("phone", 390, 844, 2, True)):
            await load(cdp, w, h, dpr, mob)
            await cdp.js(f"window.PCTheme.set('{theme}')")
            await asyncio.sleep(0.6)
            await cdp.shot(f"e2e_ui_{name}_{theme}_home")
    await cdp.js("window.PCTheme.set('system')")


async def main():
    if not CHROME:
        print("SKIP  e2e_ui  Google Chrome not found (set PC_CHROME)")
        return 0
    tmp = Path(tempfile.mkdtemp(prefix="pc_e2e_ui_"))
    srv = None
    if not os.environ.get("PC_E2E_UI_BASE"):
        srv = subprocess.Popen([str(ROOT / ".venv/bin/python"), "-m", "pps.app", "--fake", "--https", "--port", str(PORT),
                                "--https-port", str(PORT + 1)], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    args = [CHROME, "--no-first-run", "--no-default-browser-check", "--disable-extensions", "--mute-audio",
            f"--user-data-dir={tmp / 'profile'}", f"--remote-debugging-port={CDP_PORT}", "--window-size=1440,900",
            "--use-fake-ui-for-media-stream", "--use-fake-device-for-media-stream", "--ignore-certificate-errors",
            "--autoplay-policy=no-user-gesture-required", "--enable-unsafe-swiftshader", "about:blank"]
    if Y4M:
        args.insert(-1, f"--use-file-for-fake-video-capture={Y4M}")
    if os.environ.get("PC_E2E_UI_HEADFUL") != "1":
        args[1:1] = ["--headless=new", "--hide-scrollbars"]
    chrome = None
    try:
        async with aiohttp.ClientSession() as s:
            for _ in range(120):
                try:
                    async with s.get(BASE + "/") as r:
                        if r.status == 200:
                            break
                except Exception:
                    await asyncio.sleep(0.5)
            chrome = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=str(tmp))
            page = None
            for _ in range(60):
                try:
                    async with s.get(f"http://127.0.0.1:{CDP_PORT}/json") as r:
                        page = next((t for t in await r.json() if t.get("type") == "page"), None)
                    if page:
                        break
                except Exception:
                    pass
                await asyncio.sleep(0.5)
            if page is None:
                check("Chrome DevTools reachable", False, f"port {CDP_PORT}")
                return 1
            cdp = CDP(await s.ws_connect(page["webSocketDebuggerUrl"], max_msg_size=0))
            for d in ("Runtime", "Page", "Log"):
                await cdp.call(f"{d}.enable")
            for name, fn in (("zoom", zoom_checks), ("gestures", gestures_checks), ("theme", theme_checks), ("help", help_checks), ("voice", voice_checks)):
                try:
                    await fn(cdp)
                except Exception as e:
                    check(f"{name} checks ran", False, repr(e)[:200])
            try:
                await camera_checks(cdp, s)
            except Exception as e:
                check("camera checks ran", False, repr(e)[:200])
            await screenshots(cdp)
            errs = [e for e in cdp.errors if "favicon" not in e]
            check("no errors in the page console", not errs, "; ".join(errs[:4]))
            cdp.task.cancel()
            await cdp.ws.close()
    finally:
        if chrome:
            chrome.terminate()
            try:
                chrome.wait(10)
            except subprocess.TimeoutExpired:
                chrome.kill()
        if srv:
            srv.terminate()
            try:
                srv.wait(10)
            except subprocess.TimeoutExpired:
                srv.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} passed", flush=True)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
