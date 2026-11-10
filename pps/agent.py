"""Brain (UX 3): a real LLM agent that turns what you say into tool calls against the robot world.

* Model: any OpenAI-compatible **Responses API** (`POST {base}/responses`, function tools). Locally the config comes
  from `.env` (`LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL` = gpt-6.1-terra, `LLM_EFFORT` = medium). With `--byok`
  the server's key is never used: the browser sends `llm: {base_url, key, model, effort?}` with each message, and that
  key lives only for the one turn (never stored, logged or put in state).
* Each turn runs in its own background thread, so the worker loop never waits on the LLM. The agent acts **only through
  the same messages a browser sends** (`push`, i.e. shared.push_cmd) and reads the latest state frame (refreshed by the
  worker in `@ext.tick`, pps/features/agent.py), exactly like a client.
* Tools are generated from the current robot's capabilities (state.world.robot.capabilities, G1 defaults until the World
  workstream lands). Motion tools go through plan preview (`preview: true`) when "confirm before moving" is on: the plan
  card appears, the agent says "here's my plan, say go" and stops; the browser's plan card hears the "go".
* System prompt: pps/world/prompts/base.md + pps/world/prompts/<robot id>.md when they exist, else BUILTIN_PROMPT.

Messages: {type:"agent", text, llm?, confirm?} · {type:"agent", op:"reset"}
State:    agent = {available, busy, model, confirm, byok, events: [{id, kind, text, tool?, args?, ok?}]}
"""
import ipaddress
import json
import os
import socket
import ssl
import threading
import time
import traceback
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from pathlib import Path

import certifi

from .spatial import byok

ROOT = Path(__file__).resolve().parents[1]
PROMPTS = ROOT / "pps" / "world" / "prompts"
DEFAULT_MODEL, DEFAULT_EFFORT = "gpt-6.1-terra", "medium"
MAX_STEPS = 8            # LLM calls per turn
MAX_EVENTS = 40
HISTORY_TURNS = 8        # turns of conversation memory sent back to the model
MAX_OUTPUT_CHARS = 3500  # per tool result
ROBOT_IDS = ("g1", "panda", "ur5e", "xarm7", "iiwa14", "aloha", "apollo", "stretch3", "tidybot", "spot", "h2")
DEFAULT_CAPS = ["pick_place", "reach", "follow_path", "teach_by_showing", "imitate_video", "follow_me", "balance"]
MOTION = ("pick_place", "reach", "follow_path", "play_skill")
WATCH_MODES = ("off", "mirror", "learn", "respond", "assist", "coach")
SSL_CTX = ssl.create_default_context(cafile=certifi.where())
UA = "PhysicsClay-agent/1.0"     # Cloudflare in front of some proxies rejects the default Python-urllib agent

BUILTIN_PROMPT = """You are the brain of a robot in PhysicsClay, a physics simulation the user talks to. You see the world \
only through tools and act only through tools. The user types or speaks (English or Korean); answer in the user's \
language.

How to work:
- The context below already lists the robot, the objects and their positions. Call observe only when you need more \
(markers, zones, the current plan or question, skills) or after something changed.
- To move things, call the motion tools (pick_place, reach, follow_path, play_skill). Name objects the way the context \
does ("mug", "red block"). Targets can be a region (left, right, center, front, back), another object ("next to the \
vase", "on the red block" means beside it), a marker letter the user placed (A-F, or P where they pointed), or a table \
point xyz in metres (robot frame: x forward, y to the robot's left, table top z about 0.74).
- If "confirm before moving" is on, motion tools only PROPOSE a plan: a plan card appears for the user. Then stop and \
tell the user in one sentence what you plan to do and that they can say "go" (or tap Run). Never call confirm_plan \
unless the user has clearly approved the plan shown to them in their latest message.
- If a tool reports the robot is asking which object is meant, ask the user that question (or, if the user's message \
already answers it, call choose_candidate).
- If something can't be done (out of reach, blocked by an obstacle, a keep-out zone), explain why in plain words and \
suggest the alternative the tool gives (a nearer marker, drawing a route, moving the obstacle first).
- "Stay away from X" / "don't go near X" means add_zone. Several objects arranged in one go means set_goal_layout.
- Change the robot or the scene (set_robot, build_scene, add_object, remove_object) only when asked.
- Stop immediately with stop when the user says stop, wait, halt, 멈춰.
- Safety around people is not yours to change: if the robot slowed down or stopped because a person is close \
(safety.stop, or a task paused for safety), never try to get around it (no retries, no other route, no other arm). \
Tell the user plainly why ("I stopped: your hand is very close to my gripper") and that it goes on once they step back.
- Never invent results: report what the tools returned. Keep replies short (one or two sentences, they are spoken \
aloud). No markdown, no lists, no coordinates unless asked.
"""


# ---------------------------------------------------------------------------- config
def _dotenv(name):
    v = os.environ.get(name)
    if v:
        return v.strip()
    env = ROOT / ".env"
    if env.exists():
        for line in env.read_text().splitlines():
            if line.startswith(name + "="):
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    return None


def server_llm():
    """The server's own LLM config from .env, or None (missing, or --byok: the server key is never used)."""
    if byok():
        return None
    base, key = _dotenv("LLM_BASE_URL"), _dotenv("LLM_API_KEY")
    if not base or not key:
        return None
    return {"base_url": base.rstrip("/"), "key": key, "model": _dotenv("LLM_MODEL") or DEFAULT_MODEL,
            "effort": _dotenv("LLM_EFFORT") or DEFAULT_EFFORT}


def visitor_llm(llm):
    """A browser-supplied {base_url, key, model, effort?} -> config, or (None, reason)."""
    if not isinstance(llm, dict) or not str(llm.get("key") or "").strip():
        return None, "no key"
    base = str(llm.get("base_url") or "https://api.openai.com/v1").strip().rstrip("/")
    err = public_url_problem(base)
    if err:
        return None, err
    eff = str(llm.get("effort") or "").strip().lower()
    return {"base_url": base, "key": str(llm["key"]).strip(), "model": str(llm.get("model") or DEFAULT_MODEL).strip(),
            "effort": eff if eff in ("minimal", "low", "medium", "high") else None}, None


def public_url_problem(url):
    """BYOK base URLs come from visitors: https only, and never this machine or a private network (no SSRF)."""
    try:
        u = urllib.parse.urlparse(url)
    except ValueError:
        return "that LLM URL is not valid"
    if u.scheme != "https" or not u.hostname:
        return "the LLM URL must start with https://"
    try:
        addrs = {a[4][0] for a in socket.getaddrinfo(u.hostname, u.port or 443, proto=socket.IPPROTO_TCP)}
    except OSError:
        return f"can't resolve {u.hostname}"
    for a in addrs:
        ip = ipaddress.ip_address(a.split("%")[0])
        if ip.is_private or ip.is_loopback or ip.is_link_local or ip.is_reserved or ip.is_multicast or ip.is_unspecified:
            return "the LLM URL must be a public address"
    return None


class LLMError(Exception):
    pass


def responses_call(cfg, instructions, items, tools, timeout=90):
    """One Responses API call -> list of output items. Errors never contain the key."""
    body = {"model": cfg["model"], "input": [{"role": "system", "content": instructions}] + items, "tools": tools,
            "store": False, "parallel_tool_calls": False}
    if cfg.get("effort"):
        body["reasoning"] = {"effort": cfg["effort"]}
    req = urllib.request.Request(cfg["base_url"] + "/responses", json.dumps(body).encode(),
                                 {"Content-Type": "application/json", "Authorization": "Bearer " + cfg["key"], "User-Agent": UA})
    try:
        with urllib.request.urlopen(req, timeout=timeout, context=SSL_CTX) as r:
            data = json.loads(r.read())
    except urllib.error.HTTPError as e:
        try:
            raw = e.read()[:600].decode(errors="replace")
            j = json.loads(raw)
            detail = (j.get("error") or {}).get("message") if isinstance(j.get("error"), dict) else j.get("error") or raw
        except Exception:
            detail = raw if "raw" in locals() else ""
        raise LLMError(f"the model returned HTTP {e.code}: {str(detail)[:160]}".replace(cfg["key"], "***")) from None
    except (urllib.error.URLError, socket.timeout, TimeoutError) as e:
        raise LLMError(f"couldn't reach the model ({str(getattr(e, 'reason', e))[:100]})") from None
    except ValueError:
        raise LLMError("the model sent something that isn't JSON") from None
    if isinstance(data.get("error"), dict) and data["error"].get("message"):
        raise LLMError(str(data["error"]["message"])[:200])
    out = data.get("output")
    if not isinstance(out, list):
        text = data.get("output_text")
        if text:
            return [{"type": "message", "role": "assistant", "content": [{"type": "output_text", "text": text}]}]
        raise LLMError("the model's answer had no output")
    return out


def _text_of(item):
    parts = item.get("content") or []
    if isinstance(parts, str):
        return parts
    return "".join(p.get("text", "") for p in parts if isinstance(p, dict) and p.get("type") in ("output_text", "text"))


def _clean(item):
    """Output item -> input item for the next call (no server ids: the proxy runs with store=false)."""
    if item.get("type") == "function_call":
        return {"type": "function_call", "call_id": item["call_id"], "name": item["name"], "arguments": item.get("arguments") or "{}"}
    return {"type": "message", "role": "assistant", "content": [{"type": "output_text", "text": _text_of(item)}]}


# ---------------------------------------------------------------------------- prompts
def robot_id(st):
    w = st.get("world") if isinstance(st.get("world"), dict) else None
    rid = ((w or {}).get("robot") or {}).get("id") if w else None
    return str(rid or st.get("robot") or "g1").lower()


def capabilities(st):
    w = st.get("world") if isinstance(st.get("world"), dict) else None
    caps = ((w or {}).get("robot") or {}).get("capabilities") if w else None
    return list(caps) if isinstance(caps, list) and caps else list(DEFAULT_CAPS)


def system_prompt(st, confirm):
    rid = robot_id(st)
    base = PROMPTS / "base.md"
    text = base.read_text() if base.exists() else BUILTIN_PROMPT
    own = PROMPTS / f"{rid}.md"
    if own.exists():
        text += "\n\n" + own.read_text()
    text += "\n\n## Right now\n" + json.dumps(context(st, confirm), ensure_ascii=False, separators=(",", ":"))
    return text


def _r(p, n=2):
    return [round(float(x), n) for x in p] if isinstance(p, (list, tuple)) else p


def _objects(st):
    movable = {o.get("name"): o for o in ((st.get("goal") or {}).get("objects") or [])}
    out = []
    for o in st.get("scene_objects") or []:
        key = str(o.get("name", "")).replace(" ", "_")
        out.append({"name": o.get("name"), "pos": _r(o.get("pos")), "movable": key in movable or not movable})
    return out


def context(st, confirm=True):
    """Compact world summary the model gets with every call (saves an observe round trip)."""
    w = st.get("world") if isinstance(st.get("world"), dict) else None
    robot = dict((w or {}).get("robot") or {"id": robot_id(st), "label": "Unitree G1 humanoid", "kind": "humanoid"})
    robot["capabilities"] = capabilities(st)
    ctx = {"robot": robot, "confirm_before_moving": bool(confirm), "objects": _objects(st),
           "holding": {k: v for k, v in (st.get("holding") or {}).items() if v}, "regions": st.get("regions") or [],
           "markers": [{"id": m.get("id"), "pos": _r(m.get("pos"))} for m in st.get("markers") or []],
           "zones": [{"id": z.get("id"), "label": z.get("label")} for z in st.get("zones") or []],
           "robot_mode": st.get("mode"), "speed": st.get("speed_scale")}
    if w and w.get("scene"):
        ctx["scene_template"] = (w.get("scene") or {}).get("template")
    p = st.get("plan")
    if p:
        ctx["plan_card"] = {"id": p.get("id"), "phase": p.get("phase"), "intent": p.get("intent"), "verdict": p.get("verdict")}
    ch = st.get("choices")
    if ch:
        ctx["question_open"] = {"question": ch.get("question"), "options": [o.get("key") + "=" + str(o.get("label")) for o in ch.get("options") or []]}
    if st.get("skills"):
        ctx["skills"] = st.get("skills")
    h, wt = st.get("human") or {}, st.get("watch") or {}
    if h.get("visible") and h.get("src") != "synthetic":
        ctx["person"] = {k: h.get(k) for k in ("activity", "phase", "reps", "holding", "offer", "summary") if h.get(k) not in (None, 0, "")}
    sf = st.get("safety") or {}
    if sf.get("zone") in ("slow", "stop"):
        ctx["safety"] = {"stop": bool(sf.get("stop")), "speed_scale": sf.get("speed_scale"), "message": sf.get("message")}
    if wt.get("mode") and (wt.get("mode") != "off" or wt.get("rules")):
        ctx["watch"] = {"mode": wt.get("mode"), "rules": [f"{r.get('id')}: when {r.get('when') or r.get('expr')} -> {r.get('do')}"
                                                          for r in wt.get("rules") or []]}
    tj = trajectory_summary(st)
    if tj:
        ctx["trajectory"] = tj
    return ctx


def trajectory_summary(st):
    """The trajectory on screen (P2, state.traj) in a few tokens: which one, its points (1-based, what they are near,
    height above the table) and the verdict, so 'higher there' / 'slower near the vase' can name points."""
    tj = st.get("traj") if isinstance(st.get("traj"), dict) else {}
    tr = tj.get("proposed") or tj.get("current")
    if not tr:
        return None
    pts = []
    for e in (tj.get("info") or [])[:24]:
        d = {"n": e.get("i", 0) + 1, "h_cm": round(100 * float(e.get("h") or 0))}
        for k in ("near", "speed", "force", "grip", "yaw", "tilt", "hold"):
            if e.get(k) is not None:
                d[k] = e[k]
        pts.append(d)
    out = {"state": "proposed (waiting for go)" if tj.get("proposed") else ("running" if tj.get("running") else "ready"),
           "label": tr.get("label"), "source": tr.get("source"), "frame": tr.get("frame"), "points": pts,
           "duration_s": ((tr.get("check") or {}).get("duration")), "ok": tj.get("ok")}
    if tj.get("why"):
        out["why"] = tj["why"]
    if tr.get("pattern"):
        out["pattern"] = tr["pattern"]
    return out


# ---------------------------------------------------------------------------- tools
def _fn(name, desc, props=None, required=None):
    return {"type": "function", "name": name, "description": desc,
            "parameters": {"type": "object", "properties": props or {}, "required": required or [], "additionalProperties": False}}


XYZ = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
       "description": "A point on the table in metres, robot frame [x forward, y left, z up]."}
TARGET = {"type": "string", "description": "Region (left, right, center, front, back), an object ('next to the vase'), "
                                           "a marker letter (A-F, P) or 'along my path' to carry it along the user's drawn route."}
_N = lambda d: {"type": "number", "description": d}
# P2: one point of a trajectory, written by the model relative to what it sees (compiled by pps/traj.py from_keypoints)
KEYPOINT = {"type": "object", "additionalProperties": False, "required": ["where"],
            "description": "One point of the path. `where` picks how it is placed; only that form's fields are read: "
                           "object = `of` + `rel` (+ dz, dist, dx, dy, height); pass = go by `of` without stopping, `via` "
                           "over / around_left / around_right (+ clearance); point = `xyz`; step = `dxyz` from the previous "
                           "point; marker = `marker` (+ dz); region = `region` (+ dz); hand = start where the hand is now "
                           "(first point only). speed, force, grip, tol, yaw, tilt, hold, note and pattern apply to every form. "
                           "Leave out the fields you don't need.",
            "properties": {
                "where": {"type": "string", "enum": ["object", "pass", "point", "step", "marker", "region", "hand"],
                          "description": "Which kind of point this is (see above)."},
                "of": {"type": "string", "description": "Object name as in the context (e.g. 'red_cup'), or a marker letter, or a region."},
                "rel": {"type": "string", "enum": ["above", "at", "left_of", "right_of", "in_front_of", "behind", "beside", "around"],
                        "description": "Where relative to `of` (robot's view; in_front_of = toward the robot). above: dz over its "
                                       "top (default 0.10 m). at: just on its top (0.02 m). left_of/right_of/in_front_of/behind/"
                                       "beside: dist from its side (default 0.06 m), dz above the table (default 0.10 m). "
                                       "around: a loop around it (dist from its side, turns, dir)."},
                "via": {"type": "string", "enum": ["over", "around_left", "around_right"],
                        "description": "Pass by `of` without stopping: over its top + clearance, or around one side of it."},
                "xyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                        "description": "Absolute point in metres, robot frame [x forward, y left, z up]."},
                "dxyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                         "description": "Move by [dx, dy, dz] metres from the previous point ('then 10 cm forward')."},
                "marker": {"type": "string", "description": "Marker letter the user placed (A-F, P = where they point)."},
                "region": {"type": "string", "enum": ["left", "right", "center", "front", "back"]},
                "from": {"type": "string", "enum": ["hand"], "description": "Start exactly where the hand is now."},
                "dz": _N("Height in metres: over the object's top (above, at) or over the table (beside, around)."),
                "dist": _N("Gap in metres from the object's side (beside / left_of / ... / around)."),
                "dx": _N("Extra shift in metres along x (forward)."), "dy": _N("Extra shift in metres along y (left)."),
                "height": _N("Absolute height in metres above the table; overrides dz."),
                "clearance": _N("Gap in metres over the object for via over (default 0.05)."),
                "turns": _N("around: how many loops (default 1)."),
                "dir": {"type": "string", "enum": ["ccw", "cw"], "description": "around: direction seen from above."},
                "speed": _N("Speed in m/s passing this point (0.03 very slow, 0.1 careful, 0.2 normal, 0.3 fast)."),
                "force": _N("Pressing force in newtons from here on (wiping, pressing, sanding); 0 stops pressing."),
                "grip": _N("Gripper from here on: 0 open, 1 closed (close next to an object to grasp it)."),
                "tol": _N("Position tolerance in metres (default: the robot's)."),
                "yaw": _N("Wrist turn in degrees about the vertical (+ = counter-clockwise from above)."),
                "tilt": _N("Tool lean in degrees toward the direction of travel (a push angle)."),
                "hold": _N("Stop here for this many seconds."),
                "note": {"type": "string", "description": "Short note shown on the point (e.g. 'slow here')."},
                "pattern": {"type": "boolean", "description": "The pattern runs between the first and last points marked true."}}}
PATTERN = {"type": "object", "additionalProperties": False,
           "description": "A small motion superimposed on the path: zigzag (weave), circle or figure8.",
           "properties": {"kind": {"type": "string", "enum": ["zigzag", "circle", "figure8"]},
                          "amp": _N("Amplitude in metres (e.g. 0.004 for a 4 mm weave)."), "freq": _N("Cycles per second."),
                          "axis": {"type": "string", "enum": ["lateral", "vertical"],
                                   "description": "lateral = sideways across the path (default), vertical = up and down."}},
           "required": ["kind", "amp", "freq"]}
CHANGES = {"type": "object", "additionalProperties": False, "description": "What to change at the chosen points (leave out the rest).",
           "properties": {"dz": _N("Raise (+) or lower (-) by metres ('higher' = +0.05)."), "dx": _N("Shift forward (+) / back (-), m."),
                          "dy": _N("Shift left (+) / right (-), m."), "height": _N("Set the height above the table, m."),
                          "speed": _N("Set the speed, m/s."), "speed_factor": _N("Multiply the speed ('slower' = 0.5, 'faster' = 1.5)."),
                          "force": _N("Pressing force, N."), "grip": _N("0 open, 1 closed."), "tol": _N("Tolerance, m."),
                          "yaw": _N("Wrist turn about the vertical, degrees (absolute)."), "tilt": _N("Tool lean along the path, degrees."),
                          "hold": _N("Pause here, seconds."), "note": {"type": "string"},
                          "remove": {"type": "boolean", "description": "Delete the chosen points."}}}


def tool_specs(st, confirm=True):
    caps = set(capabilities(st))
    has_world = isinstance(st.get("world"), dict)
    moving = " Proposes a plan card for the user to approve (confirm before moving is on)." if confirm else " Runs it right away."
    T = [_fn("observe", "Full current state: robot + capabilities, objects with positions, markers, keep-out zones, what the robot "
                        "holds, the plan card and any open question, skills, scene.")]
    if "pick_place" in caps:
        T.append(_fn("pick_place", "Pick an object up and put it somewhere. Give exactly one of target, marker or xyz and leave the others out." + moving,
                     {"object": {"type": "string", "description": "Object name, e.g. 'mug', 'red block'."}, "target": TARGET,
                      "marker": {"type": "string", "description": "Marker letter the user placed (A-F, P)."}, "xyz": XYZ}, ["object"]))
    if "reach" in caps:
        T.append(_fn("reach", "Move a hand over a spot (no grasp). Give one of target, marker or xyz." + moving,
                     {"target": TARGET, "marker": {"type": "string"}, "xyz": XYZ}))
    if "follow_path" in caps:
        T.append(_fn("follow_path", "Trace the route the user drew (on the robot view or in 3D) with a hand." + moving))
        T.append(_fn("make_trajectory", "Make a 3D path for the hand from a description ('over the vase to the right of the bowl', "
                                        "'wipe the plate slowly pressing 5 N', 'weave 4 mm along the seam'). You write the keypoints "
                                        "relative to the objects you see; the robot turns them into a smooth, checked trajectory, lifts "
                                        "it over obstacles in the way and shows it as a ghost. Use this instead of asking the person "
                                        "to draw. Starts from where the hand is." + moving,
                     {"description": {"type": "string", "description": "The person's request in a few words (shown on the plan card)."},
                      "keypoints": {"type": "array", "items": KEYPOINT, "minItems": 1, "maxItems": 30},
                      "object": {"type": "string", "description": "The object the path belongs to (it moves with it), e.g. the plate being wiped."},
                      "role": {"type": "string", "enum": ["auto", "right", "left"],
                               "description": "Which hand on two-armed robots; auto (default) = the hand on the path's side."},
                      "speed": _N("Default speed for every point, m/s."), "force": _N("Default pressing force, N."),
                      "clearance": _N("Gap over obstacles, m (default 0.05)."), "pattern": PATTERN},
                     ["description", "keypoints"]))
        T.append(_fn("edit_trajectory", "Change the trajectory on screen instead of starting over: 'higher there', 'slower near the "
                                        "vase', 'turn the wrist 45 degrees at the end', 'remove the last point'. Choose points with "
                                        "point (number as shown, -1 = last) or near ('there' = where the person points, a marker "
                                        "letter, an object name, 'start', 'end', 'middle', 'all'); with neither, every point changes.",
                     {"point": {"type": "integer", "description": "Point number (1 = first, -1 = last)."},
                      "near": {"type": "string", "description": "'there', a marker letter, an object name, 'start', 'end', 'middle' or 'all'."},
                      "changes": CHANGES, "pattern": PATTERN,
                      "remove_pattern": {"type": "boolean", "description": "Take the pattern off."}}))
        T.append(_fn("run_trajectory", "Run the trajectory on screen. When confirm before moving is on, only after the person "
                                       "clearly approved it (go, yes, do it) in their latest message."))
    if st.get("skills"):
        T.append(_fn("play_skill", "Replay a learned skill by name." + moving, {"name": {"type": "string", "enum": list(st["skills"])}}, ["name"]))
    T += [
        _fn("go_home", "Return the robot to its home pose."),
        _fn("stop", "Stop all motion immediately."),
        _fn("reset_scene", "Put every object back where it started and send the robot home."),
        _fn("set_speed", "Change motion speed.", {"value": {"type": "string", "description": "'slower', 'faster', or a factor 0.1-1.0 as text."}}, ["value"]),
        _fn("add_zone", "Add a keep-out zone the robot and carried objects must avoid. Give near (an object name) or rect.",
            {"near": {"type": "string", "description": "Object to keep away from, e.g. 'vase'."},
             "rect": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4, "description": "[x0, y0, x1, y1] table rectangle in metres."}}),
        _fn("remove_zone", "Remove one keep-out zone by its id (letter).", {"id": {"type": "string"}}, ["id"]),
        _fn("clear_zones", "Remove all keep-out zones."),
        _fn("set_goal_layout", "Arrange several objects at once: give each movable object's goal [x, y] on the table. The robot orders "
                               "the moves itself (moves blockers first, goes over obstacles). With confirm before moving on it only shows the goal; "
                               "run_goal starts it once the user approves.",
            {"targets": {"type": "object", "description": "{object name: [x, y]}", "additionalProperties": {"type": "array", "items": {"type": "number"}}},
             "run": {"type": "boolean", "description": "Start right away (ignored while confirm before moving is on)."}}, ["targets"]),
        _fn("run_goal", "Start the goal layout that is shown (only after the user approved it)."),
        _fn("choose_candidate", "Answer the robot's open 'which one do you mean?' question.",
            {"key": {"type": "string", "description": "Option key ('1', '2', ...), a colour/name, or 'cancel'."}}, ["key"]),
        _fn("confirm_plan", "Run the plan card that is waiting. Only when the user just clearly approved it."),
        _fn("cancel_plan", "Drop the plan card that is waiting (nothing moves), or stop the plan that is running."),
    ]
    if "pick_place" in caps:
        T.append(_fn("act_on_pointing", "The person is across the table and may be pointing. Use this when their words refer to what they "
                                        "point at ('this', 'that', 'here', 'there', '이거', '저기'…): pass their words; the server resolves them "
                                        "against the pointing at speech time and runs it (plan card first when confirm is on).",
                     {"text": {"type": "string", "description": "The user's words, e.g. 'put this there'."}}, ["text"]))
        T.append(_fn("handover_to_person", "Bring the held (or named) object to the person's open hand, as far as the arm reaches.",
                     {"object": {"type": "string", "description": "Object to bring; omit to use what the robot holds."}}))
    if "drive_base" in caps:
        T.append(_fn("drive", "Drive the mobile base (do this first when the target is out of the arm's reach). World frame: +x away from "
                              "the robot toward the table, +y to the robot's left. Small moves; the base will not drive into furniture.",
                     {"dx": {"type": "number", "description": "metres along x"}, "dy": {"type": "number", "description": "metres along y"},
                      "dyaw": {"type": "number", "description": "turn in degrees, left positive"}}))
    T += [
        _fn("set_watch_mode", "Watch the person and act on what they do. mirror = copy them live (whole body on the G1, the hand "
                              "on arms: wrist position, palm turn, pinch -> gripper); learn = record a demonstration (stop it with "
                              "learn_from_demo stop); respond = run the watch rules; assist = follow them and help when they stall "
                              "(spotting); coach = count reps and correct their form; off = stop watching.",
            {"mode": {"type": "string", "enum": list(WATCH_MODES)},
             "hand": {"type": "string", "enum": ["auto", "left", "right"], "description": "Which of the person's hands a one-arm robot copies."},
             "target": {"type": "string", "description": "assist: 'hands' (default, under their hands) or an object name to steady."}},
            ["mode"]),
        _fn("add_watch_rule", "Add a rule the robot keeps watching for: when the person does something, do something. Use the "
                              "built-in conditions when they fit (combine with 'and' / 'not'): hand_open_near_robot, hand_out, "
                              "hand_open, holding, holding:<tool>, not_holding, activity:<squat|curl|press|wave|weave|reach|point|still>, "
                              "phase:<top|down|bottom|up>, reps>=<n>, stalled, still, pointing. For anything else compile the "
                              "person's words into expr, a Python-like condition over state.human fields (activity, phase, reps, "
                              "holding, speed, stalled, fatigue, offer, hands.right.open, hands.left.pinch, ...), e.g. "
                              "'activity == \"squat\" and fatigue > 0.3'. do is what to do: 'give me the wrench' (hands the object to "
                              "their hand), 'say ...', 'stop', 'help me' (assist), or an instruction for you.",
            {"when": {"type": "string", "description": "Built-in condition(s), or the person's own words when you also give expr."},
             "do": {"type": "string", "description": "The action, e.g. 'give me the wrench'."},
             "expr": {"type": "string", "description": "Optional compiled condition over state.human (only when no built-in fits)."},
             "cooldown": {"type": "number", "description": "Seconds before it may fire again (default 8)."}},
            ["when", "do"]),
        _fn("remove_watch_rule", "Remove a watch rule by id (R1, R2, ...), or 'all'.", {"id": {"type": "string"}}, ["id"]),
        _fn("learn_from_demo", "Learn by watching: start recording the person's demonstration, stop it (saves it and reports the "
                               "hand, tool, pattern such as a weave's width and frequency), or replay the last one as a proposed path "
                               "on the object it was shown over.",
            {"op": {"type": "string", "enum": ["start", "stop", "replay"]}, "name": {"type": "string", "description": "Name for the demonstration."}},
            ["op"]),
        _fn("mirror_person", "Start or stop copying the person live (shortcut for set_watch_mode mirror / off).",
            {"enabled": {"type": "boolean"}}, ["enabled"]),
    ]
    if "follow_me" in caps:
        T.append(_fn("follow_me", "Start or stop copying the user's body from their camera.", {"enabled": {"type": "boolean"}}, ["enabled"]))
    if "teach_by_showing" in caps:
        T.append(_fn("start_teaching", "Start teach-by-showing: the user demonstrates a task on camera and explains it."))
    T += [
        _fn("set_robot", "Switch to another robot body." + ("" if has_world else " (Not available in this build yet.)"),
            {"id": {"type": "string", "enum": list((r.get("id") for r in (st.get("world") or {}).get("robots") or []) if has_world else ROBOT_IDS) or list(ROBOT_IDS)}}, ["id"]),
        _fn("build_scene", "Rebuild the scene from a spec: template + objects.",
            {"spec": {"type": "object", "description": '{"template": "table|kitchen|shelf|floor", "objects": [{"name", "kind": '
                      '"mug|block|bowl|cup|bottle|can|box|plate|fruit", "color", "pos": [x, y] or "left|center|...", "movable": true}]}'}},
            ["spec"]),
        _fn("add_object", "Add one object to the scene.", {"object": {"type": "object", "description": '{"name", "kind", "color", "pos", "movable"}'}}, ["object"]),
        _fn("remove_object", "Remove an object from the scene by name.", {"name": {"type": "string"}}, ["name"]),
    ]
    return T


# ---------------------------------------------------------------------------- the agent
class Agent:
    """push(msg): send a browser-style message to the worker. state(): latest state dict (thread-safe snapshot)."""

    def __init__(self, push, state, llm=responses_call, server_cfg=server_llm, poll=0.12):
        self.push, self.get_state, self.llm, self.server_cfg, self.poll = push, state, llm, server_cfg, poll
        self.lock = threading.Lock()
        self.events = deque(maxlen=MAX_EVENTS)
        self.history = []            # [[items of one turn], ...]
        self.notes = []              # robot updates that happened between turns
        self.outbox = []             # lines to speak (the worker drains it into state.say)
        self.busy, self.confirm, self.model = False, True, None
        self.gen = 0                 # bumped by reset: a running turn's results are dropped
        self.mine = {}               # task id -> last status we already reported
        self.waiting_on = None       # task id a tool is waiting on right now
        self._eid, self._n = 0, 0
        self._proposed = None       # plan id proposed during the current turn

    # ------------------------------------------------------------ public (worker thread)
    def _server(self):
        """server_cfg() re-read at most every 5 s (state() runs every frame; .env is a file)."""
        now = time.time()
        if now - getattr(self, "_cfg_t", 0.0) > 5.0:
            self._cfg, self._cfg_t = self.server_cfg(), now
        return self._cfg

    def available(self):
        return self._server() is not None

    def state(self):
        cfg = self._server()
        with self.lock:
            return {"available": cfg is not None, "busy": self.busy, "model": self.model or (cfg or {}).get("model"),
                    "confirm": self.confirm, "byok": byok(), "events": [dict(e) for e in self.events]}

    def submit(self, text, llm=None, confirm=True):
        text = str(text or "").strip()[:2000]
        if not text:
            return False
        cfg, why = (visitor_llm(llm) if isinstance(llm, dict) and llm.get("key") else (None, "no key"))
        if cfg is None and why != "no key":
            self._event("error", f"Your LLM settings don't work: {why}.")
            return False
        cfg = cfg or self._server()
        if cfg is None:
            self._event("error", "No LLM is configured. " + ("Add your own LLM key in Settings." if byok()
                                                             else "Set LLM_BASE_URL and LLM_API_KEY in .env, or use the offline parser."))
            return False
        with self.lock:
            if self.busy:
                busy = True
            else:
                busy, self.busy, self.confirm, self.model = False, True, bool(confirm), cfg["model"]
                gen = self.gen
        if busy:
            self._event("error", "I'm still working on your last request. Say stop to interrupt the robot.")
            return False
        self._event("user", text)
        threading.Thread(target=self._turn, args=(text, cfg, bool(confirm), gen), daemon=True, name="agent-turn").start()
        return True

    def reset(self):
        with self.lock:
            self.gen += 1
            self.history, self.notes, self.events, self.busy = [], [], deque(maxlen=MAX_EVENTS), False
            self.mine, self.waiting_on = {}, None
        self._event("thinking", "Conversation cleared.")

    def watch(self, task, speak=True):
        """Worker thread, every frame: report how a task we launched ended (e.g. a plan the user confirmed later)."""
        if not task or task.get("id") not in self.mine or task.get("id") == self.waiting_on:
            return
        st = task.get("status")
        if st not in ("done", "failed") or self.mine[task["id"]] == st:
            return
        self.mine[task["id"]] = st
        msg = task.get("message") or st
        self._event("result", msg, ok=st == "done")
        self.notes.append(f"{task.get('action')} {task['id']} {st}: {msg}")
        if speak:
            self.say(msg)

    def say(self, text):
        with self.lock:
            self.outbox.append(text)

    def drain(self):
        with self.lock:
            out, self.outbox = self.outbox, []
        return out

    # ------------------------------------------------------------ events
    def _event(self, kind, text, **kw):
        with self.lock:
            self._eid += 1
            e = {"id": self._eid, "kind": kind, "text": str(text)[:600], "t": round(time.time(), 2)}
            e.update({k: v for k, v in kw.items() if v is not None})
            self.events.append(e)
            return e

    def _update(self, e, **kw):
        with self.lock:
            e.update({k: v for k, v in kw.items() if v is not None})

    # ------------------------------------------------------------ one turn (agent thread)
    def _turn(self, text, cfg, confirm, gen):
        try:
            self._run(text, cfg, confirm, gen)
        except Exception as e:      # never leave the agent stuck busy
            traceback.print_exc()
            if gen == self.gen:
                self._event("error", f"Something went wrong: {str(e)[:160]}")
        finally:
            with self.lock:
                if gen == self.gen:
                    self.busy = False
                self.waiting_on = None

    def _run(self, text, cfg, confirm, gen):
        st = self.state_now()
        notes, self.notes = self.notes, []
        user = text if not notes else "[robot updates since your last reply: " + " | ".join(notes)[-1200:] + "]\n" + text
        turn = [{"role": "user", "content": user}]
        self._proposed = None
        self._event("thinking", f"thinking · {cfg['model']}")
        for step in range(MAX_STEPS):
            if gen != self.gen:
                return
            history = [it for t in self.history[-HISTORY_TURNS:] for it in t]
            try:
                out = self.llm(cfg, system_prompt(st, confirm), history + turn, tool_specs(st, confirm))
            except LLMError as e:
                if gen == self.gen:
                    self._event("error", f"The model didn't answer: {e}")
                    self.say("Sorry, my language model didn't answer. Try again, or switch it off in Settings to use the offline parser.")
                return
            if gen != self.gen:
                return
            calls = [o for o in out if o.get("type") == "function_call"]
            texts = [_text_of(o) for o in out if o.get("type") == "message"]
            turn += [_clean(o) for o in out if o.get("type") in ("function_call", "message")]
            if not calls:
                reply = " ".join(t.strip() for t in texts if t.strip()) or "Done."
                self._event("reply", reply)
                self.say(reply)
                self._remember(turn)
                return
            for t in texts:
                if t.strip():
                    self._event("thinking", t.strip())
            for c in calls:
                name = c.get("name", "")
                try:
                    args = json.loads(c.get("arguments") or "{}")
                    if not isinstance(args, dict):
                        args = {}
                except ValueError:
                    args = {}
                ev = self._event("tool", name, tool=name, args=args)
                try:
                    res = self.execute(name, args, confirm)
                except Exception as e:
                    traceback.print_exc()
                    res = {"ok": False, "error": f"tool crashed: {str(e)[:120]}"}
                self._update(ev, ok=bool(res.get("ok")), text=_summary(name, res))
                out_s = json.dumps(res, ensure_ascii=False, default=str)
                turn.append({"type": "function_call_output", "call_id": c["call_id"],
                             "output": out_s if len(out_s) <= MAX_OUTPUT_CHARS else out_s[:MAX_OUTPUT_CHARS] + "…(truncated)"})
            st = self.state_now()
        reply = "I couldn't finish that in a few steps. Here's where I got to; tell me how to continue."
        self._event("reply", reply)
        self.say(reply)
        self._remember(turn)

    def _remember(self, turn):
        self.history.append(turn)
        del self.history[:-HISTORY_TURNS]

    # ------------------------------------------------------------ state + waiting
    def state_now(self):
        try:
            st = self.get_state()
        except Exception:
            st = None
        return st if isinstance(st, dict) else {}

    def _wait(self, pred, timeout):
        t0 = time.time()
        while True:
            st = self.state_now()
            r = pred(st)
            if r is not None and r is not False:
                return r, st
            if time.time() - t0 > timeout:
                return None, st
            time.sleep(self.poll)

    def _mine(self, tid, status=None):
        self.mine[tid] = status
        while len(self.mine) > 60:
            self.mine.pop(next(iter(self.mine)))

    def _tid(self, action):
        self._n += 1
        return f"ag{self._n}-{action}-{int(time.time() * 1000) % 100000}"

    # ------------------------------------------------------------ tool execution
    def execute(self, name, a, confirm=True):
        fn = getattr(self, "t_" + name, None)
        if fn is None:
            return {"ok": False, "error": f"unknown tool {name}"}
        return fn(a, confirm)

    def t_observe(self, a, confirm):
        st = self.state_now()
        out = context(st, confirm)
        w = st.get("world") if isinstance(st.get("world"), dict) else None
        if w:
            out["world"] = {k: w.get(k) for k in ("rev", "scene", "templates") if k in w}
            out["available_robots"] = [{"id": r.get("id"), "label": r.get("label"), "kind": r.get("kind"),
                                        "experimental": r.get("experimental")} for r in w.get("robots") or []]
        else:
            out["world_tools"] = "not available in this build (one robot, fixed table scene)"
        out["task"] = {k: (st.get("task") or {}).get(k) for k in ("action", "status", "message")} if st.get("task") else None
        out["zones"] = st.get("zones") or []
        cp = st.get("copresence") or {}
        if cp.get("on"):   # co-presence: the person across the table, what they point at, what the robot attends to
            pt = cp.get("pointing") or {}
            out["person"] = {"pointing_at": pt.get("target"), "pointing_spot": pt.get("hit"), "hand_open": cp.get("hand_open"),
                             "attention": cp.get("attention")}
        h = st.get("human") or {}
        if h:   # P3: what the person is doing (without the 33-point pose, which only costs tokens)
            out["human"] = {k: v for k, v in h.items() if k not in ("pose", "pose_filled")}
        wt = st.get("watch") or {}
        if wt:
            out["watch"] = {"mode": wt.get("mode"), "rules": wt.get("rules"), "events": (wt.get("events") or [])[-5:],
                            "last_demo": (wt.get("learn") or {}).get("last"), "coach": wt.get("coach"), "assist": wt.get("assist")}
        sf = st.get("safety") or {}
        if sf:   # P5: separation from the person, speed scale, protective stop (never something to work around)
            out["safety"] = {k: sf.get(k) for k in ("person", "d_min", "speed_scale", "stop", "latched", "reason", "zone",
                                                     "message", "closest", "coop")}
        g = st.get("goal") or {}
        if g.get("status") and g.get("status") != "idle":
            out["goal"] = {"status": g.get("status"), "message": g.get("message"), "targets": g.get("targets")}
        p = st.get("plan")
        if p:
            out["plan_card"] = {k: p.get(k) for k in ("id", "phase", "ok", "intent", "verdict", "reasons", "message")}
        out["ok"] = True
        return out

    def _resolve_target(self, a):
        """target | marker | xyz -> (target string, error). Models often fill every optional field ("", [0, 0, 0]):
        the first meaningful one wins, in that order."""
        if str(a.get("target") or "").strip():
            return str(a["target"]).strip(), None
        if str(a.get("marker") or "").strip():
            return str(a["marker"]).strip().upper()[:1], None
        p = a.get("xyz")
        if isinstance(p, (list, tuple)) and len(p) == 3 and any(abs(float(v)) > 1e-6 for v in p):
            before = {m.get("id"): m.get("pos") for m in self.state_now().get("markers") or []}
            self.push({"type": "point3d", "pos": [float(v) for v in p]})

            def new_marker(st):
                for m in st.get("markers") or []:
                    if m.get("id") not in before or before[m.get("id")] != m.get("pos"):
                        return m.get("id")
                return None
            mid, _ = self._wait(new_marker, 4)
            if mid is None:
                return None, "couldn't place a marker at that point"
            return mid, None
        return None, None

    def _motion(self, action, msg, confirm):
        if self._proposed:
            return {"ok": False, "error": f"Plan {self._proposed} is already waiting for the user. Stop and ask them to say go, "
                                          "or change it after they answer."}
        tid = self._tid(action)
        msg = dict(msg, type="task", id=tid, action=action)
        if confirm:
            msg["preview"] = True
        self._mine(tid)
        self.waiting_on = tid
        self.push(msg)
        final = ("proposed", "asking", "done", "failed") if confirm else ("asking", "done", "failed")

        seen = set()

        def settled(st):
            t = st.get("task") or {}
            if t.get("id") == tid:
                seen.add(tid)
                return t if t.get("status") in final else None
            if tid in seen:      # it was there and something else replaced it
                return {"id": tid, "status": "failed", "message": "Another command took over before it finished."}
            return None
        t, st = self._wait(settled, 25 if confirm else 75)
        self.waiting_on = None
        if t is None:
            t = st.get("task") or {}
            if t.get("id") == tid and t.get("status") == "running":
                self.mine[tid] = None
                return {"ok": True, "status": "running", "message": t.get("message"), "note": "Still running; I'll report when it ends."}
            return {"ok": False, "status": "timeout", "message": "The robot didn't answer in time."}
        self.mine[tid] = t.get("status") if t.get("status") in ("done", "failed") else None
        res = {"ok": t.get("status") not in ("failed",), "status": t.get("status"), "message": t.get("message")}
        p = st.get("plan")
        if p and p.get("id") == tid:
            res["plan"] = {k: p.get(k) for k in ("phase", "verdict", "intent", "reasons") if p.get(k)}
            res["plan"]["steps"] = [s.get("text") for s in p.get("steps") or []]
            if (p.get("alternative") or {}).get("text"):
                res["plan"]["alternative"] = p["alternative"]["text"]
            if p.get("phase") == "proposing":
                self._proposed = tid
                res["next"] = "A plan card is showing. Tell the user what you'll do and that they can say go (or tap Run). Don't call more motion tools."
            elif p.get("phase") == "needs_you":
                res["next"] = "The plan can't run as asked; the card offers the alternative. Explain and ask the user (yes uses the alternative)."
        ch = st.get("choices")
        if t.get("status") == "asking" and ch:
            res["ok"] = True
            res["question"] = ch.get("question")
            res["options"] = [{"key": o.get("key"), "label": o.get("label")} for o in ch.get("options") or []]
            res["next"] = "Ask the user which one (or call choose_candidate if their message already says)."
        return res

    def t_pick_place(self, a, confirm):
        obj = str(a.get("object") or "").strip()
        if not obj:
            return {"ok": False, "error": "which object?"}
        target, err = self._resolve_target(a)
        if err:
            return {"ok": False, "error": err}
        if target is None:
            return {"ok": False, "error": "give a target, marker or xyz"}
        return self._motion("pick_place", {"object": obj, "target": target}, confirm)

    def t_reach(self, a, confirm):
        target, err = self._resolve_target(a)
        if err or target is None:
            return {"ok": False, "error": err or "give a target, marker or xyz"}
        return self._motion("reach", {"target": target}, confirm)

    def t_follow_path(self, a, confirm):
        return self._motion("follow_path", {}, confirm)

    def t_play_skill(self, a, confirm):
        return self._motion("play_skill", {"name": str(a.get("name") or "")}, confirm)

    # ---- trajectories (P2: {type:"traj", ...}; state.traj)
    def _traj(self, msg, timeout=10):
        rid = msg.get("id") or self._tid("traj")
        self.push(dict(msg, type="traj", id=rid))
        r, st = self._wait(lambda s: (s.get("traj") or {}).get("reply") if ((s.get("traj") or {}).get("reply") or {}).get("id") == rid else None,
                           timeout)
        if r is None:
            return None, st, {"ok": False, "message": "Trajectories aren't available on this server."}
        res = {"ok": bool(r.get("ok")), "message": r.get("message")}
        tj = trajectory_summary(st)
        if tj:
            res["trajectory"] = tj
        return r, st, res

    def _traj_wait_run(self, tid, res):
        """Confirm before moving is off: the path runs right away; report how it ended."""
        if not tid:
            return res
        self._mine(tid)
        self.waiting_on = tid
        t, _ = self._wait(lambda s: (s.get("task") or {}) if (s.get("task") or {}).get("id") == tid
                          and (s.get("task") or {}).get("status") in ("done", "failed") else None, 75)
        self.waiting_on = None
        if t is None:
            self.mine[tid] = None
            return dict(res, status="running", note="Still running; I'll report when it ends.")
        self.mine[tid] = t.get("status")
        return dict(res, ok=t.get("status") == "done", status=t.get("status"), message=t.get("message"))

    def t_make_trajectory(self, a, confirm):
        if self._proposed:
            return {"ok": False, "error": f"Plan {self._proposed} is already waiting for the user. Stop and ask them to say go, "
                                          "or change it with edit_trajectory after they answer."}
        kps = [_clean_keypoint(k) for k in a.get("keypoints") or [] if isinstance(k, dict)]
        kps = [k for k in kps if k]
        for key in ("grip", "force"):     # 0 on every point is a filled-in default, not "open the gripper here" (it would drop what I hold)
            vals = [k.get(key) for k in kps]
            if vals and all(v is not None and float(v) == 0.0 for v in vals):
                for k in kps:
                    k.pop(key, None)
        if not kps:
            return {"ok": False, "error": "give at least one keypoint (where the hand should go)"}
        spec = {"label": str(a.get("description") or "")[:120], "keypoints": kps}
        if str(a.get("object") or "").strip():
            spec["object"] = str(a["object"]).strip()
        if str(a.get("role") or "").strip().lower() in ("left", "right"):
            spec["role"] = str(a["role"]).strip().lower()
        for k in ("speed", "force", "clearance"):
            if isinstance(a.get(k), (int, float)) and a[k] > 0:
                spec[k] = float(a[k])
        if isinstance(a.get("pattern"), dict) and a["pattern"].get("kind") and a["pattern"].get("amp"):
            spec["pattern"] = a["pattern"]
        rid = self._tid("traj")
        msg = {"op": "propose" if confirm else "set", "spec": spec, "id": rid}
        if confirm:
            msg["task_id"] = rid
        else:
            msg["run"] = True
        r, st, res = self._traj(msg)
        if r is None or not r.get("ok"):
            if r is not None and r.get("plan"):
                res["next"] = ("The path is blocked; the card shows why. Explain in one sentence and offer a fix (edit_trajectory, "
                               "e.g. higher or around), or ask the person to drag the red points.")
            return res
        if confirm:
            self._mine(rid)
            self._proposed = rid
            res["next"] = ("The path is showing as a ghost with a plan card. Tell the person in one sentence what it does and that "
                           "they can say go, drag a point, or say what to change. Don't run it yourself.")
            return res
        return self._traj_wait_run(r.get("task"), res)

    def t_edit_trajectory(self, a, confirm):
        msg = {"op": "edit"}
        pt = a.get("point")
        if isinstance(pt, (int, float)) and int(pt) != 0:
            msg["index"] = int(pt) - 1 if pt > 0 else int(pt)
        elif str(a.get("near") or "").strip():
            msg["near"] = str(a["near"]).strip()
        ch = {k: v for k, v in (a.get("changes") or {}).items() if v not in (None, "") and not (k != "remove" and v is False)
              and not (k in ("dx", "dy", "dz", "height", "speed", "speed_factor", "tol", "hold") and v == 0)}   # filled-in zeros
        if ch:
            msg["changes"] = ch
        if a.get("remove_pattern"):
            msg["pattern"] = None
        elif isinstance(a.get("pattern"), dict) and a["pattern"].get("kind"):
            msg["pattern"] = a["pattern"]
        if "changes" not in msg and "pattern" not in msg:
            return {"ok": False, "error": "say what to change (changes, pattern or remove_pattern)"}
        r, st, res = self._traj(msg)
        p = st.get("plan") or {}
        if r is not None and p.get("phase") == "proposing" and confirm:
            self._proposed = p.get("id")
            res["next"] = "The changed path is showing. Tell the person what changed in a few words and that they can say go."
        return res

    def t_run_trajectory(self, a, confirm):
        r, st, res = self._traj({"op": "run"})
        if r is None or not r.get("ok"):
            return res
        self._proposed = None
        tid = r.get("task")
        if not tid:
            return res
        self._mine(tid)
        t, _ = self._wait(lambda s: (s.get("task") or {}) if (s.get("task") or {}).get("id") == tid
                          and (s.get("task") or {}).get("status") in ("running", "done", "failed") else None, 5)
        return dict(res, status=(t or {}).get("status"), note="Running; I'll report when it ends.")

    def _simple(self, action, extra=None, timeout=8):
        tid = self._tid(action)
        self.push(dict(extra or {}, type="task", id=tid, action=action))
        t, st = self._wait(lambda s: (s.get("task") or {}) if (s.get("task") or {}).get("id") == tid
                           and (s.get("task") or {}).get("status") != "planning" else None, timeout)
        if t is None:
            return {"ok": False, "status": "timeout", "message": "The robot didn't answer in time."}
        return {"ok": t.get("status") != "failed", "status": t.get("status"), "message": t.get("message")}

    def t_go_home(self, a, confirm):
        return self._simple("home")

    def t_stop(self, a, confirm):
        return self._simple("stop")

    def t_reset_scene(self, a, confirm):
        return self._simple("reset_scene")

    def t_set_speed(self, a, confirm):
        return self._simple("set_speed", {"value": str(a.get("value") or "slower")})

    def _copresence(self, msg, timeout=20):
        before = json.dumps((self.state_now().get("copresence") or {}).get("reply"), sort_keys=True, default=str)
        self.push(msg)
        r, st = self._wait(lambda s: (s.get("copresence") or {}).get("reply")
                           if json.dumps((s.get("copresence") or {}).get("reply"), sort_keys=True, default=str) != before else None, 4)
        if r is None:
            return {"ok": False, "message": "Co-presence is not available (no person in view)."}
        tid = r.get("ran")
        if tid:
            t, st = self._wait(lambda s: (s.get("task") or {}) if (s.get("task") or {}).get("id") == tid
                               and (s.get("task") or {}).get("status") not in ("planning", "running") else None, timeout)
            r = dict(r, task_status=(t or {}).get("status"), task_message=(t or {}).get("message"))
        return {"ok": bool(r.get("ok")), "resolved": {k: r.get(k) for k in ("kind", "object", "candidates", "spot", "marker")},
                "message": r.get("task_message") or r.get("message"), "status": r.get("task_status")}

    def t_act_on_pointing(self, a, confirm):
        return self._copresence({"type": "copresence", "op": "utter", "text": str(a.get("text") or ""), "run": True,
                                 "preview": bool(confirm), "id": self._tid("point")})

    def t_handover_to_person(self, a, confirm):
        return self._copresence({"type": "copresence", "op": "utter", "text": "give " + (str(a.get("object") or "it")) + " to me",
                                 "run": True, "id": self._tid("handover")}, timeout=30)

    def t_drive(self, a, confirm):
        num = lambda k: float(a.get(k) or 0.0)
        tid = self._tid("drive")
        self.push({"type": "task", "id": tid, "action": "drive", "dx": num("dx"), "dy": num("dy"), "dyaw": num("dyaw")})
        t, st = self._wait(lambda s: (s.get("task") or {}) if (s.get("task") or {}).get("id") == tid
                           and (s.get("task") or {}).get("status") in ("done", "failed") else None, 25)
        if t is None:
            return {"ok": False, "status": "timeout", "message": "The base didn't finish driving in time."}
        return {"ok": t.get("status") == "done", "status": t.get("status"), "message": t.get("message")}

    def t_follow_me(self, a, confirm):
        return self._simple("follow_me", {"enabled": bool(a.get("enabled", True))})

    def _zones_after(self, msg):
        before = json.dumps(self.state_now().get("zones") or [], sort_keys=True)
        self.push(msg)
        z, st = self._wait(lambda s: s.get("zones") if json.dumps(s.get("zones") or [], sort_keys=True) != before else None, 3)
        zones = st.get("zones") or []
        return {"ok": z is not None, "zones": [{"id": q.get("id"), "label": q.get("label"), "center": _r(q.get("center")),
                                                 "size": _r(q.get("size"))} for q in zones],
                **({} if z is not None else {"message": "No change: that object or zone wasn't found."})}

    def t_add_zone(self, a, confirm):
        msg = {"type": "zone", "op": "add", "source": "agent"}
        rect = a.get("rect")
        if str(a.get("near") or "").strip():
            msg["near"] = str(a["near"]).strip()
        elif isinstance(rect, list) and len(rect) == 4 and abs(float(rect[2]) - float(rect[0])) > 0.01 and abs(float(rect[3]) - float(rect[1])) > 0.01:
            msg["rect"] = [float(v) for v in rect]
        else:
            return {"ok": False, "error": "give near (an object) or rect [x0, y0, x1, y1]"}
        return self._zones_after(msg)

    def t_remove_zone(self, a, confirm):
        return self._zones_after({"type": "zone", "op": "remove", "id": str(a.get("id") or "")})

    def t_clear_zones(self, a, confirm):
        if not self.state_now().get("zones"):
            return {"ok": True, "zones": [], "message": "There were no zones."}
        return self._zones_after({"type": "zone", "op": "clear"})

    def t_set_goal_layout(self, a, confirm):
        targets = a.get("targets") if isinstance(a.get("targets"), dict) else {}
        if not targets:
            return {"ok": False, "error": "targets must map object names to [x, y]"}
        rev0 = (self.state_now().get("goal") or {}).get("rev")
        run = bool(a.get("run")) and not confirm
        self.push({"type": "goal", "op": "set", "targets": targets, "source": "agent", "run": run})
        g, st = self._wait(lambda s: (s.get("goal") or {}) if (s.get("goal") or {}).get("rev") != rev0 else None, 4)
        if g is None:
            return {"ok": False, "message": "The goal wasn't accepted."}
        res = {"ok": g.get("status") not in ("failed",), "status": g.get("status"), "message": g.get("message"),
               "moves": [s.get("label") or s.get("object") for s in g.get("sequence") or []]}
        if g.get("status") == "ready":
            res["next"] = "The goal card shows the plan. Tell the user; run_goal starts it once they approve."
        return res

    def t_run_goal(self, a, confirm):
        self.push({"type": "goal", "op": "run"})
        g, _ = self._wait(lambda s: (s.get("goal") or {}) if (s.get("goal") or {}).get("status") in ("running", "done", "failed") else None, 4)
        return {"ok": bool(g) and g.get("status") != "failed", "status": (g or {}).get("status"), "message": (g or {}).get("message")}

    def t_choose_candidate(self, a, confirm):
        st = self.state_now()
        ch = st.get("choices")
        if not ch:
            return {"ok": False, "error": "there is no open question"}
        tid = ch.get("id")
        self.push({"type": "choose", "id": tid, "key": str(a.get("key") or "")})
        if confirm:
            self._proposed = None
        t, st = self._wait(lambda s: (s.get("task") or {}) if (s.get("task") or {}).get("id") == tid
                           and (s.get("task") or {}).get("status") in ("proposed", "running", "done", "failed") else None, 15)
        if t is None:
            ch = st.get("choices")
            return {"ok": False, "message": (st.get("task") or {}).get("message") or "No answer yet.", "question": (ch or {}).get("question")}
        self.mine[tid] = None
        res = {"ok": t.get("status") != "failed", "status": t.get("status"), "message": t.get("message")}
        p = st.get("plan")
        if p and p.get("id") == tid and p.get("phase") == "proposing":
            self._proposed = tid
            res["next"] = "A plan card is showing. Tell the user and wait for go."
        return res

    def t_confirm_plan(self, a, confirm):
        p = self.state_now().get("plan")
        if not p or p.get("phase") != "proposing":
            return {"ok": False, "error": "no plan is waiting for approval"}
        tid = p.get("id")
        self.mine[tid] = None
        self.push({"type": "plan", "op": "confirm", "id": tid})
        t, _ = self._wait(lambda s: (s.get("task") or {}) if (s.get("task") or {}).get("id") == tid
                          and (s.get("task") or {}).get("status") in ("running", "done", "failed") else None, 5)
        return {"ok": bool(t) and t.get("status") != "failed", "status": (t or {}).get("status"), "message": (t or {}).get("message"),
                "note": "Running; I'll report when it ends."}

    def t_cancel_plan(self, a, confirm):
        p = self.state_now().get("plan")
        if not p:
            return {"ok": False, "error": "there is no plan card"}
        self.push({"type": "plan", "op": "cancel", "id": p.get("id")})
        self._proposed = None
        _, st = self._wait(lambda s: True if not s.get("plan") or s["plan"].get("phase") != p.get("phase") else None, 3)
        return {"ok": True, "message": (st.get("task") or {}).get("message") or "Cancelled."}

    def t_start_teaching(self, a, confirm):
        self.push({"type": "teach", "cmd": "start"})
        ph, _ = self._wait(lambda s: (s.get("teach") or {}).get("phase") if (s.get("teach") or {}).get("phase") == "recording" else None, 3)
        return {"ok": ph is not None, "message": "Watching the user's camera now. They show the task, explain it, and say done."}

    # ---- watching the person (P3: {type:"watch", ...}; state.human, state.watch)
    def _watch(self, msg, timeout=4):
        mid = self._tid("watch")
        self.push(dict(msg, type="watch", id=mid))
        r, st = self._wait(lambda s: (s.get("watch") or {}).get("reply") if ((s.get("watch") or {}).get("reply") or {}).get("id") == mid else None, timeout)
        if r is None:
            return {"ok": False, "message": "Watching isn't available on this server."}
        out = {"ok": bool(r.get("ok")), "message": r.get("message"), "mode": (st.get("watch") or {}).get("mode")}
        for k in ("rule", "demo"):
            if r.get(k):
                out[k] = r[k]
        h = st.get("human") or {}
        out["person"] = {k: h.get(k) for k in ("visible", "activity", "phase", "reps", "holding", "summary")}
        return out

    def t_set_watch_mode(self, a, confirm):
        mode = str(a.get("mode") or "").lower()
        msg = {"op": "mode", "mode": mode}
        if a.get("hand"):
            msg["hand"] = str(a["hand"])
        if a.get("target"):
            msg["target"] = str(a["target"])
        return self._watch(msg)

    def t_add_watch_rule(self, a, confirm):
        msg = {"op": "rule", "when": str(a.get("when") or ""), "do": str(a.get("do") or "")}
        if str(a.get("expr") or "").strip():
            msg["expr"] = str(a["expr"]).strip()
        if a.get("cooldown"):
            msg["cooldown"] = float(a["cooldown"])
        return self._watch(msg)

    def t_remove_watch_rule(self, a, confirm):
        rid = str(a.get("id") or "").strip()
        return self._watch({"op": "rules_clear"} if rid.lower() == "all" else {"op": "rule_remove", "rule": rid.upper()})

    def t_learn_from_demo(self, a, confirm):
        op = str(a.get("op") or "start")
        msg = {"op": {"start": "learn_start", "stop": "learn_stop", "replay": "replay"}.get(op, "learn_start")}
        if a.get("name"):
            msg["name"] = str(a["name"])
        if op == "replay":
            msg["preview"] = bool(confirm)
        res = self._watch(msg, 8)
        if op == "stop" and res.get("ok"):
            res["next"] = "Tell the person what you learned in one sentence and offer to repeat it (learn_from_demo replay)."
        return res

    def t_mirror_person(self, a, confirm):
        return self._watch({"op": "mode", "mode": "mirror" if a.get("enabled", True) else "off"})

    # ---- world (World workstream: {type:"world", ...}; state.world)
    def _world(self, msg, timeout=40):
        st = self.state_now()
        w = st.get("world")
        if not isinstance(w, dict):
            return {"ok": False, "error": "not available in this build: this server has one robot (Unitree G1) and a fixed table scene."}
        rev0 = w.get("rev")
        self.push(dict(msg, type="world"))
        w2, st = self._wait(lambda s: s.get("world") if isinstance(s.get("world"), dict) and s["world"].get("rev") != rev0 else None, timeout)
        if w2 is None:
            t = st.get("task") or {}
            return {"ok": False, "message": t.get("message") if t.get("status") == "failed" else "The world didn't change (in time)."}
        r = w2.get("robot") or {}
        return {"ok": True, "robot": {k: r.get(k) for k in ("id", "label", "kind", "capabilities", "experimental", "note") if r.get(k) is not None},
                "scene": w2.get("scene"), "objects": _objects(st)}

    def t_set_robot(self, a, confirm):
        rid = str(a.get("id") or "").strip().lower()
        return self._world({"op": "set", "robot": rid})

    def t_build_scene(self, a, confirm):
        spec = a.get("spec") if isinstance(a.get("spec"), dict) else None
        if not spec:
            return {"ok": False, "error": "spec must be {template, objects}"}
        return self._world({"op": "set", "scene": spec})

    def t_add_object(self, a, confirm):
        o = a.get("object") if isinstance(a.get("object"), dict) else None
        if not o or not o.get("name"):
            return {"ok": False, "error": "object needs at least a name"}
        return self._world({"op": "add_object", "object": o}, 20)

    def t_remove_object(self, a, confirm):
        return self._world({"op": "remove_object", "name": str(a.get("name") or "")}, 20)


def _clean_keypoint(k):
    """Models often fill every optional field ("", 0, [0, 0, 0], and an enum value for every enum): keep only the fields
    of the keypoint's form (`where`, pps/traj.py keypoint_form), and treat an empty or zero field as 'not given' (a
    speed or height of exactly 0 is never what they meant)."""
    from .traj import keypoint_form
    k = {key: v for key, v in k.items() if not (isinstance(v, list) and v and all(isinstance(x, (int, float)) and x == 0 for x in v))}
    form, k = keypoint_form(k)
    out = {"where": form}
    for key, v in k.items():
        if v is None or v == "" or v is False:
            continue
        if isinstance(v, (int, float)) and not isinstance(v, bool) and v == 0 and key not in ("grip", "force"):
            continue
        if isinstance(v, list) and (not v or all(isinstance(x, (int, float)) and x == 0 for x in v)):
            continue
        out[key] = v
    return out


def _summary(name, res):
    """One line for the tool card in the feed."""
    if res.get("error"):
        return res["error"]
    if name == "observe":
        objs = [o.get("name") for o in res.get("objects") or []]
        return f"{len(objs)} objects: {', '.join(objs[:6])}" if objs else "looked around"
    if res.get("question"):
        return res["question"]
    plan = res.get("plan") or {}
    if plan.get("verdict") and plan.get("intent"):
        return f"{plan['verdict']}: {plan['intent']}"
    if name in ("add_zone", "remove_zone", "clear_zones"):
        zs = res.get("zones") or []
        return res.get("message") or (f"{len(zs)} zone{'s' if len(zs) != 1 else ''}: " + ", ".join(z.get("label") or z.get("id") for z in zs) if zs else "no zones")
    if name in ("set_robot", "build_scene", "add_object", "remove_object") and res.get("ok"):
        r = res.get("robot") or {}
        return f"{r.get('label') or r.get('id') or 'world'} · {len(res.get('objects') or [])} objects"
    return str(res.get("message") or res.get("status") or ("ok" if res.get("ok") else "failed"))
