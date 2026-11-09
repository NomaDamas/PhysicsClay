# PhysicsClay browser <-> server protocol (v2)

One page (`web/index.html`) served by `pps/app.py` at `http(s)://<host>:8777/`. One WebSocket at `/ws`.
Product framing: **"A new way to prompt Physical AI — turning human intent into spatial instructions robots can understand and execute."**
Gemini (Live + Robotics-ER) understands intent; the server's physics (MuJoCo + IK) grounds and validates it; the robot says what it cannot do.

## Roles of the views
- **Robot view** (main canvas, server-rendered JPEG of the MuJoCo scene). Two cameras: `robot` (over the robot's shoulder, looking at the table: what the robot sees) and `orbit` (free camera). The user taps/draws here: these become 3D points on the table via server raycast. This is the *spatial canvas*.
- **Your camera** (server-captured webcam JPEG, mirrored). Part of the prompt: frames are streamed to Gemini Live for semantic understanding of gestures/actions. Imitation only in *follow mode*.
- **Intent feed**: transcript of the live conversation + intent cards (each tool call = one card with status planning/running/done/failed and the physics verdict).

## Server -> browser
Per frame (~20-30 Hz), three messages:
1. TEXT JSON state (below)
2. BINARY: first byte `0x01` + JPEG of your camera (960x540, already mirrored)
3. BINARY: first byte `0x02` + JPEG of the robot view (640x480)

State JSON keys (all always present):
```
mode            "IDLE" | "FOLLOW" | "TASK" | "PATH" | "PREVIEW"     (FOLLOW = imitating the user)
view            "robot" | "orbit"
feasibility     0..1                    messages  [str]  robot read-back (unreachable, joint limit, collision, too fast)
speed_scale     float                   mirror    bool
fps, ik_hz      float                   source    "webcam" | "video" | "synthetic"
pose            [[u,v] x33] | null      normalized coords in the mirrored camera image (0..1); draw skeleton on camera view
pose_ok         bool                    objects   [[label, score, u1, v1, u2, v2]]  boxes in camera image
markers         [{id:"A", pos:[x,y,z], uv:[u,v]|null}]   uv = projection in current robot view (null if behind camera)
path            {uv:[[u,v]...], ok:[bool...], i:int, hand:"left"|"right"} | null    projected into current robot view
task            {id:str, action:str, status:"planning"|"running"|"done"|"failed", message:str, step:str} | null
holding         {"left": "mug"|null, "right": "mug"|null}       (robot hands)
scene_objects   [{name:"mug", pos:[x,y,z], uv:[u,v]|null}]
regions         ["left","right","center","front","back"]  named table regions usable as targets
skills          [str]   learned skill names
recording       {active:bool, seconds:float}
saved           str|null   last export folder
```

## Browser -> server (TEXT JSON)
```
{type:"tap",  u, v}                       tap on robot view -> raycast -> new marker (A, B, C...). Max 6, oldest dropped.
{type:"draw", points:[[u,v]...], hand?}   stroke on robot view -> raycast each -> 3D path for that hand, pre-checked. Then {type:"task",action:"follow_path"}.
{type:"clear"}                            clear markers and path
{type:"view", name:"robot"|"orbit"}
{type:"camera", dazimuth?, delevation?, zoom?}   orbit camera only
{type:"task", id, action, ...args}        see actions; server answers via state.task (same id)
```
Task actions (these mirror the Gemini tool declarations 1:1):
| action | args | meaning |
|---|---|---|
| `pick_place` | `object` (e.g. "mug"), `target` (region name, marker id like "A", or omitted), `target_uv` ([u,v] on robot view, optional; wins over target) | pick the object and place it at the target |
| `reach` | `target` / `target_uv` | move a hand to hover over the target |
| `follow_path` | | execute the drawn path |
| `follow_me` | `enabled` bool | imitation mode on/off (auto-records while on) |
| `learn_skill` | `name` | save the last follow-mode recording as a named skill |
| `play_skill` | `name` | replay a learned skill (validated, speed-scaled) |
| `set_speed` | `value` ("slower"/"faster"/number 0.1..1) | |
| `stop` | | stop current motion, hold pose |
| `home` | | back to the start pose, mug kept where it is |
| `reset_scene` | | home + mug back to the start |
| `export` | | write intent.json / trajectory.json / report.json, returns folder in `saved` |

`state.task.message` is a short natural sentence the assistant can speak back, e.g.
"Planned 9 waypoints, all reachable. Picking up the mug." or "I can't: the back of the table is 18 cm beyond my right arm's reach."
For Gemini tool responses: send `{ok, message}` once `task.status` becomes `running` (accepted), `done`, or `failed` for that id (whichever first; do not block > 8 s).

## Browser-only (web/gemini.js) contract
`window.PCGemini`:
```
PCGemini.connectLive({apiKey, model, systemInstruction, tools, onStatus(s), onTranscript({role:"user"|"model", text, final}),
                      onToolCall({id, name, args}) -> Promise<object>, onAudioLevel?(0..1)}) -> Promise<session>
session.startMic() / session.stopMic()             realtime audio in (16 kHz PCM16), model audio out played (24 kHz)
session.sendImage(jpegBase64, mime="image/jpeg")   your-camera frames ~1 fps (and optionally the robot view)
session.sendText(text)
session.close()
PCGemini.point({apiKey, model, imageBase64, query}) -> Promise<[{label, u, v}]>   Gemini Robotics-ER pointing on an image, u,v in 0..1
PCGemini.DEFAULT_LIVE_MODEL, PCGemini.DEFAULT_ER_MODEL
```
API key + model names are entered in the site's Settings dialog and kept in localStorage (never sent to our server).
Without a key: fall back to the browser Web Speech API (continuous, interim results) + the rule parser in app.js.
