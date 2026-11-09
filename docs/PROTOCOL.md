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
view            "robot" | "orbit" | "stage"   (stage = full-body view, used automatically while imitating a video)
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
video           {name, progress 0..1} | null    an uploaded video being imitated (max 90 s used)
skills_meta     [{name, duration}]
user_holding    {"l": label|null, "r": label|null}   the user's hands vs objects detected in your camera
```
`path.uv` entries can be null (behind the camera): skip them.

## Video upload
`POST /upload` multipart, field `video` -> `{ok, name, message}`. The server switches your-camera to that video, the robot
imitates it whole-body (arms, torso twist, squat, foot lifts) in the stage view, and saves a skill named after the file
when it ends, then returns to the webcam.

## Browser -> server (TEXT JSON)
```
{type:"tap",  u, v}                       tap on robot view -> raycast -> new marker (A, B, C...). Max 6, oldest dropped.
{type:"draw", points:[[u,v]...], hand?}   stroke on robot view -> raycast each -> 3D path for that hand, pre-checked. Then {type:"task",action:"follow_path"}.
{type:"clear"}                            clear markers and path
{type:"view", name:"robot"|"orbit"|"stage"}
{type:"camera", dazimuth?, delevation?, zoom?}   orbit / stage camera
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
| `edit_skill` | `name`, `t0`, `t1`, `part` (left_arm/right_arm/arms/legs/body), `speed`?, `amplitude`? | edit one body part over a time range |
| `compile_for` | `robot` ("ur5e") | compile the current path / last recording for another robot and report reachability |

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

## Teach by Showing (core UX)
The robot does not copy the user live. It watches a demonstration (your camera) plus speech/text, extracts a task spec,
asks back, practices in simulation with its own body, and takes corrections.
```
{type:"teach", cmd:"start"}                       begin watching (hands, grasps, objects in your camera)
{type:"teach", cmd:"utterance", text}             every final user utterance; while confirm/done it is treated as a correction
{type:"teach", cmd:"stop"}                        extract the spec -> phase "confirm" with a question
{type:"teach", cmd:"correct", text}               "yes"/"응" practices; anything else edits the spec
{type:"teach", cmd:"practice"} | {cmd:"save", name} | {cmd:"set_spec", spec} | {cmd:"cancel"}
```
`state.teach = {phase: idle|recording|confirm|practice|done, seconds, utterances, spec:{goal, steps:[{id, action, object,
target, status, message}], constraints:[{type, object?}]}, question, evidence:{hand_uv, grasps, moved, objects_seen}}`.
Objects: mug, red_block (movable); vase, blue_block (fixed obstacles). Targets: left/right/center/front/back, marker A-F,
"there" (pointing marker P or the side the hand moved toward), "next to <object>" (free side chosen automatically).
Saved task skills replay with `play_skill`.

## Other additions
- `{type:"mock", enabled}`: offline brain (pointing -> marker P, both hands up -> stop, wave), narrates verdicts as `state.say`.
- `{type:"path3d", points:[[x,y,z]...], hand?}`: 3D route from the browser editor; `pick_place` target "along my path" carries along it.
- `GET /scene.json`, `GET /mesh/<id>.bin`, `state.geom_poses`: everything a browser 3D view needs.

## v3 additions (demo build)
One-liner everywhere: **"A new generation of interactive prompt for Physical AI."** Sub-line: "Text can't carry where, which path,
or what not to touch. Show it, say it, point at it: the robot understands, checks it against physics, and does it."

- **Phone / browser camera**: client sends BINARY `0x10 + JPEG` (≈640x360, ≤10 fps) from getUserMedia. While such frames
  arrive (<1 s old) the server uses them as "your camera" (`state.source = "phone"`), else the laptop webcam.
- **Robot choice (robot-independent)**: `{type:"robot", name:"g1"|"ur5e"}`. `state.robot`, `state.robots = [{id, label}]`.
  The same intents/teach specs run on either body; each has its own sim world. After a switch `/scene.json` changes
  (3D view must reload it when `state.robot` changes).
- **Spatial model**: `{type:"spatial_model", name:"yolo-seg"|"mediapipe"|"off"}`; `{type:"spatial", target:"camera"|"robot", query?}`
  runs Gemini Robotics-ER on that frame. `state.spatial = {model, objects:[{label, score, box:[u1,v1,u2,v2], poly:[[u,v]...]|null}],
  er:{target, query, points:[{label,u,v}], ms}|null}` (camera objects use mirrored camera coords like `state.objects`).
- **Server-side Gemini** (key kept on the server, never sent to browsers): `state.server_gemini = true` when available.
  Live proxy WebSocket: `/gemini/live` (same messages as Google's BidiGenerateContent, no key). REST proxy: `POST /gemini/rest/<model>:generateContent`.
- **Demo autopilot** (scripted mock so the demo always works): `{type:"demo", name:"full"|"text_vs_spatial"|"teach"|"stop"}`.
  `state.demo = {name, step, total, title} | null`. It drives the real system (tasks, teach, 3D route) and narrates through
  `state.say` with kinds "user" (what the person says/types), "robot", "vision", "caption" (on-screen caption, e.g. "Text only").

## v4 (UX 2.0)
Features plug in through `pps/features/<name>.py`, `web/ui/<name>.js` and `scripts/checks/<name>.py` (see docs/UX2.md).

### Teach timeline (#F) — `pps/features/timeline.py`, `web/ui/timeline.js` (slot "timeline")
Each understood step is a card (object -> target, constraint chips). Reorder (drag, move buttons, arrow keys on the
step number), delete and chip toggles need no new message: the browser sends the edited spec through the existing
`{type:"teach", cmd:"set_spec", spec:{goal, steps:[{object, target}], constraints}}` (phase goes back to "confirm").
A switched-off chip is only kept in the browser so it can be switched back on. "Show again" re-records one step:
```
{type:"timeline", op:"reshow", index}     keep the spec, start recording a new demonstration for step <index>
                                          (only in phase confirm/done; ends with the usual {type:"teach", cmd:"stop"})
{type:"timeline", op:"reshow_cancel"}     stop that recording and restore the old spec
```
On stop, the first understood step replaces step `index`; the other steps keep their place and new constraints are
merged in (a new grasp_top/grasp_side replaces the old one). `state.timeline = {reshow: {index, steps:[{object, target}],
constraints} | null}` (the steps being kept while recording). The dock gets a "Show" tool that starts or finishes a
demonstration. Saved skills are a strip of cards: play sends task `play_skill`; motion skills (in `state.skills_meta`)
edit a time range of one body part with task `edit_skill {name, part, t0?, t1?, speed | amplitude}`.
