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

### Plan preview (#B): the robot proposes, you confirm
- A motion task (`pick_place`, `reach`, `follow_path`, `play_skill`) sent with **`preview: true`** is planned but not run:
  the server resolves the target and runs the same IK reach precheck, free-spot search, carry-through-obstacle check and
  nearest-reachable-spot search as a real run, then puts everything back. Nothing moves. Without `preview` (demo, self test,
  buttons, keys) tasks run exactly as before. Other actions (`stop`, `home`, ...) and taught multi-step task skills ignore
  `preview`.
- The task gets `state.task.status = "proposed"` (message: the intent sentence + "Say go, or tap Run.") when the plan is
  feasible, or `"failed"` with the reason when it is blocked.
- `{type:"plan", op:"confirm", id?}` runs the proposed task with the **same task id** (the card then goes
  planning → running → done/failed as usual). Ignored when the plan is blocked.
  `{type:"plan", op:"cancel", id?}` drops a waiting plan (task → failed "Cancelled. Nothing moved."), stops a running one,
  or clears a finished one. `id` is optional; a stale id is ignored.
- `state.plan = null` when idle, else:
  ```
  {id, action, phase: "proposing"|"needs_you"|"blocked"|"running"|"done"|"failed", ok, intent, verdict, reasons: [str],
   message, say, hand: "left"|"right"|null,
   target: {label, pos:[x,y,z]} | null,               // world/robot frame, z = table top for placements
   steps: [{text, at}],                                // at = fraction of the path where the step starts (0..1)
   constraints: [str],                                 // chips: "Keep it upright", "Clear of the vase", ...
   waypoints: [[x,y,z]...], wp_ok: [bool...],          // densified hand path (<= 90 pts), per-point reach/collision ok
   carry: {object, label, shape: "cyl"|"box", r, half_h, from:[x,y,z], to:[x,y,z]} | null,   // carried-object ghost
   alternative: {text, label?, task: {action, ...args} | null} | null,                     // nearest thing it can do
   question?,                                          // needs_you: yes = send alternative.task (with preview)
   progress?, step_i?,                                 // running / done
   task: {id, action, ...args}}                        // what confirm will run
  ```
  `needs_you` = blocked, with a runnable alternative ("Put it on marker B instead?"); `blocked` = no runnable alternative.
  A waiting or finished plan is dropped as soon as another task replaces `state.task`.
- Browser (`web/ui/plan.js`): typed, voice, Gemini and example-chip motion commands carry `preview: true`
  (`PCUI.taskFilter`, called by app.js `runTask`; `?preview=0` turns it off); "yes / go / do it" confirms and
  "no / cancel" cancels (a `pc-hear` listener; see "Hearing utterances" below). The card lives in slot `plan`; the ghost
  (path, numbered steps, target ring, carried object) in `PC3D.layer("plan")`.

### C · Candidates: the robot asks which one
- A `pick_place` task whose object words match several movable objects ("the block" with a red and a green block, or "thing") is not run. The task gets `status: "asking"` and the question as its message, and the state gets
  `choices: {id, question, options: [{key: "1"|"2"|…, name, label, color, pos: [x,y,z], uv: [u,v]|null}]}`, numbered left to right as the robot sees them.
- Optional on the task: `object_pos: [x,y,z]` or `object_uv: [u,v]` (where you pointed). One object within 10 cm is used directly; two or more ask.
- Answer: `{type: "choose", id, key}`. `key` can be the number, an object name, free text ("the red one", "two", "2번", "left"), `"cancel"`, or `pos`/`uv` instead of `key`. The original request then resumes, going through the task pre-hooks again (so plan preview still applies).
- A new marker near a candidate (a tap or pointing) also answers. A new task, or 120 s passing, clears the question.
- Scene: `green_block` is a new movable object (a 2.6 cm cube).

### Hearing utterances (all UX 2.0 modules)
Every final typed or spoken utterance (text box, browser speech, Gemini Live transcript) first fires a cancelable
`window` event `pc-hear` with `detail: {text, phase}` (teach phase), except while a demonstration is being recorded.
A module that handles the utterance calls `preventDefault()`, and the normal intent parser is skipped.

### Keep-out zones (#D) — `pps/features/zones.py`, `web/ui/zones.js`
- `{type:"zone", op:"add", rect:[x0,y0,x1,y1] | center:[x,y] + size:[w,d] | center + shape:"circle" + r, height?, label?, source?}`:
  a volume standing on the table top (robot frame, metres; default height 0.30). `source`: "paint" | "voice" | "text".
- `{type:"zone", op:"add", near:"vase"|"mug"|"red block"|"blue block"|<any target resolve() knows>, source:"voice"}`: a circle
  around that object (its radius + 6 cm, its height + 6 cm). The web client sends it when a final utterance (typed or spoken)
  says "stay away from X", "keep clear of X", "don't go near / don't touch X", "avoid X", "without touching X" (or 꽃병 근처에 가지 마).
  If the sentence also asks for a pick/put/move, the zone is sent first and the task still runs.
- `{type:"zone", op:"remove", id}` · `{type:"zone", op:"clear"}`.
- `state.zones = [{id:"A", shape:"rect"|"circle", center:[x,y,z_table], size:[w,d], height, label, name, near, source}]`
  (`label` is the sentence form used in reasons, e.g. "keep-out zone A", "the vase keep-out zone"; `name` is the short chip text).
- Voice/text changes are acknowledged through `state.say`.
- Planning (hook `tasks.keepout`, no-op without zones): drawn/3D path waypoints inside a zone are `ok:false` and
  `messages`/`follow_path` name the zone; a task whose hand or carried object would enter a zone fails naming it; `pick_place`
  detours around (either side) or over zones, checked for reach, and says so ("…, going around keep-out zone A"); place spots
  inside a zone shift to a free spot. A drawn route is re-checked when zones change.
- Web: `web/app.js` fires a cancelable `pc-hear` window event `{detail:{text, phase}}` for every final user utterance before the
  rule parser; a module that handles it calls `preventDefault()`.

### Goal: make it look like this (#E, `pps/features/goal.py`, `web/ui/goal.js`)
Show the end state instead of the steps: arrange ghost copies of the movable objects in the 3D stage, or send a photo of
the layout you want. The server orders the moves and runs them one by one.
```
{type:"goal", op:"set", targets:{<object>:[x, y]}, run?:bool, source?:"arrange"|"photo"}   plan (and run) a goal
{type:"goal", op:"photo", jpg_b64, view?:"user"|"robot", offline?:bool, run?:bool}       read a photo (base64 JPEG, < 1.5 MB)
{type:"goal", op:"run"}                                                                 re-plan from the current scene and run
{type:"goal", op:"cancel"} | {type:"goal", op:"clear"}
```
- Object names are the keys of `tasks.objs` (`mug`, `red_block`, and any movable object added later); x, y are world
  metres on the table top.
- Planning: targets are clamped onto the table and moved clear of fixed obstacles and of each other; objects already
  within 3 cm are skipped; a target still occupied by another object is freed first (that object moves first, or is set
  aside when it is part of a swap or has no target); a carry whose straight line passes an obstacle goes over it.
- Running: one `pick_place` per move (task ids `goal-<rev>-<i>`), dispatched from `@ext.tick` when the previous one
  is done. A move that lands more than 3 cm off is redone once; the first failed move stops the sequence with its reason.
  Any other task (stop, home, a new instruction) stops the goal.
- Photo: Gemini Robotics-ER with the server key (falls back to Flash, then to an offline colour-blob reader on 429/503,
  timeout or no key; `offline:true` forces it). `view:"user"` means the photo was taken from your side facing the robot
  (robot at the top of the photo); `"robot"` means from the robot's side. Positions are taken relative to the table
  (Gemini's table box, else the whole photo) and mapped onto the reachable front strip of the table, x 0.25..0.47, y -0.36..0.36.

`state.goal = {status: idle|locating|ready|running|done|failed|stopped, message, targets:{name:[x,y]}, source,
sequence:[{i, object, label, kind:"place"|"aside", to:[x,y], status: pending|running|done|failed, err_cm}], i, rev,
photo:{source:"gemini"|"offline", view, summary, found:[{name,u,v}], box, ms, error}|null,
objects:[{name, label, pos:[x,y,z], r, h, shape:"box"|"cyl"}], obstacles:[{name, pos:[x,y], r}], tolerance_cm, work_area}`.

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

### G · Gestures and nudge pad (`pps/features/gestures.py`, `web/ui/gestures.js`, `web/ui/nudge.js`)
- **Gestures** from the pose landmarks (the server exposes them to features as `ctx.lm`, `ctx.pts`, `ctx.source`):
  `stop` both hands above the head (only when the mock brain is off; it owns stop when on) · `slower` both palms pressing
  down, hands apart in front of the waist -> `speed_scale` x0.7 · `come_here` beckon with one hand -> a running target
  moves 4 cm toward you (+x), otherwise the robot reaches toward you, carrying whatever it holds · `go_on` thumb up or an
  open palm pushed forward -> confirms the proposed plan (via `ctx.store["confirm_hook"]` when a feature sets one), else
  releases a hold, runs a drawn path that is waiting, or acknowledges. Static gestures must be held still 0.5-0.7 s;
  each fires once and must end before it can fire again (2 s global cooldown). Off during the demo, teach recording,
  video imitation, follow-me, and on synthetic/video input.
- `state.gestures = {live: name|null, active: bool, last: {id, name, label, effect}|null, last_age: s|null}`.
- **Nudge**: `{type:"nudge", dx, dy, dz}` metres in the robot frame (x forward, y left, z up), each clamped to ±5 cm,
  total ±10 cm per target. The active target, in order: a feature's `ctx.store["nudge_hook"]` (e.g. the plan preview),
  the running task (waypoints after the grasp up to the release, fading out on the way home; refused if unreachable),
  the latest marker (x/y only), a drawn path that hasn't run yet.
  `state.nudge = {active, target: label|null, kind: "hook"|"running"|"marker"|"path"|null, offset: [dx,dy,dz], last: {id, message, ok}|null}`.
- **Speed**: `{type:"speed", scale}` sets `speed_scale` (0.1-1.0).
- Test only: `{type:"gesture_feed", frames:[landmarks...], fps}` plays synthetic landmark frames through the detector.
- Feature hooks (optional): `ctx.store["nudge_hook"] = fn(ctx, delta)` (delta `None` -> label of the target it owns or `None`;
  a numpy delta -> apply, return a message) and `ctx.store["confirm_hook"] = fn(ctx)` (message if it confirmed something).

## v5 (UX 3)

### Brain: the LLM agent (`pps/agent.py`, `pps/features/agent.py`, `web/ui/agent.js`)
- `{type:"agent", text, confirm?: bool (default true), llm?: {base_url, key, model, effort?}}` starts a turn;
  `{type:"agent", op:"reset"}` clears the conversation. One turn at a time: a message while busy gets an `error` event.
- The model is any OpenAI-compatible Responses API (`POST {base_url}/responses`, function tools, `store:false`).
  Server config: `.env` `LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL` (default `gpt-6.1-terra`), `LLM_EFFORT` (default
  `medium`). With `--byok` the server key is never used: the browser sends `llm` with each message (stored only in that
  browser, used for that turn only, never in state or logs); its `base_url` must be public `https`.
- Each turn runs in a background thread (at most 8 model calls). The agent acts only through browser messages
  (`task`, `plan`, `choose`, `zone`, `goal`, `point3d`, `teach`, `world`) and reads the latest state frame. Task ids it
  launches start with `ag` (the browser hides app.js's own cards for them; the tool cards cover them).
- Tools come from the robot's capabilities (`state.world.robot.capabilities`, G1 defaults without it): `observe`,
  `pick_place(object, target | marker | xyz)`, `reach`, `follow_path`, `play_skill`, `go_home`, `stop`, `reset_scene`,
  `set_speed`, `add_zone(near | rect)`, `remove_zone`, `clear_zones`, `set_goal_layout(targets)`, `run_goal`,
  `choose_candidate(key)`, `confirm_plan`, `cancel_plan`, `follow_me`, `start_teaching`, and the world tools `set_robot`,
  `build_scene`, `add_object`, `remove_object` (they answer "not available in this build" while `state.world` is absent).
- Confirm before moving (default on): motion tools send `preview:true`; once a plan card is proposing, the agent replies
  ("…say go") and won't launch another motion that turn. "go"/"yes"/"응" is heard by the plan card on `pc-hear`; when the
  confirmed task ends, the agent reports the result (`result` event + robot line) and remembers it for the next turn.
- System prompt: `pps/world/prompts/base.md` + `pps/world/prompts/<robot id>.md` when present (else a built-in prompt),
  plus a compact JSON of the current world (robot, objects, markers, zones, plan card, open question, skills).
- Replies are spoken through `state.say` (kind `robot`), so the feed and TTS work as for any robot line.
- `state.agent = {available, busy, model, confirm, byok, events: [{id, kind: "user"|"thinking"|"tool"|"result"|"reply"|"error", text, tool?, args?, ok?, t}]}`
  (last 40). A `tool` event appears with no `ok` while it runs and gets `ok` + a one-line result when it returns.
- Browser: after `pc-hear` is not handled, app.js fires a cancelable `pc-intent` (`detail: {text, source}`). agent.js
  takes it when the agent is on and available (server LLM, or a BYOK key) — except stop words, which go straight to the
  robot — otherwise the UX 2.0 rule parser runs. Settings ("Agent brain"): use the agent, confirm before moving, and the
  BYOK endpoint/key/model/effort (`localStorage` `pc.agentOn`, `pc.agentConfirm`, `pc.llmBase`, `pc.llmKey`, `pc.llmModel`, `pc.llmEffort`).

### Co-presence: the person is in the robot's world (`pps/features/copresence.py`, `web/ui/copresence.js`)
The pose (MediaPipe world landmarks, or the synthetic dancer with `--fake`) drives a mannequin standing across the
table (hip centre x 1.15, facing the robot, not mirrored: your right hand is on the robot's left). It is drawn into the
robot-eye render (through `@ext.overlay`) and into the 3D stage (`PC3D.layer("copresence")`).
- `state.copresence = {on, view, source, anchor, joints: {name: [x,y,z]}|null, pointing: {side, from, dir, hit, armed,
  target, candidates}|null, recent_point, hand_open: {l, r}, offer: "l"|"r"|null, attention: {kind: "person"|"object"|"spot",
  name, pos}|null, handover: {phase: "carrying"|"offering"|"given", object, target, hand}|null, given, hand_paths: {l, r},
  reply}`. World frame (metres, x forward from the robot, y to its left, z up).
- **Pointing**: shoulder -> index fingertip ray onto the table top. Armed after the arm is extended toward the table and
  held still for 0.5 s; an open hand held out is an offer, not a point. Movable objects within 6 cm of the ray are
  `candidates`; while armed, marker `P` follows the hit (so "there" and the "which one?" question use it).
- `{type:"copresence", op:"utter", text, id, run?: bool, preview?: bool}` resolves this/that/here/there
  (이거/저거/여기/저기, "give it to me"/줘) against the pointing at speech time (1.5 s history; "put this there" pairs
  the earlier pointing with the later one) -> `state.copresence.reply = {id, ok, kind, object, candidates, spot, marker,
  message, task}`. `task` is the task message to send (the browser runs it with `PC.runTask`, so plan preview and
  "which one?" apply). With `run: true` the server runs it itself (the path for the Brain agent, which only sends
  messages). In-process: `resolve_deixis(ctx, text) -> same dict` (adds a marker for "put it there").
- `{type:"task", action:"pick_place", hold:true, object, object_pos?|candidates?}` picks up and keeps holding.
- `{type:"task", action:"handover", object?, hand?: [x,y,z]}` or `{type:"copresence", op:"handover"}`: carry the held
  (or named) object to the offered hand, clamped to reach; release once the person's hand is within 12 cm.
- `op:"view", on` (Shared space): wider robot eye (90 deg) pitched up to see the person, turning slightly toward the
  attention target; restored when off. `op:"on"|"off"` shows/hides the person. `op:"heard"` (any utterance) makes the
  robot attend to the person. `op:"place", x, y` moves where the person stands.
- `op:"demo"` (no camera needed): a synthetic person points at the mug, at a spot, then holds out a hand (24 s);
  `phase: point_mug|point_spot|offer|take|rest` holds one pose for 30 s.
- `hand_path(ctx, side, seconds)`: the person's wrist path in robot-world coordinates (for Teach-by-Showing).
- Test only: `{type:"copresence_feed", frames:[landmarks...], fps, hold}`; `synth_pose(point_at|offer)` builds frames.
- Browser: typed or spoken text with deixis is claimed from `pc-hear` (preventDefault) only when there is a recent armed
  pointing (or it is a handover request); otherwise the normal parser gets it.
- `ext.overlay(fn)`: `fn(ctx, mjvScene)` adds geoms to each robot-eye render after the runtime's own overlays.

## v6 (v1 · P0 physics, `pps/physics.py`)
Physics is the default; `python -m pps.app --kinematic` (or `PC_PHYSICS=0`) keeps the kinematic mode, where everything
below reports `mode: "kinematic"` and the `physics` message is refused. Details and limits: `docs/V1.md` (P0, as built).
- `state.physics = {mode: "dynamic"|"kinematic", dt: 0.002, rtf, substeps, time, grasp: {role: object|null},
  ee_force: {role: [fx, fy, fz]}, contacts: [{a, b, f}], objects: {name: {p: [x,y,z], q: [w,x,y,z], v}}, params}`.
  `grasp` names the object held by contact on both jaws; `ee_force` is the contact force on the hand (wrist-sensor-like,
  N, world frame, smoothed); `contacts` are the strongest contacts involving objects or the hands (`a`/`b` are
  `<role>_pad`, `<role>_hand`, `robot`, `obj_<name>`, `table`, ...; `f` normal force in N); `rtf` is simulated / wall time.
- `{type:"physics", op:"set", params:{friction_scale, mass_scale, damping_scale, kp_scale, latency_ms, gravity,
  grip_force, impedance, force_limit}}` changes the plant at run time (unknown keys ignored, values clamped; answers in
  `state.task`, action `physics`). `op:"reset"` restores the defaults. `op:"drop", object, height` lifts an object by
  `height` m and lets it fall (a test probe). `mass_scale` scales the objects (not the robot); `grip_force` (N, default
  40) is the squeeze of every gripper; `impedance: true` softens the arms (stiffness x0.2, effort x0.4) and caps the
  end-effector force at `force_limit` (N) while pressing.
- `{type:"task", action:"press", target, force: N, hold_s}`: move over the spot (or an object's top), come down, track
  `force` along the approach axis with an admittance loop for `hold_s` seconds, retreat. The task message reports the
  mean and peak force (and "capped by the ... impedance limit" when the limit was lower than asked). Physics only.
- `pick_place` with physics: the jaws open on the way in, close at the grasp (the path waits), and the object is held
  only by friction. The task fails honestly with "It slipped: ..." (the object didn't come up / fell out), "Too heavy:
  ... I can lift X kg" (weight sensed once it hangs free), "I let go of ... but it fell over / ended up N cm from the
  spot" (checked after the release) or "... is N cm wide and my gripper opens M cm".

## v7 (v1 · P6 calibration, `pps/calib/`, `pps/features/calib.py`, `web/ui/calib.js`)
Method, math and results: `docs/calibration.md`.
- `{type:"physics", op:"set"}` also takes `armature_scale` and `frictionloss_scale` (the robot joints' rotor inertia and
  Coulomb friction); `latency_ms` now delays the servo setpoint per 2 ms substep (interpolated), not per server tick.
- `{type:"calib", op:"probe", probes?: ["swing","press","lift","slide"]}` runs the probe routine on the robot (a task,
  action `calib`, `step` = the probe running; other tasks are refused until it ends, `stop` ends it). `op:"fit",
  budget?: "quick"|"normal"|"thorough"` fits the simulator to the last probe logs in worker processes; `op:"apply"` puts
  the fit into the plant (as `physics set`); `op:"reset"` restores the default physics and drops the fit; `op:"run"`
  does probe + fit + apply; `op:"stop"` stops probing or fitting.
- `state.calib = {status: idle|probing|probed|fitting|fitted|applied|failed, message, summary, probes: [{name, label,
  status: waiting|running|done|skipped}], progress: {stage, stage_index, stages, gen, gens} | null, logs: [{probe,
  seconds}], params: {name: value} | null, per_param: [{name, value, default, lo, hi, unit, what, spread}], fit:
  {err_before, err_after, seconds, evals, budget, stages, logs: [{probe, before, after}]} | null, applied, defaults}`.
- Agent tool `calibrate_here {budget?}` (physics only): probe + fit + apply, answers with `state.calib.summary`.
