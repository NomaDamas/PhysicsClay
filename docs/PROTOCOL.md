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
  (or named) object to the offered hand, clamped to reach; present it there (0.8 s), then release once the person's
  hand has come to it (within 7 cm, or 4 cm closer than when the arm stopped and within 12 cm) and stays 0.4 s.
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

## v6 · Watching the person (v1 P3, `pps/human/`, `pps/features/watch.py`, `web/ui/watch.js`)
The robot understands what the person is doing (not only where they point) and acts on it while it watches them.

**The person** (`state.human`, every frame, robot world frame; built from co-presence's joints):
```
{visible, src: "phone"|"webcam"|"video"|"feed"|"synthetic", t,
 pose: [[x,y,z] * 33] (MediaPipe order), pose_filled: [indices estimated from the head / ankles],
 hands: {right|left: {p, q: [w,x,y,z] (x along the hand, y toward the index), pinch (m), open, near_robot, src: "hands"|"pose"}},
 holding: "torch"|"mug"|null, holding_hand, offer: "right"|"left"|null,
 activity: "squat"|"press"|"curl"|"wave"|"weave"|"reach"|"point"|"still"|"moving"|"none", phase: "top"|"down"|"bottom"|"up"|null,
 reps, stalled, speed (m/s), fatigue (0..1), weave: {amp, freq, travel, axis, dir, side}|null, summary, summary_src: "rules"|"gemini"|"openai"}
```
- Activity is rule-based (`pps/human/activity.py`): squat (knee angle + hip drop with a real down-and-up), press,
  curl, wave (hand above the shoulder swinging), weave (a hand below the shoulder oscillating across its own travel:
  detrended path, principal axis, zero crossings), reach, point (armed co-presence pointing), still, moving. Reps:
  hysteresis on the activity's normalised signal (top → down → bottom → up → top = one rep); the reps already done
  when the activity is recognised are counted too. `stalled`: mid-rep on the way up, the signal stops for 0.7 s.
- Held tool: `{type:"watch", op:"tool", name}` (what the person says) > the camera's object detector > the closest
  movable sim object within 10 cm of a hand.
- Hands from the browser: with this device's camera on and a watch mode on, `web/ui/watch.js` runs MediaPipe Hands
  (tasks-vision from jsDelivr, model from Google's model storage) and sends `{type:"hands", hands:[{uv:[u,v] (wrist,
  image 0..1), lm:[[x,y,z] * 9] (world metres, keypoints 0,4,5,8,9,12,16,17,20), score}]}` at ~8 Hz. The server
  matches them to the pose's wrists by image distance (handedness labels of an unmirrored camera are unreliable).
- Optional VLM summary (`pps/human/vlm.py`): `{type:"watch", op:"summary", on}`; Gemini, then OpenAI vision, with the
  server's key (none with `--byok`), at most every 8 s, only with a real camera, never with `--mock`, the synthetic
  person or `PC_OFFLINE=1`.

**Modes**: `{type:"watch", op:"mode", mode: "off"|"mirror"|"learn"|"respond"|"assist"|"coach", hand?: "auto"|"left"|"right",
target?: "hands"|<object>, max_force?, name?}`.
- `mirror`: the G1 runs whole-body follow-me; other bodies move each end effector by the person's wrist offset from
  their shoulder, mirrored face to face (toward the robot → away from the robot's base; their right hand → the robot's
  left hand on two-arm bodies; one-arm bodies follow the hand held up/out more), anchored where the hand was when
  mirroring started, clamped to reach and above the table, ≤ 4 cm per frame; the hand's yaw turns the gripper (±70°);
  a pinch (< 3.5 cm, browser hands) or a fist closes the gripper (`rt.set_gripper` when the runtime has one, else a
  kinematic grasp of an object within 7 cm; opening releases it). Uses the P2 `teleop` message when that handler is
  registered. Pauses while a task runs; "stop" ends it.
  `watch.mirror = {kind: "hands"|"whole_body", paused, pairs: [{person, robot, target, ee, grip, yaw_deg}]}`.
- `learn`: records the working hand (p, q, pinch, open), held tool, activity/phase/reps, the objects; `op:"learn_stop",
  name?` (or any mode change) saves `recordings/demos/<name>.json` (format in `pps/human/demo.py`: frames, tool,
  activity, reps, `pattern` (weave amp/freq/travel), `anchor` object (when the hand worked within 25 cm of one),
  `rep_stats`, and `trajectory`: a P2 trajectory object, `source: "demo"`, `frame: "object:<anchor>"|"world"`).
  `op:"replay", name?, preview? (default true)` places that trajectory on its anchor object (or at the end effector)
  as the drawn path and sends `follow_path` (a plan card when preview). `watch.learn = {recording, seconds, frames, last}`.
- `respond`: `{type:"watch", op:"rule", when, do, expr?, task?, cooldown? (8 s), hold? (0.4 s), preview?}` or
  `{op:"rule", text:"when I put my hand out, give me the wrench"}`. `when`: built-in terms joined by and/or/not —
  `hand_open_near_robot, hand_out, hand_open, holding, holding:<tool>, not_holding, activity:<name>, phase:<name>,
  reps>=N, stalled, still, pointing, visible, fast` — or everyday words compiled offline (`pps/human/rules.py`), or
  `expr`: a restricted expression over `state.human` (comparisons, and/or/not, dotted fields; no calls) that the agent
  compiles from free text. `do`: a task (`give me the X` → `handover`, `stop`), `say …`, `help me`/`spot` (→ assist),
  or anything else as an instruction for the LLM agent. A rule fires on the rising edge after `hold` s, then needs
  `cooldown` s and the condition to clear, and never while its own task (id `watch-R<n>-<k>`) is still running.
  Rules also run in assist and coach; never for the synthetic no-camera dancer. `op:"rule_remove", rule`, `op:"rules_clear"`.
  `watch.rules = [{id, when, expr, when_text, do, kind, task, cooldown, hold, enabled, fired, last_age}]`.
- `assist`: the hands follow a point 10 cm below the person's hands (or a named object), clamped to reach, ≤ 2 cm per
  frame; when the person stalls mid-rep the target rises up to 6 cm and `watch.assist.request = {force_n: max_force
  (40), dir: [0,0,1]}` asks P0 impedance control for capped help. `watch.assist = {phase: "following"|"helping"|"paused"|"idle",
  target, focus, lift, ee, max_force, request}`.
- `coach`: per rep, cues against a reference (`pps/human/coach.py`): squat depth (knee angle at the bottom), knee
  valgus (knee width / ankle width at the bottom), back angle, tempo; curl range and elbow drift; press lockout; weave
  width / frequency / travel speed. `op:"coach_ref", demo` uses a recorded demo as the reference. It counts every rep
  aloud and says a cue the first time and every third repeat (`state.say`). `watch.coach = {activity, reps, last: {rep,
  cues, text}, cues: {key: count}, history, ref}`.
- `op:"reset_reps"`, `op:"tool", name`. Every op answers in `watch.reply = {id, op, ok, message, rule?, demo?, mode?}`.
- Brain: `observe` returns `human` (minus the pose) and `watch`; the context has `person` and `watch` lines; tools
  `set_watch_mode`, `add_watch_rule`, `remove_watch_rule`, `learn_from_demo`, `mirror_person`.
- Browser: the "Watch" card (side slot): mode chips, live activity / phase / reps / held tool, the mode's line
  (recording + Done, last demo + Repeat, coach cue, assist state), rules with delete, a "when …, …" box. With the agent
  off, typed/spoken "copy me", "watch and learn", "done", "your turn", "coach me", "spot me", "stop watching" and
  "when I …, …" are handled there (`pc-hear`).
- Test only: `pps/human/synth.py` builds synthetic people (squats with valgus / stall, curls, presses, waves, a weave,
  an offered hand, a hand at a point) as landmark frames for `{type:"copresence_feed"}`.

## v7 · Safety around people (v1 P5, `pps/safety.py`, `pps/features/safety.py`, `web/ui/safety.js`)
Speed and separation monitoring in the spirit of ISO/TS 15066. **Not a certified safety system** (`docs/safety.md`).

**State** (every frame): `safety = {person, d_min (m, surface to surface, negative = touching), speed_scale (0..1),
stop, latched, reason: "body"|"critical"|"no_force"|"force"|null, zone: "clear"|"slow"|"stop"|null (no person),
closest: {robot_link, robot_kind: "ee"|"arm"|"body", robot_ee, human_part, region, d, p_robot, p_human}, binding
(the pair that sets the speed / caused the stop, same shape), message ("Slowing: you're 30 cm away" | "Stopped: too
close to your head" | null), coop (assist, mirror or a handover), force (N, or null without P0 force sensing), v_robot, v_human (m/s),
capsules: [[ax, ay, az, bx, by, bz, r, region] * 17], config: {...}, reply}`.
- A person counts when co-presence's source is `webcam`, `phone`, `vr` or `feed` (a played person in checks); never
  the synthetic no-camera dancer or a replayed video. Turning the co-presence avatar off does not turn safety off.
- Motion: both runtimes read `rt.safety_scale` in `step()`: paths, skills, previews and driving advance at `dt * s`,
  the IK joint velocity is scaled by `s` (live mirroring and assist slow too), `s = 0` holds everything still.
- A stop pauses a running task: `task.status` stays `running`, `task.step = "paused"`, `task.paused = "safety"`,
  `task.message = "Paused for safety: … I'll go on when you step back."`; the message is restored on resume.
- Pre-task guard (`ext.guard`, before every other pre_task): during a stop, `pick_place`, `reach`, `follow_path`,
  `play_skill`, `handover`, `drive`, `home`, `follow_me` fail with "I can't move right now: …"; a `pick_place` /
  `reach` target (or object) inside the person's body zone fails with "I won't reach there: that spot is inside your
  body zone …". `reset_scene` and `stop` are always allowed.
- Planners: `tasks.safety.check_point(p, role, radius=0.04, coop=None) -> {ok, d, part, region, why, role}` (also
  `pps.features.safety.check_point(ctx, p, role)`), plus `tasks.safety.scale` and `tasks.safety.stopped`.
- Messages: `{type:"safety", op:"resume", id?}` confirms a latched (force-limit) stop once the person is clear. Any
  other op answers `safety.reply = {id, op, ok: false, message: "Safety is always on…"}` and changes nothing.
- Config is server side: `pps.safety.Config` defaults, the robot profile's `safety` dict, then
  `--safety-config file.json` / `PC_SAFETY_CONFIG`; every value is clamped to a floor.
- Handover: the robot now offers the object 10 cm in front of the person's hand (`HANDOVER_GAP`), so it never has to
  touch them; they close the gap.

## v8 · Datasets (v1 P1, `pps/data/`, `pps/features/dataset.py`, `web/ui/dataset.js`)
Every task can become a LeRobot v2.1 episode (`docs/datasets.md` has the format and how to train on it).

**Messages** (each answers in `dataset.reply = {id, op, ok, message}`):
- `{type:"dataset", op:"start", name?, auto: true, fps?: 30, cameras?: ["front","wrist"] (+"robot"), size?: 256}`:
  record every task that starts moving into `datasets/<name>/<robot>/` (an existing dataset of that name and robot is
  appended to). `auto: false` keeps recording on but captures nothing.
- `{type:"dataset", op:"stop"}`: end recording; an episode still running is dropped.
- `{type:"dataset", op:"discard_last"}`: delete the last finished episode (files and metadata); asynchronous, the
  result lands in `dataset.message`.
- `{type:"dataset", op:"randomize", lighting?: 0..1, camera?: metres, seed?}`: per-episode randomization of the
  dataset cameras only (light directions / intensity, camera position and angle); `{}` turns it off.
- `{type:"dataset", op:"meta", data: {...}}`: JSON (≤ 4 kB) stored as `variation` in the following episodes' records.
- Task messages may carry `utterance` (or `text`): the words that asked for the task, used as the episode's
  language. The browser adds the user's last line (≤ 30 s old) to every task it sends.

**State** (every frame): `dataset = {recording, name, auto, fps, size, cameras, episodes, succeeded, refused, frames,
current: {task, frames, seconds} | null, last: {i, robot, task, success, outcome, seconds, frames} | null,
list: [same, last 30], root, datasets: [LeRobot roots written], pending (frames waiting for the dataset thread),
cost_ms: {snapshot, render}, randomize, error, message, reply}`. `outcome`: `done` | `failed` | `interrupted` |
`world_changed` | `timeout` | `finished` (the motion ended but its status was superseded).

## v9 · Trajectories in 3D (v1 P2, `pps/traj.py`, `pps/features/traj.py`, `web/ui/traj.js`)
Language first: the agent writes a few keypoints relative to what it sees, the server turns them into a smooth, checked
3D path and shows it as a ghost; the person fixes only what is wrong (drag a handle, Alt-drag / wheel for height, the
wrist ring, per point chips, or "higher there"). Drawing from scratch (strokes, 3D waypoints, keyframes) still works and
produces the same object.

**The trajectory object** (one schema for every source; `pps/traj.py normalize()` is the validator):
```
{id, role: "right"|"left", frame: "world"|"object:<name>", source: "draw"|"vr"|"language"|"demo"|"keyframes"|"skill", label,
 points: [{p: [x,y,z], q: [w,x,y,z]|null, t: s|null, grip: 0..1|null, speed: m/s|null, force: N|null, tol: m|null, note|null,
           yaw?: deg, tilt?: deg, hold?: s}],
 pattern: {kind: "zigzag"|"circle"|"figure8", amp: m, freq: Hz, axis: "lateral"|"normal"|"vertical", span?: [i0, i1]} | null,
 check: {ok: [bool per point], why: str|null, duration: s} | null, meta?: {...}}
```
- `frame: "object:<name>"`: points are offsets from that object's **top centre**, axes parallel to the world, resolved
  against the object's current pose every time the path is prepared (it moves with the object; it does not turn with it).
- `q` absolute wins; otherwise the hand's approach orientation (captured when the world is built: down for arms) turned
  by `yaw` (about the vertical, + = counter-clockwise from above) and then `tilt` (leans the tool's tip toward the
  direction of travel). Orientation is slerped between points.
- Timing: if every point has `t`, those times are used (demonstrations). Otherwise the hand travels at each point's
  `speed` (linear in between; default per body kind: arm 0.25, humanoid / bimanual 0.20, mobile 0.15 m/s), with
  min-jerk-shaped starts and stops (0.5 s). The hand stops at the first and last point, at `hold` points, and for
  0.4 s wherever the gripper is commanded (`grip` set and different from before).
- `grip` and `force` hold from their point on. `force` is passed to `rt.set_force(palm, N)` when the runtime has it (P0
  impedance); `grip` to `rt.set_gripper(palm, v)`, else a kinematic grasp of the object at the hand (released on open).
- `pattern` is superimposed in time (`freq` Hz of motion) across the path (`lateral`/`normal`: horizontal and
  perpendicular to travel; `vertical`: up and down), faded in and out; `span` limits it to points i0..i1. A
  demonstration's pattern only describes the weave already in its points (not applied again) unless `apply: true`.

**Messages** `{type: "traj", op, id?}` (every op answers in `state.traj.reply = {id, op, ok, message, traj?, n?, duration?,
why?, plan?, task?, notes?}`):
| op | fields | |
|---|---|---|
| `propose` | `traj` (object) \| `spec` (language keypoints, below) \| `demo` (a P3 demo name); `task_id?` | check it, show the ghost and a plan card (state.plan, action `follow_path`, id `task_id`); nothing moves until "go" (`{type:"plan", op:"confirm"}`), the go-on gesture, `op:"run"` or the F key |
| `set` | `traj` \| `spec` \| `demo`; `run?: bool`; `task_id?` | make it the current path (no card); `run` starts it right away |
| `edit` | `index` (int or list, negatives from the end) \| `near` ("there", a marker, an object, start/end/middle/all) + `point: {p?, yaw?, tilt?, speed?, force?, grip?, tol?, hold?, note?}` and/or `changes: {dx, dy, dz, height, speed (m/s \| "slower"/"faster"), speed_factor, force, grip, tol, yaw, tilt, hold, note, remove}`; `insert: {after, p?}`; `remove: index`; `pattern: {...} \| null`; `profile: {lift, arch}` (m, the 2D fallback); `label`; `world` (default true: `p` is in world metres) | edits the proposed path (else the current one) and checks it again; a proposal stays a proposal (the card updates) |
| `run` | | runs the proposal (= plan confirm), the current path, or the last one when nothing is shown |
| `clear` | | drops the proposal and the current path (stops it first if it is running) |

Language keypoints (`spec`, what the agent's `make_trajectory` sends; `pps/traj.py from_keypoints`):
```
{label, object?: anchor (the path then moves with it), role? (left|right; default: the hand on the path's side), speed?, force?, grip?, tol?, clearance? (0.05 m), pattern?,
 keypoints: [{where?: "object"|"pass"|"point"|"step"|"marker"|"region"|"hand" (only that form's fields are read; guessed when absent)}
           & ({of: "red_cup", rel: "above"|"at"|"left_of"|"right_of"|"in_front_of"|"behind"|"beside"|"around", dz?, dist?, turns?, dir?}
           | {of: "vase", via: "over"|"around_left"|"around_right", clearance?} | {xyz: [x,y,z]} | {dxyz: [dx,dy,dz]}
           | {marker: "A"|"P"} | {region: "left"} | {from: "hand"};  each + dx?, dy?, height? (above the surface),
             speed?, force?, grip?, tol?, yaw?, tilt?, hold?, note?, pattern?: bool)]}
```
`above` = dz (0.10) over the object's top, `at` = 0.02 over it, `left_of`… = `dist` (0.06) from its side at `dz` (0.10)
over the table (robot's view: `in_front_of` = toward the robot), `around` = a loop of 12 points per turn. Every
segment (and the move from the hand to the start) that would pass through or too low over an object gets lifted:
points are added where it enters and leaves the object's footprint (inflated by the hand), at top + clearance, unless
the segment ends on that object on purpose (going down to touch or grasp it). `meta.lifted` / `meta.notes` say so.

**Checks** (per sample, the first objection wins; `check.ok` per point, `why` groups consecutive failures: "Out of my
reach by 22 cm from point 3 to point 4."): reach and joint limits (IK precheck along the samples, with each sample's
orientation and the point's `tol`), the work surface (pressing points may go 2 cm in), objects (the hand going into
one; what it carries hitting one; objects it grasps along the way are exempt), keep-out zones, then any
`pps.features.traj.register_checker(fn(ctx, pts, quats, info) -> [reason|None])` (P5 safety plugs in here). The move
from the hand to the first point is checked too ("on the way to the start").

**State** `state.traj` (every frame):
```
{current: traj|null, proposed: traj|null, ok: bool|null, why: str|null, running: bool, i: point index being approached,
 progress: 0..1, t: s, rev, reply, plan_id,
 ghost: {id, pts: [[x,y,z]] (<= 160 samples), ok: [bool], q: [[w,x,y,z]] | null (null: the wrist keeps kq[0]), t: [s], key: [sample index of each point],
         keys: [[x,y,z]] (world, per point), kq: [[w,x,y,z]] (per point), grip: [per point], approach: [hand, start],
         approach_ok, origin: [x,y,z]|null, surface, duration, speed, hand, axis: [x,y,z] (the gripper's approach axis
         in its site frame)} | null,
 info: [{i, h (m above the surface), near?, speed?, force?, grip?, yaw?, tilt?, hold?, note?}], issues: [{where, i, span, why}],
 objects: [{name, pos, r, top, movable}], surface}
```
- An installed path *is* `rt.path` (`timed: true`, dense 30 Hz samples): the robot-view overlay, `follow_path`, F, "Run
  path", plan preview of `follow_path` and "carry it along my path" all keep working. The runtime follows it in time
  (`pps/traj.py follow_step`, called from `Runtime.update_targets`), starting with a min-jerk move from wherever the
  hand target is; `speed_scale` still scales time, and moving the shown points of a running path (the nudge pad, the
  come-here gesture) moves the rest of the trajectory with them.
- Old messages: a stroke (`draw`) or 3D waypoints (`path3d`) go through `tasks.set_path_world` as before and are
  adopted as a trajectory with `source: "draw"` (their own reach / zone verdicts are kept, so behaviour is unchanged);
  editing one checks it fully. `clear`, `home` and other tasks that replace `rt.path` drop it (the last one is kept for
  `op:"run"`). P3's `watch` `replay` proposes the demonstration through `propose {demo}` (timing, gripper, anchor object).
- Agent tools (when the body has `follow_path`): `make_trajectory(description, keypoints[], object?, role?, speed?,
  force?, clearance?, pattern?)` → `propose` (or `set` + run when confirm-before-moving is off); `edit_trajectory(point? |
  near?, changes?, pattern?, remove_pattern?)` → `edit` (point is 1-based as shown); `run_trajectory()` → `run`. The
  context sent with every model call has `trajectory: {state, label, source, frame, points: [{n, h_cm, near, ...}],
  duration_s, ok, why, pattern}` so "higher there" can name points.
- Browser (`web/ui/traj.js`): the ghost (tube blue / red per sample, dashed approach, a translucent gripper at every
  point and one riding along at the planned speed), handles (drag on the horizontal plane; Alt-drag, the wheel or the
  popover's ± for height, with a guide line and a height label to the surface or object below; the ring turns the
  wrist), a popover for the selected point (height, speed, gripper, press force, wrist yaw / tilt, precision, insert,
  remove), the "Path" dock tool (click 2–5 spots: keyframe grippers that connect into a proposed path), a card in the
  side slot (state, why, the point strip, Go/Run, add, clear), keyboard (arrows / PageUp / PageDown / [ ] / Delete / Esc
  on the selected point) and, on the robot's view, Lift / Arch sliders for a stroke.
- **For VR (follow-up):** send the same object. A pinch-drawn path → `{type:"traj", op:"set"|"propose", traj: {source:
  "vr", frame: "world", points: [{p, q?, speed? (from pinch distance), grip?}]}}`; a grabbed ghost gripper moved to a new
  pose → `{op:"edit", index, point: {p, yaw?} | {p, q}}`; a dropped keyframe → `{op:"edit", insert: {after, p}}` (or a
  `propose` with `source: "keyframes"`); confirm with `{op:"run"}`. Draw from `state.traj.ghost` (`keys`, `kq`, `pts`,
  `ok`, `axis`). Live teleoperation (`{type:"teleop", role, p, q, grip}`) is not registered by P2; P3's mirror mode
  already uses it when a handler exists.

## v10 · VR / MR and live teleop (v1 P2b, `web/xr.js`, `web/xr-core.js`, `pps/features/teleop.py`; guide: `docs/vr.md`)
The headset is the same page (Quest browser, https on the LAN). It sends only the messages above: a pinch stroke is a
v9 `traj` `propose` with `source: "vr"`, `frame: "world"` (≤ 60 points, every 1.5 cm, `speed` per point when "Speed: as
drawn / pinch" is on); a grabbed handle is `{op:"edit", index, world:true, point:{p, q?}}` (`q` = the wrist turn applied
to the point's orientation, sent when it is over 10°); a keyframe is `{op:"edit", insert:{after, p}}` (or a 2-point
`propose` with `source:"keyframes"`); Undo re-proposes the previous trajectory; Go is `{op:"run"}`; "Use my room" is
`{type:"world", op:"set", scene}` (the table → `template`, things on it → fixed `box` objects, current movable objects
kept). Message ids from the headset start with `vr-`.

**Teleop** `{type: "teleop", …}` (answers in `state.teleop.reply = {id, op, ok, message, t}`):
| message | |
|---|---|
| `{op:"start", role?, src?}` | take over that hand: stops a running path / task / drive, camera follow-me and P3 mirroring; the hand holds where it is |
| `{role, p:[x,y,z], q?:[w,x,y,z], grip?:0..1, t?, src?}` | end-effector target (world, metres) at up to ~30 Hz; `t` (the client's clock) is echoed back |
| `{op:"stop"}` | let go: the hand stays where it is |

Every target is clamped to the arm's reach (a sphere around the shoulder; mobile bases 1.6 m) and above the work surface
(plus what the hand carries), rate limited (1 m/s, 3 rad/s, × P5 `rt.safety_scale`), ignored during a protective stop
(`rt.safety_scale <= 0` or `tasks.safety.stopped`) or when `tasks.safety.check_point` says it is inside the person, and
ignored while a task / path runs (`held: "busy …"`). It goes into the hand's mocap target (the path P3 mirroring uses;
P3's mirror mode now sends bare commands through this handler). `grip` → `rt.set_gripper(palm, g)` when the runtime has
it, else closing next to an object grasps it (kinematic), opening releases. A bare command (no `start`) is an implicit
start; 3 s without a command ends teleop.

`state.teleop = {active, role, src, rate (commands in the last second), lag_ms (age of the newest), stale, echo (its t),
held (why the last command was not applied | null), clamped, scale (safety speed scale), hands: {right|left: {p, q, ee,
grip, holding}}, reply, events}`.
## v11 · Skills from demonstration + demo packs (v1 P4, `pps/skills/`, `pps/features/skills.py`, `web/ui/skills.js`)
Details: `docs/skills.md` and `docs/packs.md`; as-built notes in `docs/V1.md` (P4).
- `{type:"skill", op:"extract", demo?, kind?, name?, target?, propose?}` a demonstration (default: the last `learn`) →
  a parametric skill saved in `skills/<name>.json` and, when the scene has something to apply it to, proposed as a P2
  trajectory (`source: "skill"`) with a plan card whose `say` is the one sentence to confirm ("… Right?"). When two kinds
  fit about equally the robot asks through `state.choices` (options are skill kinds, answered with `choose`). `learn`'s
  stop triggers it by itself.
- `{op:"apply", name, target?, overrides?, preview?, task_id?}` compile on this scene → `traj propose` (preview) /
  `traj set` + run, the watch `assist` behaviour (`spot`) or `play_skill` (`mirror_motion`).
- `{op:"edit", name?, changes? | text?}` parameters (dotted paths, values or `{mul}` / `{add}`; or the person's words:
  "wider", "a bit slower", "more push", "closer", "6 mm", "3 Hz") → saved → recompiled → re-proposed under the same plan id.
- `{op:"list"}`, `{op:"delete", name}`, `{op:"bead_clear"}`, `{op:"pack_demo"}` (the loaded pack's synthetic person).
- Replies: `state.skillbook.reply = {id, op, ok, message, skill?, notes?, plan?, question?, changed?, task?}`.
- `state.skillbook = {skills, current, reply, rev, seams, pack, beads: [{id, seam, r, live, pts}], spot, bar, kinds}`;
  `state.plan.skill = {name, kind, describe, label, target}` while a skill is proposed.
- `{type:"world", op:"pack", name, robot?}` loads a demo pack (`weld_cell`, `gym`, `workshop`). `state.world.packs`
  lists them, `state.world.pack` names the loaded one; `state.world.robot.tool = {kind, label, tcp}` when a tool is
  mounted. Scene spec additions: `pack`, `tool`, `seams`, per object `exact` and `mass`; kinds `steel_plate`, `bench`,
  `barbell`, `wrench`, `screwdriver`, `brush`; template `gym`.
- P2 point addition: `roll` (degrees, lean about the travel direction). Trajectory `meta.work_on`: objects the P2 object
  check lets the tool touch or enter; `meta.skill`: the skill, its target and (welds) the seam polyline and arc span.
- Watch additions: `op:"mode"` takes `quiet` and, for `assist`, `spot: {v_min, for_s, lift, bar?, skill?}` (help when the
  bar, the hands' midpoint, is mid-range and slower than `v_min` for `for_s`); `state.watch.assist.bar = {vz, stalled,
  slow_s, range}`. Demonstrations skip the synthetic no-camera person and record `smoothing: {ema, dt}`.
- With P0 physics the server sends itself `{type:"physics", op:"set", params:{impedance, force_limit}}` around spotting.
- Agent tools: `learn_skill_from_demo`, `list_skills`, `apply_skill`, `edit_skill_params`, `load_pack`.

## v12 · Physics (v1 P0, `pps/physics.py`)
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
  `height` m and lets it fall (a test probe); `op:"place", object, pos: [x, y]` sets it down at rest there (a test probe). `mass_scale` scales the objects (not the robot); `grip_force` (N, default
  40) is the squeeze of every gripper; `impedance: true` softens the arms (stiffness x0.2, effort x0.4) and caps the
  end-effector force at `force_limit` (N) while pressing.
- `{type:"task", action:"press", target, force: N, hold_s}`: move over the spot (or an object's top), come down, track
  `force` along the approach axis with an admittance loop for `hold_s` seconds, retreat. The task message reports the
  mean and peak force (and "capped by the ... impedance limit" when the limit was lower than asked). Physics only.
- `pick_place` with physics first works out how (`state.task.status: "planning"`, "Working out how to pick up …", a
  slice per frame) and then runs or is refused; clients wait past `planning` for the verdict.
- `pick_place` with physics: the jaws open on the way in, close at the grasp (the path waits), and the object is held
  only by friction. The task fails honestly with "It slipped: ..." (the object didn't come up / fell out), "Too heavy:
  ... I can lift X kg" (weight sensed once it hangs free), "I let go of ... but it fell over / ended up N cm from the
  spot" (checked after the release) or "... is N cm wide and my gripper opens M cm". The done message ends with the measured placement error ("..., 0.8 cm from the spot").
  The hand stops over the grasp and over the spot until the arm has lined up, the way down follows where the object
  really is (in the hand, or if it moved), and it lets go once the object rests on the surface.
- `{type:"task", action:"handover", object, to: "left"|"right", target?}` (or `between_arms: true`) on two-armed bodies:
  pass the object from one hand to the other (pick by the near end, meet in the middle, the other hand takes the free
  end, the first lets go), then put it on `target`. `state.task.action` is `handover` throughout; one-armed bodies and
  objects too short for two grippers are refused with the reason. `preview: true` gives a plan card (#B).

## v13 · Calibration (v1 P6, `pps/calib/`, `pps/features/calib.py`, `web/ui/calib.js`)
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

## v14 · UX polish: camera in, phone camera, voice, view zoom

* **Camera in.** `state.cam_in = {fps, mobile} | null`: a browser camera is streaming `0x10 + JPEG` frames to the server
  (fps over the last 2 s; `mobile` when the last sender's user agent is a phone or tablet). The laptop UI shows
  "Camera on · 15 fps" or "Phone connected · 15 fps" from it.
* **Phone camera page.** `GET /phone` serves `web/phone.html`, a light page that only streams the phone's camera over
  the same `/ws` (getUserMedia needs https on a phone: run with `--https`). `GET /phone/info` →
  `{ip, port, https_port | null, url | null}` (`url` = `https://<lan-ip>:<https_port>/phone`, shown as a QR code).
* **Voice.** `GET /tts?voice&lang&text` answers `200 audio/mpeg`, or `204` with `X-TTS: no-key | key-rejected |
  upstream-<status> | upstream-unreachable` when there is no working OpenAI key (the browser then speaks with its own
  voice; 204 keeps the console clean). `voice` is one of OpenAI's voices (default `coral`); a key OpenAI rejected is
  not retried for the rest of the run. `state.tts_server`: the server has an OpenAI key (never with `--byok`).
* **Server views zoom.** `{type: "camera", zoom}`: one proportional factor per message (the browser sums wheel deltas
  and sends at most every 30 ms); the server clamps it to 0.25–4 and the distance to 0.35–8 m.

## v15 · Worlds: homes and industrial sites (issue #48, `pps/world/{fixtures,objects,rooms}.py`, `pps/manip.py`, `pps/features/fixtures.py`)
Guide, templates, fixtures, per-robot results and how to add your own: `docs/worlds.md`.
- `{type:"world", op:"set", template: "kitchen_full"|"dining"|"living_room"|"laundry"|"assembly_line"|"warehouse"|
  "machine_tending"|"electronics_bench"|"lab"|"packing", seed?, robot?}` builds a generated world (same seed, same
  layout). `{op:"set", seed}` alone re-generates the current one with another layout ("shuffle"); `{op:"set", scene:
  {template, seed, objects?}}` also works (objects are added on the work surface); `{op:"pack", name: <template>, seed?}`
  loads one like a demo pack (the current robot when it fits, else a Panda). `add_object` / `remove_object` / switching
  the robot keep the generated world as it is.
- The scene spec of a generated world (`state.world.scene`) carries `seed`, `surface` `{top, x, y, label, floor_dx?}`,
  `fixtures` `[{name, kind, pos, yaw, size, ...}]` and `room` `{label, group}`; its objects have a 3D `pos` (centre), a
  `quat` and `in` (the container they started in). `state.world.templates` entries have `group` (`basic` | `home` |
  `industrial`), `summary` and `seeded`.
- `state.world.fixtures = [{name, kind: drawer|door|lid|rack, label, open: 0..1, joint, type: slide|hinge, fixture,
  locked?}, {name, kind: conveyor|rollers, label, running, speed, holding: object|null}]`, `state.world.containers =
  [{name, label, kind, objects: [...], behind?: part, open?}]` (refreshed twice a second), `state.world.seed`. Each
  `scene_objects` entry has `in` when the object is in a container. Moving parts are bodies, so `/scene.json` marks their
  geoms `dynamic` and `geom_poses` streams them (the 3D view renders drawers and doors live).
- Task actions: `{type:"task", action:"open", fixture, amount?: 0..1 (0.8)}`, `{action:"close", fixture}`,
  `{action:"pick_from", container, object?}`, `{action:"place_in", object?, container}`. `fixture` / `container` are a
  name or what people say ("the top drawer", "the fridge door", "the empty tote", "the tube rack" = its first free
  slot). Answers in `state.task` like any task (`planning` while the ways in to a handle are tried, one per tick →
  `running` → `done` | `failed` with the reason: out of reach, a knob blocked from above and from the front, no grip, slipped off the handle, stuck at N % open, "open the top drawer first", didn't end up in
  the container). With `preview: true` each becomes a one-step plan card.
- `{type:"task", action:"sequence", id, steps: [task, ...], text?, preview?}`: several steps in a row. With `preview`, a
  plan card (`state.plan` with `seq: true`, `steps` = one per task, the first step checked for real, `phase`
  proposing | blocked); `{type:"plan", op:"confirm"}` runs it, `cancel` stops it. Steps run with ids `<id>#1`, `<id>#2`…
  and the sequence ends as the task `<id>` (`done`: the steps' results joined; `failed`: "Step 2 of 4 (take the spoon out
  of the top drawer) didn't work: ..."). `state.sequence = {id, status: running|done|failed, i, n, message, steps: [{text,
  result}]}`.
- `{type:"fixture", op:"run"|"stop", name?, speed?}` starts / stops a conveyor (it stops by itself while a part sits at
  its photo-eye and starts again when the part is taken); `op:"lock"|"unlock", name` jams / frees a drawer or door (a
  test probe); `op:"set", name, open: 0..1` puts a part at an opening (a setup probe: what is inside doesn't move with
  it). Answers in `state.task` (action `fixture`) when the message has an `id`.
- Agent tools: `open_fixture`, `close_fixture`, `pick_from`, `place_in`, `plan_steps`, `conveyor`; `load_world {name, seed?}` and
  `build_scene` take the world templates and a `seed`; the context has `world.fixtures`, `world.containers` and each
  object's `in`.

## v16 · Real photos → simulated objects and spaces (#50, `pps/recon/`, `pps/features/recon.py`, `web/ui/recon.js`; guide: `docs/RECON.md`)

HTTP (multipart; keys in headers `X-Gemini-Key`, `X-Fal-Key`, `X-Meshy-Key`, used for that one job, never stored):

* `POST /recon/import` — `photo` (1–4), `depth`? (16-bit PNG in mm or `.npy` in m, for photo 1), `meta` = JSON
  `{provider: "primitive" | "triposr" | "fal_trellis" | "meshy", hints: {label?, size? ("9 cm tall"), material?,
  crop? [u0, v0, u1, v1] 0–1, reference? "card" | "a4", mass_kg?, vlm? false}, K? [fx, fy, cx, cy], tag?}` →
  `{ok, job}` (400 `{ok: false, message}` for a bad upload).
* `POST /recon/scan` — `photo`, `depth`?, `meta` `{hints: {surface?, template?, vlm?}, K?, tag?}` → `{ok, job}`.
* `GET /recon/job/<id>` → the job in full: `{id, kind: "object" | "space", status: queued | running | ready | failed |
  added, step, progress, message, error, steps: [{id, label, state}], seconds, added, summary}`. Object summary:
  `{label, kind, material, hollow, recognized_by, provider: {id, label, license}, shape, size_cm [x, y, z], mass_g,
  mass_source, friction, scale: {source, label, confidence, factor}, scales [...], confidence, views, mask: {source,
  rect}, depth: metric | relative | none, faces, hulls, collision, watertight, notes, marker?}`. Space summary:
  `{count, surface, template, layout: {surface, scale, objects: [{name, label, kind, color, size_m [x, y, z], xy, box}]},
  spec, spec_real?, objects_brief, notes}`.
* `GET /recon/job/<id>/mesh.bin` (preview: `[nv, nf]` u32, verts f32, faces u32, then RGB u8 per vertex),
  `/recon/job/<id>/overlay.jpg` (outline or boxes), `/recon/colors/<lib>.bin` (RGB u8 per visual vertex of a library
  object), `/recon/library.json`, `/recon/library/<id>/photo.jpg`, `/recon/info.json` (providers with licenses, depth
  model status, library, marker), `/recon/marker.png` (print-at-home ArUco, DICT_4X4_50 id 0, 5 cm).

Websocket:

* `{type: "recon", op: "import", source: "camera", provider?, hints?, keys?: {gemini, fal, meshy}, tag?, add?}` — the
  freshest phone / browser camera frame (else the laptop webcam); `add: true` adds it to the scene when ready (agent).
* `{type: "recon", op: "scan", source: "camera", hints?, keys?, use?, shapes?}` — `use: true` builds the scene when ready.
* `{type: "recon", op: "add", job, height_m?, name?, movable?}` — save to the library, then `{type: "world", op:
  "add_object", object: {name, kind, mesh: <lib id>, scale, movable}}`.
* `{type: "recon", op: "use_layout", job, shapes?: "primitive" | "real", template?}` — `world set` with the layout
  (`real`: each object reconstructed and saved first).
* `{type: "recon", op: "add_library", id, height_m?, name?}` · `{type: "recon", op: "delete_library", id}`.

State: `recon = {jobs: [last 3 job summaries], providers: {id: usable}, depth: "ready" | "on first use" | "downloading"
| "off", library: <count>, reply: {id, ok, op, message, job?, lib?, name?}}`.

Scene specs: an object may carry `mesh: <library id>` and `scale` (its kind stays a scene kind: cup, can, bottle, bowl,
plate, fruit, vase, block, box). `/scene.json` mesh entries of imported objects add `colors: "/recon/colors/<lib>.bin"`.
