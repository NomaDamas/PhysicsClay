# UX 3 — real robots, real scenes, a real brain (contract between the three workstreams)

UX 2.0 made the robot propose and the human correct, but one robot (Unitree G1), one table, and a scripted
mock brain. UX 3 makes it real:

* **World** (`pps/world/`): many robot bodies (arms, bimanual, humanoids, mobile manipulators) and scenes that
  can be built on the fly, by the UI or by the AI ("a kitchen counter with a bowl and two cups, use the Panda").
* **Brain** (`pps/agent.py`): an LLM agent (OpenAI-compatible Responses API, tool calling) that turns what you say
  into tool calls against the world, observes the result, and replies. No scripted mock in this path.
* **Robot UX** (`web/ui/robots.js`, `pps/world/prompts/`): every robot gets its own prompt, capabilities, example
  requests and dock tools, because an arm, a two-arm rig and a humanoid are instructed differently.

The UX 2.0 principles stay: say · point · show, the robot proposes (plan card) and you correct.

## 1. World (owner: World workstream)

### Robot profiles — `pps/world/profiles.py`
```python
PROFILES: dict[str, RobotProfile]
RobotProfile(id, label, kind, model_xml, mount, ee, home_key, base, capabilities, camera, prompt)
```
* `kind`: `"humanoid" | "arm" | "bimanual" | "mobile_manipulator" | "quadruped_arm"`
* `ee`: end-effector sites keyed by role, e.g. `{"right": "right_palm", "left": "left_palm"}` (G1),
  `{"right": "gripper"}` (Panda). Single arms use the role `"right"` so existing task code keeps working.
* `base`: `"legged" | "fixed" | "mobile"`. `mount`: where the robot stands or is bolted relative to the table.
* `capabilities` (strings): `pick_place, reach, follow_path, bimanual, handover, imitate_video, follow_me,
  teach_by_showing, balance, drive_base`.
* Models come from `vendor/mink/examples/*` (already on disk and test-loaded): Unitree G1 (current), Franka
  Panda, UR5e, uFactory xArm7, KUKA iiwa 14, ALOHA (two arms), Apptronik Apollo, Hello Robot Stretch 3,
  Stanford TidyBot, Spot with arm. A robot that can't be made to pick & place reliably is listed as
  `experimental` with an honest note rather than faked.

### Scenes — `pps/world/scene.py`
```json
{"template": "table" | "kitchen" | "shelf" | "floor",
 "objects": [{"name": "mug", "kind": "mug|block|bowl|cup|bottle|can|box|plate|fruit", "color": "red",
              "size": [sx, sy, sz] | null, "pos": [x, y] | "left" | "center" | ..., "movable": true}]}
```
Movable objects become mocap bodies named `obj_<name>`; fixed ones `obs_<name>`. The default world
(G1 + the current table) must behave exactly as today, so the UX 2.0 self test keeps passing.
With physics (v1 P0, the default) the same spec is compiled twice: the IK planner keeps the objects as mocap, the plant
makes them free bodies with mass and friction (same names), adds grippers / servos and welds legged bases; see
`docs/V1.md` (P0, as built). `--kinematic` / `PC_PHYSICS=0` builds exactly the model above.

### Messages and state
* `{type: "world", op: "set", robot?: id, scene?: spec}` rebuilds the world (robot and/or scene). Ops
  `add_object {object}`, `remove_object {name}`, `reset` also exist.
* `state.world = {rev, robot: {id, label, kind, base, ee: [roles], capabilities: [...], experimental?: bool,
  note?: str}, scene: spec, robots: [{id, label, kind, capabilities, experimental?}], templates: [...]}`.
  `rev` increments on every rebuild; the browser reloads `/scene.json` and the 3D stage when it changes.
* `state.robot` / `state.robots` (UX 2.0 keys) stay as aliases.

### As built (World workstream notes for Brain / Robot UX)
* Robot ids: `g1, panda, ur5e, xarm7, iiwa14, aloha, apollo, stretch3, tidybot, spot, h2` (`h2` is experimental: no
  standing keyframe, reach only, no pick & place). Unknown kinds/colours in a scene spec fall back sensibly; `kind`
  also accepts `vase`. An object without a name is called `<color>_<kind>` when a colour is given, else `<kind>`.
* Every `world` op answers in `state.task` (`action: "world"`, `done` with a one-sentence summary, or `failed` with
  the reason). `reset` goes back to the default world (G1 + the UX 2.0 table). `set` keeps whatever it isn't given
  (`{op: "set", robot: "panda"}` keeps the scene, `{op: "set", scene: {...}}` keeps the robot).
* `state.world` also carries `surface` (`top`, `x`, `y`), `regions` (named spots, relative to the surface's near edge)
  and `notes` (e.g. an object moved onto the surface). `/scene.json` carries `surface` so the 3D view raycasts it.
* Task action `drive {dx, dy (m, world frame), dyaw (deg)}` for `drive_base` bodies (TidyBot: holonomic; Stretch:
  along the table only). Bases also move by themselves inside the IK when a target is out of the arm's reach.
* Humanoid-only actions (`follow_me`, `learn_skill`, `edit_skill`, motion `play_skill`, `teach` start, video upload)
  are refused on other bodies with a message naming the robot; the scripted demos run only in the default world.
* Task action `handover {object, to: "left"|"right", target?}` (or `between_arms: true`) on two-armed bodies (ALOHA):
  one arm picks the object by the end nearest it, both hands meet in the middle, the other hand closes on the free
  end, the hold is checked, the first hand lets go and backs off, then the receiver puts it on `target` (or keeps it).
  It needs a long object (about 20 cm or more, both grippers on it); shorter ones are refused with the set-down
  alternative. Without `to` / `between_arms`, `handover` stays the co-presence handover to the person. Agent tool
  `handover_arms`; plan card with the steps.
* With physics (V1 P0) every body but H2 does a real contact pick & place except where its note says otherwise (Stretch
  3: the SG3 fingers; Spot: the mug by its rim; TidyBot: small blocks 8/10); per-robot results in `docs/V1.md` "P0 as built" and
  `scripts/bench_grasp.py`. Physics-only profile tuning lives in `RobotProfile.physics` (gripper pads, servo stiffness,
  physics home pose, contact excludes, a riser under a mount: `mount.z`).

### Generated worlds (issue #48, `docs/worlds.md`)
Besides the surface templates, `template` can be a generated home or industrial site with a `seed`:
```json
{"template": "kitchen_full" | "dining" | "living_room" | "laundry" | "assembly_line" | "warehouse" | "machine_tending" |
             "electronics_bench" | "lab" | "packing",
 "seed": 7, "objects": [...]}
```
`pps/world/rooms.py` turns `(template, seed)` into the full spec: `surface` (the station in front of the robot, its
near edge at x = 0.22), `fixtures` (`pps/world/fixtures.py`: cabinets with drawers and doors, a sink, a fridge,
conveyors, racks, totes, a CNC, tube racks...) and objects with an exact 3D `pos` + `quat` (clutter settled by a
headless physics drop) and the container they are `in`. More object kinds (`pps/world/objects.py`): utensils,
tableware, towels, books, tools, industrial parts, lab items. Fixtures' moving parts are mocap bodies in the planner and
jointed bodies in the plant (`fx_<fixture>_<part>`); `state.world.fixtures` / `containers` report them and the task
actions `open`, `close`, `pick_from`, `place_in` and `sequence` work with them (`docs/PROTOCOL.md` "v11 · Worlds").
Up to 60 objects in a generated world (16 on a template).

## 2. Brain (owner: Brain workstream) — `pps/agent.py`
* **Config:**
  * Locally it reads `.env`: `LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL` (default `gpt-6.1-terra`), and
    `LLM_EFFORT` (default `medium`). Locally that is MyProxy at `https://proxy.nomadamas.org/v1`.
  * With `--byok` the server key is never used. The browser sends `{base_url, key, model}` with each
    `agent` message, stored only in that browser.
* **Conversation:**
  * `{type: "agent", text, llm?}` starts or continues a conversation turn and `{type: "agent", op: "reset"}` clears it.
  * `state.agent = {available, busy, model, events: [{id, kind: "user"|"thinking"|"tool"|"result"|"reply"|"error", text, tool?, args?, ok?}]}`.
* **How it acts:**
  * It runs in a background thread and talks to the world **only through the same messages a browser
    sends** (`shared.push_cmd`) and reads the latest state snapshot, like a client. Nothing in the worker
    loop blocks on the LLM.
  * Tools are generated from the current robot's capabilities, for example:
    * `observe`
    * `pick_place(object, target | xyz | marker)`, `reach`, `follow_path`, `go_home`, `stop`, `set_speed`
    * `add_zone`, `set_goal_layout`
    * `choose_candidate`
    * `set_robot`, `build_scene`, `add_object`, `remove_object`
    * `teach_*` (humanoid only)
  * The system prompt is `pps/world/prompts/base.md` plus `pps/world/prompts/<robot id>.md`.
  * Motion tools go through plan preview (`preview: true`) when the user has "confirm before moving" on
    (default on). The agent then says "here's my plan, say go", and "go" confirms the plan.
* **When no LLM is available:** the browser falls back to the UX 2.0 parser and the mock. The UI shows
  which brain is active.

## 3. Robot UX (owner: Robot UX workstream)
* **Robot picker:** in the top bar, as robot cards with kind, capability chips and an "experimental" badge.
  Picking one sends `world set`.
* **Scene picker:** template plus an object list with add/remove, sending `world` messages. The AI can do the
  same.
* **Per-robot UI:**
  * Example chips in the idle plan card, from `pps/world/prompts/<id>.json` (`{"examples": [...], "tools": [...]}`).
  * Dock tools filtered by capabilities: Show and teach only where `teach_by_showing`; a Handover tool for
    `bimanual`; a Drive tool for `drive_base`.
* **Prompts:** `pps/world/prompts/base.md` and one `.md` per robot covering its body, reach, what it can't do,
  how to phrase targets, and safety.
* **Chat:** the conversation feed shows the agent's steps (tool calls as compact cards, final reply as a
  robot bubble) and which model is thinking.

## Ports for parallel work
World 8901–8909 · Brain 8911–8919 · Robot UX 8921–8929. Never touch 8777/8443 (live demo).
