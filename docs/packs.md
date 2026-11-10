# Demo packs (v1, issue #35)

A demo pack is a ready-made scene for one story: a scene template, objects, sometimes a tool on the robot, standing
rules, what to say, a prompt addendum for the agent and a synthetic demonstration for people without a camera.

Load one from **Robot & scene → Scene → Demo packs**, by saying "set up a weld cell" (agent tool `load_pack`), or with
`{"type": "world", "op": "pack", "name": "weld_cell" | "gym" | "workshop"}`. The current body is kept when the pack
lists it (the Panda can weld too), otherwise the pack's default body is used. Picking a template, or `world reset`,
leaves the pack (its standing rules are removed with it). The pack card in the side panel shows what to say and has
**No camera? Play a demo** (`{type:"skill", op:"pack_demo"}`).

## Weld cell (`weld_cell`, UR5e or Panda)

* Two butt joints of 8 mm steel plates with 2 mm gaps: **the straight seam** (`seam_a`, 27 cm, plate_a | plate_b) and
  **the L seam** (`seam_l`, 11 + 11 cm, around plate_c). The seams are polylines in the scene spec, so the robot welds
  exactly on them.
* A MIG torch is mounted on the arm (`spec.tool = {kind: "torch", tcp: 0.16}`; `pps/world/scene.py mount_tool`): visual
  geoms (mount, gun neck, copper nozzle, wire), a collision capsule, and a `torch_tcp` site 16 cm along the flange's
  approach axis that becomes the end effector, so IK, trajectories, checks and the bead all work on the tip.
* Story: say "watch me", show the weave with your hand (any height: the robot learns the style, not the place), say
  "done". The robot extracts the weave (`weave_seam`), saves it and proposes it on the first seam not yet welded: "I'll
  weld the straight seam (27 cm): 5 mm zigzag at 2 Hz, 10 mm/s, 10° push angle. Right?". Correct it ("wider", "a bit
  slower", "more push"), say "go", then "now the L seam". The bead grows along the tip's actual path.
* **No arc or melt physics** (stated in the pack and to the agent).

## Gym (`gym`, G1 or Apollo)

* The gym floor template (a rubber mat; the lifter stands 0.8 m from the robot), a bench and a 20 kg barbell
  (`mass: 20`, a free body with P0 physics).
* "Coach me" → watch `coach`: reps counted aloud, one short cue per rep (depth, knees caving in, back angle, tempo).
* "Spot me" → the `spot_bench` skill (or your own spot skill): the robot follows the bar (the lifter's hands' midpoint)
  and pushes up when the bar slows below 5 cm/s for 0.5 s mid-rep, capped at 40 N; with P0 the cap is the impedance
  `force_limit`. In kinematic mode the barbell follows the lifter's hands while they hold it.
* The person is the webcam pose (or the synthetic lifter); there is no simulated contact with a human body. The
  synthetic spotting demo is a standing lift with a stall (the synthetic person can't lie on the bench).

## Workshop helper (`workshop`, G1 by default; any body that picks)

* A wrench, a screwdriver and a brush in stands on the table (graspable like a bottle).
* The pack adds the standing rule **"when I put my hand out, give me the wrench"** (watch `respond`,
  `hand_open_near_robot → handover wrench`): hold an open hand out over the table and the robot brings it to your hand.

## Adding a pack

Drop `pps/world/packs/<id>.json` (and optionally `<id>.md`, appended to the agent's prompt while it is loaded):

```json
{"id": "my_pack", "label": "My pack", "summary": "one line", "robot": "panda", "robots": ["panda", "ur5e"],
 "scene": {"template": "table", "tool": {"kind": "torch", "tcp": 0.16},
           "objects": [{"name": "plate", "kind": "steel_plate", "size": [0.1, 0.3, 0.008], "pos": [0.45, 0.0],
                        "movable": false, "exact": true}],
           "seams": {"seam_1": {"label": "the seam", "points": [[0.5, -0.1, 0.748], [0.5, 0.1, 0.748]], "between": ["plate"]}}},
 "examples": ["what to say", "..."],
 "setup": {"say": "a sentence when it loads", "rules": [{"text": "when I …, …"}], "watch": null},
 "demo": {"kind": "weave", "seconds": 8, "amp": 0.005, "freq": 2.0, "travel": 0.01},
 "physics": "what needs P0, said plainly"}
```

Scene additions used by packs (`pps/world/scene.py`): kinds `steel_plate` (thin, down to 4 mm), `bench`, `barbell`,
`wrench`, `screwdriver`, `brush`; colour `steel`; template `gym`; per object `exact` (keep the position and size as
given: workpieces and fixtures are not spread out or clamped) and `mass`; `pack`, `tool` and `seams` are kept in the
spec and in `state.world.scene`. `state.world.packs` lists the packs and `state.world.pack` names the loaded one.
