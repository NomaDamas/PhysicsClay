# Complex worlds: homes and industrial sites (issue #48)

PhysicsClay's default scene is one table. Robot learning needs variety: every home and every site is different. So
PhysicsClay generates **whole rooms** from a template and a seed: counters with drawers and doors, a sink, a fridge,
a conveyor, totes and racks, and real objects (spoons, towels, bolts, test tubes...) dropped in and settled by physics.
Drawers slide and doors swing on real joints, the robot opens them by their handles, takes things out and puts them
in, and every step is physically simulated (and can be recorded as a LeRobot dataset, `docs/datasets.md`).

## Using it

* **UI**: Robot & scene → Scene → *Homes* / *Industrial sites*. Each pick builds a fresh layout (a random seed);
  **Shuffle layout** builds another one of the same room. The objects list says which container each object is in.
* **Say it**: "a messy kitchen", "set up a warehouse", "a lab bench" (agent tools `load_world` / `build_scene` with a
  `seed`).
* **Message**: `{type: "world", op: "set", robot?, template: "kitchen_full", seed: 7}` (a fresh world),
  `{type: "world", op: "set", seed: 8}` (same room, new layout), `{type: "world", op: "pack", name: "warehouse",
  seed?}` (keeps the robot when it fits there, else a Panda), or a full spec `{op: "set", scene: {template, seed,
  objects?}}` (objects given with a fresh template are added on the work surface).

The same `(template, seed)` always gives the same world (the clutter's physics drop is deterministic); a different seed
changes sizes, heights, colours, which side the sink is on, what is in which drawer and how the clutter fell.

* **Datasets**: `scripts/generate.py --scenes kitchen_full,warehouse ...` gives every episode a new layout (a fresh world
  seed, stored in the episode metadata as `variation.world_seed`), with the seed task's objects added on the work
  surface: one intent, many different rooms (`docs/datasets.md`).

## Templates

| Group | Template | What's in it | Bodies |
|---|---|---|---|
| Home | `kitchen_full` | counter run (0.88–0.92 m) with a three-drawer stack (utensils in the top drawer), a door cabinet, a sink with a faucet in a two-door sink cabinet, a dishwasher (bottom-hinged door + rack drawer), a stove top over two pot drawers, a fridge with a hinged door and shelves, a microwave with a side-hinged door, two wall cabinets with doors, open shelving, clutter on the counter, in the drawers and in the sink | all but ALOHA |
| Home | `dining` | a set dining table (plates, cutlery, glasses, a vase, a fruit bowl), four chairs, a sideboard with drawers and doors | all |
| Home | `living_room` | coffee table (0.40–0.46 m) with a remote, books and mugs, a sofa, a TV unit with two drawers, a bookcase, toys and shoes on the floor for mobile robots | all |
| Home | `laundry` | front-loading washer and dryer (round hinged doors) under a counter, a laundry basket with towels, a floor basket, detergent on a wall shelf | all but ALOHA |
| Industrial | `assembly_line` | a running belt conveyor (the work surface) with a photo-eye that stops the belt while a part sits at the station, a finished-parts bin beside the arm, an assembly bench with a parts tray (bolts, nuts, screws) and a screwdriver | all but ALOHA |
| Industrial | `warehouse` | a pick table with a full tote (one layer: boxes, pouches, foam blocks, tape) and an empty tote, three racks with decks and stock, a pallet with two layers of cases | all but ALOHA |
| Industrial | `machine_tending` | a bench-top CNC with an L-shaped sliding door, a vise holding a part, raw and finished trays, a chip bin, a tool cabinet | all but ALOHA |
| Industrial | `electronics_bench` | an ESD bench with two PCBs, a 2×3 parts tray (screws, nuts, a bolt, a chip reel), a screwdriver, pliers, a multimeter, a soldering station, a monitor | all but ALOHA |
| Industrial | `lab` | a tube rack and a yellow source rack with tubes, a centrifuge with a hinged lid, a pipette, sample plates, a reagent shelf, a fume hood | all |
| Industrial | `packing` | an open carton, a tote of products, a scale, a tape dispenser, flat cartons, a driven roller conveyor bringing boxes | all but ALOHA |

The robot stands where it always does (`pps/world/profiles.py` mount): the station's work surface has its near edge at
x = 0.22 and is centred on y = 0, so arms are bolted at the station, humanoids stand at the counter and mobile bases
have the floor in front of it to drive on (where cabinets have closed fronts, bodies on the floor stand 12 cm further
back). The ALOHA rig sits on the surface, so it only goes where the surface is free.

## Fixtures (`pps/world/fixtures.py`)

A fixture is `{"name", "kind", "pos": [x, y, z], "yaw": deg, "size": [w, d, h], ...}` (local frame: the front faces
-x, y to the left seen from the front). Static parts are world geoms `fx_<fixture>_<part>`; moving parts are bodies
`fx_<fixture>_<part>`:

* the IK planner keeps moving parts as **mocap bodies**, synced from the plant when a task plans around them, so the
  planner's q stays the robot's joints only;
* the plant gives them their **joint** (`pps/physics.py fixture_joints`): drawers slide (damping 6 N·s/m, 0.8 N friction,
  travel 78 % of the depth), doors and lids turn on hinges (with limits; the dishwasher door has a spring), parts are
  light (plywood / sheet-metal density) and their contacts with the static world are excluded (the joint holds them);
* conveyor **slats** are a chain of slide joints driven by velocity servos: a slat leaving the downstream end wraps to
  the upstream end, so the surface is continuous and objects ride on it by friction; the **photo-eye** stops the belt
  while a part sits in front of it (the station). **Roller conveyors** are hinged rollers driven by velocity servos.
* **Handles** are knobs (a stem and a head the jaws close on) with a site `fx_<fixture>_<part>_handle`.

Kinds: `base_cabinet` (drawers and / or doors), `countertop` (with a sink cut-out), `sink`, `stove`, `upper_cabinet`,
`fridge`, `microwave`, `dishwasher`, `shelving`, `table`, `chair`, `sofa`, `tv_unit`, `washer`, `bin` (tote, basket,
carton, tray), `conveyor`, `roller_conveyor`, `rack`, `pallet`, `cnc`, `tube_rack`, `centrifuge`, `parts_tray`, `scale`,
`box` (a plain block: walls, machines) and `floor` (a coloured floor patch). Every builder also declares its
**containers** (a drawer's inside, the sink basin, a tote, each rack slot, each tray compartment, shelves) and extra
**surfaces**.

`state.world.fixtures = [{name, kind, label, open: 0..1, joint, type, fixture, locked?}]` (conveyors:
`{name, kind: "conveyor"|"rollers", running, speed, holding}`) and `state.world.containers = [{name, label, kind,
objects: [...], behind?, open?}]` (`behind`: the drawer or door you must open first). Each `scene_objects` entry has
`in` when it is in a container.

## Objects (`pps/world/objects.py`)

Compound primitives with real sizes, masses and friction: utensils (`spoon`, `fork`, `knife`, `spatula`, `ladle`,
`whisk`) lie with the handle toward the robot and collide as a raised handle a parallel gripper can pinch plus a
bowl / tines / blade; tableware and kitchen things (`glass`, `pot`, `cutting_board`, `sponge`, `towel` (a folded thin
box; no cloth simulation), `jar`, `banana`, `apple`, `detergent`), the living room (`book`, `remote`, `toy_car`, `ball`,
`shoe`, `cushion`), tools lying on a bench (`hammer`, `wrench_flat`, `screwdriver_flat`, `pliers`), industrial parts
(`bolt`, `nut`, `screw`, `bracket`, `gear`, `blank`, `part`, `pcb`, `chip_reel`), lab items (`tube`, `sample_plate`,
`pipette`, `beaker`, `flask_small`) and packing (`carton`, `pouch`, `tape`), plus the basic kinds (mug, block, box,
bowl, cup, can, bottle, plate, fruit, vase). Each kind has a grasp class (`box`, `round`, `long`, `flat`) the task layer
uses: long things are held across the handle at their grip point, boxes with the jaws square to a face that fits, and
in a busy container the jaw direction with room for the fingers comes first.

No scanned meshes are downloaded: the compound primitives are what the robot grasps and what the dataset cameras see.
(The one scanned mug that ships with the G1 scene is still used for `mug`.)

## Clutter (`pps/world/rooms.py`)

A generator calls `g.drop(region, items, slots?, yaw?)`: the items are dropped from above into a container (a drawer, a
tote, the sink, a rack slot) or onto a surface box, and **settled by a headless MuJoCo simulation** of just that
container's fixture (invisible walls keep a surface's clutter in its region while it falls). What comes to rest
(natural stacks, leans and occlusions) becomes the objects' exact 3D poses; an item that bounced out, is still moving
or got wedged is left out. Results are cached per `(template, seed)`.

## Working with fixtures

Task actions (`docs/PROTOCOL.md` "v15 · Worlds"):

* `open {fixture, amount 0..1 (default 0.8)}` / `close {fixture}`: the gripper comes down on the knob, closes, and
  moves it along the drawer's slide or the door's arc (planned on the real joint geometry, every point reach-checked)
  in the plant's impedance mode (compliance), then lets go and backs off. Honest failures: out of reach (with how far),
  the knob is right under another one (grasps come from above), the jaws couldn't close on it, they slipped off, or it's
  stuck (the hand falls behind the plan: "it's stuck at 5 % open").
* `pick_from {container, object?}`: take it out and hold it, lifted clear (the drawer / door in front of it must be
  open at least 50–60 %, else it says so); things in the same bin aren't kept away from (the fingers go in among them).
* `place_in {object?, container}`: a free spot inside (a rack picks a free slot), let go just above what is there,
  carried at a height over everything near the way; afterwards it checks the object is inside.
* `sequence {steps: [...], preview?}` (agent: `plan_steps`): one plan card for a multi-step request; the first step is
  checked for real, the later ones as they come; it stops at the first failure and says which step.
* `{type: "fixture", op: "run" | "stop"}` starts / stops a conveyor; `op: "lock" | "unlock", name` jams a drawer (a
  check proves the honest failure); `op: "set", name, open` puts a part at an opening (a setup probe).

Mobile bases move between stations with the existing `drive`. In kinematic mode (`--kinematic`) a grasped drawer or door
follows the hand and what is inside rides along.

## What works per robot (physics, headless runs and `scripts/checks/worlds.py`)

| Task | Panda | UR5e |
|---|---|---|
| kitchen: open the top drawer ≥ 60 % (to 78 %), take the spoon, put it in the sink, close it | seeds 1–3 | seeds 2–3 (seed 1: the spoon slides out of the jaws as it is lowered into the sink; reported as a slip) |
| warehouse: pick the box out of the tote, put it in the empty tote | seeds 1–4 | seeds 1–3 |
| assembly line: a part rides to the photo-eye, the arm picks it into the bin | seeds 1–3 | seeds 1–3 |
| lab: a tube from the source rack into a free slot of the tube rack | seeds 1–3 | seeds 1–3 |
| machine tending: open the CNC door, the part from the vise into the finished tray, close the door | seeds 1–3 | seed 1 |
| a jammed drawer / a knob under another one | "stuck at 5 % open" / refused with the reason | same code |

Other bodies build in every world they fit (`tests/test_worlds.py` checks one of each kind: arm, humanoid, two-arm rig,
mobile base, quadruped) and stand clear of the furniture; their grasps are as reliable as on the table
(`docs/V1.md` P0 table: every body picks, the H2 too since v1's final pass).

## Limitations

* Handles are grasped **from above**: a knob right under another one (the second drawer of a stack, a low door under a
  drawer) is refused with that reason; reaching in from the front needs side grasps on handles (not yet).
* Doors with a horizontal hinge (dishwasher door, centrifuge lid) turn the knob out of a top-down grasp's reach near the
  end of their swing; they open as far as the arm can follow and say how far that was.
* No cloth (towels are folded thin boxes), no liquids, no deformables; utensil handles are thicker than real ones so a
  parallel gripper can hold them.
* The conveyor's photo-eye stops the belt for the robot; picking a moving part (tracking) is not implemented.
* Fixed arms work at their station; parts of a room beyond reach (the fridge, the far racks) are for mobile bases and
  for the dataset cameras.

## Adding your own

* **A fixture kind**: write `def my_fixture(b, fx)` in `pps/world/fixtures.py` using `b.box / b.cyl` (static, local
  frame), `b.part(name, "slide" | "hinge", anchor, axis, range, geoms, handle, ...)` (moving; `knob(...)` makes a handle,
  `drawer_geoms(...)` a drawer) and `b.container / b.surface`, and register it in `BUILDERS`.
* **A room**: write `def my_room(g)` in `pps/world/rooms.py`: `g.fixture(kind, name, pos, size, yaw, **params)`,
  `g.obj(kind, centre, yaw, name)` for exact objects, `g.drop(region, items, slots)` for clutter, and set `g.surface`
  (the station: near edge x = 0.22, centred on y = 0; its `table` geom comes from a fixture with `main="table"`). Use
  `g.u / g.pick` (seeded) for every choice. Register it in `ROOMS` with a label, group, summary and the bodies that fit.
* **An object kind**: add a row to `_K` in `pps/world/objects.py` (size, colour, bounding shape, grasp class, mass,
  friction, size limits, grip) and, if it isn't a plain box or cylinder, its `visual` and `collision` primitives.
* Then run `.venv/bin/python -m pytest tests/test_worlds.py` (every template, three seeds, every body kind that fits).
