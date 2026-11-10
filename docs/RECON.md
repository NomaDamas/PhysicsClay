# Real photos → simulated objects and spaces

Bring real things into the simulation from photos, and turn a photo of your table into the scene. Built on what
PhysicsClay already has: the phone / browser camera, the vision models in `pps/spatial.py`, scenes built on the fly
(`pps/world/scene.py`), real physics (`pps/physics.py`) and the object library that datasets draw from. Issue #50.

- **An object.** 1–4 photos of one thing → recognized (label, material) → outline → 3D shape → real size → physics-ready
  (watertight, light visual mesh, convex collision pieces, mass from volume × density, friction from the material) →
  a free body in the scene that robots can actually grasp, drawn on the 3D stage with the photo's colours.
- **A space.** One photo of a table / desk / counter → the surface and the objects on it (where, how big) → a
  `world set` scene with a matching surface and the objects at the same relative positions. "This is my desk" becomes
  the sim.

## Using it

**In the app:** the **Real** tool in the dock (next to Point · Zone · Goal · Show · Path).

- *An object*: **Take or choose photos** (on a phone this opens the camera) or **Live camera** (the phone page or the
  browser camera that is streaming). Optional: what it is, and how big ("9 cm tall": this makes the size exact). Drag
  a box on the photo if it's cluttered. Pick how to reconstruct it, **Reconstruct**, check the outline and the turning
  3D preview, adjust the height, **Add to scene**. It's saved under *Your objects* and can be added again.
- *My table*: a photo taken from where the robot would stand (the table's near edge at the bottom of the photo).
  **Scan the table** shows what was found (boxes on the photo, a top view in front of the robot, sizes) → pick the
  surface, optionally **Use their real shapes**, then **Use this layout**.

**By talking to the robot** (the LLM agent, `pps/agent.py`): "put my mug in the sim" → `import_object_from_photo`
(the agent asks you to hold it up to the camera first, and for its size if you didn't say it); "this is my desk" →
`scan_space_from_photo`. Both use the live camera.

**Over HTTP / the websocket** (docs/PROTOCOL.md v12): `POST /recon/import`, `POST /recon/scan` with the photo(s) and,
optionally, a metric depth map (16-bit PNG in mm or `.npy` in m, as a depth camera or phone AR gives it) and the
intrinsics `K: [fx, fy, cx, cy]`; then `{type:"recon", op:"add" | "use_layout", job}`.

## How it works

```
photo(s) ─► recognize ─► depth ─► outline ─► 3D shape (provider) ─► metric scale ─► physics-ready ─► library ─► scene
            VLM key? Gemini /      Depth      box from you / VLM /   primitive | triposr |     you said > marker > depth  repair · ≤5k faces · CoACD
            the agent's model,     Anything   detector → GrabCut,    fal_trellis | meshy        > VLM > prior              mass = V·ρ · friction(material)
            else MediaPipe label   V2 Small   + depth "sticks out"
```

`pps/recon/`: `fit` (camera, support plane, primitive fits, meshes) · `depth` · `segment` · `scale` · `recognize`
(VLM / detector, material table) · `physical` · `providers/` · `pipeline` (one import) · `space` (scan my table) ·
`library` · `sim` (scene hooks) · `jobs` · `http`. Server side: `pps/features/recon.py`. UI: `web/ui/recon.js`.

**Provider interface** — every provider returns the same thing:

```python
provider.reconstruct(img, mask, hints, ...) -> Recon(mesh, colors, scale_m, confidence, provider, license, metric,
                                                     px_per_unit, notes)
pps.recon.reconstruct(images, mask=None, hints=None, provider="primitive", depth=None, K=None, keys=None)  # one call
```

The mesh is z up, x across the photo, y away from the camera, centred on its bounding box. `metric` says it's already in
metres; `px_per_unit` lets a reference in the photo (a marker) set the scale.

### Providers

| id | what | runs | speed (measured on this Mac, CPU, heavily loaded) | model license / terms | default |
|---|---|---|---|---|---|
| `primitive` | box / cylinder / turned profile (lathe) / mirrored hull fitted to depth + outline | offline, always | whole pipeline: ~5 s with metric depth, ~14 s with Depth Anything (first photo loads the model); a table scan 1–6 s | Apache-2.0 (PhysicsClay); depth: **Depth Anything V2 Small, Apache-2.0** | **yes** |
| `triposr` | one photo → full coloured mesh (TripoSR) | local, **opt-in** | load 10 s, then 22 s per object (resolution 192) | **MIT** (code and weights, `stabilityai/TripoSR`, 1.68 GB) | no |
| `fal_trellis` | one photo → textured mesh (Microsoft TRELLIS via fal.ai) | cloud, your fal.ai key | not measured (no key here; adapter tested with stubbed HTTP) | TRELLIS weights **MIT**; fal.ai terms apply | no |
| `meshy` | one photo → textured mesh (Meshy image-to-3d) | cloud, your Meshy key (the API needs a paid plan) | not measured (as above) | Meshy terms: free-plan outputs are owned by Meshy, **CC BY 4.0**; paid-plan outputs are yours; API outputs are deleted after 3 days | no |

Not offered on purpose: **Hunyuan3D 2.x** (its community license does not apply in the EU, the UK or South Korea, and
caps at 1 M MAU), **Stable Fast 3D / SPAR3D** (Stability AI Community License: free only under US$1 M annual revenue),
**Depth Anything V2 Base / Large / Giant** (CC BY-NC 4.0). Only the Small depth model is ever downloaded.

**Depth Anything V2 Small**: ONNX export `onnx-community/depth-anything-v2-small` (Apache-2.0), pinned revision and
SHA-256, ~99 MB, fetched once into `models/` the first time a photo needs it (needs `onnxruntime`;
`PC_RECON_DOWNLOAD=0` turns the download off). ~1–2 s per photo on a laptop CPU. Without it the pipeline uses the
outline alone. Relative depth bends the depth axis, so it chooses the *kind* of shape and the outline gives the
proportions (measured: aspect error 5 % from the outline vs 39 % straight from relative depth). Metric depth (a depth
camera, phone AR, a render) is used directly.

**TripoSR, opt-in**: `pip install 'physicsclay[recon]'`, put the TripoSR source at `vendor/TripoSR` (or point
`PC_TRIPOSR_DIR` at a clone; it is not on PyPI), install `torchmcubes` (or any module with the same
`marching_cubes(volume, threshold)`, e.g. a PyMCubes shim), and set `PC_TRIPOSR=1`, which allows the one-time 1.7 GB
weights download from Hugging Face (or `PC_TRIPOSR_WEIGHTS=/folder` with `config.yaml` + `model.ckpt`).
`PC_TRIPOSR_MPS=1` tries Apple's GPU (untested); `PC_TRIPOSR_RES` sets the marching-cubes resolution (192).

**Cloud keys** stay in the browser (the Real panel's *Keys* section stores them in this browser only) and ride one
request each (`X-Fal-Key`, `X-Meshy-Key`; recognition uses the Gemini key from Settings, `X-Gemini-Key`). They are
never stored, logged or put in state; error messages are scrubbed. Outside `--byok` the server may use `FAL_KEY` /
`MESHY_API_KEY` / `GEMINI_API_KEY` from `.env`; with `--byok` it never does.

### Metric scale

Sources in priority order; the first one present wins, and all are shown with their confidence:

| source | example | confidence |
|---|---|---|
| what you say | "it's 9 cm tall", "about 20cm long", "3 inches wide", "높이 9센티" | 95 % |
| a reference in the photo | an **ArUco marker** (DICT_4X4_50, id 0, 5 cm), or a credit card / A4 sheet when you say one is there | 85 % / 70 % |
| metric depth | a depth camera or phone AR depth | 80 % |
| the VLM's estimate | "this mug is about 9 cm tall" (Gemini / the agent's model) | 55 % |
| a typical size for the label | a cup is ~10 cm tall | 30 % |

**Print-at-home marker:** [`docs/recon/aruco_4x4_50_id0.svg`](recon/aruco_4x4_50_id0.svg) (print at 100 %; the
black square must measure 50 mm), or `/recon/marker.png` from the running app. Lay it flat next to the object, at the
same distance from the camera. On a table photo it also gives the table's plane exactly.

### Physics-ready

- **Watertight**: merge vertices, drop degenerate faces, fix normals, fill holes (trimesh). Still leaking → mass
  properties from its convex hull, and the summary says so.
- **Visual**: ≤ 5 000 faces (quadric decimation with `fast_simplification`, else vertex clustering), colours kept.
- **Collision**: one convex hull when the shape is nearly convex; else **CoACD** (MIT, `pip install coacd`) when
  installed; else convex hulls of k-means clusters of the surface. In the plant the object collides with these pieces;
  its bounding cylinder / box stays for the task layer (grasp planning, words).
- **Mass and inertia**: solid volume × density, or for hollow things (cups, bowls, boxes) wall area × thickness ×
  density; inertia from the mesh; centre of mass from the mesh. Your stated mass or the VLM's (5 g – 20 kg) wins.
- **Friction** by material: ceramic 0.6, glass 0.45, plastic 0.5, metal 0.4, wood 0.55, paper 0.5, cardboard 0.55,
  fabric 0.75, rubber 0.95, food 0.6, foam 0.8 (sliding; `pps/recon/recognize.py` MATERIALS).
- The object keeps a scene kind (cup, can, bottle, bowl, plate, fruit, vase, block, box), so "the cup" and the grasp
  heuristics work as for any object.

### Scan my table

Objects come from the VLM's boxes (with a key), else the MediaPipe detector plus what stands out from the surface:
with depth, what stands above the fitted table plane; without, what differs from the table's colour. The table plane,
best first: an ArUco marker on it · metric depth · relative depth (sizes then come from what the objects are) · an
assumed camera (35° down, 50 cm above the table; said so, low confidence). With depth each object is measured from its
3D points (footprint fit + height); otherwise from rays through its outline. The layout keeps its relative positions,
is centred on the robot's reachable band (squeezed, with a note, if it doesn't fit) and becomes primitives or, with
*Use their real shapes*, reconstructed meshes saved to the library.

### The object library

```
library/objects/<id>/meta.json          label, kind, material, provider + its model's license, scale source and
                                        confidence, size, mass, inertia, friction, collision method, notes
                     visual.obj         metres, z up, centred; colors.u8 = RGB per vertex (the 3D stage)
                     collision_<k>.obj  convex pieces
                     photo.jpg          the crop it came from
```

`library/` is user data (gitignored; `PC_LIBRARY` moves it). Use an entry in any scene spec —
`{"name": "my_mug", "kind": "cup", "mesh": "<id>", "scale": 1.0}` (a `size` without `scale` sets the scale) — and in
data generation: `scripts/generate.py --randomize objects,library` mixes your real objects into the distractors at
×0.9–1.1 of their measured size (`--library-only` uses only them).

## Accuracy (measured, this branch)

| test | input | result | truth | worst error |
|---|---|---|---|---|
| unit: box | synthetic metric depth, 320×240 | 10.0 × 5.6 × 8.0 cm | 10 × 6 × 8 | 6.5 % |
| unit: cylinder | synthetic metric depth | 7.0 × 7.0 × 12.0 cm | 7 × 7 × 12 | < 0.5 % |
| unit: ArUco scale | synthetic photo, 100 px marker | 0.505 mm/px | 0.500 | 1 % |
| server check: cup | MuJoCo render + depth (640×480) | cylinder 6.0 × 6.0 × 10.2 cm, 47 g, 1 hull | 6 × 6 × 10 | 2 % |
| server check: table scan | render + depth, 3 objects | box 11.6 × 8.8 × 8.0, cup 7 × 7 × 10.2, can 6.6 × 6.6 × 12 cm; 3/3 found | 12 × 9 × 8, 7 × 7 × 10, 6.6 × 6.6 × 12 | 3 %; positions within ~2 cm |
| scanned mug, no depth, "14 cm tall" | render of a real scan (Google Scanned Objects), Depth Anything + outline | lathe 9.8 × 9.8 × 14.0 cm | 9.1 across × 12.2 deep (handle) × 13.5 | +8 % across, −20 % deep (no handle) |
| same, TripoSR (CPU) | same photo | mesh with the handle, 8.7 × 11.9 × 13.8 cm, 8 CoACD pieces | 9.1 × 12.2 × 13.5 | 4 % |
| same, label only (size from "a mug is ~10 cm") | same photo | 7.0 × 7.0 × 10.0 cm, confidence 35 % | 9.1 × 12.2 × 13.5 | 43 % (why the size matters) |

Physics: an imported bottle rests on the table under gravity within 6 mm of its true height; a Panda picks the imported
cup and places it 22 cm away with physics (`scripts/checks/recon.py`).

## What works where

- **Offline, default install** (+ `trimesh`): the `primitive` provider from the outline, the metric scale from what
  you say / an ArUco marker / a card or A4 sheet / metric depth / label priors, physics prep with hull clusters, the
  library, scan my table from colours, scenes, the agent tools.
- **+ `onnxruntime`** (99 MB model fetched once): relative depth for outlines and table scans. **+ `coacd`**: better
  collision for concave things. Both are in the `[recon]` extra.
- **With a Gemini key** (or the agent's OpenAI-compatible model): recognition (label, material, size estimate, hollow,
  a box around the object) and the VLM's table reading.
- **With a fal.ai / Meshy key**: full textured meshes from the cloud.
- **Opt-in local**: TripoSR (above).

## Limitations

- One photo shows one side. The primitive provider assumes the hidden side mirrors the visible one; handles, spouts
  and holes are lost (the lathe of a mug has no handle). TripoSR and the cloud models guess the back.
- Several photos are not fused into one shape (no camera poses): the clearest view is kept and the others check its
  proportions (confidence goes up when they agree).
- Without a stated size, a marker or depth, the size is a guess from the label (30 %) and the panel says so.
- Relative depth is not metric; a table scan without depth or a marker positions objects well across the photo but
  less well along it.
- Hollow things are modelled as closed solids with hollow-wall mass (a cup can't hold anything in the sim yet).
- Detection without a VLM key is limited to the MediaPipe detector's COCO labels plus unlabeled blobs named by colour
  and shape.
- TripoSR on Apple's GPU (MPS) is untested; it was verified on CPU only, with PyMCubes standing in for torchmcubes
  (which has no wheels). The cloud adapters follow the providers' documented APIs but were only run against stubs.
