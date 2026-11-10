"""Imported objects inside scenes: the hooks pps/world/scene.py calls for spec objects that carry a "mesh".

    {"name": "my_mug", "kind": "cup", "mesh": "<library id>", "scale": 1.0, "pos": [x, y], "movable": true}

* the object keeps a scene kind (cup, box, bottle...), so words ("the cup"), grasp heuristics and the task layer work
  as for any object; its `<body>_collision` geom is the bounding cylinder / box the task layer reads its size from;
* the visual is the reconstructed mesh (group 2), coloured on the 3D stage from library/objects/<id>/colors.u8;
* collision in the plant (physics) is the convex pieces `<body>_hull<k>` (the bounding geom stops colliding there),
  with the library entry's mass, inertia (from the mesh) and friction (material). In the IK planner's model and in
  kinematic mode the object is the bounding primitive, like every other object.
This module stays light (no trimesh): scenes are built often."""
import numpy as np

from . import library

MESHES = {}          # MuJoCo mesh asset name -> library id (the 3D stage's vertex colours; rebuilt with every scene)
LIMITS = (0.02, 0.40)
KIND_OK = ("cup", "can", "bottle", "bowl", "plate", "fruit", "vase", "block", "box")   # generic grasp + physics paths


def _f(*v):
    return " ".join(f"{float(x):.6g}" for x in v)


def _nearest_color(rgba):
    from ..world.scene import COLORS
    c = np.asarray(rgba[:3], float)
    return min(COLORS, key=lambda k: float(np.sum((np.asarray(COLORS[k]) - c) ** 2)))


def prepare(raw):
    """Spec object with "mesh" -> a copy with kind, size (= the mesh's bounding box x scale), mass and colour filled in
    from the library, or None when the entry is missing. A given size without a scale sets the scale (generators)."""
    meta = library.load(raw.get("mesh")) if library.valid_id(raw.get("mesh")) else None
    if meta is None:
        return None
    o = dict(raw)
    size = np.asarray(meta["size"], float)
    s = raw.get("scale")
    if not (isinstance(s, (int, float)) and s > 0):
        given = raw.get("size")
        if isinstance(given, (list, tuple)) and len(given) == 3 and all(isinstance(v, (int, float)) and v > 0 for v in given):
            s = float(np.mean(np.asarray(given, float) / np.maximum(size, 1e-6)))
        else:
            s = 1.0
    s = float(np.clip(s, LIMITS[0] / max(size.max(), 1e-6), LIMITS[1] / max(size.max(), 1e-6)))
    from ..world.scene import KINDS, ROUND, _kind
    kind = _kind({"kind": raw.get("kind") or meta.get("kind") or "box", "name": ""})
    kind = "cup" if kind == "mug" else kind              # the scene's mug kind is the scanned mug's own geometry
    if kind not in KIND_OK:
        kind = "box"
    sz = size * s
    if KINDS[kind][2] in ROUND:                          # the bounding cylinder holds the whole footprint
        sz[0] = sz[1] = max(sz[0], sz[1])
    o["scale"] = round(s, 4)
    o["size"] = [round(float(v), 4) for v in sz]
    o["kind"] = kind
    if not raw.get("color"):
        o["color"] = _nearest_color(meta.get("rgba") or [0.6, 0.6, 0.6])
    if not (isinstance(raw.get("mass"), (int, float)) and raw["mass"] > 0):
        o["mass"] = round(float(meta.get("mass", 0.2)) * s ** 3, 4)
    if not raw.get("name"):
        o["name"] = meta.get("name") or meta.get("label") or "thing"
    return o


def keep(o):
    """The keys a normalised mesh object carries besides the usual ones."""
    return {"mesh": o["mesh"], "scale": o["scale"]}


def _mesh_names(o, meta):
    n = o["name"]
    return f"rc_{n}_vis", [f"rc_{n}_c{k}" for k in range(len(meta.get("collision") or []))]


def assets_xml(objects):
    """<mesh> assets for every mesh object (each at its own scale), registering them for the stage's colours."""
    out = []
    for o in objects:
        if not o.get("mesh"):
            continue
        meta = library.load(o["mesh"])
        if meta is None:
            continue
        d, s = meta["dir"], o.get("scale", 1.0)
        vis, cols = _mesh_names(o, meta)
        MESHES[vis] = o["mesh"]
        out.append(f'<mesh name="{vis}" file="{d}/{meta["visual"]}" scale="{_f(s, s, s)}" inertia="shell"/>')
        for name, f in zip(cols, meta.get("collision") or []):
            out.append(f'<mesh name="{name}" file="{d}/{f}" scale="{_f(s, s, s)}"/>')
    return "".join(out)


def object_xml(o, top):
    """The body of a mesh object: visual mesh, bounding collision primitive (task layer), convex pieces (plant)."""
    from ..world.scene import KINDS, ROUND
    meta = library.load(o["mesh"])
    body = ("obj_" if o["movable"] else "obs_") + o["name"]
    s = float(o.get("scale", 1.0))
    ex = np.asarray(meta["size"], float) * s
    hz = ex[2] / 2
    x, y = o["pos"]
    r, g, b = (meta.get("rgba") or [0.6, 0.6, 0.6])[:3]
    vis, hulls = _mesh_names(o, meta)
    mocap = ' mocap="true"' if o["movable"] else ""
    sx, sy, sz = o["size"]
    zc = -hz + sz / 2                                  # the bounding primitive stands on the same table as the mesh
    col = f'name="{body}_collision" rgba="0 0 0 0" contype="1" conaffinity="1" group="3" pos="{_f(0, 0, zc)}"'
    if KINDS.get(o["kind"], KINDS["box"])[2] in ROUND:
        prim = f'<geom {col} type="cylinder" size="{_f(max(sx, sy) / 2, sz / 2)}"/>'
    else:
        prim = f'<geom {col} type="box" size="{_f(sx / 2, sy / 2, sz / 2)}"/>'
    geoms = f'<geom type="mesh" mesh="{vis}" rgba="{_f(r, g, b, 1)}" contype="0" conaffinity="0" group="2"/>' + prim
    # compiled to collide (MuJoCo only lets geoms collide that were compiled that way); the plant turns them up to the
    # objects' collision bits and turns the bounding primitive off (fix_plant)
    geoms += "".join(f'<geom name="{body}_hull{k}" type="mesh" mesh="{h}" rgba="0 0 0 0" contype="1" conaffinity="1" group="3"/>'
                     for k, h in enumerate(hulls))
    return f'<body name="{body}" pos="{_f(x, y, top + hz + 0.001)}"{mocap}>{geoms}</body>'


def fix_plant(m, spec):
    """Plant model (physics): movable mesh objects collide with their convex pieces and get the mesh's mass, centre of
    mass, inertia and the material's friction. Runs on the compiled model, before the Plant snapshots it."""
    import mujoco

    from .physical import principal
    changed = False
    for o in spec.get("objects") or []:
        if not o.get("mesh") or not o.get("movable"):
            continue
        body = "obj_" + o["name"]
        try:
            b = m.body(body).id
            col = m.geom(f"{body}_collision").id
        except KeyError:
            continue
        meta = library.load(o["mesh"])
        hulls = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(f"{body}_hull")]
        if meta is None or not hulls or m.body_jntnum[b] != 1:
            continue
        s = float(o.get("scale", 1.0))
        mu = float(meta.get("friction", 0.6))
        m.geom_contype[col] = m.geom_conaffinity[col] = 0
        for g in hulls:
            m.geom_contype[g] = m.geom_conaffinity[g] = 3
            m.geom_condim[g] = 4
            m.geom_friction[g] = [mu, 0.01, 0.0001]
        mass = float(o.get("mass") or meta["mass"] * s ** 3)
        inertia = np.asarray(meta["inertia"], float) * (mass / max(float(meta["mass"]), 1e-9)) * s ** 2
        w, q = principal(inertia)
        m.body_mass[b] = mass
        m.body_ipos[b] = np.asarray(meta["com"], float) * s
        m.body_iquat[b] = q
        m.body_inertia[b] = w
        m.body_sameframe[b] = 0                        # the inertial frame moved off the body frame: no shortcuts
        m.body_simple[b] = 0
        da, dn = m.body_dofadr[b], m.body_dofnum[b]
        m.dof_simplenum[da:da + dn] = 0
        changed = True
    if changed:
        mujoco.mj_setConst(m, mujoco.MjData(m))
    return changed


def annotate_payload(m, payload):
    """/scene.json: point the 3D stage at the vertex colours of imported meshes (only when the vertex count matches)."""
    import mujoco
    for mid, ent in (payload.get("meshes") or {}).items():
        try:
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, int(mid)) or ""
        except (ValueError, IndexError):
            continue
        lib = MESHES.get(name)
        if not lib:
            continue
        c = library.colors(lib)
        if c and len(c) == 3 * int(m.mesh_vertnum[int(mid)]):
            ent["colors"] = f"/recon/colors/{lib}.bin"
    return payload
