"""Physics-ready: a closed mesh, a light visual mesh, convex pieces for collision, mass and inertia, friction.

* watertight: merge vertices, drop degenerate faces, fix normals, fill holes; if it still leaks, mass properties come
  from its convex hull (the visual keeps the original surface);
* visual: <= 5k faces (quadric decimation when fast_simplification is installed, else vertex clustering);
* collision: one convex hull when the shape is nearly convex; else CoACD (MIT, `pip install coacd`) when installed;
  else convex hulls of k-means clusters of the surface (a cheap approximate decomposition);
* mass = solid volume x density (or, for hollow things like cups and boxes, wall area x thickness x density),
  inertia from the mesh (scaled from the solid's for a shell), friction from the material table."""
import importlib.util

import numpy as np

from .recognize import material_props

MAX_FACES = 5000
MAX_HULLS = 8
SHELL_INERTIA = 1.6          # a thin shell's moments are ~1.6x the solid's at the same mass (sphere 5/3, cylinder axis 2)


def _has(mod):
    try:
        return importlib.util.find_spec(mod) is not None
    except (ImportError, ValueError):
        return False


def nearest(src, query):
    """Index of the nearest src point for each query point (scipy's KD-tree when installed, else chunked numpy)."""
    src, query = np.asarray(src, float), np.asarray(query, float)
    if _has("scipy"):
        from scipy.spatial import cKDTree
        return cKDTree(src).query(query)[1]
    out = np.empty(len(query), int)
    step = max(1, 2_000_000 // max(len(src), 1))
    for i in range(0, len(query), step):
        q = query[i:i + step]
        out[i:i + step] = np.argmin(((q[:, None, :] - src[None]) ** 2).sum(-1), axis=1)
    return out


def repair(mesh):
    """-> (mesh, watertight, notes). Never raises on a bad mesh: the worst case is a convex hull for physics."""
    import trimesh
    notes = []
    m = mesh.copy()
    m.merge_vertices()
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    if not m.is_watertight:
        try:
            trimesh.repair.fill_holes(m)
            notes.append("filled holes")
        except Exception as e:                    # (e.g. networkx missing): the convex hull fallback still works
            notes.append(f"couldn't fill holes ({type(e).__name__})")
    trimesh.repair.fix_normals(m)
    if m.is_watertight and m.volume < 0:
        m.invert()
    return m, bool(m.is_watertight), notes


def decimate(mesh, max_faces=MAX_FACES, colors=None):
    """-> (mesh, colors) with at most ~max_faces faces. Colours follow the vertices they came from."""
    import trimesh
    if len(mesh.faces) <= max_faces:
        return mesh, colors
    if _has("fast_simplification"):
        try:
            out = mesh.simplify_quadric_decimation(face_count=max_faces)
            if colors is not None:                        # nearest original vertex's colour
                colors = colors[nearest(mesh.vertices, out.vertices)]
            return out, colors
        except Exception:
            pass
    v, f = np.asarray(mesh.vertices), np.asarray(mesh.faces)
    ext = float(max(mesh.extents)) or 1.0
    cell = ext / 40
    for _ in range(12):                                   # vertex clustering: snap to a grid, merge, drop collapsed faces
        key = np.floor((v - v.min(0)) / cell).astype(np.int64)
        _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
        inv = inv.ravel()
        nv = np.zeros((cnt.size, 3))
        np.add.at(nv, inv, v)
        nv /= cnt[:, None]
        f2 = inv[f]
        faces = f2[(f2[:, 0] != f2[:, 1]) & (f2[:, 1] != f2[:, 2]) & (f2[:, 0] != f2[:, 2])]
        if len(faces) <= max_faces:
            out = trimesh.Trimesh(nv, faces, process=True)
            out.merge_vertices()
            if colors is not None:
                c = np.zeros((cnt.size, colors.shape[1]))
                np.add.at(c, inv, colors.astype(float))
                c = (c / cnt[:, None]).astype(np.uint8)
                colors = c[nearest(nv, out.vertices)]
            return out, colors
        cell *= 1.3
    return mesh.convex_hull, None


def convexity(mesh):
    try:
        hv = float(mesh.convex_hull.volume)
        return float(mesh.volume) / hv if hv > 0 and mesh.is_watertight else 0.0
    except Exception:
        return 0.0


def decompose(mesh, max_hulls=MAX_HULLS, method=None):
    """Convex pieces for collision -> (list of trimesh hulls, method used)."""
    import trimesh
    if convexity(mesh) > 0.9 or len(mesh.vertices) < 16:
        return [mesh.convex_hull], "hull"
    if (method in (None, "coacd")) and _has("coacd") and mesh.is_watertight:
        try:
            import coacd
            coacd.set_log_level("error")
            src = mesh
            if len(src.faces) > 3000:
                src, _ = decimate(src, 3000)
            parts = coacd.run_coacd(coacd.Mesh(np.asarray(src.vertices), np.asarray(src.faces)), threshold=0.08,
                                    max_convex_hull=max_hulls, preprocess_mode="auto", resolution=40, mcts_iterations=60,
                                    mcts_max_depth=2, seed=1)
            hulls = [trimesh.Trimesh(v, f).convex_hull for v, f in parts]
            hulls = [h for h in hulls if h.volume > 1e-12]
            if hulls:
                return hulls[:max_hulls], "coacd"
        except Exception:
            pass
    return cluster_hulls(mesh, max_hulls), "clusters"


def cluster_hulls(mesh, k=6, iters=12):
    """Approximate decomposition: k-means on surface samples, a convex hull per cluster."""
    import trimesh
    pts = np.asarray(mesh.sample(3000, return_index=False)) if len(mesh.faces) else np.asarray(mesh.vertices)
    pts = np.vstack([pts, mesh.vertices])
    rng = np.random.default_rng(0)
    c = pts[rng.choice(len(pts), min(k, len(pts)), replace=False)]
    for _ in range(iters):
        lab = np.argmin(((pts[:, None] - c[None]) ** 2).sum(-1), axis=1)
        c = np.array([pts[lab == j].mean(0) if np.any(lab == j) else c[j] for j in range(len(c))])
    hulls = []
    for j in range(len(c)):
        P = pts[lab == j]
        if len(P) >= 4:
            try:
                h = trimesh.Trimesh(P).convex_hull
                if h.volume > 1e-12:
                    hulls.append(h)
            except Exception:
                pass
    return hulls or [mesh.convex_hull]


def mass_props(mesh, material="unknown", hollow=False, mass=None):
    """-> {mass, volume, density, friction, com (3,), inertia (3, 3) about the com, hollow, source}."""
    density, mu, wall = material_props(material)
    solid = mesh if mesh.is_watertight and mesh.volume > 0 else mesh.convex_hull
    vol = float(solid.volume)
    if hollow:
        m_est = float(mesh.area) * wall * density
        m_est = min(m_est, vol * density)                  # never heavier than solid
    else:
        m_est = vol * density
    src = "volume x density" + (" (hollow)" if hollow else "")
    if mass and mass > 0:
        m_est, src = float(mass), "given"
    m_est = float(np.clip(m_est, 0.005, 25.0))
    I_unit = np.asarray(solid.moment_inertia, float) / max(float(solid.mass), 1e-12)   # per kg, about the com
    inertia = I_unit * m_est * (SHELL_INERTIA if hollow else 1.0)
    return {"mass": m_est, "volume": vol, "density": density, "friction": mu, "com": np.asarray(solid.center_mass, float),
            "inertia": inertia, "hollow": bool(hollow), "source": src, "material": material}


def principal(inertia):
    """3x3 inertia -> (diagonal moments (3,), quaternion wxyz of the principal frame)."""
    import mujoco
    w, V = np.linalg.eigh(np.asarray(inertia, float))
    if np.linalg.det(V) < 0:
        V[:, 2] = -V[:, 2]
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, V.flatten())
    return np.clip(w, 1e-9, None), q


def prepare(mesh, colors=None, material="unknown", hollow=False, mass=None, max_faces=MAX_FACES):
    """Full physics prep -> dict(visual, colors, hulls, props, notes, watertight, method)."""
    m, tight, notes = repair(mesh)
    if colors is not None and len(colors) != len(m.vertices):
        colors = np.asarray(colors)[nearest(mesh.vertices, m.vertices)]
    vis, colors = decimate(m, max_faces, colors)
    if not tight:
        notes.append("not watertight: mass from the convex hull")
    hulls, method = decompose(m)
    props = mass_props(m, material, hollow, mass)
    return {"visual": vis, "colors": colors, "hulls": hulls, "props": props, "notes": notes, "watertight": tight, "method": method}
