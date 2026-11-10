"""Geometry of one object in one photo: camera model, support plane, primitive fits and the meshes they become.

Two inputs are possible:
* a depth map (metric from a depth camera / phone AR / a synthetic render, or relative from Depth Anything V2 Small,
  turned into pseudo depth with `depth_from_disparity`): pixels -> 3D points -> support plane (RANSAC) -> the object
  in a z-up frame standing on that plane -> the best of box / cylinder / lathe (a turned profile) / mirrored hull;
* only the mask: the silhouette becomes a lathe (round things) or a box, in pixel units.
Meshes come out watertight, z up and centred on their bounding box; `sample_colors` paints them from the photo."""
import math

import cv2
import numpy as np

HFOV = 63.0          # degrees: a typical phone main camera / laptop webcam when the photo carries no intrinsics
ROUND_KINDS = ("cup", "can", "bottle", "bowl", "plate", "fruit", "vase", "mug")
BOX_KINDS = ("box", "block")


# ------------------------------------------------------------------ camera
def intrinsics(w, h, K=None, hfov=HFOV):
    """3x3 K: given ([[fx,0,cx],[0,fy,cy],[0,0,1]] or [fx, fy, cx, cy]) or guessed from a horizontal field of view."""
    if K is not None:
        K = np.asarray(K, float)
        if K.shape == (3, 3):
            return K
        if K.size == 4:
            fx, fy, cx, cy = K.ravel()
            return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    f = (w / 2) / math.tan(math.radians(hfov) / 2)
    return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])


def backproject(depth, K, mask=None, stride=1):
    """depth (H, W) metres or pseudo units -> (N, 3) camera-frame points (x right, y down, z forward), (N, 2) pixels."""
    h, w = depth.shape
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    Z = depth[::stride, ::stride]
    ok = np.isfinite(Z) & (Z > 0)
    if mask is not None:
        ok &= mask[::stride, ::stride]
    u, v, z = us[ok].astype(float), vs[ok].astype(float), Z[ok].astype(float)
    X = (u - K[0, 2]) * z / K[0, 0]
    Y = (v - K[1, 2]) * z / K[1, 1]
    return np.column_stack([X, Y, z]), np.column_stack([u, v])


def project(P, K):
    """(N, 3) camera-frame points -> (N, 2) pixels."""
    z = np.clip(P[:, 2], 1e-6, None)
    return np.column_stack([K[0, 0] * P[:, 0] / z + K[0, 2], K[1, 1] * P[:, 1] / z + K[1, 2]])


def depth_from_disparity(disp, ref_mask=None):
    """Relative inverse depth (bigger = nearer, e.g. Depth Anything) -> pseudo depth with the reference region at ~1.
    Assumes the model's shift is ~0 (Z ∝ 1/disparity); the metric scale comes later (what the person says, a marker,
    the VLM). Lateral sizes are reliable, the depth axis less so: callers lower their confidence."""
    d = np.asarray(disp, float)
    sel = d[ref_mask] if ref_mask is not None and np.any(ref_mask) else d.ravel()
    ref = float(np.median(sel[sel > 0])) if np.any(sel > 0) else 1.0
    return 1.0 / np.clip(d / max(ref, 1e-9), 0.05, None)


# ------------------------------------------------------------------ support plane
def fit_plane(P, thresh, iters=250, seed=0):
    """RANSAC plane -> (n, d, inliers) with n·p + d = 0 and the camera (origin) on the positive side (d > 0)."""
    P = np.asarray(P, float)
    if len(P) < 3:
        return None
    rng = np.random.default_rng(seed)
    best, best_n = None, -1
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        L = np.linalg.norm(n)
        if L < 1e-12:
            continue
        n = n / L
        d = -float(n @ a)
        cnt = int(np.count_nonzero(np.abs(P @ n + d) < thresh))
        if cnt > best_n:
            best, best_n = (n, d), cnt
    if best is None:
        return None
    n, d = best
    inl = np.abs(P @ n + d) < thresh
    Q = P[inl]
    if len(Q) >= 3:                                    # refine on the inliers
        c = Q.mean(0)
        _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
        n = vt[2]
        d = -float(n @ c)
        inl = np.abs(P @ n + d) < thresh
    if d < 0:
        n, d = -n, -d
    return n, d, inl


def object_frame(n, d, P_obj):
    """z = plane normal (towards the camera side), x = the camera's right projected on the plane, y = away from the
    camera. -> (R, c0): q = R @ (p - c0) in the object frame, origin under the object's centroid."""
    z = np.asarray(n, float)
    x = np.array([1.0, 0.0, 0.0]) - z[0] * z
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0.0, 0.0, 1.0]) - z[2] * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    c = np.asarray(P_obj, float).mean(0)
    c0 = c - (z @ c + d) * z
    return np.stack([x, y, z]), c0


# ------------------------------------------------------------------ primitive fits (object frame, z up from the plane)
def _top(Q):
    return float(np.percentile(Q[:, 2], 98)) if len(Q) else 0.0


def fit_box(Q):
    """-> {dims (w, l, h), center (x, y), yaw, residual}: the footprint as the min-area rectangle of the projected points."""
    H = _top(Q)
    (cx, cy), (w, ln), ang = cv2.minAreaRect(Q[:, :2].astype(np.float32))
    yaw = math.radians(ang)
    c, s = math.cos(yaw), math.sin(yaw)
    p = Q[:, :2] - [cx, cy]
    loc = np.column_stack([c * p[:, 0] + s * p[:, 1], -s * p[:, 0] + c * p[:, 1], Q[:, 2] - H / 2])
    half = np.array([w / 2, ln / 2, H / 2])
    a = np.abs(loc) - half
    outside = np.linalg.norm(np.clip(a, 0, None), axis=1)
    inside = np.where(np.all(a <= 0, axis=1), -np.max(a, axis=1), 0.0)
    res = float(np.mean(outside + inside)) if len(Q) else 1.0
    return {"dims": [float(w), float(ln), H], "center": [float(cx), float(cy)], "yaw": yaw, "residual": res}


def circle_fit(P2):
    """Least-squares circle (Kasa, then a few Gauss-Newton steps on the geometric error) -> (centre (2,), radius)."""
    P2 = np.asarray(P2, float)
    A = np.column_stack([2 * P2, np.ones(len(P2))])
    b = (P2 ** 2).sum(1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = sol[:2]
    r = math.sqrt(max(sol[2] + c @ c, 1e-12))
    for _ in range(10):
        dv = P2 - c
        dist = np.linalg.norm(dv, axis=1) + 1e-12
        J = np.column_stack([-dv / dist[:, None], -np.ones(len(P2))])
        res = dist - r
        step, *_ = np.linalg.lstsq(J, -res, rcond=None)
        c, r = c + step[:2], r + step[2]
        if np.abs(step).max() < 1e-9:
            break
    return c, abs(float(r))


def fit_cylinder(Q):
    """-> {r, h, center, residual, side}: circle through the wall points (or the rim of the visible top)."""
    H = _top(Q)
    side = Q[(Q[:, 2] > 0.12 * H) & (Q[:, 2] < 0.85 * H)]
    top = Q[Q[:, 2] > 0.9 * H]
    if len(side) >= 30:
        c, r = circle_fit(side[:, :2])
    elif len(top) >= 10:
        hull = cv2.convexHull(top[:, :2].astype(np.float32)).reshape(-1, 2)
        c, r = circle_fit(hull) if len(hull) >= 3 else (top[:, :2].mean(0), float(np.ptp(top[:, 0]) / 2))
    else:
        return {"r": 0.0, "h": H, "center": [0.0, 0.0], "residual": 1.0, "side": side}
    rad = np.linalg.norm(Q[:, :2] - c, axis=1)
    on_side = Q[:, 2] < 0.9 * H
    err = np.where(on_side, np.abs(rad - r), np.abs(Q[:, 2] - H) + np.clip(rad - r, 0, None))
    return {"r": r, "h": H, "center": [float(c[0]), float(c[1])], "residual": float(np.mean(err)), "side": side}


def radius_profile(Q, center, H, bins=12):
    """Radius of the visible wall per height band around a fixed axis -> (heights, radii); bands without points skipped."""
    rad = np.linalg.norm(Q[:, :2] - np.asarray(center), axis=1)
    hs, rs = [], []
    for k in range(bins):
        lo, hi = H * k / bins, H * (k + 1) / bins
        sel = (Q[:, 2] >= lo) & (Q[:, 2] < hi) & (Q[:, 2] < 0.92 * H)
        if np.count_nonzero(sel) >= 8:
            hs.append((lo + hi) / 2)
            rs.append(float(np.percentile(rad[sel], 70)))
    return np.array(hs), np.array(rs)


def choose_shape(Q, kind=None):
    """Best primitive for the object points -> (shape, params, confidence)."""
    box, cyl = fit_box(Q), fit_cylinder(Q)
    size = max(max(box["dims"]), 1e-6)
    rb, rc = box["residual"] / size, cyl["residual"] / max(2 * cyl["r"], cyl["h"], 1e-6)
    if kind in ROUND_KINDS:
        rc *= 0.6
    elif kind in BOX_KINDS:
        rb *= 0.6
    if cyl["r"] <= 0 or not np.isfinite(rc):
        rc = 9.0
    if min(rb, rc) > 0.09:
        return "hull", {"box": box, "cyl": cyl}, max(0.25, 0.6 - min(rb, rc))
    if rb <= rc:
        return "box", box, float(np.clip(1.0 - rb * 8, 0.3, 0.95))
    hs, rs = radius_profile(Q, cyl["center"], cyl["h"])
    conf = float(np.clip(1.0 - rc * 8, 0.3, 0.95))
    if len(rs) >= 4 and (rs.max() - rs.min()) / max(rs.max(), 1e-9) > 0.15:
        return "lathe", dict(cyl, heights=hs, radii=rs), conf
    return "cylinder", cyl, conf


# ------------------------------------------------------------------ meshes (z up, centred on the bounding box)
def _tm():
    import trimesh
    return trimesh


def centre(mesh):
    lo, hi = mesh.bounds
    mesh.apply_translation(-(lo + hi) / 2)
    return mesh


def box_mesh(dims):
    return centre(_tm().creation.box(extents=[max(float(v), 1e-4) for v in dims]))


def cylinder_mesh(r, h, sections=40):
    return centre(_tm().creation.cylinder(radius=max(float(r), 1e-4), height=max(float(h), 1e-4), sections=sections))


def lathe_mesh(heights, radii, H, sections=40):
    """Turned profile: radius per height (base at 0, top at H), closed at both ends."""
    hs = np.concatenate([[0.0], np.asarray(heights, float), [H]])
    rs = np.asarray(radii, float)
    rs = np.concatenate([[rs[0]], rs, [rs[-1]]])
    if len(rs) >= 5:                                   # light smoothing: depth noise, not the shape
        rs = np.convolve(np.pad(rs, 1, mode="edge"), [0.25, 0.5, 0.25], mode="valid")
    rs = np.clip(rs, max(rs.max() * 0.08, 1e-4), None)
    prof = [[0.0, 0.0]] + [[float(r), float(h)] for r, h in zip(rs, hs)] + [[0.0, float(H)]]
    m = _tm().creation.revolve(np.array(prof), sections=sections)
    m.merge_vertices()
    if m.volume < 0:
        m.invert()
    return centre(m)


def hull_mesh(points):
    import trimesh
    return centre(trimesh.Trimesh(vertices=np.asarray(points, float)).convex_hull)


def densify(mesh, target_faces=3000):
    """Split long edges so per-vertex colours have something to hold on to (primitives have 8-100 vertices)."""
    import trimesh
    ext = float(max(mesh.extents))
    if len(mesh.faces) >= target_faces * 0.5 or ext <= 0:
        return mesh
    edge = ext / 4
    v, f = mesh.vertices, mesh.faces
    for _ in range(8):
        v2, f2 = trimesh.remesh.subdivide_to_size(v, f, max_edge=edge, max_iter=12)
        if len(f2) > target_faces * 1.6:
            break
        v, f = v2, f2
        edge *= 0.7
    out = trimesh.Trimesh(v, f, process=True)
    out.merge_vertices()
    return out


# ------------------------------------------------------------------ colours
def mask_color(img, mask):
    px = img[mask] if mask is not None and np.any(mask) else img.reshape(-1, 3)
    b, g, r = np.median(px, axis=0)
    return np.array([r, g, b], float) / 255.0


def sample_colors(uv, img, mask, fallback_rgb):
    """Per-vertex RGBA (uint8) from image pixels uv (N, 2); vertices that land off the object get the object's median."""
    h, w = img.shape[:2]
    u = np.clip(np.round(uv[:, 0]).astype(int), 0, w - 1)
    v = np.clip(np.round(uv[:, 1]).astype(int), 0, h - 1)
    bgr = img[v, u].astype(float)
    ok = np.isfinite(uv).all(1) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    if mask is not None:
        ok &= mask[v, u]
    fb = np.asarray(fallback_rgb, float)[::-1] * 255
    bgr[~ok] = fb
    out = np.full((len(uv), 4), 255, np.uint8)
    out[:, :3] = np.clip(bgr[:, ::-1], 0, 255).astype(np.uint8)
    return out


# ------------------------------------------------------------------ silhouette only (no depth)
def silhouette_shape(mask, kind=None):
    """Mask only -> (mesh in pixel units, shape, uv(vertices) function, confidence). Round things (by kind, or a
    left-right symmetric outline) become a lathe of the row widths, others a box as deep as 0.6 x its width."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 50:
        return None
    v0, v1 = int(ys.min()), int(ys.max())
    u0, u1 = int(xs.min()), int(xs.max())
    rows = np.arange(v0, v1 + 1)
    widths, mids = [], []
    for v in rows:
        c = np.nonzero(mask[v])[0]
        if len(c):
            widths.append(c.max() - c.min() + 1)
            mids.append((c.max() + c.min()) / 2)
        else:
            widths.append(0)
            mids.append((u0 + u1) / 2)
    widths, mids = np.array(widths, float), np.array(mids, float)
    sub = mask[v0:v1 + 1, u0:u1 + 1]
    sym = float(np.logical_and(sub, sub[:, ::-1]).sum() / max(np.logical_or(sub, sub[:, ::-1]).sum(), 1))
    round_ = kind in ROUND_KINDS or (kind not in BOX_KINDS and sym > 0.88)
    H = float(v1 - v0 + 1)
    W = float(widths.max())
    cu = float(np.median(mids))
    if round_:
        r = widths / 2
        # the top (and hidden bottom) of a round thing seen from above is an ellipse: its half height e shows as the
        # rows where the width is still opening up; the wall is the rest (silhouette height = H cos φ + 2 r sin φ)
        e = int(np.argmax(r >= 0.97 * r.max()))
        e = min(e, int(0.6 * r.max()), int(0.3 * len(r)))
        Hw = H - 2 * e if e > 2 else H
        sinp = min(e / max(r.max(), 1e-6), 0.95) if e > 2 else 0.0
        Ht = Hw / math.sqrt(1 - sinp ** 2)
        body = r[e:len(r) - e] if e > 2 and len(r) - 2 * e > 4 else r
        n = min(16, len(body))
        idx = np.linspace(0, len(body) - 1, n).astype(int)
        hs = np.linspace(Ht, 0, n)                          # rows run top -> bottom
        m = lathe_mesh(hs[::-1], body[idx][::-1], Ht)
        shape, conf = "lathe", 0.45 + 0.2 * (kind in ROUND_KINDS)
        vtop = v0 + e
        scale_v = Hw / max(Ht, 1e-6)
    else:
        D = 0.6 * W
        m = box_mesh([W, D, H])
        shape, conf = "box", 0.35 + 0.2 * (kind in BOX_KINDS)
        vtop, scale_v, Ht = v0, 1.0, H

    def uv(verts):                                         # orthographic: x -> u, z -> v (top of the wall at vtop)
        return np.column_stack([cu + verts[:, 0], vtop + (Ht / 2 - verts[:, 2]) * scale_v])
    return m, shape, uv, conf
