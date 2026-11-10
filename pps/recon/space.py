"""Scan my table: one photo of a table / desk / counter -> the support surface and the objects on it -> a scene spec.

1. objects: the VLM's boxes when a key is present, else the MediaPipe detector's, else blobs that differ from the
   surface (its colour, and with depth what sticks up out of its plane);
2. the surface as a metric plane, best first: an ArUco marker lying on it (exact) · metric depth (depth camera / AR) ·
   relative depth (Depth Anything; the size then comes from what the objects are) · an assumed camera (35° down,
   50 cm above the table; a rough guess, said so);
3. each object's footprint: the ray through its lowest pixel meets the plane (the front contact point), its width from
   the box's bottom corners, its height from where its top projects, its depth from the VLM or its kind;
4. the layout keeps its relative positions and is centred on the robot's reachable band (photo taken from where the
   robot stands: the bottom of the photo is the edge nearest the robot). Objects can stay primitives or be reconstructed
   (pps/recon/providers/primitive.py on each crop) and saved to the library."""
import math

import cv2
import numpy as np

from . import depth as depthmod
from . import fit, recognize, scale, segment
from .base import slug

CAM_PITCH, CAM_HEIGHT = math.radians(35), 0.50


# ------------------------------------------------------------------ the plane
class Plane:
    """Table plane in the camera frame (n·p + d = 0, n towards the camera) + metric factor (1.0 when metric)."""

    def __init__(self, n, d, K, source, confidence, metric=True):
        self.n, self.d, self.K, self.source, self.confidence, self.metric = np.asarray(n, float), float(d), K, source, confidence, metric
        x = np.array([1.0, 0, 0]) - self.n[0] * self.n
        self.x = x / np.linalg.norm(x)
        self.y = np.cross(self.n, self.x)               # along the table, away from the camera
        self.f = 1.0                                    # metres per plane unit (pseudo depth: set from the objects)

    def ray(self, u, v):
        return np.array([(u - self.K[0, 2]) / self.K[0, 0], (v - self.K[1, 2]) / self.K[1, 1], 1.0])

    def hit(self, u, v):
        r = self.ray(u, v)
        den = float(self.n @ r)
        if abs(den) < 1e-9:
            return None
        t = -self.d / den
        return r * t if t > 0 else None

    def table_xy(self, p):
        return np.array([p @ self.x, p @ self.y])

    def height(self, base, v_top):
        """Height (plane units) of a vertical segment from `base` whose top projects to image row v_top."""
        lo, hi = 0.0, 3.0 * abs(self.d)
        if fit.project((base + self.n * hi)[None], self.K)[0, 1] > v_top:
            return hi
        for _ in range(40):
            mid = (lo + hi) / 2
            v = fit.project((base + self.n * mid)[None], self.K)[0, 1]
            lo, hi = (mid, hi) if v > v_top else (lo, mid)
        return (lo + hi) / 2


def assumed_plane(K):
    p = CAM_PITCH
    return Plane([0.0, -math.cos(p), -math.sin(p)], CAM_HEIGHT, K, "an assumed camera (35° down, 50 cm above the table)", 0.2)


def marker_plane(img, K):
    mp = scale.marker_pose(img, K)
    if mp is None:
        return None
    R, t, _ = mp
    n = R[:, 2]
    d = -float(n @ t)
    if d < 0:
        n, d = -n, -d
    return Plane(n, d, K, "the ArUco marker on the table", 0.85)


def depth_plane(Z, K, exclude, metric):
    """RANSAC plane over the pixels not covered by objects; must face up (image up) and hold a good share."""
    keep = ~exclude & np.isfinite(Z)
    h, w = Z.shape
    keep[: int(h * 0.15)] = False                       # the far wall / background is rarely the table
    stride = max(1, int(np.sqrt(keep.sum() / 15000)))
    P, _ = fit.backproject(Z, K, keep, stride)
    if len(P) < 200:
        return None
    pl = fit.fit_plane(P, thresh=0.01 * float(np.median(P[:, 2])), iters=300)
    if pl is None:
        return None
    n, d, inl = pl
    if inl.mean() < 0.25 or float(n @ np.array([0.0, -1.0, 0.0])) < 0.25:
        return None
    return Plane(n, d, K, "the depth camera" if metric else "relative depth (Depth Anything V2 Small)", 0.75 if metric else 0.4, metric)


# ------------------------------------------------------------------ objects without a VLM: blobs off the surface
def surface_region(img):
    """The surface's colour clusters (seeded from the lower middle of the photo) -> (surface pixels, table hull mask)."""
    h, w = img.shape[:2]
    lab = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2LAB).reshape(-1, 3).astype(np.float32)
    rng = np.random.default_rng(0)
    sel = rng.choice(len(lab), min(len(lab), 20000), replace=False)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
    _, _, centers = cv2.kmeans(lab[sel], 6, None, crit, 2, cv2.KMEANS_PP_CENTERS)
    labels = np.argmin(((lab[:, None, :] - centers[None]) ** 2).sum(-1), axis=1).reshape(h, w)
    seed = labels[int(h * 0.55):int(h * 0.95), int(w * 0.2):int(w * 0.8)]
    counts = np.bincount(seed.ravel(), minlength=len(centers)) / max(seed.size, 1)
    surf_ids = [i for i in range(len(centers)) if counts[i] >= 0.12] or [int(np.argmax(counts))]
    surf = np.isin(labels, surf_ids).astype(np.uint8)
    surf = cv2.morphologyEx(surf, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    comp = segment.clean(surf)
    hull = np.zeros((h, w), np.uint8)
    cs, _ = cv2.findContours(comp.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cs:
        cv2.fillPoly(hull, [cv2.convexHull(np.vstack(cs))], 1)
    return surf.astype(bool), hull.astype(bool)


def blobs(img, disp=None):
    h, w = img.shape[:2]
    surf, hull = surface_region(img)
    cand = hull & ~surf
    if disp is not None and hull.any():
        vs, us = np.nonzero(hull & surf)
        if len(us) > 100:
            sel = np.random.default_rng(1).choice(len(us), min(len(us), 5000), replace=False)
            A = np.column_stack([us[sel], vs[sel], np.ones(len(sel))]).astype(float)
            coef, *_ = np.linalg.lstsq(A, disp[vs[sel], us[sel]].astype(float), rcond=None)
            gv, gu = np.mgrid[0:h, 0:w]
            res = disp - (coef[0] * gu + coef[1] * gv + coef[2])
            spread = float(np.percentile(np.abs(res[hull & surf]), 90)) + 1e-9
            cand |= hull & (res > 4 * spread)
    k = max(3, int(min(h, w) * 0.012)) | 1
    cand = cv2.morphologyEx(cand.astype(np.uint8), cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
    cand = cv2.morphologyEx(cand, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(cand, 8)
    out = []
    for i in range(1, n):
        a = stats[i, cv2.CC_STAT_AREA]
        if a < 0.0008 * w * h or a > 0.25 * w * h:
            continue
        x, y, bw, bh = stats[i, :4]
        if bw > 0.6 * w:
            continue
        out.append({"label": None, "score": 0.5, "box": (int(x), int(y), int(x + bw), int(y + bh)), "mask": lab == i})
    return out, hull


def height_blobs(Z, plane, metric=True):
    """With depth: what stands on the plane. Height map -> table (|h| small, its largest region) and objects (h above a
    threshold inside the table's outline) -> (blobs, table hull)."""
    h, w = Z.shape
    P, uv = fit.backproject(Z, plane.K, None, 1)
    hm = np.full((h, w), np.nan)
    hm[uv[:, 1].astype(int), uv[:, 0].astype(int)] = P @ plane.n + plane.d
    scale_ = 1.0 if metric else float(np.nanmedian(Z))
    tol, thr = 0.006 * scale_, 0.012 * scale_
    table = segment.clean(np.nan_to_num(np.abs(hm), nan=9) < tol)
    hull = np.zeros((h, w), np.uint8)
    cs, _ = cv2.findContours(table.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cs:
        cv2.fillPoly(hull, [cv2.convexHull(np.vstack(cs))], 1)
    cand = (np.nan_to_num(hm, nan=-9) > thr) & hull.astype(bool)
    k = max(3, int(min(h, w) * 0.008)) | 1
    cand = cv2.morphologyEx(cand.astype(np.uint8), cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(cand, 8)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 0.0006 * w * h:
            continue
        x, y, bw, bh = stats[i, :4]
        out.append({"label": None, "score": 0.6, "box": (int(x), int(y), int(x + bw), int(y + bh)), "mask": lab == i})
    return out, hull.astype(bool)


def measure_points(Z, plane, mask, kind=None):
    """An object's 3D points on the plane -> footprint centre, extents across / along the photo, height, shape."""
    k = max(1, int(np.sqrt(mask.sum()) * 0.04))
    er = cv2.erode(mask.astype(np.uint8), np.ones((k, k), np.uint8)).astype(bool)
    P, _ = fit.backproject(Z, plane.K, er if er.sum() > 30 else mask, max(1, int(np.sqrt(mask.sum() / 6000))))
    if len(P) < 30:
        return None
    R = np.stack([plane.x, plane.y, plane.n])
    Q = (P + plane.d * plane.n) @ R.T                  # origin: the plane point nearest the camera
    H = float(np.percentile(Q[:, 2], 98))
    if H <= 0:
        return None
    Q = Q[Q[:, 2] > 0.03 * H]
    if len(Q) < 30:
        return None
    shape, prm, _ = fit.choose_shape(Q, kind)
    if shape == "box":
        w_, l_, h_ = prm["dims"]
        c, s = abs(np.cos(prm["yaw"])), abs(np.sin(prm["yaw"]))
        ex, ey, xy = c * w_ + s * l_, s * w_ + c * l_, prm["center"]
    elif shape in ("cylinder", "lathe"):
        ex = ey = 2 * prm["r"]
        h_, xy = prm["h"], prm["center"]
    else:
        lo, hi = Q[:, :2].min(0), Q[:, :2].max(0)
        ex, ey, h_, xy = hi[0] - lo[0], 2 * (hi[1] - lo[1]), H, [(lo[0] + hi[0]) / 2, hi[1]]
    return {"xy": np.asarray(xy, float), "front": False, "w": float(ex), "d": float(ey), "h": float(h_), "shape": shape}


def _kind_from_dims(shape, w, d, h):
    if shape == "box":
        return "block" if max(w, d, h) < 0.07 else "box"
    if h > 1.6 * max(w, d):
        return "bottle" if h > 0.15 else "can"
    if h < 0.3 * max(w, d):
        return "plate" if shape in ("cylinder", "lathe") else "box"
    if shape == "hull":
        return "fruit" if h < 0.9 * max(w, d) else "box"
    return "cup"


def _iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def _kind_from_shape(mask, w_m, h_m):
    if mask is not None and mask.any():
        cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = max(cs, key=cv2.contourArea)
        (_, _), (rw, rh), _ = cv2.minAreaRect(c)
        fill = cv2.contourArea(c) / max(rw * rh, 1)
    else:
        fill = 0.9
    if h_m > 1.5 * w_m:
        return "bottle" if h_m > 0.15 else "can"
    if h_m < 0.3 * w_m:
        return "plate" if fill < 0.88 else "box"
    if fill >= 0.88:
        return "block" if max(w_m, h_m) < 0.07 else "box"
    return "cup" if h_m > 0.8 * w_m else "fruit"


def color_name(img, mask=None, box=None):
    """A scene colour name by hue (light and shading change brightness, not hue): the saturated pixels' hue, or
    white / gray / black for unsaturated things."""
    if mask is not None and mask.any():
        px = img[mask]
    else:
        u0, v0, u1, v1 = box
        cu, cv_ = (u0 + u1) // 2, (v0 + v1) // 2
        du, dv = max(1, (u1 - u0) // 4), max(1, (v1 - v0) // 4)
        px = img[cv_ - dv:cv_ + dv, cu - du:cu + du].reshape(-1, 3)
    hsv = cv2.cvtColor(px.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(float)
    hue, sat, val = hsv[:, 0] * 2, hsv[:, 1] / 255, hsv[:, 2] / 255
    if np.median(sat) < 0.25:
        v = float(np.median(val))
        return "white" if v > 0.78 else "black" if v < 0.25 else "gray"
    sel = sat >= np.median(sat)
    hist = np.bincount((hue[sel] // 10).astype(int) % 36, minlength=36)
    hh = (int(np.argmax(np.convolve(np.r_[hist[-1], hist, hist[0]], [1, 2, 1], "valid"))) * 10 + 5) % 360
    v = float(np.median(val[sel]))
    if hh < 15 or hh >= 340:
        return "brown" if v < 0.35 else "red"
    if hh < 40:
        return "brown" if v < 0.5 else "orange"
    if hh < 70:
        return "yellow"
    if hh < 165:
        return "green"
    if hh < 255:
        return "blue"
    if hh < 300:
        return "purple"
    return "pink"


# ------------------------------------------------------------------ the scan
def scan(job, img, hints=None, depth=None, K=None, gemini_key=None, llm_cfg=None):
    """-> layout {surface, objects [{label, kind, color, size_m, xy (photo table frame: x right, y away), ...}], ...}"""
    hints = hints or {}
    h, w = img.shape[:2]
    K = fit.intrinsics(w, h, K)
    job.at("recognize")
    vlm = recognize.describe_space(img, gemini_key, llm_cfg) if hints.get("vlm") is not False else None
    dets = list(vlm["objects"]) if vlm else [d for d in segment.detect(img, max_results=12, score=0.3)]
    job.at("depth")
    disp = None
    if depth is None:
        if not depthmod.available() and depthmod.status()["can_download"]:
            job.message = f"Downloading the depth model once (~{depthmod.SIZE_MB} MB)…"
            depthmod.ensure()
        disp = depthmod.estimate(img)
        if disp is None:
            job.skip("depth")
    job.at("surface")
    det_mask = np.zeros((h, w), bool)
    for d in dets:
        u0, v0, u1, v1 = d["box"]
        det_mask[max(0, v0):v1, max(0, u0):u1] = True
    Z = np.asarray(depth, float) if depth is not None else (fit.depth_from_disparity(disp) if disp is not None else None)
    plane = marker_plane(img, K)
    if plane is None and Z is not None:
        plane = depth_plane(Z, K, det_mask, depth is not None)
    if plane is None:
        plane = assumed_plane(K)
    if Z is not None and (plane.metric or depth is None) and not plane.source.startswith("an assumed"):
        if plane.source.startswith("the ArUco") and depth is None:    # marker plane is metric; pseudo depth isn't
            found, hull = blobs(img, disp)
        else:
            found, hull = height_blobs(Z, plane, metric=depth is not None)
    else:
        found, hull = blobs(img, disp)
    if not vlm:                                         # detector labels for the blobs they overlap, other blobs stay
        objs = []
        for b in found:
            d = max(dets, key=lambda x: _iou(x["box"], b["box"]), default=None)
            if d is not None and _iou(d["box"], b["box"]) > 0.3:
                b = dict(b, label=d["label"], score=d["score"])
            objs.append(b)
        for d in dets:                                  # confident detections the colours missed
            if d["score"] >= 0.5 and all(_iou(d["box"], b["box"]) < 0.2 for b in objs) and hull[min(h - 1, d["box"][3] - 1), (d["box"][0] + d["box"][2]) // 2]:
                objs.append(dict(d, mask=None))
    else:
        objs = []
        for d in dets:
            m = next((b["mask"] for b in found if _iou(b["box"], d["box"]) > 0.4), None)
            objs.append(dict(d, mask=m))
    job.at("layout")
    use_points = Z is not None and not plane.source.startswith("an assumed") and not (plane.source.startswith("the ArUco") and depth is None)
    out = []
    for o in objs[:16]:
        u0, v0, u1, v1 = o["box"]
        m = o.get("mask")
        label = o.get("label")
        kind, material, _ = recognize.classify(label or "")
        meas = measure_points(Z, plane, m, kind if label else None) if use_points and m is not None and m.any() else None
        if meas is not None and depth is None:          # relative depth: trust widths and heights, not the depth axis
            meas.update(d=None, shape=None)
        if meas is None:                                # rays: the lowest pixel meets the plane, the top projects up
            if m is not None and m.any():
                ys, xs = np.nonzero(m)
                vb = float(np.percentile(ys, 99))
                ub = float(np.median(xs[ys >= vb - 2])) if np.any(ys >= vb - 2) else (u0 + u1) / 2
            else:
                vb, ub = v1 - 0.02 * (v1 - v0), (u0 + u1) / 2
            base = plane.hit(ub, vb)
            pl, pr = plane.hit(u0, vb), plane.hit(u1, vb)
            if base is None or pl is None or pr is None:
                continue
            wid = float(np.linalg.norm(pr - pl))
            meas = {"xy": plane.table_xy(base), "front": True, "w": wid, "d": None, "h": float(plane.height(base, v0)), "shape": None}
        out.append(dict(meas, label=label, kind=kind if label else None, material=material, box=[int(u0), int(v0), int(u1), int(v1)],
                        mask=m, vlm=o.get("vlm") or {}, score=o.get("score", 0.5)))
    if not out:
        return {"surface": {"kind": "table", "from": plane.source, "confidence": plane.confidence}, "objects": [],
                "scale": {"source": plane.source, "confidence": plane.confidence}, "notes": ["I couldn't find objects on the table."]}
    f, f_src, f_conf = 1.0, plane.source, plane.confidence
    if not plane.metric:                                 # relative depth: sizes from what the objects are
        ratios = []
        for o in out:
            vlm_h = o["vlm"].get("height_cm") if isinstance(o["vlm"], dict) else None
            if vlm_h:
                ratios.append(float(vlm_h) / 100 / max(o["h"], 1e-9))
            elif o["label"]:
                c = scale.prior(o["label"])
                if c["source"] == "prior":
                    have = o["h"] if c["dim"] == "height" else o["w"]
                    ratios.append(c["m"] / max(have, 1e-9))
        if ratios:
            f, f_src, f_conf = float(np.median(ratios)), "typical sizes of what's on it", 0.35
        else:
            f = 0.10 / max(float(np.median([max(o["w"], o["h"]) for o in out])), 1e-9)
            f_src, f_conf = "a guess (~10 cm objects)", 0.15
    items, names = [], set()
    for o in out:
        wid, hei = o["w"] * f, o["h"] * f
        if o["kind"] is None:
            o["kind"] = _kind_from_dims(o["shape"], wid, (o["d"] or wid) * f, hei) if o["shape"] else _kind_from_shape(o["mask"], wid, hei)
        vd = o["vlm"].get("depth_cm") if isinstance(o["vlm"], dict) else None
        if o["d"] is not None:
            dep = o["d"] * f
        else:
            dep = float(vd) / 100 if vd else (wid if o["kind"] in fit.ROUND_KINDS else 0.75 * wid)
        xy = np.asarray(o["xy"], float) * f + (np.array([0.0, dep / 2]) if o.get("front") else 0.0)
        label = o["label"] or o["kind"]
        color = color_name(img, o["mask"], o["box"])
        base = slug(f"{color}_{label}" if not o["label"] else label) or "thing"
        name, k = base, 2
        while name in names:
            name, k = f"{base}_{k}", k + 1
        names.add(name)
        items.append({"name": name, "label": label, "kind": o["kind"], "color": color, "material": o["material"],
                      "size_m": [round(float(np.clip(dep, 0.02, 0.6)), 3), round(float(np.clip(wid, 0.02, 0.6)), 3), round(float(np.clip(hei, 0.02, 0.6)), 3)],
                      "xy": [round(float(xy[0]), 3), round(float(xy[1]), 3)], "box": o["box"], "score": round(float(o["score"]), 2)})
    surface = str((vlm or {}).get("surface") or hints.get("surface") or "table").lower()
    hull_px = None
    cs, _ = cv2.findContours(hull.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cs:
        hull_px = cv2.approxPolyDP(max(cs, key=cv2.contourArea), 0.01 * max(h, w), True).reshape(-1, 2).tolist()
    return {"surface": {"kind": surface, "from": plane.source, "confidence": plane.confidence, "polygon": hull_px},
            "objects": items, "scale": {"source": f_src, "confidence": round(min(f_conf, plane.confidence) if not plane.metric else plane.confidence, 2)},
            "recognized_by": "vlm" if vlm else ("detector + colours" if dets else "colours"), "model": (vlm or {}).get("model"),
            "image": {"w": w, "h": h}, "objects_px": out, "plane": plane, "disp": disp, "K": K}


def template_for(surface):
    s = str(surface or "").lower()
    return "kitchen" if any(k in s for k in ("counter", "kitchen")) else "floor" if "floor" in s else "shelf" if "shelf" in s else "table"


def to_spec(layout, template=None, lib=None):
    """Layout -> scene spec: relative positions kept, centred on the reachable band in front of the robot (the photo's
    bottom edge is the robot's side). lib: {object name: (library id, scale)} for reconstructed shapes."""
    from ..world.scene import surface as scene_surface
    tpl = template or template_for(layout["surface"]["kind"])
    s = scene_surface(tpl)
    near = s["x"][0]
    objs = layout["objects"]
    notes = []
    if not objs:
        return {"template": tpl, "objects": []}, ["no objects found"]
    X = np.array([o["xy"][0] for o in objs])
    Y = np.array([o["xy"][1] for o in objs])
    cx, cy = (X.min() + X.max()) / 2, (Y.min() + Y.max()) / 2
    span_x, span_y = max(np.ptp(Y), 1e-6), max(np.ptp(X), 1e-6)    # sim x <- photo depth, sim y <- photo across
    k = min(1.0, 0.26 / span_x, 0.72 / span_y)
    if k < 0.999:
        notes.append(f"layout squeezed to {k * 100:.0f} % to fit the robot's reach")
    spec_objs = []
    for o in objs:
        x = near + 0.16 + (o["xy"][1] - cy) * k
        y = -(o["xy"][0] - cx) * k
        so = {"name": o["name"], "kind": o["kind"], "color": o["color"], "size": [o["size_m"][0], o["size_m"][1], o["size_m"][2]],
              "pos": [round(float(x), 3), round(float(y), 3)], "movable": True}
        if lib and o["name"] in lib:
            lid, sc = lib[o["name"]]
            so = {"name": o["name"], "kind": o["kind"], "mesh": lid, "scale": sc, "pos": so["pos"], "movable": True}
        spec_objs.append(so)
    return {"template": tpl, "objects": spec_objs}, notes


def public(layout):
    """JSON-able summary for the UI (no masks / planes)."""
    return {"surface": {k: v for k, v in layout["surface"].items()}, "scale": layout["scale"], "recognized_by": layout.get("recognized_by"),
            "model": layout.get("model"), "image": layout.get("image"), "notes": layout.get("notes", []),
            "objects": [{k: o[k] for k in ("name", "label", "kind", "color", "material", "size_m", "xy", "box", "score")} for o in layout["objects"]]}
