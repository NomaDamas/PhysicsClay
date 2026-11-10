"""Which pixels are the object: a box first (the person's crop, the VLM's box, the MediaPipe detector or the centre),
then a mask inside it: GrabCut on colour, helped by depth when there is any (what sticks up out of the support plane:
on a plane, inverse depth is linear in the pixel coordinates, so the object is what that linear fit doesn't explain)."""
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DETECTOR = ROOT / "models" / "efficientdet_lite0.tflite"
_det = [None]


def clean(mask, keep="largest"):
    """Largest connected component, holes filled."""
    m = (np.asarray(mask) > 0).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return m.astype(bool)
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    m = (lab == i).astype(np.uint8)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = np.zeros_like(m)
    cv2.drawContours(out, cs, -1, 1, thickness=cv2.FILLED)
    return out.astype(bool)


def bbox(mask):
    ys, xs = np.nonzero(mask)
    if not len(xs):
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def rect_px(rect, w, h):
    """[u0, v0, u1, v1] normalised (0..1) or pixels -> clipped pixel rect, or None."""
    if rect is None or len(rect) != 4:
        return None
    r = [float(v) for v in rect]
    if max(r) <= 1.0001:
        r = [r[0] * w, r[1] * h, r[2] * w, r[3] * h]
    u0, u1 = sorted((r[0], r[2]))
    v0, v1 = sorted((r[1], r[3]))
    u0, v0, u1, v1 = max(0, int(u0)), max(0, int(v0)), min(w, int(u1)), min(h, int(v1))
    if u1 - u0 < 8 or v1 - v0 < 8:
        return None
    return u0, v0, u1, v1


def detect(img, max_results=8, score=0.3):
    """MediaPipe object detector (EfficientDet-Lite0, Apache-2.0, models/ from setup.sh) ->
    [{label, score, box: (u0, v0, u1, v1) px}]; [] without the model."""
    if not DETECTOR.exists():
        return []
    try:
        import mediapipe as mp
        from mediapipe.tasks.python import vision
        from mediapipe.tasks.python.core.base_options import BaseOptions
        if _det[0] is None:
            _det[0] = vision.ObjectDetector.create_from_options(vision.ObjectDetectorOptions(
                base_options=BaseOptions(model_asset_path=str(DETECTOR)), running_mode=vision.RunningMode.IMAGE,
                max_results=max_results, score_threshold=score))
        h, w = img.shape[:2]
        s = 640 / max(h, w)
        small = cv2.resize(img, (max(1, int(w * s)), max(1, int(h * s)))) if s < 1 else img
        sh, sw = small.shape[:2]
        res = _det[0].detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(small, cv2.COLOR_BGR2RGB)))
        out = []
        for d in res.detections:
            b, c = d.bounding_box, d.categories[0]
            if c.category_name in ("dining table", "person", "bed", "couch", "chair"):
                continue
            out.append({"label": c.category_name, "score": round(float(c.score), 2),
                         "box": (int(b.origin_x * w / sw), int(b.origin_y * h / sh),
                                 int((b.origin_x + b.width) * w / sw), int((b.origin_y + b.height) * h / sh))})
        return out
    except Exception:
        return []


def plane_residual(disp, rect, ring=0.25):
    """Inverse depth minus the plane fitted (robust least squares in u, v) to a ring around `rect` ->
    (residual map normalised by the object's spread, ok). Positive = sticks up toward the camera."""
    h, w = disp.shape
    u0, v0, u1, v1 = rect
    pu, pv = int((u1 - u0) * ring) + 4, int((v1 - v0) * ring) + 4
    U0, V0, U1, V1 = max(0, u0 - pu), max(0, v0 - pv), min(w, u1 + pu), min(h, v1 + pv)
    ringm = np.zeros((h, w), bool)
    ringm[V0:V1, U0:U1] = True
    ringm[v0:v1, u0:u1] = False
    vs, us = np.nonzero(ringm)
    if len(us) < 50:
        return None, False
    sel = np.random.default_rng(0).choice(len(us), min(len(us), 4000), replace=False)
    us, vs = us[sel].astype(float), vs[sel].astype(float)
    y = disp[vs.astype(int), us.astype(int)].astype(float)
    A = np.column_stack([us, vs, np.ones_like(us)])
    keep = np.ones(len(y), bool)
    for _ in range(4):                                  # drop other objects in the ring
        coef, *_ = np.linalg.lstsq(A[keep], y[keep], rcond=None)
        r = y - A @ coef
        s = 1.4826 * np.median(np.abs(r[keep] - np.median(r[keep]))) + 1e-9
        keep = np.abs(r) < 2.5 * s
    gv, gu = np.mgrid[0:h, 0:w]
    res = disp - (coef[0] * gu + coef[1] * gv + coef[2])
    inside = res[v0:v1, u0:u1]
    spread = float(np.percentile(inside, 95)) if inside.size else 0.0
    ok = spread > 6 * s and keep.mean() > 0.4
    return res / max(spread, 1e-9), ok


def grabcut(img, rect, init=None, iters=4):
    """GrabCut inside rect (on a <= 640 px copy). init: optional GrabCut label map (cv2.GC_*) at full size."""
    h, w = img.shape[:2]
    s = min(1.0, 640 / max(h, w))
    small = cv2.resize(img, (max(1, int(w * s)), max(1, int(h * s)))) if s < 1 else img.copy()
    sh, sw = small.shape[:2]
    r = (int(rect[0] * s), int(rect[1] * s), max(2, int((rect[2] - rect[0]) * s)), max(2, int((rect[3] - rect[1]) * s)))
    bgd, fgd = np.zeros((1, 65), np.float64), np.zeros((1, 65), np.float64)
    try:
        if init is not None:
            m = cv2.resize(init.astype(np.uint8), (sw, sh), interpolation=cv2.INTER_NEAREST)
            if not np.any((m == cv2.GC_FGD) | (m == cv2.GC_PR_FGD)) or not np.any((m == cv2.GC_BGD) | (m == cv2.GC_PR_BGD)):
                raise ValueError("degenerate init")
            cv2.grabCut(small, m, None, bgd, fgd, iters, cv2.GC_INIT_WITH_MASK)
        else:
            m = np.zeros((sh, sw), np.uint8)
            cv2.grabCut(small, m, r, bgd, fgd, iters, cv2.GC_INIT_WITH_RECT)
    except (cv2.error, ValueError):
        m = np.zeros((sh, sw), np.uint8)
        m[r[1]:r[1] + r[3], r[0]:r[0] + r[2]] = cv2.GC_PR_FGD
    fg = ((m == cv2.GC_FGD) | (m == cv2.GC_PR_FGD)).astype(np.uint8)
    return cv2.resize(fg, (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)


def object_mask(img, rect=None, disp=None, boxes=()):
    """-> (mask, source, rect). rect: the person's crop (normalised or px); boxes: candidate boxes (VLM, detector)."""
    h, w = img.shape[:2]
    r, source = rect_px(rect, w, h), "your crop"
    if r is None:
        cands = [b for b in boxes if b]
        if cands:                                      # the most central reasonably large box
            def score(b):
                cx, cy = (b[0] + b[2]) / 2 / w - 0.5, (b[1] + b[3]) / 2 / h - 0.5
                area = (b[2] - b[0]) * (b[3] - b[1]) / (w * h)
                return np.hypot(cx, cy) - 0.5 * min(area, 0.3)
            b = min(cands, key=score)
            pu, pv = int((b[2] - b[0]) * 0.08) + 2, int((b[3] - b[1]) * 0.08) + 2
            r, source = rect_px([b[0] - pu, b[1] - pv, b[2] + pu, b[3] + pv], w, h), "detector box"
    if r is None:
        r, source = (int(w * 0.18), int(h * 0.12), int(w * 0.82), int(h * 0.92)), "centre of the photo"
    init = None
    if disp is not None:
        res, ok = plane_residual(disp, r)
        if ok:
            init = np.full((h, w), cv2.GC_BGD, np.uint8)
            u0, v0, u1, v1 = r
            sub = res[v0:v1, u0:u1]
            lab = np.full(sub.shape, cv2.GC_PR_BGD, np.uint8)
            lab[sub > 0.15] = cv2.GC_PR_FGD
            lab[sub > 0.45] = cv2.GC_FGD
            init[v0:v1, u0:u1] = lab
            source += " + depth"
    m = clean(grabcut(img, r, init))
    if m.sum() < 0.002 * w * h:                        # GrabCut gave up: the box itself (honest, low confidence)
        m = np.zeros((h, w), bool)
        m[r[1]:r[3], r[0]:r[2]] = True
        source += " (box only)"
    return m, source, r
