"""Metric scale: how big the real thing is, and how sure we are.

Sources, in priority order (the first one present wins; every candidate is kept for the UI):
  1. what the person says      "it's 9 cm tall", "about 20cm long", "높이 9센티"              confidence 0.95
  2. a reference in view        an ArUco marker (DICT_4X4_50, 5 cm by default, docs/RECON.md has a print-at-home
                                one), or a credit card / A4 sheet when the person says one is in the photo   0.85 / 0.7
  3. metric depth               a depth camera / phone AR depth (the reconstruction is already in metres)     0.8
  4. the VLM's estimate         Gemini / the agent's model: "this mug is about 9 cm tall"                     0.55
  5. a prior for the label      a cup is ~10 cm tall                                                          0.3
A candidate is {"dim": "height"|"width"|"depth"|"longest", "m": metres} or {"m_per_px": ...} (a reference) plus
{"source", "confidence", "label"}."""
import math
import re

import cv2
import numpy as np

CARD = (0.08560, 0.05398)          # ISO/IEC 7810 ID-1
A4 = (0.297, 0.210)
MARKER_M = 0.05
UNITS = {"mm": 0.001, "millimeter": 0.001, "millimetre": 0.001, "밀리": 0.001, "cm": 0.01, "centimeter": 0.01, "centimetre": 0.01,
         "센티": 0.01, "센치": 0.01, "m": 1.0, "meter": 1.0, "metre": 1.0, "미터": 1.0, "in": 0.0254, "inch": 0.0254,
         "inches": 0.0254, '"': 0.0254, "인치": 0.0254}
DIMS = [("height", ("tall", "high", "height", "높이", "키")), ("width", ("wide", "width", "across", "diameter", "폭", "너비", "지름", "가로")),
        ("depth", ("deep", "depth", "thick", "두께", "깊이", "세로")), ("longest", ("long", "length", "길이", "크기", "size"))]
# label -> (dimension, metres): a typical size, used only when nothing better is known
PRIORS = {"cup": ("height", 0.10), "mug": ("height", 0.10), "bottle": ("height", 0.22), "can": ("height", 0.12),
          "bowl": ("width", 0.15), "plate": ("width", 0.22), "apple": ("width", 0.075), "orange": ("width", 0.075),
          "banana": ("longest", 0.19), "fruit": ("width", 0.075), "book": ("longest", 0.23), "box": ("longest", 0.15),
          "block": ("longest", 0.05), "cube": ("longest", 0.05), "phone": ("longest", 0.15), "cell phone": ("longest", 0.15),
          "remote": ("longest", 0.17), "mouse": ("longest", 0.11), "vase": ("height", 0.24), "toy": ("longest", 0.10),
          "glass": ("height", 0.11), "jar": ("height", 0.12), "scissors": ("longest", 0.18), "stapler": ("longest", 0.15),
          "tape": ("width", 0.10), "wallet": ("longest", 0.11), "keyboard": ("longest", 0.40), "spoon": ("longest", 0.17),
          "ball": ("width", 0.07), "sponge": ("longest", 0.10), "shoe": ("longest", 0.28), "pen": ("longest", 0.14)}
DEFAULT = ("longest", 0.10)
CONF = {"you": 0.95, "aruco": 0.85, "card": 0.7, "a4": 0.7, "depth": 0.8, "vlm": 0.55, "prior": 0.3, "default": 0.15}
LABEL = {"you": "what you said", "aruco": "the ArUco marker", "card": "the card in the photo", "a4": "the A4 sheet",
         "depth": "the depth camera", "vlm": "the vision model's estimate", "prior": "a typical size for it", "default": "a guess"}


def parse_size(text):
    """'it's 9 cm tall' / 'about 20cm long' / '높이 9센티' / '3 inches' -> {"dim", "m"} or None."""
    t = str(text or "").lower().replace(",", ".")
    m = re.search(r"(\d+(?:\.\d+)?)\s*(millimetres?|millimeters?|centimetres?|centimeters?|metres?|meters?|inches|inch|mm|cm|in\b|m\b|\"|밀리|센티|센치|미터|인치)", t)
    if not m:
        return None
    unit = m.group(2)
    unit = next((u for u in sorted(UNITS, key=len, reverse=True) if unit.startswith(u)), unit)
    val = float(m.group(1)) * UNITS.get(unit, 0.01)
    if not (0.003 <= val <= 3.0):
        return None
    dim = next((d for d, words in DIMS if any(w in t for w in words)), "longest")
    return {"dim": dim, "m": round(val, 4)}


def _dict():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def marker_image(marker_id=0, px=400, border=1):
    """The print-at-home marker (white quiet zone around it) as a grayscale image."""
    img = cv2.aruco.generateImageMarker(_dict(), int(marker_id), int(px))
    q = int(px * 0.25 * border)
    return cv2.copyMakeBorder(img, q, q, q, q, cv2.BORDER_CONSTANT, value=255)


def find_markers(img):
    """-> [{"id", "corners" (4, 2) px}] for every DICT_4X4_50 marker in the image."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    det = cv2.aruco.ArucoDetector(_dict(), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        return []
    return [{"id": int(i), "corners": c.reshape(4, 2).astype(float)} for c, i in zip(corners, ids.ravel())]


def aruco(img, marker_m=MARKER_M):
    """Scale from an ArUco marker: metres per pixel across the image (its least foreshortened side) -> candidate or None.
    Most accurate when the marker lies next to the object at the same distance from the camera."""
    ms = find_markers(img)
    if not ms:
        return None
    c = ms[0]["corners"]
    sides = [np.linalg.norm(c[(k + 1) % 4] - c[k]) for k in range(4)]
    side = max(sides)
    if side < 12:
        return None
    return {"m_per_px": marker_m / side, "source": "aruco", "confidence": CONF["aruco"], "label": LABEL["aruco"],
            "marker": {"id": ms[0]["id"], "corners": [[round(float(x), 1) for x in p] for p in c], "side_px": round(float(side), 1)}}


def marker_pose(img, K, marker_m=MARKER_M):
    """Pose of the first marker -> (R, t) (marker frame -> camera, metres; marker z = the table's normal) or None."""
    ms = find_markers(img)
    if not ms:
        return None
    s = marker_m / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, ms[0]["corners"].astype(np.float64), np.asarray(K, np.float64), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.ravel(), ms[0]


def reference_rect(img, kind, exclude=None):
    """A credit card or an A4 sheet the person says is in the photo: the largest 4-corner contour with the right aspect
    -> candidate or None. exclude: the object's mask (the reference must not be the object)."""
    size = CARD if kind == "card" else A4
    want = size[0] / size[1]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 40, 120)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))
    cs, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    h, w = gray.shape
    best = None
    for c in cs:
        a = cv2.contourArea(c)
        if a < 0.004 * w * h or a > 0.8 * w * h:
            continue
        ap = cv2.approxPolyDP(c, 0.03 * cv2.arcLength(c, True), True)
        if len(ap) != 4 or not cv2.isContourConvex(ap):
            continue
        (_, _), (rw, rh), _ = cv2.minAreaRect(ap)
        if min(rw, rh) < 10:
            continue
        asp = max(rw, rh) / min(rw, rh)
        if abs(asp - want) / want > 0.12:
            continue
        if exclude is not None:
            m = np.zeros((h, w), np.uint8)
            cv2.drawContours(m, [ap], -1, 1, cv2.FILLED)
            if (m.astype(bool) & exclude).sum() > 0.3 * m.sum():
                continue
        if best is None or a > best[0]:
            best = (a, max(rw, rh))
    if best is None:
        return None
    return {"m_per_px": size[0] / best[1], "source": kind, "confidence": CONF[kind], "label": LABEL[kind]}


def prior(label):
    lab = str(label or "").lower()
    for k in sorted(PRIORS, key=len, reverse=True):
        if k in lab:
            d, m = PRIORS[k]
            return {"dim": d, "m": m, "source": "prior", "confidence": CONF["prior"], "label": f"{LABEL['prior']} ({k})"}
    return {"dim": DEFAULT[0], "m": DEFAULT[1], "source": "default", "confidence": CONF["default"], "label": LABEL["default"]}


def candidates(img=None, said=None, vlm=None, label=None, reference=None, mask=None, metric=False, marker_m=MARKER_M):
    """All scale candidates, best first. vlm: {"height_cm", "width_cm", "depth_cm"} from recognize."""
    out = []
    p = parse_size(said) if said else None
    if p:
        out.append(dict(p, source="you", confidence=CONF["you"], label=LABEL["you"]))
    if img is not None:
        a = aruco(img, marker_m)
        if a:
            out.append(a)
        if reference in ("card", "a4"):
            r = reference_rect(img, reference, mask)
            if r:
                out.append(r)
    if metric:
        out.append({"metric": True, "source": "depth", "confidence": CONF["depth"], "label": LABEL["depth"]})
    if isinstance(vlm, dict):
        for dim, k in (("height", "height_cm"), ("width", "width_cm"), ("depth", "depth_cm")):
            try:
                v = float(vlm.get(k) or 0) / 100
            except (TypeError, ValueError):
                v = 0
            if 0.005 < v < 3:
                out.append({"dim": dim, "m": round(v, 4), "source": "vlm", "confidence": CONF["vlm"], "label": LABEL["vlm"]})
                break
    out.append(prior(label))
    order = {"you": 0, "aruco": 1, "card": 2, "a4": 2, "depth": 3, "vlm": 4, "prior": 5, "default": 6}
    return sorted(out, key=lambda c: order.get(c["source"], 9))


def factor(c, extents, px_per_unit=None, metric=False):
    """Scale factor for a mesh (extents in its units: x across the image, y depth, z up) from candidate c, or None
    when the candidate doesn't apply (a reference without px_per_unit)."""
    ex = np.asarray(extents, float)
    if c.get("metric"):
        return 1.0 if metric else None
    if "m_per_px" in c:
        if metric:
            return None                         # the mesh is in metres already: a reference only checks it
        return None if not px_per_unit else float(c["m_per_px"] * px_per_unit)
    idx = {"width": 0, "depth": 1, "height": 2}.get(c["dim"])
    have = float(ex[idx]) if idx is not None else float(ex.max())
    if have <= 0 or not math.isfinite(have):
        return None
    return float(c["m"]) / have


def choose(cands, extents, px_per_unit=None, metric=False):
    """-> (factor, candidate used). Metric reconstructions keep their scale unless the person said a size."""
    for c in cands:
        if metric and c["source"] not in ("you", "depth"):
            break
        f = factor(c, extents, px_per_unit, metric)
        if f is not None and f > 0:
            return f, c
    if metric:
        return 1.0, {"metric": True, "source": "depth", "confidence": CONF["depth"], "label": LABEL["depth"]}
    c = cands[-1] if cands else dict(zip(("dim", "m"), DEFAULT), source="default", confidence=CONF["default"], label=LABEL["default"])
    return factor(c, extents) or 1.0, c
