"""What is it, what is it made of, how big is it: a vision-language model when a key is present (Gemini, or the
agent's own OpenAI-compatible model), else the MediaPipe detector's label, else what the person typed.

Keys: a visitor's key comes with the request and is used for that one call (never stored or logged); the server's
own keys (.env) are used only outside --byok (pps/spatial.py gemini_key / pps/agent.py server_llm)."""
import base64
import json
import re
import ssl
import urllib.request

import certifi
import cv2

from .base import slug

# material -> (density kg/m^3 of the solid, sliding friction, wall thickness m when hollow)
MATERIALS = {
    "ceramic": (2300.0, 0.6, 0.005), "glass": (2500.0, 0.45, 0.004), "plastic": (950.0, 0.5, 0.002),
    "metal": (2700.0, 0.4, 0.0012), "steel": (7850.0, 0.4, 0.002), "aluminium": (2700.0, 0.4, 0.0012),
    "wood": (600.0, 0.55, 0.008), "paper": (700.0, 0.5, 0.0015), "cardboard": (690.0, 0.55, 0.003),
    "fabric": (300.0, 0.75, 0.003), "rubber": (1100.0, 0.95, 0.004), "food": (950.0, 0.6, 0.004),
    "foam": (60.0, 0.8, 0.01), "stone": (2600.0, 0.6, 0.01), "unknown": (800.0, 0.6, 0.003),
}
# label words -> (scene kind, material, hollow). Scene kinds are pps/world/scene.py KINDS (the task layer knows how to
# talk about and grasp them); "mug" maps to "cup" because the scene's mug kind is the scanned mug's own geometry.
LABELS = [
    (("mug", "coffee cup", "teacup", "머그"), ("cup", "ceramic", True)), (("cup", "glass", "tumbler", "컵", "잔"), ("cup", "plastic", True)),
    (("bottle", "flask", "병"), ("bottle", "plastic", False)), (("can", "tin", "soda", "캔"), ("can", "metal", False)),
    (("jar",), ("can", "glass", False)), (("bowl", "그릇"), ("bowl", "ceramic", True)), (("plate", "dish", "접시"), ("plate", "ceramic", False)),
    (("apple", "orange", "lemon", "peach", "pear", "fruit", "tomato", "banana", "사과", "과일", "귤"), ("fruit", "food", False)),
    (("ball", "공"), ("fruit", "rubber", False)), (("vase", "꽃병"), ("vase", "ceramic", True)),
    (("block", "cube", "lego", "brick", "블록"), ("block", "wood", False)),
    (("book", "notebook", "책"), ("box", "paper", False)), (("box", "carton", "package", "상자", "박스"), ("box", "cardboard", True)),
    (("phone", "remote", "mouse", "wallet", "stapler", "calculator", "controller", "폰"), ("box", "plastic", False)),
    (("sponge",), ("box", "foam", False)), (("toy", "figure", "장난감"), ("box", "plastic", False)),
]
KIND_SHAPES = {"cup": "cylinder", "can": "cylinder", "bottle": "lathe", "bowl": "lathe", "plate": "cylinder", "fruit": "lathe",
               "vase": "lathe", "block": "box", "box": "box"}
API = "https://generativelanguage.googleapis.com/v1beta/models/"
MODELS = ("gemini-robotics-er-2-preview", "gemini-3.6-flash", "gemini-3.5-flash")
SSL_CTX = ssl.create_default_context(cafile=certifi.where())
PROMPT_OBJECT = (
    "Look at the single main object in this photo (the one nearest the centre, or the one being shown). Answer with JSON only: "
    '{"label": "<short noun, e.g. mug>", "material": "<one of ceramic, glass, plastic, metal, wood, paper, cardboard, fabric, '
    'rubber, food, stone, unknown>", "shape": "<box|cylinder|sphere|bottle|bowl|plate|irregular>", "height_cm": <number>, '
    '"width_cm": <number>, "depth_cm": <number>, "hollow": <true|false>, "mass_g": <number>, "color": "<one word>", '
    '"box_2d": [ymin, xmin, ymax, xmax] (0-1000), "reference": "<credit card|A4|aruco|none: a known-size item in view>"}. '
    "Sizes are your best estimate of the real object in centimetres.")


SHAPE_KIND = {"cylinder": "can", "sphere": "fruit", "bottle": "bottle", "bowl": "bowl", "plate": "plate", "box": "box"}


def match(label):
    """label words -> (scene kind, material, hollow) or None when the words name nothing known."""
    lab = str(label or "").lower()
    for words, (kind, mat, hollow) in LABELS:
        if any(re.search(rf"(^|[^a-z]){re.escape(w)}", lab) for w in words):
            return kind, mat, hollow
    return None


def classify(label, material=None):
    """label words -> (scene kind, material, hollow); unknown words -> a box of unknown material."""
    m = match(label)
    if m:
        return m[0], (material if material in MATERIALS else m[1]), m[2]
    return "box", (material if material in MATERIALS else "unknown"), False


def material_props(material):
    return MATERIALS.get(material) or MATERIALS["unknown"]


def _jpg(img, side=1024):
    h, w = img.shape[:2]
    s = min(1.0, side / max(h, w))
    small = cv2.resize(img, (int(w * s), int(h * s))) if s < 1 else img
    return cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()


def parse_json(text):
    m = re.search(r"(\{.*\}|\[.*\])", str(text or ""), re.S)
    if not m:
        return None
    try:
        return json.loads(m.group(1))
    except ValueError:
        return None


def gemini(img, key, prompt=PROMPT_OBJECT, timeout=25):
    """One Gemini call (ER preview, falling back to flash models when it's busy) -> (parsed JSON, model). Raises."""
    body = {"contents": [{"parts": [{"inline_data": {"mime_type": "image/jpeg", "data": base64.b64encode(_jpg(img)).decode()}},
                                    {"text": prompt}]}], "generationConfig": {"temperature": 0.2}}
    err = None
    for model in MODELS:
        req = urllib.request.Request(API + model + ":generateContent", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json", "x-goog-api-key": key, "User-Agent": "curl/8.7.1"})
        try:
            with urllib.request.urlopen(req, timeout=timeout, context=SSL_CTX) as r:
                data = json.loads(r.read())
            text = "".join(p.get("text", "") for p in data["candidates"][0]["content"]["parts"])
            return parse_json(text), model
        except Exception as e:                        # never echo the key
            err = str(e).replace(key, "***")[:120]
    raise RuntimeError(f"Gemini didn't answer ({err})")


def llm(img, cfg, prompt=PROMPT_OBJECT, timeout=60):
    """The agent's OpenAI-compatible Responses model with the photo as input_image -> (parsed JSON, model). Raises."""
    from ..agent import _text_of, responses_call
    url = "data:image/jpeg;base64," + base64.b64encode(_jpg(img)).decode()
    out = responses_call(cfg, "You identify objects in photos for a robot simulator. Reply with JSON only.",
                         [{"role": "user", "content": [{"type": "input_text", "text": prompt}, {"type": "input_image", "image_url": url}]}],
                         [], timeout=timeout)
    text = " ".join(_text_of(o) for o in out if o.get("type") == "message")
    return parse_json(text), cfg.get("model")


def describe(img, hints=None, gemini_key=None, llm_cfg=None, boxes=()):
    """-> {label, kind, material, hollow, vlm (sizes), box (px) | None, source, model?, note?}."""
    hints = hints or {}
    h, w = img.shape[:2]
    out = {"label": None, "material": hints.get("material"), "vlm": None, "box": None, "source": "none", "shape": None}
    ans, note = None, None
    for name, fn, arg in (("gemini", gemini, gemini_key), ("llm", llm, llm_cfg)):
        if not arg or hints.get("vlm") is False:          # vlm: false = don't call a vision model (tests, offline)
            continue
        try:
            ans, model = fn(img, arg)
            if isinstance(ans, list):
                ans = ans[0] if ans and isinstance(ans[0], dict) else None
            if isinstance(ans, dict) and ans.get("label"):
                out.update(source=name, model=model)
                break
            ans = None
        except Exception as e:
            note = str(e)[:140]
    if ans:
        out["label"] = str(ans.get("label"))[:40]
        out["vlm"] = {k: ans.get(k) for k in ("height_cm", "width_cm", "depth_cm", "mass_g", "hollow", "color") if ans.get(k) is not None}
        out["shape"] = ans.get("shape")
        if str(ans.get("material") or "").lower() in MATERIALS and not out["material"]:
            out["material"] = str(ans["material"]).lower()
        b = ans.get("box_2d")
        if isinstance(b, list) and len(b) == 4 and all(isinstance(v, (int, float)) for v in b):
            out["box"] = (int(b[1] / 1000 * w), int(b[0] / 1000 * h), int(b[3] / 1000 * w), int(b[2] / 1000 * h))
        ref = str(ans.get("reference") or "").lower()
        out["reference"] = "card" if "card" in ref else "a4" if "a4" in ref else None
    elif boxes:
        best = max(boxes, key=lambda d: d["score"] * ((d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1])) ** 0.25)
        out.update(label=best["label"], box=best["box"], source="detector")
    if hints.get("label"):                                 # the person's words win for the name
        out["label"], out["source"] = str(hints["label"])[:40], out["source"] if out["source"] != "none" else "you"
    if note and out["source"] in ("none", "detector", "you"):
        out["note"] = note
    kind, mat, hollow = classify(out["label"] or "", out["material"])
    src = "label" if match(out["label"] or "") else "default"
    if src == "default" and str(out.get("shape") or "").lower() in SHAPE_KIND:   # "a red cylinder": the VLM's shape
        kind, src = SHAPE_KIND[str(out["shape"]).lower()], "shape"
        hollow = kind in ("bowl",)
    if hints.get("kind") in KIND_SHAPES:
        kind, src = hints["kind"], "you"
    if isinstance(out.get("vlm"), dict) and isinstance(out["vlm"].get("hollow"), bool):
        hollow = out["vlm"]["hollow"]
    out.update(kind=kind, kind_source=src, material=mat, hollow=bool(hollow), name=slug(out["label"] or "thing") or "thing")
    return out


PROMPT_SPACE = (
    "This photo shows a table, desk or counter with things on it. Detect the support surface and every separate object "
    "on it (ignore people, walls and the surface itself). Answer with JSON only: "
    '{"surface": "<table|desk|counter|floor|shelf>", "surface_box_2d": [ymin, xmin, ymax, xmax], "surface_width_cm": <number>, '
    '"objects": [{"label": "<short noun>", "box_2d": [ymin, xmin, ymax, xmax], "material": "<material>", "height_cm": <number>, '
    '"width_cm": <number>, "depth_cm": <number>, "color": "<one word>"}]}. Coordinates are 0-1000.')


def describe_space(img, gemini_key=None, llm_cfg=None):
    """VLM reading of a table photo -> dict (surface, objects with px boxes) or None (no key / no answer)."""
    h, w = img.shape[:2]
    for fn, arg in ((gemini, gemini_key), (llm, llm_cfg)):
        if not arg:
            continue
        try:
            ans, model = fn(img, arg, PROMPT_SPACE)
        except Exception:
            continue
        if not isinstance(ans, dict) or not isinstance(ans.get("objects"), list):
            continue
        objs = []
        for o in ans["objects"][:16]:
            b = o.get("box_2d") if isinstance(o, dict) else None
            if not (isinstance(b, list) and len(b) == 4):
                continue
            objs.append({"label": str(o.get("label") or "thing")[:30], "score": 0.9, "color": o.get("color"), "material": o.get("material"),
                         "box": (int(b[1] / 1000 * w), int(b[0] / 1000 * h), int(b[3] / 1000 * w), int(b[2] / 1000 * h)),
                         "vlm": {k: o.get(k) for k in ("height_cm", "width_cm", "depth_cm") if o.get(k) is not None}})
        sb = ans.get("surface_box_2d")
        return {"surface": str(ans.get("surface") or "table"), "model": model, "objects": objs,
                "surface_box": (int(sb[1] / 1000 * w), int(sb[0] / 1000 * h), int(sb[3] / 1000 * w), int(sb[2] / 1000 * h))
                if isinstance(sb, list) and len(sb) == 4 else None, "surface_width_cm": ans.get("surface_width_cm")}
    return None
