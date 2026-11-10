"""Photo(s) -> a physics-ready object (one job): recognize -> depth -> outline -> shape (a provider) -> metric scale ->
watertight / decimated / convex pieces / mass and inertia. "Add to scene" then saves it to the library and returns the
scene-spec object. Several photos: each is reconstructed, the clearest one is kept and the others check its size."""
import io
import time

import cv2
import numpy as np

from . import depth as depthmod
from . import library, physical, recognize, scale, segment
from .base import ReconError, slug
from .providers import get as get_provider

MAX_SIDE = 1600


def decode_image(blob, max_side=MAX_SIDE):
    """JPEG / PNG / WebP bytes (EXIF orientation applied) -> BGR, at most max_side px."""
    img = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ReconError("That file isn't an image I can read (JPEG, PNG or WebP).")
    h, w = img.shape[:2]
    s = min(1.0, max_side / max(h, w))
    return cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA) if s < 1 else img


def decode_depth(blob, shape):
    """Metric depth for a photo: a 16-bit PNG in millimetres, or a .npy float array in metres -> (H, W) metres."""
    if blob[:6] == b"\x93NUMPY":
        d = np.load(io.BytesIO(blob), allow_pickle=False).astype(float)
    else:
        d = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_UNCHANGED)
        if d is None:
            raise ReconError("The depth file must be a 16-bit PNG in millimetres or a .npy array in metres.")
        d = d.astype(float) / 1000.0
    if d.ndim == 3:
        d = d[..., 0]
    d[(d <= 0) | ~np.isfinite(d)] = np.nan
    if d.shape != tuple(shape[:2]):
        d = cv2.resize(d, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
    return d


def crop_jpg(img, mask, side=512):
    ys, xs = np.nonzero(mask)
    h, w = img.shape[:2]
    if len(xs):
        p = int(0.15 * max(xs.ptp(), ys.ptp())) + 4
        img = img[max(0, ys.min() - p):min(h, ys.max() + p), max(0, xs.min() - p):min(w, xs.max() + p)]
    s = min(1.0, side / max(img.shape[:2]))
    if s < 1:
        img = cv2.resize(img, (int(img.shape[1] * s), int(img.shape[0] * s)), interpolation=cv2.INTER_AREA)
    return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()


def overlay_jpg(img, mask, side=900):
    """The photo with the object's outline drawn and the rest dimmed (the mask the person confirms)."""
    out = img.copy()
    out[~mask] = (out[~mask] * 0.4).astype(np.uint8)
    cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, cs, -1, (214, 105, 82), max(2, int(max(img.shape[:2]) / 300)))
    s = min(1.0, side / max(out.shape[:2]))
    if s < 1:
        out = cv2.resize(out, (int(out.shape[1] * s), int(out.shape[0] * s)), interpolation=cv2.INTER_AREA)
    return cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 82])[1].tobytes()


def mesh_bin(mesh, colors=None):
    """[nv, nf] uint32, verts f32, faces u32, then (optional) RGB u8 per vertex: the 3D stage's mesh format + colours."""
    v = np.ascontiguousarray(mesh.vertices, dtype="<f4")
    f = np.ascontiguousarray(mesh.faces, dtype="<u4")
    out = np.array([len(v), len(f)], dtype="<u4").tobytes() + v.tobytes() + f.tobytes()
    if colors is not None and len(colors) == len(v):
        out += np.ascontiguousarray(np.asarray(colors)[:, :3], np.uint8).tobytes()
    return out


def _mass_hint(hints, desc):
    m = hints.get("mass_kg")
    if isinstance(m, (int, float)) and 0.001 < m < 50:
        return float(m), "you"
    g = (desc.get("vlm") or {}).get("mass_g")
    try:
        g = float(g)
    except (TypeError, ValueError):
        g = 0
    return (g / 1000.0, "vlm") if 5 <= g <= 20000 else (None, None)


def import_object(job, images, hints=None, provider="primitive", keys=None, depths=None, K=None, gemini_key=None, llm_cfg=None):
    """Run the whole pipeline for job (pps/recon/jobs.py). images: BGR arrays; depths: metric depth per image or None."""
    hints = dict(hints or {})
    t0 = time.time()
    prov = get_provider(provider)
    ok, why = prov.available(keys)
    if not ok:
        raise ReconError(f"{prov.label} isn't available: {why}.")
    depths = list(depths or []) + [None] * (len(images) - len(depths or []))
    img = images[0]

    job.at("recognize")
    boxes = segment.detect(img)
    desc = recognize.describe(img, hints, gemini_key=gemini_key, llm_cfg=llm_cfg, boxes=boxes)
    rec_note = desc.get("note")
    label = desc["label"] or "thing"
    job.message = f"Looks like a {label}" + (f" ({desc['material']})" if desc["material"] != "unknown" else "") + "."

    results = []
    for k, im in enumerate(images[:4]):
        d_metric, disp = depths[k], None
        job.at("depth", k / max(1, len(images)))
        if d_metric is None and prov.name == "primitive":
            if not depthmod.available() and depthmod.status()["can_download"]:
                job.message = f"Downloading the depth model once (~{depthmod.SIZE_MB} MB)…"
                depthmod.ensure(lambda f: setattr(job, "progress", round(1 / 6 * (1 + f), 3)))
            disp = depthmod.estimate(im)
        if d_metric is None and disp is None:
            job.skip("depth")
        job.at("outline")
        own = [b["box"] for b in boxes if b["label"] == desc.get("label")] if k == 0 else []
        mask, msrc, rect = segment.object_mask(im, hints.get("crop") if k == 0 else None, disp if disp is not None else d_metric,
                                               boxes=([desc["box"]] if desc.get("box") and k == 0 else []) + own +
                                               ([b["box"] for b in segment.detect(im)] if k else []))
        job.at("shape", 0.1)
        kw = {"progress": lambda step, f: job.at(step, f), "keys": keys} if prov.name != "primitive" else {}
        if prov.name == "primitive":
            r = prov.reconstruct(im, mask, {"kind": desc["kind"]}, depth=d_metric, disp=disp, K=K)
        else:
            r = prov.reconstruct(im, mask, {"kind": desc["kind"]}, **kw)
        results.append((r, im, mask, msrc, rect, d_metric is not None))
    best = max(range(len(results)), key=lambda i: results[i][0].confidence)
    recon, im, mask, msrc, rect, had_metric = results[best]
    agree = None
    if len(results) > 1:                                  # do the other views agree on the proportions?
        a = [np.sort(r.extents / max(r.extents.max(), 1e-9)) for r, *_ in results]
        agree = float(1 - np.mean([np.abs(x - a[best]).max() for x in a]))
        recon.confidence = float(np.clip(recon.confidence + 0.1 * (agree - 0.8), 0.1, 0.97))

    job.at("scale")
    cands = scale.candidates(im, said=hints.get("size"), vlm=desc.get("vlm"), label=label,
                             reference=hints.get("reference") or desc.get("reference"), mask=mask, metric=recon.metric)
    f, used = scale.choose(cands, recon.extents, recon.px_per_unit, recon.metric)
    recon.mesh.apply_scale(f)
    ext = recon.extents
    notes = list(recon.notes)
    if ext.max() > 0.6 or ext.max() < 0.01:               # a robot table holds things between 1 and 60 cm
        g = float(np.clip(ext.max(), 0.01, 0.6) / ext.max())
        recon.mesh.apply_scale(g)
        notes.append(f"size clamped to {np.clip(ext.max(), 0.01, 0.6) * 100:.0f} cm (from {ext.max() * 100:.0f} cm)")
        ext = recon.extents
    recon.scale_m *= f

    job.at("physics")
    mass, mass_src = _mass_hint(hints, desc)
    prep = physical.prepare(recon.mesh, recon.colors, desc["material"], desc["hollow"], mass)
    p = prep["props"]
    conf = float(min(recon.confidence, used.get("confidence", 0.3) + 0.05))
    summary = {
        "label": label, "name": desc["name"], "kind": desc["kind"], "material": desc["material"], "hollow": desc["hollow"],
        "recognized_by": desc["source"], "model": desc.get("model"),
        "provider": {"id": prov.name, "label": prov.label, "license": recon.license},
        "shape": recon.shape, "size_cm": [round(float(x) * 100, 1) for x in prep["visual"].extents],
        "mass_g": round(p["mass"] * 1000), "mass_source": mass_src or p["source"], "friction": p["friction"],
        "scale": {"source": used.get("source"), "label": used.get("label"), "confidence": used.get("confidence"), "factor": f},
        "scales": [{"source": c.get("source"), "label": c.get("label"),
                    "value_cm": round(c["m"] * 100, 1) if "m" in c else None, "dim": c.get("dim")} for c in cands[:5]],
        "confidence": round(conf, 2), "views": len(results), "views_agree": None if agree is None else round(agree, 2),
        "mask": {"source": msrc, "rect": [int(x) for x in rect], "w": int(im.shape[1]), "h": int(im.shape[0])},
        "depth": "metric" if had_metric else ("relative" if "relative" in " ".join(recon.notes) else "none"),
        "faces": int(len(prep["visual"].faces)), "hulls": len(prep["hulls"]), "collision": prep["method"],
        "watertight": prep["watertight"], "notes": notes + prep["notes"] + ([rec_note] if rec_note else []),
        "seconds": round(time.time() - t0, 1),
    }
    marker = next((c.get("marker") for c in cands if c.get("source") == "aruco"), None)
    if marker:
        summary["marker"] = marker
    job.summary = summary
    job.data.update(prep=prep, recon=recon, photo=crop_jpg(im, mask), overlay=overlay_jpg(im, mask),
                    mesh_bin=mesh_bin(prep["visual"], prep["colors"]))
    job.message = (f"{label.capitalize()}: {summary['size_cm'][2]:.1f} cm tall, {summary['mass_g']} g "
                   f"({used.get('label')}).")
    return summary


def save_to_library(job, name=None):
    """The job's object into library/objects/ (once) -> library id."""
    if job.added and job.added.get("lib"):
        return job.added["lib"]
    s, prep = job.summary, job.data.get("prep")
    if not s or prep is None:
        raise ReconError("That reconstruction isn't ready.")
    meta = {"label": s["label"], "name": name or s["name"], "kind": s["kind"], "material": s["material"], "provider": s["provider"]["id"],
            "provider_label": s["provider"]["label"], "license": s["provider"]["license"], "shape": s["shape"],
            "scale_source": s["scale"]["source"], "scale_label": s["scale"]["label"], "scale_confidence": s["scale"]["confidence"],
            "confidence": s["confidence"], "recognized_by": s["recognized_by"], "notes": s["notes"],
            "rgba": list(job.data["recon"].rgba)}
    m = library.save(prep, meta, job.data.get("photo"))
    job.added = {"lib": m["id"]}
    return m["id"]


def spec_object(lib_id, name=None, height_m=None, pos=None, movable=True):
    """Scene-spec object for a library entry; height_m rescales it to that height."""
    meta = library.load(lib_id)
    if meta is None:
        raise ReconError("That object isn't in the library any more.")
    o = {"name": slug(name or meta.get("name") or meta.get("label")) or "thing", "kind": meta["kind"], "mesh": lib_id,
         "movable": bool(movable)}
    s = 1.0
    if isinstance(height_m, (int, float)) and height_m > 0:
        s = float(height_m) / max(float(meta["size"][2]), 1e-6)
    o["scale"] = round(s, 4)
    if pos is not None:
        o["pos"] = pos
    return o


def boxes_jpg(img, layout, side=900):
    """The table photo with each found object boxed and named (the scan's preview)."""
    out = img.copy()
    t = max(2, int(max(img.shape[:2]) / 400))
    poly = layout["surface"].get("polygon")
    if poly:
        cv2.polylines(out, [np.array(poly, np.int32)], True, (120, 200, 120), t)
    for o in layout["objects"]:
        u0, v0, u1, v1 = o["box"]
        cv2.rectangle(out, (u0, v0), (u1, v1), (214, 105, 82), t)
        cv2.putText(out, o["name"].replace("_", " "), (u0, max(14, v0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45 * t, (214, 105, 82), t)
    s = min(1.0, side / max(out.shape[:2]))
    if s < 1:
        out = cv2.resize(out, (int(out.shape[1] * s), int(out.shape[0] * s)), interpolation=cv2.INTER_AREA)
    return cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 82])[1].tobytes()


def scan_space(job, img, hints=None, depth=None, K=None, gemini_key=None, llm_cfg=None):
    """Scan-my-table job: layout + scene spec (primitives). Real shapes come later (shape_layout) on request."""
    from . import space
    t0 = time.time()
    hints = dict(hints or {})
    layout = space.scan(job, img, hints, depth=depth, K=K, gemini_key=gemini_key, llm_cfg=llm_cfg)
    spec, notes = space.to_spec(layout, hints.get("template"))
    job.skip("shapes")
    pub = space.public(layout)
    n = len(layout["objects"])
    job.summary = {"count": n, "surface": layout["surface"]["kind"], "template": spec["template"], "layout": pub, "spec": spec,
                   "objects_brief": [f"{o['color']} {o['label']}" for o in layout["objects"]][:12],
                   "scale": layout["scale"], "notes": notes + layout.get("notes", []), "seconds": round(time.time() - t0, 1)}
    job.data.update(layout=layout, img=img, depth=depth, overlay=boxes_jpg(img, layout))
    job.message = (f"Found {n} object{'s' if n != 1 else ''} on the {layout['surface']['kind']} (scale from {layout['scale']['source']})."
                   if n else "I couldn't find objects on the table in that photo.")
    return job.summary


def shape_layout(job):
    """Reconstruct every object of a scanned layout (primitive provider on its outline), save them to the library and
    make the spec use the meshes. -> spec."""
    from . import space
    from .providers import get
    layout, img, depth = job.data["layout"], job.data["img"], job.data.get("depth")
    disp, K = layout.get("disp"), layout.get("K")
    lib = {}
    prov = get("primitive")
    for s in job.steps:
        if s["id"] == "shapes":
            s["state"] = "todo"
    for i, (o, raw) in enumerate(zip(layout["objects"], layout["objects_px"])):
        job.at("shapes", i / max(1, len(layout["objects"])), f"Shaping the {o['label']}…")
        mask = raw.get("mask")
        if mask is None or not mask.any():
            mask, _, _ = segment.object_mask(img, raw["box"], disp)
        try:
            r = prov.reconstruct(img, mask, {"kind": o["kind"]}, depth=depth, disp=disp if depth is None else None, K=K)
        except ReconError:
            continue
        r.mesh.apply_scale(o["size_m"][2] / max(float(r.mesh.extents[2]), 1e-9))
        prep = physical.prepare(r.mesh, r.colors, o["material"], recognize.classify(o["label"])[2])
        m = library.save(prep, {"label": o["label"], "name": o["name"], "kind": o["kind"], "material": o["material"], "provider": "primitive",
                                "provider_label": prov.label, "license": r.license, "shape": r.shape, "scale_source": layout["scale"]["source"],
                                "confidence": round(min(r.confidence, layout["scale"]["confidence"] or 0.2), 2), "from": "scan",
                                "rgba": list(r.rgba)}, crop_jpg(img, mask))
        lib[o["name"]] = (m["id"], 1.0)
    spec, _ = space.to_spec(layout, job.summary.get("template"), lib)
    job.summary["spec_real"] = spec
    job.summary["shaped"] = len(lib)
    job.message = f"Shaped {len(lib)} of {len(layout['objects'])} objects from the photo."
    return spec


def describe_for_agent(summary):
    s = summary or {}
    return {k: s.get(k) for k in ("label", "kind", "material", "shape", "size_cm", "mass_g", "confidence")} | {
        "scale_from": (s.get("scale") or {}).get("label"), "provider": (s.get("provider") or {}).get("label")}
