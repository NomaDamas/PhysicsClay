"""Real photos -> simulated objects and spaces (pps/recon, docs/RECON.md). Photos come over HTTP (/recon/import,
/recon/scan) or, here, from the camera that is streaming (the phone page or the browser camera; the laptop webcam with
--webcam). Reconstruction runs in a background job; adding to the scene goes through the normal world message.

  {type:"recon", op:"import", source:"camera", provider?, hints?: {label, size, material, crop, reference, mass_kg},
   keys?: {gemini, fal, meshy}, tag?, add?: true (add it when ready: the agent)}
  {type:"recon", op:"scan", source:"camera", hints?, keys?, tag?, use?: true (build the scene when ready)}
  {type:"recon", op:"add", job, height_m?, name?, movable?}          the reconstructed object into the scene
  {type:"recon", op:"use_layout", job, shapes?: "primitive" | "real", template?}   the scanned table becomes the scene
  {type:"recon", op:"add_library", id, height_m?, name?} · {op:"delete_library", id}
State: recon = {jobs: [...], providers: [...], depth: {...}, library: [...], reply: {id, ok, op, message}}"""
import time

import cv2
import numpy as np

from .. import ext
from ..recon import depth as depthmod
from ..recon import jobs, library, pipeline
from ..recon.base import ReconError
from ..recon.http import server_keys, start_import, start_scan

KEY = "recon"
_cache = {"t": 0.0, "providers": {}, "library": 0, "depth": "off"}


def _st(ctx):
    s = ctx.store.get(KEY)
    if s is None:
        s = ctx.store[KEY] = {"reply": None, "auto": {}, "n": 0}
    return s


def _reply(ctx, msg, ok, message, **kw):
    s = _st(ctx)
    s["n"] += 1
    s["reply"] = dict({"id": msg.get("id") or f"recon-{s['n']}", "ok": bool(ok), "op": msg.get("op"), "message": message,
                       "t": round(time.time(), 2)}, **kw)
    if not ok and msg.get("id"):
        ctx.tasks.set_task(str(msg["id"]), "recon", "failed", message)


def camera_frame(ctx):
    """The freshest camera picture: a phone / browser camera streaming in (full resolution), else the laptop webcam."""
    shared = getattr(ctx.push, "__self__", None)
    phone = getattr(shared, "phone", None)
    if phone and time.time() - phone[0] < 2.0:
        img = cv2.imdecode(np.frombuffer(phone[1], np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            return img, "phone camera" if getattr(shared, "phone_mobile", False) else "browser camera"
    if getattr(ctx, "source", None) in ("webcam", "phone") and getattr(ctx, "frame", None) is not None:
        return ctx.frame.copy(), "camera"
    return None, None


def _keys(msg):
    k = msg.get("keys") if isinstance(msg.get("keys"), dict) else {}
    headers = {"X-Gemini-Key": k.get("gemini") or "", "X-Fal-Key": k.get("fal") or "", "X-Meshy-Key": k.get("meshy") or ""}
    return server_keys(headers)


def add_to_scene(ctx, job, height_m=None, name=None, movable=True, pos=None):
    """Save the job's object to the library and add it to the current scene (world message)."""
    lib = pipeline.save_to_library(job, name)
    o = pipeline.spec_object(lib, name or job.summary.get("name"), height_m, pos, movable)
    names = {x["name"] for x in (ctx.world.spec.get("objects") or [])} if getattr(ctx, "world", None) else set()
    base, k = o["name"], 2
    while o["name"] in names:
        o["name"], k = f"{base}_{k}", k + 1
    job.added = {"lib": lib, "name": o["name"]}
    job.status = "added"
    ctx.push({"type": "world", "op": "add_object", "object": o, "id": f"recon-add-{job.id}"})
    return o


@ext.handler("recon")
def on_recon(ctx, msg):
    op = msg.get("op")
    s = _st(ctx)
    try:
        if op in ("import", "scan"):
            img, src = camera_frame(ctx)
            if img is None:
                raise ReconError("No camera is streaming. Open the phone camera (Settings › Phone camera) or turn on the "
                                 "browser camera, or upload a photo instead.")
            keys, g, llm = _keys(msg)
            meta = {"provider": msg.get("provider"), "hints": msg.get("hints") or {}, "tag": msg.get("tag") or msg.get("id")}
            if op == "import":
                j = start_import([img], meta, keys, g, llm)
                if msg.get("add"):
                    s["auto"][j.id] = {"op": "add", "height_m": msg.get("height_m"), "name": msg.get("name")}
            else:
                j = start_scan(img, meta, g, llm)
                if msg.get("use"):
                    s["auto"][j.id] = {"op": "use", "shapes": msg.get("shapes") or "primitive"}
            _reply(ctx, msg, True, f"Got a photo from the {src}. Working on it…", job=j.id)
        elif op == "add":
            j = jobs.get(msg.get("job"))
            if j is None or j.kind != "object" or j.summary is None:
                raise ReconError("That reconstruction isn't ready (or is gone). Take the photo again.")
            o = add_to_scene(ctx, j, msg.get("height_m"), msg.get("name"), msg.get("movable", True) is not False)
            _reply(ctx, msg, True, f"Adding the {o['name'].replace('_', ' ')} to the scene.", job=j.id, lib=o["mesh"], name=o["name"])
        elif op == "use_layout":
            j = jobs.get(msg.get("job"))
            if j is None or j.kind != "space" or j.summary is None:
                raise ReconError("That table scan isn't ready (or is gone). Scan again.")
            use_layout(ctx, j, msg.get("shapes") or "primitive", msg.get("template"), msg)
        elif op == "add_library":
            o = pipeline.spec_object(str(msg.get("id") or ""), msg.get("name"), msg.get("height_m"))
            ctx.push({"type": "world", "op": "add_object", "object": o, "id": msg.get("id_task") or f"recon-lib-{o['mesh']}"})
            _reply(ctx, msg, True, f"Adding the {o['name'].replace('_', ' ')} from your library.", lib=o["mesh"])
        elif op == "delete_library":
            ok = library.delete(str(msg.get("id") or ""))
            _cache["t"] = 0.0
            _reply(ctx, msg, ok, "Removed from the library." if ok else "Not in the library.")
        else:
            raise ReconError(f"Unknown recon op '{op}'.")
    except (ReconError, ValueError) as e:
        _reply(ctx, msg, False, str(e))


def use_layout(ctx, j, shapes="primitive", template=None, msg=None):
    msg = msg or {}
    if shapes == "real":
        if j.summary.get("spec_real"):
            spec = j.summary["spec_real"]
        else:                                              # shape the objects in the background, then build
            _st(ctx)["auto"][j.id] = {"op": "use_real", "template": template}
            jobs.start(j, pipeline.shape_layout)
            _reply(ctx, msg, True, "Shaping each object from the photo, then building the scene…", job=j.id)
            return
    else:
        spec = j.summary["spec"]
    if template:
        spec = dict(spec, template=template)
    if not spec.get("objects"):
        raise ReconError("The scan found no objects to place.")
    ctx.push({"type": "world", "op": "set", "scene": spec, "id": f"recon-scan-{j.id}"})
    j.status = "added"
    j.added = {"spec": True}
    _reply(ctx, msg, True, f"Building your {j.summary.get('surface', 'table')} with {len(spec['objects'])} objects.", job=j.id)


@ext.tick
def recon_tick(ctx):
    s = _st(ctx)
    for jid, want in list(s["auto"].items()):
        j = jobs.get(jid)
        if j is None:
            s["auto"].pop(jid, None)
            continue
        if j.status == "failed":
            s["auto"].pop(jid, None)
            _reply(ctx, {"op": want["op"], "id": f"recon-auto-{jid}"}, False, j.error or "The reconstruction failed.", job=jid)
        elif j.status == "ready":
            s["auto"].pop(jid, None)
            try:
                if want["op"] == "add":
                    o = add_to_scene(ctx, j, want.get("height_m"), want.get("name"))
                    _reply(ctx, {"op": "add"}, True, f"Adding the {o['name'].replace('_', ' ')} to the scene.", job=jid, name=o["name"])
                elif want["op"] == "use":
                    use_layout(ctx, j, want.get("shapes") or "primitive", None, {"op": "use_layout"})
                elif want["op"] == "use_real":
                    use_layout(ctx, j, "real", want.get("template"), {"op": "use_layout"})
            except (ReconError, ValueError) as e:
                _reply(ctx, {"op": want["op"]}, False, str(e), job=jid)


@ext.state
def recon_state(ctx):
    """Small on purpose (it rides every frame): job summaries and which providers work. Details: GET /recon/info.json,
    /recon/job/<id>, /recon/library.json."""
    now = time.time()
    if now - _cache["t"] > 5.0:
        from ..recon.providers import listing
        d = depthmod.status()
        _cache.update(t=now, providers={p["id"]: p["ok"] for p in listing()}, library=library.count(),
                      depth="ready" if d["ready"] else ("downloading" if d["downloading"] else "on first use" if d["can_download"] else "off"))
    s = _st(ctx)
    return {KEY: {"jobs": jobs.recent(3), "providers": _cache["providers"], "depth": _cache["depth"],
                  "library": _cache["library"], "reply": s["reply"]}}
