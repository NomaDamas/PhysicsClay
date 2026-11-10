"""HTTP side of photo import (registered by pps/app.py run_server):

    POST /recon/import          multipart: photo (1-4, JPEG/PNG/WebP), depth? (16-bit PNG mm / .npy m, for photo 1),
                                meta (JSON: {provider, hints: {label, size, material, crop [u0,v0,u1,v1] 0-1, reference,
                                mass_kg}, K?: [fx, fy, cx, cy], tag?}) -> {ok, job}
    POST /recon/scan            multipart: photo, depth?, meta ({hints: {surface?, template?}, K?, tag?}) -> {ok, job}
    GET  /recon/job/<id>        the job in full (steps, summary, layout)
    GET  /recon/job/<id>/mesh.bin     preview mesh: [nv, nf] u32, verts f32, faces u32, RGB u8 per vertex
    GET  /recon/job/<id>/overlay.jpg  the photo with the outline (object) or the boxes (table)
    GET  /recon/colors/<lib>.bin      RGB u8 per visual vertex of a library object (the 3D stage)
    GET  /recon/library.json · /recon/library/<id>/photo.jpg · /recon/marker.png (print-at-home ArUco, 5 cm)

Keys (bring your own): X-Gemini-Key, X-Fal-Key, X-Meshy-Key headers, used for that one job and never stored or logged;
the server's own keys (.env) only outside --byok."""
import json

import cv2
from aiohttp import web

from . import jobs, library, pipeline, scale
from .base import ReconError

MAX_PHOTOS, MAX_BYTES = 4, 25 << 20


def server_keys(headers=None):
    """-> (keys for providers, gemini key, llm cfg): the visitor's headers first, then the server's (never in --byok)."""
    from ..agent import server_llm
    from ..spatial import gemini_key
    h = headers or {}
    keys = {"fal": (h.get("X-Fal-Key") or "").strip() or None, "meshy": (h.get("X-Meshy-Key") or "").strip() or None}
    g = (h.get("X-Gemini-Key") or "").strip() or gemini_key()
    return keys, g, server_llm()


def start_import(images, meta, keys=None, gemini=None, llm=None, depths=None, auto=None):
    meta = meta if isinstance(meta, dict) else {}
    j = jobs.new("object", tag=meta.get("tag"), auto=auto)
    hints = meta.get("hints") if isinstance(meta.get("hints"), dict) else {}
    j.data["request"] = {"provider": meta.get("provider") or "primitive", "hints": {k: v for k, v in hints.items() if k != "crop"}}
    jobs.start(j, lambda job: pipeline.import_object(job, images, hints, meta.get("provider") or "primitive", keys, depths,
                                                     meta.get("K"), gemini, llm))
    return j


def start_scan(img, meta, gemini=None, llm=None, depth=None, auto=None):
    meta = meta if isinstance(meta, dict) else {}
    j = jobs.new("space", tag=meta.get("tag"), auto=auto)
    hints = meta.get("hints") if isinstance(meta.get("hints"), dict) else {}
    jobs.start(j, lambda job: pipeline.scan_space(job, img, hints, depth, meta.get("K"), gemini, llm))
    return j


async def _read(request):
    reader = await request.multipart()
    photos, depth, meta = [], None, {}
    while True:
        part = await reader.next()
        if part is None:
            break
        if part.name in ("photo", "depth"):
            data = bytearray()
            while True:
                chunk = await part.read_chunk(1 << 20)
                if not chunk:
                    break
                data += chunk
                if len(data) > MAX_BYTES:
                    raise ReconError("That file is too large (25 MB max).")
            if part.name == "photo" and len(photos) < MAX_PHOTOS:
                photos.append(bytes(data))
            elif part.name == "depth":
                depth = bytes(data)
        elif part.name == "meta":
            try:
                meta = json.loads(await part.text())
            except ValueError:
                meta = {}
    if not photos:
        raise ReconError("No photo in the upload.")
    return photos, depth, meta


async def import_handler(request):
    try:
        photos, depth, meta = await _read(request)
        imgs = [pipeline.decode_image(p) for p in photos]
        depths = [pipeline.decode_depth(depth, imgs[0].shape)] if depth else None
    except ReconError as e:
        return web.json_response({"ok": False, "message": str(e)}, status=400)
    keys, g, llm = server_keys(request.headers)
    j = start_import(imgs, meta, keys, g, llm, depths)
    return web.json_response({"ok": True, "job": j.id})


async def scan_handler(request):
    try:
        photos, depth, meta = await _read(request)
        img = pipeline.decode_image(photos[0])
        d = pipeline.decode_depth(depth, img.shape) if depth else None
    except ReconError as e:
        return web.json_response({"ok": False, "message": str(e)}, status=400)
    _, g, llm = server_keys(request.headers)
    j = start_scan(img, meta, g, llm, d)
    return web.json_response({"ok": True, "job": j.id})


def _job(request):
    j = jobs.get(request.match_info["id"])
    if j is None:
        raise web.HTTPNotFound()
    return j


async def job_handler(request):
    return web.json_response(_job(request).public(full=True))


async def job_mesh(request):
    b = _job(request).data.get("mesh_bin")
    if not b:
        raise web.HTTPNotFound()
    resp = web.Response(body=b, content_type="application/octet-stream", headers={"Cache-Control": "no-store"})
    resp.enable_compression()
    return resp


async def job_overlay(request):
    b = _job(request).data.get("overlay")
    if not b:
        raise web.HTTPNotFound()
    return web.Response(body=b, content_type="image/jpeg", headers={"Cache-Control": "no-store"})


async def colors_handler(request):
    c = library.colors(request.match_info["lib"])
    if c is None:
        raise web.HTTPNotFound()
    return web.Response(body=c, content_type="application/octet-stream", headers={"Cache-Control": "max-age=3600"})


async def library_handler(request):
    return web.json_response({"objects": library.listing()})


async def info_handler(request):
    """Providers (with licenses and what each needs), the depth model's status, the library."""
    from . import depth
    from .providers import listing
    keys, _, _ = server_keys(request.headers)
    return web.json_response({"providers": listing(keys), "depth": depth.status(), "library": library.listing(),
                              "marker": {"url": "/recon/marker.png", "size_cm": scale.MARKER_M * 100, "dictionary": "DICT_4X4_50", "id": 0}})


async def library_photo(request):
    try:
        f = library.path(request.match_info["lib"]) / "photo.jpg"
    except ValueError:
        raise web.HTTPNotFound() from None
    if not f.exists():
        raise web.HTTPNotFound()
    return web.FileResponse(f, headers={"Cache-Control": "max-age=3600"})


async def marker_handler(request):
    png = cv2.imencode(".png", scale.marker_image(0, 400))[1].tobytes()
    return web.Response(body=png, content_type="image/png", headers={"Cache-Control": "max-age=86400"})


def add_routes(app):
    r = app.router
    r.add_post("/recon/import", import_handler)
    r.add_post("/recon/scan", scan_handler)
    r.add_get("/recon/job/{id}", job_handler)
    r.add_get("/recon/job/{id}/mesh.bin", job_mesh)
    r.add_get("/recon/job/{id}/overlay.jpg", job_overlay)
    r.add_get("/recon/colors/{lib}.bin", colors_handler)
    r.add_get("/recon/library.json", library_handler)
    r.add_get("/recon/info.json", info_handler)
    r.add_get("/recon/library/{lib}/photo.jpg", library_photo)
    r.add_get("/recon/marker.png", marker_handler)
