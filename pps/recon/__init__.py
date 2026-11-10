"""Real photos -> simulated objects and spaces (docs/RECON.md).

    object   photo(s) of a real thing -> recognize (label, material) -> outline -> 3D shape (a provider) -> metric scale ->
             physics-ready (watertight, light visual mesh, convex pieces, mass from volume x density, friction from the
             material) -> library/objects/<id>/ -> a free body in the scene the robots can grasp
    space    photo of a table -> the surface and the objects on it (where, how big) -> a `world set` scene spec

Modules: fit (camera, plane, primitive fits, meshes) · depth (Depth Anything V2 Small, ONNX) · segment (outline) ·
scale (what you said, ArUco / card / A4, VLM, priors) · recognize (VLM or detector; materials) · physical (repair,
decimate, convex decomposition, inertia) · providers (primitive, triposr, fal_trellis, meshy) · pipeline (one import)
· space (scan my table) · library (local object library) · sim (scene hooks) · jobs (background jobs) · http (routes).

Provider interface: `provider.reconstruct(img, mask, hints, ...) -> Recon(mesh, colors?, scale_m, confidence,
provider, license, ...)`; `reconstruct()` below is the one-call form."""
from .base import Recon, ReconError


def reconstruct(images, mask=None, hints=None, provider="primitive", depth=None, K=None, keys=None):
    """Photos (BGR arrays) -> Recon of the first one, in the provider's units (pps/recon/pipeline.py adds the scale).
    mask: bool array for the first photo (None: found automatically); depth: metric depth (m) for it, if any."""
    from . import depth as depthmod
    from . import segment
    from .providers import get
    img = images[0] if isinstance(images, (list, tuple)) else images
    disp = None
    if depth is None and depthmod.available():
        disp = depthmod.estimate(img)
    if mask is None:
        mask, _, _ = segment.object_mask(img, (hints or {}).get("crop"), disp if disp is not None else depth)
    p = get(provider)
    if p.name == "primitive":
        return p.reconstruct(img, mask, hints, depth=depth, disp=disp, K=K)
    return p.reconstruct(img, mask, hints, keys=keys)


__all__ = ["Recon", "ReconError", "reconstruct"]
