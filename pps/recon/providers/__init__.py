"""Reconstruction providers behind one interface:

    provider.reconstruct(img, mask, hints, ...) -> pps.recon.base.Recon      provider.available(keys) -> (ok, why)

primitive (default, offline) · triposr (opt-in local, MIT) · fal_trellis, meshy (cloud, bring your own key)."""
from .cloud import FalTrellis, Meshy
from .primitive import Primitive
from .triposr import TripoSR

PROVIDERS = {p.name: p for p in (Primitive(), TripoSR(), FalTrellis(), Meshy())}


def get(name):
    return PROVIDERS.get(name) or PROVIDERS["primitive"]


def listing(keys=None):
    out = []
    for p in PROVIDERS.values():
        ok, why = p.available(keys)
        out.append({"id": p.name, "label": p.label, "license": p.license, "offline": p.offline, "key": p.key, "ok": ok,
                    "why": why, "note": p.note})
    return out
