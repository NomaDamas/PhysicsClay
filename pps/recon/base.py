"""Shared types of the photo -> mesh pipeline (pps/recon/__init__.py has the overview)."""
from dataclasses import dataclass, field

import numpy as np


class ReconError(Exception):
    """A reconstruction step failed with a message a person can act on (shown as is)."""


@dataclass
class Recon:
    """What a provider returns. The mesh is z-up and centred on its bounding box.

    metric       the mesh is already in metres (metric depth, e.g. a depth camera / phone AR, or a marker)
    px_per_unit  image pixels per mesh unit across the object (lets a reference in view, e.g. an ArUco marker, set the
                 metric scale); None when unknown
    colors       per-vertex RGBA uint8 (what the 3D stage shows); None: one colour (rgba)"""
    mesh: object
    provider: str
    shape: str = "mesh"                 # box | cylinder | lathe | hull | mesh
    colors: np.ndarray | None = None
    rgba: tuple = (0.6, 0.6, 0.62, 1.0)
    metric: bool = False
    px_per_unit: float | None = None
    confidence: float = 0.5             # how sure the shape is (0..1)
    scale_m: float = 1.0                # metres per mesh unit applied so far
    license: str = "Apache-2.0 (PhysicsClay)"
    notes: list = field(default_factory=list)
    texture: object = None              # optional texture image (cloud providers); colours are baked from it
    debug: dict = field(default_factory=dict)

    @property
    def extents(self):
        return np.asarray(self.mesh.extents, float)


def slug(s, n=24):
    import re
    return re.sub(r"_+", "_", re.sub(r"[^a-z0-9_]", "_", str(s or "").strip().lower())).strip("_")[:n]
