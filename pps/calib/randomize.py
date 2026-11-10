"""Domain randomization over the calibration parameters (docs/calibration.md) - the one module P1 dataset generation and
P6 calibration share.

    from pps.calib.randomize import Randomizer
    rnd = Randomizer(seed=7)                         # the default ranges (pps/calib/params.py), per robot if given
    p = rnd.sample()                                 # {"friction_scale": 0.83, "mass_scale": 1.41, ...}
    ws.send(rnd.message(p))                          # {"type": "physics", "op": "set", "params": p}  (scripts/generate.py)
    rnd.apply(plant, p)                              # or straight onto a headless plant

Ranges: each parameter is drawn uniformly in its search space (log-uniform for the scale parameters, uniform for
latency_ms) between `lo` and `hi`. Narrow them around a calibrated site profile with `around(params, rel)` (a
site-specific randomization: train on "this home ± 20 %", not on every home). Sampling is seeded and reproducible:
the same seed and the same calls give the same sequence.

Lighting and textures belong to rendering (P1): register callbacks with `on("lighting", fn)` / `on("texture", fn)`;
`sample()` then also draws `{"lighting": {...}, "texture": {...}}` from the same generator and `apply()` calls them.
"""
import math

import numpy as np

from . import params as P

LIGHTING = {"intensity": (0.6, 1.4), "azimuth_deg": (0.0, 360.0), "elevation_deg": (25.0, 80.0)}
TEXTURE = {"table_hue": (0.0, 1.0), "table_value": (0.35, 0.9)}


class Randomizer:
    def __init__(self, names=None, robot=None, seed=0, ranges=None):
        """names: parameters to randomize (default: the core five). ranges: {name: (lo, hi)} overrides."""
        self.specs = {s.name: s for s in P.space(names, robot)}
        for n, (lo, hi) in (ranges or {}).items():
            if n in self.specs:
                self.specs[n] = P.Spec(n, self.specs[n].default, float(lo), float(hi), self.specs[n].log, self.specs[n].unit, self.specs[n].what)
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.hooks = {}

    # ---------------------------------------------------------------- ranges
    def ranges(self):
        """{name: {"lo", "hi", "log", "unit"}}: the interface P1 reads to describe a generated dataset."""
        return {n: {"lo": s.lo, "hi": s.hi, "log": s.log, "unit": s.unit} for n, s in self.specs.items()}

    def around(self, center, rel=0.2, abs_ms=10.0):
        """A randomizer around a calibrated profile: scale parameters within ×(1 ± rel), latency within ± abs_ms."""
        r = Randomizer(list(self.specs), seed=self.seed)
        for n, s in self.specs.items():
            c = float(center.get(n, s.default))
            lo, hi = (max(0.0, c - abs_ms), c + abs_ms) if not s.log else (c * (1 - rel), c * (1 + rel))
            r.specs[n] = P.Spec(n, c, max(lo, 1e-3) if s.log else lo, hi, s.log, s.unit, s.what)
        r.hooks = dict(self.hooks)
        return r

    # ---------------------------------------------------------------- sampling
    def on(self, kind, fn):
        """Register a rendering hook: kind "lighting" or "texture", fn(target, values) applies a draw."""
        if kind not in ("lighting", "texture"):
            raise ValueError("hooks: lighting or texture")
        self.hooks[kind] = fn
        return self

    def _draw(self, lo, hi, log):
        u = float(self.rng.random())
        return math.exp(math.log(lo) + u * (math.log(hi) - math.log(lo))) if log else lo + u * (hi - lo)

    def sample(self):
        p = {n: round(self._draw(s.lo, s.hi, s.log), 6) for n, s in self.specs.items()}
        if "lighting" in self.hooks:
            p["lighting"] = {k: round(self._draw(lo, hi, False), 4) for k, (lo, hi) in LIGHTING.items()}
        if "texture" in self.hooks:
            p["texture"] = {k: round(self._draw(lo, hi, False), 4) for k, (lo, hi) in TEXTURE.items()}
        return p

    def sample_n(self, n):
        return [self.sample() for _ in range(int(n))]

    # ---------------------------------------------------------------- applying
    @staticmethod
    def physics(p):
        return {k: v for k, v in p.items() if k in P.PARAMS}

    def message(self, p, tid=None):
        """The browser / generator message for a draw: {type: physics, op: set, params}."""
        return P.physics_message(self.physics(p), tid)

    def apply(self, plant, p, render_target=None):
        """Apply a draw to a live plant (and the rendering hooks to `render_target`, e.g. a MjModel or renderer)."""
        out = P.apply(plant, self.physics(p))
        for kind in ("lighting", "texture"):
            if kind in p and kind in self.hooks:
                self.hooks[kind](render_target, p[kind])
        return out

    def contains(self, p):
        """Is a draw inside the ranges? (tests, and P1's dataset card)"""
        return all(s.lo - 1e-9 <= p[n] <= s.hi + 1e-9 for n, s in self.specs.items() if n in p)
