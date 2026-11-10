"""`primitive`: the default, offline reconstruction. Depth (metric, or Depth Anything's relative depth) + mask -> 3D
points standing on the fitted support plane -> the best of box / cylinder / lathe / mirrored hull. Without depth the
outline alone becomes a lathe or a box. Always works; the shapes are simple on purpose (they grasp and stack well)."""
import cv2
import numpy as np

from .. import fit
from ..base import Recon, ReconError

LICENSE = "Apache-2.0 (PhysicsClay primitive fit; depth by Depth Anything V2 Small, Apache-2.0)"


class Primitive:
    name = "primitive"
    label = "Quick shape (offline)"
    license = LICENSE
    offline = True
    key = None
    note = "Box, cylinder, turned profile or hull fitted to the photo. Seconds, no network."

    def available(self, keys=None):
        return True, None

    def reconstruct(self, img, mask, hints=None, depth=None, disp=None, K=None, progress=None):
        """img BGR, mask bool. depth: metric depth (m) or None; disp: relative inverse depth or None."""
        hints = hints or {}
        if mask is None or mask.sum() < 50:
            raise ReconError("I couldn't find the object's outline in that photo. Drag a box around it and try again.")
        h, w = img.shape[:2]
        K = fit.intrinsics(w, h, K)
        kind = hints.get("kind")
        rgb = fit.mask_color(img, mask)
        force, how = None, "outline only"
        if depth is not None:
            r = self._from_depth(img, mask, np.asarray(depth, float), K, kind, rgb, True)
            if r is not None:
                r.notes.insert(0, "shape from metric depth")
                return r
            how = "outline only (the depth didn't give a usable shape)"
        elif disp is not None:
            # relative depth bends the depth axis (and with it heights seen from above): it picks the kind of shape,
            # the outline gives the proportions (measured: aspect error 39 % from relative depth, 5 % from the outline)
            r = self._from_depth(img, mask, fit.depth_from_disparity(disp, mask), K, kind, rgb, False)
            if r is not None and kind not in fit.ROUND_KINDS + fit.BOX_KINDS:
                force = "box" if r.shape in ("box", "hull") else "lathe"
            how = "the outline, shape class from relative depth (Depth Anything V2 Small)" if r is not None else how
        out = fit.silhouette_shape(mask, kind, force)
        if out is None:
            raise ReconError("The object's outline is too small. Move closer or crop tighter.")
        mesh, shape, uv, conf = out
        mesh = fit.densify(mesh)
        cols = fit.sample_colors(uv(np.asarray(mesh.vertices)), img, mask, rgb)
        x0, _, x1, _ = _bbox(mask)
        return Recon(mesh=mesh, provider=self.name, shape=shape, colors=cols, rgba=tuple(rgb) + (1.0,), metric=False,
                     px_per_unit=1.0, confidence=conf * (0.9 if force else 0.8), license=LICENSE, notes=[f"shape from {how}"],
                     debug={"mask_px": [x0, x1], "relative_depth": disp is not None})

    # ------------------------------------------------------------------ depth path
    def _from_depth(self, img, mask, Z, K, kind, rgb, metric):
        h, w = img.shape[:2]
        x0, y0, x1, y1 = _bbox(mask)
        size_px = max(x1 - x0, y1 - y0)
        k = max(2, int(size_px * 0.03))
        er = cv2.erode(mask.astype(np.uint8), np.ones((k, k), np.uint8)).astype(bool)
        if er.sum() < 30:
            er = mask
        stride = max(1, int(np.sqrt(er.sum() / 12000)))
        P, _ = fit.backproject(Z, K, er, stride)
        if len(P) < 30:
            return None
        d1 = cv2.dilate(mask.astype(np.uint8), np.ones((2 * k + 1, 2 * k + 1), np.uint8))
        d2 = cv2.dilate(mask.astype(np.uint8), np.ones((int(size_px * 0.5) | 1, int(size_px * 0.5) | 1), np.uint8))
        ring = (d2 > 0) & ~(d1 > 0)
        P_bg, _ = fit.backproject(Z, K, ring, max(1, int(np.sqrt(ring.sum() / 8000))))
        zmed = float(np.median(P[:, 2]))
        plane = fit.fit_plane(P_bg, thresh=0.012 * zmed) if len(P_bg) > 60 else None
        up_ok = plane is not None and plane[2].mean() > 0.45 and float(plane[0] @ np.array([0.0, -1.0, 0.0])) > 0.3
        if up_ok:
            n, dd, _ = plane
            R, c0 = fit.object_frame(n, dd, P)
            Q = (P - c0) @ R.T
            support = "the table"
        else:                                              # held in a hand / nothing under it: image up is up
            R = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])
            Q0 = P @ R.T
            c0 = R.T @ np.array([Q0[:, 0].mean(), Q0[:, 1].mean(), Q0[:, 2].min()])
            Q = (P - c0) @ R.T
            support = None
        H = float(np.percentile(Q[:, 2], 98))
        if H <= 0:
            return None
        Q = Q[Q[:, 2] > 0.03 * H]
        if len(Q) < 30:
            return None
        shape, prm, conf = fit.choose_shape(Q, kind)
        if not metric and shape in ("cylinder", "lathe"):  # relative depth bends the depth axis: radii from the width
            prm = fit.lateral_radii(Q, prm)
        if shape == "box":
            mesh = fit.box_mesh(prm["dims"])
            c, s = np.cos(prm["yaw"]), np.sin(prm["yaw"])
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
            off, centre_y = np.array([*prm["center"], H / 2]), prm["center"][1]
        elif shape in ("cylinder", "lathe"):
            mesh = fit.cylinder_mesh(prm["r"], prm["h"]) if shape == "cylinder" else fit.lathe_mesh(prm["heights"], prm["radii"], prm["h"])
            Rz, off, centre_y = np.eye(3), np.array([*prm["center"], prm["h"] / 2]), prm["center"][1]
        else:                                              # mirrored hull: the far side mirrors the near side
            yc = float(np.percentile(Q[:, 1], 90))
            back = Q.copy()
            back[:, 1] = 2 * yc - back[:, 1]
            base = np.vstack([Q, back])
            base0 = base.copy()
            base0[:, 2] = 0.0
            pts = np.vstack([base, base0])
            lo, hi = pts.min(0), pts.max(0)
            mesh = fit.hull_mesh(pts)
            Rz, off, centre_y = np.eye(3), (lo + hi) / 2, yc
        mesh = fit.densify(mesh)
        V = np.asarray(mesh.vertices) @ Rz.T + off          # canonical -> object frame
        Vm = V.copy()
        far = Vm[:, 1] > centre_y
        Vm[far, 1] = 2 * centre_y - Vm[far, 1]              # the hidden back takes the front's colours
        uv = fit.project(Vm @ R + c0, K)
        cols = fit.sample_colors(uv, img, mask, rgb)
        ppu = (x1 - x0) / max(float(mesh.extents[0]), 1e-9) if shape != "box" else None
        if shape == "box":                                 # a rotated box: pixels across vs. its projected width
            ppu = (x1 - x0) / max(float(np.ptp(V[:, 0])), 1e-9)
        notes = [] if support else ["no table under it: up is the photo's up"]
        return Recon(mesh=mesh, provider=self.name, shape=shape, colors=cols, rgba=tuple(rgb) + (1.0,), metric=metric,
                     px_per_unit=float(ppu), confidence=float(conf * (1.0 if metric else 0.85)), license=LICENSE, notes=notes,
                     debug={"support": support, "points": int(len(Q)), "fit": {k: v for k, v in prm.items() if k in ("dims", "r", "h", "residual")}
                            if isinstance(prm, dict) and shape != "hull" else None})


def _bbox(mask):
    ys, xs = np.nonzero(mask)
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1
