"""Offscreen dataset cameras (256x256 by default), rendered without touching the model:

    front   a fixed third-person camera across the work surface, facing the robot (sees the robot, the objects and
            the surface; placed from the surface extent, so it works for every template and body)
    wrist   a camera riding on the first end effector (right hand / the only gripper), computed every frame from the
            end-effector site: beside the wrist on the robot's side, looking along the approach axis past the fingers
    robot   (optional) the scene's `robot_view` camera, the robot's own head / overhead view that the UI shows

Both are drawn by overriding the OpenGL camera of an mjvScene after update_scene (no MjSpec change, so every robot
profile gets a wrist camera and P0's model rewrite cannot break it). Images are clean: the IK target ghosts are hidden,
the runtime's debug colouring (red links at a limit, orange collisions) is undone for the render, sites are off and
there are no overlays. Randomization (lighting, camera jitter) is applied only for the dataset render and restored.
"""
import mujoco
import numpy as np

WRIST_BACK, WRIST_SIDE, WRIST_LOOK, WRIST_FOVY = 0.03, 0.09, 0.16, 80.0
FRONT_FOVY = 58.0


def _unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def _look(pos, fwd, up_hint):
    fwd = _unit(fwd)
    up = np.asarray(up_hint, float) - np.dot(up_hint, fwd) * fwd
    if np.linalg.norm(up) < 1e-6:
        up = np.array([0.0, 0.0, 1.0]) - fwd[2] * fwd
        if np.linalg.norm(up) < 1e-6:
            up = np.array([1.0, 0.0, 0.0])
    return np.asarray(pos, float), fwd, _unit(up)


def _rot(axis, deg):
    axis = _unit(axis)
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, axis, np.radians(deg))
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    return R.reshape(3, 3)


class DatasetCameras:
    def __init__(self, model, rt, surface=None, size=256, shadows=False, names=("front", "wrist")):
        self.model, self.size, self.shadows = model, int(size), bool(shadows)
        self.names = list(names)
        self.renderer = mujoco.Renderer(model, self.size, self.size)
        self.opt = mujoco.MjvOption()
        self.opt.sitegroup[:] = 0
        m = model
        try:
            self.robot_cam = m.camera("robot_view").id
        except KeyError:
            self.robot_cam = -1
        s = surface or {"top": 0.74, "x": (0.22, 0.82), "y": (-0.5, 0.5)}
        top, (x0, x1) = float(s["top"]), s["x"]
        # across the surface, facing the robot, high enough to see over objects onto the robot's hands; a counter's
        # backsplash or a shelf's back panel would block that, so there the camera hangs just in front of the wall
        # (under the shelf's upper board) and looks down at the work area
        names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in range(m.ngeom)}
        look = np.array([(x0 + x1) / 2 - 0.12, 0.0, top + 0.12])
        self.front_fovy = FRONT_FOVY
        if "shelf_back" in names:
            pos, look = np.array([x1 - 0.02, 0.0, top + 0.36]), np.array([x0 + 0.02, 0.0, top + 0.02])
            self.front_fovy = 80.0
        elif "backsplash" in names:
            pos, look = np.array([x1 - 0.03, 0.0, top + 0.62]), np.array([x0 + 0.05, 0.0, top + 0.05])
            self.front_fovy = 75.0
        else:
            pos = np.array([x1 + 0.55, 0.0, top + 0.62])
        self.front = _look(pos, look - pos, (0, 0, 1))
        helper = {b for b in range(m.nbody) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").endswith("_target")}
        self.ghosts = np.array([g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in helper], int)
        self.clean_rgba = (getattr(rt, "default_rgba", None) if getattr(rt, "default_rgba", None) is not None
                           else m.geom_rgba).copy()
        if len(self.ghosts):
            self.clean_rgba[self.ghosts, 3] = 0.0
        hands = list(getattr(rt, "hands", []))
        prof = getattr(rt, "profile", None)
        self.ee_axis = np.asarray(getattr(prof, "ee_axis", (0, 0, 1)) or (0, 0, 1), float)
        self.wrist_sid = rt.palm_sid[hands[0]] if hands else -1
        # approach axis: the wrist body -> site offset when the site sits out on the fingers (G1 palm, Panda, ALOHA),
        # else the profile's ee_axis in the site frame
        self.axis_local, self.axis_from_body = self.ee_axis, False
        if self.wrist_sid >= 0:
            off = np.asarray(m.site_pos[self.wrist_sid], float)
            if np.linalg.norm(off) > 0.02:
                self.axis_local, self.axis_from_body = _unit(off), True
        self.hand = hands[0] if hands else None
        self.root_xy = np.zeros(2)
        roots = sorted(getattr(rt, "robot_roots", ()) or ())
        if roots:
            self.root_xy = np.asarray(rt.data.xpos[roots[0]][:2], float).copy()
        self.light0 = (m.light_dir.copy(), m.light_diffuse.copy(), m.vis.headlight.diffuse.copy(), m.vis.headlight.ambient.copy())
        self.rand = {}

    def close(self):
        try:
            self.renderer.close()
        except Exception:
            pass

    # ---------------------------------------------------------------- randomization
    def set_randomization(self, rand):
        """rand: {"light": {"dir": [[x, y, z] per light], "scale": s, "ambient": a}, "camera": {"front": {"dp": [..],
        "rot": [rx, ry]}, "wrist": {...}}} (sampled by recorder.sample_randomization)."""
        self.rand = dict(rand or {})

    def _apply_light(self):
        L = self.rand.get("light")
        if not L:
            return False
        m = self.model
        dirs = L.get("dir") or []
        for i in range(min(m.nlight, len(dirs))):
            if dirs[i] is not None:
                m.light_dir[i] = _unit(dirs[i])
        s = float(L.get("scale", 1.0))
        m.light_diffuse[:] = np.clip(self.light0[1] * s, 0, 1)
        m.vis.headlight.diffuse[:] = np.clip(self.light0[2] * s, 0, 1)
        m.vis.headlight.ambient[:] = np.clip(self.light0[3] * float(L.get("ambient", 1.0)), 0, 1)
        return True

    def _restore_light(self):
        m = self.model
        m.light_dir[:], m.light_diffuse[:] = self.light0[0], self.light0[1]
        m.vis.headlight.diffuse[:], m.vis.headlight.ambient[:] = self.light0[2], self.light0[3]

    # ---------------------------------------------------------------- poses
    def wrist_pose(self, d):
        sid = self.wrist_sid
        p = d.site_xpos[sid].copy()
        if self.axis_from_body:
            R = d.xmat[self.model.site_bodyid[sid]].reshape(3, 3)
        else:
            R = d.site_xmat[sid].reshape(3, 3)
        a = _unit(R @ self.axis_local)                    # approach axis: from the wrist out past the fingers
        # the camera sits beside the wrist, on the side that faces up (or, for a hand pointing down, the side that faces
        # the robot's base), like a wrist camera mounted on top of the hand: the fingers frame the bottom of the image
        up = np.array([0.0, 0.0, 1.0])
        side = up - np.dot(up, a) * a
        if np.linalg.norm(side) < 0.35:
            to_base = np.array([self.root_xy[0] - p[0], self.root_xy[1] - p[1], 0.0])
            side = to_base - np.dot(to_base, a) * a
        if np.linalg.norm(side) < 1e-6:
            side = R[:, 0] - np.dot(R[:, 0], a) * a
        side = _unit(side)
        pos = p - a * WRIST_BACK + side * WRIST_SIDE
        look = p + a * WRIST_LOOK
        return _look(pos, look - pos, side)

    def _set_gl_camera(self, pos, fwd, up, fovy=None):
        scn = self.renderer.scene
        for c in (scn.camera[0], scn.camera[1]):
            c.pos[:] = pos
            c.forward[:] = fwd
            c.up[:] = up
            if fovy is not None:
                h = c.frustum_near * np.tan(np.radians(fovy) / 2)
                c.frustum_bottom, c.frustum_top, c.frustum_center, c.frustum_width = -h, h, 0.0, 0.0

    def _jitter(self, name, pos, fwd, up):
        j = (self.rand.get("camera") or {}).get(name)
        if not j:
            return pos, fwd, up
        pos = pos + np.asarray(j.get("dp", (0, 0, 0)), float)
        rx, ry = (j.get("rot") or (0.0, 0.0))[:2]
        right = _unit(np.cross(fwd, up))
        R = _rot(right, rx) @ _rot(up, ry)
        fwd, up = R @ fwd, R @ up
        return pos, _unit(fwd), _unit(up)

    # ---------------------------------------------------------------- render
    def render(self, d):
        """-> {name: uint8 (size, size, 3)} for the current MjData."""
        m, r = self.model, self.renderer
        saved_rgba = m.geom_rgba.copy()
        m.geom_rgba[:] = self.clean_rgba
        lit = self._apply_light()
        out = {}
        try:
            for name in self.names:
                if name == "front":
                    r.update_scene(d, scene_option=self.opt)
                    pos, fwd, up = self._jitter("front", *self.front)
                    self._set_gl_camera(pos, fwd, up, self.front_fovy)
                elif name == "robot":
                    r.update_scene(d, camera=self.robot_cam, scene_option=self.opt)
                    g = r.scene.camera[0]
                    pos, fwd, up = self._jitter("robot", np.array(g.pos, float), _unit(g.forward), _unit(g.up))
                    self._set_gl_camera(pos, fwd, up)
                elif name == "wrist" and self.wrist_sid >= 0:
                    r.update_scene(d, scene_option=self.opt)
                    pos, fwd, up = self.wrist_pose(d)
                    pos, fwd, up = self._jitter("wrist", pos, fwd, up)
                    self._set_gl_camera(pos, fwd, up, WRIST_FOVY)
                else:
                    continue
                if not self.shadows:
                    r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                    r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                out[name] = r.render().copy()
        finally:
            m.geom_rgba[:] = saved_rgba
            if lit:
                self._restore_light()
        return out
