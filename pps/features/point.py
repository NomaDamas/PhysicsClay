"""Point (UX 2.0): drop a marker by clicking the table in the 3D stage, the 3D twin of tapping the robot's view.
{type:"point3d", pos:[x,y,z]} -> marker A-F at that table point, so "put the mug there" / "on A" resolves to it."""
import numpy as np

from .. import ext


@ext.handler("point3d")
def on_point3d(ctx, msg):
    pos = msg.get("pos")
    if not (isinstance(pos, (list, tuple)) and len(pos) == 3):
        return
    p = np.asarray([float(v) for v in pos])
    mid = ctx.tasks.add_marker(p)
    ctx.rt.path_msg = f"Marker {mid} placed. Say 'put the mug there' or 'on {mid}'."
