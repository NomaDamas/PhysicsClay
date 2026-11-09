"""Point tool: a 3D click on the table drops a marker that 'there' resolves to."""


async def run(t):
    before = len(t.state().get("markers") or [])
    await t.send(type="point3d", pos=[0.40, 0.15, 0.74])
    await t.pump(1.0)
    mk = t.state().get("markers") or []
    t.check("point: 3D click drops a marker", len(mk) == before + 1 and abs(mk[-1]["pos"][1] - 0.15) < 0.01, str(mk[-1:] if mk else mk))
