"""Template for a feature's end-to-end checks (files starting with '_' are skipped). Copy to scripts/checks/<feature>.py.

async def run(t):
    await t.send(type="zone", op="add", rect=[0.4, -0.1, 0.5, 0.1])
    await t.pump(0.5)
    t.check("zone: added", len(t.state().get("zones") or []) == 1)
"""
