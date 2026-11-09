"""Template for a feature module (files starting with '_' are not loaded). Copy to pps/features/<feature>.py.

from .. import ext

@ext.handler("hello")
def on_hello(ctx, msg):
    ctx.store["hello"] = msg.get("text")

@ext.state
def hello_state(ctx):
    return {"hello": ctx.store.get("hello")}
"""
