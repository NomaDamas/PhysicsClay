"""The phone domain only opens /phone and a camera-only socket, and only with the pairing code (pps/app.py phone_gate)."""

import asyncio

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from pps.app import phone_gate


async def _ok(request):
    return web.Response(text="ok")


def _app(phone_host="cam.example.com"):
    app = web.Application(middlewares=[phone_gate])
    app["phone_host"], app["pair_token"] = phone_host, "secret-code"
    for path in ("/", "/phone", "/ws", "/phone/info", "/scene.json"):
        app.router.add_get(path, _ok)
    return app


def _get(app, path, headers=None):
    async def go():
        async with TestClient(TestServer(app)) as c:
            r = await c.get(path, headers=headers or {})
            return r.status

    return asyncio.run(go())


PHONE = {"Host": "cam.example.com", "Cf-Connecting-Ip": "203.0.113.5"}


def test_local_requests_are_untouched():
    assert _get(_app(), "/") == 200 and _get(_app(), "/phone/info") == 200


def test_phone_domain_needs_the_pairing_code():
    assert _get(_app(), "/phone", PHONE) == 403
    assert _get(_app(), "/phone?t=wrong", PHONE) == 403
    assert _get(_app(), "/phone?t=secret-code", PHONE) == 200
    assert _get(_app(), "/ws?t=secret-code", PHONE) == 200


def test_phone_domain_cannot_open_the_app_or_anything_else():
    for path in ("/", "/phone/info?t=secret-code", "/scene.json?t=secret-code"):
        assert _get(_app(), path, PHONE) == 403


def test_without_a_phone_domain_the_gate_is_off():
    assert _get(_app(phone_host=None), "/", PHONE) == 200
