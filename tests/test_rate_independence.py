"""What must not depend on how fast the server's loop runs: a played person's frames (copresence feed, its own clock)
and the pictures only rendered for clients that show them (pps/app.py STREAMS)."""

import pytest

from pps.features import copresence as c


def _feed(n=40, fps=20.0, t0=100.0, hold=0.0):
    return {"frames": [{"i": k} for k in range(n)], "fps": fps, "t0": t0, "hold": hold, "next": 0}


@pytest.mark.parametrize("tick", [1 / 60, 1 / 20, 1 / 5, 0.45])
def test_every_feed_frame_arrives_once_in_order_on_its_own_clock(tick):
    """A slow loop (5 fps, or a 0.45 s frame) gets several frames a tick, a fast one some empty ticks; the person model
    sees the same 2 s of motion either way, each frame stamped t0 + i / fps."""
    f = _feed()
    got, now = [], f["t0"]
    while now < f["t0"] + 2.5:
        got += c._feed_due(f, now) or []
        now += tick
    assert [fr["i"] for _, fr in got] == list(range(40))
    assert [t for t, _ in got] == pytest.approx([100.0 + k / 20.0 for k in range(40)])


def test_a_feed_holding_its_last_frame_reads_like_a_camera():
    f = _feed(n=4)
    assert len(c._feed_due(f, 100.5)) == 4  # all four played by then
    assert c._feed_due(f, 100.6) is None  # holding: one reading per tick (the caller's own time)


def test_a_long_stall_skips_the_old_frames():
    f = _feed(n=200)
    got = c._feed_due(f, 100.0 + 9.0)
    assert len(got) == 40 and got[-1][1]["i"] == 180  # the last 2 s


def test_pictures_only_for_the_clients_that_show_them():
    from pps.app import STREAMS, Shared

    sh = Shared()
    assert sh.wanted() == set()  # nobody connected: nothing to render
    a, b = object(), object()
    sh.clients |= {a, b}
    assert sh.wanted() == set(STREAMS)  # clients that never said: everything
    sh.want(a, [])
    sh.want(b, ["robot", "nonsense"])
    assert sh.wanted() == {"robot"}
    sh.want(b, [])
    assert sh.wanted() == set()
    sh.clients.discard(a)
    sh.want(b, None)  # not a list: back to everything
    assert sh.wanted() == set(STREAMS)
