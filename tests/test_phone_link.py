"""pps/app.py phone_link: the QR code points at an address the phone can really open."""

from pps.app import phone_link


def test_public_domain_behind_https_tunnel():
    r = phone_link("physicsclay.example.com", "https", 8443)
    assert r == {"url": "https://physicsclay.example.com/phone", "lan": False, "self_signed": False}


def test_env_public_url_wins():
    r = phone_link("localhost:8780", "http", 8781, public_url="https://my.tunnel.dev/")
    assert r["url"] == "https://my.tunnel.dev/phone" and not r["lan"]


def test_localhost_uses_the_lan_address_and_https_port():
    r = phone_link("localhost:8780", "http", 8781, ip="192.168.0.6")
    assert r == {"url": "https://192.168.0.6:8781/phone", "lan": True, "self_signed": True}


def test_lan_ip_over_http_keeps_that_ip():
    assert phone_link("192.168.1.20:8780", "http", 8781, ip="10.0.0.2")["url"] == "https://192.168.1.20:8781/phone"


def test_lan_ip_already_on_https_keeps_the_origin():
    r = phone_link("192.168.1.20:8781", "https", 8781)
    assert r["url"] == "https://192.168.1.20:8781/phone" and r["self_signed"]


def test_no_https_means_no_link():
    assert phone_link("localhost:8780", "http", None)["url"] is None


def test_own_phone_domain_with_pairing_code():
    r = phone_link("localhost:8780", "http", 8781, phone_host="cam.example.com", token="abc")
    assert r == {"url": "https://cam.example.com/phone?t=abc", "lan": False, "self_signed": False}
