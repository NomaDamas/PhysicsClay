"""pps/diskguard.py: visitors can't fill the server's disk (free-space floor always, folder caps with --byok)."""

from pps import diskguard


def test_low_free_space_refuses(tmp_path, monkeypatch):
    monkeypatch.setenv("PC_MIN_FREE_GB", "1e9")  # more than any disk has
    ok, why = diskguard.check(tmp_path, "datasets")
    assert not ok and "nearly full" in why and "dataset recordings" in why


def test_enough_space_allows(tmp_path, monkeypatch):
    monkeypatch.setenv("PC_MIN_FREE_GB", "0")
    monkeypatch.delenv("PC_BYOK", raising=False)
    assert diskguard.check(tmp_path / "missing" / "library", "library") == (True, "")


def test_public_cap(tmp_path, monkeypatch):
    monkeypatch.setenv("PC_MIN_FREE_GB", "0")
    monkeypatch.setenv("PC_BYOK", "1")
    monkeypatch.setenv("PC_PUBLIC_LIBRARY_MB", "0.001")  # 1 kB
    (tmp_path / "obj.bin").write_bytes(b"x" * 4000)
    ok, why = diskguard.check(tmp_path, "library")
    assert not ok and "public server" in why and "imported objects" in why
    monkeypatch.delenv("PC_BYOK")  # local use: no cap
    assert diskguard.check(tmp_path, "library")[0]


def test_korean_catalog_has_the_messages():
    from pps import i18n

    for text in (
        "The server's disk is nearly full (2.5 GB free), so I won't save more dataset recordings right now.",
        "This public server keeps at most 100 MB of imported objects, and it's full. Run PhysicsClay locally to keep more.",
    ):
        ko = i18n.tr(text, "ko")
        assert ko != text
