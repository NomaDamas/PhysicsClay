"""The message catalog (pps/i18n.py, pps/locale/ko_*.py): every key has both languages with the same placeholders,
and English messages come back in Korean through the same path the state frames take."""

import re

import pytest

from pps import i18n

SAMPLE = {"obj": "the red cup", "what": "the mug", "n": "3", "cm": "2.5", "x": "1"}


def _sample(names):
    return {n: SAMPLE.get(n, f"V{i}") for i, n in enumerate(sorted(names))}


def test_catalog_has_entries():
    assert len(i18n.CATALOG) > 100


@pytest.mark.parametrize("key", sorted(i18n.CATALOG))
def test_entry_has_both_languages_and_matching_args(key):
    entry = i18n.CATALOG[key]
    assert isinstance(entry, tuple) and len(entry) == 2, key
    en, ko = entry
    assert isinstance(en, str) and en.strip(), key
    assert isinstance(ko, str) and ko.strip(), key
    assert re.search(r"[가-힣]", ko) or not re.search(
        r"[A-Za-z]{3}", re.sub(r"\{[^}]*\}", "", ko)
    ), f"{key}: the Korean text looks untranslated: {ko!r}"
    fe, fk = i18n.fields(en), i18n.fields(ko)
    assert {f for f in fe if not f.startswith("_")} == {
        f for f in fk if not f.startswith("_")
    }, f"{key}: placeholders differ: {sorted(fe)} vs {sorted(fk)}"
    assert not {f for f in fk if f not in fe}, f"{key}: Korean uses a placeholder English doesn't have"
    for f in re.findall(r"\{([^{}]*)\}", ko):
        name, _, mark = f.partition("|")
        assert not mark or mark in i18n._PARTICLES, f"{key}: unknown particle {mark!r}"
    assert "을(를)" not in ko and "이(가)" not in ko, f"{key}: use {{name|을}} instead of a hard-coded 을(를)"
    assert not re.search(r"\{(\w+)\}\{\1\|", ko), f"{key}: {{x|을}} already renders x: don't write {{x}}{{x|을}}"
    vals = _sample(fe)
    assert i18n.t(key, "en", **vals) == en.format(**vals)
    assert i18n.t(key, "ko", **vals)


def test_keys_unique_across_files():
    import importlib
    import pkgutil

    from pps import locale

    seen = {}
    for m in pkgutil.iter_modules(locale.__path__):
        if m.name.startswith("ko_"):
            for k in importlib.import_module(f"pps.locale.{m.name}").MESSAGES:
                assert k not in seen, f"{k} is in both {seen.get(k)} and {m.name}"
                seen[k] = m.name


def test_english_is_unchanged_and_unknown_stays_english():
    msg = "Something nobody wrote a template for, 42 times."
    assert i18n.tr(msg, "en") == msg
    assert i18n.tr(msg, "ko") == msg
    assert i18n.t(next(iter(i18n.CATALOG)), "fr", **_sample(i18n.fields(next(iter(i18n.CATALOG.values()))[0])))


def test_roundtrip_rendered_english_to_korean():
    """A message rendered in English and translated back gives what t() renders in Korean (most keys: some generic
    templates legitimately lose to a more specific one)."""
    ok = total = 0
    for key, (en, _ko) in i18n.CATALOG.items():
        names = i18n.fields(en)
        if any(n.startswith("_") for n in names):
            continue
        vals = _sample(names)
        total += 1
        ok += i18n.tr(en.format(**vals), "ko") == i18n.t(key, "ko", **vals)
    assert ok >= 0.9 * total, f"only {ok}/{total} catalog sentences translate back to their own Korean"


def test_particles():
    assert i18n.particle("컵", "을") == "을"
    assert i18n.particle("블록", "을") == "을"
    assert i18n.particle("머그컵", "이") == "이"
    assert i18n.particle("그릇", "와") == "과"
    assert i18n.particle("바나나", "와") == "와"
    assert i18n.particle("테이블", "으로") == "로"  # ㄹ final
    assert i18n.particle("옆", "으로") == "으로"
    assert i18n.particle("G1", "이") == "이"  # 일
    assert i18n.particle("Panda", "을") == "를"
    assert i18n.particle("H2", "은") == "는"  # 이


def test_words():
    assert i18n.word("the red cup") == "빨간 컵"
    assert i18n.word("right hand") == "오른손"
    assert i18n.word("Spot") == "Spot"  # a robot, not a place
    assert i18n.word("3.5") == "3.5"


def test_language_setting():
    before = i18n.get_lang()
    try:
        assert i18n.set_lang("ko-KR") == "ko"
        assert i18n.set_lang("xx") == "ko"  # unknown: unchanged
        assert i18n.set_lang("en-US") == "en"
    finally:
        i18n.set_lang(before)


def test_localize_is_copy_on_write():
    key, (en, _ko) = next((k, v) for k, v in i18n.CATALOG.items() if not i18n.fields(v[0]))
    task = {"id": "t1", "status": "failed", "message": en}
    state = {
        "task": task,
        "agent": {"events": [{"kind": "user", "text": en}, {"kind": "result", "text": en}]},
        "geom_poses": {"1": [0.0, 1.0]},
        "name": en,
    }
    out = i18n.localize(state, "ko")
    assert state["task"]["message"] == en and task["message"] == en  # never changed in place
    assert out["task"]["message"] == i18n.t(key, "ko")
    assert out["agent"]["events"][0]["text"] == en  # the person's own words stay
    assert out["agent"]["events"][1]["text"] == i18n.t(key, "ko")
    assert out["name"] == en  # not a message field
    assert out["geom_poses"] is state["geom_poses"]
    assert i18n.localize(state, "en") is state
