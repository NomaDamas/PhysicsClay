"""Server messages in the person's language (pps/i18n.py): Settings → Language sends {type:"settings", lang}; the
state frame's messages come back in Korean, and switching back gives English again (the rest of the self test runs
in English: selftest.py says hello with lang "en")."""
import re

HANGUL = re.compile(r"[가-힣]")


async def run(t):
    try:
        await t.send(type="settings", lang="ko-KR")
        await t.pump(0.6)
        t.check("i18n: Settings → 한국어 sets state.lang", t.state().get("lang") == "ko", str(t.state().get("lang")))
        task = await t.task("pick_place", 4, object="unicorn", target="center")
        msg = task.get("message") or ""
        t.check("i18n: a task verdict comes back in Korean", task.get("status") == "failed" and bool(HANGUL.search(msg)),
                msg[:90])
        t.check("i18n: the object name the person said is kept", "unicorn" in msg, msg[:90])
    finally:
        await t.send(type="settings", lang="en")
        await t.pump(0.6)
    task = await t.task("pick_place", 4, object="unicorn", target="center")
    msg = task.get("message") or ""
    t.check("i18n: English again after switching back", t.state().get("lang") == "en" and not HANGUL.search(msg)
            and "unicorn" in msg, msg[:90])
