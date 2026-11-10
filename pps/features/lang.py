"""Language of the server's messages (pps/i18n.py).

{type: "hello", lang, client?} when a browser connects and {type: "settings", lang} when the person changes Settings →
Language set the one, global language (a single-user app: the last browser that says so wins; the phone camera page
says nothing). State key: `lang` ("en" | "ko"). The state frame itself is localized in pps/app.py, just before sending."""
from .. import ext, i18n


@ext.handler("hello")
def on_hello(ctx, msg):
    if msg.get("lang"):
        i18n.set_lang(msg.get("lang"))


@ext.handler("settings")
def on_settings(ctx, msg):
    if msg.get("lang"):
        i18n.set_lang(msg.get("lang"))


@ext.state
def lang_state(ctx):
    return {"lang": i18n.get_lang()}
