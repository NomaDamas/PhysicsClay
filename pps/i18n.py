"""The language of the server's messages: English or 한국어.

PhysicsClay is a single-user app (one browser, maybe a phone streaming its camera), so the language is ONE global
setting. The browser sends `{type: "hello", lang}` when it connects and `{type: "settings", lang}` when the person
changes Settings → Language (System / 한국어 / English; System is `navigator.language`). The last one wins;
`state.lang` reports it. The default is English, and the self test says `lang: "en"` so its checks keep matching
English text.

Server code writes its messages in English: the English text is the message id (like gettext's msgid). The catalog
(`pps/locale/ko_*.py`, `MESSAGES = {key: (English template, Korean template)}`) maps them to Korean:

* `t(key, lang=None, **fmt)` renders one catalog entry (unknown language → English);
* `tr(text, lang=None)` translates an already-rendered English message: the whole text, else sentence by sentence,
  by matching the catalog's English templates (a `{name}` matches any text). A sentence nothing matches stays English;
* `localize(state, lang=None)` runs `tr` over the message fields of a state frame, copy on write, right before the
  frame is sent (pps/app.py), so no call site has to change and the agent keeps reading the English frame.

Korean templates may put a particle after a placeholder: `{obj|을}` → 을/를, `{obj|이}` → 이/가, `{obj|은}` → 은/는,
`{obj|와}` → 와/과, `{obj|으로}` → 으로/로, chosen from the value's last sound (Hangul, digits, English words).
Placeholders starting with `_` are English-only (a plural "s"): Korean may leave them out. Text captured by a
placeholder is translated too when it is a known phrase ("the red cup" → "빨간 컵", "right hand" → "오른손") or a
catalog sentence; otherwise it is kept as it is (an object or robot name)."""
import importlib
import pkgutil
import re
import string
from collections import OrderedDict

LANGS = ("en", "ko")
DEFAULT = "en"
_lang = DEFAULT


def norm(lang):
    """'ko-KR' / 'ko' / '한국어' -> 'ko'; anything else we don't have -> None."""
    s = str(lang or "").strip().lower()
    if s.startswith("ko") or s == "한국어":
        return "ko"
    if s.startswith("en"):
        return "en"
    return None


def set_lang(lang):
    """Set the server's language ('ko', 'ko-KR', 'en-US', …). Unknown values keep the current one. -> the language."""
    global _lang
    n = norm(lang)
    if n:
        _lang = n
    return _lang


def get_lang():
    return _lang


# ------------------------------------------------------------------ catalog
def _load():
    from . import locale as pkg
    out = {}
    for m in sorted(pkgutil.iter_modules(pkg.__path__), key=lambda m: m.name):
        if m.name.startswith("ko_"):
            out.update(importlib.import_module(f"{pkg.__name__}.{m.name}").MESSAGES)
    return out


CATALOG = _load()                     # key -> (en, ko)


def fields(template):
    """Placeholder names of a template: '{obj|을} at {x}' -> {'obj', 'x'}."""
    return {f.split("|")[0] for _, f, _, _ in string.Formatter().parse(template) if f}


# ------------------------------------------------------------------ Korean particles
_DIGIT_FINAL = {"0": "ㅇ", "1": "ㄹ", "2": "", "3": "ㅁ", "4": "", "5": "", "6": "ㄱ", "7": "ㄹ", "8": "ㄹ", "9": ""}
_LETTER_FINAL = {"l": "ㄹ", "m": "ㅁ", "n": "ㄴ", "r": "ㄹ"}          # single letters said as 엘, 엠, 엔, 알
_PARTICLES = {"을": ("을", "를"), "를": ("을", "를"), "이": ("이", "가"), "가": ("이", "가"), "은": ("은", "는"),
              "는": ("은", "는"), "과": ("과", "와"), "와": ("과", "와"), "으로": ("으로", "로"), "로": ("으로", "로"),
              "이에요": ("이에요", "예요"), "예요": ("이에요", "예요"), "이랑": ("이랑", "랑"), "랑": ("이랑", "랑")}


def _final(word):
    """The last sound's final consonant: '' (none), 'ㄹ' or another consonant."""
    w = re.sub(r"[\s\"'’”)\].,!?]+$", "", str(word))
    if not w:
        return ""
    c = w[-1]
    if "가" <= c <= "힣":
        j = (ord(c) - 0xAC00) % 28
        return "" if j == 0 else ("ㄹ" if j == 8 else "x")
    if c.isdigit():
        return _DIGIT_FINAL[c]
    if c.isalpha() and c.isascii():
        tail = re.search(r"[A-Za-z]+$", w).group(0)
        if len(tail) == 1 or tail.isupper():                 # an initialism: G, UR, ALOHA -> letter names
            return _LETTER_FINAL.get(tail[-1].lower(), "") if len(tail) <= 3 else ""
        lo = tail.lower()
        if lo.endswith(("er", "or", "ar", "ure")):
            return ""
        if lo[-1] == "l":
            return "ㄹ"
        return "x" if lo[-1] in "mnkptbg" else ""
    return ""


def particle(word, mark):
    """The particle `mark` (을/를, 이/가, …) that fits after `word`."""
    pair = _PARTICLES.get(mark)
    if pair is None:
        return mark
    f = _final(word)
    if pair[0] == "으로":
        return "로" if f in ("", "ㄹ") else "으로"
    return pair[0] if f else pair[1]


# ------------------------------------------------------------------ words inside messages
PHRASES_KO = {
    "right hand": "오른손", "left hand": "왼손", "right arm": "오른팔", "left arm": "왼팔", "both hands": "양손",
    "right gripper": "오른쪽 그리퍼", "left gripper": "왼쪽 그리퍼", "both arms": "양팔", "hand": "손", "arm": "팔",
    "right": "오른쪽", "left": "왼쪽", "front": "앞쪽", "back": "뒤쪽", "center": "가운데", "centre": "가운데",
    "middle": "가운데", "front left": "앞쪽 왼편", "front right": "앞쪽 오른편", "back left": "뒤쪽 왼편",
    "back right": "뒤쪽 오른편", "red block": "빨간 블록", "green block": "초록 블록", "blue block": "파란 블록",
    "mug": "머그컵", "table": "테이블", "the table": "테이블", "floor": "바닥", "gripper": "그리퍼", "wrist": "손목",
    "person": "사람", "you": "당신", "nothing": "아무것도", "ok": "확인", "done": "완료", "failed": "실패",
    "running": "진행 중", "planning": "계획 중", "here": "여기", "there": "저기", "zone": "구역",
}
WORDS_KO = {
    "red": "빨간", "green": "초록", "blue": "파란", "yellow": "노란", "white": "흰", "black": "검은", "orange": "주황",
    "purple": "보라", "pink": "분홍", "gray": "회색", "grey": "회색", "brown": "갈색", "small": "작은", "big": "큰",
    "large": "큰", "tall": "키 큰", "long": "긴", "short": "짧은", "empty": "빈", "steel": "강철", "wooden": "나무",
    "cup": "컵", "mug": "머그컵", "block": "블록", "cube": "큐브", "bowl": "그릇", "vase": "꽃병", "plate": "접시",
    "bottle": "병", "can": "캔", "box": "상자", "tray": "쟁반", "sponge": "스펀지", "apple": "사과", "banana": "바나나",
    "wrench": "렌치", "screwdriver": "드라이버", "hammer": "망치", "brush": "솔", "torch": "토치", "barbell": "바벨",
    "bench": "벤치", "table": "테이블", "counter": "조리대", "drawer": "서랍", "door": "문", "fridge": "냉장고",
    "sink": "싱크대", "shelf": "선반", "cabinet": "수납장", "tote": "토트", "rack": "랙", "conveyor": "컨베이어",
    "bin": "통", "basket": "바구니", "pan": "팬", "pot": "냄비", "knife": "칼", "fork": "포크", "spoon": "숟가락",
    "spatula": "뒤집개", "glass": "유리컵", "jar": "병", "book": "책", "lid": "뚜껑", "handle": "손잡이",
    "part": "부품", "bolt": "볼트", "nut": "너트", "gear": "기어", "pipe": "파이프",
    "beaker": "비커", "flask": "플라스크", "tube": "튜브", "pipette": "피펫", "board": "보드",
    "chip": "칩", "battery": "배터리", "phone": "휴대폰", "laptop": "노트북", "towel": "수건", "cloth": "천",
    "shirt": "셔츠", "sock": "양말", "marker": "마커", "spot": "자리", "seam": "용접선",
    "workpiece": "작업물", "pallet": "팔레트", "crate": "상자", "carton": "상자", "package": "택배 상자",
    "top": "위", "bottom": "아래", "side": "옆", "upper": "위", "lower": "아래", "inner": "안쪽", "outer": "바깥쪽",
    "right": "오른쪽", "left": "왼쪽", "front": "앞", "back": "뒤", "hand": "손", "arm": "팔",
}
PROPER = frozenset({"spot", "panda", "apollo", "stretch", "stretch 3", "tidybot", "aloha", "g1", "h2", "ur5e", "xarm7",
                    "iiwa14", "unitree g1", "franka panda", "boston dynamics spot"})
_ARTICLE = re.compile(r"^(?:the|a|an)\s+", re.I)


def word(value, lang="ko"):
    """A phrase captured from an English message, in `lang` when we know it ('the red cup' -> '빨간 컵')."""
    v = str(value)
    if lang != "ko" or not v.strip() or re.fullmatch(r"[\d\s.,:+\-/%°×x()]*", v):
        return v
    core = _ARTICLE.sub("", v.strip())
    low = core.lower().replace("_", " ")              # (names keep their underscores unless translated)
    if core[:1].isupper() and (low in PROPER or low.split()[0] in PROPER):    # robot names stay as they are
        return core
    if low in PHRASES_KO:
        return PHRASES_KO[low]
    if v.strip() in _EXACT:                             # a catalog phrase ("door left" -> "왼쪽 문")
        return _render(_EXACT[v.strip()], lang, {})
    if _LIST.search(core):                              # "the mug, the red block or the vase": item by item
        items = _LIST.split(core)
        if len(items) > 1 and all(len(x.split()) <= 4 for x in items):     # names, not clauses
            outs = [word(x, lang) for x in items if x.strip()]
            return ", ".join(outs[:-1]) + (" 또는 " if " or " in core else ", ") + outs[-1]
    toks = low.split()
    if toks and all(t in WORDS_KO or t.isdigit() for t in toks):
        return " ".join(WORDS_KO.get(t, t) for t in toks)
    if " " in core:                                     # a sub-sentence ("my gripper would hit the vase")
        out = _translate_sentence(v.strip(), lang, fragment=True)
        if out is None and _SPLIT.search(v.strip()):    # several sentences: each on its own
            parts = _SPLIT.split(v.strip())
            outs = [_translate_sentence(x, lang, fragment=True) for x in parts]
            if any(o is not None for o in outs):
                out = " ".join(o if o is not None else x for o, x in zip(outs, parts))
        if out is not None:
            return out
    return core if core != v.strip() and re.search(r"[A-Za-z]", core) else v


_LIST = re.compile(r",\s+(?:or\s+|and\s+)?|\s+or\s+|\s+and\s+")


# ------------------------------------------------------------------ templates -> patterns
class _Tpl:
    __slots__ = ("key", "en", "rx", "rx_head", "lit", "first", "frag")

    def __init__(self, key, en):
        self.key, self.en = key, en
        parts, seen, lit = [], set(), 0
        for literal, f, _, _ in string.Formatter().parse(en):
            if literal:
                parts.append(re.escape(literal))
                lit += len(literal)
            if f is not None:
                name = f.split("|")[0]
                if name in seen:
                    parts.append(f"(?P={name})")
                else:
                    seen.add(name)
                    parts.append(f"(?P<{name}>.*?)" if name.startswith("_") else f"(?P<{name}>.+?)")
        body = "".join(parts)
        self.rx = re.compile(body + r"\Z", re.S)
        self.rx_head = re.compile(body, re.S) if not en.rstrip().endswith("}") else None
        self.lit = lit
        first = next(iter(string.Formatter().parse(en)), ("", None, None, None))[0]
        self.first = first[:1] if first else ""
        self.frag = bool(re.match(r"[\s,;:(—–-]", en))


def _index():
    by_first, wild = {}, []
    for k, (en, _ko) in CATALOG.items():
        tp = _Tpl(k, en)
        if fields(en) and tp.lit < 5:                        # "{a}: {b}" would match anything: too generic
            continue
        (by_first.setdefault(tp.first, []) if tp.first else wild).append(tp)
    for lst in list(by_first.values()) + [wild]:
        lst.sort(key=lambda tp: -tp.lit)                     # the most specific template wins
    return by_first, wild


_BY_FIRST, _WILD = _index()
_EXACT = {en: k for k, (en, _ko) in CATALOG.items() if not fields(en)}


def _candidates(s):
    return sorted(_BY_FIRST.get(s[:1], []) + _WILD, key=lambda tp: -tp.lit)


def _render(key, lang, values):
    en, ko = CATALOG[key]
    tpl = ko if lang == "ko" else en
    if lang != "ko":
        return tpl.format(**values)
    out = []
    for literal, f, spec, _conv in string.Formatter().parse(tpl):
        out.append(literal)
        if f is None:
            continue
        name, _, mark = f.partition("|")
        v = values.get(name, "")
        v = word(v, lang) if isinstance(v, str) else format(v, spec or "")
        out.append(v)
        if mark:
            out.append(particle(v, mark))
    return "".join(out)


def _translate_sentence(s, lang, fragment=False):
    """One sentence (or a fragment) -> `lang`, or None when no template matches."""
    for tp in _candidates(s):
        m = tp.rx.match(s)
        if m:
            return _render(tp.key, lang, m.groupdict())
    for tp in _candidates(s):                                # a head template + a known trailing fragment
        if tp.rx_head is None or tp.frag:
            continue
        m = tp.rx_head.match(s)
        if m and m.end() < len(s):
            tail = s[m.end():]
            for ft in _candidates(tail):
                if ft.frag:
                    fm = ft.rx.match(tail)
                    if fm:
                        return _render(tp.key, lang, m.groupdict()) + _render(ft.key, lang, fm.groupdict())
    return None


_SPLIT = re.compile(r"(?<=[.!?])\s+(?=\S)")
_CACHE = OrderedDict()
_CACHE_MAX = 4096


def tr(text, lang=None):
    """An English message in `lang` (default: the current language). Unknown sentences stay English."""
    lang = lang or _lang
    if lang == "en" or not isinstance(text, str) or not re.search(r"[A-Za-z]{2}", text):
        return text
    key = (lang, text)
    hit = _CACHE.get(key)
    if hit is not None:
        _CACHE.move_to_end(key)
        return hit
    s = text.strip()
    out = _translate_sentence(s, lang)
    if out is None:
        parts = _SPLIT.split(s)
        if len(parts) > 1:
            outs = [_translate_sentence(p, lang) for p in parts]
            out = " ".join(o if o is not None else p for o, p in zip(outs, parts))
        else:
            out = s
    lead = text[:len(text) - len(text.lstrip())]
    trail = text[len(text.rstrip()):]
    out = lead + out + trail
    _CACHE[key] = out
    if len(_CACHE) > _CACHE_MAX:
        _CACHE.popitem(last=False)
    return out


def t(key, lang=None, /, **fmt):
    """The catalog entry `key` in `lang` (default: the current language), formatted with `fmt`."""
    lang = lang or _lang
    if key not in CATALOG:
        raise KeyError(key)
    return _render(key, lang if lang in LANGS else DEFAULT, {k: v for k, v in fmt.items()})


# ------------------------------------------------------------------ state frames
MSG_KEYS = frozenset({
    "message", "msg", "why", "reason", "reply", "text", "summary", "question", "hint", "note", "label", "title",
    "detail", "line", "cue", "warning", "error", "caption", "intent", "verdict", "status_text", "path_msg", "issue",
    "issues", "warnings", "notes", "say", "explain", "description", "desc", "tip", "subtitle", "narration", "headline",
    "prompt", "ask", "result", "step", "steps_text", "info", "lines", "events", "options", "status_line", "describe",
    "messages",
})
MSG_MAPS = frozenset({"kinds"})                  # {id: message} maps (state.skillbook.kinds)
SKIP_KEYS = frozenset({
    "geom_poses", "pose", "pts", "points", "physics", "scene_objects", "landmarks", "lm", "joints", "q", "qpos",
    "mesh", "meshes", "ghost", "samples", "dense", "pose_filled", "hands", "objects", "markers", "id", "name", "kind",
    "src", "source", "args", "model", "file", "uv", "pos", "rot", "size", "examples", "utterances", "color",
})
_KEEP_KINDS = frozenset({"user", "reply"})       # the person's words and the LLM's own reply are left alone


def _loc(obj, lang, key=None, depth=0):
    if depth > 8:
        return obj
    if isinstance(obj, str):
        return tr(obj, lang) if key in MSG_KEYS else obj
    if isinstance(obj, dict) and key in MSG_MAPS:
        out = {k: (tr(v, lang) if isinstance(v, str) else v) for k, v in obj.items()}
        return obj if all(out[k] is obj[k] for k in obj) else out
    if isinstance(obj, dict):
        out = None
        keep_text = obj.get("kind") in _KEEP_KINDS
        for k, v in obj.items():
            if k in SKIP_KEYS or (keep_text and k == "text") or v is None or isinstance(v, (bool, int, float)):
                continue
            nv = _loc(v, lang, k, depth + 1)
            if nv is not v:
                if out is None:
                    out = dict(obj)
                out[k] = nv
        return obj if out is None else out
    if isinstance(obj, list):
        first = obj[0] if obj else None
        if first is None or isinstance(first, (int, float)) or (isinstance(first, list) and first
                                                                  and isinstance(first[0], (int, float))):
            return obj
        out = None
        for i, v in enumerate(obj):
            nv = _loc(v, lang, key, depth + 1)
            if nv is not v:
                if out is None:
                    out = list(obj)
                out[i] = nv
        return obj if out is None else out
    return obj


def localize(state, lang=None):
    """The state frame with its messages in `lang` (copy on write: `state` itself is never changed)."""
    lang = lang or _lang
    if lang == "en" or not isinstance(state, dict):
        return state
    return _loc(state, lang)
