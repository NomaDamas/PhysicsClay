"""Respond rules: "when <the person does X>, <do Y>".

`when` is a small predicate over `state.human`:

* built-in terms, combined with `and` / `or` / `not`:
  `hand_open_near_robot`, `hand_out`, `hand_open`, `holding`, `holding:<tool>`, `not_holding`, `activity:<name>`,
  `phase:<name>`, `reps>=<n>`, `stalled`, `still`, `pointing`, `visible`, `fast` (moving > 0.8 m/s)
* or an expression (`expr`), which is how the agent compiles free text it can't express with the terms: Python
  syntax restricted to comparisons, and/or/not, constants and field names of `state.human` (dotted for nested ones,
  e.g. `activity == "squat" and reps >= 5`, `hands.right.open and holding == "torch"`). Nothing else is allowed (no
  calls, no subscripts, no attributes of anything but the state), so it is safe to evaluate.

`compile_when(text)` turns everyday phrasing into terms offline ("when I put my hand out" -> hand_open_near_robot,
"when I'm holding the torch" -> holding:torch, "after 5 squats" -> activity:squat and reps>=5); the LLM agent handles
the rest. `compile_do(text)` turns the action into a task message ("give me the wrench" -> handover wrench) or keeps it
as an instruction for the agent.
"""
import ast
import re

TERMS = ("hand_open_near_robot", "hand_out", "hand_open", "holding", "not_holding", "stalled", "still", "pointing",
         "visible", "fast")
PARAM_TERMS = ("holding:", "activity:", "phase:", "reps>=")
ACTIVITIES = ("squat", "press", "curl", "wave", "weave", "reach", "point", "still", "moving")


class RuleError(ValueError):
    pass


def _norm(s):
    return re.sub(r"[\s\-]+", "_", str(s or "").strip().lower())


def _same(a, b):
    a, b = _norm(a).replace("the_", ""), _norm(b).replace("the_", "")
    return bool(a) and bool(b) and (a == b or a in b or b in a)


def term(h, t):
    """One built-in term over the human state dict -> bool."""
    t = t.strip()
    hands = (h.get("hands") or {}).values()
    if not h.get("visible"):
        return t == "not_visible"
    if t == "visible":
        return True
    if t == "hand_open_near_robot":
        return any(x.get("open") and x.get("near_robot") for x in hands)
    if t == "hand_out":
        return bool(h.get("offer")) or any(x.get("near_robot") for x in hands)
    if t == "hand_open":
        return any(x.get("open") for x in hands)
    if t == "holding":
        return bool(h.get("holding"))
    if t == "not_holding":
        return not h.get("holding")
    if t == "stalled":
        return bool(h.get("stalled"))
    if t == "still":
        return h.get("activity") == "still"
    if t == "pointing":
        return h.get("activity") == "point"
    if t == "fast":
        return float(h.get("speed") or 0) > 0.8
    if t.startswith("holding:"):
        return _same(h.get("holding"), t.split(":", 1)[1])
    if t.startswith("activity:"):
        return h.get("activity") == t.split(":", 1)[1].strip()
    if t.startswith("phase:"):
        return h.get("phase") == t.split(":", 1)[1].strip()
    if t.startswith("reps>="):
        return int(h.get("reps") or 0) >= int(float(t[6:]))
    raise RuleError(f"unknown condition '{t}'")


def check_terms(when):
    """Validate a terms string; -> normalised string or raise RuleError."""
    w = str(when or "").strip().lower()
    if not w:
        raise RuleError("empty condition")
    toks = re.split(r"\s+(and|or)\s+", w)
    for i, tk in enumerate(toks):
        if i % 2:
            continue
        tk = tk.strip()
        if tk.startswith("not "):
            tk = tk[4:].strip()
        if tk in TERMS or tk == "not_visible":
            continue
        if tk.startswith(PARAM_TERMS) and len(tk.split(":", 1)[-1]) and (not tk.startswith("reps>=") or tk[6:].strip().isdigit()):
            if tk.startswith("activity:") and tk.split(":", 1)[1] not in ACTIVITIES:
                raise RuleError(f"unknown activity '{tk.split(':', 1)[1]}' (know: {', '.join(ACTIVITIES)})")
            continue
        raise RuleError(f"unknown condition '{tk}'")
    return w


def eval_terms(h, when):
    """'a and not b or c' (and binds tighter than or)."""
    for alt in re.split(r"\s+or\s+", when):
        ok = True
        for tk in re.split(r"\s+and\s+", alt):
            tk = tk.strip()
            neg = tk.startswith("not ")
            v = term(h, tk[4:].strip() if neg else tk)
            if v == neg:
                ok = False
                break
        if ok:
            return True
    return False


# ---------------------------------------------------------------------- expressions (agent-compiled free text)
_ALLOWED = (ast.Expression, ast.BoolOp, ast.And, ast.Or, ast.UnaryOp, ast.Not, ast.Compare, ast.Eq, ast.NotEq,
            ast.Lt, ast.LtE, ast.Gt, ast.GtE, ast.In, ast.NotIn, ast.Name, ast.Load, ast.Attribute, ast.Constant,
            ast.Tuple, ast.List, ast.USub)


def check_expr(expr):
    try:
        tree = ast.parse(str(expr), mode="eval")
    except SyntaxError as e:
        raise RuleError(f"expression doesn't parse: {e.msg}") from None
    for n in ast.walk(tree):
        if not isinstance(n, _ALLOWED):
            raise RuleError(f"'{type(n).__name__}' is not allowed in a watch expression")
        if isinstance(n, ast.Attribute) and n.attr.startswith("_"):
            raise RuleError("private names are not allowed")
        if isinstance(n, ast.Name) and n.id.startswith("_"):
            raise RuleError("private names are not allowed")
    return tree


class _NS(dict):
    """Dotted access into the state; a missing field is None (so a rule over a hand that isn't seen is just false)."""

    def get_path(self, node):
        if isinstance(node, ast.Name):
            return self.get(node.id)
        base = self.get_path(node.value)
        return base.get(node.attr) if isinstance(base, dict) else None


def eval_expr(h, tree):
    ns = _NS(h)

    def ev(n):
        if isinstance(n, ast.Expression):
            return ev(n.body)
        if isinstance(n, ast.BoolOp):
            vals = (ev(v) for v in n.values)
            return all(vals) if isinstance(n.op, ast.And) else any(vals)
        if isinstance(n, ast.UnaryOp):
            v = ev(n.operand)
            return (not v) if isinstance(n.op, ast.Not) else (-v if v is not None else None)
        if isinstance(n, ast.Compare):
            left = ev(n.left)
            for op, rc in zip(n.ops, n.comparators):
                right = ev(rc)
                try:
                    ok = {ast.Eq: lambda a, b: a == b, ast.NotEq: lambda a, b: a != b, ast.Lt: lambda a, b: a < b,
                          ast.LtE: lambda a, b: a <= b, ast.Gt: lambda a, b: a > b, ast.GtE: lambda a, b: a >= b,
                          ast.In: lambda a, b: a in b, ast.NotIn: lambda a, b: a not in b}[type(op)](left, right)
                except TypeError:          # None vs number: the field isn't there, the condition isn't met
                    return False
                if not ok:
                    return False
                left = right
            return True
        if isinstance(n, (ast.Name, ast.Attribute)):
            return ns.get_path(n)
        if isinstance(n, ast.Constant):
            return n.value
        if isinstance(n, (ast.Tuple, ast.List)):
            return [ev(e) for e in n.elts]
        raise RuleError("bad expression")
    return bool(ev(tree))


class Predicate:
    """when (terms) or expr -> callable(human_state) -> bool."""

    def __init__(self, when=None, expr=None):
        self.when, self.expr = None, None
        if expr:
            self.expr = str(expr).strip()
            self._tree = check_expr(self.expr)
        else:
            self.when = check_terms(when)

    def __call__(self, h):
        return eval_expr(h, self._tree) if self.expr else eval_terms(h, self.when)

    def text(self):
        return self.expr or self.when


# ---------------------------------------------------------------------- offline phrase compiler
_HAND_OUT = re.compile(r"(hand|palm)s?\s+(is\s+|are\s+)?(out|open)|hold(s|ing)?\s+out\s+(my|a|the|your)?\s*(hand|palm)|"
                       r"(put|stick|reach|hold)s?\s+(my|a|the)?\s*hand\s+out|open\s+(my\s+)?hand|손.{0,4}(내밀|펴|벌)")
_HOLD = re.compile(r"(?:hold(?:s|ing)?|grab(?:s|bing)?|pick(?:s|ing)?\s+up|take(?:s)?|have|has)\s+(?:a|an|the|my)?\s*([a-z][a-z _-]{1,30}?)"
                   r"(?=$|[,.]| and | or | then | up\b| down\b)")
_REPS = re.compile(r"(\d+)\s*(?:reps?|times|squats?|curls?|presses?)|rep\s*(\d+)|(\d+)\s*(?:번|개|회)")
_ACT_WORDS = (("squat", "squat"), ("스쿼트", "squat"), ("curl", "curl"), ("컬", "curl"), ("press", "press"),
              ("wave", "wave"), ("waving", "wave"), ("손 흔", "wave"), ("weav", "weave"), ("zigzag", "weave"),
              ("reach", "reach"))
_STALL = re.compile(r"\b(stall|stuck|get stuck|can'?t (lift|get up|push)|struggl|fail(s|ing)? (a|the) rep)|버거|못 일어")
_STILL = re.compile(r"\b(stand still|stop moving|freeze|stay still|don'?t move)|가만")
_POINT = re.compile(r"\bpoint(s|ing)?\b|가리키")


def compile_when(text):
    """Everyday phrasing -> terms string, or None if it isn't understood offline (the agent compiles it then)."""
    t = " " + str(text or "").lower().strip() + " "
    t = re.sub(r"^\s*(when(ever)?|if|once|after|as soon as)\s+", " ", t)
    terms = []
    if _HAND_OUT.search(t):
        terms.append("hand_open_near_robot")
    m = _HOLD.search(t)
    if m and not _HAND_OUT.search(t[max(0, m.start() - 3):m.end() + 3]):
        tool = m.group(1).strip()
        tool = re.sub(r"\b(it|this|that|one|something|anything)\b", "", tool).strip(" _-")
        terms.append(f"holding:{_norm(tool)}" if tool else "holding")
    act = next((a for w, a in _ACT_WORDS if w in t), None)
    if act:
        terms.append(f"activity:{act}")
    r = _REPS.search(t)
    if r and act in ("squat", "curl", "press", None):
        n = next(g for g in r.groups() if g)
        terms.append(f"reps>={int(n)}")
    if _STALL.search(t):
        terms.append("stalled")
    if _STILL.search(t):
        terms.append("still")
    if _POINT.search(t) and not terms:
        terms.append("pointing")
    if not terms:
        return None
    return " and ".join(dict.fromkeys(terms))


_GIVE = re.compile(r"(?:give|hand|pass|bring|fetch|get)\s+(?:me\s+)?(?:the|a|an|my)?\s*([a-z][a-z _-]{1,30}?)(?:\s+to\s+me)?\s*$|"
                   r"(?:give|hand|pass|bring)\s+(?:me|it)\b")
_KO_GIVE = re.compile(r"(.{1,12}?)\s*(?:을|를)?\s*(?:줘|건네|넘겨|주세요)")


def split_rule(text):
    """'when I put my hand out, give me the wrench' -> ('when I put my hand out', 'give me the wrench') or None."""
    t = str(text or "").strip().rstrip(".!")
    m = re.match(r"^\s*(when(ever)?|if|once|after|as soon as)\b(.+?)(?:,|\bthen\b)\s*(.+)$", t, re.I)
    if m:
        return m.group(3).strip(), m.group(4).strip()
    m = re.match(r"^(.+?)(?:하면|면)\s*,?\s*(.+)$", t)       # Korean: "손 내밀면 렌치 줘"
    if m and len(m.group(1)) > 1:
        return m.group(1).strip(), m.group(2).strip()
    return None


def compile_do(text, objects=()):
    """Action text -> {"task": {...}} | {"say": text} | {"mode": name} | {"agent": text}."""
    t = str(text or "").strip()
    low = t.lower()
    if not low:
        raise RuleError("empty action")
    if re.match(r"^(stop|halt|freeze|멈춰)\b", low):
        return {"task": {"action": "stop"}}
    m = re.match(r"^(say|tell me|announce|count)\s+(.+)$", low)
    if m and m.group(1) != "count":
        return {"say": t[len(m.group(1)):].strip(" :\"'")}
    if re.search(r"\b(spot|help me|assist)\b|도와", low):
        return {"mode": "assist"}
    g = _GIVE.search(low)
    obj = None
    if g:
        obj = (g.group(1) or "").strip()
        obj = re.sub(r"\b(it|this|that|one)\b", "", obj).strip()
    elif _KO_GIVE.search(t):
        obj = _KO_GIVE.search(t).group(1).strip()
    if g or _KO_GIVE.search(t):
        names = list(objects)
        match = next((n for n in names if _same(n, obj)), None) if obj else None
        return {"task": {"action": "handover", "object": match or (obj or "it")}}
    return {"agent": t}
