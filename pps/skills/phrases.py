"""Spoken corrections of a skill's parameters, offline: "wider", "slower", "a bit faster weave", "more push", "drag",
"closer", "harder", "deeper", "4 mm", "2 Hz", "5 mm/s", "15 degrees", "zigzag", "circle". With the LLM agent the same
edits arrive as edit_skill_params(changes); this covers the agent-off path and the agent's `text` shortcut.

    changes_from_text("wider and a bit slower", "weave_seam")
        -> {"weave.amp": {"mul": 1.25}, "travel_speed": {"mul": 0.85}}
"""
import re

UP, DOWN = 1.25, 0.8
BIT_UP, BIT_DOWN = 1.12, 0.89

# (regex, {kind: changes}) — first match per clause wins; "a bit" / "slightly" softens a factor
RULES = [
    (r"\b(wider|broader|bigger weave|more weave|넓게)\b", {"weave_seam": {"weave.amp": {"mul": UP}}, "stir": {"radius": {"mul": UP}},
                                                         "wipe": {"spacing": {"mul": UP}}}),
    (r"\b(narrower|tighter|smaller weave|less weave|smaller|좁게)\b", {"weave_seam": {"weave.amp": {"mul": DOWN}}, "stir": {"radius": {"mul": DOWN}},
                                                                    "wipe": {"spacing": {"mul": DOWN}}}),
    (r"\b(no weave|straight bead|stringer)\b", {"weave_seam": {"weave.amp": 0.0}}),
    (r"\b(faster weave|quicker weave|weave faster|more often)\b", {"weave_seam": {"weave.freq": {"mul": UP}}}),
    (r"\b(slower weave|weave slower|less often)\b", {"weave_seam": {"weave.freq": {"mul": DOWN}}}),
    (r"\b(slower|slow down|더 천천히|천천히)\b", {"weave_seam": {"travel_speed": {"mul": DOWN}}, "wipe": {"speed": {"mul": DOWN}},
                                             "stir": {"freq": {"mul": DOWN}}, "press": {"hold_s": {"mul": UP}}}),
    (r"\b(faster|speed up|quicker|더 빨리|빨리)\b", {"weave_seam": {"travel_speed": {"mul": UP}}, "wipe": {"speed": {"mul": UP}},
                                                 "stir": {"freq": {"mul": UP}}, "press": {"hold_s": {"mul": DOWN}}}),
    (r"\b(more push|push more|more angle|lean more)\b", {"weave_seam": {"travel_angle": {"add": 5.0}}}),
    (r"\b(less push|less angle|more upright|straighter)\b", {"weave_seam": {"travel_angle": {"add": -5.0}}}),
    (r"\b(drag|pull)\b", {"weave_seam": {"travel_angle": "drag"}}),
    (r"\bpush\b", {"weave_seam": {"travel_angle": "push"}}),
    (r"\b(straight up|square|perpendicular|no angle)\b", {"weave_seam": {"travel_angle": 0.0, "work_angle": 90.0}}),
    (r"\b(closer|lower|nearer)\b", {"weave_seam": {"standoff": {"add": -0.003}}, "stir": {"depth": {"add": 0.01}}}),
    (r"\b(further|farther|higher|more standoff)\b", {"weave_seam": {"standoff": {"add": 0.003}}, "stir": {"depth": {"add": -0.01}}}),
    (r"\b(harder|more force|press harder|stronger|세게)\b", {"wipe": {"force": {"mul": 1.5}}, "press": {"force": {"mul": 1.5}},
                                                        "spot": {"max_force": {"mul": 1.25}}}),
    (r"\b(softer|gentler|lighter|less force|살살|약하게)\b", {"wipe": {"force": {"mul": 0.67}}, "press": {"force": {"mul": 0.67}},
                                                          "spot": {"max_force": {"mul": 0.8}}}),
    (r"\b(deeper)\b", {"stir": {"depth": {"add": 0.01}}}),
    (r"\b(shallower)\b", {"stir": {"depth": {"add": -0.01}}}),
    (r"\b(more turns|longer|keep going)\b", {"stir": {"turns": {"add": 2}}, "press": {"hold_s": {"mul": UP}}}),
    (r"\b(fewer turns|shorter)\b", {"stir": {"turns": {"add": -1}}, "press": {"hold_s": {"mul": DOWN}}}),
    (r"\b(another pass|more passes|twice)\b", {"wipe": {"passes": {"add": 1}}}),
    (r"\bspiral\b", {"wipe": {"pattern": "spiral"}}),
    (r"\b(raster|back and forth|lines)\b", {"wipe": {"pattern": "raster"}}),
    (r"\b(zig ?zag|side to side)\b", {"weave_seam": {"weave.kind": "zigzag"}}),
    (r"\b(circles?|circular|loops?)\b", {"weave_seam": {"weave.kind": "circle"}}),
    (r"\b(figure ?(8|eight))\b", {"weave_seam": {"weave.kind": "figure8"}}),
    (r"\b(counter ?clockwise|anti ?clockwise)\b", {"stir": {"direction": "ccw"}}),
    (r"\b(clockwise)\b", {"stir": {"direction": "cw"}}),
    (r"\b(help earlier|help sooner|earlier)\b", {"spot": {"assist_when.bar_velocity_below": {"mul": UP}, "assist_when.for_s": {"mul": DOWN}}}),
    (r"\b(help later|let me grind|later)\b", {"spot": {"assist_when.bar_velocity_below": {"mul": DOWN}, "assist_when.for_s": {"mul": UP}}}),
]

# numbers with units: (regex, {kind: path}, scale to SI)
UNITS = [
    (r"(\d+(?:\.\d+)?)\s*(mm/s|millimet(?:er|re)s? per second)", {"weave_seam": "travel_speed", "wipe": "speed"}, 0.001),
    (r"(\d+(?:\.\d+)?)\s*(cm/s|centimet(?:er|re)s? per second)", {"weave_seam": "travel_speed", "wipe": "speed"}, 0.01),
    (r"(\d+(?:\.\d+)?)\s*(hz|hertz|times a second)", {"weave_seam": "weave.freq", "stir": "freq"}, 1.0),
    (r"(\d+(?:\.\d+)?)\s*(mm|millimet(?:er|re)s?)\b(?!/)", {"weave_seam": "weave.amp", "wipe": "spacing", "stir": "radius"}, 0.001),
    (r"(\d+(?:\.\d+)?)\s*(cm|centimet(?:er|re)s?)\b(?!/)", {"weave_seam": "weave.amp", "wipe": "spacing", "stir": "radius",
                                                         "spot": "lift_cm"}, 0.01),
    (r"(\d+(?:\.\d+)?)\s*(°|deg(?:ree)?s?)", {"weave_seam": "travel_angle"}, 1.0),
    (r"(\d+(?:\.\d+)?)\s*(n|newtons?)\b", {"wipe": "force", "press": "force", "spot": "max_force"}, 1.0),
    (r"(\d+(?:\.\d+)?)\s*(s|sec(?:ond)?s?)\b", {"press": "hold_s", "spot": "assist_when.for_s"}, 1.0),
    (r"(\d+)\s*(turns?|times|circles)\b", {"stir": "turns"}, 1.0),
    (r"(\d+)\s*(pass(?:es)?)\b", {"wipe": "passes"}, 1.0),
]
SOFT = re.compile(r"\b(a (little|bit|touch)|slightly|a tad|조금|약간)\b")


def _soften(ch):
    out = {}
    for k, v in ch.items():
        if isinstance(v, dict) and "mul" in v:
            m = v["mul"]
            out[k] = {"mul": BIT_UP if m > 1 else BIT_DOWN}
        elif isinstance(v, dict) and "add" in v:
            out[k] = {"add": v["add"] / 2}
        else:
            out[k] = v
    return out


def changes_from_text(text, kind, params=None):
    """-> {dotted path: value | {mul} | {add}} or {} when nothing was understood."""
    t = " " + str(text or "").lower().replace("-", " ") + " "
    out = {}
    for clause in re.split(r",|\band\b|\bbut\b|;|\.", t):
        if not clause.strip():
            continue
        soft = bool(SOFT.search(clause))
        hit = None
        for rx, per in RULES:
            if re.search(rx, clause) and kind in per:
                hit = dict(per[kind])
                break
        used = []
        for rx, per, scale in UNITS:
            for m in re.finditer(rx, clause):
                if kind not in per or any(a < m.end() and m.start() < b for a, b in used):
                    continue
                used.append((m.start(), m.end()))
                v = float(m.group(1)) * scale
                path = per[kind]
                if path == "lift_cm":
                    v = float(m.group(1)) * (1.0 if m.group(2).startswith("c") else 0.1)
                if path == "travel_angle" and re.search(r"\bdrag\b", clause):
                    v = -abs(v)
                hit = dict(hit or {}, **{path: v})
        if hit:
            if "travel_angle" in hit and hit["travel_angle"] in ("push", "drag"):
                cur = abs(float((params or {}).get("travel_angle") or 10.0)) or 10.0
                hit["travel_angle"] = cur if hit["travel_angle"] == "push" else -cur
            out.update(_soften(hit) if soft else hit)
    return out
