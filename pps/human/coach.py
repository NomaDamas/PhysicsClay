"""Coaching: compare each repetition against a reference and turn the differences into short spoken cues.

A reference is a dict of limits per activity. The built-in ones encode common form rules; a demonstration recorded
in `learn` mode can replace them (`reference_from_demo`): then the person is compared with their own (or a
trainer's) good reps — depth, tempo, weave width and frequency.

Cues are short, imperative and ordered by importance; the coach speaks at most one per rep.
"""
import numpy as np

DEFAULTS = {
    # squat: depth from the knee angle at the bottom, knee valgus from knee width / ankle width at the bottom,
    # back angle from the torso's lean from vertical, tempo from the rep time
    "squat": {"knee_max_bottom": 110.0, "valgus_min": 0.75, "lean_max": 50.0, "min_rep_s": 1.0},
    "curl": {"elbow_max_top": 75.0, "elbow_min_bottom": 140.0, "elbow_drift_max": 0.12, "min_rep_s": 1.0},
    "press": {"lockout_wz": 0.35, "min_rep_s": 1.0},
    "weave": {"amp": None, "freq": None, "amp_tol": 0.4, "freq_tol": 0.4, "travel": None},
}

CUES = {
    "depth": "Go a little deeper.",
    "valgus": "Push your knees out; they're caving in.",
    "back": "Chest up, keep your back straighter.",
    "fast": "Slow down, control it.",
    "curl_top": "Curl all the way up.",
    "curl_bottom": "Lower all the way down.",
    "elbow": "Keep your elbow by your side.",
    "lockout": "Lock out overhead.",
    "weave_wide": "Narrower weave.",
    "weave_narrow": "Wider weave, like the demo.",
    "weave_fast": "Slower side to side.",
    "weave_slow": "A bit quicker side to side.",
    "travel_fast": "Travel slower along the seam.",
    "travel_slow": "Travel a bit faster along the seam.",
}


def reference(activity, custom=None):
    ref = dict(DEFAULTS.get(activity, {}))
    if custom and custom.get("activity") == activity:
        ref.update({k: v for k, v in custom.items() if k != "activity"})
    return ref


def reference_from_demo(demo):
    """A recorded demonstration (pps.human.demo) -> a reference for its activity, or None."""
    act = (demo or {}).get("activity")
    stats = (demo or {}).get("rep_stats") or []
    if act == "weave" and demo.get("pattern"):
        p = demo["pattern"]
        return {"activity": "weave", "amp": p.get("amp"), "freq": p.get("freq"), "travel": p.get("travel")}
    if not stats:
        return None
    med = lambda k: float(np.median([s[k] for s in stats if s.get(k) is not None])) if any(s.get(k) is not None for s in stats) else None
    if act == "squat":
        k = med("knee_min")
        return {"activity": act, "knee_max_bottom": None if k is None else k + 12.0, "min_rep_s": max(0.8, 0.7 * (med("duration") or 1.4))}
    if act == "curl":
        return {"activity": act, "elbow_max_top": (med("elbow_min") or 60) + 15.0, "elbow_min_bottom": (med("elbow_max") or 160) - 15.0}
    if act == "press":
        return {"activity": act, "lockout_wz": (med("wz_max") or 0.45) - 0.08}
    return None


def cues(rep, ref):
    """One rep's stats (ActivityTracker.rep_stats item) + reference -> [cue keys], most important first."""
    act = rep.get("activity")
    out = []
    if act == "squat":
        if rep.get("valgus_bottom", 9) < ref.get("valgus_min", 0):
            out.append("valgus")
        if ref.get("knee_max_bottom") and rep.get("knee_min", 0) > ref["knee_max_bottom"]:
            out.append("depth")
        if rep.get("lean_max", 0) > ref.get("lean_max", 90):
            out.append("back")
    elif act == "curl":
        if rep.get("elbow_min", 0) > ref.get("elbow_max_top", 180):
            out.append("curl_top")
        if rep.get("elbow_max", 180) < ref.get("elbow_min_bottom", 0):
            out.append("curl_bottom")
        if rep.get("elfwd_max", 0) - rep.get("elfwd_min", 0) > ref.get("elbow_drift_max", 9):
            out.append("elbow")
    elif act == "press":
        if rep.get("wz_max", 9) < ref.get("lockout_wz", -9):
            out.append("lockout")
    elif act == "weave":
        a, f, tr = rep.get("amp"), rep.get("freq"), rep.get("travel")
        if ref.get("amp") and a:
            if a > ref["amp"] * (1 + ref.get("amp_tol", 0.4)):
                out.append("weave_wide")
            elif a < ref["amp"] * (1 - ref.get("amp_tol", 0.4)):
                out.append("weave_narrow")
        if ref.get("freq") and f:
            if f > ref["freq"] * (1 + ref.get("freq_tol", 0.4)):
                out.append("weave_fast")
            elif f < ref["freq"] * (1 - ref.get("freq_tol", 0.4)):
                out.append("weave_slow")
        if ref.get("travel") and tr:
            if tr > ref["travel"] * 1.6:
                out.append("travel_fast")
            elif tr < ref["travel"] * 0.5:
                out.append("travel_slow")
    dur = rep.get("duration")
    if dur and act in ("squat", "curl", "press") and dur < ref.get("min_rep_s", 0):
        out.append("fast")
    return out


def say(keys):
    return " ".join(CUES[k] for k in keys[:1])
