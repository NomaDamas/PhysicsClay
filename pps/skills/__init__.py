"""P4 Skills from demonstration (docs/skills.md, issue #32): parametric, object-relative skills.

A demonstration (P3 `learn`, recordings/demos/<name>.json) becomes a skill: *how* the person did it (speed, weave,
angles, force, pattern) as a few parameters, separate from *where* (the seam, the region, the container, the target),
which comes from the scene when the skill is applied. "Geometry gives precision, the person gives style."

    from pps import skills
    sk = skills.extract(demo, world=world)            # rank the kinds, fit the best one -> skill dict
    out = skills.compile(sk, world, target="seam_l")  # -> P2 trajectory (or the message of a behaviour)
    skills.describe(sk)                               # "4 mm zigzag at 2 Hz, 5 mm/s, 15° push angle"
    sk, said = skills.edit_text(sk, "wider")          # voice corrections -> params
    skills.save(sk); skills.load(name); skills.listing()

Kinds: weave_seam, wipe, stir, press (trajectories), spot (a watch `assist` behaviour), mirror_motion (the humanoid
video -> motion skill path, replayed with play_skill). Pure Python + numpy; the server side is pps/features/skills.py.
"""
from .compile import COMPILE, compile_skill as compile  # noqa: F401
from .core import (KINDS, SkillError, delete, describe, edit, is_trajectory, listing, load, make, needs,  # noqa: F401
                   normalize, safe_name, save, skill_dir, summary)
from .extract import ambiguous, extract, rank  # noqa: F401
from .phrases import changes_from_text


def edit_text(sk, text):
    """A spoken correction ("wider", "a bit slower", "4 mm") -> (new skill, [what changed]). Raises SkillError when the
    words don't change anything this kind has."""
    ch = changes_from_text(text, sk["kind"], sk["params"])
    if not ch:
        raise SkillError(f"I didn't get what to change in “{text}” (try wider, narrower, slower, faster, more push, closer, harder)")
    return edit(sk, ch)
