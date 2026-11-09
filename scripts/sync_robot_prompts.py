"""Copy the per-robot UI data (pps/world/prompts/<id>.json) into one static file the browser loads:
web/robots/catalog.json = {id: {label, kind, tagline, examples, tools}}. The .md prompts stay server-side (LLM only).

Run after editing any pps/world/prompts/*.json:   .venv/bin/python scripts/sync_robot_prompts.py
`--check` exits non-zero when the static copy is stale (scripts/checks/robots.py does the same over HTTP)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "pps" / "world" / "prompts"
OUT = ROOT / "web" / "robots" / "catalog.json"
ORDER = ["g1", "panda", "ur5e", "xarm7", "iiwa14", "aloha", "apollo", "stretch3", "tidybot", "spot", "h2"]


def build():
    ids = ORDER + sorted(p.stem for p in SRC.glob("*.json") if p.stem not in ORDER)
    return {i: json.loads((SRC / f"{i}.json").read_text()) for i in ids if (SRC / f"{i}.json").exists()}


def render(cat):
    return json.dumps(cat, indent=1, ensure_ascii=False) + "\n"


if __name__ == "__main__":
    text = render(build())
    if "--check" in sys.argv:
        ok = OUT.exists() and OUT.read_text() == text
        print("catalog up to date" if ok else f"stale: run python scripts/sync_robot_prompts.py ({OUT.relative_to(ROOT)})")
        sys.exit(0 if ok else 1)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text)
    print(f"wrote {OUT.relative_to(ROOT)} ({len(text)} bytes)")
