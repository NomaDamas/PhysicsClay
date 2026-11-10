"""Check the relative links in the repository's Markdown files.

    .venv/bin/python scripts/check_links.py            # every *.md in the repo
    .venv/bin/python scripts/check_links.py README.md docs/USAGE.md

For each `[text](target)` (and `![alt](target)`) whose target is a relative path, the file or directory must exist.
A `#anchor` must match a heading of the target Markdown file (GitHub's slugs: lower case, punctuation dropped, spaces
to hyphens, a `-1`, `-2` … suffix for repeated headings) or an explicit `<a id="…">` / `<a name="…">`. Links to
http(s), mailto and other schemes are not checked. Links inside fenced code blocks and inline code are ignored.

Exits 1 and lists every broken link; prints nothing else unless -v. Standard library only.
"""
import re
import sys
import unicodedata
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
SKIP_DIRS = {".venv", "vendor", "node_modules", "models", ".git", "datasets", "recordings", ".claude", "certs",
             "library", ".pytest_cache", ".ruff_cache", "__pycache__"}
LINK = re.compile(r"!?\[(?:[^\[\]]|\[[^\]]*\])*\]\(\s*<?([^)\s>]+)>?(?:\s+\"[^\"]*\")?\s*\)")
REF_DEF = re.compile(r"^\s{0,3}\[[^\]]+\]:\s+<?(\S+?)>?(?:\s+.*)?$")
HEADING = re.compile(r"^\s{0,3}(#{1,6})\s+(.*?)\s*#*\s*$")
ANCHOR_TAG = re.compile(r"<a\s+(?:id|name)\s*=\s*\"([^\"]+)\"", re.I)
SCHEME = re.compile(r"^[a-zA-Z][a-zA-Z0-9+.-]*:")


def md_files(root=ROOT):
    out = []
    for p in root.rglob("*.md"):
        rel = p.relative_to(root).parts
        if any(part in SKIP_DIRS for part in rel[:-1]):
            continue
        out.append(p)
    return sorted(out)


def _strip_code(text):
    """The text with fenced code blocks blanked (line numbers kept) and inline code removed."""
    lines, fence = [], None
    for line in text.split("\n"):
        m = re.match(r"^\s{0,3}(`{3,}|~{3,})", line)
        if fence is None and m:
            fence = m.group(1)[0] * len(m.group(1))
            lines.append("")
            continue
        if fence is not None:
            if line.strip().startswith(fence):
                fence = None
            lines.append("")
            continue
        lines.append(re.sub(r"(`+)(?:(?!\1).)+?\1", "", line))
    return lines


def slug(heading):
    """GitHub's anchor for a heading."""
    h = re.sub(r"<[^>]+>", "", heading)                      # inline html
    h = re.sub(r"!?\[([^\]]*)\]\([^)]*\)", r"\1", h)         # links -> their text
    h = h.replace("`", "").strip().lower()
    out = []
    for ch in h:
        cat = unicodedata.category(ch)
        if ch in " -":
            out.append("-" if ch == " " else ch)
        elif ch == "_" or cat[0] in "LN" or cat == "Mn" or cat == "Mc":
            out.append(ch)
    return "".join(out)


_ANCHORS = {}


def anchors(path):
    """The anchors a Markdown file defines (cached per run)."""
    _cache = _ANCHORS
    if path not in _cache:
        found, seen = set(), {}
        for line in _strip_code(path.read_text(encoding="utf-8", errors="replace")):
            m = HEADING.match(line)
            if m:
                s = slug(m.group(2))
                n = seen.get(s, 0)
                found.add(s if n == 0 else f"{s}-{n}")
                seen[s] = n + 1
        found.update(a.lower() for a in ANCHOR_TAG.findall(path.read_text(encoding="utf-8", errors="replace")))
        _cache[path] = found
    return _cache[path]


def check_file(path, root=ROOT):
    """Broken links in one Markdown file: [(line number, target, why)]."""
    bad = []
    for i, line in enumerate(_strip_code(path.read_text(encoding="utf-8", errors="replace")), 1):
        targets = [m.group(1) for m in LINK.finditer(line)]
        m = REF_DEF.match(line)
        if m:
            targets.append(m.group(1))
        for target in targets:
            if SCHEME.match(target) or target.startswith("//"):
                continue
            file_part, _, frag = target.partition("#")
            file_part = file_part.split("?", 1)[0]
            dest = path if not file_part else (
                (root / file_part.lstrip("/")) if file_part.startswith("/") else (path.parent / file_part))
            try:
                dest = dest.resolve()
            except OSError:
                bad.append((i, target, "unresolvable path"))
                continue
            if not dest.exists():
                bad.append((i, target, "no such file"))
                continue
            if frag and dest.is_file() and dest.suffix.lower() == ".md":
                if unquote(frag).lower() not in anchors(dest):
                    bad.append((i, target, f"no heading #{frag} in {dest.relative_to(root) if dest.is_relative_to(root) else dest}"))
    return bad


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    verbose = "-v" in argv
    paths = [Path(a).resolve() for a in argv if a != "-v"] or md_files()
    broken = 0
    for p in paths:
        for line, target, why in check_file(p):
            broken += 1
            rel = p.relative_to(ROOT) if p.is_relative_to(ROOT) else p
            print(f"{rel}:{line}: {target}  ({why})")
    if verbose:
        print(f"{len(paths)} files, {broken} broken links")
    return 1 if broken else 0


if __name__ == "__main__":
    sys.exit(main())
