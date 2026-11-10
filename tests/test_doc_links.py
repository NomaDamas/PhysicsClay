"""Relative links in the Markdown docs resolve (scripts/check_links.py), so a moved or renamed doc fails CI."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("check_links", ROOT / "scripts" / "check_links.py")
check_links = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(check_links)


def test_repo_docs_have_no_broken_links():
    broken = {str(p.relative_to(ROOT)): check_links.check_file(p) for p in check_links.md_files()}
    broken = {k: v for k, v in broken.items() if v}
    assert not broken, broken


def test_readmes_and_docs_index_are_scanned():
    names = {str(p.relative_to(ROOT)) for p in check_links.md_files()}
    for must in ("README.md", "README.ko.md", "docs/README.md", "CHANGELOG.md", "docs/USAGE.md"):
        assert must in names


def test_catches_missing_files_and_anchors(tmp_path):
    (tmp_path / "b.md").write_text("# Quick start\n\n## VR / MR (Quest)\n\n## 빠른 시작\n", encoding="utf-8")
    a = tmp_path / "a.md"
    a.write_text(
        "[ok](b.md) [ok](b.md#quick-start) [ok](b.md#vr--mr-quest) [ok](b.md#빠른-시작)\n"
        "[web](https://example.com/x) [mail](mailto:a@b.c)\n"
        "[gone](missing.md)\n"
        "[bad anchor](b.md#nope)\n"
        "`[in code](nowhere.md)`\n"
        "```\n[in a fence](nowhere.md)\n```\n",
        encoding="utf-8",
    )
    bad = check_links.check_file(a, root=tmp_path)
    assert [(line, target) for line, target, _ in bad] == [(3, "missing.md"), (4, "b.md#nope")]


def test_slug_matches_github():
    assert check_links.slug("3 ways to instruct") == "3-ways-to-instruct"
    assert check_links.slug("VR / MR (Quest)") == "vr--mr-quest"
    assert check_links.slug("Plan card (계획 카드)") == "plan-card-계획-카드"
    assert check_links.slug("`make run` and HTTPS") == "make-run-and-https"
