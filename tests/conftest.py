"""Unit tests: pure functions only. No server, no camera, no network, no API keys (docs/V1.md, "Open source quality").

Run: `make test` or `.venv/bin/python -m pytest`. The end-to-end checks live in scripts/selftest.py + scripts/checks/."""

import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:  # works without `pip install -e .`
    sys.path.insert(0, str(ROOT))


def models_missing(why):
    """The robot models (vendor/mink, fetched by scripts/setup.sh) aren't on disk: skip locally, but fail in CI, which
    fetches them, so a broken fetch never turns model tests into silent skips."""
    if os.environ.get("CI"):
        pytest.fail(f"{why} (CI runs scripts/setup.sh, which fetches them)")
    pytest.skip(f"{why} (run scripts/setup.sh)")
