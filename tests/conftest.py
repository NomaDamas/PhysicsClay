"""Unit tests: pure functions only. No server, no camera, no network, no API keys (docs/V1.md, "Open source quality").

Run: `make test` or `.venv/bin/python -m pytest`. The end-to-end checks live in scripts/selftest.py + scripts/checks/."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:  # works without `pip install -e .`
    sys.path.insert(0, str(ROOT))
