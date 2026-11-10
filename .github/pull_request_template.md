## What

<!-- What changes, in a few bullets. -->

## Why

<!-- The problem this solves. Link the issue: "Refs #N" or "Closes #N". -->

## How it was tested

- [ ] `make lint` (ruff)
- [ ] `make test` (pytest)
- [ ] `make selftest` (or `PC_SELFTEST_OFFLINE=1 PC_SELFTEST_PORT=<port> .venv/bin/python scripts/selftest.py`), with the result: `N/N passed`
- [ ] New behaviour has a unit test in `tests/` and/or a check in `scripts/checks/`
- [ ] Manual check in the browser (add screenshots or a clip for UI changes)

## Checklist

- [ ] No keys, `.env`, `certs/`, recordings or large binaries are committed
- [ ] New messages / state keys are documented (module docstring and `docs/PROTOCOL.md`)
- [ ] New third-party code, models or assets are listed in `NOTICE` with their license
- [ ] A new robot has its prompts and `web/robots/catalog.json` is regenerated (`scripts/sync_robot_prompts.py`)
