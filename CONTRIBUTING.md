# Contributing to PhysicsClay

Thanks for helping. PhysicsClay is a MuJoCo + mink + aiohttp + three.js web app where people instruct robots by
saying, pointing and showing. This guide covers setup, running and testing, the extension points, and how we
review changes. `docs/architecture.md` explains how the pieces fit together.

## Setup

You need [uv](https://docs.astral.sh/uv/), git, curl and Python 3.10+ (uv can install Python for you). macOS and
Linux are supported.

```bash
make setup            # = ./scripts/setup.sh: .venv, Python deps, robot models (vendor/mink), MediaPipe models
```

`scripts/setup.sh` installs the package in editable mode with the `vision` and `dev` extras. Pick others with
`PC_EXTRAS`:

| Extra | What it adds | License note |
|---|---|---|
| `vision` | MediaPipe pose and object detection on the server (`--webcam`, `--video`, `--objects`) | Apache-2.0 |
| `data` | `pyarrow`, for LeRobot datasets | Apache-2.0 |
| `dev` | `ruff`, `pytest` | MIT |
| `yolo` | Ultralytics YOLO11 segmentation and SAM 2.1 | **AGPL-3.0**: see `NOTICE` |

```bash
PC_EXTRAS=vision,data,dev ./scripts/setup.sh
```

API keys are optional. To use them locally, put them in `.env`, which is git-ignored and must never be committed.
The keys are `GEMINI_API_KEY`, `OPENAI_API_KEY`, `LLM_BASE_URL`, `LLM_API_KEY` and `LLM_MODEL`. Without keys the
app falls back to browser speech recognition, a rule parser and the offline mock brain.

## Run

```bash
make run                                  # http://127.0.0.1:8777, synthetic person unless you add --webcam
make run ARGS="--mock"                    # start with the offline mock brain (gestures + narration)
make run ARGS="--https --webcam"          # also https://<lan-ip>:8443 so a phone can use its mic and camera
make run ARGS="--byok"                    # public deployment: never use the server's keys; visitors bring their own
make run PORT=8935                        # another port
```

## Test

```bash
make lint                 # ruff check (pps, scripts, tests)
make test                 # pytest unit suite: pure functions, no server, under a few seconds
make selftest             # end-to-end: starts an isolated server and drives every feature over the WebSocket
PC_SELFTEST_OFFLINE=1 PC_SELFTEST_PORT=8935 .venv/bin/python scripts/selftest.py   # as CI runs it: no keys
```

CI (`.github/workflows/ci.yml`) runs ruff and pytest, the offline self test on headless Linux (MuJoCo through
OSMesa), and a gitleaks secret scan. A PR must be green on all three.

* Unit tests go in `tests/test_<module>.py`. Keep them pure and fast: no server, no network, no keys, no camera.
* Behaviour that needs the running server goes in a check under `scripts/checks/` (see below).
* Lint: `ruff check` must pass. New files follow `ruff format`. `make fmt` formats `tests/`. The older modules keep
  their compact style for now, so don't reformat a whole file in a feature PR.

## Extension points

Most features never touch `pps/app.py` or `web/index.html`.

### Add a robot

1. Put the MJCF model somewhere setup provides: `vendor/mink/examples/<robot>/`, or `assets/<robot>/` for a model
   we ship. Check its license and add it to `NOTICE`.
2. Add a `RobotProfile` to `PROFILES` in `pps/world/profiles.py`. Set the model path, the mount (`on: floor` or
   `on: surface`), the end-effector sites by role (a single arm uses `"right"`), the home keyframe, the base
   (`legged`, `fixed` or `mobile`) and the capabilities. Optionally add `ALIASES`.
3. Write `pps/world/prompts/<id>.md`, the LLM prompt for this body. Write `pps/world/prompts/<id>.json` with
   `{label, kind, tagline, examples, tools}` for the UI.
4. Run `.venv/bin/python scripts/sync_robot_prompts.py` to regenerate `web/robots/catalog.json`.
5. Run `make test`. `tests/test_profiles.py` checks that the profile is complete, its capabilities are known, its
   prompts exist and the catalog is in sync. Then run `make selftest` (`scripts/checks/robots.py` and
   `scripts/checks/world.py` build every robot).

### Add a scene template

1. Add an entry to `TEMPLATES` in `pps/world/scene.py` with `label`, `top` (work-surface height) and the `x`/`y`
   extent. The near edge stays at `x = 0.22`.
2. Draw its furniture in `_template_xml()`. The main work surface must be the geom named `table`, because the
   collision pairs and the validator refer to it.
3. The named regions (`left`, `center`, `back`…) follow from the extent automatically. `tests/test_scene.py` checks
   them for every template.

### Add a feature (server + UI)

Server side, copy `pps/features/_example.py` to `pps/features/<feature>.py`. Every module there is imported at
startup (`pps/ext.py`) and registers itself with decorators:

```python
from .. import ext

@ext.handler("zone")        # browser message {"type": "zone", ...}
def on_zone(ctx, msg): ...

@ext.pre_task               # sees every {"type": "task"} first; return True to consume it
def preview_first(ctx, msg): ...

@ext.state                  # merged into every state frame
def zones_state(ctx): return {"zones": ctx.store.get("zones", [])}

@ext.tick                   # once per worker frame, after physics
def step(ctx): ...

@ext.overlay                # draw into the robot-eye render (fn(ctx, mjvScene))
def paint(ctx, scn): ...
```

`ctx` carries the runtime (`rt`), `tasks`, `teacher`, `brain`, `demo`, `spatial`, the camera parameters, a `store`
dict for your state, `push(msg)` to queue a message as if a browser had sent it, and `dispatch(msg)` to run a task
message straight away. An exception in a feature is
logged and never stops the robot loop.

Browser side, add `web/ui/<feature>.js`. It is listed by `/ui/modules.json` and loaded after `web/ui/core.js`. Use
the `PCUI` registry (`PCUI.onState`, `PCUI.send`, `PCUI.slot("tools"|"plan"|"stage"|"timeline"|"side")`,
`PCUI.tool({...})`, `PCUI.with3D`). See the header of `web/ui/core.js`.

Document new messages and state keys in the module docstring and in `docs/PROTOCOL.md`.

### Add a check

Copy `scripts/checks/_example.py` to `scripts/checks/<feature>.py` and define `async def run(t)`. The self test
calls it with `t.send(**msg)`, `t.pump(seconds)`, `t.task(action, wait, **kw)`, `t.until_done()`,
`t.check(name, ok, detail)`, `t.state()`, `t.frames()`, `t.http` and `t.base`. Every feature needs a check. Checks
that need an API key must skip cleanly when no key is configured, because CI has none.

## Branches, commits and pull requests

* Branch from the current integration branch (`v1/integration` during v1, otherwise `main`). Name the branch
  `<area>/<short-topic>`, for example `v1/oss` or `fix/scene-overlap`.
* Keep each PR to one topic, and link the issue (`Refs #12`, or `Closes #12` when it finishes the issue).
* Write commit messages in the imperative mood, with a short subject line and a body that says *why*.
* Fill in the PR template: what changed, why, how you tested it (lint, unit tests, self test), and screenshots for
  UI changes.
* Never commit `.env`, keys, `certs/`, recordings, datasets or large binaries (anything over about 1 MB needs a
  reason). Models and robot assets are fetched by setup.
* Use ports other than the defaults (8777 and 8443) when you run a second server for tests.

## Code of conduct and security

Everyone taking part follows the [Code of Conduct](CODE_OF_CONDUCT.md). Report vulnerabilities privately as
described in [SECURITY.md](SECURITY.md), not in public issues.

By contributing, you agree that your contributions are licensed under the Apache License 2.0 (see `LICENSE`).
