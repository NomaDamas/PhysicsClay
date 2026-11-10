# Security policy

## Reporting a vulnerability

Please **do not open a public issue** for a security problem. Report it privately through GitHub's security
advisories: go to the repository's **Security** tab and choose **Report a vulnerability**
(https://github.com/NomaDamas/PhysicsClay/security/advisories/new).

Include what you found, how to reproduce it, and the impact you expect. We aim to acknowledge a report within
7 days and to agree a disclosure date with you once a fix is ready. Fixes land on `main`; there are no maintained
release branches yet.

## Scope

In scope: the PhysicsClay server (`pps/`), the web client (`web/`), the scripts in `scripts/`, and the CI
configuration in this repository. Out of scope: bugs in third-party components (MuJoCo, mink, MediaPipe,
Ultralytics, three.js…), which should be reported upstream. Tell us anyway if PhysicsClay uses one of them in an
unsafe way.

## API keys and secrets

* **Keys live only in `.env`**, which is git-ignored. Never commit `.env`, keys, tokens, `certs/` (the generated
  TLS key) or recordings that show a key on screen. CI runs a gitleaks scan over the full history on every push and
  pull request.
* The server never sends its keys to the browser, never logs them and never puts them in the state frame. The
  self test checks this (`scripts/checks/agent.py`).
* If you committed a key by accident, **revoke it at the provider first**. Removing the commit does not make the
  key safe again. Then tell the maintainers so the history can be cleaned.

## Public deployments: bring your own key

Run a deployment that strangers can reach with **`--byok`**:

```bash
.venv/bin/python -m pps.app --byok --https
```

With `--byok` the server's own keys are never used, even when `.env` exists. Each visitor enters their own key in
the browser's Settings. That key is stored only in their browser and is sent only with the requests that need it.
The server does not store or log it. LLM endpoints that a visitor supplies must be public `https` URLs, so the server
cannot be used to reach private addresses (SSRF). Without `--byok`, anyone who can open the page spends your keys.

Other things to consider for a public deployment:

* The app has no user accounts. Anyone who can reach the port can drive the simulated robot, upload videos and
  start demos. Put it behind a reverse proxy with authentication, or bind it to a private network, if that matters
  to you.
* `--https` uses a self-signed certificate (`certs/`). For a real domain, terminate TLS at a reverse proxy with a
  proper certificate.
* `--webcam` opens the server machine's camera. Leave it off on shared machines.
