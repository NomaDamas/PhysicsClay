# PhysicsClay documentation

Start with the [README](../README.md) ([한국어](../README.ko.md)) for what PhysicsClay is and the quickstart. What
changed in each release is in the [CHANGELOG](../CHANGELOG.md).

## Using it

| Guide | What it covers |
|---|---|
| [USAGE.md](USAGE.md) · [USAGE.ko.md](USAGE.ko.md) | The user guide: the three ways to instruct, the plan card, dock tools, the 3D view, trajectories, Watch modes, camera and phone camera, gestures, packs, datasets, VR, safety, voice and language, settings, keys, example commands, troubleshooting |
| [worlds.md](worlds.md) | Generated homes and industrial sites: templates, fixtures (drawers, doors, conveyors), objects, multi-step tasks, what works per robot, adding your own |
| [RECON.md](RECON.md) | Real photos → simulated objects and spaces: recognition, outline, 3D shape, metric scale, physics prep, providers and licenses, accuracy |
| [vr.md](vr.md) | VR / MR on a Meta Quest: drawing in the air, passthrough alignment, room scan, live teleop, testing without a headset |
| [skills.md](skills.md) | Skills from demonstration: kinds, extraction, corrections, messages |
| [packs.md](packs.md) | Demo packs (weld cell, gym, workshop helper) and how to add one |
| [safety.md](safety.md) | Safety around people: what the robot does, how it works, the contact policy, configuration, limitations (not a certified safety system) |
| [datasets.md](datasets.md) | Recording LeRobot v2.1 datasets, the layout, generating randomized episodes, loading, training, what is honest about the data |
| [calibration.md](calibration.md) | Real-to-sim calibration: parameters, probes, system identification, the hidden-robot proof, domain randomization, the driver interface |

## Reference

| Document | What it covers |
|---|---|
| [architecture.md](architecture.md) | How the server, the worker loop, features, the world and runtime, physics, the agent and the web client fit together |
| [PROTOCOL.md](PROTOCOL.md) | The browser ↔ server protocol: every message and state key, by version (v3 … v17) |
| [WORLD.md](WORLD.md) | The UX 3 contract: robot profiles, scene specs, the agent (world model and brain) |
| [../CONTRIBUTING.md](../CONTRIBUTING.md) | Setup, tests, extension points (a robot, a scene, a feature, a check), PR conventions |
| [../SECURITY.md](../SECURITY.md) | Keys, bring-your-own-key deployments, reporting vulnerabilities |
| [../NOTICE](../NOTICE) | Third-party models, assets, libraries and services with their licenses |

## The v1 contract

[V1.md](V1.md) is the contract between the v1 workstreams (physics, datasets, trajectories, VR, watching the person,
skills, safety, calibration, open source, worlds) with an "as built" section for each: what shipped, deviations, measured
results and limitations. It starts with a status table.

## History

| Document | What it is |
|---|---|
| [hackathon/](hackathon/README.md) | The hackathon pitch script, stills and the 2-minute demo video |
| [UX2.md](UX2.md) | The UX 2.0 design: "Say · Point · Show. The robot proposes, you correct." |
| [UX_SCENARIOS.md](UX_SCENARIOS.md) | UX scenarios (Korean): the moments words alone can't handle |
| [ROADMAP_DATA.md](ROADMAP_DATA.md) | The data-engine roadmap: PhysicsClay as a source of robot-learning data |
| [ux2/](ux2/) | UX 2.0 screenshots and design canvases |
