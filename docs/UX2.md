# PhysicsClay UX 2.0 — "Say · Point · Show. The robot proposes, you correct."

UX 1 asked people to author robot motion (draw a path, place waypoints). UX 2.0 flips it: you give intent the most
natural way (voice, a point, a demonstration, a photo of the result) and the **robot shows its plan back before it
moves**. You only step in where it's wrong. The 3D stage is mainly for the robot to show what it understood, not for
you to draw in.

Words carry *what* and *why*. Everything else needs another channel:

| What words can't carry | Channel | Feature |
|---|---|---|
| where / which one | point, tap, robot proposes candidates | Candidates (#C) |
| how to move (path, approach) | robot proposes, you drag the wrong part | Plan preview (#B) |
| what not to touch | paint a keep-out zone, or say "stay away from X" | Zones (#D) |
| what "done" looks like | goal photo, or arrange ghosts in 3D | Goal (#E) |
| style / timing / steps | show it on camera or video, edit step cards | Teach timeline (#F) |
| live correction (noisy room, hands busy) | gestures, nudge pad | Gestures & nudge (#G) |
| what the robot understood | 3D ghost + plan card, before anything moves | Plan preview (#B) |

## Layout (desktop, ≥ 1100 px)

```
┌ top bar: logo · one-liner ·· robot status pill ·· demo pill · ⚙ ─────────────────────────────┐
│ YOU (320)            │ STAGE (fluid)                                   │ ROBOT'S PLAN (360)   │
│ ┌ camera (always on  │  3D shared space (default) | Robot eye toggle   │ slot "plan"          │
│ │ screen, overlays)  │  slot "stage" overlays (badges, hints)          │ intent · target ·    │
│ └ source chip        │                                                 │ steps · constraints  │
│ conversation feed    │                                                 │ verdict + why        │
│                      │  slot "timeline" (teach steps / skills strip)   │ [Run] [Fix] [Cancel] │
│                      │  ┌ dock: (● Talk) [type an instruction…] ┐      │                      │
│                      │  └ slot "tools": Point Path Zone Goal Show ┘    │                      │
└──────────────────────┴─────────────────────────────────────────────────┴──────────────────────┘
```

* Phone (< 760 px): stage full width; camera as a small picture-in-picture at the top left; plan card as a bottom sheet
  that peeks; the dock is fixed to the bottom; tools scroll horizontally.
* The camera panel is always visible. The camera itself is **off by default** (opt-in in settings).

## Look

Optimum Zone light design (existing tokens in `web/style.css` `:root`): ground `#fbfbfd`, ink `#1d1d1f`,
accent indigo `#5c77df`, hot orange `#f59145`, ok `#1f8f55`, bad `#d6453a`, cyan `#1da8e5`. SF/system sans with
Pretendard and Noto Sans KR for Korean. Cards have 18–24 px radii, soft shadows, 44 px touch targets, no emoji, and
stroke icons from `PCUI.icon()`.

## Plan card states (slot "plan", owned by #B)

1. **idle**: "Tell me, point, or show me." plus 3 example chips.
2. **proposing**: intent sentence, target chip, numbered steps, constraint chips (avoid vase, keep upright, zone A),
   verdict (green "I can do this" / red "I can't — 13 cm too low") and [Run] [Fix] [Cancel].
3. **needs you**: one question with candidate chips (#C) or a yes/no.
4. **running**: step progress, Stop.
5. **done / failed**: result, why, and the nearest thing it can do.

## How features plug in (no edits to index.html, minimal edits to app.py/app.js)

* Server: `pps/features/<name>.py` using `pps/ext.py` (`@ext.handler`, `@ext.pre_task`, `@ext.state`, `@ext.tick`).
* Web: `web/ui/<name>.js` using `window.PCUI` (`slot`, `tool`, `onState`, `send`, `with3D`) and `PC3D.layer/capture`.
* Checks: `scripts/checks/<name>.py` with `async def run(t)`, run by `scripts/selftest.py`.
* Protocol additions go in `docs/PROTOCOL.md` under "v4 (UX 2.0)".
