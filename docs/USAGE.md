# Using PhysicsClay

English · [한국어](USAGE.ko.md)

PhysicsClay lets you instruct a simulated robot the way you would instruct a person: **say it, point at it, show it**.
The robot shows its plan before it moves, and you say "go" or fix only the part that is wrong.
In the app, the **Help** button (**?**) in the top bar opens a short version of this guide (English / 한국어).

[Quick start](#quick-start) · [The idea](#the-idea-in-one-minute) · [Instruct](#3-ways-to-instruct) ·
[Plan card](#the-plan-card) · [Dock tools](#dock-tools) · [3D view](#3d-view-controls) · [Trajectories](#trajectories) ·
[Watch](#watch-modes) · [Camera](#camera-and-phone-camera) · [Gestures](#gestures) · [Packs](#demo-packs) ·
[Datasets](#recording-a-dataset) · [VR / MR](#vr--mr-quest) · [Safety](#safety) · [Voice](#voice-and-language) ·
[Settings](#settings) · [Keys](#keyboard-shortcuts) · [Examples](#example-commands) · [Troubleshooting](#troubleshooting)

## Quick start

You need macOS or Linux with [uv](https://docs.astral.sh/uv/), git and curl.

```bash
git clone https://github.com/NomaDamas/PhysicsClay && cd PhysicsClay
make setup                  # .venv, Python deps, robot models, MediaPipe models
make run ARGS="--https"     # http://127.0.0.1:8777 here, plus https://<LAN IP>:8443 for phones and Quest
```

Open **http://127.0.0.1:8777** in Chrome, Edge or Safari on the same computer. The terminal prints the phone and
Quest addresses.

| You want | Do this |
|---|---|
| Laptop only | Plain `make run` is enough (http on 8777; browsers allow the camera on 127.0.0.1). |
| Phone camera or Quest | Keep `--https`. Plain `make run` has **no HTTPS**. Port 8443, self-signed certificate: accept the warning once. |
| Other ports | `make run PORT=9000 ARGS="--https --https-port 9443"` |
| No API keys | It works: an offline rule parser and the browser's speech recognition. Add keys later in Settings. |
| Offline demo brain | `make run ARGS="--https --mock"` (scripted replies and gestures, no keys). |
| A public server | `make run ARGS="--https --byok"`: visitors bring their own keys ([SECURITY.md](../SECURITY.md)). |
| A guided tour | Press **Demo** in the top bar. |

## The idea in one minute

**Say · Point · Show. The robot proposes, you correct.**

1. **You give intent** the natural way: words for *what* and *why*, a point for *where*, a demonstration for *how*.
2. **The robot shows its plan** before anything moves: a plan card (steps, target, constraints) and a translucent
   ghost in 3D. Every waypoint is physics-checked: reach, joint limits, collisions, keep-out zones, people nearby.
3. **You say "go"**, or fix only what is wrong. If it can't do it, it says why ("13 cm too low") and offers the
   nearest thing it can do.

| Area | What is there |
|---|---|
| Top bar | robot pill (click it: **Robot & scene**), Feasible meter, **Record**, **Demo**, **Help**, **Settings** |
| You (left) | your camera, **Gestures**, **Show a video**, the conversation |
| Stage (centre) | 3D shared space or **Robot eye**; below it the dock: Talk, command box, Stop, More (…), tools |
| Robot's plan (right) | plan card, Watch card, zones, the loaded pack, robot status |

On a phone-sized screen the stage is full width, the camera is a small picture-in-picture and the plan is a bottom sheet.

## 3 ways to instruct

| Way | How | Best for |
|---|---|---|
| **Say** | Press **Space** or the mic button and speak, or type in the command box and press Enter. | what to do, constraints ("slowly", "stay away from the vase") |
| **Point** | **Point** tool: click the 3D table. **Robot eye**: tap a spot, or drag to draw a path. Camera on: point with your arm and say "put it there". | where, which one |
| **Show** | **Show** tool: do the task on camera and explain it. **Show a video**: a humanoid imitates the clip. **Watch → Learn**: show a motion once. **Goal**: show the end state. | how, style, timing, the result |

Who understands your words (the chip in the Conversation header tells you):

* **LLM agent** (when a key is set): free requests in English or Korean, multi-step tasks, packs, paths from words.
* **Offline parser** (no key): common phrases in English and Korean. See [Example commands](#example-commands).
* Always handled locally, whichever brain is on: "go" / "cancel" for a plan, "which one?" answers, keep-out zones,
  teaching.

## The plan card

Motion requests you type, say or tap (move, reach, follow a path, play a skill) are **proposed first**. Buttons and
keyboard shortcuts act directly.

| Phase | You see | You can |
|---|---|---|
| Proposing | intent, target chip, numbered steps, constraint chips, a green verdict, a ghost in 3D | **Run** · **Fix** · **Cancel** |
| Needs you | a question and the nearest alternative | accept the alternative · **Cancel** |
| Can't do this | the reason in plain words | **Fix** · **Cancel** |
| Running | step progress | **Stop** |
| Done / Didn't finish | the result and why | **OK** |

* **Run**: press Run, say "go", "yes", "do it" ("네", "좋아", "가자", "실행해"), or show the *Go on* gesture.
* **Fix**: puts the cursor in the command box. Say what to change ("put it on the left instead"), point at a new spot,
  or drag the wrong part of the ghost path.
* **Cancel**: "no", "cancel" ("아니", "취소").
* **Which one?** If your words fit several objects, numbered chips appear on the stage. Tap one, say "two" or "the red
  one" ("두 번째", "빨간 거"), press **1–4**, or point at it.
* **Nudge pad**: while a task runs or a marker, plan or path exists, its arrows move the target 2 cm and its speed
  buttons set the speed.
* A skill plan adds quick corrections: Wider · Narrower · Slower · Faster …
* To act without previews: Settings → Agent brain → untick "Confirm before moving" (agent), or open `/?preview=0`.

## Dock tools

| Tool | Use it to | How |
|---|---|---|
| **Point** | say *where* | Click a spot on the 3D table: a marker (A–F) appears. Then "put the mug there" or "on marker B". |
| **Zone** | say *not here* | Drag a rectangle on the table: a red keep-out volume. Or say "stay away from the vase". Zones are listed on the right, removable. |
| **Goal** | show the result | **Arrange in 3D**: drag ghost copies of the objects, then **Make it so**. **From a photo**: a photo of the layout you want. |
| **Show** | teach a task | Starts *Teach by showing*: do the task on camera and say what matters, then "done". The robot lists the steps it understood, asks about anything unclear, practises in sim; you correct and save it as a skill. |
| **Path** | draw a route | Click 2–5 spots: ghost grippers connect into a checked path. Drag them to adjust ([Trajectories](#trajectories)). |

Some robots add **Handover** (pass an object between two arms) and **Drive** (mobile bases: point first, then
Drive); both ask the LLM agent. Change the robot or the scene from the robot pill in the top bar.

## 3D view controls

| Do | Mouse / trackpad | Touch |
|---|---|---|
| Orbit | drag | one-finger drag |
| Pan | right-drag | two-finger drag |
| Zoom (smooth) | wheel or trackpad scroll | pinch |
| Presets | **Stage** · **Orbit** · **Top** · **Side** · **↺** (reset) | same |

* **3D / Robot eye** (top left of the stage) switches between the shared 3D space and the robot's head camera. The
  small picture-in-picture swaps them too.
* In **Robot eye**: tap to mark a spot, drag to draw a path.

## Trajectories

A proposed path is a ghost gripper riding along a tube with numbered handles: **blue** = fine, **red** = a point
fails a check (reach, collision, zone, safety). Paths come from words (agent), a demonstration, a stroke in Robot eye,
the Path tool, or VR.

| Do | How |
|---|---|
| Move a point | drag its handle (it slides on its horizontal plane) |
| Change its height | **Alt**-drag the handle, or turn the wheel over it (5 mm a notch, Shift 2 cm), or the ↕ chip |
| Turn the wrist | select the point and drag the knob on the ring around it |
| Tilt, speed, force, grip, tolerance | chips in the point's popover |
| Add keyframes | **Path** tool: click 2–5 spots |
| Fix it by voice | "higher there", "slower near the vase" (LLM agent) |
| Run it | **Run** on the plan card, or say "go" |

Keyboard, with a point selected: arrows move it 1 cm in the view's directions (Shift 5 cm), **PageUp / PageDown**
change its height, **[** and **]** turn the wrist 15°, **Delete** removes it, **Esc** deselects.

## Watch modes

The **Watch** card (right column) lets the robot act on what you *do*. Pick a mode chip or say it. It needs your
camera, or a pack's synthetic demo.

| Mode | What the robot does | Example |
|---|---|---|
| **Mirror** | copies you live: the G1 copies your whole body, an arm follows your hand; pinch or a fist closes the gripper | "copy me" |
| **Learn** | records one demonstration; "done" saves it (the weld cell extracts a skill); "your turn" repeats it as a proposed path | "watch me" … "done" … "your turn" |
| **Respond** | runs your standing rules, "when …, …" | "when I put my hand out, give me the wrench" |
| **Assist** | follows under your hands and pushes, force-limited, when you stall (spotting a lift) | "spot me" |
| **Coach** | counts reps and gives one short cue per rep (depth, knees, back, tempo) | "coach me" |

* "stop watching" turns it off. Rules are also checked in Assist and Coach.
* The card shows the live reading (activity, phase, reps, held tool), the rules (deletable) and a rule box.
* The **AI** chip adds a one-line description from a vision model. It is opt-in: it sends a camera frame every few
  seconds and needs a server key.
* More: [docs/skills.md](skills.md).

## Camera and phone camera

The camera is **off by default**.

**This computer's camera**

1. In the **You** column, press **Turn on camera**. Allow the camera when the browser asks.
2. You see yourself, and a grey mannequin of you appears across the table in the 3D stage and in Robot eye. The robot
   now sees where you point and what you do.

**Your phone as the camera** (useful when the laptop can't see your whole upper body)

1. Start the server with `--https` ([Quick start](#quick-start)).
2. Press **Use my phone**. A QR code and an address appear: `https://<laptop LAN IP>:8443/phone`.
3. On the phone, on the **same Wi-Fi**, scan the code or type the address.
4. Accept the certificate warning once. Safari: **Show Details → visit this website**. Chrome: **Advanced → Proceed**.
5. Tap **Start camera** and allow it. The laptop shows "Phone connected · streaming 15 fps".
6. Stand the phone where it sees you from the waist up, facing you.

| Problem | Fix |
|---|---|
| The phone can't open the page | Server started with `--https`? Use `https://`, port 8443, the LAN IP the terminal printed, the same Wi-Fi (guest networks often isolate devices). |
| "camera not available (needs https)" | You opened `http://<IP>:8777`. Use the https address, or 127.0.0.1 on the laptop itself. |
| Permission denied | Allow the camera in the site settings (lock icon in the address bar) and reload. macOS: also System Settings → Privacy & Security → Camera for your browser. |
| "No person detected", gestures never fire | Step back until your head, shoulders, elbows and hands are in view; face the camera; add light. |
| The mannequin or your pointing is off | Face the camera squarely and keep only one person in view. |

## Gestures

Press **Gestures** under the camera for the guide card. The camera must be on, with your upper body in view.

| Gesture | How | What happens |
|---|---|---|
| **Stop** | both hands above your head | the robot stops |
| **Slower** | both palms pressing down in front of your waist | speed ×0.7 |
| **Come here** | beckon with one hand | it reaches toward you, bringing what it holds (during a task: the target moves 4 cm closer) |
| **Go on** | thumb up, or an open palm pushed forward | confirms the proposed plan, or resumes |

* Hold a static gesture still for about half a second (0.5–0.7 s); a beckon is read from the motion.
* A chip on the camera shows the recognised gesture. Each fires once; drop it before you repeat it.
* Paused during the guided demo, teaching, video imitation and mirroring. With **Mock AI** on, the mock brain handles
  Stop itself.

## Demo packs

| Pack | Robot | What it shows | Try |
|---|---|---|---|
| **Weld cell** (`weld_cell`) | UR5e (or Panda) with a MIG torch | two steel butt joints, a straight seam and an L seam. Show the weave with your hand; it welds in your style and the bead grows along the torch tip's path. No arc physics. | "watch me" … "done", "a bit wider", "go", "now weld the L seam like that" |
| **Gym** (`gym`) | G1 (or Apollo) | a bench and a 20 kg barbell. Counts your squats and corrects form; spots your lift. | "coach me", "spot me", "help me earlier" |
| **Workshop helper** (`workshop`) | G1 (or any robot that picks) | a wrench, a screwdriver and a brush in stands; a standing rule hands you the wrench | hold an open hand out over the table |

How to load one:

1. Click the robot pill in the top bar → **Robot & scene** → **Scene** tab → **Demo packs**. (More (…) → **Robot &
   scene…** opens the Scene tab directly.)
2. Or say "set up a weld cell" (LLM agent).
3. The pack card on the right shows what to say and **No camera? Play a demo** (a synthetic person).

Picking another scene template leaves the pack. Details: [docs/packs.md](packs.md), [docs/skills.md](skills.md).

## Recording a dataset

1. Press **Record** in the top bar, name the dataset, press **Start**. It shows `REC · 0 ep`.
2. Instruct the robot as usual. Each task that starts moving is one episode, until it is done or fails. Tasks refused
   before moving are counted, not recorded.
3. **Discard last** deletes a bad take. **Stop** ends recording (a task still running is dropped).

Output: LeRobot v2.1 in `datasets/<name>/<robot>/`: joint states, actions, front and wrist video (256×256, 30 Hz),
your words as the task, success labels and your pose. Needs `pyarrow` (installed by `make setup`) and `ffmpeg`.
Format and headless generation: [docs/datasets.md](datasets.md).

## VR / MR (Quest)

1. `make run ARGS="--https"`. The terminal prints a `Quest :` line with `https://<LAN IP>:8443`.
2. Quest on the same Wi-Fi → Quest browser → that address → **Advanced → Proceed** once.
3. Next to **3D / Robot eye**, press **Enter VR** or **Enter MR**. Allow hand tracking (and spatial data for MR).

| In the headset | Hands | Controllers |
|---|---|---|
| Draw a path | pinch and move | hold the trigger |
| Add a keyframe | a quick pinch in empty space | a quick trigger pull |
| Move a point, turn the wrist | pinch a handle, move it, twist your hand | trigger on a handle |
| Drive the robot's hand live | make a fist (pinch to close its gripper) | hold the grip button |
| Menu | turn your left palm toward you | on the left controller |

* **MR**: pinch your real table's near-left corner, then its near-right corner: the robot stands on your table.
  **Scan → Use my room** turns what is on your table into obstacles.
* Nothing runs until **Go** (menu) or "go". Safety applies in VR too.
* Without the certificate: USB + `adb reverse tcp:8777 tcp:8777`, then `http://localhost:8777`.
  Full guide: [docs/vr.md](vr.md).

## Safety

What the robot does near you:

* **Slows down** when any part of you is within about half a metre of any part of it.
* **Stops** before it would touch you, sooner near your head, neck and torso. A task pauses with a plain message
  ("Paused for safety: you're 9 cm from my gripper") and **resumes by itself** once you have stepped back for a second.
* Won't plan a reach or pick whose target is inside your body zone. Never touches your head, neck or body.
* In Assist, Mirror and handovers your hands may come close, at a crawl; a handover stops 10 cm short of your hand.
* Shows your safety outline in 3D (amber = slowing, red = stopped) and one line above the plan card. A force-limit
  stop needs **I'm clear** once you are clear.
* **Cannot be turned off**, in the app or in the config.
* **Not a certified safety system.** Never use it to guard a real robot. Details: [docs/safety.md](safety.md).

## Voice and language

* **Talk**: **Space** or the mic. With a Gemini API key (Settings → AI) it is a live voice conversation with Gemini
  Live, which also sees your camera when "Send my camera to Gemini" is on. Without a key the browser's speech
  recognition is used (Chrome, Edge or Safari) in the language of Settings → AI → **Speech language**.
* **Language**: English and Korean both work. The LLM agent and Gemini Live answer in the language you use; the
  offline parser's replies are in English.
* **The robot's voice**: Settings → **Voice**: pick a voice for English and for Korean, press **Test**. With an
  OpenAI key (Settings → AI) the robot uses the natural OpenAI voice; otherwise the best voice of your browser or OS.
* **Better system voices**: macOS: System Settings → Accessibility → Spoken Content → System voice → Manage Voices,
  install **Yuna (Premium)** for Korean or **Ava (Premium)** / **Zoe (Premium)** for English. Chrome: "Google 한국어"
  and "Google US English" are good.

## Settings

Press **Settings** (gear) in the top bar.

| Section | What |
|---|---|
| Appearance | System / Light / Dark |
| Voice | the robot's voice per language, Test |
| Robot | G1 humanoid or UR5e arm (all robots and scenes: the robot pill) |
| Camera | use this device's camera, switch front / back |
| Skills | play a learned skill; edit a time range (faster, slower, bigger, smaller) |
| Spatial model | object understanding on your camera (YOLO segmentation, MediaPipe, Off); ask Gemini Robotics-ER |
| AI | Gemini API key, OpenAI API key (robot voice), models, what to send to Gemini, Mock AI, speech language |
| Agent brain | use the LLM agent, confirm before moving; on `--byok` servers your own LLM base URL, key and model |
| Demo | full demo, text vs spatial, teach demo, stop demo |

**Keys (bring your own key)**

* Keys typed in Settings are stored **only in this browser** and sent only with the requests that need them.
* On your own machine you can put server keys in `.env` (git-ignored): `GEMINI_API_KEY`, `OPENAI_API_KEY`, and for the
  agent `LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL` (any OpenAI-compatible Responses API).
* A server started with `--byok` never uses its own keys; each visitor brings theirs. See [SECURITY.md](../SECURITY.md).

## Keyboard shortcuts

| Key | Does |
|---|---|
| Space | Talk: start / stop listening |
| Esc | stop the robot (first deselects a path point; closes an open panel) |
| / | jump to the command box (Esc leaves it) |
| Enter | send the typed command |
| F | run the drawn path |
| C | clear markers and the drawn path |
| 1–4 | answer a "Which one?" question |
| Arrows | move the selected path point 1 cm (Shift 5 cm) |
| PageUp / PageDown | raise / lower the selected point |
| [ and ] | turn the wrist at the selected point by 15° |
| Delete | remove the selected path point |
| Alt + drag, wheel | change a handle's height |

Shortcuts are ignored while you type in a text field.

## Example commands

"—" = works offline (rule parser or a feature module); **agent** = needs the LLM agent; **camera** = needs your
camera. Motion commands show a plan card first.

| What | English | 한국어 | Needs |
|---|---|---|---|
| Move an object | "Put the mug on the right" | "머그를 오른쪽에 놓아줘" | — |
| Move a named object | "Move the red block to the left" | "빨간 블록을 왼쪽으로 옮겨줘" | — |
| Reach | "Reach to the front" | "앞쪽으로 뻗어" | — |
| Use your marker | "Put the mug there" (after Point) | "머그를 저기에 놔" | — |
| Use your drawn path | "Carry the mug along my path" | "내 경로대로 머그 옮겨줘" | — |
| Run the drawn path | "Follow the path" | "경로 실행해" | — |
| Point with your arm | "Pick this up" / "Give it to me" | "이거 집어" / "이거 줘" | camera |
| Keep out | "Stay away from the vase" | "꽃병 근처에 가지 마" | — |
| Remove zones | "Clear the zones" | "구역 다 지워" | — |
| Confirm a plan | "Go" / "Yes" | "좋아" / "가자" | — |
| Drop a plan | "Cancel" | "취소" | — |
| Answer "Which one?" | "The red one" / "Two" | "빨간 거" / "두 번째" | — |
| Speed | "Slower" | "천천히" | — |
| Stop | "Stop" | "멈춰" | — |
| Home pose | "Go home" | "원위치" | — |
| Path from words | "Take the mug over the vase to the right" | "꽃병 위로 넘겨서 머그를 오른쪽에 놔" | agent |
| Edit the path | "Higher there" | "거기 더 높게" | agent |
| Mirror | "Copy me" | "따라 해" | camera |
| Learn | "Watch me" … "done" … "your turn" | "보고 배워" … "끝" … "이제 해 봐" | camera |
| Coach | "Coach me" | "자세 봐 줘" | camera |
| Assist | "Spot me" | "보조해 줘" | camera; Korean: agent |
| Standing rule | "When I put my hand out, give me the wrench" | "손 내밀면 렌치 줘" | Korean: agent |
| Teach a task | "Let me show you" … "done" | "보여 줄게" … "끝" | camera |
| Load a pack | "Set up a weld cell" | "용접 셀 준비해 줘" | agent |
| Correct a skill | "A bit wider" / "now weld the L seam like that" | "조금 넓게" / "이제 L 심도 그렇게 용접해" | Korean L seam: agent |

## Troubleshooting

| Problem | What to do |
|---|---|
| The robot pill says "connecting" | The server isn't running or was restarted: check the terminal, then reload the page. |
| "I didn't catch an action" | The offline parser didn't recognise it. Rephrase like the [examples](#example-commands), or add an LLM key (Settings → Agent brain, or `.env`). |
| Talk does nothing | Allow the microphone for the site. Without a Gemini key, use Chrome, Edge or Safari. On a phone, use the https address. |
| The phone or Quest can't connect | Run with `--https`; open `https://<LAN IP>:8443` on the same Wi-Fi; accept the certificate once. |
| It can't see you, gestures don't fire | Step back so your upper body and hands are in view, add light, hold the gesture still. Gestures pause during the demo, teaching and mirroring. |
| "Can't do this" on the plan card | Read the reason; take the offered alternative, press **Fix**, draw a route around the obstacle, or move it first. |
| The robot stopped and waits | Safety: step back and it resumes by itself. After a force-limit stop, press **I'm clear**. |
| The 3D stage stays blank or is slow | The 3D stage loads three.js from a CDN (needs internet) and WebGL. The G1 is heavy: try the UR5e or Panda. |

More: [docs/vr.md](vr.md) · [docs/safety.md](safety.md) · [docs/datasets.md](datasets.md) · [docs/packs.md](packs.md) ·
[docs/skills.md](skills.md) · [README](../README.md)
