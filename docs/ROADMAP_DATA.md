# Roadmap: PhysicsClay as a robot-learning data engine

**Pivot thesis.** Robot foundation models (VLAs) are limited by data. Teleoperated demonstrations are slow and
expensive, and one demonstration only covers one robot in one scene. PhysicsClay already captures what a
person means in space (a pointed spot, a drawn route, a keep-out zone, a demonstration, a video). It also
turns that intent into **physically validated trajectories** for many robot bodies. The same machinery can
**generate training data at scale**: show it once, and get thousands of validated, labelled episodes across
robots and scenes.

Two pipelines feed one exporter.

## A. Trajectory → dataset (intent once, many episodes)
1. **Seed.** One intent from the existing UI: a spoken request, a pointed target, a drawn 3D route, zones, a
   goal layout, or a Teach-by-Showing demo.
2. **Vary (domain randomisation).**
   - Object poses, sizes and colours; distractors.
   - Scene templates (table, kitchen, shelf, floor).
   - Lighting, textures and camera poses.
   - Robot bodies (same intent compiled for G1, Panda, UR5e, ALOHA…).
   - Speed and route perturbations.
3. **Generate and validate.**
   - IK plus physics rollout per variation.
   - Reach, joint-limit, collision, balance and zone checks.
   - Success detection (the object ends within tolerance).
   - Failures are kept as labelled negatives or discarded.
4. **Render observations.** Robot-eye and wrist cameras plus third-person cameras: RGB, depth and
   segmentation masks. Also proprioception and actions (joint positions, end-effector deltas, gripper).
5. **Language.**
   - The LLM writes paraphrased instructions per episode (ko / en / …).
   - The spatial prompt itself (points, routes, zones) is kept as an extra modality, so the data teaches
     *spatial* grounding, not only text.

## B. Video → simulation → dataset (real2sim)
1. **Perceive the video** with off-the-shelf models:
   - **Masks:** segmentation of people, hands and objects (SAM 2 / SAM 3).
   - **Motion:** body motion (MediaPipe, WHAM / 4D-Humans) and hand pose (HaMeR / WiLoR).
   - **Depth:** monocular or video depth (Depth Anything family).
   - **Objects:** 6-DoF object pose (FoundationPose).
2. **Rebuild the scene in simulation.** Fit objects with primitives or image-to-3D meshes (TRELLIS /
   Hunyuan3D class), and place the table and support surfaces.
3. **Retarget to robot bodies.** Human body and hands map to each robot's end-effectors (extends today's
   video → G1 retargeting), checked by the same physics validator.
4. **Replay and render.** Then reuse pipeline A's variation, rendering and labelling from step 2 onward: one
   human video becomes many robot episodes.

## Shared: quality, export, proof
* **Quality.**
  - Physics validity, success rate and diversity metrics.
  - Dedup and filtering.
  - Human-in-the-loop fixes using the existing plan card, nudge pad and "which one?" UI, so the person
    corrects only the episodes that failed (active learning).
* **Export.** The LeRobot dataset format (Hugging Face) and RLDS / Open X-Embodiment, with dataset cards
  covering robots, scenes, variation ranges and licences.
* **Proof.**
  - Train small policies (ACT / Diffusion Policy) in simulation on Panda and ALOHA from generated data.
  - Report success against amounts of teleop-only data.

## Milestones
| | Scope | Done when |
|---|---|---|
| M0 | Exporter from the current recorder (IntentRecorder frames + task results) to the LeRobot format; seeded variation generator for pick & place on the table | 100 validated Panda episodes from one spoken request, loadable with `lerobot` |
| M1 | Multi-camera rendering (RGB/depth/masks); language paraphrases; dataset card | Dataset viewer shows synced cameras, actions and instructions |
| M2 | Video → sim MVP: segmentation + body/hand pose + depth → primitive scene + retarget → replay | One kitchen video produces validated episodes on 2 robot bodies |
| M3 | Contact-rich grasping (replace today's kinematic grasp with physical grasps where it matters); scenes and robots at scale | Grasp success measured by physics, not attachment |
| M4 | Policy-training demo + benchmark; "Dataset Studio" UI (seed → vary → review → export); batch generation in the cloud | Sim policy trained on generated data beats a teleop-only baseline |

## Risks and open questions
- **Sim-to-real gap:** rendering realism, physics parameters and contact. Mitigate with randomisation, real
  textures, and real2sim scans.
- **Grasp realism:** today's grasp is kinematic (the object follows the hand). Learning data needs real
  contact (M3).
- **Retargeting fidelity:** dexterous hands and humanoid whole-body motion.
- **Video licensing and consent** for public footage; on-prem processing for customer videos.
- **Compute cost** for rendering and batch IK/physics; cloud workers.

## Why this is the business
Robot teams, integrators and labs pay for **validated demonstrations per robot**. PhysicsClay turns one human
intent or one video into many, across bodies, so the price is per validated episode or dataset rather than per
seat. The interface (say · point · show; the robot proposes, you correct) becomes the data-collection and
data-repair front end.
