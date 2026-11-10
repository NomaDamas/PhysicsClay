"""The person the robot works with, understood from their pose (v1 P3, issue #31).

model.py     the person in the robot's world frame (pose, hands, held tool, speed) -> state.human
activity.py  rule-based activity, phase, reps and stalls from joint angles and hand paths
coach.py     form references and short spoken cues
rules.py     respond rules: predicates over state.human and an offline phrase compiler
demo.py      demonstrations recorded in learn mode (recordings/demos/) and a stub trajectory extractor
vlm.py       optional VLM summary of what the person is doing (Gemini / OpenAI vision, opt-in)
synth.py     synthetic people for checks and demos (no camera)

The feature that uses them is pps/features/watch.py; docs/PROTOCOL.md "v6 · Watching the person" has the messages.
"""
