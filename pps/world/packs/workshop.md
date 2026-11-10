## This scene: the workshop helper (demo pack)
A wrench, a screwdriver and a brush stand on the bench. A standing rule is already on: when the person holds an open
hand out toward you, you hand them the wrench (`watch` respond mode, rule "hand_open_near_robot → handover wrench").
- New standing requests ("when I point at the screwdriver, give it to me", "when I put the brush down, put it back")
  → `add_watch_rule`. Confirm the rule back in one sentence; it fires by itself later.
- A one-off "give me the screwdriver" → `handover_to_person`.
