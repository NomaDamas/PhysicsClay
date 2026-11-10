"""Korean server messages: watch (pps/features/watch.py, pps/human/*.py, pps/features/skills.py, pps/skills/*.py,
pps/features/safety.py, pps/safety.py). English is the message id; see pps/i18n.py."""

MESSAGES = {
    # key: (English template, Korean template)

    # ------------------------------------------------------------------ pps/features/watch.py: watch modes
    "watch.mode_off": ("Stopped watching.", "그만 볼게요."),
    "watch.mode_mirror_copying": ("Copying you.", "따라 할게요."),
    "watch.mode_mirror_move": ("Move your hand; I'll follow.", "손을 움직여 보세요. 제가 따라갈게요."),
    "watch.mode_learn_recording": ("Watching and recording.", "보면서 기록하고 있어요."),
    "watch.mode_learn_show": ("Show me, then say done.", "보여 주시고 'done'(또는 '끝')이라고 말하세요."),
    "watch.mode_respond": ("Watching for your cues.", "신호를 기다리며 보고 있어요."),
    "watch.mode_assist": ("Following you; I'll help if you get stuck.", "따라가다가 막히시면 도와드릴게요."),
    "watch.mode_coach": (
        "Coaching: I'll count and correct your form.",
        "코칭할게요: 횟수를 세고 자세를 바로잡아 드릴게요.",
    ),
    "watch.already_off": ("Already in off.", "이미 꺼져 있어요."),
    "watch.already_mirror": ("Already in mirror.", "이미 따라하기 모드예요."),
    "watch.already_respond": ("Already in respond.", "이미 반응 모드예요."),
    "watch.already_assist": ("Already in assist.", "이미 보조 모드예요."),
    "watch.already_coach": ("Already in coach.", "이미 코칭 모드예요."),
    "watch.already_mode": ("Already in {mode}.", "이미 {mode} 모드예요."),
    "watch.unknown_mode": ("Unknown mode '{mode}'. Modes: {modes}.", "모르는 모드예요: '{mode}'. 모드: {modes}."),
    "watch.stopped_said_stop": ("Stopped watching: you said stop.", "그만 볼게요: 멈추라고 하셨어요."),
    "watch.unknown_op": ("Unknown watch op '{op}'.", "모르는 watch 동작이에요: '{op}'."),

    # ------------------------------------------------------------------ mirror: the gripper follows your pinch
    "watch.mirror_grasped": ("Grasped the {obj} with you.", "함께 {obj|을} 잡았어요."),
    "watch.mirror_let_go": ("Let go of the {obj}.", "{obj|을} 놓았어요."),

    # ------------------------------------------------------------------ assist (spotting)
    "watch.assist_bar_slow": (
        "The bar slowed below {v} cm/s: helping, capped at {f} N.",
        "바가 {v} cm/s 아래로 느려졌어요: 최대 {f} N으로 도울게요.",
    ),
    "watch.assist_stalled": ("You stalled: helping, capped at {f} N.", "멈추셨네요: 최대 {f} N으로 도울게요."),
    "watch.assist_push": ("I've got you. Push.", "제가 받칠게요. 미세요!"),

    # ------------------------------------------------------------------ coach
    "watch.coach_squats": ("Squats. I'll count.", "스쿼트네요. 제가 셀게요."),
    "watch.coach_curls": ("Curls. I'll count.", "컬이네요. 제가 셀게요."),
    "watch.coach_presses": ("Presses. I'll count.", "프레스네요. 제가 셀게요."),
    "watch.coach_weave": ("I'm watching your weave.", "위빙을 보고 있어요."),
    "watch.coach_rep_cue": ("rep {n}: {cue}", "{n}회: {cue}"),
    "watch.coach_ref_demo": ("Coaching against demo {name}.", "시범 '{name}' 기준으로 코칭할게요."),
    "watch.coach_ref": ("Coaching against {source}.", "{source} 기준으로 코칭할게요."),
    "watch.coach_ref_builtin": ("Coaching against the built-in form rules.", "기본 자세 규칙을 기준으로 코칭할게요."),
    "watch.coach_ref_bad": ("I couldn't use that demo as a reference.", "그 시범은 기준으로 쓸 수 없었어요."),
    "watch.reps_reset": ("Reps reset.", "횟수를 초기화했어요."),
    "watch.ai_on": ("AI description on.", "AI 설명을 켰어요."),
    "watch.ai_off": ("AI description off.", "AI 설명을 껐어요."),

    # ------------------------------------------------------------------ respond rules
    "watch.rule_format": (
        "Say it as 'when I …, do …' (e.g. 'when I put my hand out, give me the wrench').",
        "'when I …, do …' 형식으로 말해 주세요 (예: 'when I put my hand out, give me the wrench').",
    ),
    "watch.rule_needs": ("A rule needs a when and a do.", "규칙에는 조건(when)과 할 일(do)이 둘 다 있어야 해요."),
    "watch.rule_max": (
        "I keep at most {n} rules; remove one first.",
        "규칙은 {n}개까지만 둘 수 있어요. 먼저 하나를 지워 주세요.",
    ),
    "watch.rule_unknown_when": (
        "I can't watch for “{when}” on my own. Known: hand_open_near_robot, holding:<tool>, "
        "activity:<squat|curl|press|wave|weave|reach>, phase:<top|down|bottom|up>, reps>=N, stalled, still, pointing "
        "(or an expression over state.human).",
        "저 혼자서는 이 조건을 지켜볼 수 없어요: “{when}”. 아는 조건: hand_open_near_robot, holding:<tool>, "
        "activity:<squat|curl|press|wave|weave|reach>, phase:<top|down|bottom|up>, reps>=N, stalled, still, pointing "
        "(또는 state.human에 대한 식).",
    ),
    "watch.rule_bad": ("That rule doesn't work: {e}.", "그 규칙은 안 돼요: {e}."),
    "watch.rule_ok": ("Okay: when {when}, I'll {do}.", "알겠어요. “{when}” 하면 “{do}” 할게요."),
    "watch.rule_new": ("New rule {id}: when {when} → {do}", "새 규칙 {id}: {when} → {do}"),
    "watch.rule_fired_agent": ("{what} → asked the agent: {do}", "{what} → 에이전트에게 부탁: {do}"),
    "watch.rule_fired_said": ("{what} → said “{say}”", "{what} → “{say}”라고 말함"),
    "watch.rule_fired_no_agent": ("{what} → (no agent) {do}", "{what} → (에이전트 없음) {do}"),
    "watch.rule_needs_agent": (
        "You {what}. I'd {do}, but I need the agent brain for that.",
        "“{what}” 하셨네요. “{do}” 하려면 에이전트 두뇌가 있어야 해요.",
    ),
    "watch.rule_removed": ("Removed rule {rid}.", "규칙 {rid|을} 지웠어요."),
    "watch.rule_missing": ("There's no rule {rid}.", "규칙 {rid|이} 없어요."),
    "watch.rules_cleared": ("Cleared all rules.", "규칙을 모두 지웠어요."),
    "watch.tool_noted": ("Noted: you're holding the {tool}.", "알겠어요: {tool|을} 들고 계시네요."),
    "watch.tool_empty": ("Noted: empty hands.", "알겠어요: 빈손이시네요."),

    # ----------------------------------------------- rule errors (pps/human/rules.py), after “That rule doesn't work: ”
    "watch.err_unknown_condition": ("unknown condition '{t}'", "모르는 조건 '{t}'"),
    "watch.err_unknown_activity": (
        "unknown activity '{a}' (know: squat, press, curl, wave, weave, reach, point, still, moving)",
        "모르는 동작 '{a}' (아는 동작: squat, press, curl, wave, weave, reach, point, still, moving)",
    ),
    "watch.err_empty_condition": ("empty condition", "조건이 비어 있음"),
    "watch.err_empty_action": ("empty action", "할 일이 비어 있음"),
    "watch.err_expr_parse": ("expression doesn't parse: {msg}", "식을 해석할 수 없음: {msg}"),
    "watch.err_expr_not_allowed": (
        "'{name}' is not allowed in a watch expression",
        "watch 식에서 쓸 수 없는 요소: '{name}'",
    ),
    "watch.err_private": ("private names are not allowed", "비공개 이름은 쓸 수 없음"),
    "watch.err_bad_expr": ("bad expression", "잘못된 식"),

    # ------------------------------------------------------------------ learn: the demonstration
    "watch.learn_too_short": ("I didn't see enough to learn from.", "배울 만큼 충분히 보지 못했어요."),
    "watch.not_recording": ("I'm not recording a demonstration.", "지금은 시범을 기록하고 있지 않아요."),
    "watch.learned_pattern_weave": (
        "Learned '{name}': {dur} s of weave with your {hand}, a {amp} cm weave at {freq} Hz.",
        "'{name}' 배웠어요: 위빙, {hand}, {dur}초, 폭 {amp} cm · {freq} Hz.",
    ),
    "watch.learned_pattern": (
        "Learned '{name}': {dur} s of {act} with your {hand}, a {amp} cm weave at {freq} Hz.",
        "'{name}' 배웠어요: {act}, {hand}, {dur}초, 폭 {amp} cm · {freq} Hz 위빙.",
    ),
    "watch.learned_reps_squat": (
        "Learned '{name}': {dur} s of squat with your {hand}, {n} reps.",
        "'{name}' 배웠어요: 스쿼트 {n}회, {hand}, {dur}초.",
    ),
    "watch.learned_reps_curl": (
        "Learned '{name}': {dur} s of curl with your {hand}, {n} reps.",
        "'{name}' 배웠어요: 컬 {n}회, {hand}, {dur}초.",
    ),
    "watch.learned_reps_press": (
        "Learned '{name}': {dur} s of press with your {hand}, {n} reps.",
        "'{name}' 배웠어요: 프레스 {n}회, {hand}, {dur}초.",
    ),
    "watch.learned_reps": (
        "Learned '{name}': {dur} s of {act} with your {hand}, {n} reps.",
        "'{name}' 배웠어요: {act} {n}회, {hand}, {dur}초.",
    ),
    "watch.learned_squat": (
        "Learned '{name}': {dur} s of squat with your {side} hand.",
        "'{name}' 배웠어요: 스쿼트, {side} 손, {dur}초.",
    ),
    "watch.learned_curl": (
        "Learned '{name}': {dur} s of curl with your {side} hand.",
        "'{name}' 배웠어요: 컬, {side} 손, {dur}초.",
    ),
    "watch.learned_press": (
        "Learned '{name}': {dur} s of press with your {side} hand.",
        "'{name}' 배웠어요: 프레스, {side} 손, {dur}초.",
    ),
    "watch.learned_wave": (
        "Learned '{name}': {dur} s of wave with your {side} hand.",
        "'{name}' 배웠어요: 손 흔들기, {side} 손, {dur}초.",
    ),
    "watch.learned_weave": (
        "Learned '{name}': {dur} s of weave with your {side} hand.",
        "'{name}' 배웠어요: 위빙, {side} 손, {dur}초.",
    ),
    "watch.learned_reach": (
        "Learned '{name}': {dur} s of reach with your {side} hand.",
        "'{name}' 배웠어요: 손 뻗기, {side} 손, {dur}초.",
    ),
    "watch.learned_point": (
        "Learned '{name}': {dur} s of point with your {side} hand.",
        "'{name}' 배웠어요: 가리키기, {side} 손, {dur}초.",
    ),
    "watch.learned_still": (
        "Learned '{name}': {dur} s of still with your {side} hand.",
        "'{name}' 배웠어요: 가만히 있기, {side} 손, {dur}초.",
    ),
    "watch.learned_moving": (
        "Learned '{name}': {dur} s of moving with your {side} hand.",
        "'{name}' 배웠어요: 움직임, {side} 손, {dur}초.",
    ),
    "watch.learned_none": (
        "Learned '{name}': {dur} s of none with your {side} hand.",
        "'{name}' 기록했어요: {side} 손, {dur}초, 뚜렷한 동작 없음.",
    ),
    "watch.learned": (
        "Learned '{name}': {dur} s of {act} with your {side} hand.",
        "'{name}' 배웠어요: {act}, {side} 손, {dur}초.",
    ),
    "watch.replay_none": (
        "I have no demonstration to repeat yet. Say 'watch and learn' and show me.",
        "아직 따라 할 시범이 없어요. 'watch and learn'(또는 '보고 배워')이라고 말하고 보여 주세요.",
    ),
    "watch.replay_over_timed": (
        "My version of '{name}' over the {obj}: {n} points, {dur} s.",
        "제가 따라 한 '{name}': {obj} 위에서, 점 {n}개, {dur}초.",
    ),
    "watch.replay_hand_timed": (
        "My version of '{name}' from where my hand is: {n} points, {dur} s.",
        "제가 따라 한 '{name}': 지금 손 위치에서, 점 {n}개, {dur}초.",
    ),
    "watch.replay_over": (
        "My version of '{name}' over the {obj}: {n} points.",
        "제가 따라 한 '{name}': {obj} 위에서, 점 {n}개.",
    ),
    "watch.replay_hand": (
        "My version of '{name}' from where my hand is: {n} points.",
        "제가 따라 한 '{name}': 지금 손 위치에서, 점 {n}개.",
    ),

    # ----------- pps/human/coach.py: one short spoken cue per rep (said after the rep number: “4. Go a little deeper.”)
    "watch.cue_depth": ("Go a little deeper.", "조금 더 깊이 앉으세요."),
    "watch.cue_valgus": ("Push your knees out; they're caving in.", "무릎을 바깥으로 미세요. 안으로 모이고 있어요."),
    "watch.cue_back": ("Chest up, keep your back straighter.", "가슴을 펴고 등을 더 곧게 하세요."),
    "watch.cue_fast": ("Slow down, control it.", "천천히, 조절하면서 하세요."),
    "watch.cue_curl_top": ("Curl all the way up.", "끝까지 올리세요."),
    "watch.cue_curl_bottom": ("Lower all the way down.", "끝까지 내리세요."),
    "watch.cue_elbow": ("Keep your elbow by your side.", "팔꿈치를 옆구리에 붙이세요."),
    "watch.cue_lockout": ("Lock out overhead.", "머리 위에서 팔을 끝까지 펴세요."),
    "watch.cue_weave_wide": ("Narrower weave.", "위빙을 더 좁게 하세요."),
    "watch.cue_weave_narrow": ("Wider weave, like the demo.", "시범처럼 위빙을 더 넓게 하세요."),
    "watch.cue_weave_fast": ("Slower side to side.", "좌우로 더 천천히 움직이세요."),
    "watch.cue_weave_slow": ("A bit quicker side to side.", "좌우로 조금 더 빠르게 움직이세요."),
    "watch.cue_travel_fast": ("Travel slower along the seam.", "용접선을 따라 더 천천히 이동하세요."),
    "watch.cue_travel_slow": ("Travel a bit faster along the seam.", "용접선을 따라 조금 더 빠르게 이동하세요."),

    # ------------------------------------------------------------ pps/human/demo.py: a demonstration's trajectory label
    "watch.demo_label_tool": ("{act} demonstrated by hand with the {tool}", "{tool|을} 들고 손으로 보여 준 {act}"),
    "watch.demo_label": ("{act} demonstrated by hand", "손으로 보여 준 {act}"),

    # --------- pps/human/model.py: the rule-based reading, 'squatting · rep 4 · down · holding mug' (each part in turn)
    "watch.sum_squatting": ("squatting", "스쿼트 중"),
    "watch.sum_squatting_more": ("squatting · {rest}", "스쿼트 중 · {rest}"),
    "watch.sum_pressing_overhead": ("pressing overhead", "오버헤드 프레스 중"),
    "watch.sum_pressing_overhead_more": ("pressing overhead · {rest}", "오버헤드 프레스 중 · {rest}"),
    "watch.sum_doing_curls": ("doing curls", "컬 하는 중"),
    "watch.sum_doing_curls_more": ("doing curls · {rest}", "컬 하는 중 · {rest}"),
    "watch.sum_waving": ("waving", "손 흔드는 중"),
    "watch.sum_waving_more": ("waving · {rest}", "손 흔드는 중 · {rest}"),
    "watch.sum_weaving_a_hand": ("weaving a hand", "손으로 위빙하는 중"),
    "watch.sum_weaving_a_hand_more": ("weaving a hand · {rest}", "손으로 위빙하는 중 · {rest}"),
    "watch.sum_reaching_out": ("reaching out", "손 뻗는 중"),
    "watch.sum_reaching_out_more": ("reaching out · {rest}", "손 뻗는 중 · {rest}"),
    "watch.sum_pointing": ("pointing", "가리키는 중"),
    "watch.sum_pointing_more": ("pointing · {rest}", "가리키는 중 · {rest}"),
    "watch.sum_standing_still": ("standing still", "가만히 서 있음"),
    "watch.sum_standing_still_more": ("standing still · {rest}", "가만히 서 있음 · {rest}"),
    "watch.sum_moving": ("moving", "움직이는 중"),
    "watch.sum_moving_more": ("moving · {rest}", "움직이는 중 · {rest}"),
    "watch.sum_nobody_in_view": ("nobody in view", "보이는 사람 없음"),
    "watch.sum_nobody_in_view_more": ("nobody in view · {rest}", "보이는 사람 없음 · {rest}"),
    "watch.sum_squatting_rep": ("squatting · rep {n}", "스쿼트 중 · {n}회"),
    "watch.sum_squatting_rep_more": ("squatting · rep {n} · {rest}", "스쿼트 중 · {n}회 · {rest}"),
    "watch.sum_squatting_down": ("squatting · down", "스쿼트 중 · 내려가는 중"),
    "watch.sum_squatting_rep_down": ("squatting · rep {n} · down", "스쿼트 중 · {n}회 · 내려가는 중"),
    "watch.sum_squatting_up": ("squatting · up", "스쿼트 중 · 올라가는 중"),
    "watch.sum_squatting_rep_up": ("squatting · rep {n} · up", "스쿼트 중 · {n}회 · 올라가는 중"),
    "watch.sum_pressing_overhead_rep": ("pressing overhead · rep {n}", "오버헤드 프레스 중 · {n}회"),
    "watch.sum_pressing_overhead_rep_more": (
        "pressing overhead · rep {n} · {rest}",
        "오버헤드 프레스 중 · {n}회 · {rest}",
    ),
    "watch.sum_pressing_overhead_down": ("pressing overhead · down", "오버헤드 프레스 중 · 내려가는 중"),
    "watch.sum_pressing_overhead_rep_down": (
        "pressing overhead · rep {n} · down",
        "오버헤드 프레스 중 · {n}회 · 내려가는 중",
    ),
    "watch.sum_pressing_overhead_up": ("pressing overhead · up", "오버헤드 프레스 중 · 올라가는 중"),
    "watch.sum_pressing_overhead_rep_up": (
        "pressing overhead · rep {n} · up",
        "오버헤드 프레스 중 · {n}회 · 올라가는 중",
    ),
    "watch.sum_doing_curls_rep": ("doing curls · rep {n}", "컬 하는 중 · {n}회"),
    "watch.sum_doing_curls_rep_more": ("doing curls · rep {n} · {rest}", "컬 하는 중 · {n}회 · {rest}"),
    "watch.sum_doing_curls_down": ("doing curls · down", "컬 하는 중 · 내려가는 중"),
    "watch.sum_doing_curls_rep_down": ("doing curls · rep {n} · down", "컬 하는 중 · {n}회 · 내려가는 중"),
    "watch.sum_doing_curls_up": ("doing curls · up", "컬 하는 중 · 올라가는 중"),
    "watch.sum_doing_curls_rep_up": ("doing curls · rep {n} · up", "컬 하는 중 · {n}회 · 올라가는 중"),
    "watch.sum_weaving_a_hand_rep": ("weaving a hand · rep {n}", "손으로 위빙하는 중 · {n}회"),
    "watch.sum_weaving_a_hand_rep_more": ("weaving a hand · rep {n} · {rest}", "손으로 위빙하는 중 · {n}회 · {rest}"),
    "watch.sum_rep_more": ("rep {n} · {rest}", "{n}회 · {rest}"),
    "watch.sum_down_more": ("down · {rest}", "내려가는 중 · {rest}"),
    "watch.sum_up_more": ("up · {rest}", "올라가는 중 · {rest}"),
    "watch.sum_top_more": ("top · {rest}", "위 · {rest}"),
    "watch.sum_bottom_more": ("bottom · {rest}", "아래 · {rest}"),
    "watch.sum_stuck": ("stuck", "막힘"),
    "watch.sum_stuck_more": ("stuck · {rest}", "막힘 · {rest}"),
    "watch.sum_weave": ("{amp} cm at {freq} Hz", "폭 {amp} cm · {freq} Hz"),
    "watch.sum_weave_more": ("{amp} cm at {freq} Hz · {rest}", "폭 {amp} cm · {freq} Hz · {rest}"),
    "watch.sum_holding": ("holding {obj}", "{obj} 들고 있음"),
    "watch.sum_holding_hand_out": ("holding {obj} · hand held out", "{obj} 들고 있음 · 손 내밂"),
    "watch.sum_hand_out": ("hand held out", "손 내밂"),

    # --------------- pps/human/periodic.py: why a hand motion isn't a weave (skill notes: “it wasn't a clean weave: …”)
    "watch.weave_too_short": ("too short", "너무 짧음"),
    "watch.weave_no_travel": ("no travel ({v} mm/s)", "이동 없음 ({v} mm/s)"),
    "watch.weave_not_straight": ("no straight travel (r2 {r2})", "직선으로 이동하지 않음 (r2 {r2})"),
    "watch.weave_travelled": ("travelled {d} cm", "{d} cm 이동함"),
    "watch.weave_freq": ("{f} Hz outside {lo}-{hi} Hz", "{f} Hz, 범위 {lo}-{hi} Hz 밖"),
    "watch.weave_width": ("width {w} mm outside {lo}-{hi} mm", "폭 {w} mm, 범위 {lo}-{hi} mm 밖"),
    "watch.weave_weak": ("weak periodicity (peak {peak})", "주기성이 약함 (peak {peak})"),
    "watch.weave_irregular": ("irregular (acf {acf})", "불규칙함 (acf {acf})"),
    "watch.weave_and_no_travel": ("{first}; no travel ({v} mm/s)", "{first}; 이동 없음 ({v} mm/s)"),
    "watch.weave_and_not_straight": (
        "{first}; no straight travel (r2 {r2})",
        "{first}; 직선으로 이동하지 않음 (r2 {r2})",
    ),
    "watch.weave_and_travelled": ("{first}; travelled {d} cm", "{first}; {d} cm 이동함"),
    "watch.weave_and_freq": ("{first}; {f} Hz outside {lo}-{hi} Hz", "{first}; {f} Hz, 범위 {lo}-{hi} Hz 밖"),
    "watch.weave_and_width": ("{first}; width {w} mm outside {lo}-{hi} mm", "{first}; 폭 {w} mm, 범위 {lo}-{hi} mm 밖"),
    "watch.weave_and_weak": ("{first}; weak periodicity (peak {peak})", "{first}; 주기성이 약함 (peak {peak})"),
    "watch.weave_and_irregular": ("{first}; irregular (acf {acf})", "{first}; 불규칙함 (acf {acf})"),

    # ------------------------------------------------ pps/features/skills.py: the plan card and the sentence to confirm
    "skills.plan_say": (
        "{say} Right? Say go, or tell me what to change.",
        "{say} 맞나요? 'go'(또는 '실행')라고 하시거나, 바꿀 점을 말해 주세요.",
    ),
    "skills.say_right": ("{say} Right?", "{say} 맞나요?"),
    "skills.right": ("Right?", "맞나요?"),
    "skills.say_go": ("Say go, or tell me what to change.", "'go'(또는 '실행')라고 하시거나, 바꿀 점을 말해 주세요."),
    "skills.blocked": ("{say} But I can't do it yet: {why}", "{say} 하지만 아직은 할 수 없어요: {why}"),
    "skills.blocked_alone": ("But I can't do it yet: {why}", "하지만 아직은 할 수 없어요: {why}"),
    "skills.torch_note_said": (
        "I couldn't read your torch angle from the camera, so the angles are defaults (10° push, square to the "
        "plate). {said}",
        "카메라로 토치 각도를 읽지 못해서 기본 각도를 써요 (미는 각도 10°, 판에 수직). {said}",
    ),
    "skills.torch_note_dot": (
        "I couldn't read your torch angle from the camera, so the angles are defaults (10° push, square to the plate).",
        "카메라로 토치 각도를 읽지 못해서 기본 각도를 써요 (미는 각도 10°, 판에 수직).",
    ),

    # ------------------------------------------------------------------ extract: a demonstration -> a skill
    "skills.no_demo": (
        "I have no demonstration to learn from yet. Say 'watch me' and show me.",
        "아직 배울 시범이 없어요. 'watch me'(또는 '보고 배워')라고 말하고 보여 주세요.",
    ),
    "skills.not_a_skill": (
        "I watched {dur} s of {motion}, but it isn't a skill I know how to learn (a weave, a wipe, a stir, a press or "
        "a lift).",
        "{dur}초 동안 {motion|을} 봤지만, 제가 배울 수 있는 스킬이 아니에요 (위빙, 닦기, 젓기, 누르기, 리프트만 "
        "배워요).",
    ),
    "skills.learned_err": ("Learned '{name}', {label}: {desc}. {err}", "'{name}' 배웠어요 ({label}): {desc}. {err}"),
    "skills.learned": ("Learned '{name}', {label}: {desc}.", "'{name}' 배웠어요 ({label}): {desc}."),
    "skills.ask_kind3": (
        "Was that 1, {a} or 2, {b} or 3, {c}?",
        "방금 건 1번 {a}, 2번 {b}, 3번 {c} 중 어느 쪽이었나요?",
    ),
    "skills.ask_kind2": ("Was that 1, {a} or 2, {b}?", "방금 건 1번 {a}, 2번 {b} 중 어느 쪽이었나요?"),
    "skills.ask_kind": ("Was that {options}?", "방금 건 {options} 중 어느 쪽이었나요?"),
    "skills.kind_word_wipe": ("wiping", "닦기"),
    "skills.kind_word_stir": ("stirring", "젓기"),
    "skills.kind_word_press": ("pressing", "누르기"),
    "skills.kind_word_weave": ("a weave along a seam", "용접선 따라 위빙"),
    "skills.kind_word_spot": ("a lift to spot", "리프트 보조"),
    "skills.note_unclean": ("it wasn't a clean weave: {why}", "깔끔한 위빙이 아니었어요: {why}"),
    "skills.note_torch": (
        "I couldn't read your torch angle from the camera, so the angles are defaults (10° push, square to the plate)",
        "카메라로 토치 각도를 읽지 못해서 기본 각도를 써요 (미는 각도 10°, 판에 수직)",
    ),
    "skills.note_force": (
        "I can't feel how hard you pressed, so it presses {f} N (say harder or softer)",
        "얼마나 세게 누르셨는지 느낄 수 없어서 {f} N으로 눌러요 ('세게' 또는 '살살'이라고 말하세요)",
    ),
    "skills.note_depth": (
        "I can't see how deep you went into the container, so it stirs 3 cm below the rim (say deeper or shallower)",
        "용기 안으로 얼마나 깊이 넣으셨는지 보이지 않아서 가장자리 3 cm 아래에서 저어요 ('deeper' 또는 "
        "'shallower'라고 말하세요)",
    ),
    "skills.no_clear_activity": ("no clear activity", "뚜렷한 동작 없음"),

    # ------------------------------------------------------------------ apply, list, delete, edit
    "skills.no_such_skill": (
        "I don't have a skill called '{name}' (I have: {have}).",
        "그런 스킬은 없어요: '{name}' (있는 스킬: {have}).",
    ),
    "skills.none_yet": ("none yet", "아직 없음"),
    "skills.deleted": ("Deleted '{name}'.", "'{name}' 스킬을 지웠어요."),
    "skills.not_deletable": (
        "There's no skill '{name}' of yours to delete (examples stay).",
        "지울 수 있는 '{name}' 스킬이 없어요 (예제는 지울 수 없어요).",
    ),
    "skills.edit_nothing_on_screen": (
        "There is no skill on screen to change. Name one, or show me first.",
        "화면에 바꿀 스킬이 없어요. 스킬 이름을 말하거나 먼저 보여 주세요.",
    ),
    "skills.edit_say_what": (
        "Say what to change: wider, narrower, slower, faster, more push, closer, harder…",
        "무엇을 바꿀지 말해 주세요: 넓게, 좁게, 천천히, 빨리, 세게, 살살… (wider, more push, closer 같은 영어도 돼요)",
    ),
    "skills.edit_not_understood": (
        "I didn't get what to change in “{text}” (try wider, narrower, slower, faster, more push, closer, harder).",
        "“{text}”에서 무엇을 바꿀지 모르겠어요 (넓게, 좁게, 천천히, 빨리, 세게, 살살 또는 wider, more push, "
        "closer처럼 말해 보세요).",
    ),
    "skills.nothing_changed": ("nothing changed (already at the limit)", "바뀐 게 없어요 (이미 한계값이에요)"),
    "skills.okay_now": ("Okay: {changed}. '{name}' is now {desc}.", "알겠어요: {changed}. 이제 '{name}' 스킬: {desc}."),
    "skills.okay_said": ("Okay: {changed}. {said}", "알겠어요: {changed}. {said}"),
    "skills.okay": ("Okay: {changed}.", "알겠어요: {changed}."),
    "skills.beads_cleared": ("Cleared the weld beads.", "용접 비드를 지웠어요."),
    "skills.unknown_op": (
        "Unknown skill op '{op}'. Use extract, apply, edit, list, delete, bead_clear or pack_demo.",
        "모르는 스킬 동작이에요: '{op}'. extract, apply, edit, list, delete, bead_clear, pack_demo 중 하나를 쓰세요.",
    ),
    "skills.err_no_watch": ("Watching isn't available.", "보기 기능을 쓸 수 없어요."),

    # ------------------------------------------------------------------ demo packs played without a camera
    "skills.pack_first": (
        "Load a demo pack first (weld cell, gym or workshop).",
        "먼저 데모 팩을 불러오세요 (용접 셀, 체육관, 작업장 중 하나).",
    ),
    "skills.pack_welder": (
        "Playing a synthetic welder: a 5 mm weave at 2 Hz, 1 cm/s. I'm learning it.",
        "가상의 용접사를 재생해요: 5 mm 위빙, 2 Hz, 1 cm/s. 보고 배울게요.",
    ),
    "skills.pack_lifter": (
        "Playing a synthetic lifter who gets stuck on the third rep; I'm spotting.",
        "세 번째 반복에서 막히는 가상의 리프터를 재생해요. 제가 보조할게요.",
    ),
    "skills.pack_person": (
        "Playing a synthetic person holding an open hand out.",
        "편 손을 내미는 가상의 사람을 재생해요.",
    ),

    # -------------------------------- pps/skills/core.py: kinds (labels in the skill list; lower case inside sentences)
    "skills.kind_weave": ("Weave along a seam", "용접선 따라 위빙"),
    "skills.kind_weave_lc": ("weave along a seam", "용접선 따라 위빙"),
    "skills.kind_wipe": ("Wipe an area", "영역 닦기"),
    "skills.kind_wipe_lc": ("wipe an area", "영역 닦기"),
    "skills.kind_stir": ("Stir in a container", "용기 안 젓기"),
    "skills.kind_stir_lc": ("stir in a container", "용기 안 젓기"),
    "skills.kind_press": ("Press on something", "대상 누르기"),
    "skills.kind_press_lc": ("press on something", "대상 누르기"),
    "skills.kind_spot": ("Spot a lift", "리프트 보조"),
    "skills.kind_spot_lc": ("spot a lift", "리프트 보조"),
    "skills.kind_motion": ("Whole-body motion", "전신 동작"),
    "skills.kind_motion_lc": ("whole-body motion", "전신 동작"),

    # ---------------------------------- describe: the style in one line (“4 mm zigzag at 2 Hz, 5 mm/s, 15° push angle”)
    "skills.say_weld_zigzag": (
        "I'll weld {seam} ({length} cm): {amp} mm zigzag at {freq} Hz, {speed}, {angle}.",
        "{seam|을} 용접할게요 ({length} cm): {amp} mm 지그재그 위빙 {freq} Hz, {speed}, {angle}.",
    ),
    "skills.learned_zigzag": (
        "Learned '{name}', {label}: {amp} mm zigzag at {freq} Hz, {speed}, {angle}.",
        "'{name}' 배웠어요 ({label}): {amp} mm 지그재그 위빙 {freq} Hz, {speed}, {angle}.",
    ),
    "skills.learned_zigzag_err": (
        "Learned '{name}', {label}: {amp} mm zigzag at {freq} Hz, {speed}, {angle}. {err}",
        "'{name}' 배웠어요 ({label}): {amp} mm 지그재그 위빙 {freq} Hz, {speed}, {angle}. {err}",
    ),
    "skills.okay_now_zigzag": (
        "Okay: {changed}. '{name}' is now {amp} mm zigzag at {freq} Hz, {speed}, {angle}.",
        "알겠어요: {changed}. 이제 '{name}' 스킬: {amp} mm 지그재그 위빙 {freq} Hz, {speed}, {angle}.",
    ),
    "skills.blocked_weld_zigzag": (
        "I'll weld {seam} ({length} cm): {amp} mm zigzag at {freq} Hz, {speed}, {angle}. But I can't do it yet: {why}",
        "{seam|을} 용접할게요 ({length} cm): {amp} mm 지그재그 위빙 {freq} Hz, {speed}, {angle}. 하지만 아직은 할 수 "
        "없어요: {why}",
    ),
    "skills.say_weld_circle": (
        "I'll weld {seam} ({length} cm): {amp} mm circle at {freq} Hz, {speed}, {angle}.",
        "{seam|을} 용접할게요 ({length} cm): {amp} mm 원형 위빙 {freq} Hz, {speed}, {angle}.",
    ),
    "skills.learned_circle": (
        "Learned '{name}', {label}: {amp} mm circle at {freq} Hz, {speed}, {angle}.",
        "'{name}' 배웠어요 ({label}): {amp} mm 원형 위빙 {freq} Hz, {speed}, {angle}.",
    ),
    "skills.learned_circle_err": (
        "Learned '{name}', {label}: {amp} mm circle at {freq} Hz, {speed}, {angle}. {err}",
        "'{name}' 배웠어요 ({label}): {amp} mm 원형 위빙 {freq} Hz, {speed}, {angle}. {err}",
    ),
    "skills.okay_now_circle": (
        "Okay: {changed}. '{name}' is now {amp} mm circle at {freq} Hz, {speed}, {angle}.",
        "알겠어요: {changed}. 이제 '{name}' 스킬: {amp} mm 원형 위빙 {freq} Hz, {speed}, {angle}.",
    ),
    "skills.blocked_weld_circle": (
        "I'll weld {seam} ({length} cm): {amp} mm circle at {freq} Hz, {speed}, {angle}. But I can't do it yet: {why}",
        "{seam|을} 용접할게요 ({length} cm): {amp} mm 원형 위빙 {freq} Hz, {speed}, {angle}. 하지만 아직은 할 수 "
        "없어요: {why}",
    ),
    "skills.say_weld_figure8": (
        "I'll weld {seam} ({length} cm): {amp} mm figure8 at {freq} Hz, {speed}, {angle}.",
        "{seam|을} 용접할게요 ({length} cm): {amp} mm 8자 위빙 {freq} Hz, {speed}, {angle}.",
    ),
    "skills.learned_figure8": (
        "Learned '{name}', {label}: {amp} mm figure8 at {freq} Hz, {speed}, {angle}.",
        "'{name}' 배웠어요 ({label}): {amp} mm 8자 위빙 {freq} Hz, {speed}, {angle}.",
    ),
    "skills.learned_figure8_err": (
        "Learned '{name}', {label}: {amp} mm figure8 at {freq} Hz, {speed}, {angle}. {err}",
        "'{name}' 배웠어요 ({label}): {amp} mm 8자 위빙 {freq} Hz, {speed}, {angle}. {err}",
    ),
    "skills.okay_now_figure8": (
        "Okay: {changed}. '{name}' is now {amp} mm figure8 at {freq} Hz, {speed}, {angle}.",
        "알겠어요: {changed}. 이제 '{name}' 스킬: {amp} mm 8자 위빙 {freq} Hz, {speed}, {angle}.",
    ),
    "skills.blocked_weld_figure8": (
        "I'll weld {seam} ({length} cm): {amp} mm figure8 at {freq} Hz, {speed}, {angle}. But I can't do it yet: {why}",
        "{seam|을} 용접할게요 ({length} cm): {amp} mm 8자 위빙 {freq} Hz, {speed}, {angle}. 하지만 아직은 할 수 "
        "없어요: {why}",
    ),
    "skills.say_wipe_raster_pass": (
        "I'll wipe {label}: raster wipe, strokes {spacing} cm apart, {f} N, {n} pass.",
        "{label|을} 닦을게요: 왕복 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.learned_raster_pass": (
        "Learned '{name}', {label}: raster wipe, strokes {spacing} cm apart, {f} N, {n} pass.",
        "'{name}' 배웠어요 ({label}): 왕복 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.learned_raster_pass_err": (
        "Learned '{name}', {label}: raster wipe, strokes {spacing} cm apart, {f} N, {n} pass. {err}",
        "'{name}' 배웠어요 ({label}): 왕복 닦기, 줄 간격 {spacing} cm, {f} N, {n}회. {err}",
    ),
    "skills.okay_now_raster_pass": (
        "Okay: {changed}. '{name}' is now raster wipe, strokes {spacing} cm apart, {f} N, {n} pass.",
        "알겠어요: {changed}. 이제 '{name}' 스킬: 왕복 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.say_wipe_raster_passes": (
        "I'll wipe {label}: raster wipe, strokes {spacing} cm apart, {f} N, {n} passes.",
        "{label|을} 닦을게요: 왕복 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.learned_raster_passes": (
        "Learned '{name}', {label}: raster wipe, strokes {spacing} cm apart, {f} N, {n} passes.",
        "'{name}' 배웠어요 ({label}): 왕복 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.learned_raster_passes_err": (
        "Learned '{name}', {label}: raster wipe, strokes {spacing} cm apart, {f} N, {n} passes. {err}",
        "'{name}' 배웠어요 ({label}): 왕복 닦기, 줄 간격 {spacing} cm, {f} N, {n}회. {err}",
    ),
    "skills.okay_now_raster_passes": (
        "Okay: {changed}. '{name}' is now raster wipe, strokes {spacing} cm apart, {f} N, {n} passes.",
        "알겠어요: {changed}. 이제 '{name}' 스킬: 왕복 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.say_wipe_spiral_pass": (
        "I'll wipe {label}: spiral wipe, strokes {spacing} cm apart, {f} N, {n} pass.",
        "{label|을} 닦을게요: 나선형 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.learned_spiral_pass": (
        "Learned '{name}', {label}: spiral wipe, strokes {spacing} cm apart, {f} N, {n} pass.",
        "'{name}' 배웠어요 ({label}): 나선형 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.learned_spiral_pass_err": (
        "Learned '{name}', {label}: spiral wipe, strokes {spacing} cm apart, {f} N, {n} pass. {err}",
        "'{name}' 배웠어요 ({label}): 나선형 닦기, 줄 간격 {spacing} cm, {f} N, {n}회. {err}",
    ),
    "skills.okay_now_spiral_pass": (
        "Okay: {changed}. '{name}' is now spiral wipe, strokes {spacing} cm apart, {f} N, {n} pass.",
        "알겠어요: {changed}. 이제 '{name}' 스킬: 나선형 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.say_wipe_spiral_passes": (
        "I'll wipe {label}: spiral wipe, strokes {spacing} cm apart, {f} N, {n} passes.",
        "{label|을} 닦을게요: 나선형 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.learned_spiral_passes": (
        "Learned '{name}', {label}: spiral wipe, strokes {spacing} cm apart, {f} N, {n} passes.",
        "'{name}' 배웠어요 ({label}): 나선형 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.learned_spiral_passes_err": (
        "Learned '{name}', {label}: spiral wipe, strokes {spacing} cm apart, {f} N, {n} passes. {err}",
        "'{name}' 배웠어요 ({label}): 나선형 닦기, 줄 간격 {spacing} cm, {f} N, {n}회. {err}",
    ),
    "skills.okay_now_spiral_passes": (
        "Okay: {changed}. '{name}' is now spiral wipe, strokes {spacing} cm apart, {f} N, {n} passes.",
        "알겠어요: {changed}. 이제 '{name}' 스킬: 나선형 닦기, 줄 간격 {spacing} cm, {f} N, {n}회.",
    ),
    "skills.desc_push": ("{deg}° push angle", "미는 각도 {deg}°"),
    "skills.desc_drag": ("{deg}° drag angle", "끄는 각도 {deg}°"),
    "skills.desc_no_push": ("no push or drag", "밀거나 끌지 않음"),
    "skills.desc_push_lean_right": (
        "{deg}° push angle, leaning {lean}° to the right",
        "미는 각도 {deg}°, 오른쪽으로 {lean}° 기울임",
    ),
    "skills.desc_drag_lean_right": (
        "{deg}° drag angle, leaning {lean}° to the right",
        "끄는 각도 {deg}°, 오른쪽으로 {lean}° 기울임",
    ),
    "skills.desc_no_push_lean_right": (
        "no push or drag, leaning {lean}° to the right",
        "밀거나 끌지 않음, 오른쪽으로 {lean}° 기울임",
    ),
    "skills.desc_push_lean_left": (
        "{deg}° push angle, leaning {lean}° to the left",
        "미는 각도 {deg}°, 왼쪽으로 {lean}° 기울임",
    ),
    "skills.desc_drag_lean_left": (
        "{deg}° drag angle, leaning {lean}° to the left",
        "끄는 각도 {deg}°, 왼쪽으로 {lean}° 기울임",
    ),
    "skills.desc_no_push_lean_left": (
        "no push or drag, leaning {lean}° to the left",
        "밀거나 끌지 않음, 왼쪽으로 {lean}° 기울임",
    ),
    "skills.desc_stir_ccw": (
        "{turns} counter-clockwise turns of {r} cm at {f} Hz, {d} cm deep",
        "반시계 방향으로 {turns}바퀴, 반지름 {r} cm, {f} Hz, 깊이 {d} cm",
    ),
    "skills.desc_stir_cw": (
        "{turns} clockwise turns of {r} cm at {f} Hz, {d} cm deep",
        "시계 방향으로 {turns}바퀴, 반지름 {r} cm, {f} Hz, 깊이 {d} cm",
    ),
    "skills.desc_press": ("press with {f} N for {s} s", "{f} N으로 {s}초 누르기"),
    "skills.desc_spot_bar": (
        "follow the bar under your hands and help when it slows below {v} cm/s for {s} s, up to {f} N ({lift} cm lift)",
        "손 아래에서 바를 따라가다가 {s}초 동안 {v} cm/s보다 느려지면 최대 {f} N으로 돕기 ({lift} cm 들어 올림)",
    ),
    "skills.desc_spot_hands": (
        "follow your hands under your hands and help when it slows below {v} cm/s for {s} s, up to {f} N ({lift} cm "
        "lift)",
        "손 아래를 따라가다가 {s}초 동안 {v} cm/s보다 느려지면 최대 {f} N으로 돕기 ({lift} cm 들어 올림)",
    ),
    "skills.desc_motion": ("replay the whole-body motion '{clip}'", "전신 동작 '{clip}' 재생하기"),

    # ---------------------------- corrections: what changed (“weave width 4 mm → 5 mm; travel speed 8 mm/s → 6.4 mm/s”)
    "skills.change_weave_width": ("weave width {old} → {new}", "위빙 폭 {old} → {new}"),
    "skills.change_weave_freq": ("weave frequency {old} → {new}", "위빙 주파수 {old} → {new}"),
    "skills.change_weave": ("weave {old} → {new}", "위빙 모양 {old} → {new}"),
    "skills.change_travel_speed": ("travel speed {old} → {new}", "이동 속도 {old} → {new}"),
    "skills.change_travel_angle": ("travel angle {old} → {new}", "진행 각도 {old} → {new}"),
    "skills.change_work_angle": ("work angle {old} → {new}", "작업 각도 {old} → {new}"),
    "skills.change_standoff": ("standoff {old} → {new}", "토치 거리 {old} → {new}"),
    "skills.change_direction": ("direction {old} → {new}", "방향 {old} → {new}"),
    "skills.change_force": ("force {old} → {new}", "힘 {old} → {new}"),
    "skills.change_pattern": ("pattern {old} → {new}", "패턴 {old} → {new}"),
    "skills.change_passes": ("passes {old} → {new}", "반복 횟수 {old} → {new}"),
    "skills.change_spacing": ("spacing {old} → {new}", "간격 {old} → {new}"),
    "skills.change_speed": ("speed {old} → {new}", "속도 {old} → {new}"),
    "skills.change_radius": ("radius {old} → {new}", "반지름 {old} → {new}"),
    "skills.change_freq": ("freq {old} → {new}", "주파수 {old} → {new}"),
    "skills.change_depth": ("depth {old} → {new}", "깊이 {old} → {new}"),
    "skills.change_turns": ("turns {old} → {new}", "바퀴 수 {old} → {new}"),
    "skills.change_hold": ("hold {old} → {new}", "누르는 시간 {old} → {new}"),
    "skills.change_help_below": ("help below {old} → {new}", "도움 기준 속도 {old} → {new}"),
    "skills.change_after": ("after {old} → {new}", "대기 시간 {old} → {new}"),
    "skills.change_max_force": ("max force {old} → {new}", "최대 힘 {old} → {new}"),
    "skills.change_lift": ("lift {old} → {new}", "들어 올림 {old} → {new}"),
    "skills.change_more_weave_width": ("weave width {old} → {new}; {rest}", "위빙 폭 {old} → {new}; {rest}"),
    "skills.change_more_weave_freq": ("weave frequency {old} → {new}; {rest}", "위빙 주파수 {old} → {new}; {rest}"),
    "skills.change_more_weave": ("weave {old} → {new}; {rest}", "위빙 모양 {old} → {new}; {rest}"),
    "skills.change_more_travel_speed": ("travel speed {old} → {new}; {rest}", "이동 속도 {old} → {new}; {rest}"),
    "skills.change_more_travel_angle": ("travel angle {old} → {new}; {rest}", "진행 각도 {old} → {new}; {rest}"),
    "skills.change_more_work_angle": ("work angle {old} → {new}; {rest}", "작업 각도 {old} → {new}; {rest}"),
    "skills.change_more_standoff": ("standoff {old} → {new}; {rest}", "토치 거리 {old} → {new}; {rest}"),
    "skills.change_more_direction": ("direction {old} → {new}; {rest}", "방향 {old} → {new}; {rest}"),
    "skills.change_more_force": ("force {old} → {new}; {rest}", "힘 {old} → {new}; {rest}"),
    "skills.change_more_pattern": ("pattern {old} → {new}; {rest}", "패턴 {old} → {new}; {rest}"),
    "skills.change_more_passes": ("passes {old} → {new}; {rest}", "반복 횟수 {old} → {new}; {rest}"),
    "skills.change_more_spacing": ("spacing {old} → {new}; {rest}", "간격 {old} → {new}; {rest}"),
    "skills.change_more_speed": ("speed {old} → {new}; {rest}", "속도 {old} → {new}; {rest}"),
    "skills.change_more_radius": ("radius {old} → {new}; {rest}", "반지름 {old} → {new}; {rest}"),
    "skills.change_more_freq": ("freq {old} → {new}; {rest}", "주파수 {old} → {new}; {rest}"),
    "skills.change_more_depth": ("depth {old} → {new}; {rest}", "깊이 {old} → {new}; {rest}"),
    "skills.change_more_turns": ("turns {old} → {new}; {rest}", "바퀴 수 {old} → {new}; {rest}"),
    "skills.change_more_hold": ("hold {old} → {new}; {rest}", "누르는 시간 {old} → {new}; {rest}"),
    "skills.change_more_help_below": ("help below {old} → {new}; {rest}", "도움 기준 속도 {old} → {new}; {rest}"),
    "skills.change_more_after": ("after {old} → {new}; {rest}", "대기 시간 {old} → {new}; {rest}"),
    "skills.change_more_max_force": ("max force {old} → {new}; {rest}", "최대 힘 {old} → {new}; {rest}"),
    "skills.change_more_lift": ("lift {old} → {new}; {rest}", "들어 올림 {old} → {new}; {rest}"),

    # ------------------------------------------------------------------ pps/skills/compile.py: what the robot will do
    "skills.say_weld": ("I'll weld {seam} ({length} cm): {desc}.", "{seam|을} 용접할게요 ({length} cm): {desc}."),
    "skills.say_wipe": ("I'll wipe {label}: {desc}.", "{label|을} 닦을게요: {desc}."),
    "skills.say_stir": ("I'll stir {label}: {desc}.", "{label} 안을 저을게요: {desc}."),
    "skills.say_press": ("I'll press with {f} N for {s} s on {label}.", "{label|을} {f} N으로 {s}초 동안 누를게요."),
    "skills.blocked_press": (
        "I'll press with {f} N for {s} s on {label}. But I can't do it yet: {why}",
        "{label|을} {f} N으로 {s}초 동안 누를게요. 하지만 아직은 할 수 없어요: {why}",
    ),
    "skills.say_spot": ("I'll spot you: {desc}.", "리프트를 보조할게요: {desc}."),
    "skills.say_motion": ("I'll replay the whole-body motion '{clip}'.", "전신 동작 '{clip}' 재생할게요."),
    "skills.err_wipe_small": ("The {obj} is too small to wipe.", "{obj|은} 닦기엔 너무 작아요."),
    "skills.err_wipe_small_region": (
        "The {side} of the table is too small to wipe.",
        "테이블 {side|은} 닦기엔 너무 작아요.",
    ),
    "skills.err_wipe_small_area": ("That area is too small to wipe.", "그 영역은 닦기엔 너무 작아요."),
    "skills.err_wipe_strokes": (
        "That wipe needs too many strokes; make the spacing wider.",
        "그렇게 닦으려면 왕복이 너무 많아요. 간격을 더 넓혀 주세요.",
    ),
    "skills.err_robot_cant": (
        "This robot can't do a {label} skill (it needs {missing}).",
        "이 로봇은 {label} 스킬을 할 수 없어요 ({missing} 기능이 필요해요).",
    ),
    "skills.err_no_clip": ("This motion skill has no clip.", "이 동작 스킬에는 클립이 없어요."),

    # ----------------------------------------------------------------- pps/skills/geom.py: where (targets in the scene)
    "skills.seam_given": ("the seam you gave me", "알려 주신 용접선"),
    "skills.edge_of": ("the edge of the {obj}", "{obj} 가장자리"),
    "skills.that_area": ("that area", "그 영역"),
    "skills.that_spot": ("that spot", "그 자리"),
    "skills.err_polyline": (
        "A seam polyline needs at least two [x, y, z] points.",
        "용접선 폴리라인에는 [x, y, z] 점이 두 개 이상 필요해요.",
    ),
    "skills.err_no_seam": (
        "There is no seam in this scene (load the weld cell, or give the seam as points).",
        "이 장면에는 용접선이 없어요 (용접 셀을 불러오거나 용접선을 점으로 알려 주세요).",
    ),
    "skills.err_seam_name": (
        "I don't see a seam called '{target}' (seams here: {have}).",
        "그런 용접선이 보이지 않아요: '{target}' (여기 있는 용접선: {have}).",
    ),
    "skills.err_wipe_what": ("I don't know what to wipe: '{target}'.", "무엇을 닦을지 모르겠어요: '{target}'."),
    "skills.err_stir_what": ("I don't see '{target}' to stir in.", "저을 용기가 보이지 않아요: '{target}'."),
    "skills.err_nothing_to_stir": (
        "There's nothing to stir in here (a bowl, pot or cup).",
        "여기엔 저을 용기가 없어요 (그릇, 냄비, 컵).",
    ),
    "skills.err_press_what": ("I don't see '{target}' to press on.", "누를 대상이 보이지 않아요: '{target}'."),
    "skills.err_press_say": ("Say what to press on.", "무엇을 누를지 말해 주세요."),

    # --------------------- pps/skills/core.py, extract.py: skills that can't be made (said with a capital and a period)
    "skills.err_one_of": ("{param} must be one of {opts}.", "{param} 값은 {opts} 중 하나여야 해요."),
    "skills.err_kind": (
        "I don't know the skill kind '{kind}'. Kinds: {kinds}.",
        "모르는 스킬 종류예요: '{kind}'. 종류: {kinds}.",
    ),
    "skills.err_kind_unknown": ("Unknown skill kind '{kind}'.", "모르는 스킬 종류예요: '{kind}'."),
    "skills.err_object": ("A skill must be an object.", "스킬은 객체여야 해요."),
    "skills.err_say_change": ("Say what to change.", "무엇을 바꿀지 말해 주세요."),
    "skills.err_scale": ("{param} isn't a number I can scale.", "{param} 값은 배율로 바꿀 수 있는 숫자가 아니에요."),
    "skills.err_no_param": ("A {kind} skill has no '{param}'.", "{kind} 스킬에는 '{param}' 항목이 없어요."),
    "skills.err_weave_short": (
        "I didn't see a weave long enough to learn (show it for a few seconds).",
        "배울 만큼 긴 위빙을 보지 못했어요 (몇 초 동안 보여 주세요).",
    ),
    "skills.err_no_skill_found": (
        "I couldn't find a skill in that demonstration ({act}): show a weave, a wipe, a stir, a press or a lift.",
        "그 시범에서 스킬을 찾지 못했어요 ({act}): 위빙, 닦기, 젓기, 누르기, 리프트 중 하나를 보여 주세요.",
    ),
    "skills.err_kind_from_hand": (
        "I can't learn a '{kind}' skill from a hand demonstration.",
        "손 시범으로는 '{kind}' 스킬을 배울 수 없어요.",
    ),

    # ---------------------------------------------- pps/safety.py, pps/features/safety.py: slowing and protective stops
    "safety.slowing": ("Slowing: you're {cm} cm away", "감속 중: {cm} cm 거리에 계세요"),
    "safety.stop_force": (
        "Stopped: I pressed on you harder than allowed. Say resume when you're clear.",
        "정지: 허용보다 세게 눌렀어요. 물러나신 뒤 'resume'이라고 말씀하세요.",
    ),
    "safety.force": (
        "I pressed on you harder than allowed. Say resume when you're clear.",
        "허용보다 세게 눌렀어요. 물러나신 뒤 'resume'이라고 말씀하세요.",
    ),
    "safety.stop_force_first": ("Stopped: I pressed on you harder than allowed.", "정지: 허용보다 세게 눌렀어요."),
    "safety.say_resume_when_clear": ("Say resume when you're clear.", "물러나신 뒤 'resume'이라고 말씀하세요."),
    "safety.stop_close": ("Stopped: too close to {part}", "정지: {part}에 너무 가까워요"),
    "safety.stop_close_said": ("Stopped: too close to {part}.", "정지: {part}에 너무 가까워요."),
    "safety.stop_no_force": (
        "Stopped: I'd touch {part} and I can't feel force here",
        "정지: {part}에 닿을 것 같은데 여기서는 힘을 느낄 수 없어요",
    ),
    "safety.stop_no_force_said": (
        "Stopped: I'd touch {part} and I can't feel force here.",
        "정지: {part}에 닿을 것 같은데 여기서는 힘을 느낄 수 없어요.",
    ),
    "safety.stop_touching": ("Stopped: {part} is touching {limb}", "정지: {part|이} {limb}에 닿아 있어요"),
    "safety.stop_touching_said": ("Stopped: {part} is touching {limb}.", "정지: {part|이} {limb}에 닿아 있어요."),
    "safety.stop_cm": ("Stopped: you're {cm} cm from {limb}", "정지: {limb}에서 {cm} cm 거리에 계세요"),
    "safety.stop_cm_said": ("Stopped: you're {cm} cm from {limb}.", "정지: {limb}에서 {cm} cm 거리에 계세요."),
    "safety.why_close": ("too close to {part}", "{part}에 너무 가까워요"),
    "safety.why_no_force": (
        "I'd touch {part} and I can't feel force here",
        "{part}에 닿을 것 같은데 여기서는 힘을 느낄 수 없어요",
    ),
    "safety.why_touching": ("{part} is touching {limb}", "{part|이} {limb}에 닿아 있어요"),
    "safety.why_cm": ("you're {cm} cm from {limb}", "{limb}에서 {cm} cm 거리에 계세요"),
    "safety.too_close": ("you're too close", "너무 가까워요"),
    "safety.too_close_said": ("you're too close.", "너무 가까워요."),
    "safety.your_head": ("your head", "머리"),
    "safety.your_body": ("your body", "몸"),
    "safety.your_arm": ("your arm", "팔"),
    "safety.your_hand": ("your hand", "손"),
    "safety.your_leg": ("your leg", "다리"),
    "safety.my_arm": ("my arm", "제 팔"),
    "safety.my_body": ("my body", "제 몸"),
    "safety.my_gripper": ("my gripper", "제 그리퍼"),
    "safety.my_right_hand": ("my right hand", "제 오른손"),
    "safety.my_left_hand": ("my left hand", "제 왼손"),
    "safety.paused_step_back": (
        "Paused for safety: {line}. I'll go on when you step back.",
        "안전을 위해 멈췄어요: {line}. 물러나시면 계속할게요.",
    ),
    "safety.paused_resume": (
        "Paused for safety: {line}. Say resume once you're clear.",
        "안전을 위해 멈췄어요: {line}. 물러나신 뒤 'resume'이라고 말씀하세요.",
    ),
    "safety.clear": ("You're clear. Carrying on.", "이제 괜찮아요. 계속할게요."),
    "safety.cant_move": (
        "I can't move right now: {line}. Step back a little and ask again.",
        "지금은 움직일 수 없어요: {line}. 조금 물러나신 뒤 다시 부탁해 주세요.",
    ),
    "safety.wont_reach_spot": (
        "I won't reach there: that spot is {why}. Step back, or choose another spot.",
        "거기엔 손을 뻗지 않을게요: 그 자리가 {why}에 있어요. 물러나시거나 다른 자리를 골라 주세요.",
    ),
    "safety.wont_reach_object": (
        "I won't reach there: that object is {why}. Step back, or choose another spot.",
        "거기엔 손을 뻗지 않을게요: 그 물체가 {why}에 있어요. 물러나시거나 다른 자리를 골라 주세요.",
    ),
    "safety.zone_inside": ("inside your body zone (in {part})", "몸 보호 구역 안({part} 쪽)"),
    "safety.zone_cm": ("inside your body zone: {cm} cm from {part}", "몸 보호 구역 안({part}에서 {cm} cm)"),
    "safety.nothing_to_confirm": ("Nothing to confirm.", "확인할 게 없어요."),
    "safety.go_on_self": ("I'll go on by myself once you step back.", "물러나시면 알아서 계속할게요."),
    "safety.thanks": ("Thanks. I'll go on once you're clear.", "고마워요. 물러나시면 계속할게요."),
    "safety.always_on": (
        "Safety is always on. Its distances are set in the server's config, not from the app.",
        "안전 기능은 항상 켜져 있어요. 거리는 앱이 아니라 서버 설정에서 정해요.",
    ),
}
