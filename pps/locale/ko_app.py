"""Korean server messages: app (pps/app.py, pps/compile_other.py, pps/mock.py, pps/demo.py, pps/teach.py, pps/agent.py).
English is the message id; see pps/i18n.py."""

MESSAGES = {
    # key: (English template, Korean template)

    # ------------------------------------------------------------------ pps/app.py: uploads (/upload)
    "app.upload_needs_humanoid": (
        "Imitating a video needs a humanoid body (the Unitree G1). Switch robots first.",
        "영상 따라 하기는 휴머노이드 몸(Unitree G1)이 있어야 해요. 먼저 로봇을 바꿔 주세요.",
    ),
    "app.upload_too_large": ("video too large (300 MB max)", "영상이 너무 커요 (최대 300 MB)"),
    "app.upload_imitating": ("Imitating '{stem}'.", "'{stem}' 영상을 따라 하고 있어요."),
    "app.upload_becomes_skill": ("It becomes a skill when the video ends.", "영상이 끝나면 스킬로 저장돼요."),

    # ------------------------------------------------------------------ humanoid-only features on other bodies
    "app.needs_humanoid": (
        "{what} needs a humanoid body: the {label} can't do that. Switch to the Unitree G1 for it.",
        "{what|은} 휴머노이드 몸이 있어야 해요. {label|은} 할 수 없어요. Unitree G1로 바꿔 주세요.",
    ),
    "app.feat_following": ("Following you", "따라 하기"),
    "app.feat_learning_skill": ("Learning a motion skill", "동작 스킬 배우기"),
    "app.feat_editing_skill": ("Editing a motion skill", "동작 스킬 편집"),
    "app.feat_replay_taught": ("Replaying a task taught by showing", "시범으로 배운 작업 재생"),
    "app.feat_replay_skill": ("Replaying a motion skill", "동작 스킬 재생"),
    "app.feat_teaching": ("Teaching by showing", "시범 가르치기"),
    "app.feat_imitating_video": ("Imitating a video", "영상 따라 하기"),

    # ------------------------------------------------------------------ video imitation
    "app.video_replay_tail": (" Say 'do {name}' to replay it.", " 다시 하려면 'do {name}'(또는 '{name} 실행')이라고 말하세요."),
    "app.video_replay": ("Say 'do {name}' to replay it.", "다시 하려면 'do {name}'(또는 '{name} 실행')이라고 말하세요."),
    "app.video_open_failed": ("I couldn't open that video.", "그 영상을 열 수 없었어요."),
    "app.video_imitating": (
        "Imitating '{name}' from your video, whole body.",
        "영상 속 '{name}' 동작을 온몸으로 따라 하고 있어요.",
    ),

    # ------------------------------------------------------------------ markers / taps / camera
    "app.marker_placed": ("Marker {mid} placed.", "마커 {mid|을} 놓았어요."),
    "app.marker_say_put": (
        "Say 'put the mug on {mid}'.",
        "'put the mug on {mid}'(또는 '머그를 {mid}에 놔')라고 말하세요.",
    ),
    "app.tap_missed": ("That tap missed the scene.", "누른 곳이 장면 밖이에요."),
    "app.cant_see_arms": (
        "I can't see your arms. Step back so your shoulders and hands are in your camera.",
        "팔이 안 보여요. 어깨와 손이 카메라에 다 들어오도록 조금 뒤로 물러나 주세요.",
    ),

    # ------------------------------------------------------------------ teach / demo / mock toggles
    "app.teach_watching": (
        "I'm watching. Show me the task and tell me what you're doing.",
        "보고 있어요. 작업을 보여 주면서 무엇을 하는지 말해 주세요.",
    ),
    "app.demo_default_world": (
        "The scripted demos run in the default world (the G1 at the wooden table). Reset the world first.",
        "스크립트 데모는 기본 월드(나무 테이블 앞의 G1)에서만 돌아가요. 먼저 월드를 초기화해 주세요.",
    ),
    "app.mock_on": (
        "Mock AI on: point at the table, raise both hands to stop, or wave.",
        "Mock AI 켜짐: 테이블을 가리키거나, 양손을 들어 멈추게 하거나, 손을 흔들어 보세요.",
    ),
    "app.internal_error": ("Internal error: {e}", "내부 오류: {e}"),

    # ------------------------------------------------------------------ robot switch: the same task compiled for the arm
    "app.same_task_body": ("Same task, different body: {verdict}", "같은 작업, 다른 몸: {verdict}"),
    "app.compile_all_reachable": (
        "Same intent compiled for a UR5e arm: all {n} waypoints reachable (worst error {cm} cm).",
        "같은 의도를 UR5e 팔용으로 컴파일했어요: 경유점 {n}개 모두 닿아요 (최대 오차 {cm} cm).",
    ),
    "app.compile_some_out": (
        "Same intent on a UR5e arm: {bad} of {n} waypoints out of reach, worst by {cm} cm.",
        "UR5e 팔에서는 경유점 {n}개 중 {bad}개가 팔이 닿는 범위 밖이에요 (가장 먼 곳은 {cm} cm 모자라요).",
    ),
    "app.compile_no_model": ("UR5e model not found (vendor/mink missing).", "UR5e 모델을 찾을 수 없어요 (vendor/mink 없음)."),
    "app.compile_nothing": (
        "Nothing to compile yet. Run a task or record a motion first.",
        "아직 컴파일할 게 없어요. 먼저 작업을 실행하거나 동작을 녹화해 주세요.",
    ),
    "app.compile_nothing_1": ("Nothing to compile yet.", "아직 컴파일할 게 없어요."),
    "app.compile_nothing_2": ("Run a task or record a motion first.", "먼저 작업을 실행하거나 동작을 녹화해 주세요."),
    "app.compile_only_ur5e": ("I can compile for: ur5e.", "컴파일할 수 있는 대상: ur5e."),
    "app.compiling_ur5e": ("Compiling for UR5e.", "UR5e용으로 컴파일하고 있어요."),

    # ------------------------------------------------------------------ physics ops ({type: "physics"})
    "app.physics_off": (
        "Physics is off (kinematic mode): restart without --kinematic / PC_PHYSICS=0.",
        "물리가 꺼져 있어요 (운동학 모드). --kinematic / PC_PHYSICS=0 없이 다시 시작하세요.",
    ),
    "app.physics_defaults": ("Physics back to the defaults.", "물리 설정을 기본값으로 되돌렸어요."),
    "app.physics_numbers": (
        "Physics parameters must be numbers (impedance: true/false).",
        "물리 파라미터는 숫자여야 해요 (impedance: true/false).",
    ),
    "app.physics_set": ("Physics: {shown}.", "물리 설정: {shown}."),
    "app.physics_set_unknown": ("Physics: {shown}. Unknown: {unknown}.", "물리 설정: {shown}. 알 수 없는 항목: {unknown}."),
    "app.physics_nothing_changed": ("nothing changed", "바뀐 것 없음"),
    "app.physics_drop_missing": ("I can't find '{object}' to drop.", "떨어뜨릴 '{object}' 물체를 찾을 수 없어요."),
    "app.physics_dropped": ("Dropped {obj} from {cm} cm.", "{obj|을} {cm} cm 높이에서 떨어뜨렸어요."),
    "app.physics_put": ("Put {obj} at ({x}, {y}).", "{obj|을} ({x}, {y})에 놓았어요."),
    "app.physics_unknown_op": (
        "Unknown physics op '{op}'. Use set, reset, drop or place.",
        "알 수 없는 물리 명령 '{op}'이에요. set, reset, drop, place 중 하나를 쓰세요.",
    ),

    # ------------------------------------------------------------------ dispatch
    "app.running_taught": ("Running the task '{name}' you taught me.", "가르쳐 주신 작업 '{name}' 실행할게요."),
    "app.speed_set": ("Speed set to x{speed}.", "속도를 x{speed|으로} 맞췄어요."),
    "app.nothing_recorded": ("Nothing recorded yet.", "아직 녹화된 게 없어요."),
    "app.exported": (
        "Exported intent, trajectory and report to {name}.",
        "의도, 궤적, 리포트를 {name}에 내보냈어요.",
    ),
    "app.unknown_action": ("Unknown action '{a}'.", "알 수 없는 동작이에요: '{a}'."),

    # ------------------------------------------------------------------ pps/mock.py: the offline mock brain's narration
    "mock.stopped_hands": ("You raised both hands, so I stopped.", "양손을 드셔서 멈췄어요."),
    "mock.wave": (
        "I see you waving. Hi! Say 'follow me' if you want me to copy you.",
        "손 흔드시는 거 보여요. 안녕하세요! 제가 따라 하길 원하시면 'follow me'(또는 '따라 해')라고 말하세요.",
    ),
    "mock.pointing_table": ("You're pointing at the {side} of the table.", "테이블 {side|을} 가리키고 계세요."),
    "mock.pointing_side_table": ("You're pointing at the {side} side of the table.", "테이블 {side|을} 가리키고 계세요."),
    "mock.pointing_side": ("You're pointing at the {side} side.", "{side|을} 가리키고 계세요."),
    "mock.marked_p": ("I marked it P.", "P로 표시했어요."),
    "mock.say_put_there": (
        "Say 'put the mug there'.",
        "'put the mug there'(또는 '머그를 저기에 놔')라고 말하세요.",
    ),

    # ------------------------------------------------------------------ pps/demo.py: captions, narrator and robot lines
    "demo.cap_text_only": ("Text only", "텍스트만으로"),
    "demo.cap_words": ("You can't put everything you think into words.", "생각하는 걸 전부 말로 표현할 수는 없어요."),
    "demo.cap_show_path": (
        "Show the path instead: lift it over the vase in 3D",
        "대신 경로를 보여 주세요: 3D로 꽃병 위로 들어 넘기기",
    ),
    "demo.route_checked": ("Route checked.", "경로를 확인했어요."),
    "demo.cap_point_there": ("Point at it, and say “there”", "가리키면서 “저기”라고 말하기"),
    "demo.pointing_right_table": (
        "You're pointing at the right side of the table. I marked it P.",
        "테이블 오른쪽을 가리키고 계세요. P로 표시했어요.",
    ),
    "demo.cap_teach_friend": ("Teach by showing — like teaching a friend", "시범으로 가르치기 — 친구에게 가르치듯이"),
    "demo.teach_watching": (
        "I'm watching. Show me and tell me what matters.",
        "보고 있어요. 보여 주면서 중요한 점을 말해 주세요.",
    ),
    "demo.cap_video": (
        "Some motions you can't describe. Show a video instead.",
        "말로 설명할 수 없는 동작도 있어요. 대신 영상을 보여 주세요.",
    ),
    "demo.learned_motion": (
        "Got it. I learned that motion from your video and saved it as a skill.",
        "알겠어요. 영상에서 그 동작을 배워서 스킬로 저장했어요.",
    ),
    "demo.cap_dance": ("A dance — the whole body, not just the arms", "춤 — 팔만이 아니라 온몸으로"),
    "demo.learned_dance_too": (
        "Learned the dance too: arms, torso twist and steps, checked for balance.",
        "춤도 배웠어요: 팔, 몸통 비틀기, 발걸음까지, 균형도 확인했어요.",
    ),
    "demo.cap_outro": ("New Generation of interactive prompt for Physical AI", "피지컬 AI를 위한 새로운 세대의 인터랙티브 프롬프트"),
    "demo.cap_show_path_short": ("Show the path instead: over the vase, in 3D", "대신 경로를 보여 주세요: 꽃병 위로, 3D로"),
    "demo.cap_point_short": ("Point, and say “there”", "가리키고 “저기”라고 말하기"),
    "demo.pointing_right": ("You're pointing at the right side. I marked it P.", "오른쪽을 가리키고 계세요. P로 표시했어요."),
    "demo.cap_teach_short": ("Teach it like a friend", "친구에게 가르치듯이"),
    "demo.practice_passed": (
        "Practice passed. Both steps done in simulation, the vase untouched.",
        "연습 성공. 시뮬레이션에서 두 단계를 모두 해냈고, 꽃병은 건드리지 않았어요.",
    ),
    "demo.er_sees": ("Gemini Robotics-ER sees: {labels}.", "Gemini Robotics-ER이 보고 있는 것: {labels}."),
    "demo.er_realtime": ("I can see you and the scene in real time.", "당신과 장면을 실시간으로 보고 있어요."),
    "demo.kf1": (
        "Key feature one: video to robot. Show a video, and the robot performs it.",
        "핵심 기능 하나: 영상에서 로봇으로. 영상을 보여 주면 로봇이 그대로 해내요.",
    ),
    "demo.cap_kf1": ("① Video → Robot", "① 영상 → 로봇"),
    "demo.learned_dance": (
        "Learned that dance from the video, whole body, checked for balance.",
        "영상에서 그 춤을 온몸으로 배웠고, 균형도 확인했어요.",
    ),
    "demo.kf2": (
        "Key feature two: real-time trajectory lining and pointing, with voice.",
        "핵심 기능 둘: 실시간 궤적 그리기와 가리키기, 음성과 함께.",
    ),
    "demo.cap_kf2": ("② Trajectory lining + pointing, with voice", "② 궤적 그리기 + 가리키기, 음성과 함께"),
    "demo.mug_hits_vase": (
        "I can't: the mug would hit the vase, thirteen centimeters too low.",
        "못 해요: 머그컵이 꽃병에 부딪혀요. 13센티미터나 낮아요.",
    ),
    "demo.waypoints_ok": (
        "Every waypoint checks out. Carrying it along your line.",
        "모든 경유점이 문제없어요. 그려 주신 선을 따라 옮길게요.",
    ),
    "demo.cap_pointing_there": ("Pointing: “there”", "가리키기: “저기”"),
    "demo.pointed_right": (
        "You pointed at the right side. Placing the red block there.",
        "오른쪽을 가리키셨어요. 빨간 블록을 거기에 놓을게요.",
    ),
    "demo.kf3": (
        "Key feature three: real-time video understanding. Your camera is part of the prompt.",
        "핵심 기능 셋: 실시간 영상 이해. 당신의 카메라도 프롬프트의 일부예요.",
    ),
    "demo.cap_kf3": ("③ Real-time video understanding", "③ 실시간 영상 이해"),

    # ------------------------------------------------------------------ pps/teach.py: teach by showing
    # The confirmation question is "Let me check: <steps>[. I'll <constraints>]. Is that right? ...", optionally after
    # "Updated (<changes>). ". <steps>, <constraints> and <changes> are translated piece by piece (entries below).
    "teach.ask_cons": (
        "Let me check: {steps}. I'll {cons}. Is that right? Say yes and I'll practice it in simulation.",
        "확인할게요: {steps}. 이렇게 할게요: {cons}. 맞나요? 'yes'(또는 '네')라고 하시면 시뮬레이션에서 연습해 볼게요.",
    ),
    "teach.ask": (
        "Let me check: {steps}. Is that right? Say yes and I'll practice it in simulation.",
        "확인할게요: {steps}. 맞나요? 'yes'(또는 '네')라고 하시면 시뮬레이션에서 연습해 볼게요.",
    ),
    "teach.updated_ask_cons": (
        "Updated ({changes}). Let me check: {steps}. I'll {cons}. Is that right? Say yes and I'll practice it in "
        "simulation.",
        "고쳤어요 ({changes}). 확인할게요: {steps}. 이렇게 할게요: {cons}. 맞나요? 'yes'(또는 '네')라고 하시면 "
        "시뮬레이션에서 연습해 볼게요.",
    ),
    "teach.updated_ask": (
        "Updated ({changes}). Let me check: {steps}. Is that right? Say yes and I'll practice it in simulation.",
        "고쳤어요 ({changes}). 확인할게요: {steps}. 맞나요? 'yes'(또는 '네')라고 하시면 시뮬레이션에서 연습해 볼게요.",
    ),
    # steps (describe(): "1) pick up the mug and put it on the left of the table; 2) ...")
    "teach.steps_1": ("1) pick up {o1} and put it on {t1}", "1) {o1|을} 집어서 {t1}에 놓기"),
    "teach.steps_2": (
        "1) pick up {o1} and put it on {t1}; 2) pick up {o2} and put it on {t2}",
        "1) {o1|을} 집어서 {t1}에 놓기; 2) {o2|을} 집어서 {t2}에 놓기",
    ),
    "teach.steps_3": (
        "1) pick up {o1} and put it on {t1}; 2) pick up {o2} and put it on {t2}; 3) pick up {o3} and put it on {t3}",
        "1) {o1|을} 집어서 {t1}에 놓기; 2) {o2|을} 집어서 {t2}에 놓기; 3) {o3|을} 집어서 {t3}에 놓기",
    ),
    "teach.target_table": ("the {side} of the table", "테이블 {side}"),
    "teach.target_shown": ("the spot you showed me", "보여 주신 자리"),
    "teach.target_marker": ("marker {m}", "마커 {m}"),
    "teach.target_next_to": ("the spot next to {x}", "{x} 옆 자리"),
    "teach.next_to": ("next to {x}", "{x} 옆"),
    # constraints (joined by ", "; each entry also exists with ", {rest}" so any list translates)
    "teach.con_avoid": ("stay clear of {o}", "{o} 피하기"),
    "teach.con_avoid_more": ("stay clear of {o}, {rest}", "{o} 피하기, {rest}"),
    "teach.con_keep": ("leave {o} where it is", "{o|은} 그대로 두기"),
    "teach.con_keep_more": ("leave {o} where it is, {rest}", "{o|은} 그대로 두기, {rest}"),
    "teach.con_upright": ("keep it upright", "똑바로 세운 채 옮기기"),
    "teach.con_upright_more": ("keep it upright, {rest}", "똑바로 세운 채 옮기기, {rest}"),
    "teach.con_slow": ("move slowly", "천천히 움직이기"),
    "teach.con_slow_more": ("move slowly, {rest}", "천천히 움직이기, {rest}"),
    "teach.con_top": ("grab it from the top", "위에서 잡기"),
    "teach.con_top_more": ("grab it from the top, {rest}", "위에서 잡기, {rest}"),
    "teach.con_side": ("grab it from the side", "옆에서 잡기"),
    "teach.con_side_more": ("grab it from the side, {rest}", "옆에서 잡기, {rest}"),
    # what a correction changed ("Updated (changed the target, added a constraint). ...")
    "teach.chg_step": ("added a step", "단계 추가"),
    "teach.chg_step_more": ("added a step, {rest}", "단계 추가, {rest}"),
    "teach.chg_target": ("changed the target", "놓을 곳 변경"),
    "teach.chg_target_more": ("changed the target, {rest}", "놓을 곳 변경, {rest}"),
    "teach.chg_object": ("changed the object", "물체 변경"),
    "teach.chg_object_more": ("changed the object, {rest}", "물체 변경, {rest}"),
    "teach.chg_constraint": ("added a constraint", "조건 추가"),
    "teach.chg_constraint_more": ("added a constraint, {rest}", "조건 추가, {rest}"),
    # practice
    "teach.practice_passed": (
        "Practice passed: {n} step{_s} done in simulation. Say 'save it as' and a name.",
        "연습 성공: 시뮬레이션에서 {n}단계를 마쳤어요. 'save it as' 뒤에 이름을 말하거나 '<이름>으로 저장해'라고 "
        "말하세요.",
    ),
    "teach.step_moves_kept": (
        "This step moves {x}, but you told me to leave it.",
        "이 단계는 {x|을} 옮기는데, 그대로 두라고 하셨어요.",
    ),
    "teach.replanning_over": (
        "The straight carry hits {x}; replanning a route over it.",
        "곧장 옮기면 {x|와} 부딪혀요. 그 위로 넘어가는 경로로 다시 계획할게요.",
    ),
    "teach.step_failed_reach": (
        "Step {id} failed: {message} Try a spot on my {side} side or the middle, e.g. 'put the {x} in the middle'. "
        "Tell me how to fix it, or show me again.",
        "{id}단계 실패: {message} 가운데나 제 {side} 자리로 해 보세요. 예: '{x|을} 가운데에 놔'. 어떻게 고칠지 "
        "말해 주시거나, 다시 보여 주세요.",
    ),
    "teach.step_failed_hit": (
        "Step {id} failed: {message} Draw a route over it in 3D, or tell me another spot. Tell me how to fix it, or "
        "show me again.",
        "{id}단계 실패: {message} 3D로 그 위를 넘어가는 경로를 그려 주시거나, 다른 자리를 알려 주세요. 어떻게 "
        "고칠지 말해 주시거나, 다시 보여 주세요.",
    ),
    "teach.step_failed": (
        "Step {id} failed: {message} Tell me how to fix it, or show me again.",
        "{id}단계 실패: {message} 어떻게 고칠지 말해 주시거나, 다시 보여 주세요.",
    ),
    "teach.hint_hit": (
        "Draw a route over it in 3D, or tell me another spot.",
        "3D로 그 위를 넘어가는 경로를 그려 주시거나, 다른 자리를 알려 주세요.",
    ),
    "teach.fix_or_show": ("Tell me how to fix it, or show me again.", "어떻게 고칠지 말해 주시거나, 다시 보여 주세요."),
    # save
    "teach.nothing_to_save": ("Nothing to save yet.", "아직 저장할 게 없어요."),
    "teach.saved": (
        "Saved '{name}'. Say 'do {name}' and I'll run it again.",
        "'{name}' 저장했어요. 'do {name}'(또는 '{name} 실행')이라고 말하면 다시 할게요.",
    ),

    # ------------------------------------------------------------------ pps/agent.py: chat feed (events, tool cards, say)
    "agent.llm_settings_bad": ("Your LLM settings don't work: {why}.", "LLM 설정이 동작하지 않아요: {why}."),
    "agent.url_invalid": ("that LLM URL is not valid", "LLM URL이 올바르지 않아요"),
    "agent.url_https": ("the LLM URL must start with https://", "LLM URL은 https://로 시작해야 해요"),
    "agent.url_resolve": ("can't resolve {host}", "{host} 주소를 찾을 수 없어요"),
    "agent.url_public": ("the LLM URL must be a public address", "LLM URL은 공개 주소여야 해요"),
    "agent.no_llm": ("No LLM is configured.", "설정된 LLM이 없어요."),
    "agent.no_llm_byok": ("Add your own LLM key in Settings.", "설정에서 내 LLM 키를 추가하세요."),
    "agent.no_llm_env": (
        "Set LLM_BASE_URL and LLM_API_KEY in .env, or use the offline parser.",
        ".env에 LLM_BASE_URL과 LLM_API_KEY를 설정하거나, 오프라인 파서를 쓰세요.",
    ),
    "agent.busy": (
        "I'm still working on your last request. Say stop to interrupt the robot.",
        "아직 지난 요청을 처리하고 있어요. 로봇을 멈추려면 'stop'(또는 '멈춰')이라고 말하세요.",
    ),
    "agent.cleared": ("Conversation cleared.", "대화를 지웠어요."),
    "agent.went_wrong": ("Something went wrong: {e}", "문제가 생겼어요: {e}"),
    "agent.thinking": ("thinking · {model}", "생각 중 · {model}"),
    "agent.model_no_answer": ("The model didn't answer: {e}", "모델이 답하지 않았어요: {e}"),
    "agent.llm_http": ("the model returned HTTP {code}: {detail}", "모델이 HTTP {code} 오류를 돌려줬어요: {detail}"),
    "agent.llm_unreachable": ("couldn't reach the model ({why})", "모델에 연결할 수 없었어요 ({why})"),
    "agent.llm_not_json": ("the model sent something that isn't JSON", "모델이 JSON이 아닌 응답을 보냈어요"),
    "agent.llm_no_output": ("the model's answer had no output", "모델 응답에 출력이 없었어요"),
    "agent.sorry_no_answer": (
        "Sorry, my language model didn't answer. Try again, or switch it off in Settings to use the offline parser.",
        "죄송해요, 언어 모델이 답하지 않았어요. 다시 해 보시거나, 설정에서 끄고 오프라인 파서를 쓰세요.",
    ),
    "agent.done_reply": ("Done.", "완료했어요."),
    "agent.out_of_steps": (
        "I couldn't finish that in a few steps. Here's where I got to; tell me how to continue.",
        "몇 단계 안에 끝내지 못했어요. 여기까지 했어요. 어떻게 이어 갈지 알려 주세요.",
    ),
    # tool cards (_summary)
    "agent.sum_objects": ("{n} objects: {names}", "물체 {n}개: {names}"),
    "agent.sum_looked": ("looked around", "주변을 둘러봤어요"),
    "agent.sum_zones": ("{n} zones: {labels}", "구역 {n}개: {labels}"),
    "agent.sum_zone": ("{n} zone: {labels}", "구역 {n}개: {labels}"),
    "agent.sum_no_zones": ("no zones", "구역 없음"),
    "agent.sum_skills": ("{n} skills: {names}", "스킬 {n}개: {names}"),
    "agent.sum_skills_none": ("{n} skills:", "스킬 {n}개"),
    "agent.sum_world": ("{label} · {n} objects", "{label} · 물체 {n}개"),
    "agent.status_failed": ("failed", "실패"),
    "agent.status_done": ("done", "완료"),
    "agent.status_running": ("running", "진행 중"),
    "agent.status_timeout": ("timeout", "시간 초과"),
    "agent.status_proposed": ("proposed", "제안됨"),
    "agent.status_asking": ("asking", "질문 중"),
    # tool results shown on the cards
    "agent.unknown_tool": ("unknown tool {name}", "알 수 없는 도구: {name}"),
    "agent.tool_crashed": ("tool crashed: {e}", "도구 오류: {e}"),
    "agent.which_object": ("which object?", "어떤 물체요?"),
    "agent.give_target": ("give a target, marker or xyz", "목표 위치, 마커, 또는 xyz 좌표가 필요해요"),
    "agent.marker_failed": ("couldn't place a marker at that point", "그 지점에 마커를 놓을 수 없었어요"),
    "agent.plan_waiting": ("Plan {id} is already waiting for the user.", "계획 {id|이} 이미 사용자의 답을 기다리고 있어요."),
    "agent.plan_waiting_stop": (
        "Stop and ask them to say go, or change it after they answer.",
        "여기서 멈추고 'go'라고 해 달라고 물어보거나, 답을 들은 뒤에 바꾸세요.",
    ),
    "agent.plan_waiting_traj": (
        "Stop and ask them to say go, or change it with edit_trajectory after they answer.",
        "여기서 멈추고 'go'라고 해 달라고 물어보거나, 답을 들은 뒤에 edit_trajectory로 바꾸세요.",
    ),
    "agent.plan_waiting_skill": (
        "Ask them to say go, or change it with edit_skill_params.",
        "'go'라고 해 달라고 물어보거나, edit_skill_params로 바꾸세요.",
    ),
    "agent.took_over": ("Another command took over before it finished.", "끝나기 전에 다른 명령이 이어받았어요."),
    "agent.robot_timeout": ("The robot didn't answer in time.", "로봇이 제때 답하지 않았어요."),
    "agent.no_traj": ("Trajectories aren't available on this server.", "이 서버에서는 궤적 기능을 쓸 수 없어요."),
    "agent.need_keypoint": (
        "give at least one keypoint (where the hand should go)",
        "키포인트를 하나 이상 주세요 (손이 가야 할 곳)",
    ),
    "agent.say_what_change": (
        "say what to change (changes, pattern or remove_pattern)",
        "무엇을 바꿀지 알려 주세요 (changes, pattern 또는 remove_pattern)",
    ),
    "agent.no_copresence": (
        "Co-presence is not available (no person in view).",
        "마주 보기 기능을 쓸 수 없어요 (화면에 사람이 없어요).",
    ),
    "agent.base_timeout": ("The base didn't finish driving in time.", "베이스가 제때 이동을 마치지 못했어요."),
    "agent.zone_not_found": (
        "No change: that object or zone wasn't found.",
        "바뀐 것 없음: 그 물체나 구역을 찾지 못했어요.",
    ),
    "agent.zone_usage": (
        "give near (an object) or rect [x0, y0, x1, y1]",
        "near(물체)나 rect [x0, y0, x1, y1]가 필요해요",
    ),
    "agent.no_zones_were": ("There were no zones.", "구역이 없었어요."),
    "agent.targets_usage": ("targets must map object names to [x, y]", "targets는 물체 이름마다 [x, y]를 정해야 해요"),
    "agent.goal_rejected": ("The goal wasn't accepted.", "목표가 받아들여지지 않았어요."),
    "agent.no_question": ("there is no open question", "열린 질문이 없어요"),
    "agent.no_answer_yet": ("No answer yet.", "아직 답이 없어요."),
    "agent.no_plan_waiting": ("no plan is waiting for approval", "승인을 기다리는 계획이 없어요"),
    "agent.no_plan_card": ("there is no plan card", "계획 카드가 없어요"),
    "agent.cancelled": ("Cancelled.", "취소했어요."),
    "agent.teach_watching": (
        "Watching the user's camera now. They show the task, explain it, and say done.",
        "지금 사용자의 카메라를 보고 있어요. 작업을 보여 주고 설명한 뒤 'done'(또는 '끝')이라고 말하면 돼요.",
    ),
    "agent.no_watch": ("Watching isn't available on this server.", "이 서버에서는 지켜보기 기능을 쓸 수 없어요."),
    "agent.no_skills": ("Skills aren't available on this server.", "이 서버에서는 스킬 기능을 쓸 수 없어요."),
    "agent.need_words": ("give the person's words (text) or changes", "사람이 한 말(text)이나 changes가 필요해요"),
    "agent.which_fixture": ("which drawer or door?", "어느 서랍이나 문이요?"),
    "agent.need_step": ("give at least one step", "단계를 하나 이상 주세요"),
    "agent.no_world_build": (
        "not available in this build: this server has one robot (Unitree G1) and a fixed table scene.",
        "이 빌드에서는 쓸 수 없어요: 이 서버에는 로봇 하나(Unitree G1)와 고정된 테이블 장면만 있어요.",
    ),
    "agent.not_in_build": ("not available in this build", "이 빌드에서는 쓸 수 없어요"),
    "agent.world_unchanged": ("The world didn't change (in time).", "월드가 (제때) 바뀌지 않았어요."),
    "agent.calib_needs_physics": (
        "Calibration needs physics; this server runs the kinematic fallback.",
        "보정에는 물리가 필요해요. 이 서버는 운동학 모드로 돌아가고 있어요.",
    ),
    "agent.calib_running": (
        "Calibration is still running; I'll keep going in the background.",
        "보정이 아직 진행 중이에요. 백그라운드에서 계속할게요.",
    ),
    "agent.object_needs_name": ("object needs at least a name", "물체에는 적어도 이름이 있어야 해요"),
    "agent.camera_timeout": ("The camera picture didn't arrive in time.", "카메라 사진이 제때 오지 않았어요."),
    "agent.still_reconstructing": (
        "Still reconstructing; it will appear in the scene when done.",
        "아직 복원하고 있어요. 끝나면 장면에 나타나요.",
    ),
}
