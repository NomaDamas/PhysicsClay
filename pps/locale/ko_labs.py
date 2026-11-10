"""Korean server messages: labs (pps/features/calib.py, pps/calib/, pps/features/recon.py, pps/recon/,
pps/features/dataset.py, pps/data/). English is the message id; see pps/i18n.py."""

MESSAGES = {
    # key: (English template, Korean template)

    # ------------------------------------------------------------------ pps/features/calib.py: the calib ops
    "calib.stopped": ("Calibration stopped.", "보정을 멈췄어요."),
    "calib.reset": (
        "Calibration reset: physics back to the defaults.",
        "보정을 초기화했어요: 물리 설정을 기본값으로 되돌렸어요.",
    ),
    "calib.needs_physics": (
        "Calibration needs physics: the server runs in kinematic mode (restart without --kinematic).",
        "보정하려면 물리 모드가 필요해요: 지금 서버가 운동학 모드로 돌고 있어요(--kinematic 없이 다시 시작하세요).",
    ),
    "calib.already_probing": (
        "I'm already probing; say stop first.",
        "이미 프로브 동작 중이에요. 먼저 '멈춰'라고 말하세요.",
    ),
    "calib.already_fitting": (
        "I'm already fitting; say stop first.",
        "이미 시뮬레이터를 맞추는 중이에요. 먼저 '멈춰'라고 말하세요.",
    ),
    "calib.busy_probing": ("I'm probing right now.", "지금 프로브 동작 중이에요."),
    "calib.busy_fitting": ("I'm fitting right now.", "지금 시뮬레이터를 맞추는 중이에요."),
    "calib.unknown_op": (
        "Unknown calib op '{op}'. Use probe, fit, apply, reset, run or stop.",
        "알 수 없는 보정 명령이에요: '{op}'. probe, fit, apply, reset, run, stop 중 하나를 쓰세요.",
    ),
    "calib.busy_task": (
        "I'm calibrating (probing the table): say stop first, or wait until it's done.",
        "지금 보정 중이에요(테이블에서 프로브 동작 중). 먼저 '멈춰'라고 말하거나 끝날 때까지 기다려 주세요.",
    ),

    # ------------------------------------------------------------------ probing: start, progress, safety
    "calib.safety_abort": (
        "someone came close to the robot (safety). Keep the table clear and say 'calibrate' again",
        "누군가 로봇 가까이 왔어요(안전). 테이블 주변을 비우고 다시 'calibrate'(또는 '보정해')라고 말하세요",
    ),
    "calib.cant_probe_person": (
        "I can't probe right now: someone is close to me (safety). Keep the table clear and ask again.",
        "지금은 프로브 동작을 할 수 없어요: 누군가 제 가까이에 있어요(안전). 테이블 주변을 비우고 다시 요청하세요.",
    ),
    "calib.no_probes": (
        "No probes to run (swing, press, lift, slide).",
        "실행할 프로브 동작이 없어요(swing, press, lift, slide).",
    ),
    "calib.start": (
        "Calibrating: I'll run {n} short probes ({names}). Keep the table clear.",
        "보정할게요: 짧은 프로브 동작 {n}개를 할게요({names}). 테이블 주변을 비워 두세요.",
    ),
    "calib.world_changed": ("The world changed while probing.", "프로브 동작 중에 월드가 바뀌었어요."),
    "calib.probing": ("Probing: {probe} ({i}/{n})", "프로브 동작: {probe} ({i}/{n})"),
    "calib.probing_swing": ("Probing: Swing ({i}/{n})", "프로브 동작: 흔들기 ({i}/{n})"),
    "calib.probing_press": ("Probing: Press ({i}/{n})", "프로브 동작: 누르기 ({i}/{n})"),
    "calib.probing_lift": ("Probing: Lift ({i}/{n})", "프로브 동작: 들기 ({i}/{n})"),
    "calib.probing_slide": ("Probing: Slide ({i}/{n})", "프로브 동작: 밀기 ({i}/{n})"),
    "calib.label_swing": ("Swing", "흔들기"),
    "calib.label_press": ("Press", "누르기"),
    "calib.label_lift": ("Lift", "들기"),
    "calib.label_slide": ("Slide", "밀기"),

    # -------------------------------- probing: the routine's lists of probes ("swing, press, lift" inside the messages)
    "calib.list_swing_press_lift_slide": ("swing, press, lift, slide", "흔들기, 누르기, 들기, 밀기"),
    "calib.list_swing_press_lift": ("swing, press, lift", "흔들기, 누르기, 들기"),
    "calib.list_swing_press_slide": ("swing, press, slide", "흔들기, 누르기, 밀기"),
    "calib.list_swing_lift_slide": ("swing, lift, slide", "흔들기, 들기, 밀기"),
    "calib.list_press_lift_slide": ("press, lift, slide", "누르기, 들기, 밀기"),
    "calib.list_swing_press": ("swing, press", "흔들기, 누르기"),
    "calib.list_swing_lift": ("swing, lift", "흔들기, 들기"),
    "calib.list_swing_slide": ("swing, slide", "흔들기, 밀기"),
    "calib.list_press_lift": ("press, lift", "누르기, 들기"),
    "calib.list_press_slide": ("press, slide", "누르기, 밀기"),
    "calib.list_lift_slide": ("lift, slide", "들기, 밀기"),

    # -------------------------------------------------------- probing: finished, stopped, skipped (pps/calib/probes.py)
    "calib.stopped_why": ("Probing stopped: {why}.", "프로브 동작을 멈췄어요: {why}."),
    "calib.stopped_why_skipped": (
        "Probing stopped: {why}. Skipped {skipped}.",
        "프로브 동작을 멈췄어요: {why}. 건너뛴 동작: {skipped}.",
    ),
    "calib.stopped_none_kept": (
        "Probing stopped: {n} probe logs recorded; skipped {names}. Skipped {skipped}.",
        "프로브 동작을 멈췄어요: 프로브 기록 {n}개를 남겼고 {names|은} 건너뛰었어요. 건너뛴 동작: {skipped}.",
    ),
    "calib.no_logs": ("no logs", "기록 없음"),
    "calib.logs_recorded": ("{n} probe logs recorded", "프로브 기록 {n}개를 남겼어요"),
    "calib.logs_recorded_skipped": (
        "{n} probe logs recorded; skipped {names}",
        "프로브 기록 {n}개를 남겼고 {names|은} 건너뛰었어요",
    ),
    "calib.probe_error": ("probe error: {err}", "프로브 동작 오류: {err}"),
    "calib.probed": ("Probed: {names} ({secs} s).", "프로브 동작을 마쳤어요: {names} ({secs} s)."),
    "calib.skipped": ("Skipped {skipped}.", "건너뛴 동작: {skipped}."),
    "calib.skipped_one": ("Skipped {probe}: {why}.", "{probe} 동작은 건너뛰었어요: {why}."),
    "calib.skipped_swing": ("Skipped swing: {why}.", "흔들기 동작은 건너뛰었어요: {why}."),
    "calib.skipped_press": ("Skipped press: {why}.", "누르기 동작은 건너뛰었어요: {why}."),
    "calib.skipped_lift": ("Skipped lift: {why}.", "들기 동작은 건너뛰었어요: {why}."),
    "calib.skipped_slide": ("Skipped slide: {why}.", "밀기 동작은 건너뛰었어요: {why}."),
    "calib.skipped_swing_press": ("Skipped swing: {r1}; press: {r2}.", "건너뛴 동작: 흔들기({r1}), 누르기({r2})."),
    "calib.skipped_swing_lift": ("Skipped swing: {r1}; lift: {r2}.", "건너뛴 동작: 흔들기({r1}), 들기({r2})."),
    "calib.skipped_swing_slide": ("Skipped swing: {r1}; slide: {r2}.", "건너뛴 동작: 흔들기({r1}), 밀기({r2})."),
    "calib.skipped_press_lift": ("Skipped press: {r1}; lift: {r2}.", "건너뛴 동작: 누르기({r1}), 들기({r2})."),
    "calib.skipped_press_slide": ("Skipped press: {r1}; slide: {r2}.", "건너뛴 동작: 누르기({r1}), 밀기({r2})."),
    "calib.skipped_lift_slide": ("Skipped lift: {r1}; slide: {r2}.", "건너뛴 동작: 들기({r1}), 밀기({r2})."),
    "calib.skipped_swing_press_lift": (
        "Skipped swing: {r1}; press: {r2}; lift: {r3}.",
        "건너뛴 동작: 흔들기({r1}), 누르기({r2}), 들기({r3}).",
    ),
    "calib.skipped_swing_press_slide": (
        "Skipped swing: {r1}; press: {r2}; slide: {r3}.",
        "건너뛴 동작: 흔들기({r1}), 누르기({r2}), 밀기({r3}).",
    ),
    "calib.skipped_swing_lift_slide": (
        "Skipped swing: {r1}; lift: {r2}; slide: {r3}.",
        "건너뛴 동작: 흔들기({r1}), 들기({r2}), 밀기({r3}).",
    ),
    "calib.skipped_press_lift_slide": (
        "Skipped press: {r1}; lift: {r2}; slide: {r3}.",
        "건너뛴 동작: 누르기({r1}), 들기({r2}), 밀기({r3}).",
    ),
    "calib.skipped_swing_press_lift_slide": (
        "Skipped swing: {r1}; press: {r2}; lift: {r3}; slide: {r4}.",
        "건너뛴 동작: 흔들기({r1}), 누르기({r2}), 들기({r3}), 밀기({r4}).",
    ),
    "calib.skiplist_swing": ("swing: {r1}", "흔들기({r1})"),
    "calib.skiplist_press": ("press: {r1}", "누르기({r1})"),
    "calib.skiplist_lift": ("lift: {r1}", "들기({r1})"),
    "calib.skiplist_slide": ("slide: {r1}", "밀기({r1})"),
    "calib.skiplist_swing_press": ("swing: {r1}; press: {r2}", "흔들기({r1}), 누르기({r2})"),
    "calib.skiplist_swing_lift": ("swing: {r1}; lift: {r2}", "흔들기({r1}), 들기({r2})"),
    "calib.skiplist_swing_slide": ("swing: {r1}; slide: {r2}", "흔들기({r1}), 밀기({r2})"),
    "calib.skiplist_press_lift": ("press: {r1}; lift: {r2}", "누르기({r1}), 들기({r2})"),
    "calib.skiplist_press_slide": ("press: {r1}; slide: {r2}", "누르기({r1}), 밀기({r2})"),
    "calib.skiplist_lift_slide": ("lift: {r1}; slide: {r2}", "들기({r1}), 밀기({r2})"),
    "calib.skiplist_swing_press_lift": (
        "swing: {r1}; press: {r2}; lift: {r3}",
        "흔들기({r1}), 누르기({r2}), 들기({r3})",
    ),
    "calib.skiplist_swing_press_slide": (
        "swing: {r1}; press: {r2}; slide: {r3}",
        "흔들기({r1}), 누르기({r2}), 밀기({r3})",
    ),
    "calib.skiplist_swing_lift_slide": ("swing: {r1}; lift: {r2}; slide: {r3}", "흔들기({r1}), 들기({r2}), 밀기({r3})"),
    "calib.skiplist_press_lift_slide": ("press: {r1}; lift: {r2}; slide: {r3}", "누르기({r1}), 들기({r2}), 밀기({r3})"),
    "calib.skiplist_swing_press_lift_slide": (
        "swing: {r1}; press: {r2}; lift: {r3}; slide: {r4}",
        "흔들기({r1}), 누르기({r2}), 들기({r3}), 밀기({r4})",
    ),
    "calib.skipped_two": ("Skipped {p1}: {r1}; {p2}: {r2}.", "건너뛴 동작: {p1}({r1}), {p2}({r2})."),
    "calib.skipped_three": (
        "Skipped {p1}: {r1}; {p2}: {r2}; {p3}: {r3}.",
        "건너뛴 동작: {p1}({r1}), {p2}({r2}), {p3}({r3}).",
    ),
    "calib.skip_no_arm_joints": ("no servoed arm joints to swing", "흔들 수 있는 서보 팔 관절이 없어요"),
    "calib.skip_no_swing_clear": (
        "no joint can swing clear of the table here",
        "여기서는 테이블에 닿지 않고 흔들 수 있는 관절이 없어요",
    ),
    "calib.skip_no_press_spot": (
        "no free spot of the table in reach to press on",
        "팔이 닿는 범위에 누를 수 있는 빈 테이블 자리가 없어요",
    ),
    "calib.skip_no_gripper": ("no gripper", "그리퍼가 없어요"),
    "calib.skip_nothing_in_reach": ("nothing I can grasp in reach", "팔이 닿는 범위에 집을 수 있는 게 없어요"),
    "calib.skip_nothing_to_slide": ("no object I can hold and slide here", "여기에는 잡고 밀 수 있는 물체가 없어요"),
    "calib.say_fit": (
        "Say 'fit' to calibrate the simulator to it.",
        "시뮬레이터를 여기에 맞추려면 'fit'이라고 말하세요.",
    ),

    # ------------------------------------------------------------------ fitting (pps/calib/sysid.py stages and budgets)
    "calib.nothing_to_fit": (
        "Nothing to fit yet: run the probes first (op: probe).",
        "아직 맞출 게 없어요: 먼저 프로브 동작을 실행하세요(op: probe).",
    ),
    "calib.fitting": (
        "Fitting the simulator to {n} probe logs ({budget}).",
        "프로브 기록 {n}개에 시뮬레이터를 맞추고 있어요({budget}).",
    ),
    "calib.fitting_quick": (
        "Fitting the simulator to {n} probe logs (quick).",
        "프로브 기록 {n}개에 시뮬레이터를 맞추고 있어요(빠르게).",
    ),
    "calib.fitting_normal": (
        "Fitting the simulator to {n} probe logs (normal).",
        "프로브 기록 {n}개에 시뮬레이터를 맞추고 있어요(보통).",
    ),
    "calib.fitting_thorough": (
        "Fitting the simulator to {n} probe logs (thorough).",
        "프로브 기록 {n}개에 시뮬레이터를 맞추고 있어요(꼼꼼하게).",
    ),
    "calib.fit_progress": (
        "Fitting: {stage} ({i}/{n}), generation {gen}/{gens}",
        "맞추는 중: {stage} ({i}/{n}), {gen}/{gens}세대",
    ),
    "calib.fit_progress_dynamics": (
        "Fitting: dynamics ({i}/{n}), generation {gen}/{gens}",
        "맞추는 중: 동역학 ({i}/{n}), {gen}/{gens}세대",
    ),
    "calib.fit_progress_mass": (
        "Fitting: mass ({i}/{n}), generation {gen}/{gens}",
        "맞추는 중: 질량 ({i}/{n}), {gen}/{gens}세대",
    ),
    "calib.fit_progress_friction": (
        "Fitting: friction ({i}/{n}), generation {gen}/{gens}",
        "맞추는 중: 마찰 ({i}/{n}), {gen}/{gens}세대",
    ),
    "calib.fit_progress_other": (
        "Fitting: other ({i}/{n}), generation {gen}/{gens}",
        "맞추는 중: 기타 ({i}/{n}), {gen}/{gens}세대",
    ),
    "calib.fit_progress_polish": (
        "Fitting: polish ({i}/{n}), generation {gen}/{gens}",
        "맞추는 중: 마무리 조정 ({i}/{n}), {gen}/{gens}세대",
    ),
    "calib.fit_failed": ("The fit failed: {error}", "맞추기에 실패했어요: {error}"),

    # -------------------------------------------- the fit's one-sentence summary (summarize) and the task line after it
    "calib.calibrated_gain": (
        "Calibrated: {parts}; joint tracking error {q0}° → {q1}°.",
        "보정했어요: {parts}. 관절 추종 오차 {q0}° → {q1}°.",
    ),
    "calib.calibrated": ("Calibrated: {parts}.", "보정했어요: {parts}."),
    "calib.calibrated_gain_say_apply": (
        "Calibrated: {parts}; joint tracking error {q0}° → {q1}°. Say 'apply' to use it.",
        "보정했어요: {parts}. 관절 추종 오차 {q0}° → {q1}°. 적용하려면 'apply'라고 말하세요.",
    ),
    "calib.calibrated_say_apply": (
        "Calibrated: {parts}. Say 'apply' to use it.",
        "보정했어요: {parts}. 적용하려면 'apply'라고 말하세요.",
    ),
    "calib.calibrated_gain_applied": (
        "Calibrated: {parts}; joint tracking error {q0}° → {q1}°. Applied.",
        "보정했어요: {parts}. 관절 추종 오차 {q0}° → {q1}°. 적용했어요.",
    ),
    "calib.calibrated_applied": ("Calibrated: {parts}. Applied.", "보정했어요: {parts}. 적용했어요."),
    "calib.calibrated_plain": ("Calibrated.", "보정했어요."),
    "calib.applied": ("Applied.", "적용했어요."),
    "calib.say_apply": ("Say 'apply' to use it.", "적용하려면 'apply'라고 말하세요."),
    "calib.part_friction": ("friction {v}", "마찰 {v}"),
    "calib.part_mass": ("mass {v}", "질량 {v}"),
    "calib.part_damping": ("damping {v}", "감쇠 {v}"),
    "calib.part_stiffness": ("stiffness {v}", "강성 {v}"),
    "calib.part_rotor_inertia": ("rotor inertia {v}", "회전자 관성 {v}"),
    "calib.part_joint_friction": ("joint friction {v}", "관절 마찰 {v}"),
    "calib.part_latency": ("latency {v} ms", "지연 {v} ms"),
    "calib.as_modelled": ("as modelled", "모델 그대로"),
    "calib.slippery": ("×{v} (slippery)", "×{v} (미끄러움)"),
    "calib.grippy": ("×{v} (grippy)", "×{v} (마찰 큼)"),

    # ------------------------------------------------------------------ applying the fit
    "calib.nothing_fitted": (
        "Nothing fitted yet: run the probes and the fit first.",
        "아직 맞춘 결과가 없어요: 먼저 프로브 동작과 맞추기를 실행하세요.",
    ),
    "calib.wrong_robot": (
        "That fit is for the {robot}; probe this robot first.",
        "그 결과는 {robot}용이에요. 먼저 이 로봇으로 프로브 동작을 하세요.",
    ),
    "calib.applied_to": ("Applied to the simulator: {shown}.", "시뮬레이터에 적용했어요: {shown}."),

    # --------------------------------------------------------- pps/features/recon.py: photo import, table scan, library
    "recon.no_camera": (
        "No camera is streaming. Open the phone camera (Settings › Phone camera) or turn on the browser camera, or "
        "upload a photo instead.",
        "스트리밍 중인 카메라가 없어요. 휴대폰 카메라를 열거나(설정 › 휴대폰 카메라) 브라우저 카메라를 켜세요. 아니면 "
        "사진을 업로드해 주세요.",
    ),
    "recon.got_photo_phone": ("Got a photo from the phone camera.", "휴대폰 카메라에서 사진을 받았어요."),
    "recon.got_photo_browser": ("Got a photo from the browser camera.", "브라우저 카메라에서 사진을 받았어요."),
    "recon.got_photo_camera": ("Got a photo from the camera.", "카메라에서 사진을 받았어요."),
    "recon.working": ("Working on it…", "처리하고 있어요…"),
    "recon.not_ready_gone": (
        "That reconstruction isn't ready (or is gone). Take the photo again.",
        "그 재구성은 아직 준비되지 않았거나 사라졌어요. 사진을 다시 찍어 주세요.",
    ),
    "recon.adding_to_scene": ("Adding the {obj} to the scene.", "{obj|을} 장면에 추가할게요."),
    "recon.scan_not_ready": (
        "That table scan isn't ready (or is gone). Scan again.",
        "그 테이블 스캔은 아직 준비되지 않았거나 사라졌어요. 다시 스캔해 주세요.",
    ),
    "recon.adding_from_library": ("Adding the {obj} from your library.", "라이브러리에서 {obj|을} 추가할게요."),
    "recon.removed": ("Removed from the library.", "라이브러리에서 지웠어요."),
    "recon.not_in_library": ("Not in the library.", "라이브러리에 없어요."),
    "recon.unknown_op": ("Unknown recon op '{op}'.", "알 수 없는 재구성 명령이에요: '{op}'."),
    "recon.shaping_each": (
        "Shaping each object from the photo, then building the scene…",
        "사진 속 물체마다 모양을 만든 다음 장면을 만들고 있어요…",
    ),
    "recon.scan_no_objects": ("The scan found no objects to place.", "스캔에서 놓을 물체를 찾지 못했어요."),
    "recon.building": ("Building your {surface} with {n} objects.", "물체 {n}개가 놓인 {surface|을} 만들고 있어요."),
    "recon.failed": ("The reconstruction failed.", "재구성에 실패했어요."),

    # --------------------------------------------------- pps/recon/jobs.py: job steps (labels) and progress (label + …)
    "recon.step_recognizing": ("Recognizing it", "무엇인지 알아보기"),
    "recon.step_recognizing_msg": ("Recognizing it…", "무엇인지 알아보고 있어요…"),
    "recon.step_estimating_depth": ("Estimating depth", "깊이 추정"),
    "recon.step_estimating_depth_msg": ("Estimating depth…", "깊이를 추정하고 있어요…"),
    "recon.step_finding_outline": ("Finding the outline", "윤곽 찾기"),
    "recon.step_finding_outline_msg": ("Finding the outline…", "윤곽을 찾고 있어요…"),
    "recon.step_fitting_shape": ("Fitting a 3D shape", "3D 모양 맞추기"),
    "recon.step_fitting_shape_msg": ("Fitting a 3D shape…", "3D 모양을 맞추고 있어요…"),
    "recon.step_setting_size": ("Setting the real size", "실제 크기 정하기"),
    "recon.step_setting_size_msg": ("Setting the real size…", "실제 크기를 정하고 있어요…"),
    "recon.step_making_physical": ("Making it physical", "물리 속성 입히기"),
    "recon.step_making_physical_msg": ("Making it physical…", "물리 속성을 입히고 있어요…"),
    "recon.step_finding_objects": ("Finding the objects", "물체 찾기"),
    "recon.step_finding_objects_msg": ("Finding the objects…", "물체를 찾고 있어요…"),
    "recon.step_finding_table": ("Finding the table", "테이블 찾기"),
    "recon.step_finding_table_msg": ("Finding the table…", "테이블을 찾고 있어요…"),
    "recon.step_measuring_layout": ("Measuring the layout", "배치 측정"),
    "recon.step_measuring_layout_msg": ("Measuring the layout…", "배치를 재고 있어요…"),
    "recon.step_shaping_objects": ("Shaping the objects", "물체 모양 만들기"),
    "recon.step_shaping_objects_msg": ("Shaping the objects…", "물체 모양을 만들고 있어요…"),
    "recon.waiting": ("Waiting…", "기다리는 중…"),
    "recon.ready": ("Ready.", "준비됐어요."),
    "recon.went_wrong": ("Something went wrong while reconstructing ({e}).", "재구성하다가 문제가 생겼어요({e})."),

    # ---------------------- pps/recon/http.py (JSON answers to the upload) and pps/recon/pipeline.py: reading the files
    "recon.too_large": ("That file is too large (25 MB max).", "파일이 너무 커요(최대 25 MB)."),
    "recon.no_photo": ("No photo in the upload.", "업로드에 사진이 없어요."),
    "recon.not_image": (
        "That file isn't an image I can read (JPEG, PNG or WebP).",
        "제가 읽을 수 있는 이미지 파일이 아니에요(JPEG, PNG, WebP).",
    ),
    "recon.depth_file": (
        "The depth file must be a 16-bit PNG in millimetres or a .npy array in metres.",
        "깊이 파일은 밀리미터 단위 16비트 PNG나 미터 단위 .npy 배열이어야 해요.",
    ),

    # ------------------------------------------------------------------ pps/recon/pipeline.py: one object from photos
    "recon.unavailable": ("{prov} isn't available: {why}.", "{prov|은} 지금 쓸 수 없어요: {why}."),
    "recon.looks_like": ("Looks like a {label}.", "{label|으로} 보여요."),
    "recon.looks_like_material": ("Looks like a {label} ({material}).", "{label|으로} 보여요({material})."),
    "recon.looks_like_ceramic": ("Looks like a {label} (ceramic).", "{label|으로} 보여요(도자기)."),
    "recon.looks_like_glass": ("Looks like a {label} (glass).", "{label|으로} 보여요(유리)."),
    "recon.looks_like_plastic": ("Looks like a {label} (plastic).", "{label|으로} 보여요(플라스틱)."),
    "recon.looks_like_metal": ("Looks like a {label} (metal).", "{label|으로} 보여요(금속)."),
    "recon.looks_like_wood": ("Looks like a {label} (wood).", "{label|으로} 보여요(나무)."),
    "recon.looks_like_paper": ("Looks like a {label} (paper).", "{label|으로} 보여요(종이)."),
    "recon.looks_like_cardboard": ("Looks like a {label} (cardboard).", "{label|으로} 보여요(골판지)."),
    "recon.looks_like_fabric": ("Looks like a {label} (fabric).", "{label|으로} 보여요(천)."),
    "recon.looks_like_rubber": ("Looks like a {label} (rubber).", "{label|으로} 보여요(고무)."),
    "recon.looks_like_food": ("Looks like a {label} (food).", "{label|으로} 보여요(음식)."),
    "recon.looks_like_stone": ("Looks like a {label} (stone).", "{label|으로} 보여요(돌)."),
    "recon.downloading_depth": (
        "Downloading the depth model once (~{mb} MB)…",
        "깊이 모델을 한 번 내려받고 있어요(~{mb} MB)…",
    ),
    "recon.size_clamped": ("size clamped to {a} cm (from {b} cm)", "크기를 {a} cm로 맞췄어요(원래 {b} cm)"),
    "recon.result": ("{label}: {h} cm tall, {g} g ({src}).", "{label}: 높이 {h} cm, {g} g (크기 기준: {src})."),
    "recon.result_guess": ("{label}: {h} cm tall, {g} g (a guess).", "{label}: 높이 {h} cm, {g} g (크기는 추정치)."),
    "recon.not_ready": ("That reconstruction isn't ready.", "그 재구성은 아직 준비되지 않았어요."),
    "recon.not_in_library_any_more": (
        "That object isn't in the library any more.",
        "그 물체는 이제 라이브러리에 없어요.",
    ),

    # ------------------------------------------------------------------ pps/recon/pipeline.py + space.py: a table scan
    "recon.found": (
        "Found {n} object{_s} on the {surface} (scale from {source}).",
        "{surface}에서 물체 {n}개를 찾았어요(크기 기준: {source}).",
    ),
    "recon.found_none_photo": (
        "I couldn't find objects on the table in that photo.",
        "그 사진에서는 테이블 위 물체를 찾지 못했어요.",
    ),
    "recon.found_none": ("I couldn't find objects on the table.", "테이블 위 물체를 찾지 못했어요."),
    "recon.no_objects_found": ("no objects found", "찾은 물체가 없어요"),
    "recon.shaping_label": ("Shaping the {label}…", "{label} 모양을 만들고 있어요…"),
    "recon.shaped": (
        "Shaped {n} of {total} objects from the photo.",
        "사진 속 물체 {total}개 중 {n}개의 모양을 만들었어요.",
    ),
    "recon.layout_squeezed": (
        "layout squeezed to {k} % to fit the robot's reach",
        "로봇 팔이 닿는 범위에 맞추려고 배치를 {k} %로 줄였어요",
    ),

    # -------------------------------------- where the size came from (pps/recon/scale.py labels, space.py table planes)
    "recon.scale_you": ("what you said", "말씀하신 크기"),
    "recon.scale_aruco": ("the ArUco marker", "ArUco 마커"),
    "recon.scale_card": ("the card in the photo", "사진 속 카드"),
    "recon.scale_a4": ("the A4 sheet", "A4 용지"),
    "recon.scale_depth": ("the depth camera", "깊이 카메라"),
    "recon.scale_vlm": ("the vision model's estimate", "비전 모델의 추정"),
    "recon.scale_prior": ("a typical size for it ({kind})", "{kind}의 일반적인 크기"),
    "recon.scale_default": ("a guess", "추정치"),
    "recon.plane_assumed": (
        "an assumed camera (35° down, 50 cm above the table)",
        "가정한 카메라 위치(35° 아래로, 테이블 위 50 cm)",
    ),
    "recon.plane_marker": ("the ArUco marker on the table", "테이블 위 ArUco 마커"),
    "recon.plane_relative": ("relative depth (Depth Anything V2 Small)", "상대 깊이(Depth Anything V2 Small)"),
    "recon.scale_typical": ("typical sizes of what's on it", "올려진 물건들의 일반적인 크기"),
    "recon.scale_guess_10cm": ("a guess (~10 cm objects)", "추정치(물체 ~10 cm로 가정)"),

    # --------------------------------------------------------------- pps/recon/providers: which ones work, their errors
    "recon.prov_primitive": ("Quick shape (offline)", "빠른 모양(오프라인)"),
    "recon.prov_triposr": ("Local AI (TripoSR)", "로컬 AI(TripoSR)"),
    "recon.prov_trellis": ("Cloud: TRELLIS (fal.ai)", "클라우드: TRELLIS (fal.ai)"),
    "recon.prov_meshy": ("Cloud: Meshy", "클라우드: Meshy"),
    "recon.why_add_key": ("add a {svc} key", "{svc} 키를 추가하세요"),
    "recon.why_needs_pip": (
        "needs {what} (pip install 'physicsclay[recon]')",
        "{what|이} 필요해요(pip install 'physicsclay[recon]')",
    ),
    "recon.why_needs_triposr": (
        "needs the TripoSR source (clone github.com/VAST-AI-Research/TripoSR to vendor/TripoSR)",
        "TripoSR 소스가 필요해요(github.com/VAST-AI-Research/TripoSR를 vendor/TripoSR에 clone하세요)",
    ),
    "recon.why_triposr_off": (
        "off: set PC_TRIPOSR=1 to allow the one-time 1.7 GB weights download",
        "꺼져 있어요: PC_TRIPOSR=1로 설정하면 1.7 GB 가중치를 한 번 내려받을 수 있어요",
    ),
    "recon.no_outline": (
        "I couldn't find the object's outline in that photo. Drag a box around it and try again.",
        "그 사진에서 물체의 윤곽을 찾지 못했어요. 물체 주위로 상자를 그리고 다시 해 보세요.",
    ),
    "recon.outline_too_small": (
        "The object's outline is too small. Move closer or crop tighter.",
        "물체의 윤곽이 너무 작아요. 더 가까이 가거나 더 좁게 잘라 주세요.",
    ),
    "recon.mesh_unreadable": (
        "{svc} sent a mesh I couldn't read ({err}).",
        "{svc}에서 받은 메시를 읽을 수 없었어요({err}).",
    ),
    "recon.mesh_empty": ("{svc} sent an empty mesh.", "{svc}에서 빈 메시를 보냈어요."),
    "recon.key_rejected": ("{svc}: the key was rejected ({detail})", "{svc}: 키가 거부됐어요({detail})"),
    "recon.no_credits": (
        "{svc}: the account has no credits (Meshy's API needs a paid plan) ({detail})",
        "{svc}: 계정에 크레딧이 없어요(Meshy API는 유료 플랜이 필요해요) ({detail})",
    ),
    "recon.refused": (
        "{svc}: refused (key or content policy) ({detail})",
        "{svc}: 거부됐어요(키 또는 콘텐츠 정책) ({detail})",
    ),
    "recon.too_many": (
        "{svc}: too many requests, try again in a minute ({detail})",
        "{svc}: 요청이 너무 많아요. 1분 뒤에 다시 해 보세요 ({detail})",
    ),
    "recon.http_error": ("{svc}: HTTP {status} ({detail})", "{svc}: HTTP {status} 오류({detail})"),
    "recon.fal_needs_key": (
        "TRELLIS on fal.ai needs a fal.ai key (Settings in the Real panel).",
        "fal.ai의 TRELLIS를 쓰려면 fal.ai 키가 필요해요(Real 패널의 설정).",
    ),
    "recon.fal_model_failed": ("fal.ai: the model failed ({err})", "fal.ai: 모델이 실패했어요({err})"),
    "recon.timeout": (
        "{svc} didn't finish in time. Try again later or use the offline quick shape.",
        "{svc|이} 제시간에 끝내지 못했어요. 나중에 다시 하거나 오프라인 빠른 모양을 쓰세요.",
    ),
    "recon.no_mesh": ("{svc} answered without a mesh.", "{svc}의 응답에 메시가 없었어요."),
    "recon.no_glb": ("{svc} answered without a GLB.", "{svc}의 응답에 GLB가 없었어요."),
    "recon.download_failed": ("couldn't download the mesh from {svc}.", "{svc}에서 메시를 내려받지 못했어요."),
    "recon.meshy_needs_key": (
        "Meshy needs a Meshy API key (Settings in the Real panel).",
        "Meshy를 쓰려면 Meshy API 키가 필요해요(Real 패널의 설정).",
    ),
    "recon.meshy_no_task": ("Meshy didn't start a task.", "Meshy가 작업을 시작하지 않았어요."),

    # ------------------------------------------------------------------ notes on a reconstruction (GET /recon/job/<id>)
    "recon.note_filled_holes": ("filled holes", "구멍을 메웠어요"),
    "recon.note_not_watertight": (
        "not watertight: mass from the convex hull",
        "완전히 닫힌 형태가 아니에요: 질량은 볼록 껍질로 계산했어요",
    ),
    "recon.note_no_table": (
        "no table under it: up is the photo's up",
        "아래에 테이블이 없어요: 사진의 위쪽을 위로 봤어요",
    ),
    "recon.note_metric_depth": ("shape from metric depth", "실측 깊이로 만든 모양"),
    "recon.note_outline_only": ("shape from outline only", "윤곽만으로 만든 모양"),
    "recon.note_outline_only_depth": (
        "shape from outline only (the depth didn't give a usable shape)",
        "윤곽만으로 만든 모양(깊이로는 쓸 만한 모양이 나오지 않았어요)",
    ),
    "recon.note_outline_relative": (
        "shape from the outline, shape class from relative depth (Depth Anything V2 Small)",
        "윤곽으로 만든 모양, 모양 종류는 상대 깊이로 판단(Depth Anything V2 Small)",
    ),
    "recon.note_triposr": ("TripoSR on {device}", "TripoSR ({device}에서 실행)"),
    "recon.depth_download_failed": ("depth model download failed ({e})", "깊이 모델을 내려받지 못했어요({e})"),

    # -------------------------------------------------------- pps/features/dataset.py + pps/data/recorder.py: recording
    "dataset.unknown_op": (
        "Unknown dataset op '{op}'. Use start, stop, discard_last, randomize or meta.",
        "알 수 없는 데이터셋 명령이에요: '{op}'. start, stop, discard_last, randomize, meta 중 하나를 쓰세요.",
    ),
    "dataset.error": ("Dataset error: {e}", "데이터셋 오류: {e}"),
    "dataset.already_recording": ("Already recording '{name}'.", "이미 데이터셋 '{name}'에 녹화하고 있어요."),
    "dataset.recording": ("Recording every task into datasets/{name}.", "모든 작업을 datasets/{name}에 녹화할게요."),
    "dataset.recording_paused": (
        "Recording datasets/{name}, paused (auto off).",
        "datasets/{name}에 녹화 중이지만 일시정지 상태예요(auto 꺼짐).",
    ),
    "dataset.not_recording": ("Not recording.", "녹화하고 있지 않아요."),
    "dataset.saved": ("Saved {n} episode{_s} to datasets/{name}.", "에피소드 {n}개를 datasets/{name}에 저장했어요."),
    "dataset.saved_dropped": (
        "Saved {n} episode{_s} to datasets/{name}. The task still running was not kept.",
        "에피소드 {n}개를 datasets/{name}에 저장했어요. 진행 중이던 작업은 저장하지 않았어요.",
    ),
    "dataset.not_kept": ("The task still running was not kept.", "진행 중이던 작업은 저장하지 않았어요."),
    "dataset.no_discard": ("There is no episode to discard.", "버릴 에피소드가 없어요."),
    "dataset.discarding": ("Discarding the last episode.", "마지막 에피소드를 버릴게요."),
    "dataset.discarded": ("Discarded episode {i} ({task}).", "에피소드 {i|을} 버렸어요({task})."),
    "dataset.meta_json": ("meta must be JSON.", "meta는 JSON이어야 해요."),
    "dataset.meta_large": ("meta is too large (4 kB max).", "meta가 너무 커요(최대 4 kB)."),
    "dataset.meta_set": ("Episode metadata set.", "에피소드 메타데이터를 설정했어요."),
    "dataset.meta_cleared": ("Episode metadata cleared.", "에피소드 메타데이터를 지웠어요."),
    "dataset.rand_off": ("Randomization: off", "랜덤화: 꺼짐"),
    "dataset.rand_both": (
        "Randomization: lighting {lighting}, camera {camera}",
        "랜덤화: 조명 {lighting}, 카메라 {camera}",
    ),
    "dataset.rand_lighting": ("Randomization: lighting {lighting}", "랜덤화: 조명 {lighting}"),
    "dataset.rand_camera": ("Randomization: camera {camera}", "랜덤화: 카메라 {camera}"),

    # --------------------------------------- how an episode ended (meta/episodes.jsonl) and the dataset thread's errors
    "dataset.world_rebuilt": ("The world was rebuilt during the task.", "작업 중에 월드가 다시 만들어졌어요."),
    "dataset.interrupted": ("Another task took over before it finished.", "끝나기 전에 다른 작업이 넘겨받았어요."),
    "dataset.timeout": ("Longer than {s} s.", "{s} s보다 오래 걸렸어요."),
    "dataset.no_ffmpeg": (
        "RuntimeError: ffmpeg not found: install imageio-ffmpeg (scripts/setup.sh) or ffmpeg",
        "RuntimeError: ffmpeg를 찾을 수 없어요: imageio-ffmpeg(scripts/setup.sh)나 ffmpeg를 설치하세요",
    ),
    "dataset.video_failed": (
        "RuntimeError: video encoding failed: {bad}",
        "RuntimeError: 영상 인코딩에 실패했어요: {bad}",
    ),
}
