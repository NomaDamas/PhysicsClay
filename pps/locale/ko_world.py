"""Korean server messages: world (pps/world/*.py: scene build, robot profiles, generated rooms and their fixtures, the
mobile base's drive, the body validator, demo packs; pps/app.py world_target / switch_world / world_message).
English is the message id; see pps/i18n.py."""

MESSAGES = {
    # key: (English template, Korean template)
    # ---------------------------------------------------------------- switching worlds (pps/app.py)
    "world.unknown_robot_list": (
        "I don't know the robot '{robot}'. I know: {robots}.",
        "모르는 로봇이에요: '{robot}'. 제가 아는 로봇: {robots}."),
    "world.add_object_needs_object": (       # {example}: the literal "{kind: 'cup', color: 'red'}"
        "add_object needs an object, e.g. {example}.", "add_object에는 물체가 필요해요. 예: {example}."),
    "world.no_such_object": (
        "There is no '{name}' in the scene. Objects: {objs}.",
        "장면에 그런 물체가 없어요: '{name}'. 있는 물체: {objs}."),
    "world.no_such_object_empty": (
        "There is no '{name}' in the scene. Objects: none.",
        "장면에 그런 물체가 없어요: '{name}'. 장면에 물체가 하나도 없어요."),
    "world.unknown_op": (
        "Unknown world op '{op}'. Use set, add_object, remove_object, pack or reset.",
        "모르는 월드 명령이에요: '{op}'. set, add_object, remove_object, pack, reset 중 하나를 쓰세요."),
    "world.busy_video": (
        "I'm imitating your video right now. Let it finish first.",
        "지금 영상을 따라 하고 있어요. 끝날 때까지 기다려 주세요."),
    "world.build_failed": ("I couldn't build that world: {err}.", "그 월드를 만들지 못했어요: {err}."),
    "world.build_failed_robot": (
        "I couldn't build that world: unknown robot '{robot}'.",
        "그 월드를 만들지 못했어요. 모르는 로봇이에요: '{robot}'."),
    "world.build_failed_template": (
        "I couldn't build that world: unknown world template '{template}'.",
        "그 월드를 만들지 못했어요. 모르는 월드 템플릿이에요: '{template}'."),
    "world.no_pack": (
        "There is no demo pack '{name}'. Packs: {packs}; worlds: {worlds}.",
        "그런 데모 팩은 없어요: '{name}'. 팩: {packs}, 월드: {worlds}."),
    # scene build errors (pps/world/scene.py, rooms.py), also inside "I couldn't build that world: …"
    "world.err_unknown_robot": ("unknown robot '{robot}'", "모르는 로봇 '{robot}'"),
    "world.err_unknown_template": ("unknown world template '{template}'", "모르는 월드 템플릿 '{template}'"),
    "world.err_model_missing": (
        "the {robot} model is not on disk ({path})", "{robot} 모델 파일이 디스크에 없어요 ({path})"),
    "world.err_scene": ("could not build the scene: {err}", "장면을 만들지 못했어요: {err}"),
    # ---------------------------------------------------------------- the switch message (world_message)
    "world.now_at_with": (
        "Now {robot} at the {surface} with {objs}.",
        "이제 {robot|이} {surface} 앞에 있어요. 물체: {objs}."),
    "world.now_at_empty": (
        "Now {robot} at the {surface}, nothing on it yet.",
        "이제 {robot|이} {surface} 앞에 있어요. 아직 아무것도 없어요."),
    "world.now_at_with_experimental": (
        "Now {robot} at the {surface} with {objs}. Experimental: {note}",
        "이제 {robot|이} {surface} 앞에 있어요. 물체: {objs}. 실험 단계: {note}"),
    "world.now_at_empty_experimental": (
        "Now {robot} at the {surface}, nothing on it yet. Experimental: {note}",
        "이제 {robot|이} {surface} 앞에 있어요. 아직 아무것도 없어요. 실험 단계: {note}"),
    "world.experimental": ("Experimental: {note}", "실험 단계: {note}"),
    "world.now_in_room": (
        "Now {robot} in the {room} (seed {seed}): {n} objects, {k} things that open.",
        "이제 {robot|이} {room}에 있어요 (레이아웃 {seed}): 물체 {n}개, 여닫을 수 있는 것 {k}개."),
    "world.now_in_room_parts": (
        "Now {robot} in the {room} (seed {seed}): {n} objects, {k} things that open ({parts}).",
        "이제 {robot|이} {room}에 있어요 (레이아웃 {seed}): 물체 {n}개, 여닫을 수 있는 것 {k}개 ({parts})."),
    "world.now_in_room_belts": (
        "Now {robot} in the {room} (seed {seed}): {n} objects, {k} things that open, {belts}.",
        "이제 {robot|이} {room}에 있어요 (레이아웃 {seed}): 물체 {n}개, 여닫을 수 있는 것 {k}개, {belts}."),
    "world.now_in_room_parts_belts": (
        "Now {robot} in the {room} (seed {seed}): {n} objects, {k} things that open ({parts}), {belts}.",
        "이제 {robot|이} {room}에 있어요 (레이아웃 {seed}): 물체 {n}개, 여닫을 수 있는 것 {k}개 ({parts}), {belts}."),
    "world.now_in_room_belts_experimental": (
        "Now {robot} in the {room} (seed {seed}): {n} objects, {k} things that open, {belts}. Experimental: {note}",
        "이제 {robot|이} {room}에 있어요 (레이아웃 {seed}): 물체 {n}개, 여닫을 수 있는 것 {k}개, {belts}. "
        "실험 단계: {note}"),
    # one-word room names: a placeholder can't look them up
    "world.now_in_laundry": (
        "Now {robot} in the laundry (seed {seed}): {n} objects, {k} things that open ({parts}).",
        "이제 {robot|이} 세탁실에 있어요 (레이아웃 {seed}): 물체 {n}개, 여닫을 수 있는 것 {k}개 ({parts})."),
    "world.now_in_laundry_none": (
        "Now {robot} in the laundry (seed {seed}): {n} objects, {k} things that open.",
        "이제 {robot|이} 세탁실에 있어요 (레이아웃 {seed}): 물체 {n}개, 여닫을 수 있는 것 {k}개."),
    "world.now_in_warehouse": (
        "Now {robot} in the warehouse (seed {seed}): {n} objects, {k} things that open.",
        "이제 {robot|이} 창고에 있어요 (레이아웃 {seed}): 물체 {n}개, 여닫을 수 있는 것 {k}개."),
    # demo packs: "<pack>: <robot>[ with a <tool>], <objects>." (+ the first sentence of the pack's physics note)
    "world.pack_gym": ("Gym: {robot}, {objs}.", "체육관: {robot}, {objs}."),
    "world.pack_gym_physics": (
        "Gym: {robot}, {objs}. The barbell is a 20 kg free body with P0 physics.",
        "체육관: {robot}, {objs}. 바벨은 P0 물리가 적용된 20 kg짜리 자유 물체예요."),
    "world.pack_weld": ("Weld cell: {robot}, {objs}.", "용접 셀: {robot}, {objs}."),
    "world.pack_weld_tool": (
        "Weld cell: {robot} with a {tool}, {objs}.", "용접 셀: {tool|을} 단 {robot}, {objs}."),
    "world.pack_weld_physics": (
        "Weld cell: {robot}, {objs}. No arc or melt physics: the bead is drawn along the torch tip's actual path.",
        "용접 셀: {robot}, {objs}. 아크나 용융 물리는 없어요: 비드는 토치 끝이 실제로 지나간 경로를 따라 그려져요."),
    "world.pack_weld_tool_physics": (
        "Weld cell: {robot} with a {tool}, {objs}. No arc or melt physics: the bead is drawn along the torch tip's "
        "actual path.",
        "용접 셀: {tool|을} 단 {robot}, {objs}. 아크나 용융 물리는 없어요: 비드는 토치 끝이 실제로 지나간 경로를 따라 "
        "그려져요."),
    "world.pack_workshop": ("Workshop helper: {robot}, {objs}.", "작업장 도우미: {robot}, {objs}."),
    "world.pack_workshop_physics": (
        "Workshop helper: {robot}, {objs}. Handing over uses the robot's grasp (contact grasp with P0 physics, a "
        "kinematic grasp otherwise); the person's hand is the camera pose.",
        "작업장 도우미: {robot}, {objs}. 건네주기는 로봇의 집기를 그대로 써요(P0 물리에서는 접촉 집기, 아니면 운동학 "
        "집기). 사람의 손은 카메라로 본 자세예요."),
    # ---------------------------------------------------------------- scene notes (state.world.notes and "(…)")
    "world.note_no_room": ("no room for {name}", "{name}: 놓을 자리가 없어요"),
    "world.note_missing_mesh": (
        "{name} is missing from the object library", "물체 라이브러리에 {name|이} 없어요"),
    "world.note_tool_single_arm": (
        "the {tool} mounts on single arms only, so the {robot} works without it",
        "{tool|은} 팔이 하나인 로봇에만 달 수 있어서, {robot|은} 그것 없이 작업해요"),
    "world.note_tool_failed": ("the tool could not be mounted", "도구를 달지 못했어요"),
    "world.note_moved_paren": ("({name} moved onto the {surface})", "({name|이} {surface} 위로 옮겨졌어요)"),
    "world.note_no_room_paren": ("(no room for {name})", "({name}: 놓을 자리가 없어요)"),
    "world.note_missing_mesh_paren": (
        "({name} is missing from the object library)", "(물체 라이브러리에 {name|이} 없어요)"),
    "world.note_tool_single_arm_paren": (
        "(the {tool} mounts on single arms only, so the {robot} works without it)",
        "({tool|은} 팔이 하나인 로봇에만 달 수 있어서, {robot|은} 그것 없이 작업해요)"),
    "world.note_tool_failed_paren": ("(the tool could not be mounted)", "(도구를 달지 못했어요)"),
    # the same notes after a sentence the engine matched as a head
    "world.note_moved_tail": (" ({name} moved onto the {surface})", " ({name|이} {surface} 위로 옮겨졌어요)"),
    "world.note_no_room_tail": (" (no room for {name})", " ({name}: 놓을 자리가 없어요)"),
    "world.note_tool_single_arm_tail": (
        " (the {tool} mounts on single arms only, so the {robot} works without it)",
        " ({tool|은} 팔이 하나인 로봇에만 달 수 있어서, {robot|은} 그것 없이 작업해요)"),
    "world.imported_object": ("an imported object", "가져온 물체"),
    # ---------------------------------------------------------------- robots (state.world.robot: label, note)
    "world.robot_g1": ("Unitree G1 humanoid", "Unitree G1 휴머노이드"),
    "world.robot_panda": ("Franka Emika Panda arm", "Franka Emika Panda 로봇팔"),
    "world.robot_ur5e": ("Universal Robots UR5e arm", "Universal Robots UR5e 로봇팔"),
    "world.robot_aloha": ("ALOHA two-arm rig", "ALOHA 양팔 로봇"),
    "world.robot_apollo": ("Apptronik Apollo humanoid", "Apptronik Apollo 휴머노이드"),
    "world.robot_tidybot": (
        "Stanford TidyBot (Kinova on a mobile base)", "Stanford TidyBot (모바일 베이스 위의 Kinova)"),
    "world.robot_spot": ("Boston Dynamics Spot with arm", "Boston Dynamics Spot (팔 장착)"),
    "world.robot_h2": ("Unitree H2 humanoid (experimental)", "Unitree H2 휴머노이드 (실험 단계)"),
    "world.note_legged": (
        "Physics: manipulation runs with the base welded in place and the legs held by their servos; dynamic walking "
        "needs a learned locomotion policy and is out of scope.",
        "물리: 조작하는 동안 몸통은 제자리에 고정되고 다리는 서보가 붙잡고 있어요. 동적 보행에는 학습된 보행 정책이 "
        "필요해서 지원 범위 밖이에요."),
    "world.note_apollo_ik": ("Whole-body IK with balance.", "균형을 잡으며 전신 IK로 움직여요."),
    "world.note_apollo_g1_only": (
        "Following you, video imitation and teach-by-showing are tuned for the G1 only.",
        "사람 따라가기, 영상 모방, 시범으로 가르치기는 G1에만 맞춰져 있어요."),
    "world.note_h2": (
        "The model ships without a standing keyframe and its dexterous hands are not driven: it starts from the zero "
        "pose, reaches with whole-body IK and keeps its feet planted, but pick & place is not supported.",
        "이 모델에는 서 있는 자세의 키프레임이 없고 정교한 손도 구동되지 않아요: 영점 자세에서 시작해 전신 IK로 팔을 "
        "뻗고 발은 바닥에 붙이고 있지만, 집어서 놓기는 지원하지 않아요."),
    # ---------------------------------------------------------------- surfaces and worlds (labels, summaries)
    "world.tpl_table": ("Wooden table", "나무 테이블"),
    "world.tpl_kitchen": ("Kitchen counter", "주방 조리대"),
    "world.tpl_shelf": ("Shelf unit (work on the lower board)", "선반 (아래 칸에서 작업)"),
    "world.tpl_floor": ("Floor mat", "바닥 매트"),
    "world.tpl_gym": ("Gym floor", "체육관 바닥"),
    # lower case: the surface inside "Now … at the <surface> …" and the notes
    "world.tpl_kitchen_lc": ("kitchen counter", "주방 조리대"),
    "world.tpl_shelf_lc": ("shelf unit (work on the lower board)", "선반 (아래 칸에서 작업)"),
    "world.tpl_floor_lc": ("floor mat", "바닥 매트"),
    "world.tpl_gym_lc": ("gym floor", "체육관 바닥"),
    "world.room_kitchen_full": ("Full kitchen", "주방"),
    "world.room_dining": ("Dining room", "다이닝룸"),
    "world.room_living": ("Living room", "거실"),
    "world.room_laundry": ("Laundry", "세탁실"),
    "world.room_assembly": ("Assembly line", "조립 라인"),
    "world.room_warehouse": ("Warehouse", "창고"),
    "world.room_machine": ("Machine tending", "머신 텐딩"),
    "world.room_electronics": ("Electronics bench", "전자 작업대"),
    "world.room_lab": ("Lab bench", "실험대"),
    "world.room_packing": ("Packing station", "포장 스테이션"),
    # lower case: the room inside "Now … in the <room> (seed n): …"
    "world.room_kitchen_full_lc": ("full kitchen", "주방"),
    "world.room_dining_lc": ("dining room", "다이닝룸"),
    "world.room_living_lc": ("living room", "거실"),
    "world.room_assembly_lc": ("assembly line", "조립 라인"),
    "world.room_machine_lc": ("machine tending", "머신 텐딩"),
    "world.room_electronics_lc": ("electronics bench", "전자 작업대"),
    "world.room_lab_lc": ("lab bench", "실험대"),
    "world.room_packing_lc": ("packing station", "포장 스테이션"),
    # a generated world's work surface
    "world.surf_dining": ("Dining table", "식탁"),
    "world.surf_coffee": ("Coffee table", "커피 테이블"),
    "world.surf_laundry": ("Laundry counter", "세탁실 작업대"),
    "world.surf_conveyor": ("Conveyor station", "컨베이어 스테이션"),
    "world.surf_pick": ("Pick table", "피킹 테이블"),
    "world.surf_machine": ("Machine-tending station", "머신 텐딩 스테이션"),
    "world.surf_packing": ("Packing table", "포장 테이블"),
    "world.sum_kitchen_full": (
        "Counter run with drawers and doors, a sink, a stove, a fridge, a microwave, wall cabinets and clutter.",
        "서랍과 문이 달린 긴 조리대, 싱크대, 가스레인지, 냉장고, 전자레인지, 벽 수납장, 그리고 어질러진 물건들."),
    "world.sum_dining": (
        "A set table with plates, cutlery and glasses, chairs and a sideboard with drawers.",
        "접시와 수저, 유리컵이 차려진 식탁, 의자, 서랍 달린 찬장."),
    "world.sum_living": (
        "Coffee table, sofa, TV unit with drawers, bookcase and toys on the floor for mobile robots.",
        "커피 테이블, 소파, 서랍 달린 TV 장, 책장, 그리고 모바일 로봇이 치울 바닥의 장난감."),
    "world.sum_laundry": (
        "Washer and dryer with doors under a counter, laundry baskets with towels, detergent on a shelf.",
        "작업대 아래 문 달린 세탁기와 건조기, 수건이 담긴 빨래 바구니, 선반 위의 세제."),
    "world.sum_assembly": (
        "A running belt conveyor that stops parts at the photo-eye, a parts bin and an assembly bench.",
        "광전 센서 앞에서 부품을 멈춰 세우는 벨트 컨베이어, 부품 통, 조립 작업대."),
    "world.sum_warehouse": (
        "Bin picking from a tote at a pick table, racks with bays, a pallet of stacked cases.",
        "피킹 테이블에서 토트 속 물건 집기, 칸이 나뉜 랙, 상자가 쌓인 팔레트."),
    "world.sum_machine": (
        "A CNC enclosure with a sliding door and a part in the vise, raw and finished trays.",
        "미닫이문이 달린 CNC 장비와 바이스에 물린 부품, 소재 트레이와 완성품 트레이."),
    "world.sum_electronics": (
        "An ESD bench with PCBs, a parts tray of screws and nuts, a screwdriver and pliers.",
        "PCB가 놓인 정전기 방지 작업대, 나사와 너트가 든 부품 트레이, 드라이버와 펜치."),
    "world.sum_lab": (
        "Test-tube racks with tubes, a centrifuge with a hinged lid, a pipette and sample plates.",
        "튜브가 꽂힌 시험관 랙, 경첩 뚜껑이 달린 원심분리기, 피펫과 샘플 플레이트."),
    "world.sum_packing": (
        "An open carton, a tote of products, a scale and a roller conveyor bringing boxes.",
        "열린 상자, 상품이 담긴 토트, 저울, 상자를 실어 오는 롤러 컨베이어."),
    # ---------------------------------------------------------------- fixtures: what opens, holds or carries things
    # (shown as labels and inside other messages: "Opening the top drawer.")
    "world.fx_top_drawer": ("the top drawer", "맨 위 서랍"),
    "world.fx_second_drawer": ("the second drawer", "두 번째 서랍"),
    "world.fx_bottom_drawer": ("the bottom drawer", "맨 아래 서랍"),
    "world.fx_stove_drawer": ("the drawer under the stove", "가스레인지 아래 서랍"),
    "world.fx_stove_deep_drawer": ("the deep drawer under the stove", "가스레인지 아래 깊은 서랍"),
    "world.fx_cabinet_door_front": ("the cabinet door in front of me", "제 앞의 수납장 문"),
    "world.fx_sink_cabinet_door": ("a sink cabinet door", "싱크대 수납장 문"),
    "world.fx_sideboard_top": ("the top drawer of the sideboard", "찬장 맨 위 서랍"),
    "world.fx_sideboard_second": ("the second drawer of the sideboard", "찬장 두 번째 서랍"),
    "world.fx_door_left": ("door left", "왼쪽 문"),
    "world.fx_door_right": ("door right", "오른쪽 문"),
    "world.fx_wall_cabinet_left": ("left door of the wall cabinet", "벽 수납장 왼쪽 문"),
    "world.fx_wall_cabinet_right": ("right door of the wall cabinet", "벽 수납장 오른쪽 문"),
    "world.fx_wall_cabinet_door": ("wall cabinet door", "벽 수납장 문"),
    "world.fx_wall_cabinet_shelf": ("the wall cabinet shelf", "벽 수납장 선반"),
    "world.fx_inside": ("inside the {fixture}", "{fixture} 안"),
    "world.fx_inside_wall_cabinet": ("inside the wall cabinet", "벽 수납장 안"),
    "world.fx_inside_sink_cabinet": ("inside the sink cabinet", "싱크대 수납장 안"),
    "world.fx_inside_center_cabinet": ("inside the center cabinet", "가운데 수납장 안"),
    "world.fx_inside_sideboard": ("inside the sideboard", "찬장 안"),
    "world.fx_fridge": ("the fridge", "냉장고"),
    "world.fx_fridge_door": ("the fridge door", "냉장고 문"),
    "world.fx_fridge_shelf": ("the fridge shelf", "냉장고 선반"),
    "world.fx_microwave": ("the microwave", "전자레인지"),
    "world.fx_microwave_door": ("the microwave door", "전자레인지 문"),
    "world.fx_dishwasher_door": ("the dishwasher door", "식기세척기 문"),
    "world.fx_dishwasher_rack": ("the dishwasher rack", "식기세척기 랙"),
    "world.fx_sink": ("the sink", "싱크대"),
    "world.fx_stove": ("the stove", "가스레인지"),
    "world.fx_sofa": ("the sofa", "소파"),
    "world.fx_tv_unit": ("the TV unit", "TV 장"),
    "world.fx_tv_drawer_left": ("the left drawer of the TV unit", "TV 장 왼쪽 서랍"),
    "world.fx_tv_drawer_right": ("the right drawer of the TV unit", "TV 장 오른쪽 서랍"),
    "world.fx_washer": ("the washer", "세탁기"),
    "world.fx_washer_door": ("the washer door", "세탁기 문"),
    "world.fx_dryer": ("the dryer", "건조기"),
    "world.fx_dryer_door": ("the dryer door", "건조기 문"),
    "world.fx_laundry_basket": ("the laundry basket", "빨래 바구니"),
    "world.fx_floor_basket": ("the basket on the floor", "바닥에 있는 바구니"),
    "world.fx_conveyor": ("the conveyor", "컨베이어"),
    "world.fx_finished_bin": ("the finished-parts bin", "완성 부품 통"),
    "world.fx_assembly_bench": ("the assembly bench", "조립 작업대"),
    "world.fx_tote": ("the tote", "토트"),
    "world.fx_empty_tote": ("the empty tote", "빈 토트"),
    "world.fx_pallet": ("the pallet", "팔레트"),
    "world.fx_raw_tray": ("the raw blanks tray", "소재 트레이"),
    "world.fx_done_tray": ("the finished parts tray", "완성 부품 트레이"),
    "world.fx_vise": ("the vise", "바이스"),
    "world.fx_cnc_door": ("the CNC door", "CNC 문"),
    "world.fx_tube_rack": ("the tube rack", "튜브 랙"),
    "world.fx_source_rack": ("the yellow source rack", "노란 공급 랙"),
    "world.fx_centrifuge_lid": ("the centrifuge lid", "원심분리기 뚜껑"),
    "world.fx_open_carton": ("the open carton", "열린 상자"),
    "world.fx_product_tote": ("the product tote", "상품 토트"),
    "world.fx_infeed": ("the infeed rollers", "투입 롤러"),
    "world.fx_scale": ("the scale", "저울"),
    "world.fx_shelf_bottom": ("the bottom shelf", "맨 아래 칸"),
    "world.fx_shelf_second": ("the second shelf", "두 번째 칸"),
    "world.fx_shelf_third": ("the third shelf", "세 번째 칸"),
    "world.fx_shelf_fourth": ("the fourth shelf", "네 번째 칸"),
    "world.fx_shelf_bottom_of": ("the bottom shelf of the {shelf}", "{shelf}의 맨 아래 칸"),
    "world.fx_shelf_second_of": ("the second shelf of the {shelf}", "{shelf}의 두 번째 칸"),
    "world.fx_shelf_third_of": ("the third shelf of the {shelf}", "{shelf}의 세 번째 칸"),
    "world.fx_shelf_fourth_of": ("the fourth shelf of the {shelf}", "{shelf}의 네 번째 칸"),
    "world.fx_shelf_fifth_of": ("the fifth shelf of the {shelf}", "{shelf}의 다섯 번째 칸"),
    "world.fx_bookcase_bottom": ("the bottom shelf of the bookcase", "책장의 맨 아래 칸"),
    "world.fx_bookcase_second": ("the second shelf of the bookcase", "책장의 두 번째 칸"),
    "world.fx_bookcase_third": ("the third shelf of the bookcase", "책장의 세 번째 칸"),
    "world.fx_bookcase_fourth": ("the fourth shelf of the bookcase", "책장의 네 번째 칸"),
    "world.fx_open_shelf": ("open shelf", "오픈 선반"),
    "world.fx_parts_shelf": ("parts shelf", "부품 선반"),
    "world.fx_reagent_shelf": ("reagent shelf", "시약 선반"),
    "world.fx_wall_shelf": ("wall shelf", "벽 선반"),
    "world.fx_rack_bay": ("level {level}, bay {bay} of the {rack}", "{rack}의 {level}단 {bay}번 칸"),
    "world.fx_slot": ("slot {k} of {rack}", "{rack}의 {k}번 슬롯"),
    "world.fx_compartment": ("compartment {k} of {tray}", "{tray}의 {k}번 칸"),
    "world.fx_compartment_bench_tray": ("compartment {k} of the bench tray", "작업대 트레이의 {k}번 칸"),
    "world.fx_compartment_tray": ("compartment {k} of the tray", "트레이의 {k}번 칸"),
    # ---------------------------------------------------------------- driving the base (pps/world/runtime.py)
    "world.no_base": ("I have no base to drive.", "저는 움직일 수 있는 베이스가 없어요."),
    "world.base_along_table": (
        "My base only drives along the table, not towards it.",
        "제 베이스는 테이블을 따라 옆으로만 움직이고, 테이블 쪽으로는 못 가요."),
    "world.base_stop_before_table": ("I stop before the table.", "테이블 앞에서 멈출게요."),
    "world.base_stay_along_table": ("I stay along the table.", "테이블 옆 범위 안에서만 움직일게요."),
    "world.base_cant_move": ("My base can't move that way.", "제 베이스는 그쪽으로 못 움직여요."),
    "world.base_cant_move_axis": (
        "My base can't move that way (it only drives along its own axis).",
        "제 베이스는 그쪽으로 못 움직여요 (자기 축 방향으로만 움직여요)."),
    "world.base_axis_left_out": (
        "My base only drives along its own axis, so {cm} cm of that is left out.",
        "제 베이스는 자기 축 방향으로만 움직여서, 그중 {cm} cm는 빼고 움직여요."),
    "world.drive_run_fwd": ("Driving {cm} cm forward.", "앞으로 {cm} cm 이동할게요."),
    "world.drive_run_back": ("Driving {cm} cm back.", "뒤로 {cm} cm 이동할게요."),
    "world.drive_run_left": ("Driving {cm} cm to the left.", "왼쪽으로 {cm} cm 이동할게요."),
    "world.drive_run_right": ("Driving {cm} cm to the right.", "오른쪽으로 {cm} cm 이동할게요."),
    "world.turn_run_left": ("Turning {deg}° left.", "왼쪽으로 {deg}° 돌게요."),
    "world.turn_run_right": ("Turning {deg}° right.", "오른쪽으로 {deg}° 돌게요."),
    "world.drive_run_fwd_turn_left": (
        "Driving {cm} cm forward and turning {deg}° left.", "앞으로 {cm} cm 이동하고 왼쪽으로 {deg}° 돌게요."),
    "world.drive_run_fwd_turn_right": (
        "Driving {cm} cm forward and turning {deg}° right.", "앞으로 {cm} cm 이동하고 오른쪽으로 {deg}° 돌게요."),
    "world.drive_run_back_turn_left": (
        "Driving {cm} cm back and turning {deg}° left.", "뒤로 {cm} cm 이동하고 왼쪽으로 {deg}° 돌게요."),
    "world.drive_run_back_turn_right": (
        "Driving {cm} cm back and turning {deg}° right.", "뒤로 {cm} cm 이동하고 오른쪽으로 {deg}° 돌게요."),
    "world.drive_run_left_turn_left": (
        "Driving {cm} cm to the left and turning {deg}° left.", "왼쪽으로 {cm} cm 이동하고 왼쪽으로 {deg}° 돌게요."),
    "world.drive_run_left_turn_right": (
        "Driving {cm} cm to the left and turning {deg}° right.", "왼쪽으로 {cm} cm 이동하고 오른쪽으로 {deg}° 돌게요."),
    "world.drive_run_right_turn_left": (
        "Driving {cm} cm to the right and turning {deg}° left.", "오른쪽으로 {cm} cm 이동하고 왼쪽으로 {deg}° 돌게요."),
    "world.drive_run_right_turn_right": (
        "Driving {cm} cm to the right and turning {deg}° right.",
        "오른쪽으로 {cm} cm 이동하고 오른쪽으로 {deg}° 돌게요."),
    "world.drive_done_fwd": ("Drove {cm} cm forward.", "앞으로 {cm} cm 이동했어요."),
    "world.drive_done_back": ("Drove {cm} cm back.", "뒤로 {cm} cm 이동했어요."),
    "world.drive_done_left": ("Drove {cm} cm to the left.", "왼쪽으로 {cm} cm 이동했어요."),
    "world.drive_done_right": ("Drove {cm} cm to the right.", "오른쪽으로 {cm} cm 이동했어요."),
    "world.turn_done_left": ("Turned {deg}° left.", "왼쪽으로 {deg}° 돌았어요."),
    "world.turn_done_right": ("Turned {deg}° right.", "오른쪽으로 {deg}° 돌았어요."),
    "world.drive_done_fwd_turn_left": (
        "Drove {cm} cm forward and turned {deg}° left.", "앞으로 {cm} cm 이동하고 왼쪽으로 {deg}° 돌았어요."),
    "world.drive_done_fwd_turn_right": (
        "Drove {cm} cm forward and turned {deg}° right.", "앞으로 {cm} cm 이동하고 오른쪽으로 {deg}° 돌았어요."),
    "world.drive_done_back_turn_left": (
        "Drove {cm} cm back and turned {deg}° left.", "뒤로 {cm} cm 이동하고 왼쪽으로 {deg}° 돌았어요."),
    "world.drive_done_back_turn_right": (
        "Drove {cm} cm back and turned {deg}° right.", "뒤로 {cm} cm 이동하고 오른쪽으로 {deg}° 돌았어요."),
    "world.drive_done_left_turn_left": (
        "Drove {cm} cm to the left and turned {deg}° left.", "왼쪽으로 {cm} cm 이동하고 왼쪽으로 {deg}° 돌았어요."),
    "world.drive_done_left_turn_right": (
        "Drove {cm} cm to the left and turned {deg}° right.", "왼쪽으로 {cm} cm 이동하고 오른쪽으로 {deg}° 돌았어요."),
    "world.drive_done_right_turn_left": (
        "Drove {cm} cm to the right and turned {deg}° left.", "오른쪽으로 {cm} cm 이동하고 왼쪽으로 {deg}° 돌았어요."),
    "world.drive_done_right_turn_right": (
        "Drove {cm} cm to the right and turned {deg}° right.",
        "오른쪽으로 {cm} cm 이동하고 오른쪽으로 {deg}° 돌았어요."),
    # ---------------------------------------------------------------- feasibility checks (pps/world/validate.py;
    # the G1's pps/validate.py writes the same lines)
    "world.unreachable_left": ("Left hand unreachable by {cm} cm", "왼손이 {cm} cm 모자라서 닿지 않아요"),
    "world.unreachable_right": ("Right hand unreachable by {cm} cm", "오른손이 {cm} cm 모자라서 닿지 않아요"),
    "world.unreachable_gripper": ("The gripper unreachable by {cm} cm", "그리퍼가 {cm} cm 모자라서 닿지 않아요"),
    "world.joint_limit": ("{joint} at joint limit", "{joint}: 관절이 한계에 닿았어요"),
    "world.arm_hits_table": ("The arm would hit the table", "팔이 테이블에 부딪힐 거예요"),
    "world.too_fast": ("Too fast for joints: replay at {x}x", "관절이 따라가기엔 너무 빨라요: {x}배속으로 재생해요"),
    "world.tip_no_foot": ("Would tip over: no foot is on the ground", "넘어질 거예요: 바닥에 닿은 발이 없어요"),
    "world.tip_com_feet": (
        "Would tip over: centre of mass {cm} cm outside the feet",
        "넘어질 거예요: 무게 중심이 발 밖으로 {cm} cm 벗어났어요"),
    "world.tip_both_feet_off": (
        "Would tip over: both feet are off the ground", "넘어질 거예요: 두 발이 모두 바닥에서 떨어졌어요"),
    "world.tip_com_standing_foot": (
        "Would tip over: centre of mass {cm} cm outside the standing foot",
        "넘어질 거예요: 무게 중심이 디딘 발 밖으로 {cm} cm 벗어났어요"),
    # ---------------------------------------------------------------- demo packs (pps/world/packs/*.json)
    "world.tool_torch": ("welding torch", "용접 토치"),
    "world.seam_straight": ("the straight seam", "직선 용접선"),
    "world.seam_l": ("the L seam", "L자 용접선"),
    "world.gym_physics": (
        "The barbell is a 20 kg free body with P0 physics.", "바벨은 P0 물리가 적용된 20 kg짜리 자유 물체예요."),
    "world.weld_physics": (
        "No arc or melt physics: the bead is drawn along the torch tip's actual path.",
        "아크나 용융 물리는 없어요: 비드는 토치 끝이 실제로 지나간 경로를 따라 그려져요."),
    "world.workshop_physics": (
        "Handing over uses the robot's grasp (contact grasp with P0 physics, a kinematic grasp otherwise); the "
        "person's hand is the camera pose.",
        "건네주기는 로봇의 집기를 그대로 써요(P0 물리에서는 접촉 집기, 아니면 운동학 집기). 사람의 손은 카메라로 본 "
        "자세예요."),
}
