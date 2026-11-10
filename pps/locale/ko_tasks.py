"""Korean server messages: tasks (pps/tasks.py, pps/manip.py, pps/physics.py, pps/robot_runtime.py, pps/traj.py).
English is the message id; see pps/i18n.py.

Object names reach these messages through Tasks.pretty ("the mug", "the red block"), so object placeholders sit after a
literal "the " ("I'm holding the {obj}."): the template stays specific and the captured name has no article. Hands and
arms come from Tasks.limb ("right hand", "left arm" on two-armed bodies, "gripper" / "arm" on single arms) and the
glossary turns them into 오른손 / 왼팔 / 그리퍼 / 팔. Messages joined from pieces at run time (a verdict plus a hint, a done
message plus how far off it landed) have an entry per sentence, or one for the whole message where a piece isn't a
sentence of its own. pps/physics.py writes no user-facing text. The generic trajectory-check sentences ("{why} at point
{i}.") and a few lines shared with the features live in ko_features."""

MESSAGES = {
    # key: (English template, Korean template)

    # ------------------------------------------------------------------ places (Tasks.resolve labels, used inside messages)
    # ("marker {m}" and "the end of your route" are in ko_features / ko_app)
    "tasks.label_pointed": ("the spot you pointed at ({id})", "가리킨 자리({id})"),
    "tasks.label_marked": ("the spot you marked ({id})", "표시한 자리({id})"),
    "tasks.label_region": ("the {region} of the {surface}", "{surface} {region}"),
    "tasks.label_next_to": ("next to the {obj}", "{obj} 옆"),
    "tasks.label_beside": (
        "{label} (beside the {blocker}, which is there)",
        "{label} 근처 빈자리({blocker|이} 거기 있어서)",
    ),
    # a pick & place detouring around a keep-out zone: the place gets ", going around keep-out zone A" (pps/features/
    # zones.py). Each starts with the place's own lowercase words, so it can't swallow a whole sentence.
    "tasks.via_around_marker": ("marker {mid}, going around {zones}", "{zones|을} 돌아 마커 {mid}"),
    "tasks.via_over_marker": ("marker {mid}, going over {zones}", "{zones} 위로 넘어 마커 {mid}"),
    "tasks.via_around_region": (
        "the {region} of the {surface}, going around {zones}",
        "{zones|을} 돌아 {surface} {region}",
    ),
    "tasks.via_over_region": ("the {region} of the {surface}, going over {zones}", "{zones} 위로 넘어 {surface} {region}"),
    "tasks.via_around_pointed": (
        "the spot you pointed at ({id}), going around {zones}",
        "{zones|을} 돌아 가리킨 자리({id})",
    ),
    "tasks.via_over_pointed": (
        "the spot you pointed at ({id}), going over {zones}",
        "{zones} 위로 넘어 가리킨 자리({id})",
    ),
    "tasks.via_around_next_to": ("next to the {obj}, going around {zones}", "{zones|을} 돌아 {obj} 옆"),
    "tasks.via_over_next_to": ("next to the {obj}, going over {zones}", "{zones} 위로 넘어 {obj} 옆"),
    "tasks.via_around_obj": ("the {obj}, going around {zones}", "{zones|을} 돌아 {obj}"),
    "tasks.via_over_obj": ("the {obj}, going over {zones}", "{zones} 위로 넘어 {obj}"),

    # ------------------------------------------------------------------ where? (Tasks.resolve: why a target can't be used)
    "tasks.spot_not_visible": ("That spot is not on anything I can see.", "그 자리에는 제가 볼 수 있는 게 없어요."),
    "tasks.point_first": (
        "Point at the spot first (or tap it on my view), then say 'there'.",
        "먼저 그 자리를 가리키거나 제 화면에서 탭한 다음 'there'(또는 '저기')라고 말하세요.",
    ),
    "tasks.unknown_where": ("I don't know where '{target}' is.", "'{target}' 위치를 모르겠어요."),
    "tasks.unknown_where_hint": (
        "Tap it on my view or use left, right, center, front or back.",
        "제 화면에서 탭하거나 왼쪽, 오른쪽, 가운데, 앞, 뒤 중 하나로 말해 주세요.",
    ),
    "tasks.no_target": ("No target given.", "어디로 할지 알려 주세요."),

    # ------------------------------------------------------------------ the reach check before a task moves (_launch)
    "tasks.checking_reach": ("Checking reach for every waypoint.", "모든 경유점에 팔이 닿는지 확인하고 있어요."),
    "tasks.all_reachable": ("Every waypoint is reachable.", "모든 경유점에 팔이 닿아요."),
    "tasks.carry_hits": (
        "I can't carry it that way: the {obj} would hit the {obstacle} ({cm} cm too low).",
        "그렇게는 옮길 수 없어요: {obj|이} {obstacle}에 부딪혀요 ({cm} cm 너무 낮아요).",
    ),
    "tasks.exact_grasp": (
        "I can't: my {arm} can't get my {hand} exactly to the grasp ({cm} cm off: it is at the edge of my reach or "
        "something is in the way).",
        "안 돼요: {arm|이} {hand|을} 집을 위치에 정확히 가져가지 못해요 ({cm} cm 어긋나요: 팔이 닿는 범위의 끝이거나 "
        "무언가가 가로막고 있어요).",
    ),
    "tasks.exact_spot": (
        "I can't: my {arm} can't get my {hand} exactly to the spot ({cm} cm off: it is at the edge of my reach or "
        "something is in the way).",
        "안 돼요: {arm|이} {hand|을} 놓을 자리에 정확히 가져가지 못해요 ({cm} cm 어긋나요: 팔이 닿는 범위의 끝이거나 "
        "무언가가 가로막고 있어요).",
    ),
    "tasks.out_of_reach": (
        "I can't: {bad} of {n} waypoints are out of my {arm}'s reach, worst by {cm} cm.",
        "안 돼요: 경유점 {n}개 중 {bad}개가 {arm|이} 닿는 범위 밖이에요. 가장 먼 곳은 {cm} cm 벗어나요.",
    ),
    "tasks.out_of_reach_fixture": (
        "I can't: {bad} of {n} waypoints are out of my {arm}'s reach, worst by {cm} cm ({part} is out of reach for "
        "this robot).",
        "안 돼요: 경유점 {n}개 중 {bad}개가 {arm|이} 닿는 범위 밖이에요. 가장 먼 곳은 {cm} cm 벗어나요 ({part|은} 이 "
        "로봇의 팔이 닿지 않는 곳에 있어요).",
    ),
    "tasks.zone_hand": ("I can't: my {hand} would enter {zone}.", "안 돼요: {hand|이} {zone}에 들어가게 돼요."),
    "tasks.zone_carry": (
        "I can't carry it that way: the {obj} would pass through {zone}.",
        "그렇게는 옮길 수 없어요: {obj|이} {zone|을} 지나가게 돼요.",
    ),
    "tasks.zone_draw_around": (
        "Draw a route around it in 3D, or remove the zone.",
        "3D에서 구역을 돌아가는 경로를 그리거나 구역을 지워 주세요.",
    ),
    "tasks.suggest_closest": (
        "The closest spot I can reach is marker {mid}, {cm} cm nearer.",
        "제가 닿을 수 있는 가장 가까운 자리는 {cm} cm 더 가까운 마커 {mid|이에요}.",
    ),
    "tasks.suggest_say": ("Say 'put it on {mid}'.", "'put it on {mid}'(또는 '{mid}에 놔')라고 말하세요."),

    # ------------------------------------------------------------------ pick & place: refusals before planning
    "tasks.no_pick_body": ("I can't pick things up with this body.", "이 몸으로는 물건을 집을 수 없어요."),
    "tasks.fixed_can_move": (
        "The {fixed} is fixed in this scene; I can move {list}.",
        "{fixed|은} 이 장면에 고정돼 있어요. {list|을} 옮길 수 있어요.",
    ),
    "tasks.fixed_can_move_nothing": (
        "The {fixed} is fixed in this scene; I can move nothing here.",
        "{fixed|은} 이 장면에 고정돼 있어요. 여기엔 제가 옮길 수 있는 물건이 없어요.",
    ),
    "tasks.dont_see_can_move": (
        "I don't see '{obj}'. I can move {list}.",
        "{obj|이} 보이지 않아요. {list|을} 옮길 수 있어요.",
    ),
    "tasks.dont_see_can_move_nothing": (
        "I don't see '{obj}'. I can move nothing here.",
        "{obj|이} 보이지 않아요. 여기엔 제가 옮길 수 있는 물건이 없어요.",
    ),
    "tasks.still_holding": ("I'm still holding the {obj}.", "아직 {obj|을} 들고 있어요."),
    "tasks.no_route": ("There is no route yet.", "아직 경로가 없어요."),
    "tasks.not_on_surface": (
        "I can't place it there: {label} is not on the {surface}.",
        "거기에는 놓을 수 없어요: {label|은} {surface} 위가 아니에요.",
    ),
    "tasks.spot_taken": (
        "{label} is taken by the {blocker} and there is no free spot nearby.",
        "{label}에는 {blocker|이} 있고 근처에 빈자리가 없어요.",
    ),
    "tasks.spot_taken_marker": (
        "Marker {mid} is taken by the {blocker} and there is no free spot nearby.",
        "마커 {mid}에는 {blocker|이} 있고 근처에 빈자리가 없어요.",
    ),
    "tasks.spot_taken_region": (
        "The {region} of the {surface} is taken by the {blocker} and there is no free spot nearby.",
        "{surface} {region}에는 {blocker|이} 있고 근처에 빈자리가 없어요.",
    ),
    "tasks.spot_taken_pointed": (
        "The spot you pointed at ({id}) is taken by the {blocker} and there is no free spot nearby.",
        "가리킨 자리({id})에는 {blocker|이} 있고 근처에 빈자리가 없어요.",
    ),
    "tasks.spot_taken_marked": (
        "The spot you marked ({id}) is taken by the {blocker} and there is no free spot nearby.",
        "표시한 자리({id})에는 {blocker|이} 있고 근처에 빈자리가 없어요.",
    ),
    "tasks.working_out": ("Working out how to pick up the {obj}.", "{obj|을} 어떻게 집을지 궁리하고 있어요."),
    "tasks.no_gripper": (
        "I have no gripper on my {hand}: I can't pick up the {obj}.",
        "{hand}에 그리퍼가 없어서 {obj|을} 집을 수 없어요.",
    ),
    "tasks.too_wide": (
        "The {obj} is {w} cm wide and my gripper opens {open} cm, so I can't get a grip on it.",
        "{obj|은} 폭이 {w} cm인데 제 그리퍼는 {open} cm까지만 벌어져서 잡을 수 없어요.",
    ),

    # ------------------------------------------------------------------ pick & place: the plan, and what happened
    "tasks.picking_placing": (
        "Picking up the {obj} with my {hand} and placing it on {label}.",
        "{hand|으로} {obj|을} 집어서 {label}에 놓을게요.",
    ),
    "tasks.putting_on": ("Putting the {obj} on {label} with my {hand}.", "{hand|으로} {obj|을} {label}에 놓을게요."),
    "tasks.putting_in": ("Putting the {obj} in {label} with my {hand}.", "{hand|으로} {obj|을} {label}에 넣을게요."),
    "tasks.taking": ("Taking the {obj} with my {hand}.", "{hand|으로} {obj|을} 집을게요."),
    "tasks.taking_from": (
        "Taking the {obj} from {label} with my {hand}.",
        "{hand|으로} {label}에서 {obj|을} 꺼낼게요.",
    ),
    "tasks.placed_on": ("Placed the {obj} on {label}.", "{obj|을} {label}에 놓았어요."),
    "tasks.placed_on_err": (
        "Placed the {obj} on {label}, {cm} cm from the spot.",
        "{obj|을} {label}에 놓았어요 (오차 {cm} cm).",
    ),
    "tasks.placed_on_around_err": (
        "Placed the {obj} on {label}, going around {zones}, {cm} cm from the spot.",
        "{obj|을} {zones|을} 돌아 {label}에 놓았어요 (오차 {cm} cm).",
    ),
    "tasks.placed_on_over_err": (
        "Placed the {obj} on {label}, going over {zones}, {cm} cm from the spot.",
        "{obj|을} {zones} 위로 넘어 {label}에 놓았어요 (오차 {cm} cm).",
    ),
    "tasks.from_spot_tail":(", {cm} cm from the spot.", " (오차 {cm} cm)."),
    "tasks.put_in": ("Put the {obj} in {label}.", "{obj|을} {label}에 넣었어요."),
    "tasks.holding": ("I'm holding the {obj}.", "{obj|을} 들고 있어요."),
    "tasks.holding_taken": ("I'm holding the {obj}, taken from {label}.", "{label}에서 꺼낸 {obj|을} 들고 있어요."),
    "tasks.placed_not_in": (
        "I let go of the {obj} but it didn't end up in {label}.",
        "{obj|을} 놓았는데 {label} 안에 들어가지 않았어요.",
    ),
    "tasks.placed_fell": ("I let go of the {obj} but it fell over ({tilt}°).", "{obj|을} 놓았는데 넘어졌어요 ({tilt}°)."),
    "tasks.placed_off": (
        "I let go of the {obj} but it ended up {cm} cm from the spot.",
        "{obj|을} 놓았는데 목표 자리에서 {cm} cm 떨어진 곳에 놓였어요.",
    ),

    # ------------------------------------------------------------------ physics: grip, slip, weight
    "tasks.slip_fell": ("It slipped: the {obj} fell out of my {hand}.", "미끄러졌어요: {obj|이} {hand}에서 빠졌어요."),
    "tasks.slip_force": (
        "It slipped: the {obj} didn't come up with my {hand} (grip force {f} N).",
        "미끄러졌어요: {obj|이} {hand}에 들려 올라오지 않았어요 (잡는 힘 {f} N).",
    ),
    "tasks.slip_no_grip": (
        "It slipped: the {obj} didn't come up with my {hand} (I couldn't get a firm grip).",
        "미끄러졌어요: {obj|이} {hand}에 들려 올라오지 않았어요 (단단히 잡지 못했어요).",
    ),
    "tasks.grip_force_tail": (" (grip force {f} N).", " (잡는 힘 {f} N)."),
    "tasks.too_heavy": (
        "Too heavy: the {obj} weighs about {kg} kg and I can lift {pay} kg, so I let it back down.",
        "너무 무거워요: {obj|은} 약 {kg} kg인데 저는 {pay} kg까지 들 수 있어서 다시 내려놓았어요.",
    ),
    "tasks.arm_couldnt_follow": (
        "My arm couldn't follow that path with the {obj} in hand: it stopped {cm} cm short of where I'd put it down, so "
        "I'm still holding it.",
        "{obj|을} 든 채로는 팔이 그 경로를 따라가지 못했어요: 내려놓을 곳보다 {cm} cm 못 미쳐 멈춰서 아직 들고 있어요.",
    ),

    # ------------------------------------------------------------------ press (compliance)
    "tasks.pressing": ("Pressing {label} with {f} N for {s} s.", "{label|을} {f} N으로 {s} s 동안 누를게요."),
    "tasks.pressed": ("Pressed {label}.", "{label|을} 눌렀어요."),
    "tasks.pressed_force": (
        "Pressed {label} with {f} N for {s} s (peak {peak} N).",
        "{label|을} {f} N으로 {s} s 동안 눌렀어요 (최대 {peak} N).",
    ),
    "tasks.pressed_capped": (
        "Pressed {label} with {f} N for {s} s (peak {peak} N, capped by the {lim} N impedance limit).",
        "{label|을} {f} N으로 {s} s 동안 눌렀어요 (최대 {peak} N, 임피던스 한계 {lim} N으로 제한됨).",
    ),
    "tasks.press_no_contact": (
        "I couldn't press {label}: no firm contact after 8 s (peak {peak} N).",
        "{label|을} 누르지 못했어요: 8 s가 지나도 단단히 닿지 않았어요 (최대 {peak} N).",
    ),
    "tasks.press_needs_physics": (
        "Pressing needs contact forces: physics is off (kinematic mode).",
        "누르려면 접촉력이 필요한데 물리가 꺼져 있어요 (운동학 모드).",
    ),
    "tasks.press_say_how": (
        "Say how hard (force in N) and how long (hold_s).",
        "얼마나 세게(힘, N), 얼마나 오래(hold_s) 누를지 말해 주세요.",
    ),

    # ------------------------------------------------------------------ two arms: handover
    "tasks.one_arm": (
        "I have one arm: I can't pass things from one hand to the other.",
        "팔이 하나라서 한 손에서 다른 손으로 건넬 수 없어요.",
    ),
    "tasks.one_arm_hint": ("Set it down and pick it up again.", "내려놓았다가 다시 집으라고 말해 주세요."),
    "tasks.dont_see_can_pass": (
        "I don't see '{obj}'. I can pass {list}.",
        "{obj|이} 보이지 않아요. {list|을} 다른 손으로 건넬 수 있어요.",
    ),
    "tasks.dont_see_can_pass_nothing": (
        "I don't see '{obj}'. I can pass nothing here.",
        "{obj|이} 보이지 않아요. 여기엔 제가 건넬 수 있는 물건이 없어요.",
    ),
    "tasks.already_in_hand": ("The {obj} is already in my {hand}.", "{obj|은} 이미 {hand}에 있어요."),
    "tasks.grippers_too_narrow": (
        "The {obj} is {w} cm wide and my grippers open {open} cm.",
        "{obj|은} 폭이 {w} cm인데 제 그리퍼는 {open} cm까지만 벌어져요.",
    ),
    "tasks.too_short_handover": (
        "The {obj} is {len} cm long: too short for both my grippers at once (they need about {need} cm).",
        "{obj|은} 길이가 {len} cm라서 두 그리퍼로 동시에 잡기엔 너무 짧아요 (약 {need} cm가 필요해요).",
    ),
    "tasks.handover_instead": (
        "I can set it down in the middle and pick it up with my other hand instead.",
        "대신 가운데에 내려놓고 다른 손으로 다시 집을 수 있어요.",
    ),
    "tasks.handover_present": (
        "Passing the {obj} from my {giver} to my {receiver}: picking it up and holding it out in the middle.",
        "{obj|을} {giver}에서 {receiver|으로} 건넬게요: 먼저 집어서 가운데로 내밀게요.",
    ),
    "tasks.holding_out": ("Holding the {obj} out in the middle.", "{obj|을} 가운데로 내밀고 있어요."),
    "tasks.lost_on_way": (
        "I lost the {obj} on the way: it isn't in my {giver} any more.",
        "가는 도중에 {obj|을} 놓쳤어요: 이제 {giver}에 없어요.",
    ),
    "tasks.takes_free_end": (
        "My {receiver} takes the free end of the {obj}.",
        "{receiver|으로} {obj}의 비어 있는 끝을 잡을게요.",
    ),
    "tasks.took_with": ("Took the {obj} with my {receiver}.", "{receiver|으로} {obj|을} 받았어요."),
    "tasks.receiver_no_grip": (
        "My {receiver} couldn't get a firm grip on the {obj}, so my {giver} keeps it.",
        "{receiver|이} {obj|을} 단단히 잡지 못해서 {giver|이} 계속 들고 있어요.",
    ),
    "tasks.has_lets_go": (
        "My {receiver} has the {obj}; my {giver} lets go.",
        "{receiver|이} {obj|을} 잡았어요. 이제 {giver|은} 놓을게요.",
    ),
    "tasks.passed": (
        "Passed the {obj} from my {giver} to my {receiver}.",
        "{obj|을} {giver}에서 {receiver|으로} 넘겼어요.",
    ),
    "tasks.passed_in_hand": (
        "Passed the {obj} from my {giver} to my {receiver}. It's in my {receiver}.",
        "{obj|을} {giver}에서 {receiver|으로} 넘겼어요. 지금 {receiver}에 있어요.",
    ),
    "tasks.passed_placed": (
        "Passed the {obj} from my {giver} to my {receiver}. Placed the {obj} on {label}.",
        "{obj|을} {giver}에서 {receiver|으로} 넘겨서 {label}에 놓았어요.",
    ),
    "tasks.passed_placed_err": (
        "Passed the {obj} from my {giver} to my {receiver}. Placed the {obj} on {label}, {cm} cm from the spot.",
        "{obj|을} {giver}에서 {receiver|으로} 넘겨서 {label}에 놓았어요 (오차 {cm} cm).",
    ),

    # ------------------------------------------------------------------ reach
    "tasks.reaching_over": ("Reaching over {label}.", "{label} 위로 손을 뻗을게요."),
    "tasks.hand_over": ("My hand is over {label}.", "손이 {label} 위에 있어요."),

    # ------------------------------------------------------------------ drawn paths (Tasks.set_path_world, RobotRuntime.set_path)
    "tasks.stroke_missed": ("The stroke missed the table.", "그은 선이 테이블을 벗어났어요."),
    "tasks.stroke_draw_over": ("Draw over the table or objects.", "테이블이나 물건 위에 그려 주세요."),
    "tasks.path_two_points": ("A path needs at least two waypoints.", "경로에는 경유점이 적어도 두 개 있어야 해요."),
    "tasks.path_all_ok": ("{lim} path: {n} waypoints, all reachable.", "{lim} 경로: 경유점 {n}개, 모두 닿아요."),
    "tasks.path_all_ok_lifted": (
        "{lim} path: {n} waypoints, all reachable, lifted over {over}.",
        "{lim} 경로: 경유점 {n}개, 모두 닿아요. {over} 위로 들어 올렸어요.",
    ),
    "tasks.path_out_of_reach": (
        "{lim} path: {bad}/{n} waypoints out of reach (red).",
        "{lim} 경로: 경유점 {n}개 중 {bad}개가 닿지 않아요 (빨간색).",
    ),
    "tasks.path_out_of_reach_lifted": (
        "{lim} path: {bad}/{n} waypoints out of reach (red), lifted over {over}.",
        "{lim} 경로: 경유점 {n}개 중 {bad}개가 닿지 않아요 (빨간색). {over} 위로 들어 올렸어요.",
    ),
    "tasks.path_hits": (
        "{lim} path: {k}/{n} waypoints where {why} (red).",
        "{lim} 경로: 경유점 {n}개 중 {k}개에서 {why} (빨간색).",
    ),
    "tasks.path_in_zone": (
        "{lim} path: {k}/{n} waypoints inside {zone} (red).",
        "{lim} 경로: 경유점 {n}개 중 {k}개가 {zone} 안에 있어요 (빨간색).",
    ),
    "tasks.say_go_or_f": ("Say 'go' or press F.", "'go'(또는 '가자')라고 말하거나 F를 누르세요."),
    "tasks.press_f_to_follow": ("Press F to follow.", "따라가려면 F를 누르세요."),
    "tasks.redraw_or_f": ("Redraw or press F.", "다시 그리거나 F를 누르세요."),
    "tasks.stand_in_view": (
        "Stand in view first: the path is placed in your body plane",
        "먼저 카메라에 보이게 서 주세요: 경로는 몸 앞 평면에 놓여요",
    ),
    "tasks.path_runs_into": (
        "Part of that path runs into something: {why} (red).",
        "경로 일부가 무언가에 부딪혀요: {why} (빨간색).",
    ),
    "tasks.path_draw_around": ("Draw it around, or over it in 3D.", "돌아가게 그리거나 3D에서 위로 넘어가게 그려 주세요."),
    "tasks.path_through_zone": ("Part of that path goes through {zone} (red).", "경로 일부가 {zone|을} 지나가요 (빨간색)."),
    "tasks.path_redraw_around": ("Redraw it around.", "돌아가도록 다시 그려 주세요."),
    # ("Part of that path is out of my reach (red).", "Redraw it closer." and "There is no path yet." are in ko_features)
    "tasks.no_path_draw": ("Draw one on my view.", "제 화면에 그려 주세요."),
    "tasks.following_path": ("Following your path with my {hand}.", "{hand|으로} 경로를 따라갈게요."),
    "tasks.finished_path": ("Finished the path.", "경로를 끝까지 따라갔어요."),
    # why a waypoint is red (tasks.path_object_hits, traj.surface_checker; the other reasons and the generic
    # "{why} at point {i}." sentences of traj.check are in ko_features)
    "tasks.why_obj_hits": ("the {obj} would hit the {thing}", "{obj|이} {thing}에 부딪혀요"),
    "tasks.why_below_surface": ("{cm} cm below the surface", "표면보다 {cm} cm 아래로 내려가요"),
    "tasks.why_check_failed": ("check failed ({e})", "확인 중에 오류가 났어요 ({e})"),
    "tasks.why_check_failed_cap": ("Check failed ({e})", "확인 중에 오류가 났어요 ({e})"),
    # traj.check's first issue when its subject is one the generic sentences can't put into Korean
    # ("My right hand would hit the mug at point 3, and 1 more place.")
    "tasks.check_my_at": ("My {hand} would hit the {thing} at point {i}.", "{i}번 지점에서 {hand|이} {thing}에 부딪혀요."),
    "tasks.check_my_at_more": (
        "My {hand} would hit the {thing} at point {i}, and {n} more place{_s}.",
        "{i}번 지점에서 {hand|이} {thing}에 부딪혀요. 그 밖에도 {n}곳이 더 있어요.",
    ),
    "tasks.check_my_between": (
        "My {hand} would hit the {thing} between points {i0} and {i1}.",
        "{i0}번과 {i1}번 지점 사이에서 {hand|이} {thing}에 부딪혀요.",
    ),
    "tasks.check_my_between_more": (
        "My {hand} would hit the {thing} between points {i0} and {i1}, and {n} more place{_s}.",
        "{i0}번과 {i1}번 지점 사이에서 {hand|이} {thing}에 부딪혀요. 그 밖에도 {n}곳이 더 있어요.",
    ),
    "tasks.check_my_from": (
        "My {hand} would hit the {thing} from point {i0} to point {i1}.",
        "{i0}번부터 {i1}번 지점까지 {hand|이} {thing}에 부딪혀요.",
    ),
    "tasks.check_my_from_more": (
        "My {hand} would hit the {thing} from point {i0} to point {i1}, and {n} more place{_s}.",
        "{i0}번부터 {i1}번 지점까지 {hand|이} {thing}에 부딪혀요. 그 밖에도 {n}곳이 더 있어요.",
    ),
    "tasks.check_my_start": (
        "My {hand} would hit the {thing} on the way to the start.",
        "시작점으로 가는 길에 {hand|이} {thing}에 부딪혀요.",
    ),
    "tasks.check_my_start_more": (
        "My {hand} would hit the {thing} on the way to the start, and {n} more place{_s}.",
        "시작점으로 가는 길에 {hand|이} {thing}에 부딪혀요. 그 밖에도 {n}곳이 더 있어요.",
    ),
    "tasks.check_carry_at": ("What I carry would hit the {thing} at point {i}.", "{i}번 지점에서 들고 있는 물건이 {thing}에 부딪혀요."),
    "tasks.check_carry_at_more": (
        "What I carry would hit the {thing} at point {i}, and {n} more place{_s}.",
        "{i}번 지점에서 들고 있는 물건이 {thing}에 부딪혀요. 그 밖에도 {n}곳이 더 있어요.",
    ),
    "tasks.check_carry_between": (
        "What I carry would hit the {thing} between points {i0} and {i1}.",
        "{i0}번과 {i1}번 지점 사이에서 들고 있는 물건이 {thing}에 부딪혀요.",
    ),
    "tasks.check_carry_between_more": (
        "What I carry would hit the {thing} between points {i0} and {i1}, and {n} more place{_s}.",
        "{i0}번과 {i1}번 지점 사이에서 들고 있는 물건이 {thing}에 부딪혀요. 그 밖에도 {n}곳이 더 있어요.",
    ),
    "tasks.check_carry_from": (
        "What I carry would hit the {thing} from point {i0} to point {i1}.",
        "{i0}번부터 {i1}번 지점까지 들고 있는 물건이 {thing}에 부딪혀요.",
    ),
    "tasks.check_carry_from_more": (
        "What I carry would hit the {thing} from point {i0} to point {i1}, and {n} more place{_s}.",
        "{i0}번부터 {i1}번 지점까지 들고 있는 물건이 {thing}에 부딪혀요. 그 밖에도 {n}곳이 더 있어요.",
    ),
    "tasks.check_carry_start": (
        "What I carry would hit the {thing} on the way to the start.",
        "시작점으로 가는 길에 들고 있는 물건이 {thing}에 부딪혀요.",
    ),
    "tasks.check_carry_start_more": (
        "What I carry would hit the {thing} on the way to the start, and {n} more place{_s}.",
        "시작점으로 가는 길에 들고 있는 물건이 {thing}에 부딪혀요. 그 밖에도 {n}곳이 더 있어요.",
    ),
    # how a trajectory from keypoints goes past things (plan card reasons: the note, capitalized, with a period)
    "tasks.note_over": ("Over the {x} with {cm} cm to spare.", "{x} 위로 {cm} cm 여유를 두고 지나가요."),
    "tasks.note_around_left": ("Around the {x} on its left.", "{x}의 왼쪽으로 돌아가요."),
    "tasks.note_around_right": ("Around the {x} on its right.", "{x}의 오른쪽으로 돌아가요."),
    "tasks.note_around_circle": ("Around the {x}, {cm} cm from its centre.", "{x} 중심에서 {cm} cm 떨어져 둘레를 돌아요."),
    "tasks.note_lifted": ("Lifted over the {x}.", "{x} 위로 들어 올려 지나가요."),

    # ------------------------------------------------------------------ trajectories that can't be used (traj.TrajError)
    "tasks.traj_numbers": ("{what} must be {n} numbers.", "{what|은} 숫자 {n}개여야 해요."),
    "tasks.traj_point_object": (
        "Point {i} must be an object with p: [x, y, z].",
        "{i}번 지점은 p: [x, y, z]가 있는 객체여야 해요.",
    ),
    "tasks.traj_point_no_p": ("Point {i} has no position p.", "{i}번 지점에 위치 p가 없어요."),
    "tasks.traj_point_far": ("Point {i} is more than 10 m away.", "{i}번 지점이 10 m보다 멀리 있어요."),
    "tasks.traj_pattern": (
        "Pattern must be {{kind, amp, freq, axis}}.",
        "패턴은 {{kind, amp, freq, axis}} 형식이어야 해요.",
    ),
    "tasks.traj_pattern_kind": ("Pattern kind must be one of {kinds}.", "패턴 종류는 {kinds} 중 하나여야 해요."),
    "tasks.traj_pattern_axis": ("Pattern axis must be one of {axes}.", "패턴 축은 {axes} 중 하나여야 해요."),
    "tasks.traj_frame": (
        "Frame must be 'world' or 'object:<name>'.",
        "frame은 'world' 또는 'object:<name>'이어야 해요.",
    ),
    "tasks.traj_object": ("A trajectory must be an object with points.", "궤적은 points가 있는 객체여야 해요."),
    "tasks.traj_one_point": ("A trajectory needs at least one point.", "궤적에는 지점이 적어도 하나 있어야 해요."),
    "tasks.traj_max_points": ("A trajectory can have at most {n} points.", "궤적의 지점은 최대 {n}개까지예요."),
    "tasks.traj_role": ("Role must be 'right' or 'left'.", "role은 'right' 또는 'left'여야 해요."),
    "tasks.traj_source": ("Source must be one of {sources}.", "source는 {sources} 중 하나여야 해요."),
    "tasks.traj_frame_missing": (
        "The trajectory is relative to the {x}, which isn't in the scene.",
        "궤적이 {x} 기준인데 장면에 {x|이} 없어요.",
    ),
    "tasks.traj_spec_object": ("The trajectory spec must be an object.", "궤적 명세는 객체여야 해요."),
    "tasks.traj_give_keypoint": ("Give at least one keypoint.", "키포인트를 적어도 하나 주세요."),
    "tasks.traj_max_keypoints": ("At most 40 keypoints.", "키포인트는 최대 40개예요."),
    "tasks.traj_keypoint_object": ("Keypoint {i} must be an object.", "{i}번 키포인트는 객체여야 해요."),
    "tasks.traj_keypoint_hand": (
        "Keypoint {i}: only the first point can be where the hand is now.",
        "{i}번 키포인트: 지금 손이 있는 곳은 첫 번째 지점만 될 수 있어요.",
    ),
    "tasks.traj_keypoint_needs": (
        "Keypoint {i} needs one of: of (an object), xyz, dxyz, marker, region, from: hand.",
        "{i}번 키포인트에는 of(물체), xyz, dxyz, marker, region, from: hand 중 하나가 필요해요.",
    ),
    "tasks.traj_hand_unknown": ("I don't know where my hand is.", "제 손이 어디 있는지 모르겠어요."),
    "tasks.traj_dxyz": ("A relative step (dxyz) needs a point before it.", "상대 이동(dxyz) 앞에는 지점이 있어야 해요."),
    "tasks.traj_no_marker_tap": (
        "There is no marker {m}; tap the spot first.",
        "마커 {m|이} 없어요. 먼저 그 자리를 탭해 주세요.",
    ),
    "tasks.traj_no_marker": ("There is no marker {m}.", "마커 {m|이} 없어요."),
    "tasks.traj_region": ("I don't know the region '{region}'.", "'{region}' 영역은 몰라요."),
    "tasks.traj_dont_see_here": ("I don't see '{x}' here.", "여기에 {x|이} 보이지 않아요."),
    "tasks.traj_dont_see_have": ("I don't see '{x}' here (I see {have}).", "여기에 {x|이} 보이지 않아요 ({have|이} 보여요)."),
    "tasks.traj_dont_see_nothing": (
        "I don't see '{x}' here (I see nothing).",
        "여기에 {x|이} 보이지 않아요 (아무것도 안 보여요).",
    ),
    "tasks.traj_dont_see_near": ("I don't see '{x}' near the path.", "경로 근처에 {x|이} 보이지 않아요."),
    "tasks.traj_via": ("Via must be one of {via}.", "via는 {via} 중 하나여야 해요."),
    "tasks.traj_rel": ("Rel must be one of {rel}.", "rel은 {rel} 중 하나여야 해요."),
    "tasks.traj_no_point": (
        "There is no point {i} (the trajectory has {n}).",
        "{i}번 지점은 없어요 (궤적의 지점은 {n}개예요).",
    ),
    "tasks.traj_which_point": ("Say which point: index or near.", "어느 지점인지 말해 주세요: index 또는 near."),
    "tasks.traj_point_first": (
        "Point at the spot or tap it first, then say 'there'.",
        "먼저 그 자리를 가리키거나 탭한 다음 'there'(또는 '저기')라고 말하세요.",
    ),
    "tasks.traj_changes_object": ("Changes must be an object.", "changes는 객체여야 해요."),
    "tasks.traj_surface_unknown": ("I don't know where the surface is.", "표면이 어디 있는지 모르겠어요."),
    "tasks.traj_speed": (
        "Speed must be m/s, or 'slower' / 'faster'.",
        "속도는 m/s 값이거나 'slower' / 'faster'여야 해요.",
    ),
    "tasks.traj_demo_no_traj": ("That demonstration has no trajectory.", "그 시범에는 궤적이 없어요."),
    "tasks.traj_demo_anchor": (
        "A demonstration without an anchor object needs the hand position to start from.",
        "기준 물체가 없는 시범은 시작할 손 위치가 있어야 해요.",
    ),

    # ------------------------------------------------------------------ drawers, doors, containers (pps/manip.py)
    "tasks.nothing_opens": ("There is nothing here that opens.", "여기에는 열 수 있는 게 없어요."),
    "tasks.no_containers": ("There are no containers here.", "여기에는 물건을 넣을 곳이 없어요."),
    "tasks.no_drawer_door": ("There is no drawer or door here.", "여기에는 서랍이나 문이 없어요."),
    "tasks.no_container": ("There is no container here.", "여기에는 물건을 넣을 곳이 없어요."),
    "tasks.which_drawer_door": ("Which drawer or door: {list}?", "{list} 중 어느 거요?"),
    "tasks.which_container": ("Which container: {list}?", "{list} 중 어디요?"),
    "tasks.which_one": ("Which one: {list}?", "{list} 중 어느 거요?"),
    "tasks.dont_see_there_is": (
        "I don't see '{text}'. There is {list}.",
        "{text|이} 보이지 않아요. 여기에는 {list|이} 있어요.",
    ),
    "tasks.cant_open_body": (
        "I can't open things with this body: it has no gripper I can drive.",
        "이 몸으로는 물건을 열 수 없어요: 제가 움직일 수 있는 그리퍼가 없어요.",
    ),
    "tasks.cant_close_body": (
        "I can't close things with this body: it has no gripper I can drive.",
        "이 몸으로는 물건을 닫을 수 없어요: 제가 움직일 수 있는 그리퍼가 없어요.",
    ),
    "tasks.holding_put_down_open": (
        "I'm holding the {obj}. Put it down first, then I'll open {label}.",
        "{obj|을} 들고 있어요. 먼저 내려놓은 다음에 {label|을} 열게요.",
    ),
    "tasks.holding_put_down_close": (
        "I'm holding the {obj}. Put it down first, then I'll close {label}.",
        "{obj|을} 들고 있어요. 먼저 내려놓은 다음에 {label|을} 닫을게요.",
    ),
    "tasks.already_open": ("{label} is already open ({pct}%).", "{label|은} 이미 열려 있어요 ({pct}%)."),
    "tasks.already_closed": ("{label} is already closed.", "{label|은} 이미 닫혀 있어요."),
    "tasks.knob_blocked": (
        "I can't get my fingers around the handle of {label}: {above} is right above it and my gripper only grasps "
        "handles from above (a robot that reaches in from the front could).",
        "{label} 손잡이에 손가락을 걸 수 없어요: 바로 위에 {above|이} 있는데 제 그리퍼는 손잡이를 위에서만 잡을 수 "
        "있어요 (앞에서 손을 넣는 로봇이라면 할 수 있어요).",
    ),
    "tasks.no_gripper_handle": (
        "I have no gripper on my {hand} to hold {label}'s handle.",
        "{hand}에 {label} 손잡이를 잡을 그리퍼가 없어요.",
    ),
    "tasks.opening_to": ("Opening {label} to {pct}%.", "{label|을} {pct}%까지 열게요."),
    "tasks.closing": ("Closing {label}.", "{label|을} 닫을게요."),
    "tasks.opened": ("Opened {label}.", "{label|을} 열었어요."),
    "tasks.opened_to": ("Opened {label} to {pct}%.", "{label|을} {pct}%까지 열었어요."),
    "tasks.closed": ("Closed {label}.", "{label|을} 닫았어요."),
    "tasks.pushed_to": ("Pushed {label} to {pct}% open.", "{label|을} {pct}% 열린 데까지 밀었어요."),
    "tasks.only_got_to": (
        "{label} only got to {pct}% open (I wanted {want}%).",
        "{label|이} {pct}% 열린 데서 멈췄어요 (목표는 {want}%였어요).",
    ),
    "tasks.guard_closed": (
        "{label} is closed: open it first (I need it at least {need}% open to get in).",
        "{label|이} 닫혀 있어요: 먼저 열어야 해요 (안에 손을 넣으려면 적어도 {need}% 열려 있어야 해요).",
    ),
    "tasks.guard_partly": (
        "{label} is only {pct}% open: open it first (I need it at least {need}% open to get in).",
        "{label|은} {pct}%만 열려 있어요: 먼저 더 열어야 해요 (안에 손을 넣으려면 적어도 {need}% 열려 있어야 해요).",
    ),
    "tasks.handle_no_grip": (
        "I couldn't get a grip on the handle of {label}: my fingers closed without holding it.",
        "{label} 손잡이를 잡지 못했어요: 손가락이 아무것도 잡지 못하고 닫혔어요.",
    ),
    "tasks.handle_slipped": (
        "My gripper slipped off the handle of {label}; it's {pct}% open now.",
        "그리퍼가 {label} 손잡이에서 미끄러졌어요. 지금 {pct}% 열려 있어요.",
    ),
    "tasks.stuck_open_slipped": (
        "{label} won't open: it's stuck at {pct}% open (it didn't move when I pulled, and my gripper slipped off the "
        "handle).",
        "{label|이} 열리지 않아요: {pct}% 열린 채로 꼼짝하지 않아요 (당겨도 움직이지 않았고 그리퍼가 손잡이에서 "
        "미끄러졌어요).",
    ),
    "tasks.stuck_close_slipped": (
        "{label} won't close: it's stuck at {pct}% open (it didn't move when I pulled, and my gripper slipped off the "
        "handle).",
        "{label|이} 닫히지 않아요: {pct}% 열린 채로 꼼짝하지 않아요 (밀어도 움직이지 않았고 그리퍼가 손잡이에서 "
        "미끄러졌어요).",
    ),
    "tasks.stuck_open": (
        "{label} won't open: it's stuck at {pct}% open (it didn't move when I pulled).",
        "{label|이} 열리지 않아요: {pct}% 열린 채로 꼼짝하지 않아요 (당겨도 움직이지 않았어요).",
    ),
    "tasks.stuck_close": (
        "{label} won't close: it's stuck at {pct}% open (it didn't move when I pulled).",
        "{label|이} 닫히지 않아요: {pct}% 열린 채로 꼼짝하지 않아요 (밀어도 움직이지 않았어요).",
    ),
    "tasks.stuck_open_further": (
        "{label} won't open any further: it's stuck at {pct}% open.",
        "{label|이} 더 이상 열리지 않아요: {pct}% 열린 채로 꼼짝하지 않아요.",
    ),
    "tasks.stuck_close_further": (
        "{label} won't close any further: it's stuck at {pct}% open.",
        "{label|이} 더 이상 닫히지 않아요: {pct}% 열린 채로 꼼짝하지 않아요.",
    ),
    "tasks.stuck_open_further_still": (
        "{label} won't open any further: it's stuck at {pct}% open (it didn't move when I pulled).",
        "{label|이} 더 이상 열리지 않아요: {pct}% 열린 채로 꼼짝하지 않아요 (당겨도 움직이지 않았어요).",
    ),
    "tasks.stuck_close_further_still": (
        "{label} won't close any further: it's stuck at {pct}% open (it didn't move when I pulled).",
        "{label|이} 더 이상 닫히지 않아요: {pct}% 열린 채로 꼼짝하지 않아요 (밀어도 움직이지 않았어요).",
    ),
    "tasks.let_go_handle": ("I let go of the handle.", "손잡이를 놓았어요."),
    "tasks.is_empty": ("{label} is empty.", "{label|은} 비어 있어요."),
    "tasks.dont_see_in": ("I don't see '{obj}' in {label}.", "{label}에 {obj|이} 보이지 않아요."),
    "tasks.isnt_in": (
        "The {obj} isn't in {label}. In {label} there is {have}.",
        "{obj|은} {label}에 없어요. {label}에는 {have|이} 있어요.",
    ),
    "tasks.isnt_in_nothing": (
        "The {obj} isn't in {label}. In {label} there is nothing.",
        "{obj|은} {label}에 없어요. {label}에는 아무것도 없어요.",
    ),
    "tasks.isnt_in_where": (
        "The {obj} isn't in {label} (it's in {where}). In {label} there is {have}.",
        "{obj|은} {label}에 없어요 ({where}에 있어요). {label}에는 {have|이} 있어요.",
    ),
    "tasks.isnt_in_where_nothing": (
        "The {obj} isn't in {label} (it's in {where}). In {label} there is nothing.",
        "{obj|은} {label}에 없어요 ({where}에 있어요). {label}에는 아무것도 없어요.",
    ),
    "tasks.already_holding": ("I'm already holding the {obj}.", "이미 {obj|을} 들고 있어요."),
    "tasks.what_put_in": (
        "What should I put in {label}? I can move {list}.",
        "{label}에 무엇을 넣을까요? {list|을} 옮길 수 있어요.",
    ),
    "tasks.what_put_in_nothing": (
        "What should I put in {label}? I can move nothing here.",
        "{label}에 무엇을 넣을까요? 여기엔 제가 옮길 수 있는 물건이 없어요.",
    ),
    "tasks.container_full": (
        "{label} is full: there is no room for the {obj}.",
        "{label|은} 꽉 찼어요: {obj|을} 넣을 자리가 없어요.",
    ),

    # ------------------------------------------------------------------ follow me, skills, drive, stop, home
    "tasks.following_you": ("Following you.", "따라 할게요."),
    "tasks.move_your_arms": (
        "Move your arms; I'll record it so you can name it.",
        "팔을 움직여 보세요. 녹화해 두었다가 이름을 붙일 수 있게 할게요.",
    ),
    "tasks.stopped_following": ("Stopped following.", "따라 하기를 멈췄어요."),
    "tasks.recorded": ("I recorded {s} s ({n} frames).", "{s} s 분량({n}프레임)을 녹화했어요."),
    "tasks.say_learn_as": (
        "Say 'learn this as' and a name.",
        "'learn this as'와 이름을 말해 주세요 (또는 '인사라고 배워'처럼).",
    ),
    "tasks.give_skill_name": ("Give the skill a name.", "스킬 이름을 정해 주세요."),
    "tasks.nothing_to_learn": ("I have nothing to learn yet.", "아직 배울 동작이 없어요."),
    "tasks.say_follow_me": (
        "Say 'follow me', show the motion, then name it.",
        "'follow me'(또는 '따라 해')라고 말하고 동작을 보여 준 다음 이름을 붙여 주세요.",
    ),
    "tasks.learned": ("Learned '{name}': {s} s, {pct}% feasible.", "'{name}' 동작을 배웠어요: 길이 {s} s, {pct}% 실행 가능해요."),
    "tasks.unknown_skill": ("I don't know '{name}'.", "'{name}' 동작은 몰라요."),
    "tasks.known_skills_none": ("I know: nothing yet.", "아직 아는 동작이 없어요."),
    "tasks.finished_skill": ("Finished '{skill}'.", "'{skill}' 동작을 마쳤어요."),
    "tasks.drive_how_far": (
        "Say how far: dx / dy in metres, dyaw in degrees.",
        "얼마나 움직일지 말해 주세요: dx / dy는 미터, dyaw는 도 단위예요.",
    ),
    "tasks.still_busy": ("I'm still busy with the last task.", "아직 이전 작업을 하고 있어요."),
    "tasks.say_stop_first": ("Say stop first.", "먼저 'stop'(또는 '멈춰')이라고 말해 주세요."),
    "tasks.stopped_by_you": ("Stopped by you before it finished.", "끝나기 전에 멈추라고 하셔서 멈췄어요."),
    "tasks.holding_pose": ("Holding this pose.", "이 자세를 유지할게요."),
    "tasks.scene_reset": ("Scene reset.", "장면을 초기화했어요."),
    "tasks.back_to_start": ("Back to my start pose.", "시작 자세로 돌아왔어요."),
    "tasks.no_skill_called": ("I don't know a skill called '{name}'.", "'{name}' 스킬은 없어요."),
    "tasks.time_range": ("Pick a time range longer than 0.1 s.", "0.1 s보다 긴 시간 구간을 골라 주세요."),
    "tasks.say_faster": ("Say faster, slower, bigger or smaller.", "빠르게, 느리게, 크게, 작게 중 하나로 말해 주세요."),
    "tasks.edited_skill": (
        "Edited '{skill}': {part} from {t0} to {t1} s, {changes}. New length {t} s.",
        "'{skill}' 동작을 고쳤어요: {part} {t0}~{t1} s 구간, {changes}. 새 길이는 {t} s예요.",
    ),
    "tasks.edit_bigger": ("bigger x{a}", "크게 x{a}"),
    "tasks.edit_smaller": ("smaller x{a}", "작게 x{a}"),
    "tasks.edit_faster": ("faster x{v}", "빠르게 x{v}"),
    "tasks.edit_slower": ("slower x{v}", "느리게 x{v}"),
    "tasks.edit_bigger_faster": ("bigger x{a} and faster x{v}", "크게 x{a}, 빠르게 x{v}"),
    "tasks.edit_bigger_slower": ("bigger x{a} and slower x{v}", "크게 x{a}, 느리게 x{v}"),
    "tasks.edit_smaller_faster": ("smaller x{a} and faster x{v}", "작게 x{a}, 빠르게 x{v}"),
    "tasks.edit_smaller_slower": ("smaller x{a} and slower x{v}", "작게 x{a}, 느리게 x{v}"),
}
