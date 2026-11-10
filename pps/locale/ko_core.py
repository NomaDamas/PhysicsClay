"""Korean server messages: texts that live in data files (demo packs, pps/world/packs/*.json) and shared lines.
English is the message id; see pps/i18n.py."""

MESSAGES = {
    # demo packs: labels, summaries and the line the robot says when a pack is set up
    "core.pack_gym": ("Gym", "체육관"),
    "core.pack_gym_summary": (
        "A bench and a 20 kg barbell. The humanoid counts your squats and corrects your form, and spots your lift: it "
        "follows the bar under your hands and helps, force-limited, when it stalls.",
        "벤치와 20 kg 바벨이 있어요. 휴머노이드가 스쿼트 횟수를 세고 자세를 바로잡아 주며, 리프트를 보조해요. 손 아래에서 "
        "바를 따라가다가 멈추면 힘을 제한한 채로 도와줘요.",
    ),
    "core.pack_gym_say": (
        "Gym ready. Say “coach me” and squat, or pick up the bar and say “spot me”: I'll follow it under your hands and "
        "push when you get stuck.",
        "체육관 준비됐어요. “coach me”라고 말하고 스쿼트를 하거나, 바를 들고 “spot me”라고 말하세요. 손 아래에서 바를 "
        "따라가다가 막히면 밀어 드릴게요.",
    ),
    "core.pack_weld": ("Weld cell", "용접 셀"),
    "core.pack_weld_summary": (
        "Two steel plate butt joints (a straight seam and an L seam) and a welding torch on the arm. Show the weave with "
        "your hand; the robot welds the other seam in your style.",
        "강철판 맞대기 이음 두 개(직선 용접선과 L자 용접선)와 팔에 단 용접 토치가 있어요. 손으로 위빙을 보여 주면 로봇이 "
        "다른 용접선을 같은 스타일로 용접해요.",
    ),
    "core.pack_weld_say": (
        "Weld cell ready: a straight seam and an L seam, and a torch on my arm. Say “watch me” and show me the weave with "
        "your hand, then say done.",
        "용접 셀 준비됐어요. 직선 용접선과 L자 용접선이 있고, 제 팔에 토치가 달려 있어요. “watch me”라고 말하고 손으로 "
        "위빙을 보여 준 다음 done이라고 말하세요.",
    ),
    "core.pack_workshop": ("Workshop helper", "작업장 도우미"),
    "core.pack_workshop_summary": (
        "A wrench, a screwdriver and a brush in stands on the bench. “When I put my hand out, give me the wrench” is a "
        "standing rule: hold your open hand out and the robot hands it over.",
        "작업대 거치대에 렌치, 드라이버, 솔이 있어요. “손을 내밀면 렌치를 줘”가 상시 규칙이에요. 편 손을 내밀면 로봇이 "
        "건네줘요.",
    ),
    "core.pack_workshop_say": (
        "Workshop ready. I'm watching for your open hand: hold it out and I'll give you the wrench.",
        "작업장 준비됐어요. 편 손을 내미는지 보고 있을게요. 손을 내밀면 렌치를 건네 드릴게요.",
    ),
    # robot labels in the robot picker
    "core.robot_humanoid": ("{name} humanoid", "{name} 휴머노이드"),
    "core.robot_arm": ("{name} arm", "{name} 로봇팔"),
    "core.robot_two_arms": ("{name} two arms", "{name} 양팔 로봇"),
    "core.robot_mobile": ("{name} mobile manipulator", "{name} 모바일 매니퓰레이터"),
    "core.robot_quadruped": ("{name} quadruped with an arm", "{name} 팔 달린 사족 보행 로봇"),
    # plan.py: a spot that was taken, moved beside what is there
    "core.label_beside": ("{label}, beside {who}", "{label} ({who} 바로 옆)"),
    # shared short lines
    "core.done": ("Done.", "완료했어요."),
    "core.stopped": ("Stopped.", "멈췄어요."),
    "core.cleared": ("Cleared.", "지웠어요."),
    "core.lang_ko": ("Messages are in Korean now.", "이제 메시지를 한국어로 보여 드릴게요."),
}
