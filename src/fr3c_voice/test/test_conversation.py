from pathlib import Path

from fr3c_voice.conversation import (
    ShortConversation,
    build_vision_prompt,
    load_dialogue_settings,
)


WORKSPACE = Path(__file__).resolve().parents[3]


def test_stage8_settings_are_loaded_from_the_shared_config():
    settings = load_dialogue_settings(WORKSPACE / 'config/voice/qwen3vl.yaml')
    assert settings.camera_topic == '/camera/color/image_raw'
    assert settings.max_history_turns == 4
    assert settings.camera_max_age_s == 1.0


def test_short_conversation_keeps_complete_recent_turns_within_budget():
    memory = ShortConversation(max_turns=2, max_prompt_bytes=1400)
    memory.record('第一问', '第一答')
    memory.record('第二问', '第二答')
    memory.record('第三问', '第三答')
    assert [turn.user for turn in memory.turns] == ['第二问', '第三问']
    prompt = memory.build_prompt('请接着说')
    assert '第二问' in prompt
    assert '第三答' in prompt
    assert '第一问' not in prompt
    assert '请接着说' in prompt
    assert len(prompt.encode('utf-8')) <= 1400


def test_chat_and_vision_prompts_do_not_claim_robot_or_camera_actions():
    prompt = ShortConversation().build_prompt('你是谁？')
    assert '不要声称已经控制机械臂' in prompt
    vision = build_vision_prompt('桌面上有什么？', '2026-09-17 12:00:00 +0800')
    assert '真实截图' in vision
    assert '不要生成或猜测机器人坐标' in vision
    assert '桌面上有什么' in vision
