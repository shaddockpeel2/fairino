import pytest

from fr3c_voice.asr_text import normalize_robot_asr_text
from fr3c_voice.intent import safe_local_action_route


@pytest.mark.parametrize('raw', [
    '红色方块放到放置去',
    '红色方块放到放置居',
    '红色方块放到放治区',
    '红色方块放到放制取',
])
def test_observed_placement_homophones_are_normalized(raw):
    corrected = normalize_robot_asr_text(raw)
    assert corrected == '红色方块放到放置区'
    intent = safe_local_action_route(corrected)
    assert intent is not None
    assert intent.task == 'pick_sort'
    assert intent.color == 'red'


def test_other_destination_homophones_are_normalized():
    assert normalize_robot_asr_text('把蓝色方块放到分类去') == (
        '把蓝色方块放到分类区')
    assert normalize_robot_asr_text('把黄色方块放到对应居') == (
        '把黄色方块放到对应区')
    assert normalize_robot_asr_text('把红色方块放到D去') == (
        '把红色方块放到D区')


@pytest.mark.parametrize(('raw', 'expected'), [
    ('转起红色方块', '抓取红色方块'),
    ('只要娶黄色方块', '抓取黄色方块'),
    ('抓去蓝色方块', '抓取蓝色方块'),
    ('折取绿色方块', '抓取绿色方块'),
    ('打红色方块放到分类区', '把红色方块放到分类区'),
])
def test_recorded_grasp_homophones_are_normalized(raw, expected):
    assert normalize_robot_asr_text(raw) == expected


def test_erhua_is_removed_only_in_robot_context():
    assert normalize_robot_asr_text('绿色方块儿放到放治区') == (
        '绿色方块放到放置区')


@pytest.mark.parametrize('text', [
    '我准备放置居家用品',
    '黑色方块放到分类区',
    '这方块放到分类区',
])
def test_normalizer_does_not_invent_colour_or_missing_parameters(text):
    corrected = normalize_robot_asr_text(text)
    if text == '我准备放置居家用品':
        assert corrected == text
    assert safe_local_action_route(corrected) is None


@pytest.mark.parametrize('text', [
    '我每天早起去公园',
    '他转起身就走了',
    '只要娶她就好',
    '我们去D区看看',
    '打红色方块',
])
def test_grasp_corrections_do_not_rewrite_conversation(text):
    assert normalize_robot_asr_text(text) == text
