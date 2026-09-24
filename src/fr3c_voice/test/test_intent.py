import json
from pathlib import Path

import pytest

from fr3c_voice.intent import (
    IntentProtocolError,
    classify_text,
    direct_shortcut,
    parse_dialogue_intent,
    safe_local_action_route,
    safe_local_non_action_route,
    source_location_for_intent,
)


WORKSPACE = Path(__file__).resolve().parents[3]
CORPUS = WORKSPACE / 'config/voice/p4_intent_corpus.json'


def encoded(route='chat', task=None, color=None, zone=None,
            support_color=None, reply='好的。'):
    return json.dumps({
        'schema_version': '1.0', 'route': route,
        'task': task, 'color': color, 'zone': zone,
        'support_color': support_color, 'reply': reply,
    }, ensure_ascii=False)


def test_parser_accepts_only_exact_contract():
    parsed = parse_dialogue_intent(encoded('action', 'pick_hold', 'red'))
    assert parsed.route == 'action'
    assert parsed.task == 'pick_hold'
    assert parsed.color == 'red'
    assert parsed.zone is None


def test_parser_accepts_and_normalizes_pick_place_zone():
    parsed = parse_dialogue_intent(
        encoded('action', 'pick_place', 'red', ' D3 '))
    assert parsed.task == 'pick_place'
    assert parsed.color == 'red'
    assert parsed.zone == 'd3'


def test_parser_and_semantics_accept_pick_stack_with_ordered_roles():
    raw = encoded(
        'action', 'pick_stack', 'red', support_color='blue',
        reply='准备码放。')
    intent, _ = classify_text('把红色方块放到蓝色方块上面', lambda _prompt: raw)
    assert intent.task == 'pick_stack'
    assert intent.color == 'red'
    assert intent.support_color == 'blue'
    assert intent.zone is None


@pytest.mark.parametrize('text,task,source', [
    ('把分类区里的红色方块放到蓝色方块上面', 'pick_stack', 'sort_area'),
    ('把分类区里的红色方块拿起来', 'pick_hold', 'sort_area'),
    ('把分类区里的红色方块放到A3', 'pick_place', 'sort_area'),
    ('把红色方块放到分类区', 'pick_sort', ''),
    ('把红色方块放到蓝色方块上面', 'pick_stack', ''),
])
def test_sort_area_source_requires_explicit_source_wording(text, task, source):
    intent = safe_local_action_route(text)
    assert intent is not None
    assert intent.task == task
    assert source_location_for_intent(text, intent) == source


def test_model_cannot_swap_sort_area_source_color():
    raw = encoded('action', 'pick_stack', 'blue', support_color='red')
    with pytest.raises(IntentProtocolError):
        classify_text('把分类区里的红色方块放到蓝色方块上面', lambda _: raw)


def test_auto_stack_does_not_silently_treat_sort_area_as_tabletop():
    text = '把分类区的方块自动码垛'
    assert safe_local_action_route(text) is None
    raw = encoded('action', 'pick_auto_stack', reply='准备自动码垛。')
    with pytest.raises(IntentProtocolError):
        classify_text(text, lambda _: raw)


def test_stack_roles_cannot_be_swapped_by_the_model():
    wrong = encoded(
        'action', 'pick_stack', 'blue', support_color='red',
        reply='准备码放。')
    with pytest.raises(IntentProtocolError) as caught:
        classify_text('把红色方块码到蓝色方块上面', lambda _prompt: wrong)
    assert caught.value.code == 'model_stack_role_mismatch'


@pytest.mark.parametrize('raw,code', [
    ('```json\n{}\n```', 'invalid_json'),
    ('[]', 'invalid_top_level'),
    (encoded('action', None, 'red'), 'action_missing_parameters'),
    (encoded('chat', 'pick_hold', 'red'), 'non_action_with_parameters'),
    (encoded('action', 'pick_place', 'red'), 'action_missing_zone'),
    (encoded('action', 'pick_sort', 'red', 'd'), 'zone_not_allowed'),
    (encoded('action', 'pick_place', 'red', 'd0'), 'invalid_zone'),
    (encoded('action', 'pick_stack', 'red'), 'action_missing_support_color'),
    (encoded('action', 'pick_stack', 'red', support_color='red'),
     'action_missing_support_color'),
    (encoded('action', 'pick_hold', 'red', support_color='blue'),
     'support_color_not_allowed'),
    (json.dumps({
        'schema_version': '1.0', 'route': 'chat', 'task': None,
        'color': None, 'zone': None, 'support_color': None,
        'reply': 'ok', 'x': 1}), 'invalid_fields'),
    (encoded(reply='运行 ros2 命令'), 'forbidden_reply'),
])
def test_parser_fails_closed(raw, code):
    with pytest.raises(IntentProtocolError) as caught:
        parse_dialogue_intent(raw)
    assert caught.value.code == code


def test_all_corpus_cases_have_valid_expected_contract_and_safe_semantics():
    document = json.loads(CORPUS.read_text(encoding='utf-8'))
    cases = document['cases']
    assert len(cases) >= 60
    assert len({case['id'] for case in cases}) == len(cases)
    for case in cases:
        raw = encoded(
            case['route'], case.get('task'), case.get('color'),
            case.get('zone'), case.get('support_color'), '已正确分类。')
        intent, _ = classify_text(case['text'], lambda _prompt, value=raw: value)
        assert (intent.route, intent.task, intent.color, intent.support_color) == (
            case['route'], case.get('task'), case.get('color'),
            case.get('support_color'))


@pytest.mark.parametrize('text', [
    '不要抓红色方块',
    '怎么抓红色方块？',
    '如果有红色方块就抓起来',
    '把“抓红色方块”翻译成英文',
    '抓起红色方块然后放到分类区',
    '抓红色和蓝色方块',
])
def test_defense_in_depth_rejects_model_false_action(text):
    malicious = encoded(
        'action', 'pick_hold', 'red', reply='准备执行。')
    with pytest.raises(IntentProtocolError):
        classify_text(text, lambda _prompt: malicious)


@pytest.mark.parametrize('text,zone', [
    ('把黄色方块放置到D区域', 'd'),
    ('把红色方块放到A3', 'a3'),
    ('请将蓝方块放入 b 区', 'b'),
])
def test_named_region_routes_to_pick_place_independent_of_color(text, zone):
    raw = encoded(
        'action', 'pick_place',
        'yellow' if '黄' in text else 'red' if '红' in text else 'blue',
        zone.upper(), reply='准备执行指定区域放置。')
    intent, _ = classify_text(text, lambda _prompt: raw)
    assert intent.task == 'pick_place'
    assert intent.zone == zone


def test_model_cannot_substitute_color_mapping_for_named_region():
    wrong = encoded(
        'action', 'pick_sort', 'red', reply='准备放入分类区。')
    with pytest.raises(IntentProtocolError) as caught:
        classify_text('把红色方块放到D区', lambda _prompt: wrong)
    assert caught.value.code == 'named_zone_requires_pick_place'


def test_model_zone_must_match_the_users_named_region():
    wrong = encoded(
        'action', 'pick_place', 'red', 'c', reply='准备放置。')
    with pytest.raises(IntentProtocolError) as caught:
        classify_text('把红色方块放到D区', lambda _prompt: wrong)
    assert caught.value.code == 'model_zone_mismatch'


def test_designated_zone_means_the_preconfigured_color_sort_region():
    raw = encoded(
        'action', 'pick_sort', 'red',
        reply='准备将红色方块放入预设颜色区域。')

    intent, _ = classify_text(
        '把红色方块放到指定放置区', lambda _prompt: raw)

    assert intent.route == 'action'
    assert intent.task == 'pick_sort'
    assert intent.color == 'red'
    assert intent.zone is None


def test_invalid_model_response_fails_after_one_generation():
    calls = []

    def generate(prompt):
        calls.append(prompt)
        return encoded(
            'action', 'pick_sort', 'red',
            reply='准备放入分类区。')

    with pytest.raises(IntentProtocolError) as caught:
        classify_text('把红色方块放到D区', generate)

    assert caught.value.code == 'named_zone_requires_pick_place'
    assert len(calls) == 1


@pytest.mark.parametrize('text', [
    '把绿色方块放到放置区',
    '把绿色方块放到放置区域',
])
def test_bare_placement_zone_means_the_preconfigured_color_sort_region(text):
    intent = safe_local_action_route(text)

    assert intent is not None
    assert intent.route == 'action'
    assert intent.task == 'pick_sort'
    assert intent.color == 'green'
    assert intent.zone is None


def test_stop_shortcuts_do_not_call_model():
    for text, route in (
            ('停止抓取！', 'stop_action'), ('请马上停止抓取吧', 'stop_action'),
            ('别说了', 'stop_playback')):
        shortcut = direct_shortcut(text)
        assert shortcut is not None
        assert shortcut.route == route
        intent, raw = classify_text(
            text, lambda _prompt: pytest.fail('model must not be called'))
        assert intent.route == route
        assert raw == ''


@pytest.mark.parametrize('text', [
    '物体已取下',
    '方块已经拿走',
    '上一个任务已完成',
    '解除持物状态',
])
def test_release_ack_shortcuts_do_not_call_model(text):
    intent, raw = classify_text(
        text, lambda _prompt: pytest.fail('model must not be called'))
    assert intent.route == 'release_ack'
    assert intent.task is None
    assert intent.color is None
    assert raw == ''


def test_model_cannot_acknowledge_release_from_ambiguous_text():
    unsafe = encoded('release_ack', reply='正在同步状态。')
    with pytest.raises(IntentProtocolError) as caught:
        classify_text('好了', lambda _prompt: unsafe)
    assert caught.value.code == 'release_not_explicitly_confirmed'


@pytest.mark.parametrize('text', [
    '你好', '你是谁？', '请解释一下什么是人工智能', '今天天气这个词是什么意思？',
])
def test_obvious_general_chat_can_skip_the_model_classifier(text):
    route = safe_local_non_action_route(text)
    assert route is not None
    assert route.route == 'chat'


def test_explicit_vision_can_skip_classifier_but_uses_no_action_route():
    route = safe_local_non_action_route('你看到了什么？')
    assert route is not None
    assert route.route == 'vision'
    assert route.task is None


@pytest.mark.parametrize('text,task,color,zone,support_color', [
    ('将蓝色方块放到A区域', 'pick_place', 'blue', 'a', None),
    ('请将红方块放入 b3 区', 'pick_place', 'red', 'b3', None),
    ('分类绿色方块', 'pick_sort', 'green', None, None),
    ('帮我拿起黄色方块', 'pick_hold', 'yellow', None, None),
    ('把红色方块放到蓝色方块上面',
     'pick_stack', 'red', None, 'blue'),
])
def test_unambiguous_single_action_uses_safe_local_route(
        text, task, color, zone, support_color):
    route = safe_local_action_route(text)
    assert route is not None
    assert (route.task, route.color, route.zone, route.support_color) == (
        task, color, zone, support_color)


@pytest.mark.parametrize('text', [
    '不要将蓝色方块放到A区域',
    '怎么将蓝色方块放到A区域？',
    '如果有蓝色方块就放到A区域',
    '把蓝色方块放好',
    '把蓝色方块放到A区域然后抓红色方块',
])
def test_local_action_route_never_guesses_ambiguous_or_unsafe_commands(text):
    assert safe_local_action_route(text) is None


@pytest.mark.parametrize('text', [
    '抓起红色方块', '怎么抓红色方块？', '不要抓黄色方块',
    '物体已经取下', '请停止任务', '分类绿色方块',
])
def test_any_control_related_text_still_requires_strict_routing(text):
    # Exact stop/release phrases may use their established direct shortcuts;
    # no phrase in this set may become ordinary chat or vision.
    route = safe_local_non_action_route(text)
    assert route is None or route.route in {
        'stop_action', 'stop_playback', 'release_ack'}


AUTO_STACK_REPLY = (
    '准备自动码垛：按红、黄、蓝、绿从上到下的顺序堆叠桌面上现有的方块。')


@pytest.mark.parametrize('text', [
    '自动码垛', '自动堆叠', '自动堆放', '自动码放',
    '一键码垛', '一键堆放', '全自动堆叠',
    '帮我自动码垛', '自动把它们码起来',
])
def test_auto_stack_phrases_take_the_safe_local_route(text):
    route = safe_local_action_route(text)

    assert route is not None
    assert route.route == 'action'
    assert route.task == 'pick_auto_stack'
    assert route.color is None
    assert route.zone is None
    assert route.support_color is None
    assert route.reply == AUTO_STACK_REPLY


@pytest.mark.parametrize('text', ['自动码垛', '自动堆叠', '自动堆放', '自动码放'])
def test_auto_stack_commands_are_never_fast_routed_to_chat(text):
    assert safe_local_non_action_route(text) is None


@pytest.mark.parametrize('text', [
    '不要自动堆放', '什么是自动堆放', '自动堆放是什么',
])
def test_auto_stack_alias_does_not_turn_negation_or_questions_into_motion(text):
    assert safe_local_action_route(text) is None


def test_auto_stack_alias_with_explicit_color_does_not_dispatch_whole_tower():
    text = '把红色方块自动堆放'
    assert safe_local_action_route(text) is None
    with pytest.raises(IntentProtocolError) as caught:
        classify_text(
            text, lambda _prompt: encoded('action', 'pick_auto_stack'))
    assert caught.value.code == 'auto_stack_with_colors'


@pytest.mark.parametrize('raw', [
    encoded('action', 'pick_auto_stack', 'red'),
    encoded('action', 'pick_auto_stack', zone='d'),
    encoded('action', 'pick_auto_stack', support_color='blue'),
])
def test_parser_rejects_parameters_on_pick_auto_stack(raw):
    with pytest.raises(IntentProtocolError) as caught:
        parse_dialogue_intent(raw)
    assert caught.value.code == 'auto_stack_parameters_not_allowed'


def test_model_auto_stack_route_is_accepted_for_colourless_text():
    raw = encoded('action', 'pick_auto_stack', reply='准备自动码垛。')

    intent, _ = classify_text('自动堆叠', lambda _prompt: raw)

    assert intent.route == 'action'
    assert intent.task == 'pick_auto_stack'
    assert intent.color is None
    assert intent.zone is None
    assert intent.support_color is None


def test_model_auto_stack_route_is_rejected_when_text_names_a_colour():
    raw = encoded('action', 'pick_auto_stack', reply='准备自动码垛。')

    with pytest.raises(IntentProtocolError) as caught:
        classify_text('把红色方块自动码垛', lambda _prompt: raw)

    assert caught.value.code == 'auto_stack_with_colors'


@pytest.mark.parametrize('route,task,color,code', [
    ('action', 'pick_hold', 'red', 'auto_stack_task_required'),
    ('action', 'pick_sort', 'red', 'auto_stack_task_required'),
    ('chat', None, None, 'explicit_action_not_routed'),
])
def test_auto_stack_text_cannot_be_routed_to_another_task(
        route, task, color, code):
    raw = encoded(route, task, color, reply='好的。')

    with pytest.raises(IntentProtocolError) as caught:
        classify_text('自动码垛', lambda _prompt: raw)

    assert caught.value.code == code


def test_model_cannot_invent_auto_stack_without_the_automatic_marker():
    raw = encoded('action', 'pick_auto_stack', reply='准备自动码垛。')

    with pytest.raises(IntentProtocolError) as caught:
        classify_text('把红色方块放到蓝色方块上面', lambda _prompt: raw)

    assert caught.value.code in {'auto_stack_with_colors', 'auto_stack_without_cue'}


def test_auto_marker_with_explicit_colours_still_means_pick_stack():
    route = safe_local_action_route('自动把红色方块放到蓝色方块上面')
    assert route is not None
    assert route.task == 'pick_stack'
    assert (route.color, route.support_color) == ('red', 'blue')

    raw = encoded(
        'action', 'pick_stack', 'red', support_color='blue',
        reply='准备码放。')
    intent, _ = classify_text('把红色方块放到蓝色方块上面', lambda _prompt: raw)
    assert intent.task == 'pick_stack'
    assert intent.color == 'red'
    assert intent.support_color == 'blue'


@pytest.mark.parametrize('text', [
    '把红色方块放到蓝色方块上面',
    '把红色方块码到蓝色方块上面',
])
def test_bare_stack_words_never_become_auto_stack(text):
    route = safe_local_action_route(text)
    assert route is not None
    assert route.task == 'pick_stack'
