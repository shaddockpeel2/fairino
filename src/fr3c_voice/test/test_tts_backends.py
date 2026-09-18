"""P5 text splitter tests: pure logic, no numpy and no sherpa-onnx needed."""
import pytest

from fr3c_voice.tts_backends import FirstSegmentSplitter, SegmentPolicy


def collect(splitter, chunks):
    segments = []
    for chunk in chunks:
        segments.extend(splitter.feed(chunk))
    segments.extend(splitter.flush())
    return segments


def test_policy_rejects_inconsistent_limits():
    with pytest.raises(ValueError):
        SegmentPolicy(first_min_characters=0)
    with pytest.raises(ValueError):
        SegmentPolicy(first_min_characters=10, first_max_characters=5)
    with pytest.raises(ValueError):
        SegmentPolicy(first_max_characters=50, segment_max_characters=20)
    with pytest.raises(ValueError):
        SegmentPolicy(segment_target_characters=10, segment_max_characters=5)


def test_first_segment_waits_for_the_minimum():
    splitter = FirstSegmentSplitter(SegmentPolicy(first_min_characters=4, first_max_characters=8))
    assert splitter.feed('好的') == ()
    assert splitter.buffered_characters == 2
    assert splitter.feed('收到') == ()
    # The first segment is strict: below the cap it waits for a sentence end
    # rather than cutting a half phrase, and it is not lost either.
    assert splitter.feed('了吗') == ()
    assert splitter.buffered_characters == 6
    assert splitter.feed('？') == ('好的收到了吗？',)
    assert splitter.emitted_segments == 1


def test_first_segment_cuts_at_strong_punctuation_then_later_segments_grow():
    policy = SegmentPolicy(first_min_characters=4, first_max_characters=10,
                           segment_target_characters=12, segment_max_characters=20)
    splitter = FirstSegmentSplitter(policy)
    segments = collect(splitter, '好的，我这就去办。第二句话稍微长一点点，需要更多上下文。')
    # The first segment stops at the sentence end, not at the hard maximum.
    assert segments[0] == '好的，我这就去办。'
    assert all(len(segment) <= 20 for segment in segments)
    assert ''.join(segments).replace(' ', '') == '好的，我这就去办。第二句话稍微长一点点，需要更多上下文。'


def test_later_segment_cuts_at_clause_punctuation_or_target():
    """Non-first segments cut at a clause break, or at the target length."""
    splitter = FirstSegmentSplitter(SegmentPolicy(first_min_characters=2, first_max_characters=4,
                                                 segment_target_characters=12,
                                                 segment_max_characters=20))
    segments = collect(splitter, '你好。接下来我想请你把桌面上的红色方块，拿起来放到分类区里面去')
    assert segments[0] == '你好。'
    # A clause mark that arrives in time is kept with its clause.
    assert segments[1] == '接下来我想请你把桌面上的'
    assert segments[2] == '红色方块，拿起来放到分类'
    assert segments[3] == '区里面去'

    # Structural invariants that must hold for any text.
    for text in ('你好。接下来我想请你把桌面上的红色方块，拿起来放到分类区里面去',
                 '甲' * 40,
                 '第一句。第二句稍微长一点，第三句再长一些而且完全没有任何停顿'):
        split = FirstSegmentSplitter(SegmentPolicy(first_min_characters=2, first_max_characters=4,
                                                   segment_target_characters=12,
                                                   segment_max_characters=20))
        chunks = collect(split, text)
        assert ''.join(chunks) == text
        assert all(len(chunk) <= 20 for chunk in chunks)
        assert all(chunk.strip() for chunk in chunks)
        assert not any(chunk[0] in '，、,;：:。！？；' for chunk in chunks)


def test_soft_cut_keeps_a_clause_mark_that_arrives_on_the_boundary():
    """The lookahead exists so a boundary clause mark is not orphaned."""
    splitter = FirstSegmentSplitter(SegmentPolicy(first_min_characters=2, first_max_characters=4,
                                                 segment_target_characters=6,
                                                 segment_max_characters=20))
    splitter.feed('你好。')
    splitter.feed('一二三四五')     # exactly the target length, punctuation not yet seen
    assert splitter.feed('，') == ('一二三四五，',)

def test_first_segment_hard_cuts_when_no_punctuation_arrives():
    splitter = FirstSegmentSplitter(SegmentPolicy(first_min_characters=4, first_max_characters=6))
    segments = collect(splitter, '一二三四五六七八九十')
    assert segments[0] == '一二三四五六'
    assert ''.join(segments) == '一二三四五六七八九十'


def test_first_segment_is_released_as_soon_as_it_is_speakable():
    """The first segment cuts at the first sentence end, not later."""
    splitter = FirstSegmentSplitter(SegmentPolicy(first_min_characters=4, first_max_characters=16,
                                                 segment_target_characters=6,
                                                 segment_max_characters=16))
    assert splitter.feed('好的我') == ()
    assert splitter.feed('知道了') == ()
    assert splitter.feed('。') == ('好的我知道了。',)


def test_first_segment_releases_at_the_cap_when_no_punctuation_arrives():
    """An unpunctuated stream must not stall: the first segment goes at max."""
    splitter = FirstSegmentSplitter(SegmentPolicy(first_min_characters=4, first_max_characters=8,
                                                 segment_target_characters=12,
                                                 segment_max_characters=20))
    assert splitter.feed('好的我这就去') == ()
    assert splitter.feed('办事情') == ('好的我这就去办事',)


def test_streaming_delivery_is_incremental():
    """The first segment must be available before the whole text has arrived."""
    splitter = FirstSegmentSplitter(SegmentPolicy(first_min_characters=4, first_max_characters=8,
                                                 segment_target_characters=6,
                                                 segment_max_characters=12))
    assert splitter.feed('好的我') == ()
    assert splitter.feed('知道了') == ()
    assert splitter.feed('。') == ('好的我知道了。',)
    # The rest of the answer has not even been produced yet, which is the point.
    assert splitter.buffered_characters == 0
    later = collect(splitter, '接下来还有一句话，再补一点点。')
    assert ''.join(later) == '接下来还有一句话，再补一点点。'


def test_flush_emits_the_tail_and_reset_clears_state():
    splitter = FirstSegmentSplitter()
    splitter.feed('好的')
    assert splitter.flush() == ('好的',)
    assert splitter.flush() == ()
    splitter.reset()
    assert splitter.buffered_characters == 0
    assert splitter.emitted_segments == 0


def test_finish_adds_one_terminal_mark_to_an_unpunctuated_tail():
    splitter = FirstSegmentSplitter()
    splitter.feed('这是流式回答的最后一段')
    assert splitter.finish() == ('这是流式回答的最后一段。',)

    punctuated = FirstSegmentSplitter()
    emitted = punctuated.feed('这句话已经结束！')
    assert emitted + punctuated.finish() == ('这句话已经结束！',)


def test_whitespace_only_segments_are_dropped():
    splitter = FirstSegmentSplitter(SegmentPolicy(first_min_characters=2, first_max_characters=4))
    segments = collect(splitter, '  。  ')
    assert all(segment.strip() for segment in segments)


def test_segments_never_exceed_the_hard_maximum():
    splitter = FirstSegmentSplitter(SegmentPolicy(first_min_characters=4, first_max_characters=6,
                                                 segment_target_characters=8,
                                                 segment_max_characters=12))
    segments = collect(splitter, '甲' * 100)
    assert len(segments) >= 8
    assert all(len(segment) <= 12 for segment in segments)
    assert ''.join(segments) == '甲' * 100
