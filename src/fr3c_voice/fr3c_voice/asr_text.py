"""Conservative post-processing for robot-domain ASR final text.

Only terminology with an unambiguous correction is normalized.  In
particular, colours and missing parameters are never guessed because either
could change the physical action selected by the operator.
"""

from __future__ import annotations

import re
import unicodedata


_ROBOT_NOUN = re.compile(r'方块|物体|红色|黄色|蓝色|绿色')
_ROBOT_ACTION = re.compile(
    r'抓|拿|取|拾|捡|夹|放|分类|码|堆|叠|停止|取下|拿走|释放|'
    r'转起|折取|只要[娶取]|爪取')


def normalize_robot_asr_text(text: str) -> str:
    """Fix high-confidence command homophones without inventing parameters."""
    normalized = unicodedata.normalize('NFKC', str(text or '')).strip()
    if not normalized:
        return ''
    if not _ROBOT_NOUN.search(normalized) or not _ROBOT_ACTION.search(normalized):
        return normalized

    # Erhua does not carry command meaning and can interfere with exact phrase
    # matching in downstream action routing.
    normalized = normalized.replace('方块儿', '方块')

    # These grasp variants are all present in the project's recorded ASR
    # evidence.  Keep the rule start-anchored and require a robot noun above so
    # ordinary uses of “早起/抓去/转起” in conversation are not rewritten.
    normalized = re.sub(
        r'^(?P<prefix>(?:(?:你好|请你|请|麻烦你|麻烦|帮我|现在|立刻|马上)[，, ]*)*)'
        r'(?:转起|折取|抓去|抓娶|爪取|早起|只要娶|只要取)',
        r'\g<prefix>抓取', normalized, count=1)

    # The same evidence contains “打红色方块放到分类区” for an intended 把字句.
    # A following placement cue is required; “打红色方块” by itself is left
    # untouched because it could be a real request to hit an object.
    normalized = re.sub(
        r'^(?P<prefix>(?:(?:请你|请|麻烦你|麻烦|帮我)[，, ]*)*)'
        r'打(?=(?:红色|黄色|蓝色|绿色)方块.{0,10}(?:放|分类|对应))',
        r'\g<prefix>把', normalized, count=1)

    # Observed final-ASR variants include “放置去”, “放置居” and
    # “放治区”.  Correct only the complete destination term; never infer a
    # colour, a lettered region, or a missing object.
    normalized = re.sub(r'放[置治制][区去居取曲]', '放置区', normalized)
    normalized = re.sub(r'分类[去居取曲]', '分类区', normalized)
    normalized = re.sub(r'对应[去居取曲]', '对应区', normalized)
    normalized = re.sub(
        r'(?i)((?:放到|放入|放进|放置到)\s*)([a-z][0-9]{0,2})[去居取曲]',
        r'\1\2区', normalized)
    return normalized
