"""Turn model output into speakable text.

The model is asked for plain sentences, but real output contains emoji,
markdown emphasis, tool-call JSON and stray symbols.  Phase 5 measured what the
engines do with them: Kokoro printed ``Unknown token: ❓`` and dropped the
character, melo printed one ``Ignore OOV`` per brace and quote.  Both waste time
and can drop the meaning of a sentence, so the cleaning happens here, once,
before segmentation.

Two different jobs live in this module and are kept apart deliberately:

* :func:`clean_for_speech` rewrites text for the synthesiser
* :func:`strip_for_intent` only decides whether a chunk is speakable at all,
  which is what stops an intent JSON from being read out loud
"""
from __future__ import annotations

import re
import unicodedata

# Markdown that should be removed rather than pronounced.
CODE_FENCE = re.compile(r'```.*?```', re.DOTALL)
INLINE_CODE = re.compile(r'`([^`]*)`')
EMPHASIS = re.compile(r'(\*\*|__|\*|~~)')
HEADING = re.compile(r'^\s{0,3}#{1,6}\s*', re.MULTILINE)
LIST_MARKER = re.compile(r'^\s{0,3}[-*+]\s+', re.MULTILINE)
LINK = re.compile(r'\[([^\]]*)\]\([^)]*\)')
HTML_TAG = re.compile(r'</?[A-Za-z][A-Za-z0-9]*\s*/?>')

# A structured payload must never be read aloud. This is a cheap syntactic
# guard, not a parser: the pipeline only needs to refuse, not to interpret.
JSON_LIKE = re.compile(r'^\s*[{\[].*[}\]]\s*$', re.DOTALL)

# Latin letters, digits, CJK, and the punctuation a TTS front-end understands.
SPEAKABLE = re.compile(
    r'[\u4e00-\u9fff\u3400-\u4dbf0-9A-Za-z'
    r'\u3000-\u303f\uff00-\uffef'      # CJK punctuation and fullwidth forms
    r'，。！？；：、,.!?;:()（）\[\]【】"\'“”‘’—…·%°℃$/+\-]'
)

# Symbols that are never spoken and would otherwise reach the phonemiser.
SILENT_SYMBOLS = '✅❓❗⚠️🎉👍🙂😀😊🤖✨🔥💡📌📍🔴🟡🔵🟢⚪⚫⏹▶️⏸'

WHITESPACE = re.compile(r'[ \t\u00a0\u3000]+')
BLANK_LINES = re.compile(r'\n{3,}')


def strip_for_intent(text: str) -> str:
    """Return the text that would actually be spoken, for emptiness checks.

    Used to decide whether the model produced an answer or only a structured
    payload; the caller must not speak the payload either way.
    """
    return clean_for_speech(text)


def is_speakable(text: str, minimum_characters: int = 1) -> bool:
    """True when ``text`` contains something worth saying out loud."""
    cleaned = clean_for_speech(text)
    effective = sum(1 for character in cleaned if character.isalnum()
                    or '\u4e00' <= character <= '\u9fff')
    return effective >= minimum_characters


def looks_structured(text: str) -> bool:
    """True when the whole chunk is a JSON-like payload rather than prose."""
    return bool(JSON_LIKE.match(text.strip()))


def clean_for_speech(text: str, *, ensure_terminal: bool = True) -> str:
    """Normalise model output into text a Chinese TTS front-end can read.

    Kept conservative: it removes things that cannot be pronounced and repairs
    spacing and punctuation, but it never rewrites wording, numbers or units.

    ``ensure_terminal`` must be false for an incremental model token.  Adding a
    full stop to every token turns ``我`` + ``可以`` into ``我。可以。`` and
    destroys sentence prosody.  Complete strings keep the historical default.
    """
    if not text:
        return ''
    result = text.replace('\r\n', '\n').replace('\r', '\n')
    result = CODE_FENCE.sub(' ', result)
    result = INLINE_CODE.sub(r'\1', result)
    result = LINK.sub(r'\1', result)
    result = HTML_TAG.sub(' ', result)
    result = HEADING.sub('', result)
    result = LIST_MARKER.sub('', result)
    result = EMPHASIS.sub('', result)

    kept: list[str] = []
    for character in result:
        if character in SILENT_SYMBOLS:
            continue
        category = unicodedata.category(character)
        if category in ('So', 'Sk', 'Cs', 'Co', 'Cn'):
            # Emoji, variation selectors, surrogates and unassigned code points
            # have no pronunciation and become an unknown token in the engine.
            continue
        if character == '\ufe0f':  # variation selector-16 survives some fonts
            continue
        if category == 'Cc' and character not in '\n\t':
            continue
        if character == '\t':
            character = ' '
        kept.append(character)
    result = ''.join(kept)
    result = WHITESPACE.sub(' ', result)
    result = BLANK_LINES.sub('\n\n', result)

    # Collapse punctuation runs such as "……" or "!!!" that stretch prosody.
    result = re.sub(r'([，。！？；：、,.!?;:])\1{2,}', r'\1', result)
    # A space *between two CJK characters* is not pronounced and only adds
    # pauses. Spaces around Latin text ("与 代码") are kept: removing them makes
    # the sentence harder to read and the front-end treats them as boundaries.
    result = re.sub(r'(?<=[\u4e00-\u9fff]) (?=[\u4e00-\u9fff])', '', result)
    # Ensure the final clause has terminal punctuation so the segmenter can
    # close it; without this the tail waits for a boundary that never comes.
    stripped = result.strip()
    if ensure_terminal and stripped and stripped[-1] not in '。！？；.!?;':
        stripped += '。'
    return stripped
