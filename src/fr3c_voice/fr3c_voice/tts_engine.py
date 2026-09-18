"""Build a sherpa-onnx TTS engine from a P5 candidate registry entry.

Kept separate from :mod:`fr3c_voice.tts_backends` so the engine construction
(the part that must match the P5 measurements exactly) has one implementation
shared by the benchmark, the acoustic tooling and the runtime adapter.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any


def build_engine(candidate_name: str, candidate: dict, asset_root: Path | str,
                 threads: int, provider: str = 'cpu') -> tuple[Any, int, int]:
    """Return ``(engine, sample_rate, num_speakers)`` for one candidate.

    ``asset_root`` is the directory that holds this candidate's files, not the
    root of all candidates.
    """
    import sherpa_onnx

    root = Path(asset_root)
    if not root.is_dir():
        raise FileNotFoundError(f'{candidate_name}: asset directory not found: {root}')
    missing = [asset['name'] for asset in candidate['assets']
               if not (root / asset['name']).is_file()]
    if missing:
        raise FileNotFoundError(
            f'{candidate_name}: missing assets {missing}; run scripts/voice/p5_fetch_models.py')

    def path(name: str) -> str:
        return os.fspath(root / name)

    family = candidate['family']
    dict_dir = os.fspath(root / candidate['dict_dir']) if candidate.get('dict_dir') else ''
    model_kwargs: dict[str, Any] = {}

    if family == 'kokoro':
        # Kokoro needs espeak-ng data plus lexicons; it has no jieba dictionary.
        model_kwargs['kokoro'] = sherpa_onnx.OfflineTtsKokoroModelConfig(
            model=path('model.onnx'),
            voices=path('voices.bin'),
            tokens=path('tokens.txt'),
            data_dir=path(candidate.get('espeak_data_dir', 'espeak-ng-data')),
            lexicon=','.join(path(name) for name in ('lexicon-us-en.txt', 'lexicon-zh.txt')
                             if (root / name).is_file()),
        )
        model_name = 'model.onnx'
    elif family == 'matcha':
        model_name = 'model-steps-3.onnx'
        model_kwargs['matcha'] = sherpa_onnx.OfflineTtsMatchaModelConfig(
            acoustic_model=path(model_name),
            vocoder=path(candidate['vocoder']['name']),
            tokens=path('tokens.txt'),
            lexicon=path('lexicon.txt'),
            dict_dir=dict_dir,
        )
    elif family == 'vits':
        model_name = next((asset['name'] for asset in candidate['assets']
                           if asset['name'].endswith('.onnx') and 'int8' not in asset['name']),
                          'model.onnx')
        model_kwargs['vits'] = sherpa_onnx.OfflineTtsVitsModelConfig(
            model=path(model_name),
            tokens=path('tokens.txt'),
            lexicon=path('lexicon.txt') if (root / 'lexicon.txt').is_file() else '',
            dict_dir=dict_dir,
        )
    else:
        raise ValueError(f'{candidate_name}: unsupported family {family!r}')

    model_config = sherpa_onnx.OfflineTtsModelConfig(
        **model_kwargs, num_threads=threads, provider=provider, debug=False)
    rules = [path(name) for name in candidate.get('rule_fsts', []) if (root / name).is_file()]
    config = sherpa_onnx.OfflineTtsConfig(
        model=model_config, rule_fsts=','.join(rules), max_num_sentences=1)
    if not config.validate():
        raise RuntimeError(f'{candidate_name}: OfflineTtsConfig failed validation')
    engine = sherpa_onnx.OfflineTts(config)
    return engine, int(engine.sample_rate), int(engine.num_speakers)
