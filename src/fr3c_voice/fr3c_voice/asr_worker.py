#!/usr/bin/env python3
"""Isolated RKNN Sherpa C-API worker for raw PCM streaming ASR.

stdin accepts headerless 16-kHz mono PCM16-LE. stdout emits one JSON object per
line. Native Sherpa/RKNN logs remain on stderr so the parent can parse stdout
without scraping human-readable recognizer output.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import sys
import time
from pathlib import Path

import numpy as np


def emit(event: str, **payload: object) -> None:
    print(
        json.dumps({"event": event, **payload}, ensure_ascii=False, separators=(",", ":")),
        flush=True,
    )


class OnlineTransducerModelConfig(ctypes.Structure):
    _fields_ = [("encoder", ctypes.c_char_p), ("decoder", ctypes.c_char_p), ("joiner", ctypes.c_char_p)]


class OnlineParaformerModelConfig(ctypes.Structure):
    _fields_ = [("encoder", ctypes.c_char_p), ("decoder", ctypes.c_char_p)]


class OnlineZipformer2CtcModelConfig(ctypes.Structure):
    _fields_ = [("model", ctypes.c_char_p)]


class OnlineNemoCtcModelConfig(ctypes.Structure):
    _fields_ = [("model", ctypes.c_char_p)]


class OnlineToneCtcModelConfig(ctypes.Structure):
    _fields_ = [("model", ctypes.c_char_p)]


class OnlineModelConfig(ctypes.Structure):
    _fields_ = [
        ("transducer", OnlineTransducerModelConfig),
        ("paraformer", OnlineParaformerModelConfig),
        ("zipformer2_ctc", OnlineZipformer2CtcModelConfig),
        ("tokens", ctypes.c_char_p),
        ("num_threads", ctypes.c_int32),
        ("provider", ctypes.c_char_p),
        ("debug", ctypes.c_int32),
        ("model_type", ctypes.c_char_p),
        ("modeling_unit", ctypes.c_char_p),
        ("bpe_vocab", ctypes.c_char_p),
        ("tokens_buf", ctypes.c_char_p),
        ("tokens_buf_size", ctypes.c_int32),
        ("nemo_ctc", OnlineNemoCtcModelConfig),
        ("t_one_ctc", OnlineToneCtcModelConfig),
    ]


class FeatureConfig(ctypes.Structure):
    _fields_ = [("sample_rate", ctypes.c_int32), ("feature_dim", ctypes.c_int32)]


class OnlineCtcFstDecoderConfig(ctypes.Structure):
    _fields_ = [("graph", ctypes.c_char_p), ("max_active", ctypes.c_int32)]


class HomophoneReplacerConfig(ctypes.Structure):
    _fields_ = [("dict_dir", ctypes.c_char_p), ("lexicon", ctypes.c_char_p), ("rule_fsts", ctypes.c_char_p)]


class OnlineRecognizerConfig(ctypes.Structure):
    _fields_ = [
        ("feat_config", FeatureConfig),
        ("model_config", OnlineModelConfig),
        ("decoding_method", ctypes.c_char_p),
        ("max_active_paths", ctypes.c_int32),
        ("enable_endpoint", ctypes.c_int32),
        ("rule1_min_trailing_silence", ctypes.c_float),
        ("rule2_min_trailing_silence", ctypes.c_float),
        ("rule3_min_utterance_length", ctypes.c_float),
        ("hotwords_file", ctypes.c_char_p),
        ("hotwords_score", ctypes.c_float),
        ("ctc_fst_decoder_config", OnlineCtcFstDecoderConfig),
        ("rule_fsts", ctypes.c_char_p),
        ("rule_fars", ctypes.c_char_p),
        ("blank_penalty", ctypes.c_float),
        ("hotwords_buf", ctypes.c_char_p),
        ("hotwords_buf_size", ctypes.c_int32),
        ("hr", HomophoneReplacerConfig),
    ]


def bind_library(path: Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
    pointer = ctypes.c_void_p
    lib.SherpaOnnxCreateOnlineRecognizer.argtypes = [ctypes.POINTER(OnlineRecognizerConfig)]
    lib.SherpaOnnxCreateOnlineRecognizer.restype = pointer
    lib.SherpaOnnxDestroyOnlineRecognizer.argtypes = [pointer]
    lib.SherpaOnnxCreateOnlineStream.argtypes = [pointer]
    lib.SherpaOnnxCreateOnlineStream.restype = pointer
    lib.SherpaOnnxDestroyOnlineStream.argtypes = [pointer]
    lib.SherpaOnnxOnlineStreamAcceptWaveform.argtypes = [
        pointer, ctypes.c_int32, ctypes.POINTER(ctypes.c_float), ctypes.c_int32,
    ]
    lib.SherpaOnnxIsOnlineStreamReady.argtypes = [pointer, pointer]
    lib.SherpaOnnxIsOnlineStreamReady.restype = ctypes.c_int32
    lib.SherpaOnnxDecodeOnlineStream.argtypes = [pointer, pointer]
    lib.SherpaOnnxGetOnlineStreamResultAsJson.argtypes = [pointer, pointer]
    lib.SherpaOnnxGetOnlineStreamResultAsJson.restype = pointer
    lib.SherpaOnnxDestroyOnlineStreamResultJson.argtypes = [pointer]
    lib.SherpaOnnxOnlineStreamIsEndpoint.argtypes = [pointer, pointer]
    lib.SherpaOnnxOnlineStreamIsEndpoint.restype = ctypes.c_int32
    lib.SherpaOnnxOnlineStreamInputFinished.argtypes = [pointer]
    return lib


def result_text(lib: ctypes.CDLL, recognizer: int, stream: int) -> str:
    address = lib.SherpaOnnxGetOnlineStreamResultAsJson(recognizer, stream)
    if not address:
        return ""
    try:
        payload = json.loads(ctypes.string_at(address).decode("utf-8", errors="replace"))
        return str(payload.get("text", "")).strip() if isinstance(payload, dict) else ""
    finally:
        lib.SherpaOnnxDestroyOnlineStreamResultJson(address)


def run(args: argparse.Namespace) -> int:
    if args.hotwords_file:
        print('RKNN ASR 不支持热词偏置，已忽略 --hotwords-file',
              file=sys.stderr, flush=True)
    model = Path(args.model_dir)
    keepalive = {
        "encoder": str(model / "encoder.rknn").encode(),
        "decoder": str(model / "decoder.rknn").encode(),
        "joiner": str(model / "joiner.rknn").encode(),
        "tokens": str(model / "tokens.txt").encode(),
        "provider": b"rknn",
        "model_type": b"zipformer",
        "modeling_unit": b"cjkchar",
        "decoding": args.decoding_method.encode(),
    }
    config = OnlineRecognizerConfig()
    config.feat_config = FeatureConfig(args.sample_rate, 80)
    config.model_config.transducer = OnlineTransducerModelConfig(
        keepalive["encoder"], keepalive["decoder"], keepalive["joiner"],
    )
    config.model_config.tokens = keepalive["tokens"]
    config.model_config.num_threads = args.num_threads
    config.model_config.provider = keepalive["provider"]
    config.model_config.model_type = keepalive["model_type"]
    config.model_config.modeling_unit = keepalive["modeling_unit"]
    config.decoding_method = keepalive["decoding"]
    config.max_active_paths = 4
    config.enable_endpoint = 1
    config.rule1_min_trailing_silence = args.empty_endpoint_seconds
    config.rule2_min_trailing_silence = args.speech_endpoint_seconds
    config.rule3_min_utterance_length = args.maximum_utterance_seconds

    lib = bind_library(Path(args.c_api_library))
    started = time.monotonic()
    recognizer = lib.SherpaOnnxCreateOnlineRecognizer(ctypes.byref(config))
    if not recognizer:
        raise RuntimeError("SherpaOnnxCreateOnlineRecognizer returned null")
    stream = lib.SherpaOnnxCreateOnlineStream(recognizer)
    if not stream:
        lib.SherpaOnnxDestroyOnlineRecognizer(recognizer)
        raise RuntimeError("SherpaOnnxCreateOnlineStream returned null")
    emit("ready", load_seconds=time.monotonic() - started)

    last_text = ""
    final_reason = "input_finished"
    try:
        while True:
            raw = sys.stdin.buffer.read(args.chunk_samples * 2)
            if not raw:
                # Flush the final decoder window without waiting for real silence.
                tail = np.zeros(args.sample_rate // 2, dtype=np.float32)
                lib.SherpaOnnxOnlineStreamAcceptWaveform(
                    stream, args.sample_rate,
                    tail.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), tail.size)
                lib.SherpaOnnxOnlineStreamInputFinished(stream)
            else:
                if len(raw) % 2:
                    raw = raw[:-1]
                samples = np.frombuffer(raw, dtype="<i2").astype(np.float32)
                samples *= 1.0 / 32768.0
                lib.SherpaOnnxOnlineStreamAcceptWaveform(
                    stream,
                    args.sample_rate,
                    samples.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                    samples.size,
                )
            while lib.SherpaOnnxIsOnlineStreamReady(recognizer, stream):
                lib.SherpaOnnxDecodeOnlineStream(recognizer, stream)
            text = result_text(lib, recognizer, stream)
            endpoint = bool(lib.SherpaOnnxOnlineStreamIsEndpoint(recognizer, stream))
            if text != last_text:
                last_text = text
                emit("partial", text=text, endpoint=endpoint)
            if endpoint and text:
                final_reason = "sherpa_endpoint"
                break
            if not raw:
                break
        emit("final", text=last_text, reason=final_reason)
        return 0
    finally:
        lib.SherpaOnnxDestroyOnlineStream(stream)
        lib.SherpaOnnxDestroyOnlineRecognizer(recognizer)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--decoding-method", default="modified_beam_search")
    parser.add_argument("--hotwords-file", default="")
    parser.add_argument("--hotwords-score", type=float, default=2.0)
    parser.add_argument("--c-api-library", required=True)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--num-threads", type=int, default=-1)
    parser.add_argument("--chunk-samples", type=int, default=1600)
    parser.add_argument("--empty-endpoint-seconds", type=float, default=2.4)
    parser.add_argument("--speech-endpoint-seconds", type=float, default=1.1)
    parser.add_argument("--maximum-utterance-seconds", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    try:
        return run(parse_args())
    except Exception as error:
        emit("error", message=f"{type(error).__name__}: {error}")
        print(f"streaming ASR worker failed: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
