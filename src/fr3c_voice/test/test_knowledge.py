from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
import pytest

from fr3c_voice.controller import ControllerState
from fr3c_voice.coordinator import NpuOwner
from fr3c_voice.events import ChatResult
from fr3c_voice.knowledge import (
    BgeTokenizer,
    KnowledgeRuntime,
    KnowledgeSettings,
    KnowledgeStore,
    build_knowledge_prompt,
    load_knowledge_settings,
)
from fr3c_voice.model_client import ModelServiceClient
from fr3c_voice.service import QwenServiceCore


WORKSPACE = Path(__file__).resolve().parents[3]


def _bundle(tmp_path: Path) -> Path:
    index = tmp_path / "index"
    index.mkdir()
    database = index / "kb.sqlite"
    connection = sqlite3.connect(database)
    connection.executescript("""
        CREATE TABLE chunks (
            vector_row INTEGER PRIMARY KEY, chunk_id TEXT NOT NULL UNIQUE,
            source TEXT NOT NULL, title TEXT NOT NULL, section TEXT NOT NULL,
            page INTEGER, ordinal INTEGER NOT NULL, text TEXT NOT NULL,
            token_count INTEGER NOT NULL, content_sha256 TEXT NOT NULL);
    """)
    rows = [
        (0, "company", "company.txt", "苏开鸿科技（无锡）有限公司", "", None, 0,
         "苏开鸿科技（无锡）有限公司成立于2024年07月17日。", 20, "sha-company"),
        (1, "stage", "verified/p14.md", "P14 统一 NPU 资源协调完成记录", "结论", None, 0,
         "P14 完成了统一NPU调度。", 15, "sha-stage"),
    ]
    connection.executemany(
        "INSERT INTO chunks VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", rows)
    connection.commit()
    connection.close()
    vectors = np.zeros((2, 512), dtype=np.float32)
    vectors[0, 0] = 1.0
    vectors[1, 1] = 1.0
    np.save(index / "embeddings.npy", vectors)
    files = {}
    for name in ("kb.sqlite", "embeddings.npy"):
        files[name] = hashlib.sha256((index / name).read_bytes()).hexdigest()
    (index / "manifest.json").write_text(json.dumps({
        "index_version": 1, "knowledge_version": "fixture-v1",
        "files": files,
    }), encoding="utf-8")
    return index


class FakeEmbedder:
    loaded = True

    def encode_query(self, text):
        vector = np.zeros(512, dtype=np.float32)
        vector[1 if "P14" in text else 0] = 1.0
        return vector

    def close(self):
        pass


def _runtime(tmp_path: Path) -> KnowledgeRuntime:
    model = tmp_path / "tokenizer"
    model.mkdir()
    (model / "vocab.txt").write_text(
        "\n".join(["[PAD]", "[UNK]", "[CLS]", "[SEP]", "苏", "开", "鸿"]),
        encoding="utf-8")
    settings = KnowledgeSettings(
        enabled=True, root=tmp_path, index_dir=_bundle(tmp_path),
        tokenizer_dir=model, rknn_model_path=tmp_path / "model.rknn",
        top_k=2, minimum_score=0.45, maximum_context_chars=1000)
    store = KnowledgeStore(settings.index_dir)
    tokenizer = BgeTokenizer(model)
    return KnowledgeRuntime(settings, store, tokenizer, FakeEmbedder())


def test_shared_config_reuses_the_previous_bundle_without_copying_it():
    settings = load_knowledge_settings(WORKSPACE / "config/voice/qwen3vl.yaml")
    assert settings.index_dir == Path(
        "/home/industio/DofbotPro_JetsonNano_0106/dofbot/knowledge/builds/current")
    assert settings.top_k == 4
    assert settings.minimum_score == 0.45


def test_router_skips_small_talk_and_routes_named_sources_and_followups(tmp_path):
    runtime = _runtime(tmp_path)
    try:
        assert runtime.wants_retrieval("你好") is False
        assert runtime.wants_retrieval("什么是人工智能") is False
        assert runtime.wants_retrieval("苏开鸿科技什么时候成立？") is True
        result = runtime.retrieve("苏开鸿科技什么时候成立？")
        assert result.reliable
        assert result.hits[0].chunk.source == "company.txt"
        assert runtime.wants_retrieval("再详细一点") is True
        stage = runtime.retrieve("P14完成了什么？")
        assert stage.hits[0].chunk.source == "verified/p14.md"
    finally:
        runtime.close()


def test_prompt_treats_documents_as_read_only_data_and_metadata_is_structured(tmp_path):
    runtime = _runtime(tmp_path)
    try:
        result = runtime.retrieve("苏开鸿科技什么时候成立？")
        prompt = build_knowledge_prompt(result.question, result)
        assert "不可信的只读数据" in prompt
        assert "不能触发机械臂动作" in prompt
        assert "[来源1]" in prompt
        metadata = result.metadata()
        assert metadata["mode"] == "knowledge"
        assert metadata["version"] == "fixture-v1"
        assert metadata["sources"][0]["source"] == "company.txt"
    finally:
        runtime.close()


def test_corrupt_bundle_is_rejected_without_mutating_it(tmp_path):
    index = _bundle(tmp_path)
    (index / "embeddings.npy").write_bytes(b"corrupt")
    with pytest.raises(RuntimeError, match="checksum mismatch"):
        KnowledgeStore(index)


class _Trace:
    def write(self, *_args, **_kwargs):
        pass


class _Controller:
    state = ControllerState.READY
    worker_pid = 123
    active_request_id = ""
    last_error = ""
    trace = _Trace()

    def health(self):
        return True

    def reset_context(self):
        pass

    def chat_text(self, prompt, request_id="", on_token=None):
        if on_token:
            on_token("回答")
        self.last_prompt = prompt
        return ChatResult(request_id, "回答", 0.1, 0.2)


class _Coordinator:
    fault = ""
    owner = ""

    def __init__(self):
        self.owners = []

    def run(self, owner, _request_id, operation):
        self.owners.append(owner)
        return operation()


def test_service_uses_one_rag_qwen_lease_and_skips_small_talk(tmp_path):
    runtime = _runtime(tmp_path)
    controller = _Controller()
    coordinator = _Coordinator()
    core = QwenServiceCore(controller, coordinator, True, runtime)
    try:
        chat, metadata = core.chat({
            "op": "chat_auto", "request_id": "small", "question": "你好",
            "prompt": "普通聊天提示", "reset_before": True,
        }, lambda _token: None)
        assert chat.text == "回答"
        assert metadata["mode"] == "skipped"
        assert coordinator.owners[-1] == NpuOwner.QWEN_TEXT
        assert controller.last_prompt == "普通聊天提示"

        _chat, metadata = core.chat({
            "op": "chat_auto", "request_id": "knowledge",
            "question": "苏开鸿科技什么时候成立？", "prompt": "fallback",
            "reset_before": True,
        }, lambda _token: None)
        assert metadata["mode"] == "knowledge"
        assert coordinator.owners[-1] == NpuOwner.RAG_QWEN_TEXT
        assert "[来源1]" in controller.last_prompt
    finally:
        runtime.close()


def test_model_client_requests_auto_knowledge_and_returns_metadata(monkeypatch):
    client = ModelServiceClient('/unused.sock')
    captured = {}

    def request(payload, on_event=None):
        captured.update(payload)
        if on_event:
            on_event({'event': 'token', 'text': '答'})
        return [{
            'event': 'result', 'text': '答案',
            'knowledge': {'mode': 'knowledge', 'sources': []},
        }]

    monkeypatch.setattr(client, '_request', request)
    tokens = []
    metadata = {}
    answer = client.generate(
        'fallback', knowledge_question='本项目资料是什么？',
        on_token=tokens.append, on_knowledge=metadata.update)
    assert answer == '答案'
    assert captured['op'] == 'chat_auto'
    assert captured['question'] == '本项目资料是什么？'
    assert tokens == ['答']
    assert metadata['mode'] == 'knowledge'


def test_retrieval_failure_disables_only_knowledge_and_keeps_chat():
    class BrokenKnowledge:
        available = True
        last_error = ''

        def wants_retrieval(self, _question):
            return True

        def retrieve(self, _question):
            raise RuntimeError('simulated index failure')

        def disable(self, error):
            self.available = False
            self.last_error = str(error)

    controller = _Controller()
    coordinator = _Coordinator()
    knowledge = BrokenKnowledge()
    core = QwenServiceCore(controller, coordinator, True, knowledge)
    chat, metadata = core.chat({
        'op': 'chat_auto', 'request_id': 'degrade',
        'question': '请查本项目资料', 'prompt': '普通聊天回退提示',
    }, lambda _token: None)
    assert chat.text == '回答'
    assert controller.last_prompt == '普通聊天回退提示'
    assert metadata['mode'] == 'unavailable'
    assert metadata['routed'] is True
    assert knowledge.available is False
    assert coordinator.owners[-1] == NpuOwner.RAG_QWEN_TEXT
