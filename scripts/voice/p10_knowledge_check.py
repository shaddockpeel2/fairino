#!/usr/bin/env python3
"""Validate the reused knowledge bundle or ask one live knowledge question."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "src/fr3c_voice"))

from fr3c_voice.knowledge import KnowledgeStore, load_knowledge_settings  # noqa: E402
from fr3c_voice.model_client import ModelServiceClient  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path,
        default=WORKSPACE / "config/voice/qwen3vl.yaml")
    parser.add_argument("--question", help="ask through the running Qwen service")
    parser.add_argument(
        "--socket", type=Path,
        default=WORKSPACE / "evidence/voice/P3/runtime/fr3c-qwen-service.sock")
    args = parser.parse_args()

    settings = load_knowledge_settings(args.config)
    store = KnowledgeStore(settings.index_dir)
    try:
        print(json.dumps({
            "index": "ok",
            "version": store.version,
            "chunks": store.chunk_count,
            "sources": len(store.source_titles),
            "path": str(store.index_dir),
        }, ensure_ascii=False))
    finally:
        store.close()
    if not args.question:
        return 0

    metadata: dict = {}
    client = ModelServiceClient(args.socket)
    answer = client.generate(
        "请简洁回答：" + args.question,
        knowledge_question=args.question,
        on_knowledge=metadata.update)
    print(answer)
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
