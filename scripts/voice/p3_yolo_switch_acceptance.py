#!/usr/bin/env python3
"""Verify real YOLO pause/drain/resume while Qwen owns the RK3588 NPU."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time


WORKSPACE = Path(__file__).resolve().parents[2]
PACKAGE = WORKSPACE / 'src/fr3c_voice'
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

from fr3c_voice.config import load_config  # noqa: E402
from fr3c_voice.controller import QwenController  # noqa: E402
from fr3c_voice.coordinator import NpuCoordinator, NpuOwner  # noqa: E402
from fr3c_voice.ros_bridge import RosBridgeRuntime  # noqa: E402
from fr3c_voice.trace import TraceWriter  # noqa: E402


def wait_yolo(bridge, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = bridge.perception_snapshot()
        if (snapshot.available and snapshot.npu_inference
                and snapshot.enabled and snapshot.frames_inferred > 0
                and snapshot.last_success_ns is not None):
            return snapshot
        time.sleep(0.1)
    raise TimeoutError('isolated YOLO perception did not become ready')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=WORKSPACE / 'config/voice/qwen3vl.yaml')
    parser.add_argument('--namespace', default='/p3_yolo')
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--output', type=Path,
                        default=WORKSPACE / 'evidence/voice/P3/yolo_switch.json')
    args = parser.parse_args()
    qwen, service = load_config(args.config, WORKSPACE)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    trace = TraceWriter(args.output.with_suffix('.trace.jsonl'))
    prefix = args.namespace.rstrip('/') + '/blue_cube_localizer'
    runtime = RosBridgeRuntime(
        prefix + '/status', prefix + '/set_inference_enabled',
        service.grasp_status_topic)
    controller = QwenController(qwen, service.runtime_dir, trace=trace)
    coordinator = NpuCoordinator(
        runtime.bridge, service.pause_confirm_timeout_s,
        service.resume_confirm_timeout_s, service.status_stale_timeout_s,
        service.minimum_available_memory_mib, trace)
    records = []
    try:
        initial = wait_yolo(runtime.bridge, 15.0)
        coordinator.run(NpuOwner.QWEN_LOAD, 'yolo-qwen-load', controller.start)
        for index in range(1, args.count + 1):
            result = coordinator.run(
                NpuOwner.QWEN_TEXT, f'yolo-text-{index:02d}',
                lambda index=index: (
                    controller.reset_context(),
                    controller.chat_text(
                        '请只回答：YOLO恢复验证正常。',
                        request_id=f'yolo-text-{index:02d}'))[1])
            records.append(asdict(result))
        final = runtime.bridge.perception_snapshot()
        summary = {
            'ok': True,
            'requests': len(records),
            'responses': records,
            'worker_pid': controller.worker_pid,
            'initial_perception': asdict(initial),
            'final_perception': asdict(final),
            'fresh_frames_after_switches': final.frames_inferred - initial.frames_inferred,
            'pause_generations': final.pause_generation - initial.pause_generation,
            'coordination_fault': coordinator.fault,
        }
        if final.frames_inferred <= initial.frames_inferred:
            raise RuntimeError('YOLO produced no fresh frame after Qwen switches')
        if final.pause_generation < initial.pause_generation + args.count + 1:
            raise RuntimeError('not every Qwen lease paused YOLO')
        args.output.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8')
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0
    finally:
        controller.close()
        runtime.close()


if __name__ == '__main__':
    raise SystemExit(main())
