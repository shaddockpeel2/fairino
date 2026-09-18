from pathlib import Path

import pytest

from fr3c_voice.config import ConfigError, load_config


def document(asset_root: Path) -> str:
    return f'''schema_version: 1
assets:
  root: {asset_root}
model:
  id: qwen
  vision_path: model/vision.rknn
  language_path: model/language.rkllm
worker:
  binary_path: bin/worker
  rkllm_runtime_path: lib/librkllmrt.so
  rknn_runtime_path: lib/librknnrt.so
  transport: unix
  socket_name: worker.sock
  max_new_tokens: 32
  max_context_len: 256
  npu_core_num: 1
  startup_timeout_s: 2
  request_timeout_s: 3
  cancel_grace_s: 0.2
  terminate_grace_s: 1
  max_prompt_bytes: 1024
  max_image_bytes: 2048
  tools_enabled: false
service:
  runtime_directory: evidence/runtime
  trace_directory: evidence/runs
  socket_name: service.sock
  auto_recover: true
coordination:
  perception_status_topic: /perception/status
  perception_control_service: /perception/control
  grasp_status_topic: /grasp/state
  pause_confirm_timeout_s: 1
  resume_confirm_timeout_s: 2
  status_stale_timeout_s: 1
  minimum_available_memory_mib: 128
'''


def test_config_separates_read_only_assets_and_mutable_workspace(tmp_path):
    asset_root = tmp_path / 'assets'
    asset_root.mkdir()
    path = tmp_path / 'config.yaml'
    path.write_text(document(asset_root), encoding='utf-8')
    qwen, service = load_config(path, tmp_path / 'workspace')
    assert qwen.vision_path == asset_root / 'model/vision.rknn'
    assert service.runtime_dir == tmp_path / 'workspace/evidence/runtime'
    assert service.trace_directory == tmp_path / 'workspace/evidence/runs'


def test_config_rejects_escape_tools_and_unbounded_context(tmp_path):
    asset_root = tmp_path / 'assets'
    asset_root.mkdir()
    base = document(asset_root)
    for changed, message in (
        (base.replace('vision_path: model/vision.rknn',
                      'vision_path: ../vision.rknn'), 'escapes'),
        (base.replace('tools_enabled: false', 'tools_enabled: true'), 'forbidden'),
        (base.replace('max_context_len: 256', 'max_context_len: 9000'),
         'runtime bound'),
    ):
        path = tmp_path / 'bad.yaml'
        path.write_text(changed, encoding='utf-8')
        with pytest.raises(ConfigError, match=message):
            load_config(path, tmp_path / 'workspace')
