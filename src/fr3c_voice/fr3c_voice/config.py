"""Strict configuration loader for the FR3C Qwen service."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any, Mapping

import yaml


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class QwenConfig:
    asset_root: Path
    model_id: str
    vision_path: Path
    language_path: Path
    worker_path: Path
    rkllm_runtime_path: Path
    rknn_runtime_path: Path
    socket_name: str
    max_new_tokens: int
    max_context_len: int
    npu_core_num: int
    startup_timeout_s: float
    request_timeout_s: float
    cancel_grace_s: float
    terminate_grace_s: float
    max_prompt_bytes: int
    max_image_bytes: int
    platform: str = 'rk3588'
    tools_enabled: bool = False


@dataclass(frozen=True)
class ServiceConfig:
    runtime_dir: Path
    service_socket_name: str
    trace_directory: Path
    perception_status_topic: str
    perception_control_service: str
    grasp_status_topic: str
    pause_confirm_timeout_s: float
    resume_confirm_timeout_s: float
    status_stale_timeout_s: float
    minimum_available_memory_mib: int
    auto_recover: bool


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigError(f'{label} must be a mapping')
    return value


def _positive_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise ConfigError(f'{label} must be a positive integer')
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise ConfigError(f'{label} must be a positive integer') from error
    if result <= 0:
        raise ConfigError(f'{label} must be a positive integer')
    return result


def _positive_float(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ConfigError(f'{label} must be positive') from error
    if result <= 0:
        raise ConfigError(f'{label} must be positive')
    return result


def _plain_filename(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or Path(value).name != value:
        raise ConfigError(f'{label} must be a plain filename')
    return value


def _inside(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f'{label} must be a non-empty asset-relative path')
    raw = Path(value)
    if raw.is_absolute():
        raise ConfigError(f'{label} must be asset-relative')
    resolved = (root / raw).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ConfigError(f'{label} escapes asset_root') from error
    return resolved


def _workspace_path(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f'{label} must be a non-empty workspace-relative path')
    raw = Path(value)
    if raw.is_absolute() or '..' in raw.parts:
        raise ConfigError(f'{label} must be workspace-relative')
    return (root / raw).resolve()


def load_config(
    config_path: Path | str,
    workspace_root: Path | str,
) -> tuple[QwenConfig, ServiceConfig]:
    workspace = Path(workspace_root).resolve()
    try:
        document = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(f'cannot read Qwen configuration: {error}') from error
    data = _mapping(document, 'configuration')
    if data.get('schema_version') != 1:
        raise ConfigError('unsupported schema_version')
    assets = _mapping(data.get('assets'), 'assets')
    model = _mapping(data.get('model'), 'model')
    worker = _mapping(data.get('worker'), 'worker')
    service = _mapping(data.get('service'), 'service')
    coordination = _mapping(data.get('coordination'), 'coordination')

    configured_root = str(assets.get('root') or '').strip()
    root_text = os.environ.get('FR3C_QWEN_ASSET_ROOT', configured_root)
    if not root_text:
        raise ConfigError('assets.root or FR3C_QWEN_ASSET_ROOT is required')
    asset_root = Path(root_text).expanduser().resolve()
    if worker.get('transport') != 'unix':
        raise ConfigError('only Unix worker transport is permitted')
    if bool(worker.get('tools_enabled', False)):
        raise ConfigError('model-side tools are forbidden')
    runtime_override = os.environ.get('FR3C_VOICE_RUNTIME_DIR')
    runtime_dir = (
        Path(runtime_override).expanduser().resolve()
        if runtime_override
        else _workspace_path(workspace, service.get('runtime_directory'),
                             'service.runtime_directory')
    )
    trace_directory = _workspace_path(
        workspace, service.get('trace_directory'), 'service.trace_directory')

    qwen = QwenConfig(
        asset_root=asset_root,
        model_id=str(model.get('id') or ''),
        vision_path=_inside(asset_root, model.get('vision_path'), 'model.vision_path'),
        language_path=_inside(asset_root, model.get('language_path'), 'model.language_path'),
        worker_path=_inside(asset_root, worker.get('binary_path'), 'worker.binary_path'),
        rkllm_runtime_path=_inside(
            asset_root, worker.get('rkllm_runtime_path'),
            'worker.rkllm_runtime_path'),
        rknn_runtime_path=_inside(
            asset_root, worker.get('rknn_runtime_path'),
            'worker.rknn_runtime_path'),
        socket_name=_plain_filename(worker.get('socket_name'), 'worker.socket_name'),
        max_new_tokens=_positive_int(worker.get('max_new_tokens'), 'worker.max_new_tokens'),
        max_context_len=_positive_int(worker.get('max_context_len'), 'worker.max_context_len'),
        npu_core_num=_positive_int(worker.get('npu_core_num'), 'worker.npu_core_num'),
        startup_timeout_s=_positive_float(worker.get('startup_timeout_s'), 'worker.startup_timeout_s'),
        request_timeout_s=_positive_float(worker.get('request_timeout_s'), 'worker.request_timeout_s'),
        cancel_grace_s=_positive_float(worker.get('cancel_grace_s'), 'worker.cancel_grace_s'),
        terminate_grace_s=_positive_float(worker.get('terminate_grace_s'), 'worker.terminate_grace_s'),
        max_prompt_bytes=_positive_int(worker.get('max_prompt_bytes'), 'worker.max_prompt_bytes'),
        max_image_bytes=_positive_int(worker.get('max_image_bytes'), 'worker.max_image_bytes'),
    )
    if qwen.max_context_len > 8192:
        raise ConfigError('model context limit exceeds the verified runtime bound')
    if qwen.max_new_tokens >= qwen.max_context_len:
        raise ConfigError('model output limit must be smaller than the context limit')

    service_config = ServiceConfig(
        runtime_dir=runtime_dir,
        service_socket_name=_plain_filename(
            service.get('socket_name'), 'service.socket_name'),
        trace_directory=trace_directory,
        perception_status_topic=str(coordination.get('perception_status_topic') or ''),
        perception_control_service=str(coordination.get('perception_control_service') or ''),
        grasp_status_topic=str(coordination.get('grasp_status_topic') or ''),
        pause_confirm_timeout_s=_positive_float(
            coordination.get('pause_confirm_timeout_s'),
            'coordination.pause_confirm_timeout_s'),
        resume_confirm_timeout_s=_positive_float(
            coordination.get('resume_confirm_timeout_s'),
            'coordination.resume_confirm_timeout_s'),
        status_stale_timeout_s=_positive_float(
            coordination.get('status_stale_timeout_s'),
            'coordination.status_stale_timeout_s'),
        minimum_available_memory_mib=_positive_int(
            coordination.get('minimum_available_memory_mib'),
            'coordination.minimum_available_memory_mib'),
        auto_recover=bool(service.get('auto_recover', True)),
    )
    return qwen, service_config
