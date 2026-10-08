"""Select the tracking overlay before launch without instantiating robot hardware."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('follow_launch_modes',
                                            ROOT/'src/fr3c_target_follow/launch/position_follow.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


@pytest.fixture(autouse=True)
def isolate_launch_logs(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))


def context(tmp_path, selection, configured):
    path = tmp_path/'driver.yaml'
    path.write_text(yaml.safe_dump({'position_follow': {'ros__parameters': {'collision_enabled': configured}}}))
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'collision_enabled': selection, 'driver_params_file': str(path),
        'tcp_file': str(ROOT/'src/fairino3mt_v6_moveit2_config/config/ydi_j22_tcp.yaml'),
        'mock': 'false', 'robot_ip': '192.168.58.2', 'output_dir': '',
    })
    return ctx


@pytest.mark.parametrize('selection,configured,expected', [
    ('auto', False, False), ('auto', True, True),
    ('false', True, False), ('true', False, True),
])
def test_cli_override_and_configuration_inheritance(tmp_path, selection, configured, expected):
    assert launch.collision_enabled(context(tmp_path, selection, configured)) is expected


def test_legacy_launch_skips_collision_model_loading(tmp_path, monkeypatch):
    def unexpected_model(*args, **kwargs):
        raise AssertionError('legacy launch must not load/expand the collision model')
    monkeypatch.setattr(launch.xacro, 'process_file', unexpected_model)
    # Construct a launch action only; no processes or hardware are started.
    assert len(launch.driver(context(tmp_path, 'false', True))) == 1
