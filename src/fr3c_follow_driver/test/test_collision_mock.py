"""Real model/FCL in the real ROS follow loop, with a kinematically consistent mock SDK."""
import os

import pytest
import yaml
from test_follow_mock import Rig, ROOT

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1',
                               reason='explicit isolated mock ROS run only')


def parameters(tmp_path, table_z=0, joints=None):
    import xacro
    robot = ROOT/'src/fairino3mt_v6_moveit2_config/config'
    urdf = xacro.process_file(str(robot/'fairino3mt_v6_robot.urdf.xacro'), mappings={
        'initial_positions_file': str(robot/'initial_positions.yaml'),
        'hardware_plugin': 'mock_components/GenericSystem'}).toxml()
    path = tmp_path/'collision.yaml'
    path.write_text(yaml.safe_dump({'position_follow': {'ros__parameters': {
        'collision_enabled': True, 'robot_description': urdf,
        'robot_description_semantic': (robot/'fairino3mt_v6_robot.srdf').read_text(),
        'table_z_m': float(table_z), 'max_speed_m_s': .02, 'max_accel_m_s2': .06,
        'follow_radius_m': 1., 'collision_check_period_ms': 20.0,
        'collision_result_timeout_s': .15}}}))
    if joints is not None:
        data = yaml.safe_load(path.read_text())
        data['position_follow']['ros__parameters']['mock_joints_rad'] = joints
        path.write_text(yaml.safe_dump(data))
    return path


def test_guarded_follow_stops_and_retreats_from_table(tmp_path):
    # Clear-space follow using the real model and original SDK loop.
    config = parameters(tmp_path)
    rig = Rig('collision_default', tmp_path, collision_parameters=config)
    try:
        rig.ready()
        state = rig.wait(lambda s: s.get('collision', {}).get('ready'))
        initial = state['actual_tcp_m']
        assert state['collision']['speed_scale'] > 0
        assert rig.call('start').success
        rig.target[0] += .006
        rig.wait(lambda s: s['servo_commands_sent'] > 20)
        rig.pump(1.5)
        assert rig.states[-1]['actual_tcp_m'][0] > initial[0]+.002
        assert rig.call('stop').success
        rig.wait(lambda s: s['status']=='stopped')
    finally:
        code, output = rig.close()
        assert code == 0, output

    # Offline IK places the end effector near the normal base XY table.
    low_joints = [2.1766016770598067, -1.4856929496811448, 2.0658144309246964, -2.2415218324601884, -1.3556854595821786, 1.7066582629413074e-06]
    config = parameters(tmp_path, table_z=0.0, joints=low_joints)
    rig = Rig('collision_table_pause', tmp_path, collision_parameters=config)
    try:
        rig.ready(); state = rig.wait(lambda s: s.get('collision', {}).get('ready'))
        assert rig.call('start').success
        rig.target[2] -= .060
        paused = rig.wait(lambda s: s['status']=='paused_collision' and
                          s.get('collision', {}).get('reason', '').startswith('collision_margin'), timeout=8)
        assert paused['actual_tcp_m'][2] > 0.0
        count = paused['servo_commands_sent']; rig.pump(.2)
        assert rig.states[-1]['servo_commands_sent']==count
        rig.target[2] += .080
        resumed = rig.wait(lambda s: s['status']=='following' and s['servo_commands_sent']>count, timeout=5)
        assert resumed['collision']['speed_scale'] > 0
        assert rig.call('stop').success
        rig.wait(lambda s: s['status']=='stopped')
    finally:
        code, output = rig.close()
        assert code == 0, output
