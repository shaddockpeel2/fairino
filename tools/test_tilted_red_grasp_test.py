import math

import pytest

from tilted_red_grasp_test import check_runtime, make_config, tilted_parameters as parameters


def rotate(q, v):
    # Independent quaternion-to-matrix check of the physical axes.
    x, y, z, w = q
    return (
        (1-2*y*y-2*z*z)*v[0] + (2*x*y-2*z*w)*v[1] + (2*x*z+2*y*w)*v[2],
        (2*x*y+2*z*w)*v[0] + (1-2*x*x-2*z*z)*v[1] + (2*y*z-2*x*w)*v[2],
        (2*x*z-2*y*w)*v[0] + (2*y*z+2*x*w)*v[1] + (1-2*x*x-2*y*y)*v[2],
    )


@pytest.mark.parametrize('angle', [-45, -30, -20, 5, 20, 30, 45])
def test_tilt_preserves_closing_axis_and_requested_inclination(angle):
    q = parameters(angle)['grasp_orientation_xyzw']
    assert sum(v*v for v in q) == pytest.approx(1)
    assert rotate(q, (1, 0, 0)) == pytest.approx((-1, 0, 0))
    tool_z = rotate(q, (0, 0, 1))
    assert math.degrees(math.acos(-tool_z[2])) == pytest.approx(abs(angle))
    assert tool_z[1] == pytest.approx(-math.sin(math.radians(angle)))


@pytest.mark.parametrize('angle', [0, 4.9, 45.1, -46, float('nan'), float('inf')])
def test_bad_angles_are_refused(angle):
    with pytest.raises(ValueError):
        parameters(angle)


def test_config_preserves_collision_checks_and_production_defaults():
    config = make_config(30, 10, 0)['fr3c_first_grasp']['ros__parameters']
    assert config['minimum_cartesian_fraction'] == pytest.approx(0.999999)
    assert config['maximum_cartesian_joint_step_rad'] == pytest.approx(0.10)
    assert config['tip_frame'] == 'grasp_frame'
    assert config['target_top_z_bounds_m'][1] < 0.060  # rejects stacked target
    assert config['grasp_depth_m'] == 0.010
    assert config['grasp_tilt_candidates_rad'] == [0.0]
    assert config['grasp_contact_offset_xyz_m'] == [0.0, 0.0, 0.0]
    assert config['lift_distance_m'] == 0.050
    assert config['sort_enabled'] is False
    assert config['stack_enabled'] is False
    assert make_config(-20, 10, 0)['fr3c_first_grasp']['ros__parameters'][
        'grasp_orientation_xyzw'][2] > 0


def test_edge_contact_is_expressed_from_tcp_toward_flange():
    config = make_config(30, 10, 27.5)['fr3c_first_grasp']['ros__parameters']
    assert config['grasp_contact_offset_xyz_m'] == [0.0, 0.0, -0.0275]


@pytest.mark.parametrize('depth,offset', [(4.9, 0), (15.1, 0), (10, -1), (10, 55.1)])
def test_bad_depth_or_contact_offset_is_refused(depth, offset):
    with pytest.raises(ValueError):
        parameters(30, depth, offset)


@pytest.mark.parametrize('name,value', [
    ('execute', True), ('sort_enabled', True), ('target_top_z_bounds_m', [0.025, 0.5]),
    ('grasp_orientation_xyzw', [0., 1., 0., 0.]), ('log_directory', '/tmp/production'),
    ('grasp_contact_offset_xyz_m', [0.0, 0.0, -0.015]),
])
def test_run_refuses_wrong_server_or_mode(name, value):
    actual = {**parameters(30, 10, 0), 'execute': False}
    check_runtime(actual, 30, 10, 0, False)
    actual[name] = value
    with pytest.raises(RuntimeError):
        check_runtime(actual, 30, 10, 0, False)
