import json
from types import SimpleNamespace

import pytest

from fr3c_calibration.paper_grid import COLORS, _load_session, project, reset, session_markers
from builtin_interfaces.msg import Time


def _options(path):
    return SimpleNamespace(session=str(path), base_frame='base_link',
                           tcp_frame='grasp_frame')


def test_reset_archives_empty_session_and_creates_reusable_file(tmp_path, capsys):
    path = tmp_path / 'paper.json'
    path.write_bytes(b'')
    with pytest.raises(ValueError, match='empty or invalid JSON'):
        _load_session(path)

    reset(_options(path))
    result = json.loads(capsys.readouterr().out)
    archive = tmp_path / result['archived_previous_session'].split('/')[-1]
    assert archive.is_file()
    assert archive.read_bytes() == b''
    session = _load_session(path)
    assert session['corners'] == {}
    assert session['grid'] is None


def test_reset_preserves_previous_valid_corner_records(tmp_path, capsys):
    path = tmp_path / 'paper.json'
    old = {'schema_version': 1, 'type': 'a4_tcp_grid',
           'base_frame': 'base_link', 'tcp_frame': 'grasp_frame',
           'corners': {'TL': {'xyz_m': [1, 2, 3]}}}
    path.write_text(json.dumps(old))
    reset(_options(path))
    result = json.loads(capsys.readouterr().out)
    assert json.loads(open(result['archived_previous_session']).read()) == old
    assert _load_session(path)['corners'] == {}


def test_project_preserves_raw_corners_and_moves_markers_to_xy_plane(tmp_path, capsys):
    path = tmp_path / 'paper.json'
    xy = ((0, 0), (.210, 0), (.210, .297), (0, .297))
    corners = {name: {'xyz_m': [x, y, .003 + index * .0002],
                      'quaternion_xyzw': [0, 0, 0, 1]}
               for index, (name, (x, y)) in enumerate(zip(('TL', 'TR', 'BR', 'BL'), xy))}
    old = {'schema_version': 1, 'type': 'a4_tcp_grid',
           'base_frame': 'base_link', 'tcp_frame': 'grasp_frame',
           'corners': corners, 'grid': None}
    path.write_text(json.dumps(old))
    options = _options(path)
    options.plane_z = 0.0
    project(options)
    result = json.loads(capsys.readouterr().out)
    assert json.loads(open(result['backup']).read()) == old
    current = _load_session(path)
    assert current['corners'] == corners
    assert current['grid']['projection']['plane_z_m'] == 0.0
    markers = session_markers(current, Time()).markers
    spheres = [item for item in markers if item.type == item.SPHERE]
    assert len(spheres) == 4
    assert all(item.pose.position.z == 0.0 for item in spheres)
    assert {item.text for item in markers if item.text} == set('ABCD') | {'TL', 'TR', 'BR', 'BL'}
    fills = [item for item in markers if item.type == item.TRIANGLE_LIST]
    assert len(fills) == 4
    assert all(len(item.points) == 6 and item.color.a == pytest.approx(.45)
               for item in fills)
    assert [(item.color.r, item.color.g, item.color.b) for item in fills] == [
        COLORS[name] for name in 'ABCD']
    assert COLORS['C'][0] > COLORS['C'][1]  # C is red in the workcell.
    assert COLORS['D'][1] > COLORS['D'][0]  # D is green in the workcell.
