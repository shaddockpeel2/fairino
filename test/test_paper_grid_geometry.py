import numpy as np
import pytest

from fr3c_calibration.paper_grid_geometry import make_grid


def _corners():
    # A4 in a tilted plane, with TCP +Z parallel to the paper normal.
    angle = np.deg2rad(20)
    u = np.asarray([1., 0., 0.])
    v = np.asarray([0., np.cos(angle), np.sin(angle)])
    origin = np.asarray([.3, -.1, .2])
    quaternion = [np.sin(angle / 2), 0., 0., np.cos(angle / 2)]
    return {
        name: {'xyz_m': (origin + x * .210 * u + y * .297 * v).tolist(),
               'quaternion_xyzw': quaternion}
        for name, x, y in [('TL', 0, 0), ('TR', 1, 0),
                           ('BR', 1, 1), ('BL', 0, 1)]
    }


def test_a4_grid_has_four_equal_rectangles_on_tilted_plane():
    result = make_grid(_corners())
    assert list(result['regions']) == ['A', 'B', 'C', 'D']
    assert result['width_m'] == pytest.approx(.210)
    assert result['height_m'] == pytest.approx(.297)
    assert result['max_tcp_z_to_plane_normal_deg'] == pytest.approx(0, abs=1e-6)
    for region in result['regions'].values():
        assert region['width_m'] == pytest.approx(.105)
        assert region['height_m'] == pytest.approx(.1485)
        assert region['area_m2'] == pytest.approx(.210 * .297 / 4)
    assert np.allclose(result['regions']['A']['corners_m'][1],
                       result['regions']['B']['corners_m'][0])
    assert np.allclose(result['regions']['A']['corners_m'][2],
                       result['regions']['C']['corners_m'][0])


def test_rejects_wrong_tcp_axis():
    corners = _corners()
    corners['BL']['quaternion_xyzw'] = [0., 0., 0., 1.]
    with pytest.raises(ValueError, match='TCP Z'):
        make_grid(corners)


def test_rejects_wrong_sheet_size_or_out_of_plane_corner():
    corners = _corners()
    corners['TR']['xyz_m'][0] += .05
    with pytest.raises(ValueError, match='A4|rectangle'):
        make_grid(corners)

    corners = _corners()
    corners['BR']['xyz_m'][2] += .02
    with pytest.raises(ValueError, match='coplanar|rectangle'):
        make_grid(corners)


def test_vertical_projection_puts_all_regions_on_requested_plane():
    corners = _corners()
    for record in corners.values():
        record['quaternion_xyzw'] = [0., 0., 0., 1.]
    projected = make_grid(corners, plane_z_m=0.0)
    assert projected['projection'] == {'mode': 'base_z', 'plane_z_m': 0.0}
    assert projected['measured_tcp_z_span_mm'] > 0
    assert projected['max_plane_error_mm'] == pytest.approx(0)
    for region in projected['regions'].values():
        assert region['center_m'][2] == 0.0
        assert all(point[2] == 0.0 for point in region['corners_m'])
    assert corners['BR']['xyz_m'][2] != 0.0  # raw measurement is retained
