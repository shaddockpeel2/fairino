from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from fr3c_target_follow.marker_pose import (
    MarkerDetector, camera_parameters, candidate_orientation_quality, quaternion_xyzw, rotation_distance_deg,
    solve_marker_pose, square_points, stamp_is_fresh,
)


K = np.array([[480., 0, 320], [0, 480., 240], [0, 0, 1]])


def test_demo_candidate_policy_accepts_close_ambiguous_branches_and_keeps_wide_ambiguity():
    rvec = np.array([np.pi-.04, .07, 0.])
    corners = cv2.projectPoints(square_points(.02), rvec, np.array([.005, -.01, .3]), K, np.zeros(5))[0].reshape(4, 2)
    corners += .05*np.array([[1., -1.], [-1., 1.], [1., 1.], [-1., -1.]])
    original = solve_marker_pose(corners, .02, K, np.zeros(5), ambiguity_margin_px=.1)
    demo = solve_marker_pose(corners, .02, K, np.zeros(5), ambiguity_margin_px=.1,
                             orientation_candidate_tolerance_deg=35.)
    assert original.orientation_ambiguous and not demo.orientation_ambiguous
    assert demo.position_m == pytest.approx(original.position_m)
    assert demo.quaternion == pytest.approx(original.quaternion)
    assert demo.selected_candidate_index == original.selected_candidate_index
    strict = candidate_orientation_quality('IPPE_SQUARE', 18., .05, 1, .1)
    assert strict['orientation_ambiguous'] and not strict['orientation_measurement_valid']
    approximate = candidate_orientation_quality('IPPE_SQUARE', 18., .05, 1, .1, 35.)
    assert approximate['orientation_measurement_valid'] and approximate['orientation_approximate']
    assert not candidate_orientation_quality('IPPE_SQUARE', 36., .05, 1, .1, 35.)['orientation_measurement_valid']
    assert not candidate_orientation_quality('ITERATIVE_FALLBACK', None, None, 0, .1, 35.)['orientation_measurement_valid']


def projected_tag(size=0.020, distortion=None):
    rvec = np.array([2.8, 0.18, -0.12])
    tvec = np.array([0.015, -0.008, 0.34])
    d = np.zeros(5) if distortion is None else distortion
    corners = cv2.projectPoints(square_points(size), rvec, tvec, K, d)[0].reshape(4, 2)
    return corners, rvec, tvec, d


def test_recover_metric_position_and_orientation_with_realistic_distortion():
    corners, rvec, position, d = projected_tag(distortion=np.array([-.08, .02, .001, -.002, 0]))
    pose = solve_marker_pose(corners, .020, K, d)
    assert np.linalg.norm(pose.position_m - position) < 1e-7
    assert rotation_distance_deg(pose.rotation, cv2.Rodrigues(rvec)[0]) < 1e-4
    assert pose.reprojection_error_px < 1e-5
    assert np.linalg.norm(pose.quaternion) == pytest.approx(1)
    assert len(pose.candidate_errors_px) == pose.candidate_count
    assert pose.candidate_errors_px == sorted(pose.candidate_errors_px)
    assert pose.selected_candidate_index == 0
    if pose.candidate_count > 1:
        assert 0 <= pose.candidate_rotation_distance_deg <= 180
        assert pose.candidate_margin_px == pytest.approx(pose.candidate_errors_px[1]-pose.candidate_errors_px[0])


def test_cube_edge_must_not_be_used_as_label_edge():
    corners, _, _, d = projected_tag()
    correct = solve_marker_pose(corners, .020, K, d)
    incorrect = solve_marker_pose(corners, .030, K, d)
    assert incorrect.position_m == pytest.approx(correct.position_m * 1.5)


def test_exactly_front_facing_tag_keeps_correct_axes_and_metric_scale():
    rotation_vector = np.array([np.pi, 0., 0.])
    translation = np.array([0., 0., .3])
    corners = cv2.projectPoints(square_points(.02), rotation_vector, translation, K, np.zeros(5))[0]
    pose = solve_marker_pose(corners, .02, K, np.zeros(5))
    assert pose.position_m == pytest.approx(translation, abs=1e-7)
    assert rotation_distance_deg(pose.rotation, cv2.Rodrigues(rotation_vector)[0]) < 1e-4


@pytest.mark.parametrize('size', [0, -0.02, np.nan, np.inf])
def test_invalid_marker_size_is_rejected(size):
    with pytest.raises(ValueError):
        square_points(size)


def test_tiny_or_nonconvex_label_is_rejected():
    corners, _, _, d = projected_tag()
    with pytest.raises(ValueError, match='marker_too_small'):
        solve_marker_pose(corners, .020, K, d, min_edge_px=100)
    with pytest.raises(ValueError, match='quadrilateral'):
        solve_marker_pose(corners[[0, 2, 1, 3]], .020, K, d)


def test_known_tag_is_detected_without_accepting_a_different_id():
    detector = MarkerDetector('DICT_APRILTAG_36h11')
    dictionary = detector.dictionary
    marker = (cv2.aruco.generateImageMarker(dictionary, 1, 120)
              if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dictionary, 1, 120))
    image = np.full((240, 320), 255, np.uint8)
    image[60:180, 100:220] = marker
    corners, ids, _ = detector.detect(image)
    assert ids.flatten().tolist() == [1]
    assert len(corners) == 1
    assert detector.detect(np.full_like(image, 255))[1] is None


def test_quaternion_handles_zero_and_near_half_turn():
    assert quaternion_xyzw(np.zeros(3)) == pytest.approx([0, 0, 0, 1])
    assert np.abs(quaternion_xyzw([np.pi, 0, 0])) == pytest.approx([1, 0, 0, 0], abs=1e-10)


def test_failed_threshold_retries_without_fabricating_target_id():
    detector = MarkerDetector('DICT_APRILTAG_36h11', adaptive_thresh_constant=300.,
                              fallback_thresh_constants=[7.], target_id=1)
    dictionary = detector.dictionary
    def image_for(marker_id):
        marker = (cv2.aruco.generateImageMarker(dictionary, marker_id, 120)
                  if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dictionary, marker_id, 120))
        image = np.full((240,320),255,np.uint8)
        image[60:180,100:220] = marker
        return image
    assert detector.detect(image_for(1))[1].flatten().tolist() == [1]
    assert detector.last_pass_count == 2 and detector.last_threshold == 7.
    assert 1 not in detector.detect(image_for(2))[1].flatten()
    assert detector.detect(np.full((240,320),255,np.uint8))[1] is None


def camera_info():
    return SimpleNamespace(height=480, width=640, header=SimpleNamespace(frame_id='camera_optical'),
                           binning_x=0, binning_y=0, roi=SimpleNamespace(x_offset=0, y_offset=0),
                           k=K.flatten().tolist(), d=[0.] * 5, distortion_model='plumb_bob')


def test_camera_frame_and_dimensions_cannot_be_silently_mixed():
    info = camera_info()
    camera_parameters(info, (480, 640, 3), 'camera_optical')
    with pytest.raises(ValueError, match='frames'):
        camera_parameters(info, (480, 640, 3), 'base_link')
    with pytest.raises(ValueError, match='dimensions'):
        camera_parameters(info, (240, 320, 3), 'camera_optical')
    info.distortion_model = 'equidistant'
    with pytest.raises(ValueError, match='distortion'):
        camera_parameters(info, (480, 640, 3), 'camera_optical')


def test_stale_future_and_unstamped_measurements_are_not_fresh():
    assert stamp_is_fresh(100, 100.1, .25)
    assert not stamp_is_fresh(0, .1, .25)
    assert not stamp_is_fresh(100, 100.5, .25)
    assert not stamp_is_fresh(100.2, 100, .25)
