import cv2
import numpy as np

from fr3c_camera.color_target_2d import (
    TargetContinuity,
    candidate_bbox,
    detect_color_target,
)


INTERVALS = {
    'red': [0, 10, 170, 179],
    'yellow': [18, 34],
    'blue': [90, 135],
    'green': [40, 85],
}


def image_with_rectangles(rectangles, shape=(240, 320, 3)):
    image = np.zeros(shape, dtype=np.uint8)
    for top_left, bottom_right, bgr in rectangles:
        cv2.rectangle(image, top_left, bottom_right, bgr, cv2.FILLED)
    return image


def test_no_requested_target_is_invalid():
    image = image_with_rectangles([((80, 60), (120, 100), (255, 0, 0))])
    result = detect_color_target(image, 'red', INTERVALS)
    assert result.candidate is None
    assert result.candidate_count == 0
    assert result.reason == 'target_not_found'


def test_one_requested_target_reports_center_bbox_and_quality_candidate():
    image = image_with_rectangles([((80, 60), (120, 100), (0, 0, 255))])
    result = detect_color_target(image, 'red', INTERVALS)
    assert result.reason == 'candidate_unique'
    assert result.candidate_count == 1
    assert result.candidate.center_px == (100.0, 80.0)
    assert candidate_bbox(result.candidate) == (80, 60, 41, 41)
    assert result.candidate.solidity > 0.99


def test_two_same_color_targets_are_invalid_instead_of_selecting_largest():
    image = image_with_rectangles([
        ((30, 30), (65, 65), (0, 0, 255)),
        ((180, 120), (225, 165), (0, 0, 255)),
    ])
    result = detect_color_target(image, 'red', INTERVALS)
    assert result.candidate is None
    assert result.candidate_count == 2
    assert result.reason == 'target_not_unique'


def test_area_outside_configured_range_is_rejected():
    too_small = image_with_rectangles([((20, 20), (25, 25), (0, 0, 255))])
    too_large = image_with_rectangles([((20, 20), (200, 200), (0, 0, 255))])
    assert detect_color_target(too_small, 'red', INTERVALS).candidate_count == 0
    assert detect_color_target(too_large, 'red', INTERVALS).candidate_count == 0


def test_all_four_configured_colors_use_shared_perception_path():
    bgr_values = {
        'red': (0, 0, 255),
        'yellow': (0, 255, 255),
        'blue': (255, 0, 0),
        'green': (0, 255, 0),
    }
    for color, bgr in bgr_values.items():
        image = image_with_rectangles([((80, 60), (120, 100), bgr)])
        result = detect_color_target(image, color, INTERVALS)
        assert result.candidate_count == 1


def test_continuity_requires_two_consecutive_frames():
    continuity = TargetContinuity(required_samples=2, maximum_jump_px=20.0)
    assert continuity.update(1.0, (100.0, 80.0)) == \
        (False, 'collecting_stable_samples')
    assert continuity.update(1.1, (102.0, 81.0)) == (True, 'stable_target')
    assert continuity.sample_count == 2


def test_missing_frame_breaks_consecutive_validity():
    continuity = TargetContinuity(required_samples=3, maximum_jump_px=20.0)
    continuity.update(1.0, (100.0, 80.0))
    continuity.update(1.1, (101.0, 80.0))
    continuity.reject()
    stable, reason = continuity.update(1.2, (102.0, 80.0))
    assert not stable
    assert reason == 'collecting_stable_samples'
    assert continuity.sample_count == 1


def test_one_transient_bad_frame_recovers_a_nearby_locked_target():
    continuity = TargetContinuity(required_samples=2, maximum_jump_px=20.0)
    continuity.update(1.0, (100.0, 80.0))
    assert continuity.update(1.1, (101.0, 80.0))[0]
    continuity.reject(transient=True)
    assert continuity.update(1.2, (103.0, 81.0)) == \
        (True, 'stable_target')

    continuity.reject(transient=True)
    continuity.reject(transient=True)
    assert continuity.update(1.3, (104.0, 81.0)) == \
        (False, 'collecting_stable_samples')


def test_transient_recovery_rejects_old_or_distant_candidate():
    continuity = TargetContinuity(required_samples=2, maximum_jump_px=20.0)
    continuity.update(1.0, (100.0, 80.0))
    continuity.update(1.1, (101.0, 80.0))
    continuity.reject(transient=True)
    assert continuity.update(1.2, (150.0, 80.0)) == (False, 'center_jump')
    assert continuity.sample_count == 1
    continuity.update(1.3, (151.0, 80.0))
    continuity.reject(transient=True)
    assert continuity.update(1.6, (152.0, 80.0)) == \
        (False, 'collecting_stable_samples')


def test_center_jump_restarts_window_with_current_observation():
    continuity = TargetContinuity(required_samples=3, maximum_jump_px=20.0)
    continuity.update(1.0, (100.0, 80.0))
    continuity.update(1.1, (101.0, 80.0))
    stable, reason = continuity.update(1.2, (150.0, 80.0))
    assert not stable
    assert reason == 'center_jump'
    assert continuity.sample_count == 1


def test_timestamp_must_strictly_increase_and_clears_window():
    continuity = TargetContinuity(required_samples=3, maximum_jump_px=20.0)
    continuity.update(1.0, (100.0, 80.0))
    continuity.update(1.1, (101.0, 80.0))
    stable, reason = continuity.update(1.1, (102.0, 80.0))
    assert not stable
    assert reason == 'timestamp_not_increasing'
    assert continuity.sample_count == 0

    # Keep the previous high-water mark: timestamps must recover beyond the
    # last accepted image instead of forming a new sequence in the past.
    stable, reason = continuity.update(1.05, (103.0, 80.0))
    assert not stable
    assert reason == 'timestamp_not_increasing'
    assert continuity.sample_count == 0
