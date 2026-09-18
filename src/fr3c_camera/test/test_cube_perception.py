from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from fr3c_camera.blue_cube_localizer import select_sort_target
from fr3c_camera.cube_perception import (
    blue_mask,
    compensate_top_surface_z,
    find_color_candidates,
    is_tabletop_cube,
    extract_top_surface,
    find_blue_candidates,
    quaternion_angular_distance,
    registered_points_from_mask,
    rpy_to_quaternion,
    tabletop_cube_tier,
    transform_points,
)


def camera_info(width=640, height=480, distortion=None):
    if distortion is None:
        distortion = np.zeros(5)
    return SimpleNamespace(
        width=width, height=height,
        k=[480.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0],
        d=list(distortion), distortion_model='plumb_bob')


def test_blue_candidate_is_unique_and_uses_eroded_interior():
    image = np.full((240, 320, 3), 220, dtype=np.uint8)
    cv2.rectangle(image, (130, 90), (175, 137), (120, 30, 10), -1)
    mask = blue_mask(image)
    candidates = find_blue_candidates(mask)
    assert len(candidates) == 1
    candidate = candidates[0]
    assert 1500 < candidate.area_px < 2500
    assert np.count_nonzero(candidate.mask) < np.count_nonzero(mask)
    assert candidate.solidity > 0.95


def test_multiple_blue_squares_are_not_silently_merged():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (30, 30), (65, 65), (255, 0, 0), -1)
    cv2.rectangle(image, (180, 120), (220, 160), (255, 0, 0), -1)
    assert len(find_blue_candidates(blue_mask(image))) == 2


def test_four_color_mode_identifies_each_color_and_keeps_multiple_candidates():
    intervals = {'red': [0, 10, 170, 179], 'green': [40, 85],
                 'blue': [90, 135], 'yellow': [18, 34]}
    colors = {'red': (0, 0, 255), 'green': (0, 255, 0),
              'blue': (255, 0, 0), 'yellow': (0, 255, 255)}
    image = np.zeros((160, 240, 3), dtype=np.uint8)
    for name, bgr in colors.items():
        image[:] = 0
        cv2.rectangle(image, (80, 50), (125, 95), bgr, -1)
        matches = find_color_candidates(image, intervals)
        assert len(matches) == 1
        assert matches[0][0] == name
    cv2.rectangle(image, (150, 50), (195, 95), colors['blue'], -1)
    assert len(find_color_candidates(image, intervals)) == 2


def test_sort_target_selection_uses_red_yellow_blue_green_then_position():
    def match(color, x, y):
        surface = SimpleNamespace(center=np.array([x, y, 0.03]))
        return color, SimpleNamespace(), surface

    matches = [
        match('green', 0.1, 0.0),
        match('red', 0.3, 0.0),
        match('yellow', 0.0, 0.0),
        match('red', 0.2, 0.0),
        match('blue', -0.1, 0.0),
    ]
    selected, skipped, actionable = select_sort_target(matches, [], 0.025)
    assert selected[0] == 'red'
    assert selected[2].center[0] == pytest.approx(0.2)
    assert skipped == 0
    assert actionable == 5


def test_sort_target_selection_ignores_only_nearby_same_color_target():
    def match(color, x):
        surface = SimpleNamespace(center=np.array([x, -0.4, 0.03]))
        return color, SimpleNamespace(), surface

    matches = [match('red', 0.20), match('red', 0.27), match('yellow', 0.10)]
    skipped_targets = [('red', np.array([0.205, -0.4, 0.03]))]
    selected, skipped, actionable = select_sort_target(
        matches, skipped_targets, 0.025)
    assert selected[0] == 'red'
    assert selected[2].center[0] == pytest.approx(0.27)
    assert skipped == 1
    assert actionable == 2


def test_requested_color_filters_other_colors_and_requires_uniqueness():
    def match(color, x):
        surface = SimpleNamespace(center=np.array([x, -0.4, 0.03]))
        return color, SimpleNamespace(), surface

    matches = [match('red', 0.20), match('yellow', 0.10)]
    selected, skipped, actionable = select_sort_target(
        matches, [], 0.025, target_color='yellow')
    assert selected[0] == 'yellow'
    assert skipped == 0
    assert actionable == 1

    duplicate_yellow = matches + [match('yellow', 0.18)]
    selected, skipped, actionable = select_sort_target(
        duplicate_yellow, [], 0.025, target_color='yellow')
    assert selected is None
    assert skipped == 0
    assert actionable == 2


def test_requested_color_reports_no_target_instead_of_falling_back():
    surface = SimpleNamespace(center=np.array([0.20, -0.4, 0.03]))
    selected, skipped, actionable = select_sort_target(
        [('blue', SimpleNamespace(), surface)], [], 0.025,
        target_color='red')
    assert selected is None
    assert skipped == 0
    assert actionable == 0


def test_flat_colored_background_does_not_count_as_second_cube():
    image = np.zeros((160, 240, 3), dtype=np.uint8)
    cv2.rectangle(image, (35, 50), (74, 89), (255, 0, 0), -1)
    cv2.rectangle(image, (145, 50), (184, 89), (0, 0, 255), -1)
    matches = find_color_candidates(
        image, {'blue': [90, 135], 'red': [0, 10, 170, 179]})
    assert len(matches) == 2
    info = camera_info(width=240, height=160)
    info.k = [600.0, 0.0, 120.0, 0.0, 600.0, 80.0, 0.0, 0.0, 1.0]
    depth = np.full((160, 240), 0.47, dtype=np.float32)
    depth[50:90, 35:75] = 0.50  # blue cube top is 30 mm above the plane
    physical = []
    for color, candidate in matches:
        points, fraction = registered_points_from_mask(depth, info, info,
                                                       candidate.mask)
        surface = extract_top_surface(
            transform_points(points, [0, 0, -0.47], [0, 0, 0, 1]),
            fraction, min_top_points=30)
        if is_tabletop_cube(surface, 0.03, 1.25, 0.0, 0.01):
            physical.append(color)
    assert physical == ['blue']


def test_mild_perspective_stretch_is_still_a_cube_but_old_limit_rejects_it():
    # Live RGB-D data measured the visible blue 30 mm cube at 1.301. Accept that
    # perspective/depth-edge stretch while keeping the 1.40..1.55 ambiguity
    # band before a 60 x 30 mm rectangular top is classified.
    surface = SimpleNamespace(
        center=np.array([0.25, -0.35, 0.0295]), axis_ratio=1.301)
    assert tabletop_cube_tier(surface, 0.030, 1.40, 0.0, 0.010) == 0
    assert tabletop_cube_tier(surface, 0.030, 1.25, 0.0, 0.010) is None

    surface.center[2] = 0.0279
    assert tabletop_cube_tier(surface, 0.030, 1.40, 0.0, 0.010) is None


def test_stacked_cube_tier_distinguishes_a_cube_on_the_table_from_one_on_top():
    """
    The upper cube of a stack must stay detectable, and it must be reported as
    a different tier. Without that, the lower cube of a new stack could be
    mistaken for a free table position and a third cube would be piled on.
    """
    info = camera_info(width=100, height=80)
    info.k = [200.0, 0.0, 50.0, 0.0, 200.0, 40.0, 0.0, 0.0, 1.0]

    def top_surface_at(top_z_m):
        # The depth values have to stay inside the sensor's valid range, so the
        # cube region gets a fixed 30 mm depth step and the base-frame height is
        # then set by the translation.
        depth = np.full((80, 100), 0.50, dtype=np.float32)
        depth[34:47, 44:57] = np.float32(0.47)
        mask = np.zeros_like(depth, dtype=np.uint8)
        mask[34:47, 44:57] = 255
        points, fraction = registered_points_from_mask(depth, info, info, mask)
        surface = extract_top_surface(
            transform_points(points, [0.1, -0.2, top_z_m - 0.47], [0, 0, 0, 1]),
            fraction, top_quantile=60.0, top_tolerance_m=0.002,
            min_top_points=20)
        assert surface.center[2] == pytest.approx(top_z_m, abs=0.002)
        return surface

    # Bottom of a cube: top face one cube height above the plane.
    on_table = top_surface_at(0.030)
    assert tabletop_cube_tier(
        on_table, 0.030, 1.25, 0.0, 0.010,
        stacked_offsets_m=[0.030], stack_tolerance_m=0.012) == 0

    # Upper cube of a two-cube stack: top face two cube heights above the plane.
    stacked = top_surface_at(0.060)
    assert tabletop_cube_tier(
        stacked, 0.030, 1.25, 0.0, 0.010,
        stacked_offsets_m=[0.030], stack_tolerance_m=0.012) == 1

    # A three-high tower is outside what pick_stack builds and must be refused
    # rather than reported as another stackable tier.
    too_high = top_surface_at(0.090)
    assert tabletop_cube_tier(
        too_high, 0.030, 1.25, 0.0, 0.010,
        stacked_offsets_m=[0.030], stack_tolerance_m=0.012) is None

    # With stack-tier detection disabled the stacked cube is simply invisible,
    # which preserves the pre-stack behavior of the colour filter.
    assert tabletop_cube_tier(
        stacked, 0.030, 1.25, 0.0, 0.010,
        stacked_offsets_m=(), stack_tolerance_m=0.012) is None
    assert tabletop_cube_tier(
        on_table, 0.030, 1.25, 0.0, 0.010,
        stacked_offsets_m=(), stack_tolerance_m=0.012) == 0


def test_stacked_cube_tier_rejects_invalid_configuration_and_shapes():
    """A bad tier configuration or a non-cube footprint must never match."""
    info = camera_info(width=100, height=80)
    info.k = [200.0, 0.0, 50.0, 0.0, 200.0, 40.0, 0.0, 0.0, 1.0]
    width, height = 13, 26
    depth = np.full((80, 100), 0.47, dtype=np.float32)
    depth[34:34 + height, 44:44 + width] = 0.50
    mask = np.zeros_like(depth, dtype=np.uint8)
    mask[34:34 + height, 44:44 + width] = 255
    points, fraction = registered_points_from_mask(depth, info, info, mask)
    surface = extract_top_surface(
        transform_points(points, [0.1, -0.2, 0.0], [0, 0, 0, 1]),
        fraction, top_quantile=60.0, top_tolerance_m=0.002, min_top_points=20)

    with pytest.raises(ValueError):
        tabletop_cube_tier(surface, 0.030, 1.25, 0.0, 0.010,
                           stacked_offsets_m=[0.0], stack_tolerance_m=0.012)
    with pytest.raises(ValueError):
        tabletop_cube_tier(surface, 0.030, 1.25, 0.0, 0.010,
                           stacked_offsets_m=[0.030], stack_tolerance_m=0.0)
    # A 60 x 30 mm top is not a cube and must not be accepted at any tier.
    assert tabletop_cube_tier(surface, 0.030, 1.25, 0.0, 0.010,
                              stacked_offsets_m=[0.030],
                              stack_tolerance_m=0.012) is None


def test_registered_depth_back_projection_and_top_fit():
    info = camera_info(width=100, height=80)
    info.k = [200.0, 0.0, 50.0, 0.0, 200.0, 40.0, 0.0, 0.0, 1.0]
    depth = np.full((80, 100), 0.50, dtype=np.float32)
    mask = np.zeros_like(depth, dtype=np.uint8)
    mask[34:47, 44:57] = 255
    points, valid_fraction = registered_points_from_mask(depth, info, info, mask)
    points_base = transform_points(points, [0.1, -0.2, 0.0], [0.0, 0.0, 0.0, 1.0])
    surface = extract_top_surface(
        points_base, valid_fraction, top_quantile=60.0,
        top_tolerance_m=0.002, min_top_points=20)
    assert surface.center == pytest.approx([0.1, -0.2, 0.5], abs=0.002)
    assert surface.size_xy == pytest.approx([0.03, 0.03], abs=0.006)
    assert surface.depth_valid_fraction == 1.0


def test_top_surface_z_compensation_is_signed_and_preserves_quality():
    points = np.array([
        [0.10, -0.20, 0.035], [0.11, -0.20, 0.036],
        [0.10, -0.19, 0.035], [0.11, -0.19, 0.036],
    ])
    surface = extract_top_surface(
        points, 0.98, top_quantile=50.0,
        top_tolerance_m=0.010, min_top_points=4)
    corrected = compensate_top_surface_z(surface, -0.004)
    assert corrected.center[:2] == pytest.approx(surface.center[:2])
    assert corrected.center[2] == pytest.approx(surface.center[2] - 0.004)
    assert corrected.z_spread == surface.z_spread
    assert corrected.depth_valid_fraction == surface.depth_valid_fraction
    assert surface.center[2] == pytest.approx(0.0355)


def test_top_surface_z_compensation_rejects_nonfinite_value():
    surface = SimpleNamespace(center=np.array([0.1, -0.2, 0.035]))
    with pytest.raises(ValueError, match='invalid_top_surface_z_compensation'):
        compensate_top_surface_z(surface, float('nan'))


def test_top_fit_recovers_rotated_rectangle_long_axis_in_base_frame():
    yaw = np.deg2rad(32.0)
    short = np.linspace(-0.015, 0.015, 31)
    long = np.linspace(-0.030, 0.030, 61)
    local_long, local_short = np.meshgrid(long, short)
    local = np.column_stack((local_long.ravel(), local_short.ravel()))
    rotation = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)],
    ])
    xy = local @ rotation.T + np.array([0.20, -0.40])
    points = np.column_stack((xy, np.full(xy.shape[0], 0.035)))

    surface = extract_top_surface(
        points, 1.0, top_quantile=60.0,
        top_tolerance_m=0.002, min_top_points=100)

    axis_error = (surface.long_axis_yaw_rad - yaw + np.pi / 2) % np.pi - np.pi / 2
    assert axis_error == pytest.approx(0.0, abs=np.deg2rad(0.2))
    assert surface.size_xy == pytest.approx([0.030, 0.058], abs=0.001)
    assert surface.axis_ratio > 1.8


def test_top_fit_recovers_square_edge_modulo_ninety_degrees():
    yaw = np.deg2rad(27.0)
    edge = np.linspace(-0.015, 0.015, 41)
    local_x, local_y = np.meshgrid(edge, edge)
    local = np.column_stack((local_x.ravel(), local_y.ravel()))
    rotation = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)],
    ])
    xy = local @ rotation.T + np.array([-0.10, -0.45])
    points = np.column_stack((xy, np.full(xy.shape[0], 0.035)))

    surface = extract_top_surface(
        points, 1.0, top_quantile=60.0,
        top_tolerance_m=0.002, min_top_points=100)

    period = np.pi / 2
    edge_error = (surface.square_edge_yaw_rad - yaw + period / 2) % period - period / 2
    assert edge_error == pytest.approx(0.0, abs=np.deg2rad(0.2))
    assert surface.axis_ratio == pytest.approx(1.0, abs=0.02)


def test_rpy_quaternion_distance_detects_mismatched_online_tf():
    expected = rpy_to_quaternion(0.1, -0.2, 0.3)
    same_sign_flipped = -expected
    mismatch = rpy_to_quaternion(0.1, -0.2, 0.5)
    assert quaternion_angular_distance(expected, same_sign_flipped) < 1e-9
    assert quaternion_angular_distance(expected, mismatch) == pytest.approx(0.2, abs=0.01)
