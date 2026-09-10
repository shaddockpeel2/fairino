"""Conservative depth preprocessing for review and future grasp perception.

The functions in this module never fill holes.  A depth value is retained only
when the newest frame contains a valid measurement and a short stationary
window agrees with it.  This avoids turning visualization smoothing into fake
geometry that could later be consumed by a grasp planner.
"""
import math
import cv2
import numpy as np


def valid_depth_mask(depth_m, min_depth_m=0.3, max_depth_m=1.2):
    """Return finite, non-zero depth samples inside the configured workspace."""
    depth = np.asarray(depth_m)
    if depth.ndim != 2 or not 0 < min_depth_m < max_depth_m:
        raise ValueError('invalid_depth_or_range')
    return np.isfinite(depth) & (depth >= min_depth_m) & (depth <= max_depth_m)


def colorize_metric_depth(depth_m, min_depth_m=0.3, max_depth_m=1.2):
    """Colorize a fixed metric range; invalid/out-of-range samples stay black."""
    depth = np.asarray(depth_m, dtype=np.float32)
    valid = valid_depth_mask(depth, min_depth_m, max_depth_m)
    clipped = np.clip((depth - min_depth_m) / (max_depth_m - min_depth_m), 0.0, 1.0)
    scaled = np.nan_to_num(255.0 * (1.0 - clipped), nan=0.0).astype(np.uint8)
    color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color, valid


def colorize_adaptive_depth(depth_m, low_percentile=2.0, high_percentile=98.0):
    """Match the Windows retest preview: single-frame adaptive D2C color.

    This is a visualization only.  The returned scale makes the changing color
    range explicit; metric calculations must continue to use ``depth_m``.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if (depth.ndim != 2 or not 0 <= low_percentile < high_percentile <= 100):
        raise ValueError('invalid_adaptive_color_parameters')
    valid = np.isfinite(depth) & (depth > 0)
    color = np.zeros((*depth.shape, 3), dtype=np.uint8)
    if not np.any(valid):
        return color, valid, {'low_mm': None, 'high_mm': None}
    low_m, high_m = np.percentile(depth[valid], [low_percentile, high_percentile])
    scaled = np.clip((depth - low_m) * 255.0 / max(high_m - low_m, 0.001), 0, 255)
    color = cv2.applyColorMap(255 - np.nan_to_num(scaled).astype(np.uint8), cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color, valid, {'low_mm': float(low_m * 1000.0),
                          'high_mm': float(high_m * 1000.0)}


def temporal_consensus_depth(frames, min_depth_m=0.3, max_depth_m=1.2,
                             min_valid_fraction=0.6, max_span_m=0.015):
    """Return a short-window median only where stationary measurements agree.

    The latest frame must itself be valid, so an object that has disappeared
    cannot leave a stale median behind.  This filter is intended for a stopped
    robot and static target, not for tracking moving objects.
    """
    stack = np.asarray(frames, dtype=np.float32)
    if (stack.ndim != 3 or stack.shape[0] < 1 or
            not 0 < min_valid_fraction <= 1 or max_span_m < 0):
        raise ValueError('invalid_temporal_consensus_parameters')
    valid = np.stack([valid_depth_mask(frame, min_depth_m, max_depth_m) for frame in stack])
    # Sorting five short-window samples is substantially cheaper on RK3588 than
    # full-image nanpercentile.  Max-min is also more conservative than P10-P90.
    ordered = np.sort(np.where(valid, stack, np.inf), axis=0)
    required = max(1, int(math.ceil(stack.shape[0] * min_valid_fraction)))
    count = valid.sum(axis=0)
    index = np.maximum(count - 1, 0)[None, ...]
    maximum = np.take_along_axis(ordered, index, axis=0)[0]
    median_index = np.maximum((count - 1) // 2, 0)[None, ...]
    median = np.take_along_axis(ordered, median_index, axis=0)[0]
    span = np.full(maximum.shape, np.inf, dtype=np.float32)
    any_valid = count > 0
    span[any_valid] = maximum[any_valid] - ordered[0][any_valid]
    stable = (valid[-1] & (count >= required) & np.isfinite(median) &
              np.isfinite(span) & (span <= max_span_m))
    filtered = np.full(median.shape, np.nan, dtype=np.float32)
    filtered[stable] = median[stable]
    report = {
        'frames': int(stack.shape[0]),
        'required_valid_frames': required,
        'max_min_max_span_mm': float(max_span_m * 1000.0),
        'stable_fraction': float(stable.mean()),
    }
    return filtered, stable, report


def roi_depth_statistics(depth_m, u, v, radius, min_depth_m=0.3, max_depth_m=1.2):
    """Compute raw, auditable statistics for one square image ROI."""
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2 or radius < 1 or not np.isfinite([u, v]).all():
        raise ValueError('invalid_roi_parameters')
    height, width = depth.shape
    x, y = int(round(u)), int(round(v))
    if not (0 <= x < width and 0 <= y < height):
        raise ValueError('pixel_out_of_image')
    x0, x1 = max(0, x - radius), min(width, x + radius + 1)
    y0, y1 = max(0, y - radius), min(height, y + radius + 1)
    roi = depth[y0:y1, x0:x1]
    valid = valid_depth_mask(roi, min_depth_m, max_depth_m)
    values_mm = roi[valid] * 1000.0
    result = {
        'bounds_xyxy': [x0, y0, x1, y1],
        'total_pixels': int(roi.size),
        'valid_pixels': int(values_mm.size),
        'valid_fraction': float(valid.mean()) if roi.size else 0.0,
        'median_mm': None,
        'p05_mm': None,
        'p95_mm': None,
        'p05_p95_span_mm': None,
        'mad_sigma_mm': None,
    }
    if values_mm.size:
        median = float(np.median(values_mm))
        p05, p95 = (float(value) for value in np.percentile(values_mm, [5, 95]))
        result.update({
            'median_mm': median,
            'p05_mm': p05,
            'p95_mm': p95,
            'p05_p95_span_mm': p95 - p05,
            'mad_sigma_mm': float(np.median(np.abs(values_mm - median)) * 1.4826),
        })
    return result


def depth_discontinuity_mask(depth_m, min_depth_m=0.3, max_depth_m=1.2,
                             absolute_jump_m=0.02, relative_jump=0.03):
    """Mark supported depth jumps while excluding hole boundaries and speckle."""
    depth = np.asarray(depth_m, dtype=np.float32)
    valid = valid_depth_mask(depth, min_depth_m, max_depth_m)
    if absolute_jump_m <= 0 or relative_jump <= 0:
        raise ValueError('invalid_edge_threshold')
    safe = np.where(valid, depth, 0.0)
    gx = cv2.Sobel(safe, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(safe, cv2.CV_32F, 0, 1, ksize=3)
    # Requiring a full 3x3 valid neighbourhood keeps zero-depth hole borders
    # from dominating the overlay.
    supported = cv2.erode(valid.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    threshold = np.maximum(absolute_jump_m, depth * relative_jump) * 4.0
    return supported & (np.hypot(gx, gy) > threshold)
