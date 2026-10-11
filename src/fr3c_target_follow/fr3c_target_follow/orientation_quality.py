"""Exact image-stamp pairing and base-frame orientation checks, without ROS/SDK."""
from collections import OrderedDict
import math

import numpy as np

from .marker_pose import rotation_distance_deg


def stamp_ns(stamp):
    """Require original integer ROS stamps; an epoch float loses nanoseconds."""
    if not isinstance(stamp, dict):
        raise ValueError('missing_exact_image_stamp')
    sec, ns = stamp.get('sec'), stamp.get('nanosec')
    if type(sec) is not int or type(ns) is not int or sec < 0 or not 0 <= ns < 10**9:
        raise ValueError('invalid_image_stamp')
    value = sec * 10**9 + ns
    if value <= 0:
        raise ValueError('invalid_image_stamp')
    return value


def measurement_problem(quality):
    if not isinstance(quality.get('status'), str):
        return 'invalid_quality_status'
    if quality.get('status') != 'valid':
        return quality.get('status', 'invalid_quality')
    if quality.get('valid_position') is not True:
        return 'missing_position_quality'
    if quality.get('orientation_ambiguous') is True:
        return 'orientation_ambiguous'
    if quality.get('orientation_ambiguous') is not False:
        return 'missing_orientation_quality'
    if quality.get('orientation_measurement_valid') is not True:
        return 'orientation_measurement_rejected'
    if quality.get('solver') == 'ITERATIVE_FALLBACK':
        return 'orientation_solver_fallback'
    if quality.get('solver') != 'IPPE_SQUARE':
        return 'unknown_pose_solver'
    errors = quality.get('candidate_errors_px')
    count = quality.get('candidate_count')
    selected = quality.get('selected_candidate_index')
    if (type(count) is not int or count < 1 or not isinstance(errors, list) or len(errors) != count or
            type(selected) is not int or not 0 <= selected < count or
            not all(type(e) in (int, float) and math.isfinite(e) and e >= 0 for e in errors)):
        return 'invalid_candidate_quality'
    separation = quality.get('candidate_rotation_distance_deg')
    if count > 1 and (type(separation) not in (int, float) or
                      not math.isfinite(separation) or not 0 <= separation <= 180):
        return 'invalid_candidate_quality'
    tolerance = quality.get('orientation_candidate_tolerance_deg', 5.)
    if type(tolerance) not in (int, float) or not math.isfinite(tolerance) or not 0 <= tolerance <= 45:
        return 'invalid_orientation_candidate_tolerance'
    if selected != 0 and separation > tolerance:
        return 'continuity_selected_alternative'
    if selected != 0 and separation > 5 and quality.get('orientation_approximate') is not True:
        return 'missing_approximate_orientation_policy'
    return None


class ExactPosePairer:
    """Bounded independent arrivals; only identical integer stamps can pair."""
    def __init__(self, capacity=32):
        self.capacity = capacity
        self.poses, self.qualities = OrderedDict(), OrderedDict()
        self.last_paired_ns = 0
        self.paired = self.evicted = self.expired = self.late = 0

    def add(self, kind, stamp, value):
        ns = stamp_ns(stamp)
        if ns <= self.last_paired_ns:
            self.late += 1
            return None
        own, other = (self.poses, self.qualities) if kind == 'pose' else (self.qualities, self.poses)
        if ns in own:
            self.late += 1
            return None
        own[ns] = value
        if ns not in other:
            while len(own) > self.capacity:
                own.popitem(last=False)
                self.evicted += 1
            return None
        pose, quality = self.poses.pop(ns), self.qualities.pop(ns)
        self.last_paired_ns = ns
        self.paired += 1
        self.discard_before(ns)
        return ns, pose, quality

    def discard_before(self, ns):
        for cache in (self.poses, self.qualities):
            for old in list(cache):
                if old <= ns:
                    del cache[old]
                    self.expired += 1

    def expire(self, now_ns, timeout_s):
        self.discard_before(now_ns - int(timeout_s * 10**9))

    def stats(self):
        return dict(paired=self.paired, evicted=self.evicted, expired=self.expired, late=self.late,
                    pending_poses=len(self.poses), pending_qualities=len(self.qualities))


class OrientationGate:
    """Quality precedes continuity. Rejected spikes never replace the good anchor."""
    def __init__(self, confirmation_frames=5, timeout_s=.35,
                 max_angular_speed_deg_s=60., jitter_allowance_deg=3.):
        if type(confirmation_frames) is not int or confirmation_frames < 1:
            raise ValueError('invalid_confirmation_frames')
        if (not np.isfinite([timeout_s, max_angular_speed_deg_s, jitter_allowance_deg]).all() or
                timeout_s <= 0 or max_angular_speed_deg_s <= 0 or jitter_allowance_deg < 0):
            raise ValueError('invalid_orientation_limits')
        self.confirmation_frames = confirmation_frames
        self.timeout_s = timeout_s
        self.max_speed = max_angular_speed_deg_s
        self.jitter = jitter_allowance_deg
        self.last_ns = 0
        self.raw_rotation = None
        self.good_rotation = None
        self.good_ns = 0
        self.confirmed = 0

    def reset(self):
        self.good_rotation = None
        self.good_ns = 0
        self.confirmed = 0
        self.raw_rotation = None

    def observe(self, ns, rotation, problem=None):
        if ns <= self.last_ns:
            raise ValueError('orientation_timestamp_not_increasing')
        r = np.asarray(rotation, dtype=np.float64)
        if (r.shape != (3, 3) or not np.isfinite(r).all() or
                not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or np.linalg.det(r) < .999):
            raise ValueError('invalid_rotation')
        dt = (ns - self.last_ns) / 1e9 if self.last_ns else None
        delta = rotation_distance_deg(self.raw_rotation, r) if self.raw_rotation is not None else None
        result = dict(valid_orientation=False, status='acquiring_orientation',
                      orientation_delta_deg=delta, angular_speed_deg_s=delta/dt if delta is not None else None,
                      continuity_frame='base_link', confirmation_count=0,
                      confirmation_frames=self.confirmation_frames)
        self.last_ns, self.raw_rotation = ns, r.copy()
        if problem:
            self.confirmed = 0
            result['status'] = problem
        else:
            good_dt = (ns - self.good_ns) / 1e9 if self.good_ns else None
            if good_dt is not None and good_dt > self.timeout_s:
                self.good_rotation = None
                self.confirmed = 0
            if self.good_rotation is not None:
                change = rotation_distance_deg(self.good_rotation, r)
                bound = self.jitter + self.max_speed * good_dt
                result.update(accepted_anchor_delta_deg=change, allowed_delta_deg=bound)
                if change > bound:
                    self.confirmed = 0
                    result['status'] = 'orientation_jump'
                    return result
            self.good_rotation, self.good_ns = r.copy(), ns
            self.confirmed = min(self.confirmation_frames, self.confirmed + 1)
            result['valid_orientation'] = self.confirmed == self.confirmation_frames
            result['status'] = 'valid' if result['valid_orientation'] else 'acquiring_orientation'
        result['confirmation_count'] = self.confirmed
        return result
