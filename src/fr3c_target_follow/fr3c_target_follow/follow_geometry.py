"""Time-aligned transforms and bounded position PBVS preview, independent of ROS."""
from collections import deque
import math
import cv2
import numpy as np
from .marker_pose import quaternion_xyzw


def unit_quaternion(value):
    q = np.asarray(value, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError('invalid_quaternion')
    return q / norm


def rotation_from_quaternion(value):
    x, y, z, w = unit_quaternion(value)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion_from_rotation(rotation):
    return quaternion_xyzw(cv2.Rodrigues(np.asarray(rotation, np.float64))[0])


def transform(position, quaternion):
    p = np.asarray(position, np.float64).reshape(3)
    if not np.isfinite(p).all():
        raise ValueError('invalid_position')
    result = np.eye(4)
    result[:3, :3] = rotation_from_quaternion(quaternion)
    result[:3, 3] = p
    return result


def transform_rpy(position, rpy):
    roll, pitch, yaw = np.asarray(rpy, np.float64)
    if not np.isfinite([roll, pitch, yaw]).all():
        raise ValueError('invalid_rpy')
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return transform(position, quaternion_from_rotation(rz @ ry @ rx))


def slerp(first, second, fraction):
    a, b = unit_quaternion(first), unit_quaternion(second)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    dot = min(dot, 1.0)
    if dot > .9995:
        return unit_quaternion(a + fraction*(b-a))
    angle = math.acos(dot)
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b) / math.sin(angle)


class PoseHistory:
    def __init__(self, duration_s=3.0):
        self.samples = deque()
        self.duration_s = duration_s

    def add(self, stamp_s, position, quaternion):
        if not math.isfinite(stamp_s) or stamp_s <= 0:
            raise ValueError('invalid_feedback_timestamp')
        matrix = transform(position, quaternion)
        if self.samples and stamp_s <= self.samples[-1][0]:
            raise ValueError('feedback_timestamp_not_increasing')
        self.samples.append((stamp_s, matrix[:3, 3], unit_quaternion(quaternion)))
        while len(self.samples) > 2 and self.samples[1][0] < stamp_s-self.duration_s:
            self.samples.popleft()

    def at(self, stamp_s, max_gap_s=.05):
        if not self.samples or stamp_s < self.samples[0][0]:
            raise ValueError('image_before_feedback_history')
        if stamp_s > self.samples[-1][0]:
            raise ValueError('waiting_for_feedback_bracket')
        previous = self.samples[0]
        for following in self.samples:
            if following[0] == stamp_s:
                return transform(following[1], following[2]), 0.0
            if following[0] > stamp_s:
                gap = following[0]-previous[0]
                if gap > max_gap_s:
                    raise ValueError('feedback_interpolation_gap')
                weight = (stamp_s-previous[0])/gap
                return transform(previous[1]+weight*(following[1]-previous[1]),
                                 slerp(previous[2], following[2], weight)), gap
            previous = following
        raise ValueError('waiting_for_feedback_bracket')

    def latest(self):
        if not self.samples:
            raise ValueError('waiting_for_robot_feedback')
        stamp, p, q = self.samples[-1]
        return stamp, transform(p, q)


def clip_norm(vector, limit):
    length = float(np.linalg.norm(vector))
    return vector if length <= limit or length == 0 else vector * limit/length


class PositionPreview:
    """A fixed base-frame offset, locked TCP rotation, and an acceleration-limited v.

    reference is a single step from measured TCP, not an accumulated simulated robot.
    This preview never substitutes its reference for real robot feedback.
    """
    def __init__(self, kp=1.5, max_speed_m_s=.01, max_accel_m_s2=.03):
        if not np.isfinite([kp, max_speed_m_s, max_accel_m_s2]).all() or min(kp, max_speed_m_s, max_accel_m_s2) <= 0:
            raise ValueError('invalid_preview_limits')
        self.kp, self.max_speed, self.max_accel = kp, max_speed_m_s, max_accel_m_s2
        self.offset = self.rotation = None
        self.velocity = np.zeros(3)

    def teach(self, target_position, tcp):
        self.offset = tcp[:3, 3] - np.asarray(target_position, np.float64)
        self.rotation = tcp[:3, :3].copy()
        self.stop()

    def stop(self):
        self.velocity = np.zeros(3)

    def step(self, target_position, tcp, dt):
        if self.offset is None:
            raise ValueError('offset_not_recorded')
        if not math.isfinite(dt) or not 0 < dt <= .05:
            self.stop()
            raise ValueError('preview_period_gap')
        goal = np.eye(4)
        goal[:3, :3] = self.rotation
        goal[:3, 3] = np.asarray(target_position, np.float64) + self.offset
        error = goal[:3, 3] - tcp[:3, 3]
        requested = clip_norm(self.kp*error, self.max_speed)
        self.velocity += clip_norm(requested-self.velocity, self.max_accel*dt)
        reference = goal.copy()
        reference[:3, 3] = tcp[:3, 3] + self.velocity*dt
        return goal, reference, error, self.velocity.copy()
