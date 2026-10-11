"""Filtered full-relative-pose preview. Pure geometry; no ROS or robot interface."""
import math

import cv2
import numpy as np

from .follow_geometry import clip_norm


def rotation_vector(rotation):
    return cv2.Rodrigues(np.asarray(rotation, np.float64))[0].reshape(3)


def rotation_exp(vector):
    return cv2.Rodrigues(np.asarray(vector, np.float64).reshape(3))[0]


class PosePreview:
    def __init__(self, position_tau_s=.15, rotation_tau_s=.4,
                 deadband_enter_deg=3., deadband_exit_deg=2., rotation_mapping_gain=1.,
                 kp_position=2., kp_rotation=1., max_speed_m_s=.02, max_accel_m_s2=.06,
                 max_angular_speed_deg_s=3., max_angular_accel_deg_s2=10.,
                 target_timeout_s=.35):
        values = [position_tau_s, rotation_tau_s, deadband_enter_deg, deadband_exit_deg,
                  rotation_mapping_gain, kp_position, kp_rotation, max_speed_m_s, max_accel_m_s2,
                  max_angular_speed_deg_s, max_angular_accel_deg_s2, target_timeout_s]
        if not np.isfinite(values).all() or min(values) < 0:
            raise ValueError('invalid_pose_preview_parameters')
        if (deadband_exit_deg > deadband_enter_deg or deadband_enter_deg >= 180 or
                not 0 < rotation_mapping_gain <= 1 or
                min(values[5:]) <= 0):
            raise ValueError('invalid_pose_preview_parameters')
        self.position_tau, self.rotation_tau = position_tau_s, rotation_tau_s
        self.enter, self.exit = np.radians([deadband_enter_deg, deadband_exit_deg])
        self.mapping_gain = rotation_mapping_gain
        self.kp_position, self.kp_rotation = kp_position, kp_rotation
        self.max_speed, self.max_accel = max_speed_m_s, max_accel_m_s2
        self.max_angular_speed = math.radians(max_angular_speed_deg_s)
        self.max_angular_accel = math.radians(max_angular_accel_deg_s2)
        self.timeout_s = target_timeout_s
        self.raw = self.filtered = self.accepted_rotation = None
        self.initial_rotation = self.offset = None
        self.last_ns = 0
        self.rotation_active = False
        self.velocity, self.omega = np.zeros(3), np.zeros(3)

    @staticmethod
    def matrix(value):
        value = np.asarray(value, np.float64)
        if (value.shape != (4, 4) or not np.isfinite(value).all() or
                not np.allclose(value[3], [0, 0, 0, 1]) or
                not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6) or
                np.linalg.det(value[:3, :3]) < .999):
            raise ValueError('invalid_pose_transform')
        return value.copy()

    def seed(self, ns, target):
        if type(ns) is not int or ns <= 0:
            raise ValueError('invalid_pose_timestamp')
        target = self.matrix(target)
        self.last_ns = ns
        self.raw = target.copy()
        self.filtered = target.copy()
        self.accepted_rotation = target[:3, :3].copy()
        self.rotation_active = False

    def observe(self, ns, target):
        if type(ns) is not int or ns <= self.last_ns:
            raise ValueError('pose_timestamp_not_increasing')
        target = self.matrix(target)
        dt = (ns-self.last_ns)/1e9 if self.last_ns else None
        if dt is None:
            self.seed(ns, target)
            return
        if dt > self.timeout_s:
            # Missing observations are not evidence of continuous target motion.
            # Resume filtering from the previous target and deadband anchor.
            dt = min(dt, .1)
        self.raw = target.copy()
        self.last_ns = ns
        alpha_p = -math.expm1(-dt/self.position_tau) if self.position_tau else 1.
        alpha_r = -math.expm1(-dt/self.rotation_tau) if self.rotation_tau else 1.
        self.filtered[:3, 3] += alpha_p*(target[:3, 3]-self.filtered[:3, 3])
        change = rotation_vector(target[:3, :3] @ self.filtered[:3, :3].T)
        self.filtered[:3, :3] = rotation_exp(change*alpha_r) @ self.filtered[:3, :3]
        delta = rotation_vector(self.filtered[:3, :3] @ self.accepted_rotation.T)
        angle = float(np.linalg.norm(delta))
        if self.enter == 0:
            self.accepted_rotation = self.filtered[:3, :3].copy()
            self.rotation_active = False
            return
        if angle >= self.enter:
            self.rotation_active = True
        elif angle <= self.exit + 1e-12:
            self.rotation_active = False
        if self.rotation_active and angle > self.exit:
            # Leave the allowed residual instead of jumping to every noisy input.
            self.accepted_rotation = rotation_exp(delta*((angle-self.exit)/angle)) @ self.accepted_rotation

    def teach(self, ns, target, actual_tcp):
        actual_tcp = self.matrix(actual_tcp)
        self.seed(ns, target)
        self.initial_rotation = self.filtered[:3, :3].copy()
        self.offset = np.linalg.inv(self.filtered) @ actual_tcp
        self.stop()

    def mapped_target(self):
        if self.offset is None:
            raise ValueError('offset_not_recorded')
        mapped = self.filtered.copy()
        delta = rotation_vector(self.accepted_rotation @ self.initial_rotation.T)
        mapped[:3, :3] = rotation_exp(delta*self.mapping_gain) @ self.initial_rotation
        return mapped

    def goal(self):
        return self.mapped_target() @ self.offset

    def stop(self):
        self.velocity[:] = 0.
        self.omega[:] = 0.

    def step(self, actual_tcp, dt):
        if not math.isfinite(dt) or not 0 < dt <= .1:
            self.stop()
            raise ValueError('preview_period_gap')
        actual_tcp = self.matrix(actual_tcp)
        goal = self.goal()
        error = goal[:3, 3]-actual_tcp[:3, 3]
        angular_error = rotation_vector(goal[:3, :3] @ actual_tcp[:3, :3].T)
        requested_v = clip_norm(self.kp_position*error, self.max_speed)
        requested_w = clip_norm(self.kp_rotation*angular_error, self.max_angular_speed)
        self.velocity += clip_norm(requested_v-self.velocity, self.max_accel*dt)
        self.omega += clip_norm(requested_w-self.omega, self.max_angular_accel*dt)
        reference = actual_tcp.copy()
        reference[:3, 3] += self.velocity*dt
        reference[:3, :3] = rotation_exp(self.omega*dt) @ actual_tcp[:3, :3]
        return dict(goal=goal, reference=reference, position_error=error,
                    angular_error=angular_error, velocity=self.velocity.copy(), omega=self.omega.copy())
