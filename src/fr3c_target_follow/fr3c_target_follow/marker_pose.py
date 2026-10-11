"""OpenCV-only marker measurement. T_camera_tag, metres, quaternion xyzw.

Tag frame: centre of the top label, X right / Y up in its decoded orientation,
Z out of its front face. Optical camera: X right, Y down, Z forward.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np


def square_points(size_m):
    if not math.isfinite(size_m) or size_m <= 0:
        raise ValueError('marker_size_m must be positive and measured')
    h = size_m / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)


def rotation_distance_deg(first, second):
    cosine = np.clip((np.trace(first.T @ second) - 1) / 2, -1, 1)
    return math.degrees(math.acos(float(cosine)))


def quaternion_xyzw(rvec):
    v = np.asarray(rvec, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(v))
    xyz = v * (math.sin(angle / 2) / angle) if angle > 1e-12 else v / 2
    q = np.r_[xyz, math.cos(angle / 2)]
    return q / np.linalg.norm(q)


def rpy_degrees(rotation):
    r = rotation
    pitch = math.asin(float(np.clip(-r[2, 0], -1, 1)))
    if abs(math.cos(pitch)) > 1e-8:
        roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = math.atan2(-r[1, 2], r[1, 1]), 0.0
    return np.degrees([roll, pitch, yaw]).tolist()


def camera_parameters(info, image_shape, image_frame):
    if (info.height, info.width) != tuple(image_shape[:2]):
        raise ValueError('CameraInfo and image dimensions differ')
    if not image_frame or info.header.frame_id != image_frame:
        raise ValueError('CameraInfo and image frames differ')
    if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
        raise ValueError('CameraInfo ROI/binning requires adjusted intrinsics')
    k = np.array(info.k, np.float64).reshape(3, 3)
    d = np.array(info.d, np.float64)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or k[2, 2] != 1:
        raise ValueError('invalid camera calibration')
    if len(d) and info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError('unsupported distortion model: ' + info.distortion_model)
    if len(d) not in (0, 4, 5, 8, 12, 14):
        raise ValueError('unsupported distortion coefficient count')
    return k, d


def stamp_is_fresh(stamp_s, now_s, max_age_s):
    return stamp_s > 0 and -0.05 <= now_s - stamp_s <= max_age_s


def candidate_orientation_quality(solver, separation_deg, margin_px, selected_index,
                                  ambiguity_margin_px=.15, tolerance_deg=5.):
    """Explicit approximate policy; candidate selection and raw poses stay intact."""
    if not math.isfinite(tolerance_deg) or not 0 <= tolerance_deg <= 45:
        raise ValueError('invalid_orientation_candidate_tolerance')
    ambiguous = (solver != 'IPPE_SQUARE' or
                 (margin_px is not None and margin_px < ambiguity_margin_px and
                  separation_deg > tolerance_deg))
    valid = not ambiguous and (selected_index == 0 or
                              (separation_deg is not None and separation_deg <= tolerance_deg))
    approximate = bool(valid and separation_deg is not None and separation_deg > 5 and
                       (selected_index != 0 or margin_px < ambiguity_margin_px))
    return dict(orientation_ambiguous=bool(ambiguous), orientation_measurement_valid=bool(valid),
                orientation_approximate=approximate, orientation_candidate_tolerance_deg=float(tolerance_deg))


@dataclass
class TagPose:
    rvec: np.ndarray
    position_m: np.ndarray
    rotation: np.ndarray
    quaternion: np.ndarray
    reprojection_error_px: float
    min_edge_px: float
    candidate_count: int
    candidate_margin_px: float | None
    orientation_ambiguous: bool
    solver: str
    candidate_errors_px: list[float]
    candidate_rotation_distance_deg: float | None
    selected_candidate_index: int


class MarkerDetector:
    def __init__(self, dictionary_name, adaptive_thresh_constant=13.0, adaptive_thresh_step=4,
                 fallback_thresh_constants=(21.0,25.0), target_id=None):
        aruco = getattr(cv2, 'aruco', None)
        if aruco is None or not dictionary_name.startswith('DICT_') or not hasattr(aruco, dictionary_name):
            raise ValueError('unsupported marker dictionary: ' + dictionary_name)
        self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
        constants = list(dict.fromkeys([adaptive_thresh_constant, *fallback_thresh_constants]))
        if not all(math.isfinite(value) for value in constants) or adaptive_thresh_step < 1:
            raise ValueError('invalid adaptive threshold parameters')
        if target_id is not None and not 0 <= target_id < len(self.dictionary.bytesList):
            raise ValueError('target_id is outside the dictionary')
        self.target_id = target_id
        self.passes = []
        for constant in constants:
            parameters = (aruco.DetectorParameters() if hasattr(aruco, 'ArucoDetector')
                          else aruco.DetectorParameters_create())
            parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            # A 20 mm tag is only ~30 px here; a 5 px window can cross several modules.
            parameters.cornerRefinementWinSize = 2
            parameters.cornerRefinementMaxIterations = 30
            parameters.cornerRefinementMinAccuracy = 0.01
            parameters.adaptiveThreshConstant = constant
            parameters.adaptiveThreshWinSizeStep = adaptive_thresh_step
            detector = (aruco.ArucoDetector(self.dictionary, parameters)
                        if hasattr(aruco, 'ArucoDetector') else None)
            self.passes.append((float(constant), parameters, detector))
        self.parameters, self.detector = self.passes[0][1:]
        self.last_threshold = None
        self.last_pass_count = 0

    def detect(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        best = None
        best_count = -1
        best_threshold = None
        self.last_pass_count = 0
        for constant, parameters, detector in self.passes:
            result = (detector.detectMarkers(gray) if detector is not None else
                      cv2.aruco.detectMarkers(gray, self.dictionary, parameters=parameters))
            self.last_pass_count += 1
            ids = result[1]
            count = 0 if ids is None else len(ids)
            if count > best_count:
                best, best_count, best_threshold = result, count, constant
            if ids is not None and (self.target_id is None or np.any(ids == self.target_id)):
                self.last_threshold = constant
                return result
        self.last_threshold = best_threshold
        return best


def solve_marker_pose(corners, size_m, k, d, *, min_edge_px=12.0,
                      max_error_px=2.0, previous_rotation=None,
                      continuity_margin_px=0.20, ambiguity_margin_px=0.15,
                      orientation_candidate_tolerance_deg=5.):
    if not math.isfinite(orientation_candidate_tolerance_deg) or not 0 <= orientation_candidate_tolerance_deg <= 45:
        raise ValueError('invalid_orientation_candidate_tolerance')
    object_points = square_points(size_m)
    image_points = np.ascontiguousarray(corners, dtype=np.float64).reshape(4, 2)
    if not np.isfinite(image_points).all() or not cv2.isContourConvex(image_points.astype(np.float32)):
        raise ValueError('invalid marker quadrilateral')
    edges = np.linalg.norm(image_points - np.roll(image_points, -1, axis=0), axis=1)
    if edges.min() < min_edge_px:
        raise ValueError('marker_too_small')
    result = cv2.solvePnPGeneric(object_points, image_points, k, d,
                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    solver = 'IPPE_SQUARE'

    def consider(rvec, tvec):
        rotation = cv2.Rodrigues(rvec)[0]
        position = tvec.reshape(3)
        if not np.isfinite(rotation).all() or not np.isfinite(position).all():
            return
        camera_points = (rotation @ object_points.T).T + position
        if camera_points[:, 2].min() <= 0:
            return
        projected = cv2.projectPoints(object_points, rvec, tvec, k, d)[0].reshape(4, 2)
        error = float(np.sqrt(np.mean(np.sum((projected - image_points)**2, axis=1))))
        if math.isfinite(error) and error <= max_error_px:
            candidates.append((error, rvec.reshape(3), position, rotation))

    if result[0]:
        for rvec, tvec in zip(result[1], result[2]):
            consider(rvec, tvec)
    if not candidates:
        # OpenCV 4.5 IPPE can degenerate for an exactly front-facing centred tag.
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, k, d,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
        if success:
            consider(rvec, tvec)
            solver = 'ITERATIVE_FALLBACK'
    if not candidates:
        raise ValueError('no_valid_pose')
    candidates.sort(key=lambda c: c[0])
    eligible = [c for c in candidates if c[0] <= candidates[0][0] + continuity_margin_px]
    chosen = (min(eligible, key=lambda c: rotation_distance_deg(previous_rotation, c[3]))
              if previous_rotation is not None else candidates[0])
    margin = candidates[1][0] - candidates[0][0] if len(candidates) > 1 else None
    separation = rotation_distance_deg(candidates[0][3], candidates[1][3]) if len(candidates) > 1 else None
    selected_index = next(i for i, c in enumerate(candidates) if c is chosen)
    ambiguous = candidate_orientation_quality(solver, separation, margin, selected_index,
        ambiguity_margin_px, orientation_candidate_tolerance_deg)['orientation_ambiguous']
    error, rvec, position, rotation = chosen
    return TagPose(rvec, position, rotation, quaternion_xyzw(rvec), error,
                   float(edges.min()), len(candidates), margin, ambiguous, solver,
                   [c[0] for c in candidates], separation,
                   selected_index)
