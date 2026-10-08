"""Independent 2-D perception adapters. No ROS, robot or motion interfaces."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import time

import cv2
import numpy as np

from .cube_perception import find_color_candidates
from .rknn_detector import CLASS_NAMES, RKNNDetector


WASTE_CLASSES = {
    'recyclable': ('Newspaper', 'Zip_top_can', 'Book', 'Old_school_bag'),
    'kitchen': ('Fish_bone', 'Egg_shell', 'Apple_core', 'Watermelon_rind'),
    'hazardous': ('Syringe', 'Expired_cosmetics', 'Used_batteries', 'Expired_tablets'),
    'other': ('Toilet_paper', 'Peach_pit', 'Cigarette_butts', 'Disposable_chopsticks'),
}
CATEGORY_LABELS = {
    'recyclable': '可回收物', 'kitchen': '厨余垃圾',
    'hazardous': '有害垃圾', 'other': '其他垃圾',
}
OBJECT_LABELS = dict(zip(CLASS_NAMES, (
    '苹果核', '书本', '烟头', '一次性筷子', '蛋壳', '过期化妆品',
    '过期药品', '鱼骨', '报纸', '旧书包', '桃核', '注射器',
    '卫生纸', '废电池', '西瓜皮', '易拉罐')))
COLOR_LABELS = {'red': '红色', 'yellow': '黄色', 'blue': '蓝色', 'green': '绿色'}
COLOR_BGR = {'red': (0, 0, 255), 'yellow': (0, 220, 255),
             'blue': (255, 100, 0), 'green': (0, 200, 0)}


def waste_category(name: str) -> str:
    for category, names in WASTE_CLASSES.items():
        if name in names:
            return category
    raise ValueError(f'unmapped model class: {name}')


def box_center(box):
    x1, y1, x2, y2 = box
    return ((x1 + x2) / 2, (y1 + y2) / 2)


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    overlap = max(0, x2 - x1) * max(0, y2 - y1)
    union = ((a[2] - a[0]) * (a[3] - a[1])
             + (b[2] - b[0]) * (b[3] - b[1]) - overlap)
    return overlap / union if union > 0 else 0.0


class SingleFaceTracker:
    """Largest-face acquisition, gated association, smoothing and time-based loss.

    IDs represent a continuous track, not a person's identity. Never jump to
    an unrelated face while the selected track is temporarily missing.
    """

    def __init__(self, lost_after_s=0.8, smoothing=0.65, trail_length=40):
        self.lost_after_s = lost_after_s
        self.smoothing = smoothing
        self.trail = deque(maxlen=trail_length)
        self.next_id = 1
        self.reset()

    def reset(self):
        self.track_id = None
        self.box = None
        self.last_seen = None
        self.last_update = None
        self.velocity = np.zeros(2)
        self.trail.clear()

    def update(self, detections, now):
        if self.last_seen is not None and now - self.last_seen > self.lost_after_s:
            self.reset()
        match = None
        if self.box is None and detections:
            match = max(detections, key=lambda d:
                        (d['bbox_xyxy'][2] - d['bbox_xyxy'][0]) *
                        (d['bbox_xyxy'][3] - d['bbox_xyxy'][1]))
            self.track_id = self.next_id
            self.next_id += 1
        elif self.box is not None:
            elapsed = max(0, now - self.last_seen)
            shift = self.velocity * min(elapsed, self.lost_after_s)
            predicted = self.box + np.tile(shift, 2)
            center = np.array(box_center(predicted))
            diagonal = np.linalg.norm(predicted[2:] - predicted[:2])
            best_score = -1.0
            for detection in detections:
                box = detection['bbox_xyxy']
                distance = np.linalg.norm(np.array(box_center(box)) - center)
                overlap = box_iou(predicted, box)
                size_ratio = ((box[2] - box[0]) * (box[3] - box[1]) /
                              max(1, (predicted[2] - predicted[0]) *
                                  (predicted[3] - predicted[1])))
                if not 0.25 <= size_ratio <= 4 or distance > max(25, diagonal * 0.65):
                    continue
                score = overlap - distance / max(diagonal, 1)
                if score > best_score:
                    best_score, match = score, detection
        if match is None:
            return {'state': 'temporarily_lost' if self.box is not None else 'searching',
                    'track_id': self.track_id, 'trail': list(self.trail),
                    'targets': []}
        measured = np.asarray(match['bbox_xyxy'], dtype=float)
        if self.box is not None:
            dt = max(0.001, now - self.last_seen)
            velocity = (np.array(box_center(measured)) -
                        np.array(box_center(self.box))) / dt
            self.velocity = self.velocity * 0.5 + velocity * 0.5
            self.box = self.box * (1 - self.smoothing) + measured * self.smoothing
        else:
            self.box = measured
        self.last_seen = now
        center = box_center(self.box)
        self.trail.append(tuple(map(float, center)))
        target = {**match, 'bbox_xyxy': self.box.tolist(),
                  'center_uv': list(center), 'track_id': self.track_id,
                  'label': f'人脸 #{self.track_id}'}
        return {'state': 'tracking', 'track_id': self.track_id,
                'trail': list(self.trail), 'targets': [target]}


class EdgePerception:
    def __init__(self, config):
        self.config = config
        self._objects = None
        self._faces = None
        self.tracker = SingleFaceTracker(**config.get('tracking', {}))

    def prepare(self, mode):
        if mode == 'objects' and self._objects is None:
            self._objects = RKNNDetector(**self.config['objects'])
        elif mode == 'faces' and self._faces is None:
            path = Path(self.config['faces']['model_path'])
            if not path.is_file():
                raise FileNotFoundError(f'YuNet model missing: {path}')
            self._faces = cv2.FaceDetectorYN.create(
                str(path), '', (320, 320),
                float(self.config['faces'].get('confidence_threshold', 0.75)),
                0.3, 5000, cv2.dnn.DNN_BACKEND_OPENCV, cv2.dnn.DNN_TARGET_CPU)

    def reset_tracking(self):
        self.tracker.reset()

    def process(self, mode, bgr, now=None):
        started = time.monotonic()
        self.prepare(mode)
        result = {'mode': mode, 'targets': [], 'counts': {}, 'state': 'running'}
        annotated = bgr.copy()
        if mode == 'objects':
            for detection in self._objects.infer(bgr):
                category = waste_category(detection.class_name)
                result['targets'].append({
                    'class_id': detection.class_id, 'class_name': detection.class_name,
                    'label': OBJECT_LABELS[detection.class_name],
                    'category': category, 'category_label': CATEGORY_LABELS[category],
                    'confidence': float(detection.confidence),
                    'bbox_xyxy': list(detection.bbox_xyxy),
                    'center_uv': list(detection.center_uv)})
                result['counts'][category] = result['counts'].get(category, 0) + 1
            result['timings_ms'] = dict(self._objects.last_timings_ms)
        elif mode == 'colors':
            settings = self.config['colors']
            matches = find_color_candidates(
                bgr, settings['intervals'], **settings['segmentation'],
                **settings['candidate_filter'])
            for color, candidate in matches:
                x, y, width, height = cv2.boundingRect(candidate.contour)
                result['targets'].append({
                    'color': color, 'label': COLOR_LABELS[color],
                    'bbox_xyxy': [x, y, x + width, y + height],
                    'center_uv': list(candidate.center_px),
                    'area_px': candidate.area_px})
                result['counts'][color] = result['counts'].get(color, 0) + 1
                cv2.drawContours(annotated, [candidate.contour], -1, COLOR_BGR[color], 2)
        elif mode == 'faces':
            height, width = bgr.shape[:2]
            scale = min(1.0, 320 / max(height, width))
            small = cv2.resize(bgr, (max(1, round(width * scale)),
                                    max(1, round(height * scale))))
            self._faces.setInputSize((small.shape[1], small.shape[0]))
            _, faces = self._faces.detect(small)
            detections = []
            for face in ([] if faces is None else faces):
                x, y, w, h = face[:4] / scale
                detections.append({'bbox_xyxy': [float(x), float(y), float(x+w), float(y+h)],
                                   'confidence': float(face[-1])})
            result.update(self.tracker.update(
                detections, time.monotonic() if now is None else now))
            result['faces_detected'] = len(detections)
            trail = result['trail']
            if result['state'] == 'tracking' and len(trail) > 1:
                cv2.polylines(annotated, [np.asarray(trail, np.int32)], False,
                              (255, 180, 0), 2)
        else:
            raise ValueError(f'unsupported perception mode: {mode}')
        for target in result['targets']:
            x1, y1, x2, y2 = map(round, target['bbox_xyxy'])
            color = COLOR_BGR.get(target.get('color'), (0, 210, 100))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.drawMarker(annotated, tuple(map(round, target['center_uv'])), color,
                           cv2.MARKER_CROSS, 12, 1)
            # Chinese text is drawn by the UI using Pillow, not OpenCV's ASCII font.
        result['processing_ms'] = (time.monotonic() - started) * 1000
        return annotated, result

    def close(self):
        if self._objects is not None:
            self._objects.close()
            self._objects = None
        self._faces = None
