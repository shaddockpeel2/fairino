from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fr3c_camera.edge_perception import EdgePerception, SingleFaceTracker, WASTE_CLASSES, waste_category
from fr3c_camera.edge_runtime import load_settings
from fr3c_camera.rknn_detector import CLASS_NAMES, Detection


WORKSPACE = Path(__file__).resolve().parents[3]


def face(box, confidence=0.9):
    return {'bbox_xyxy': box, 'confidence': confidence}


def test_all_model_classes_have_exactly_one_four_bin_mapping():
    mapped = [name for names in WASTE_CLASSES.values() for name in names]
    assert len(mapped) == len(set(mapped)) == 16
    assert set(mapped) == set(CLASS_NAMES)
    assert waste_category('Used_batteries') == 'hazardous'
    with pytest.raises(ValueError):
        waste_category('unknown')


def test_object_adapter_keeps_multiple_targets_and_detection_scores():
    engine = EdgePerception(load_settings(WORKSPACE))
    class Detector:
        last_timings_ms = {'npu': 1.0}
        def infer(self, _image):
            return [Detection(15, 'Zip_top_can', 0.91, (10, 10, 50, 60)),
                    Detection(13, 'Used_batteries', 0.85, (70, 10, 100, 60))]
    engine._objects = Detector()
    _, result = engine.process('objects', np.zeros((120, 160, 3), np.uint8))
    assert result['counts'] == {'recyclable': 1, 'hazardous': 1}
    assert result['targets'][0]['confidence'] == 0.91
    assert result['targets'][0]['label'] == '易拉罐'


def test_colors_reuse_thresholds_and_allow_multiple_same_color_targets():
    engine = EdgePerception(load_settings(WORKSPACE))
    image = np.zeros((240, 500, 3), np.uint8)
    for offset, color in enumerate(((0, 0, 255), (0, 255, 255),
                                    (255, 0, 0), (0, 255, 0), (0, 0, 255))):
        cv2.rectangle(image, (10 + offset * 90, 25), (60 + offset * 90, 75), color, -1)
    _, result = engine.process('colors', image)
    assert result['counts'] == {'red': 2, 'yellow': 1, 'blue': 1, 'green': 1}
    assert len(result['targets']) == 5
    assert 'confidence' not in result['targets'][0]


def test_yunet_model_loads_and_runs_on_installed_opencv():
    engine = EdgePerception(load_settings(WORKSPACE))
    _, result = engine.process('faces', np.zeros((480, 640, 3), np.uint8))
    assert result['state'] == 'searching'
    assert result['faces_detected'] == 0
    engine.close()


def test_face_track_keeps_id_through_short_loss_and_rejects_unrelated_face():
    tracker = SingleFaceTracker(lost_after_s=0.8)
    first = tracker.update([face([10, 10, 70, 90]), face([250, 10, 280, 50])], 10)
    identifier = first['track_id']
    moved = tracker.update([face([20, 10, 80, 90])], 10.1)
    assert moved['track_id'] == identifier
    lost = tracker.update([face([250, 10, 280, 50])], 10.2)
    assert lost['state'] == 'temporarily_lost'
    assert lost['targets'] == []
    restored = tracker.update([face([30, 10, 90, 90])], 10.3)
    assert restored['track_id'] == identifier
    assert len(restored['trail']) == 3
    expired = tracker.update([], 11.2)
    assert expired['state'] == 'searching'
    assert expired['track_id'] is None
    acquired = tracker.update([face([250, 10, 280, 50])], 11.3)
    assert acquired['track_id'] != identifier


def test_camera_gap_discards_face_track_even_if_a_new_face_is_present():
    tracker = SingleFaceTracker(lost_after_s=0.8)
    identifier = tracker.update([face([10, 10, 70, 90])], 10)['track_id']
    after = tracker.update([face([11, 10, 71, 90])], 12)
    assert after['track_id'] != identifier
