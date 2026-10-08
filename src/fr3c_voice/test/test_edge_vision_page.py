import base64
from pathlib import Path
import queue
import time

import cv2
import numpy as np
import pytest

from fr3c_voice.edge_vision_page import EdgeVisionPage


@pytest.fixture
def page(tmp_path):
    import tkinter as tk
    from tkinter import ttk
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('Tk display is not available')
    root.withdraw()
    notebook = ttk.Notebook(root)
    notebook.pack(fill='both', expand=True)
    original = ttk.Frame(notebook)
    notebook.add(original, text='original')
    page = EdgeVisionPage(notebook, tmp_path)
    # No process launch, microphone, ROS, model or device use in these tests.
    page._ensure_backend = lambda: None
    page._fetch = lambda *args: None
    yield page
    page.recognizing = False
    page.close()
    root.destroy()


def payload():
    image = np.zeros((120, 160, 3), np.uint8)
    cv2.rectangle(image, (10, 30), (65, 85), (0, 0, 255), 2)
    _, encoded = cv2.imencode('.jpg', image)
    return {'ok': True, 'jpeg': base64.b64encode(encoded).decode(), 'result': {
        'mode': 'colors', 'state': 'running', 'counts': {'red': 1}, 'sequence': 1,
        'targets': [{'label': '红色', 'color': 'red', 'bbox_xyxy': [10, 30, 65, 85],
                     'center_uv': [37.5, 57.5]}]}}


def test_page_renders_chinese_labels_and_saves_paired_result(page):
    page._render(payload())
    assert page.photo is not None
    assert len(page.tree.get_children()) == 1
    assert '红' in page.summary.get()
    page.save()
    captures = page.workspace / 'evidence/vision/captures'
    assert len(list(captures.glob('*.jpg'))) == 1
    assert len(list(captures.glob('*.json'))) == 1


def test_lost_stream_clears_image_results_and_disables_saving_stale_frame(page):
    page._render(payload())
    page._render({'result': {'state': 'camera_stale', 'targets': [], 'counts': {}}, 'jpeg': None})
    assert page.last_image is None
    assert len(page.tree.get_children()) == 0
    page.save()
    assert not (page.workspace / 'evidence/vision/captures').exists()


def test_leaving_page_stops_recognition_without_calling_original_controls(page):
    calls = []
    page.recognizing = True
    page.was_visible = True
    page.stop = lambda: calls.append('stop_edge_only')
    page._tick()
    assert calls == ['stop_edge_only']


def test_late_responses_from_previous_ui_generation_are_ignored(page):
    page.notebook.select(page.page)
    page.fetching = True
    page.generation = 2
    page.events.put(('snapshot', 1, payload()))
    page._tick()
    assert page.photo is None
    assert len(page.tree.get_children()) == 0
