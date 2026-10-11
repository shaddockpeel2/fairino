"""Window removal must not be confused with an unsupported GTK property."""
import os
import time

import cv2
import numpy as np
import pytest

from fr3c_target_follow.viewer import window_is_closed


@pytest.mark.parametrize('visible,autosize,closed', [
    (-1., 0., False),  # GTK: visible unsupported, WINDOW_NORMAL still exists.
    (-1., 1., False),  # GTK: WINDOW_AUTOSIZE still exists.
    (-1., -1., True),  # GTK: removed window.
    (1., None, False),  # Qt/Win32: visible supported.
    (0., None, True),
])
def test_window_properties(monkeypatch, visible, autosize, closed):
    def property_value(name, prop):
        assert name == 'test'
        if prop == cv2.WND_PROP_VISIBLE:
            return visible
        assert prop == cv2.WND_PROP_AUTOSIZE and autosize is not None
        return autosize
    monkeypatch.setattr(cv2, 'getWindowProperty', property_value)
    assert window_is_closed('test') is closed


def test_removed_window_can_raise(monkeypatch):
    def removed(name, prop):
        raise cv2.error('NULL window')
    monkeypatch.setattr(cv2, 'getWindowProperty', removed)
    assert window_is_closed('test')


@pytest.mark.skipif(os.environ.get('FR3C_RUN_GUI_TESTS') != '1', reason='working display required')
def test_real_window_survives_frames_and_detects_removal():
    name = 'FR3C window lifecycle regression - data only'
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    try:
        for _ in range(10):
            cv2.imshow(name, np.zeros((120, 240, 3), np.uint8))
            cv2.waitKey(20)
            assert not window_is_closed(name)
        if 'GUI:                           GTK3' in cv2.getBuildInformation():
            # Exercise the title-bar delete event. OpenCV 4.5.4 destroyWindow
            # can leave a registered GTK window after destroying its widgets.
            gi = pytest.importorskip('gi')
            gi.require_version('Gtk', '3.0')
            from gi.repository import Gtk
            window = next(w for w in Gtk.Window.list_toplevels() if w.get_title() == name)
            window.close()
        else:
            cv2.destroyWindow(name)
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            cv2.waitKey(20)
            if window_is_closed(name):
                break
        assert window_is_closed(name)
    finally:
        cv2.destroyAllWindows()
