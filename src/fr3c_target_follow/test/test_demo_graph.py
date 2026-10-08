"""Reuse and exclusive ownership decisions before any camera or SDK is opened."""
import pytest
from fr3c_target_follow.demo_graph import source_plan


def test_missing_partial_and_existing_sources():
    assert source_plan({'publishers': {}, 'nodes': []}) == {
        'start_camera': True, 'start_detector': True, 'start_viewer': True}
    assert source_plan({'publishers': {'camera': 1}, 'nodes': []})['start_detector']
    existing = {'publishers': {'camera': 1, 'detector': 1}, 'nodes': ['target_pose_viewer']}
    assert source_plan(existing) == {
        'start_camera': False, 'start_detector': False, 'start_viewer': False}
    assert not source_plan({'publishers': {}, 'nodes': []}, mock=True)['start_camera']


def test_duplicate_sources_and_existing_sdk_owner_are_rejected():
    for publishers in ({'camera': 2}, {'detector': 2}, {'flange': 1}, {'preview': 1}, {'follow': 1}):
        with pytest.raises(RuntimeError):
            source_plan({'publishers': publishers, 'nodes': []})
