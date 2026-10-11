import pytest
from fr3c_target_follow.demo_graph import pose_follow_source_plan


def test_pose_follow_owns_driver_converter_and_monitor_but_reuses_one_detector():
    plan = pose_follow_source_plan({'publishers': {'camera': 1, 'detector': 1, 'quality': 1}, 'nodes': []})
    assert not plan['start_camera'] and not plan['start_detector'] and plan['start_viewer']
    assert not pose_follow_source_plan({'publishers': {}, 'nodes': []}, mock=True)['start_camera']


@pytest.mark.parametrize('counts', [{'pose_observation': 1}, {'orientation': 1}, {'pose_preview': 1},
                                   {'flange': 1}, {'servo': 1}, {'quality': 2}])
def test_pose_follow_rejects_existing_owners_and_duplicate_pose_quality(counts):
    with pytest.raises(RuntimeError):
        pose_follow_source_plan({'publishers': counts, 'nodes': []})
