import pytest
from fr3c_target_follow.demo_graph import orientation_source_plan


def test_orientation_launch_starts_missing_read_only_sources_and_reuses_existing_owner():
    assert all(orientation_source_plan({'publishers': {}}).values())
    graph = {'publishers': {'camera': 1, 'detector': 1, 'quality': 1, 'flange': 1, 'feedback': 1}}
    assert orientation_source_plan(graph) == dict(start_camera=False, start_detector=False,
                                                 start_feedback=False, start_base=True)
    graph['publishers']['base_target'] = 1
    assert not any(orientation_source_plan(graph).values())
    assert not any(orientation_source_plan({'publishers': {}}, start_sources=False).values())


@pytest.mark.parametrize('counts', [{'follow': 1}, {'servo': 1}, {'flange': 2},
                                   {'flange': 1}, {'base_target': 1}, {'orientation': 1}, {'pose_preview': 1}])
def test_orientation_launch_rejects_motion_owner_duplicates_and_incomplete_sources(counts):
    with pytest.raises(RuntimeError):
        orientation_source_plan({'publishers': counts})
