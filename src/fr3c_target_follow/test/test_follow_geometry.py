import numpy as np
import pytest
from fr3c_target_follow.follow_geometry import (
    PoseHistory, PositionPreview, quaternion_from_rotation, rotation_from_quaternion,
    transform, transform_rpy,
)


def test_image_time_interpolation_uses_history_not_current_robot_pose():
    history = PoseHistory()
    history.add(100, [0,0,0], [0,0,0,1])
    q = [0,0,np.sin(np.pi/4),np.cos(np.pi/4)]
    history.add(100.04, [.04,0,0], q)
    at_image, gap = history.at(100.02)
    assert at_image[:3,3] == pytest.approx([.02,0,0])
    assert at_image[:3,:3] @ [1,0,0] == pytest.approx([np.sqrt(.5),np.sqrt(.5),0])
    assert gap == pytest.approx(.04)
    assert history.latest()[1][:3,3] == pytest.approx([.04,0,0])


def test_target_fixed_in_base_stays_fixed_when_camera_moves():
    handeye = transform_rpy([.01,-.07,.04], [0,.2,np.pi/2])
    base_target = transform_rpy([.2,-.4,.01], [.1,.2,.3])
    for x, yaw in [(0,0),(.05,.1),(-.02,-.2)]:
        base_flange = transform_rpy([x,-.3,.35], [np.pi,0,yaw])
        camera_target = np.linalg.inv(base_flange@handeye)@base_target
        assert base_flange@handeye@camera_target == pytest.approx(base_target, abs=1e-12)


def test_no_extrapolation_or_interpolation_across_missing_feedback():
    history = PoseHistory()
    history.add(100,[0,0,0],[0,0,0,1])
    history.add(100.2,[.1,0,0],[0,0,0,1])
    with pytest.raises(ValueError,match='before'): history.at(99.9)
    with pytest.raises(ValueError,match='bracket'): history.at(100.3)
    with pytest.raises(ValueError,match='gap'): history.at(100.1)
    with pytest.raises(ValueError,match='increasing'): history.add(100.1,[0,0,0],[0,0,0,1])


def test_quaternion_sign_is_not_a_half_turn_in_interpolation():
    history = PoseHistory()
    q = quaternion_from_rotation(transform_rpy([0,0,0],[np.pi,.1,.2])[:3,:3])
    history.add(100,[0,0,0],q)
    history.add(100.04,[0,0,0],-q)
    value,_ = history.at(100.02)
    assert value[:3,:3] == pytest.approx(rotation_from_quaternion(q))


def test_preview_preserves_initial_offset_and_ignores_tag_rotation():
    tcp = transform_rpy([.2,-.3,.2], [np.pi,.1,.2])
    preview = PositionPreview()
    tag = np.array([.2,-.4,.0])
    preview.teach(tag,tcp)
    goal, reference, error, velocity = preview.step(tag,tcp,.02)
    assert goal == pytest.approx(tcp)
    assert error == pytest.approx(np.zeros(3), abs=1e-12)
    new_tag = tag + [.02,-.01,.01]
    goal, reference, error, velocity = preview.step(new_tag,tcp,.02)
    assert goal[:3,3] == pytest.approx(tcp[:3,3]+[.02,-.01,.01])
    assert goal[:3,:3] == pytest.approx(tcp[:3,:3])
    assert error == pytest.approx([.02,-.01,.01])
    assert np.linalg.norm(velocity) <= .03*.02+1e-12
    assert reference[:3,3] == pytest.approx(tcp[:3,3]+velocity*.02)


def test_preview_limits_speed_acceleration_and_never_integrates_fake_feedback():
    tcp=transform([0,0,0],[0,0,0,1]);preview=PositionPreview()
    preview.teach([0,0,0],tcp)
    old_v=np.zeros(3)
    for _ in range(100):
        goal,ref,error,v=preview.step([1,0,0],tcp,.02)
        assert np.linalg.norm(v)<=.01+1e-12
        assert np.linalg.norm(v-old_v)<=.03*.02+1e-12
        assert ref[0,3]<=.01*.02+1e-12
        old_v=v
    preview.stop()
    assert preview.velocity == pytest.approx([0,0,0])
    with pytest.raises(ValueError,match='period_gap'): preview.step([1,0,0],tcp,.2)
    assert preview.velocity == pytest.approx([0,0,0])
