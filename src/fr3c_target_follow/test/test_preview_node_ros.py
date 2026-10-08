"""Opt-in isolated ROS contract for image-time conversion and stale preview stop."""
import json
import os
import time
import numpy as np
import pytest

pytestmark=pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS')!='1', reason='isolated ROS domain required')


def test_base_frame_teach_motion_preview_and_stale_stop(tmp_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.duration import Duration
    from geometry_msgs.msg import PoseStamped,TransformStamped,TwistStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from fr3c_target_follow.preview_node import FollowPreview

    config=tmp_path/'tcp.yaml'
    config.write_text('ydi_j22_tcp:\n  calibrated: true\n  parent_frame: flange_link\n  child_frame: grasp_frame\n  xyz_m: [0, 0, 0.1]\n  rpy_rad: [0, 0, 0]\n')
    rclpy.init(args=['--ros-args','-p','tcp_file:='+str(config)])
    follower=FollowPreview();probe=Node('follow_preview_contract_probe')
    executor=SingleThreadedExecutor();executor.add_node(follower);executor.add_node(probe)
    states=[];poses=[];velocities=[]
    probe.create_subscription(String,'/target_follow/preview_state',lambda m:states.append(json.loads(m.data)),10)
    probe.create_subscription(PoseStamped,'/target_follow/tag_pose_base',poses.append,10)
    probe.create_subscription(TwistStamped,'/target_follow/preview_velocity',velocities.append,10)
    flange_pub=probe.create_publisher(PoseStamped,'/target_follow/flange_pose',20)
    tag_pub=probe.create_publisher(PoseStamped,'/target_follow/tag_pose',5)
    client=probe.create_client(Trigger,'/target_follow/teach_offset')
    broadcaster=StaticTransformBroadcaster(probe)
    static=TransformStamped();static.header.stamp=probe.get_clock().now().to_msg()
    static.header.frame_id='flange_link';static.child_frame_id='camera_color_optical_frame'
    static.transform.translation.x=.01;static.transform.translation.y=.02;static.transform.translation.z=.03
    static.transform.rotation.w=1.;broadcaster.sendTransform(static)

    def spin(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)

    def robot():
        message=PoseStamped();message.header.frame_id='base_link';message.header.stamp=probe.get_clock().now().to_msg()
        message.pose.position.x=.1;message.pose.position.y=-.2;message.pose.position.z=.3;message.pose.orientation.w=1.
        flange_pub.publish(message)
    timer=probe.create_timer(.008,robot)

    def target(x=.05, yaw=0):
        message=PoseStamped();message.header.frame_id='camera_color_optical_frame'
        message.header.stamp=(probe.get_clock().now()-Duration(seconds=.06)).to_msg()
        message.pose.position.x=x;message.pose.position.y=.06;message.pose.position.z=.2
        message.pose.orientation.z=float(np.sin(yaw/2));message.pose.orientation.w=float(np.cos(yaw/2))
        tag_pub.publish(message);spin(.12)
        return message.header.stamp

    try:
        spin(.3);stamp=target()
        assert poses,states
        assert poses[-1].header.stamp==stamp
        p=poses[-1].pose.position
        assert [p.x,p.y,p.z]==pytest.approx([.16,-.12,.53])
        assert states[-1]['status']=='offset_not_recorded'
        assert client.wait_for_service(timeout_sec=1)
        result=client.call_async(Trigger.Request())
        deadline=time.monotonic()+1
        while not result.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        assert result.done() and result.result().success
        target(.075, yaw=np.pi/2)
        assert states[-1]['status']=='preview',states[-5:]
        assert states[-1]['position_error_m']==pytest.approx([.025,0,0],abs=1e-10)
        assert states[-1]['desired_tcp_m']==pytest.approx([.125,-.2,.4])
        assert np.linalg.norm(states[-1]['velocity_m_s'])<=.01+1e-12
        spin(.4)
        assert states[-1]['status']=='stale_or_missing_target'
        assert velocities[-1].twist.linear.x==0
        target(.075)
        assert states[-1]['status']=='preview'
        timer.cancel();spin(.25)
        assert states[-1]['status']=='stale_robot_feedback'
        v=velocities[-1].twist.linear
        assert [v.x,v.y,v.z]==[0,0,0]
        assert all(not s['motion_commands_enabled'] and not s['orientation_control_enabled'] for s in states)
    finally:
        executor.shutdown();follower.destroy_node();probe.destroy_node();rclpy.shutdown()
