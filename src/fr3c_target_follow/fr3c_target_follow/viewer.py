"""Live view with teach/start/stop controls. Q/Esc closes this process."""
import argparse
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from fr3c_camera.rgbd import image_array


def window_is_closed(name):
    """OpenCV 4.5.4 GTK lacks VISIBLE; AUTOSIZE still detects user closure."""
    try:
        visible = cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE)
        if visible >= 0:
            return visible < 1
        return cv2.getWindowProperty(name, cv2.WND_PROP_AUTOSIZE) < 0
    except cv2.error:
        # Some backends raise instead of returning -1 for a removed window.
        return True


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scale', type=float, default=1.5)
    view_mode = parser.add_mutually_exclusive_group()
    view_mode.add_argument('--read-only', action='store_true', help='No teach/start/stop clients or keyboard commands.')
    view_mode.add_argument('--pose-preview', action='store_true', help='Stage 7: T records preview only; no motion clients.')
    options, ros_args = parser.parse_known_args(args)
    if not np.isfinite(options.scale) or options.scale <= 0:
        parser.error('--scale must be positive')
    cv2.setNumThreads(2)
    rclpy.init(args=ros_args)
    node = Node('pose_preview_viewer' if options.pose_preview else 'target_pose_viewer')
    latest = {}
    def receive(message):
        latest.update(message=message, arrival=time.monotonic())
    node.create_subscription(Image, '/target_follow/image_annotated', receive, qos_profile_sensor_data)
    def receive_preview(message):
        latest.update(preview=json.loads(message.data), preview_arrival=time.monotonic())
    node.create_subscription(String, '/target_follow/preview_state', receive_preview, 10)
    def receive_follow(message):
        latest.update(follow=json.loads(message.data), follow_arrival=time.monotonic())
    node.create_subscription(String, '/target_follow/follow_state', receive_follow, 10)
    node.create_subscription(String, '/target_follow/orientation_state',
                             lambda m: latest.update(orientation=json.loads(m.data), orientation_arrival=time.monotonic()), 10)
    if options.pose_preview:
        node.create_subscription(String, '/target_follow/pose_preview/state',
                                 lambda m: latest.update(pose_preview=json.loads(m.data), pose_preview_arrival=time.monotonic()), 10)
    clients = {}
    if options.pose_preview:
        clients = {ord('t'): ('Record preview', node.create_client(Trigger, '/target_follow/pose_preview/teach'))}
    elif not options.read_only:
        clients = {ord('t'): ('Teach', node.create_client(Trigger, '/target_follow/teach_offset')),
                   ord('s'): ('Start', node.create_client(Trigger, '/target_follow/follow_start')),
                   ord('x'): ('Stop', node.create_client(Trigger, '/target_follow/servo_stop'))}
    name = ('FR3C pose preview - MOTION OFF - T record - Q/Esc close' if options.pose_preview else
            'FR3C orientation check - READ ONLY - Q/Esc close view' if options.read_only else
            'FR3C target follow - T teach - S start - X stop - Q/Esc close view')
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.03)
            message = latest.get('message')
            if message is None:
                canvas = np.zeros((480, 640, 3), np.uint8)
                cv2.putText(canvas, 'Waiting for target pose image', (30, 50), 0, .7, (0, 180, 255), 1)
            else:
                canvas = image_array(message).copy()
                if time.monotonic() - latest['arrival'] > .3:
                    cv2.putText(canvas, 'NO FRESH IMAGE', (10, 90), 0, .6, (0, 0, 255), 2)
            follow = None if options.pose_preview else latest.get('follow')
            preview = None if options.pose_preview else follow if follow is not None else latest.get('preview')
            arrival = latest.get('follow_arrival' if follow is not None else 'preview_arrival', 0)
            if preview is not None and time.monotonic()-arrival < .5:
                def xyz(key):
                    return ' '.join(f'{x*1000:.1f}' for x in preview[key]) if key in preview else 'waiting'
                range_stop = follow is not None and follow.get('reason') in (
                    'target is outside the follow range', 'actual TCP is outside the follow range')
                mode = ('FOLLOW RANGE LIMIT' if range_stop else
                        'FAULT - CHECK DRIVER' if follow['status'] == 'fault' else
                        'COLLISION PAUSE' if follow['status'] == 'paused_collision' else
                        'TARGET LOST - WAITING TO RESUME' if follow['status'] == 'paused_target' else
                        'FOLLOW ON' if follow.get('motion_commands_enabled') else 'FOLLOW OFF') if follow is not None else 'PREVIEW | MOTION OFF'
                if follow is not None and follow.get('mock'):
                    mode = 'MOCK | ' + follow['status']
                controls = (' | restart driver' if follow.get('status') == 'fault' else ' | S start X stop') if follow is not None else ' | T teach'
                if options.read_only:
                    controls = ' | read-only'
                rows = [
                    'BASE tag mm: '+xyz('target_base_m'),
                    'Actual TCP mm: '+xyz('actual_tcp_m'),
                    'Desired TCP mm: '+xyz('desired_tcp_m'),
                    'Error mm: '+xyz('position_error_m'),
                    ('Follow' if follow is not None else 'Preview')+' v mm/s: '+xyz('velocity_m_s'),
                    mode+' | '+preview['status'][:35]+controls,
                ]
                if follow is not None and follow.get('follow_mode') == 'pose':
                    def attitude(key):
                        return ' '.join(f'{v:.1f}' for v in follow[key]) if key in follow else 'waiting'
                    gap = follow.get('rotation_error_deg')
                    rows.extend([
                        'POSE '+('HOLD' if follow.get('settled_hold') else 'FOLLOW')+
                        (' | motion pending' if follow.get('goal_pending') else '')+' | Actual RPY: '+attitude('actual_tcp_rpy_deg'),
                        'Desired RPY: '+attitude('desired_tcp_rpy_deg')+' | gap '+(f'{gap:.1f} deg' if gap is not None else 'waiting'),
                        'w deg/s: '+attitude('angular_velocity_deg_s')+f" | mapping {follow['rotation_mapping_gain']:.2f}",
                    ])
                    if follow.get('requires_reteach'):
                        rows.append('TARGET RELOCATED | S records the current relation')
                def age(key):
                    value = preview.get(key)
                    return f'{value:.0f}' if value is not None else 'waiting'
                rows.append('Age ms: target '+age('target_age_ms')+' | feedback '+age('feedback_age_ms'))
                if follow is not None and 'stop_motion_code' in follow:
                    rows.append('SDK stop / end: '+str(follow['stop_motion_code'])+' / '+str(follow.get('servo_end_code', '?')))
                if follow is not None and follow.get('reason'):
                    rows.append('Reason: '+follow['reason'][:72])
                if follow is not None and 'collision' in follow:
                    guard = follow['collision']
                    if not guard.get('enabled'):
                        rows.append('Model collision: OFF | '+('pose following' if follow.get('follow_mode') == 'pose' else 'legacy following'))
                    elif guard.get('ready'):
                        rows.append(f"Model collision: ON | {guard['distance_mm']:.1f} mm | "
                                    f"speed {guard['speed_scale']*100:.0f}% | check {guard['check_ms']:.1f} ms")
                    else:
                        rows.append('Model collision: ON | waiting for check')
                for i,text in enumerate(rows):
                    y=canvas.shape[0]-(len(rows)*20-5)+i*20
                    cv2.putText(canvas,text,(10,y),0,.46,(0,0,0),3,cv2.LINE_AA)
                    cv2.putText(canvas,text,(10,y),0,.46,(0,255,255),1,cv2.LINE_AA)
            elif follow is not None:
                cv2.putText(canvas, 'NO FRESH FOLLOW STATE - check driver', (10, canvas.shape[0]-20),
                            0, .6, (0, 0, 255), 2)
            pose_preview = latest.get('pose_preview')
            if options.pose_preview and pose_preview is not None:
                fresh = time.monotonic()-latest.get('pose_preview_arrival', 0) < .5
                def vector(key, scale=1.):
                    return ' '.join(f'{v*scale:.1f}' for v in pose_preview[key]) if key in pose_preview else 'waiting'
                gap = pose_preview.get('rotation_error_deg')
                angle_gap = f'{gap:.1f} deg' if gap is not None else 'unavailable'
                rows = [
                    'POSE PREVIEW | MOTION OFF | '+(pose_preview['status'] if fresh else 'NO FRESH STATE'),
                    'Raw tag RPY deg: '+vector('raw_target_rpy_deg'),
                    'Filtered RPY deg: '+vector('filtered_target_rpy_deg'),
                    'Actual TCP mm: '+vector('actual_tcp_m', 1000.),
                    'Desired TCP mm: '+vector('desired_tcp_m', 1000.),
                    'Actual TCP RPY: '+vector('actual_tcp_rpy_deg'),
                    'Desired TCP RPY: '+vector('desired_tcp_rpy_deg'),
                    'Goal-actual mm: '+vector('position_error_m', 1000.)+' | angle '+angle_gap,
                    'Preview w deg/s: '+vector('angular_velocity_deg_s'),
                    f"Gain {pose_preview['rotation_mapping_gain']:.2f} | deadband {pose_preview['deadband_enter_deg']:.1f} deg | T record",
                ]
                for i, text in enumerate(rows):
                    y = canvas.shape[0]-(len(rows)*19-5)+i*19
                    cv2.putText(canvas, text, (10, y), 0, .44, (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.putText(canvas, text, (10, y), 0, .44, (0, 255, 255), 1, cv2.LINE_AA)
            orientation = latest.get('orientation')
            if orientation is not None:
                fresh = time.monotonic()-latest.get('orientation_arrival', 0) < .3
                status = orientation['status'] if fresh else 'NO FRESH ORIENTATION STATE'
                if fresh and orientation.get('quality', {}).get('orientation_approximate'):
                    status += ' | APPROX DEMO'
                colour = (30, 230, 30) if fresh and orientation.get('valid_orientation') else (0, 140, 255)
                rpy = orientation.get('rpy_deg')
                angle = ' '.join(f'{v:.1f}' for v in rpy) if rpy is not None else 'unavailable'
                cv2.putText(canvas, 'BASE orientation: '+status, (10, 90), 0, .48, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(canvas, 'BASE orientation: '+status, (10, 90), 0, .48, colour, 1, cv2.LINE_AA)
                cv2.putText(canvas, 'BASE RPY deg: '+angle+' | DATA ONLY', (10, 110), 0, .46, colour, 1, cv2.LINE_AA)
            if latest.get('command'):
                command_y = 135 if options.pose_preview else 115
                cv2.putText(canvas, latest['command'][:78], (10, command_y), 0, .46, (0,0,0), 3, cv2.LINE_AA)
                cv2.putText(canvas, latest['command'][:78], (10, command_y), 0, .46, (0,255,255), 1, cv2.LINE_AA)
            canvas = cv2.resize(canvas, None, fx=options.scale, fy=options.scale)
            cv2.imshow(name, canvas)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord('q'), ord('Q')) or window_is_closed(name):
                node.get_logger().info('View closed by Q/Esc or window close.')
                break
            command = clients.get(key | 32)
            if command is not None:
                action, client = command
                if not client.service_is_ready():
                    latest['command'] = action + ': service unavailable - check driver / ROS domain'
                    continue
                latest['command'] = action + ': waiting for driver'
                def completed(future, action=action):
                    try:
                        result = future.result()
                        latest['command'] = action + (' OK: ' if result.success else ' rejected: ') + result.message
                    except Exception as exc:
                        latest['command'] = action + ' failed: ' + str(exc)
                    node.get_logger().info(latest['command'])
                client.call_async(Trigger.Request()).add_done_callback(completed)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
