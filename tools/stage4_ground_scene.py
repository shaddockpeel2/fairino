#!/usr/bin/env python3
"""Add or remove the project's measured ground slab for stage-four Servo tests."""

import argparse
from pathlib import Path
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
import yaml


DEFAULT_CONFIG = (Path(__file__).resolve().parents[1] /
                  'src/fairino_mtc_demo/config/first_grasp.yaml')


def ground_from_config(path):
    params = yaml.safe_load(path.read_text(encoding='utf-8'))[
        'fr3c_first_grasp']['ros__parameters']
    size_xy = [float(v) for v in params['ground_plane_xy_size_m']]
    thickness = float(params['ground_plane_thickness_m'])
    top_z = float(params['ground_plane_z_m'])
    epsilon = float(params['ground_plane_contact_epsilon_m'])
    return {
        'id': str(params['ground_plane_id']),
        'frame': str(params['base_frame']),
        'dimensions_m': [*size_xy, thickness],
        'center_z_m': top_z - epsilon - thickness / 2.0,
    }


def call(node, client, request, timeout_s=5.0):
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError('planning_scene_service_unavailable')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done() or future.result() is None:
        raise RuntimeError('planning_scene_service_timeout')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--remove', action='store_true')
    parser.add_argument('--require-servo-sync', action='store_true',
                        help='Require the running Servo scene to report the update.')
    args = parser.parse_args()
    ground = ground_from_config(args.config)
    rclpy.init()
    node = Node('fr3c_stage4_ground_scene')
    try:
        obj = CollisionObject()
        obj.header.frame_id = ground['frame']
        obj.id = ground['id']
        obj.operation = (CollisionObject.REMOVE if args.remove
                         else CollisionObject.ADD)
        if not args.remove:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = ground['dimensions_m']
            obj.primitives.append(primitive)
            pose = Pose()
            pose.position.z = ground['center_z_m']
            pose.orientation.w = 1.0
            obj.primitive_poses.append(pose)
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects.append(obj)
        applied = call(node, node.create_client(
            ApplyPlanningScene, '/apply_planning_scene'), request)
        if not applied.success:
            raise RuntimeError('planning_scene_update_rejected')
        check = GetPlanningScene.Request()
        check.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = call(node, node.create_client(
            GetPlanningScene, '/get_planning_scene'), check)
        ids = [item.id for item in response.scene.world.collision_objects]
        present = ground['id'] in ids
        if present == args.remove:
            raise RuntimeError('planning_scene_update_not_visible')
        servo_updates = []
        if args.require_servo_sync:
            node.create_subscription(
                PlanningScene, '/servo_node/publish_planning_scene',
                lambda scene: servo_updates.extend(
                    item.id for item in scene.world.collision_objects), 10)
            publisher = node.create_publisher(
                CollisionObject, '/collision_object', 10)
            if not publisher.get_subscription_count():
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if publisher.get_subscription_count():
                        break
            if not publisher.get_subscription_count():
                raise RuntimeError('collision_object_has_no_subscribers')
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                publisher.publish(obj)
                rclpy.spin_once(node, timeout_sec=0.1)
                if ground['id'] in servo_updates:
                    break
            if ground['id'] not in servo_updates:
                raise RuntimeError('servo_scene_update_not_observed')
        print({
            'operation': 'remove' if args.remove else 'add',
            'ground': ground,
            'scene_object_ids': ids,
            'servo_scene_update_seen': ground['id'] in servo_updates,
        })
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
