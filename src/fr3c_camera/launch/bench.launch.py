"""Only camera and optional diagnostic probe; no robot or hand-eye TF."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def start(context):
    config = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    params = config['camera']
    name = config['camera_name']
    component = ComposableNode(package='orbbec_camera',
        plugin='orbbec_camera::OBCameraNodeDriver', name=name, namespace=name, parameters=[params])
    actions = [ComposableNodeContainer(name='camera_container', namespace=name,
        package='rclcpp_components', executable='component_container',
        composable_node_descriptions=[component], output='screen')]
    if LaunchConfiguration('probe').perform(context).lower() == 'true':
        actions.append(Node(package='fr3c_camera', executable='rgbd_probe',
            parameters=[{'camera_namespace': '/' + name}], output='screen'))
    return actions

def generate_launch_description():
    default = str(Path(get_package_share_directory('fr3c_camera')) / 'config/dabai_dcw2.yaml')
    return LaunchDescription([DeclareLaunchArgument('config', default_value=default),
        DeclareLaunchArgument('probe', default_value='true'), OpaqueFunction(function=start)])
