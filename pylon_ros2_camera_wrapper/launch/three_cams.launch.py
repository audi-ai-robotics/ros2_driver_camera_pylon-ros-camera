#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context: LaunchContext):
    # Log format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    pkg_share = get_package_share_directory('pylon_ros2_camera_wrapper')

    mtu_size = LaunchConfiguration('mtu_size')
    startup_user_set = LaunchConfiguration('startup_user_set')
    respawn = LaunchConfiguration('respawn').perform(context).lower() == 'true'

    cams = [
        {
            'namespace': LaunchConfiguration('cam1_namespace').perform(context),
            'node_name': LaunchConfiguration('cam1_node_name').perform(context),
            'config_file': LaunchConfiguration('cam1_config_file').perform(context),
        },
        {
            'namespace': LaunchConfiguration('cam2_namespace').perform(context),
            'node_name': LaunchConfiguration('cam2_node_name').perform(context),
            'config_file': LaunchConfiguration('cam2_config_file').perform(context),
        },
        {
            'namespace': LaunchConfiguration('cam3_namespace').perform(context),
            'node_name': LaunchConfiguration('cam3_node_name').perform(context),
            'config_file': LaunchConfiguration('cam3_config_file').perform(context),
        },
    ]

    nodes = []
    for cam in cams:
        # Resolve config file path (either absolute or relative to package share)
        config_path = cam['config_file']
        if not os.path.isabs(config_path):
            config_path = os.path.join(pkg_share, config_path)

        nodes.append(
            Node(
                package='pylon_ros2_camera_wrapper',
                namespace=cam['namespace'],
                executable='pylon_ros2_camera_wrapper',
                name=cam['node_name'],
                output='screen',
                respawn=respawn,
                emulate_tty=True,
                parameters=[
                    config_path,
                    {
                        'startup_user_set': startup_user_set,
                        'gige/mtu_size': mtu_size,
                    }
                ]
            )
        )

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('pylon_ros2_camera_wrapper')

    return LaunchDescription([
        # Camera 1 (left: 10.20.20.70)
        DeclareLaunchArgument('cam1_namespace', default_value='cam_left',
                              description='Namespace for camera 1'),
        DeclareLaunchArgument('cam1_node_name', default_value='pylon_ros2_camera_node',
                              description='Node name for camera 1'),
        DeclareLaunchArgument('cam1_config_file',
                              default_value=os.path.join(pkg_share, 'config', 'cam_left_ur20.yaml'),
                              description='Path to config YAML for camera 1'),

        # Camera 2 (wrist: 10.20.20.71)
        DeclareLaunchArgument('cam2_namespace', default_value='cam_wrist',
                              description='Namespace for camera 2'),
        DeclareLaunchArgument('cam2_node_name', default_value='pylon_ros2_camera_node',
                              description='Node name for camera 2'),
        DeclareLaunchArgument('cam2_config_file',
                              default_value=os.path.join(pkg_share, 'config', 'cam_wrist_ur20.yaml'),
                              description='Path to config YAML for camera 2'),

        # Camera 3 (top: 10.20.20.72)
        DeclareLaunchArgument('cam3_namespace', default_value='cam_top',
                              description='Namespace for camera 3'),
        DeclareLaunchArgument('cam3_node_name', default_value='pylon_ros2_camera_node',
                              description='Node name for camera 3'),
        DeclareLaunchArgument('cam3_config_file',
                              default_value=os.path.join(pkg_share, 'config', 'cam_top_ur20.yaml'),
                              description='Path to config YAML for camera 3'),

        # Shared parameters
        DeclareLaunchArgument('mtu_size', default_value='1500',
                              description='GigE MTU size'),
        DeclareLaunchArgument('startup_user_set', default_value='UserSet1',
                              description='Camera UserSet loaded at startup'),
        DeclareLaunchArgument('respawn', default_value='false',
                              description='Whether to respawn the camera nodes if they crash'),

        OpaqueFunction(function=_launch_nodes)
    ])
