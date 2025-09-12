#!/usr/bin/env python3
"""
ROS Jazzy Camera Launch File
Launches camera node with configuration
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rpi_camera_example'),
            'config',
            'camera_params.yaml'
        ]),
        description='Path to camera configuration file'
    )

    # Camera node
    camera_node = Node(
        package='rpi_camera_example',
        executable='camera_publisher',
        name='rpi_camera',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        camera_node,
    ])