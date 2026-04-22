#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch the geometric controller node with parameters from YAML config."""

    pkg_share = get_package_share_directory('controller')
    default_config = os.path.join(pkg_share, 'config', 'controller_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to controller configuration YAML file'
    )

    controller_node = Node(
        package='controller',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        controller_node,
    ])
