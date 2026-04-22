#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch the frontier detector node with parameters from YAML config."""

    pkg_share = get_package_share_directory('frontier_detector')
    default_config = os.path.join(pkg_share, 'config', 'frontier_detector_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to frontier detector configuration YAML file'
    )

    frontier_detector_node = Node(
        package='frontier_detector',
        executable='frontier_detector_node',
        name='frontier_detector',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        respawn=True,
    )

    return LaunchDescription([
        config_arg,
        frontier_detector_node,
    ])
