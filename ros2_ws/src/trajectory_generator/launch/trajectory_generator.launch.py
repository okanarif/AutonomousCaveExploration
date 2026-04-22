#!/usr/bin/env python3
"""Launch the sampling-based trajectory planner with YAML config."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('trajectory_generator'),
        'config', 'trajectory_generator_params.yaml')

    return LaunchDescription([
        Node(
            package='trajectory_generator',
            executable='trajectory_generation_node',
            name='trajectory_generation_node',
            output='screen',
            respawn=True,
            parameters=[config],
        ),
    ])
