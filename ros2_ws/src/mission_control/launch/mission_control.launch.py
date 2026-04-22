#!/usr/bin/env python3
"""
Standalone launch file for the mission control node.

Launches the mission_control_node with its YAML config.
This is useful for testing or running mission control independently
from the full integrated_mission.launch.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mission_control'),
        'config', 'mission_control_params.yaml')

    return LaunchDescription([
        Node(
            package='mission_control',
            executable='mission_control_node',
            name='mission_control_node',
            output='screen',
            parameters=[config, {'use_sim_time': True}],
        ),
    ])
