#!/usr/bin/env python3
"""
Cave Exploration launch file.

Launches the two core nodes of the exploration stack:
  1. frontier_detector_node  — identifies exploration frontiers in the OctoMap
  2. trajectory_generation_node — generates collision-free quintic trajectories
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    frontier_config = os.path.join(
        get_package_share_directory('frontier_detector'),
        'config', 'frontier_detector_params.yaml')

    planner_config = os.path.join(
        get_package_share_directory('trajectory_generator'),
        'config', 'trajectory_generator_params.yaml')

    return LaunchDescription([
        # Frontier Detector
        Node(
            package='frontier_detector',
            executable='frontier_detector_node',
            name='frontier_detector',
            output='screen',
            respawn=True,
            parameters=[frontier_config],
        ),

        # Sampling-Based Trajectory Planner
        Node(
            package='trajectory_generator',
            executable='trajectory_generation_node',
            name='trajectory_generation_node',
            output='screen',
            respawn=True,
            parameters=[planner_config],
        ),
    ])
