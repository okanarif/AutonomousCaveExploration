#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 visualization'
    )

    # Simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("simulation"), "launch", "simulation.launch.py"])
        )
    )

    # Perception (3D Octomap)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("octomap_server"), "launch", "perception_mapping.launch.py"])
        )
    )

    # Cave Navigation Stack (Frontier + Trajectory)
    cave_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("cave_navigation"), "launch", "cave_exploration.launch.py"])
        )
    )

    # Mission Control Node
    mission_control_config = os.path.join(
        get_package_share_directory('mission_control'),
        'config',
        'mission_control_params.yaml'
    )

    mission_control_node = Node(
        package="mission_control",
        executable="mission_control_node",
        name="mission_control_node",
        output="screen",
        parameters=[
            mission_control_config,
            {'use_sim_time': True}
        ],
    )
    # Lantern Detection 
    lantern_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("lantern_detection"), "launch", "lantern_detection.launch.py"])
        )
    )
    # RViz2
    rviz_config = os.path.join(
        get_package_share_directory('cave_navigation'),
        'rviz',
        'cave_navigation.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        simulation_launch,
        perception_launch,
        mission_control_node,
        cave_navigation_launch,
        lantern_detection_launch,
        rviz_node,
    ])
