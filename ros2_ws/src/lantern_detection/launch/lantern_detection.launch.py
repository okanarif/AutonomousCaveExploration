from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_name = "lantern_detection"
    pkg_share = get_package_share_directory(pkg_name)

    param_file = os.path.join(
        pkg_share,
        "config",
        "lantern_detection_params.yaml"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock"
        ),

        Node(
            package=pkg_name,
            executable="lantern_detection_node",
            name="lantern_detection_node",
            output="screen",
            parameters=[
                param_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=[
                # ("/current_state_est", "/odometry"),
                # ("/realsense/depth/image", "/camera/depth/image_raw"),
            ]
        )
    ])
