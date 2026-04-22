from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = True 

    base_link_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'true_body', 'Quadrotor/DepthCamera'],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    static_tf_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_alias',
        arguments=[
            '0', '0', '0', 
            '-1.570795', '0', '-1.570795', 
            'Quadrotor/DepthCamera', 
            'Quadrotor/Sensors/DepthCamera'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='depth_to_pointcloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', '/realsense/depth/image'),
            ('camera_info', '/realsense/depth/camera_info'),
            ('points', '/realsense/depth/points'),
        ]
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            {'resolution': 1.0},
            {'frame_id': 'world'}, 
            {'sensor_model/max_range': 100.0},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cloud_in', '/realsense/depth/points')
        ]
    )

    return LaunchDescription([
        base_link_to_camera,
        static_tf_alias,
        pointcloud_node,
        octomap_node
    ])