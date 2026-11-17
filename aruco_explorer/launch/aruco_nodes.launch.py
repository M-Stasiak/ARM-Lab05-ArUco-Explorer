from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 1. Detektor ArUco
        Node(
            package='aruco_ros',
            executable='marker_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'marker_size': 0.09,
                'camera_frame': 'camera_rgb_optical_frame',
                'reference_frame': 'map',
                'image_is_rectified': False,
            }],
            remappings=[
                ('/image', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
            ],
        ),

        # 2. Generator punktow
        Node(
            package='aruco_explorer',
            executable='map_points_generator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'grid_spacing': 3.0,
                'max_cost': 20,
            }],
        ),

        # 3. Tracker + serwer akcji
        Node(
            package='aruco_explorer',
            executable='aruco_tracker',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # 4. Nawigacja
        Node(
            package='aruco_explorer',
            executable='map_points_navigation',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

    ])
