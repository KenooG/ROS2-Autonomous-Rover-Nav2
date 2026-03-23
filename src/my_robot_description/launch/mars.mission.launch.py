import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    slam_yaml = os.path.join(pkg_path, 'config', 'slam_params.yaml')
    nav2_yaml = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'perseverance.rviz')

    return LaunchDescription([
        # 1. TFs 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'perseverance/lidar_link', 'perseverance/lidar_link/lidar'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '2.0', '0', '0', '0', 'perseverance/base_link', 'perseverance/lidar_link/lidar'],
            parameters=[{'use_sim_time': True}]
        ),

        # 2. SLAM Toolbox 
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
                    ),
                    launch_arguments={'slam_params_file': slam_yaml, 'use_sim_time': 'True'}.items()
                )
            ]
        ),

        # 3. RViz 
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

        # 4. Nav2 
        TimerAction(
            period=20.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={'params_file': nav2_yaml, 'use_sim_time': 'True'}.items()
                )
            ]
        )
    ])