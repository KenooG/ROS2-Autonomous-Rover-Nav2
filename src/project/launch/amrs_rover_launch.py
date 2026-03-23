import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    pkg_description_path = get_package_share_directory('my_robot_description')
    world_file_path = os.path.join(pkg_description_path, 'worlds', 'mars_mission.world')
    sdf_model_path = os.path.join(pkg_description_path, 'models', 'perseverance', 'model.sdf')

    return LaunchDescription([
        # 1. Gazebo
        ExecuteProcess(cmd=['gz', 'sim', '-r', world_file_path], output='screen'),

        # 2. Bridge 
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/mars_mission/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/perseverance/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/model/perseverance/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/perseverance/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/perseverance/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/perseverance/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/mars_mission/set_pose@ros_gz_interfaces/srv/SetEntityPose'
            ],
            remappings=[
                ('/world/mars_mission/clock', '/clock'),
                ('/model/perseverance/scan', '/scan'),
                ('/model/perseverance/cmd_vel', '/cmd_vel'),
                ('/model/perseverance/odometry', '/odom'),
                ('/model/perseverance/tf', '/tf')
            ],
            output='screen'
        ),

        # 3. Robot State Publisher 
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    parameters=[{'use_sim_time': True, 'frame_prefix': 'perseverance/'}],
                    arguments=[sdf_model_path]
                )
            ]
        ),

        # 4. C++ 
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='project',
                    executable='Mars_Rover',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        )
    ])