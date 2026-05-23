"""Bring up slam_toolbox (online async) + RViz for the single robot.

Pair with `mobile_robot_gazebo mobile_robot_10x10_world.launch.py` running in
another terminal. Drive the robot with teleop to scan the environment, then
save the map with `nav2_map_server map_saver_cli`.

This embeds slam_toolbox so one launch covers the whole SLAM session — no
need for a separate `ros2 launch slam_toolbox online_async_launch.py` call.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_slam')

    default_slam_params = os.path.join(pkg_share, 'param', 'slam_toolbox.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'slam_toolbox.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    slam_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_slam_params,
            description='Full path to the slam_toolbox params YAML'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config,
            description='Full path to the RViz config'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_launch_dir, 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
