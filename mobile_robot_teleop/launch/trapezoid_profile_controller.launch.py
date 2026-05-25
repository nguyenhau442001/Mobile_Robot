from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_robot_teleop',
            executable='trapezoid_profile_controller',
            name='trapezoid_profile_controller',
            output='screen',
        ),
    ])
