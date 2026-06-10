"""Launch the waypoint follower with an optional table_id argument."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate a LaunchDescription for the waypoint_follower_node."""
    pkg_share = get_package_share_directory('mobile_robot_custom_nav')

    table_id_arg = DeclareLaunchArgument(
        'table_id', default_value='1',
        description='Table number (1-5); selects waypoints_table<N>.yaml',
    )
    dwell_time_arg = DeclareLaunchArgument(
        'dwell_time', default_value='5.0',
        description='Seconds to dwell at each waypoint',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='odom',
        description='Coordinate frame for navigation goals',
    )

    table_id = LaunchConfiguration('table_id')
    dwell_time = LaunchConfiguration('dwell_time')
    frame_id = LaunchConfiguration('frame_id')

    follower_node = Node(
        package='mobile_robot_custom_nav',
        executable='waypoint_follower_node',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'waypoint_file': [
                os.path.join(pkg_share, 'config', 'waypoints_table'),
                table_id,
                '.yaml',
            ],
            'dwell_time': dwell_time,
            'frame_id': frame_id,
        }],
    )

    return LaunchDescription([
        table_id_arg,
        dwell_time_arg,
        frame_id_arg,
        follower_node,
    ])
