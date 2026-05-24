import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('mobile_robot_gazebo')
    pkg_description = get_package_share_directory('mobile_robot_description')

    default_world = os.path.join(
        pkg_gazebo, 'worlds', 'small_warehouse', 'small_warehouse.world'
    )

    world = LaunchConfiguration('world')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    headless = LaunchConfiguration('headless')

    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_x = DeclareLaunchArgument('x_pos', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pos', default_value='0.0')
    declare_z = DeclareLaunchArgument('z_pos', default_value='0.0')
    declare_headless = DeclareLaunchArgument('headless', default_value='False')

    xacro_file = os.path.join(pkg_description, 'urdf', 'mobile_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # `-r` starts the world running (otherwise UserCommands won't advertise create);
    # `-s` runs headless (server only) when requested.
    gz_args = PythonExpression([
        "'-r ' + ('-s ' if '", headless, "' == 'True' else '') + '", world, "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '--name', 'mobile_robot',
            '--topic', 'robot_description',
            '--x', x_pos,
            '--y', y_pos,
            '--z', z_pos,
        ],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/mobile_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/mobile_robot/tf', '/tf'),
        ],
        output='screen'
    )

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/model/mobile_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/default/model/mobile_robot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    return LaunchDescription([
        # Force EGL to use X11 so the sensor render thread can fall back to llvmpipe.
        SetEnvironmentVariable('EGL_PLATFORM', 'x11'),
        # Let Gazebo resolve `model://aws_robomaker_warehouse_*` URIs against the
        # installed models/ directory.
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_gazebo, 'models'),
        ),
        declare_world,
        declare_x,
        declare_y,
        declare_z,
        declare_headless,
        gz_sim,
        state_pub,
        spawn_robot,
        ros_gz_bridge,
        joint_state_bridge,
    ])
