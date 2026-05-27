"""Spawn N differential-drive robots into a single Gazebo Sim world.

Robots are read from a YAML list (config/robots.yaml). For each robot:
  - robot_state_publisher in its own namespace, with frame_prefix
  - ros_gz_sim create to spawn the entity
  - ros_gz_bridge for cmd_vel / odom / imu / scan (namespaced topics)
  - ros_gz_bridge for joint_states (namespaced model topic -> /<ns>/joint_states)
  - static world -> <ns>/odom TF at the spawn pose (skip when nav2 owns it)

Gazebo only — no RViz. Pair with mobile_robot_multi/multi_robot_nav2.launch.py
for visualization + navigation.
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_robots(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['robots']


def _grid_robots(n, spacing=1.5):
    """Place N robots on a centered square grid, all facing +X."""
    cols = max(1, math.ceil(math.sqrt(n)))
    rows = math.ceil(n / cols)
    cx = (cols - 1) / 2.0
    cy = (rows - 1) / 2.0
    return [
        {
            'name': f'robot{i + 1}',
            'x': (i % cols - cx) * spacing,
            'y': (i // cols - cy) * spacing,
            'z': 0.0,
            'yaw': 0.0,
        }
        for i in range(n)
    ]


def _spawn_action(robot):
    """The gz `create` Node — kept separate so spawns can be chained sequentially."""
    name = robot['name']
    return Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{name}',
        arguments=[
            '--name', name,
            '--topic', f'/{name}/robot_description',
            '--x', str(robot.get('x', 0.0)),
            '--y', str(robot.get('y', 0.0)),
            '--z', str(robot.get('z', 0.0)),
            '--Y', str(robot.get('yaw', 0.0)),
        ],
        output='screen',
    )


def _robot_aux_actions(robot, xacro_file, world_to_odom_static=True):
    """All per-robot actions except the gz spawn — RSP, bridges, static TF.

    When nav2 runs, AMCL owns map -> <name>/odom; set world_to_odom_static=False
    so <name>/odom isn't given two parents.
    """
    name = robot['name']
    x = str(robot.get('x', 0.0))
    y = str(robot.get('y', 0.0))
    z = str(robot.get('z', 0.0))
    yaw = str(robot.get('yaw', 0.0))

    # Per-robot URDF (frame prefixed inside the URDF by the gazebo xacro).
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' robot_name:=', name]),
        value_type=str,
    )

    # robot_state_publisher's TransformBroadcaster publishes to the absolute
    # /tf, so the namespace does NOT push it to /<ns>/tf — all RSPs share /tf
    # and rely on prefixed frame names to stay disjoint. use_tf_static=False
    # streams fixed-joint transforms on /tf at publish_frequency instead of
    # latching them on /tf_static.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name,
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': f'{name}/',
            'use_sim_time': True,
            'use_tf_static': False,
            'publish_frequency': 30.0,
        }],
        output='screen',
    )

    # cmd_vel / odom / imu / scan bridge — all topics already namespaced by the xacro.
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{name}',
        arguments=[
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    # Joint states: Gazebo emits on /world/<world>/model/<name>/joint_state — remap
    # into the robot's namespace so its RSP picks them up.
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'joint_state_bridge_{name}',
        arguments=[
            f'/world/default/model/{name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            (f'/world/default/model/{name}/joint_state', f'/{name}/joint_states'),
        ],
        output='screen',
    )

    # TF: DiffDrive plugin publishes odom -> <name>/chassis on Gazebo's
    # /model/<name>/tf — bridge it onto the global /tf so it joins the rest
    # of the tree (RSP publishes chassis -> wheels there too).
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'tf_bridge_{name}',
        arguments=[
            f'/model/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            (f'/model/{name}/tf', '/tf'),
        ],
        output='screen',
    )

    # Static world -> <name>/odom transform from the spawn pose. Stays global
    # (no namespace) so it lands on the shared /tf_static — each robot owns a
    # unique (world, <name>/odom) frame pair so the latched messages don't
    # collide between publishers.
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'world_to_{name}_odom',
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--yaw', yaw, '--pitch', '0', '--roll', '0',
            '--frame-id', 'world',
            '--child-frame-id', f'{name}/odom',
        ],
        output='screen',
    )

    actions = [rsp, sensor_bridge, joint_state_bridge, tf_bridge]
    if world_to_odom_static:
        actions.append(world_to_odom)
    return actions


def generate_launch_description():
    pkg_multi = get_package_share_directory('mobile_robot_multi')
    pkg_gazebo = get_package_share_directory('mobile_robot_gazebo')
    pkg_description = get_package_share_directory('mobile_robot_description')

    default_robots_file = os.path.join(pkg_multi, 'config', 'robots.yaml')
    default_world = os.path.join(pkg_gazebo, 'worlds', '10x10', '10x10.sdf')
    xacro_file = os.path.join(pkg_description, 'urdf', 'mobile_robot.urdf.xacro')

    # Resolve robot list at parse time so we can loop in Python. If N_ROBOTS is
    # set, auto-arrange that many on a grid; otherwise read from robots.yaml
    # (override path via ROBOTS_FILE env var).
    n_robots_env = os.environ.get('N_ROBOTS')
    if n_robots_env:
        n_robots = int(n_robots_env)
        robots = _grid_robots(n_robots)
        print(f'[multi_robot_world] grid layout: {n_robots} robots')
    else:
        robots_file = os.environ.get('ROBOTS_FILE', default_robots_file)
        robots = _load_robots(robots_file)

    # Set WORLD_TO_ODOM_STATIC=false when nav2 will own map -> <name>/odom,
    # otherwise the static here gives <name>/odom two parents.
    world_to_odom_static = (
        os.environ.get('WORLD_TO_ODOM_STATIC', 'true').lower() != 'false'
    )

    robots_file_arg = DeclareLaunchArgument(
        'robots_file',
        default_value=default_robots_file,
        description='YAML file listing robots (name + pose) to spawn',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the SDF/.world file Gazebo loads',
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items(),
    )

    actions = [
        # Force EGL to use X11 platform so sensor render thread can fall back to llvmpipe.
        SetEnvironmentVariable('EGL_PLATFORM', 'x11'),
        robots_file_arg,
        world_arg,
        gz_sim,
    ]
    # Aux nodes (RSP, bridges, static TF) fire concurrently at t=0 — they're
    # idempotent against late Gazebo topics and the latched URDFs need to be
    # on the network before any spawn subscribes.
    for r in robots:
        actions.extend(_robot_aux_actions(r, xacro_file, world_to_odom_static))

    # Chain `ros_gz_sim create` calls: each spawn fires only after the previous
    # one's process exits. Gazebo's /world/default/create service is
    # single-threaded and DiffDrive-plugin init per model takes real time, so
    # serialising spawns eliminates the race that drops the last entity at high
    # N. Total launch time becomes the sum of actual spawn durations — no slack.
    spawns = [_spawn_action(r) for r in robots]
    for prev, curr in zip(spawns, spawns[1:]):
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=prev, on_exit=[curr]),
        ))
    actions.append(spawns[0])

    return LaunchDescription(actions)
