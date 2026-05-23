"""Bring up a Nav2 stack inside a robot's namespace.

Scaffold: a single robot. Instantiates the 11 nav2 lifecycle nodes and 2
lifecycle managers under /<robot>/... directly (not via nav2_bringup) — the
existing multi_robot_world launch publishes RSP TF to absolute /tf with frames
already prefixed (robot1/odom, robot1/chassis, ...), and nav2_bringup's
default `/tf` -> `tf` remap would fight that.

This commit only wires the lifecycle structure; the params yaml is passed in
unmodified, so its `chassis` / `odom` frame references and `/scan` topics
won't yet match the namespaced versions Gazebo publishes. Follow-up commits
add the per-namespace yaml rewriter, $(find-pkg-share ...) expansion, the
namespace re-root, initial pose seeding, RViz, and the multi-robot loop.
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _grid_pose(n_robots, robot_name, spacing=1.5):
    """Mirror multi_robot_world._grid_robots so the lookup matches the spawn."""
    cols = max(1, math.ceil(math.sqrt(n_robots)))
    rows = math.ceil(n_robots / cols)
    cx = (cols - 1) / 2.0
    cy = (rows - 1) / 2.0
    for i in range(n_robots):
        name = f'robot{i + 1}'
        if name == robot_name:
            return {
                'name': name,
                'x': (i % cols - cx) * spacing,
                'y': (i // cols - cy) * spacing,
                'z': 0.0,
                'yaw': 0.0,
            }
    raise RuntimeError(f'{robot_name!r} not in grid of {n_robots}')


def _resolve_robot(robot_name, robots_file):
    """Match the world launch: N_ROBOTS overrides robots_file."""
    n_robots_env = os.environ.get('N_ROBOTS')
    if n_robots_env:
        return _grid_pose(int(n_robots_env), robot_name)
    with open(robots_file, 'r') as f:
        data = yaml.safe_load(f)
    for r in data['robots']:
        if r['name'] == robot_name:
            return r
    raise RuntimeError(f'{robot_name!r} not found in {robots_file}')


PREFIXED_FRAMES = {'chassis', 'odom', 'imu_link', 'lidar_link', 'base_link'}

# Keys whose value names a TF frame that should be prefixed with <ns>/.
FRAME_KEYS = {
    'base_frame_id', 'odom_frame_id', 'global_frame_id',
    'robot_base_frame', 'global_frame', 'local_frame',
    'base_frame', 'fixed_frame',
    'frame_id', 'child_frame_id',
}

# Keys whose value names a topic that, if absolute, should be made relative
# so the node's namespace picks it up.
TOPIC_KEYS = {'topic', 'scan_topic', 'map_topic', 'odom_topic',
              'cmd_vel_in_topic', 'cmd_vel_out_topic', 'state_topic'}


def _rewrite_value(key, value, ns, map_yaml_path):
    if key == 'yaml_filename' and isinstance(value, str):
        # map_server expects an absolute path
        return map_yaml_path
    if key in FRAME_KEYS and isinstance(value, str) and value in PREFIXED_FRAMES:
        return f'{ns}/{value}'
    if key in TOPIC_KEYS and isinstance(value, str) and value.startswith('/'):
        # Strip the leading slash so the namespace pushes it.
        return value.lstrip('/')
    return value


def _rewrite_tree(node, ns, map_yaml_path):
    if isinstance(node, dict):
        return {
            k: _rewrite_tree(_rewrite_value(k, v, ns, map_yaml_path), ns, map_yaml_path)
            for k, v in node.items()
        }
    if isinstance(node, list):
        return [_rewrite_tree(v, ns, map_yaml_path) for v in node]
    return node


def _generate_params(ns, src_params, map_yaml_path, out_path):
    with open(src_params, 'r') as f:
        params = yaml.safe_load(f)
    params = _rewrite_tree(params, ns, map_yaml_path)
    with open(out_path, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)
    return out_path


# Lifecycle nodes the lifecycle_manager will start. Order matters for the
# navigation manager (controller before bt_navigator, etc.).
LOCALIZATION_NODES = ['map_server', 'amcl']
NAVIGATION_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
    'waypoint_follower',
    'docking_server',
]

# (package, executable, node_name) for each lifecycle node.
NAV2_NODES = [
    ('nav2_map_server', 'map_server', 'map_server'),
    ('nav2_amcl', 'amcl', 'amcl'),
    ('nav2_controller', 'controller_server', 'controller_server'),
    ('nav2_smoother', 'smoother_server', 'smoother_server'),
    ('nav2_planner', 'planner_server', 'planner_server'),
    ('nav2_behaviors', 'behavior_server', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
    ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
    ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
    ('opennav_docking', 'opennav_docking', 'docking_server'),
]


def generate_launch_description():
    pkg_multi = get_package_share_directory('mobile_robot_multi')
    pkg_nav = get_package_share_directory('mobile_robot_navigation2')

    default_robots_file = os.path.join(pkg_multi, 'config', 'robots.yaml')
    default_src_params = os.path.join(pkg_nav, 'param', 'mobile_robot.yaml')
    default_map = os.path.join(pkg_nav, 'map', 'map.yaml')

    # Resolve at parse time. Override via env vars (same pattern as the world
    # launch so the two stay in lockstep).
    robot_name = os.environ.get('NAV2_ROBOT', 'robot1')
    robots_file = os.environ.get('ROBOTS_FILE', default_robots_file)
    src_params = os.environ.get('NAV2_PARAMS', default_src_params)
    map_yaml = os.environ.get('NAV2_MAP', default_map)

    robot = _resolve_robot(robot_name, robots_file)
    print(f'[multi_robot_nav2] {robot_name} pose: '
          f'x={robot["x"]} y={robot["y"]} yaw={robot["yaw"]}')

    params_out = f'/tmp/{robot_name}_nav2.yaml'
    _generate_params(robot_name, src_params, map_yaml, params_out)
    print(f'[multi_robot_nav2] generated params: {params_out}')

    common_params = [params_out, {'use_sim_time': True}]

    nav_nodes = [
        Node(
            package=pkg,
            executable=exe,
            name=name,
            namespace=robot_name,
            output='screen',
            parameters=common_params,
        )
        for (pkg, exe, name) in NAV2_NODES
    ]

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 20.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0,
            'node_names': LOCALIZATION_NODES,
        }],
    )

    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 20.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0,
            'node_names': NAVIGATION_NODES,
        }],
    )

    return LaunchDescription([
        *nav_nodes,
        lifecycle_localization,
        lifecycle_navigation,
    ])
