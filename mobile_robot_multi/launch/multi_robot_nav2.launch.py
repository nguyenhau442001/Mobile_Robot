"""Bring up a Nav2 stack inside each robot's namespace.

Stage 1: a single robot. Stack runs under /<robot>/... with frame IDs prefixed
(robot_base_frame=<robot>/chassis, odom_frame_id=<robot>/odom). The `map` frame
stays shared. No /tf remap is applied, so nav2 reads from and writes to the
absolute /tf shared with the multi_robot_world TF tree.

Per the design choice in multi_robot_world.launch.py, RSP publishes to absolute
/tf with frames already namespaced (robot1/odom, robot1/chassis, ...). Nav2's
default ros_remap of /tf -> tf would fight that, so this launch does NOT use
nav2_bringup's launch files and instead instantiates the nav2 nodes directly
inside the namespace with no /tf remap.
"""

import colorsys
import glob
import math
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_FIND_PKG_SHARE_RE = re.compile(r'\$\(find-pkg-share\s+([\w_-]+)\)')


def _expand_substitutions(value):
    """Resolve $(find-pkg-share <pkg>) ourselves — yaml.safe_load doesn't."""
    if not isinstance(value, str):
        return value
    return _FIND_PKG_SHARE_RE.sub(
        lambda m: get_package_share_directory(m.group(1)), value
    )


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


def _grid_pose(n_robots, robot_name, spacing=1.5):
    """Mirror multi_robot_world._grid_robots so AMCL initial pose matches spawn.

    Robots are named robot1..robotN on a centered square grid, all facing +X.
    """
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


def _grid_robots(n, spacing=1.5):
    """All N robots on a centered square grid — mirrors multi_robot_world."""
    return [_grid_pose(n, f'robot{i + 1}', spacing) for i in range(n)]


def _resolve_robots(robots_file):
    """Match the world launch: N_ROBOTS overrides robots_file."""
    n_robots_env = os.environ.get('N_ROBOTS')
    if n_robots_env:
        return _grid_robots(int(n_robots_env))
    with open(robots_file, 'r') as f:
        data = yaml.safe_load(f)
    return data['robots']


def _color_for(i, n):
    """Distinct RGB string per robot — same hue spread as multi_robot_world."""
    r, g, b = colorsys.hsv_to_rgb(i / max(n, 1), 0.7, 0.9)
    return f'{int(r * 255)}; {int(g * 255)}; {int(b * 255)}'


def _rewrite_value(key, value, ns, map_yaml_path):
    value = _expand_substitutions(value)
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


def _load_param_dir(param_dir):
    """Merge every *_params.yaml in `param_dir` into one dict.

    Collide-check on duplicate top-level keys so a typo fails loudly.
    """
    files = sorted(glob.glob(os.path.join(param_dir, '*_params.yaml')))
    # Skip broken symlinks — colcon --symlink-install leaves stale symlinks
    # in install/ when source files are renamed.
    files = [f for f in files if os.path.isfile(f)]
    if not files:
        raise RuntimeError(f'No *_params.yaml files in {param_dir}')
    combined = {}
    for f in files:
        with open(f) as fp:
            data = yaml.safe_load(fp) or {}
        for k, v in data.items():
            if k in combined:
                raise RuntimeError(f'Duplicate top-level key {k!r} in {f}')
            combined[k] = v
    return combined


def _generate_params(robot, ns, src_params_dir, map_yaml_path, out_path):
    params = _load_param_dir(src_params_dir)

    params = _rewrite_tree(params, ns, map_yaml_path)

    # AMCL: seed initial pose from the spawn pose so it converges immediately.
    amcl = params.get('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {
        'x': float(robot.get('x', 0.0)),
        'y': float(robot.get('y', 0.0)),
        'z': float(robot.get('z', 0.0)),
        'yaw': float(robot.get('yaw', 0.0)),
    }

    # Re-root under the namespace so /<ns>/<node> finds its params. Same trick
    # nav2_bringup's RewrittenYaml(root_key=ns) uses.
    wrapped = {ns: params}

    with open(out_path, 'w') as f:
        yaml.safe_dump(wrapped, f, sort_keys=False)
    return out_path


RVIZ_HEADER = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset: {X: 0, Y: 0, Z: 0}
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.5
      Name: TF
      Show Arrows: false
      Show Axes: true
      Show Names: false
      Update Interval: 0
      Value: true
"""


RVIZ_ROBOT_BLOCK = """    - Class: rviz_common/Group
      Displays:
        - Alpha: 0.7
          Class: rviz_default_plugins/Map
          Color Scheme: map
          Draw Behind: true
          Enabled: true
          Name: Map
          Topic:
            Depth: 1
            Durability Policy: Transient Local
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /{ns}/map
          Use Timestamp: false
          Value: true
        - Alpha: 0.7
          Class: rviz_default_plugins/Map
          Color Scheme: costmap
          Draw Behind: false
          Enabled: false
          Name: GlobalCostmap
          Topic:
            Depth: 1
            Durability Policy: Transient Local
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /{ns}/global_costmap/costmap
          Use Timestamp: false
          Value: true
        - Alpha: 0.7
          Class: rviz_default_plugins/Map
          Color Scheme: costmap
          Draw Behind: false
          Enabled: false
          Name: LocalCostmap
          Topic:
            Depth: 1
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /{ns}/local_costmap/costmap
          Use Timestamp: false
          Value: true
        - Alpha: 1
          Class: rviz_default_plugins/RobotModel
          Description Source: Topic
          Description Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /{ns}/robot_description
          Enabled: true
          Links:
            All Links Enabled: true
            Expand Joint Details: false
            Expand Link Details: false
            Expand Tree: false
            Link Tree Style: Links in Alphabetic Order
          Name: RobotModel
          TF Prefix: {ns}
          Update Interval: 0
          Value: true
          Visual Enabled: true
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds: {{Max Value: 10, Min Value: -10, Value: true}}
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/LaserScan
          Color: {color}
          Color Transformer: FlatColor
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 0
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: LaserScan
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.04
          Style: Flat Squares
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Best Effort
            Value: /{ns}/scan
          Use Fixed Frame: true
          Use rainbow: false
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz_default_plugins/Path
          Color: {color}
          Enabled: true
          Head Diameter: 0.3
          Head Length: 0.2
          Length: 0.3
          Line Style: Lines
          Line Width: 0.03
          Name: Plan
          Offset: {{X: 0, Y: 0, Z: 0}}
          Pose Color: 255; 85; 255
          Pose Style: None
          Radius: 0.03
          Shaft Diameter: 0.1
          Shaft Length: 0.1
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /{ns}/plan
          Value: true
      Enabled: true
      Name: {ns}
"""


# RViz Goal/InitialPose tools only support one topic each. Default-target the
# first robot; for multi-robot goal dispatch use nav2_simple_commander scripts.
RVIZ_FOOTER = """  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /{first_ns}/initialpose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /{first_ns}/goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Angle: 0
      Class: rviz_default_plugins/TopDownOrtho
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Scale: 60
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz_default_plugins)
      X: 0
      Y: 0
    Saved: ~
"""


def _generate_rviz_config(robots, out_path):
    n = len(robots)
    blocks = ''.join(
        RVIZ_ROBOT_BLOCK.format(ns=r['name'], color=_color_for(i, n))
        for i, r in enumerate(robots)
    )
    body = RVIZ_HEADER + blocks + RVIZ_FOOTER.format(first_ns=robots[0]['name'])
    with open(out_path, 'w') as f:
        f.write(body)
    return out_path


# Lifecycle nodes that the lifecycle_manager will start. Order matters for
# the navigation manager (controller before bt_navigator, etc.).
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


def _robot_nav2_actions(robot, src_params_dir, map_yaml):
    """Per-robot: params yaml + all nav2 nodes + 2 lifecycle managers."""
    ns = robot['name']
    params_out = f'/tmp/{ns}_nav2.yaml'
    _generate_params(robot, ns, src_params_dir, map_yaml, params_out)
    print(f'[multi_robot_nav2] {ns} pose: '
          f'x={robot["x"]} y={robot["y"]} yaw={robot["yaw"]}  '
          f'params={params_out}')

    common_params = [params_out, {'use_sim_time': True}]

    nodes = [
        Node(
            package=pkg,
            executable=exe,
            name=name,
            namespace=ns,
            output='screen',
            parameters=common_params,
        )
        for (pkg, exe, name) in NAV2_NODES
    ]

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=ns,
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
        namespace=ns,
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

    return [*nodes, lifecycle_localization, lifecycle_navigation]


def generate_launch_description():
    pkg_multi = get_package_share_directory('mobile_robot_multi')
    pkg_nav = get_package_share_directory('mobile_robot_navigation2')

    default_robots_file = os.path.join(pkg_multi, 'config', 'robots.yaml')
    default_src_params_dir = os.path.join(pkg_nav, 'param')
    default_map_name = os.environ.get('NAV2_MAP_NAME', '10x10')
    default_map = os.path.join(pkg_nav, 'maps', default_map_name, 'map.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true')

    # Resolve at parse time so we can pre-generate per-robot yaml + RViz config.
    # NAV2_PARAMS now points to a directory of *_params.yaml files (was a
    # single mobile_robot.yaml file); merged in-memory before per-namespace
    # rewriting.
    robots_file = os.environ.get('ROBOTS_FILE', default_robots_file)
    src_params_dir = os.environ.get('NAV2_PARAMS', default_src_params_dir)
    map_yaml = os.environ.get('NAV2_MAP', default_map)
    robots = _resolve_robots(robots_file)
    print(f'[multi_robot_nav2] bringing up {len(robots)} robots '
          f'using params from {src_params_dir}')

    nav_actions = []
    for r in robots:
        nav_actions.extend(_robot_nav2_actions(r, src_params_dir, map_yaml))

    # Shared map. AMCL publishes map -> <ns>/odom per robot; the world -> map
    # static anchors the map frame so the multi_robot_world `world` frame still
    # resolves in TF tools.
    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'map',
        ],
        output='screen',
    )

    rviz_config = '/tmp/multi_robot_nav2.rviz'
    _generate_rviz_config(robots, rviz_config)
    print(f'[multi_robot_nav2] generated rviz config: {rviz_config}')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        rviz_arg,
        world_to_map,
        *nav_actions,
        rviz,
    ])
