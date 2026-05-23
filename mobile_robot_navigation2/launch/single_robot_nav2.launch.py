# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import glob
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _merge_param_files(param_dir):
    """Merge every *_params.yaml in `param_dir` into one /tmp YAML.

    Each split file owns disjoint top-level node keys (amcl, bt_navigator, ...).
    We collide-check on merge so a typo or duplicate node block fails loudly
    instead of silently shadowing.
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

    out = '/tmp/mobile_robot_nav2_combined.yaml'
    with open(out, 'w') as fp:
        yaml.safe_dump(combined, fp, sort_keys=False)
    print(f'[single_robot_nav2] merged {len(files)} param files -> {out}')
    for f in files:
        print(f'  - {os.path.basename(f)}')
    return out


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_share = get_package_share_directory('mobile_robot_navigation2')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_share, 'map', 'map.yaml'))

    # Build the combined params YAML at parse time so nav2_bringup gets a
    # single file (its API expects one path, not a directory).
    combined_params = _merge_param_files(os.path.join(pkg_share, 'param'))
    param_dir = LaunchConfiguration('params_file', default=combined_params)

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        pkg_share, 'rviz', 'mobile_robot_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
