# SLAM

SLAM uses **slam_toolbox** (online async mode). The `mobile_robot_slam` launch file embeds slam_toolbox + RViz, so the whole mapping session needs only two terminals (Gazebo + SLAM) plus a teleop terminal.

```bash
# Terminal 1 — launch the mobile robot in Gazebo (10x10 world by default)
ros2 launch mobile_robot_gazebo mobile_robot_10x10_world.launch.py
```

<img width="3018" height="1892" alt="image" src="https://github.com/user-attachments/assets/fd8911b9-ba57-4429-86a3-b67e4e172fff" />

```bash
# Terminal 2 — launch slam_toolbox (online async) + RViz
ros2 launch mobile_robot_slam mobile_robot_slam_box.launch.py
```

<img width="3830" height="2102" alt="image" src="https://github.com/user-attachments/assets/9e054ebc-53e8-4093-803c-260d865d3b07" />

```bash
# Terminal 3 — drive the robot with the keyboard to scan the environment
ros2 run diff_drive_robot_control teleop_node

Differential-Drive Mobile Robot — Keyboard Teleop
--------------------------------------------------
        w
   a    s    d
        x

  Linear axis (forward / backward):
    w / W  →  +v  increase linear velocity by linear_step  (forward)
    x / X  →  -v  decrease linear velocity by linear_step  (backward)

  Angular axis (left / right):
    a / A  →  +ω  increase angular velocity by angular_step  (turn left,  CCW)
    d / D  →  -ω  decrease angular velocity by angular_step  (turn right, CW)

  Emergency stop:
    s / S / SPACE  →  zero both v and ω immediately

  Quit:
    Ctrl-C  →  exit
--------------------------------------------------

  linear: +0.00 m/s  |  angular: +0.00 rad/s

```

```bash
# Terminal 4 — save the map (writes map.yaml + map.pgm into mobile_robot_navigation2/maps/10x10/)
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/Mobile_Robot/mobile_robot_navigation2/maps/10x10/map
```

> The slam_toolbox params used here live in [mobile_robot_slam/param/slam_toolbox.yaml](../mobile_robot_slam/param/slam_toolbox.yaml) (frames set to `chassis`/`odom`/`map`, scan topic `/scan`, sim time on). Override with `slam_params_file:=<path>` or `use_sim_time:=false` if needed.

## Other Gazebo worlds

The `mobile_robot_gazebo` package also ships the AWS RoboMaker small-warehouse scene:

```bash
ros2 launch mobile_robot_gazebo small_warehouse.launch.py            # walls + ceiling
ros2 launch mobile_robot_gazebo no_roof_small_warehouse.launch.py    # open-top variant
```

Pick a clear spawn pose (avoiding shelves/clutter) with the bundled helper:

```bash
ros2 run mobile_robot_gazebo find_safe_spawn.py \
  $(ros2 pkg prefix mobile_robot_gazebo)/share/mobile_robot_gazebo/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world
# then pass the suggested coords:
ros2 launch mobile_robot_gazebo no_roof_small_warehouse.launch.py x_pos:=1.5 y_pos:=-2.5
```

Map the new world by running slam_toolbox the same way as above, then save under `mobile_robot_navigation2/maps/<name>/` so Nav2 can load it via `map_name:=<name>`.
