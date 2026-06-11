# Navigation

## Nav2 (DWB controller + NavFn planner)

Nav2 runs against the saved map produced during SLAM — no slam_toolbox needed at navigation time.

### Single robot

```bash
# Terminal 1 — Gazebo world + robot
ros2 launch mobile_robot_gazebo mobile_robot_10x10_world.launch.py

# Terminal 2 — Nav2 (map_server + AMCL + planner + controller) + RViz
ros2 launch mobile_robot_navigation2 single_robot_nav2.launch.py use_sim_time:=true
```

> **Picking a map.** Maps live at `mobile_robot_navigation2/maps/<name>/{map.yaml,map.pgm}`. The default is `10x10`. To use another, drop the directory and pass `map_name:=<name>`:
> ```bash
> ros2 launch mobile_robot_navigation2 single_robot_nav2.launch.py use_sim_time:=true map_name:=<name>
> ```
> An absolute path still overrides everything: `map:=/abs/path/map.yaml`.

### N robots

Replace `<n>` with the number of robots (e.g. `N_ROBOTS=3`). Both launches read the same `N_ROBOTS` env var and lay the robots out on a centered square grid.

```bash
# Terminal 1 — spawn N robots into the shared Gazebo world
N_ROBOTS=<n> ros2 launch mobile_robot_multi multi_robot_world.launch.py

# Terminal 2 — bring up Nav2 in each robot's namespace + multi-robot RViz
N_ROBOTS=<n> ros2 launch mobile_robot_multi multi_robot_nav2.launch.py

# Pick a different map subdirectory:
NAV2_MAP_NAME=<name> N_ROBOTS=<n> ros2 launch mobile_robot_multi multi_robot_nav2.launch.py
```

Each robot's stack runs under `/<robot>/...` with frames `<robot>/chassis`, `<robot>/odom`, etc. The `map` frame is shared. RViz's Goal/InitialPose tools target the **first** robot by default — for multi-robot goal dispatch, use a `nav2_simple_commander` script per namespace.

In RViz, click **2D Pose Estimate** and set the initial pose of the robot.
To move to a goal, click **Nav2 Goal** and set the goal location and pose.

---

## Custom navigation (go-to-goal, no Nav2)

`mobile_robot_custom_nav` provides a lightweight alternative to Nav2: a hand-written action server that drives the robot to a `(x, y, yaw)` target using a two-phase cascaded control loop.

**Phase 1** — rotate toward the goal, then drive forward until within `goal_tolerance` (default 0.10 m).  
**Phase 2** — spin in place until within `heading_tolerance` (default 10°) of the target yaw.

Velocity commands come from `mobile_robot_control`'s **CascadedController** (outer position loop + inner velocity loop with asymmetric accel/decel limits).

```bash
# Terminal 1 — Gazebo
ros2 launch mobile_robot_gazebo mobile_robot_10x10_world.launch.py

# Terminal 2 — go-to-goal action server
ros2 run mobile_robot_custom_nav go_to_goal_server

# Terminal 3a — send a goal from the CLI
ros2 run mobile_robot_custom_nav go_to_goal_client 3.0 2.0        # x=3, y=2, yaw=0
ros2 run mobile_robot_custom_nav go_to_goal_client 3.0 2.0 1.57   # with target yaw

# Terminal 3b — or drop a goal in RViz with the "2D Goal Pose" tool
ros2 run mobile_robot_custom_nav goal_bridge_node
```

---

## Motion controllers

`mobile_robot_control` ships two interchangeable controllers, both implementing the same `RobotController` base interface so the go-to-goal server is unaware of which one is active:

| Controller | Description |
|---|---|
| `ProportionalController` | P control with asymmetric accel/decel rate limiting |
| `CascadedController` | Outer P(position) → inner P(velocity) loop with rate limiting; requires odometry feedback via `update_feedback()` |

Controller gains, velocity caps, and acceleration limits are tuned in YAML and loaded at runtime:

```
mobile_robot_control/config/proportional_controller_params.yaml
mobile_robot_control/config/cascaded_controller_params.yaml
```

---

## Velocity monitor

`mobile_robot_monitor` opens a live PyQtGraph window that plots the cmd_vel setpoint against the actual odom velocity over a rolling 10-second window — useful for tuning controller gains:

```bash
ros2 run mobile_robot_monitor velocity_plotter
```
