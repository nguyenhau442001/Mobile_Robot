# Mobile Robot

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square)
![Genesis](https://img.shields.io/badge/Genesis-0.4.7-purple?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python)
![Stars](https://img.shields.io/github/stars/nguyenhau442001/Differential_Drive_Mobile_Robot?style=flat-square)
![Last Commit](https://img.shields.io/github/last-commit/nguyenhau442001/Differential_Drive_Mobile_Robot?style=flat-square)


**Supported platforms:** Ubuntu 24.04 · macOS 26.5 Tahoe

A full-stack mobile robotics simulation project built on **ROS 2 Jazzy**,
**Gazebo Sim (Harmonic)**, and **Genesis** — from a single URDF to a
fleet of 100 robots navigating in parallel.

▶️ **[Watch Full Demo on YouTube](https://www.youtube.com/watch?v=wHatIgc6cNg&t=398s)**

---

## What's inside

| Capability | Stack |
|---|---|
| Robot modelling | URDF / xacro, IMU, 2D LiDAR, interactive dimensions editor |
| Physics simulation | Gazebo Sim Harmonic (ODE / TPE / Bullet / DART), MuJoCo, Genesis |
| Autonomous mapping | slam_toolbox (online async) |
| Autonomous navigation | Nav2 — AMCL, NavFn planner, DWB controller |
| Custom navigation | Hand-written go-to-goal action server (no Nav2) |
| Motion control | Proportional controller · Cascaded P(pos)→P(vel) controller |
| Multi-robot | N robots sharing one map, independent Nav2 stacks |
| Teleoperation | Keyboard teleop (incremental) · Trapezoidal velocity profiler |
| Velocity monitoring | Real-time cmd_vel vs odom plotter (PyQtGraph) |
| Web dashboard | rosbridge + roslibjs / ros2djs / ros3djs — map, LiDAR, teleop in the browser |
| Fleet simulation | Genesis (Apple Metal / CUDA) — 100+ robots, batched physics, task-assignment fleet manager |
| Benchmarking | RTF measurement across physics engines, real-time factor analysis |

---

## Packages

| Package | Description |
|---|---|
| `mobile_robot_description` | URDF/xacro robot models (Rooster & YuHou). Includes a PyQt5 GUI editor (`robot_dimensions_config_editor`) for live-editing robot dimensions. |
| `mobile_robot_gazebo` | Launch files for spawning the robot in Gazebo (10×10 and AWS small-warehouse worlds) and bridging topics. |
| `mobile_robot_slam` | slam_toolbox bring-up (online async) + RViz for mapping. |
| `mobile_robot_navigation2` | Nav2 bring-up against a saved map (AMCL, DWB controller, NavFn planner). |
| `mobile_robot_custom_nav` | Custom go-to-goal action server/client and RViz goal bridge. Nav2-free alternative — drives the robot to a (x, y, yaw) pose using the cascaded controller. |
| `mobile_robot_interfaces` | Custom ROS 2 interfaces: `GoToGoal.action` (used by `mobile_robot_custom_nav`). |
| `mobile_robot_control` | Interchangeable velocity controllers (Proportional, Cascaded), incremental keyboard teleop node, shared math utilities (`yaw_from_quaternion`, `normalize_angle`, `clamp`), and unit/integration tests. |
| `mobile_robot_multi` | Multi-robot Gazebo + Nav2 bring-up (N robots in one world). |
| `mobile_robot_teleop` | Python nodes for teleoperation (keyboard control, trapezoidal velocity controller). |
| `mobile_robot_monitor` | Real-time velocity monitor: plots cmd_vel setpoint vs odom actual in a rolling 10-second window (PyQtGraph + PyQt5). |
| `mobile_robot_web` | Browser dashboard via rosbridge + roslibjs / ros2djs / ros3djs (map, lidar scan, pose, goal, teleop). |
| `mobile_robot_genesis` | Genesis physics scripts for batched fleet simulation (100+ robots in parallel envs, fleet manager with task assignment). |
| `mobile_robot_mujoco` | Standalone MuJoCo / Gymnasium scratch scripts. |
| `mobile_robot` | Meta-package depending on description, gazebo, navigation2, and teleop. |

## Highlights

- **AWS small-warehouse world** — SLAM + Nav2 tested in a realistic
  warehouse layout, not just an empty box
- **Genesis fleet manager** — 100 differential-drive robots in a single
  batched physics step; nearest-task assignment, completion metrics,
  Apple M-series Metal GPU support
- **Browser dashboard** — control and monitor the robot from any browser,
  no RViz needed; same-origin HTTP server eliminates CORS issues
- **Physics engine benchmarking** — reproducible RTF measurement script
  to compare ODE vs TPE vs Bullet vs DART on the same world
- **Custom navigation stack** — hand-written go-to-goal action server with
  a cascaded P(position)→P(velocity) controller as an alternative to Nav2
- **Interchangeable controllers** — swap between Proportional and Cascaded
  controllers at launch time without changing the navigation code

## Documentation

| Topic | Link |
|---|---|
| Environment setup | [docs/setup.md](docs/setup.md) |
| Robot description & kinematics | [docs/robot-description.md](docs/robot-description.md) |
| SLAM & Gazebo worlds | [docs/slam.md](docs/slam.md) |
| Navigation (Nav2 + custom go-to-goal) | [docs/navigation.md](docs/navigation.md) |
| Simulation (RTF benchmarking + Genesis fleet) | [docs/simulation.md](docs/simulation.md) |
| Web dashboard | [docs/web-dashboard.md](docs/web-dashboard.md) |

---

## Known macOS limitations

- `gz sim` requires separate server/GUI processes — [upstream issue](https://github.com/gazebosim/gz-sim/issues/44).
- OGRE-Next (`ogre2`) fails on macOS arm64; uses OGRE 1.x instead.
- DDS multicast must be restricted to localhost via `GZ_IP` and `ROS_AUTOMATIC_DISCOVERY_RANGE`.
