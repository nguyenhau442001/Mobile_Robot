# Mobile Robot

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square)
![Genesis](https://img.shields.io/badge/Genesis-0.4.7-purple?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python)
![Stars](https://img.shields.io/github/stars/nguyenhau442001/Differential_Drive_Mobile_Robot?style=flat-square)
![Last Commit](https://img.shields.io/github/last-commit/nguyenhau442001/Differential_Drive_Mobile_Robot?style=flat-square)

<img width="3018" height="1892" alt="image" src="https://github.com/user-attachments/assets/fd8911b9-ba57-4429-86a3-b67e4e172fff" />

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
| Robot description | [docs/robot-description.md](docs/robot-description.md) |
| Kinematics | [docs/kinematics.md](docs/kinematics.md) |
| SLAM & Gazebo worlds | [docs/slam.md](docs/slam.md) |
| Navigation (Nav2 + custom go-to-goal) | [docs/navigation.md](docs/navigation.md) |
| Simulation (RTF benchmarking + Genesis fleet) | [docs/simulation.md](docs/simulation.md) |
| Web dashboard | [docs/web-dashboard.md](docs/web-dashboard.md) |

---

## Known macOS limitations

- `gz sim` requires separate server/GUI processes — [upstream issue](https://github.com/gazebosim/gz-sim/issues/44).
- OGRE-Next (`ogre2`) fails on macOS arm64; uses OGRE 1.x instead.
- DDS multicast must be restricted to localhost via `GZ_IP` and `ROS_AUTOMATIC_DISCOVERY_RANGE`.

---

# 🛠️ Advanced Autonomous Mobile Robot Platform - Development Roadmap

This roadmap outlines the systematic upgrade of the `Mobile_Robot` repository to bridge the gap toward **Embedded AI Robotics** and **Heterogeneous System Architectures** (targeting modern SoCs like Renesas R-Car/RZ series).

---

## 📌 Phase 1: AI Perception & Vision Pipeline (`robot_ai_perception`)
*Objective: Integrate real-time deep learning inference into the ROS2 framework using software simulation.*

- [ ] **1.1. ROS2 Camera Integration Package**
  - Create a dedicated ROS2 package `robot_ai_perception` inside the workspace.
  - Implement a subscriber node to capture raw image streams and depth data published by the Gazebo (Gz-Sim) RGB-D camera sensor.
- [ ] **1.2. Deep Learning Model Integration (ONNX Runtime / TensorRT)**
  - Integrate a lightweight object detection model (e.g., YOLOv8-nano or MobileNet SSD) optimized for edge deployment.
  - Setup pre-processing and post-processing pipelines in C++/Python to handle inference on simulated frames.
- [ ] **1.3. ROS2 Custom Perception Topic Publishing**
  - Publish processed bounding boxes, object classes, and relative 3D coordinates to `/perception/detected_objects`.
  - Ensure message synchronization using `message_filters` to align sensor data with robot state.

---

## 📌 Phase 2: Simulation & Advanced Navigation Stack (`Nav2 & Gz-Sim`)
*Objective: Validate autonomous navigation and sensor fusion algorithms in a complex virtual environment.*

- [ ] **2.1. Virtual Warehouse Environment Modeling**
  - Design and construct a complex indoor map (warehouse/office layout) within Gazebo (Gz-Sim).
  - Configure walls, obstacles, and dynamic elements to test edge-case navigation scenarios.
- [ ] **2.2. Robust Sensor Fusion & Localization**
  - Integrate LiDAR (`/scan`), IMU (`/imu`), and wheel encoder data.
  - Tune the Extended Kalman Filter (EKF) via Robot Localization package to produce a drift-free `/odom` topic.
- [ ] **2.3. Navigation2 (Nav2) Stack Tuning**
  - Setup Global Planner and Local Planner parameters (e.g., TEB Local Planner or DWB Controller) for smooth trajectory generation.
  - Implement obstacle avoidance behavior reacting directly to inputs from the AI perception pipeline.
- [ ] **2.4. Simulation Automation Scripts**
  - Write Bash/Python automation scripts to launch multi-node testing environments automatically, enabling repeatable regression testing for path planning.

---

## 📌 Phase 3: Heterogeneous Architecture & Low-Level Middleware Simulation (`/architecture_sim`)
*Objective: Simulate multi-core task offloading and low-latency inter-process communication (IPC) inspired by heterogeneous SoCs (Cortex-A / Cortex-R / Cortex-M).*

- [ ] **3.1. Multithreading Task Allocation Framework**
  - Build a C++ module simulating asymmetric multi-processing (AMP) behavior:
    - **High-Performance Core simulation (Cortex-A style):** Handling heavy tasks like path planning, mapping, and AI inference logic.
    - **Real-Time Core simulation (Cortex-R/M style):** Handling high-frequency sensor polling, motor control loops, and safety watchdog interrupts.
- [ ] **3.2. Low-Latency Inter-Process Communication (IPC)**
  - Implement Shared Memory (`shm`) and Lock-free Message Queues to minimize communication latency between the simulated high-performance and real-time tiers.
  - Benchmark message passing overhead under various load conditions to demonstrate real-time readiness.

