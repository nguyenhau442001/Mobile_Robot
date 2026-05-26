# Differential Drive Mobile Robot

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square)
![Genesis](https://img.shields.io/badge/Genesis-0.4.7-purple?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python)
![Platform](https://img.shields.io/badge/Platform-Ubuntu_24.04-E95420?style=flat-square&logo=ubuntu)
![Stars](https://img.shields.io/github/stars/nguyenhau442001/Differential_Drive_Mobile_Robot?style=flat-square)
![Last Commit](https://img.shields.io/github/last-commit/nguyenhau442001/Differential_Drive_Mobile_Robot?style=flat-square)

A full-stack mobile robotics simulation project built on **ROS 2 Jazzy**,
**Gazebo Sim (Harmonic)**, and **Genesis** — from a single URDF to a
fleet of 100 robots navigating in parallel.

▶️ **[Watch Full Demo on YouTube](https://www.youtube.com/watch?v=wHatIgc6cNg&t=398s)**

---

## What's inside

| Capability | Stack |
|---|---|
| Robot modelling | URDF / xacro, IMU, 2D LiDAR |
| Physics simulation | Gazebo Sim Harmonic (ODE / TPE / Bullet / DART), Mujoco, Genesis |
| Autonomous mapping | slam_toolbox (online async) |
| Autonomous navigation | Nav2 — AMCL, NavFn planner, DWB controller |
| Multi-robot | N robots sharing one map, independent Nav2 stacks |
| Teleoperation | Keyboard node + trapezoidal velocity profile controller |
| Web dashboard | rosbridge + roslibjs / ros2djs / ros3djs — map, LiDAR, teleop in the browser |
| Fleet simulation | Genesis (Apple Metal / CUDA) — 100+ robots, batched physics, task-assignment fleet manager |
| Benchmarking | RTF measurement across physics engines, real-time factor analysis |

---

- **mobile_robot_description** → Robot geometry and physical description (URDF/xacro).
- **mobile_robot_gazebo** → Launch files for spawning the robot in Gazebo (10x10 and AWS small-warehouse worlds) and bridging topics.
- **mobile_robot_slam** → slam_toolbox bring-up (online async) + RViz for mapping.
- **mobile_robot_navigation2** → Nav2 bring-up against a saved map (AMCL, DWB controller, NavFn planner).
- **mobile_robot_multi** → Multi-robot Gazebo + Nav2 bring-up (N robots in one world).
- **mobile_robot_teleop** → Python nodes for teleoperation (keyboard control, trapezoidal velocity controller).
- **mobile_robot_web** → Browser dashboard via rosbridge + roslibjs / ros2djs / ros3djs (map, lidar scan, pose, goal, teleop).
- **mobile_robot_genesis** → Genesis physics scripts for batched fleet simulation (100+ robots in parallel envs, fleet manager with task assignment).
- **mobile_robot_mujoco** → Standalone MuJoCo / Gymnasium scratch scripts.
- **mobile_robot** → Meta-package depending on description, gazebo, navigation2, and teleop.

## Highlights

- **Trapezoidal velocity controller** — 0 → 5 m/s in 5 s, full stop in 1 s,
  verified against IMU and odometry via rqt_plot
- **AWS small-warehouse world** — SLAM + Nav2 tested in a realistic
  warehouse layout, not just an empty box
- **Genesis fleet manager** — 100 differential-drive robots in a single
  batched physics step; nearest-task assignment, completion metrics,
  Apple M-series Metal GPU support
- **Browser dashboard** — control and monitor the robot from any browser,
  no RViz needed; same-origin HTTP server eliminates CORS issues
- **Physics engine benchmarking** — reproducible RTF measurement script
  to compare ODE vs TPE vs Bullet vs DART on the same world

## 1. Environment Setup
Install ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html
```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-xacro ros-jazzy-tf2-tools \
  ros-jazzy-robot-localization
```

### Python virtual environment (for `trimesh` and other pip-only deps)
Ubuntu 24.04 is PEP 668 protected, so pip-only packages (like `trimesh`) cannot be installed into the system Python directly. We use a venv with `--system-site-packages` so ROS 2 packages (`rclpy`, etc.) remain importable from inside the venv.

```bash
# One-time: install the venv module (Ubuntu 24.04 / Python 3.12)
sudo apt install -y python3.12-venv

# Create the venv (inherits system site-packages so ROS 2 still works)
python3 -m venv --system-site-packages ~/.venvs/robot

# Install this workspace's Python deps (declared in pyproject.toml)
~/.venvs/robot/bin/pip install -e ~/ros2_ws/src/Differential_Drive_Mobile_Robot
```

Activate it before running ROS 2 nodes that need the pip-installed deps:
```bash
source ~/.venvs/robot/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Verify:
```bash
python -c "import trimesh, rclpy; print('trimesh', trimesh.__version__, '| rclpy OK')"
```

## 2. Clone the source code from GitHub and build the packages
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:nguyenhau442001/Differential_Drive_Mobile_Robot.git
cd ~/ros2_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Source the workspace in every new shell
ROS 2 does not source environments globally — every new terminal needs the ROS 2 distro and this workspace's `install/` overlay sourced before `ros2 run` / `ros2 launch` can find any package in this repo. Append both `source` lines to your shell rc once so it happens automatically on every new shell:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

If you also use the Python venv from section 1, activate it **before** the ROS sources by prepending to the same file.

## 3. Robot description
  Robot Structure Overview:
  - Chassis (base)
  - 4 Caster links (non-driven support wheels)
  - Left & Right drive wheels
  - IMU sensor
  - Lidar sensor (180° forward Field of View, 181 samples at 1° resolution, 5 m range, 5 Hz)

<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/f2a69b68-876a-4dc6-a20b-09cbea5dca7f" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/bc2bd408-ba4a-40ff-aa22-a653fca2b56f" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/b5821ce8-12c4-463e-9612-f24e65e3f2ed" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/b8801daf-88c0-42fd-bf59-0ac4c683639e" />

To view the TF tree, run:

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```
<img width="1660" height="355" alt="image" src="https://github.com/user-attachments/assets/f0e7d94b-a6f3-4449-afa0-6563179bcbc4" />


## 4. SLAM
SLAM uses **slam_toolbox** (online async mode). The `mobile_robot_slam` launch file embeds slam_toolbox + RViz, so the whole mapping session needs only two terminals (Gazebo + SLAM) plus a teleop terminal. Source the workspace in every new shell (see [section 2](#source-the-workspace-in-every-new-shell)).

```bash
# Terminal 1 — launch the mobile robot in Gazebo (10x10 world by default)
ros2 launch mobile_robot_gazebo mobile_robot_10x10_world.launch.py
```
<img width="1842" height="787" alt="image" src="https://github.com/user-attachments/assets/73d45c8f-ca78-4eba-aee3-7f56daa7a36d" />

```bash
# Terminal 2 — launch slam_toolbox (online async) + RViz
ros2 launch mobile_robot_slam mobile_robot_slam_box.launch.py
```

<img width="1842" height="787" alt="image" src="https://github.com/user-attachments/assets/9b732f06-5fa5-4dbb-bd24-c7150c04f626" />

```bash
# Terminal 3 — drive the robot with the keyboard to scan the environment
ros2 run mobile_robot_teleop mobile_robot_teleop_key --ros-args -r cmd_vel:=/cmd_vel

Control Your Differential-Drive Mobile Robot!!!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
```

```bash
# Terminal 4 — save the map (writes map.yaml + map.pgm into mobile_robot_navigation2/maps/10x10/)
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/Differential_Drive_Mobile_Robot/mobile_robot_navigation2/maps/10x10/map
```

> The slam_toolbox params used here live in [mobile_robot_slam/param/slam_toolbox.yaml](mobile_robot_slam/param/slam_toolbox.yaml) (frames set to `chassis`/`odom`/`map`, scan topic `/scan`, sim time on). Override with `slam_params_file:=<path>` or `use_sim_time:=false` if needed.

### Other Gazebo worlds
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

## 5. Navigation (Nav2 with DWB controller and NavFn planner)
Nav2 runs against the saved map produced in section 4 — no slam_toolbox needed at navigation time. Source the workspace in every new shell (see [section 2](#source-the-workspace-in-every-new-shell)).

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
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/21b207db-d197-46dd-814f-11dad260dea4" />

In RViz, click **2D Pose Estimate** and set the initial pose of the robot.
To move to a goal, click **Nav2 Goal** and set the goal location and pose.
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/3aa62a34-f76b-467e-8cc2-cf3bae7c9bd4" />
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/ca103906-d5ed-46c6-affd-837b079a9dd5" />
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/aee2a359-069e-47e6-b6ca-138fd3075ada" />

## 6. Controller
Source the workspace in every new shell (see [section 2](#source-the-workspace-in-every-new-shell)).

### Verify the velocity
```bash
ros2 run rqt_plot rqt_plot
ros2 launch mobile_robot_gazebo mobile_robot_10x10_world.launch.py
ros2 run mobile_robot_teleop mobile_robot_teleop_key --ros-args -r cmd_vel:=/cmd_vel
```

Add the two topics below to the plot to check the velocity response when increasing velocity slowly:

`/cmd_vel/linear/x`          => Set Point (SP)

`/odom/twist/twist/linear/x` => Process Variable (PV)

To monitor acceleration, add `/imu/linear_acceleration/x` to rqt_plot.

<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/3ed75f91-36b0-4e06-a09b-962bb9176319" />

---------------
# Case study: The robot should reach 5 m/s within 5 s from standstill, and stop completely within 1 s.

To meet this requirement, a controller was created based on the profile below:

The motion has three distinct phases:

- Acceleration phase (`t_acc`): Velocity increases linearly from 0 → `v_target`.
- Cruise phase (`t_cruise`):    Velocity stays constant at `v_target`.
- Deceleration phase (`t_dec`): Velocity decreases smoothly to 0 using a cosine function.

```
velocity
   ^
   |           ┌──────────────┐
   |          /|              |\
   |         / |              | \
   |        /  |              |  \
   |       /   |              |   \
   +-----------------------------------> time
       Accel     Cruise       Decel
```

ROS Node: Trapezoidal Velocity Controller
-----------------------------------------
This node generates a trapezoidal velocity profile and publishes
linear velocity commands to the `/cmd_vel` topic.

The profile consists of three phases:

  1. Acceleration phase - linearly increases velocity from 0 to `v_target`
  2. Cruise phase       - maintains a constant target velocity
  3. Deceleration phase - smoothly decreases velocity to 0 using a cosine function

Why discretize?
---------------
Instead of sending the velocity set point (`v_target`) directly to the robot,
we ramp velocity up and down over time (`dt`). This prevents jerky motion and
improves robot stability.


The `trapezoid_cmd_vel_node` was created to send the desired velocity to the robot.

```bash
ros2 launch mobile_robot_gazebo mobile_robot_10x10_world.launch.py
ros2 launch mobile_robot_teleop trapezoid_profile_controller.launch.py
```

The robot accelerates from standstill (t = 9) to 5 m/s (t = 14), then decelerates from 5 m/s (t = 14) to 0 m/s (t = 15).
<img width="1827" height="746" alt="image" src="https://github.com/user-attachments/assets/e92fa2c7-2004-4760-afc4-ae60f8adcf35" />

## 7. Real-Time Factor (RTF)

The **Real-Time Factor** is the ratio between simulated time and wall-clock time. RTF = 1.0 means the simulation advances at real speed; RTF < 1.0 means the physics step is too expensive for the host to keep up, and RTF > 1.0 means it is running faster than real time. Tracking RTF is the standard way to compare the cost of different physics engines (ODE, TPE, Bullet, DART) or to detect when world complexity has outgrown the host.

The world stats are published on `/world/<world_name>/stats` (`gz.msgs.WorldStatistics`). Two ways to read them, depending on whether you want a quick spot check or a reproducible measurement.

### 7.1 Quick check — `gz topic` one-liner

Single-shot inspection of the latest stats message:

```bash
gz topic --echo --topic /world/default/stats -n 1
```

Example output (only the relevant fields shown):
```
sim_time          { sec: 287  nsec: 897000000 }
real_time         { sec: 351  nsec: 671898917 }
iterations:       287897
real_time_factor: 1.01432737416001
step_size         { nsec: 1000000 }
```

Rolling average over the next 30 samples:

```bash
gz topic -e -t /world/default/stats \
  | grep real_time_factor \
  | head -30 \
  | awk '{sum += $2; count++} END {print "Average RTF:", sum/count}'
```

### 7.2 Reproducible benchmark — `physic_engines_rtf_measure.py`

For comparing physics engines or capturing the variance (not just the mean), use the bundled benchmark script. It subscribes to the stats topic for a fixed duration and reports mean, median, stdev, min, and max:

```bash
ros2 run mobile_robot_gazebo physic_engines_rtf_measure.py --duration 60 --world default
```

Example output:
```
Collecting RTF samples for 60s on /world/default/stats...

=== RTF Benchmark Results ===
  Samples     : 50
  Mean RTF    : 0.9062
  Median RTF  : 0.9997
  Stdev RTF   : 0.2657
  Min RTF     : 0.1304
  Max RTF     : 1.6060
```

**How to read the numbers.** A large gap between **mean** and **median** (here 0.91 vs 1.00) signals occasional stalls dragging the average down — the simulation is mostly real-time but loses ground during bursts of work (model spawns, sensor updates, contact spikes). A high **stdev** confirms that variance, and the **min/max** bracket the worst and best step the host produced over the window.

**Workflow for comparing physics engines.** Swap the engine in the SDF (`<physics name="..." type="ode|tpe|bullet|dart">`), restart Gazebo, run the script against the same world and duration, and compare medians (more robust than means under jitter).

## 8. Web dashboard

The `mobile_robot_web` package serves a browser-based dashboard that talks to ROS 2 over rosbridge. It renders the map, lidar scan, and robot pose, and exposes goal-setting and teleop — handy when you don't want to start RViz.

```bash
# Terminal 1 — Gazebo + robot (any world)
ros2 launch mobile_robot_gazebo mobile_robot_10x10_world.launch.py

# Terminal 2 — Nav2 (optional, only if you want goal-setting from the browser)
ros2 launch mobile_robot_navigation2 single_robot_nav2.launch.py use_sim_time:=true

# Terminal 3 — rosbridge (port 9090) + HTTP server (port 8000) + auto-open browser
ros2 launch mobile_robot_web web_bringup.launch.py
```

A single HTTP server serves both the dashboard assets and the URDF meshes from one port so the 3D view's STL fetches stay same-origin (no CORS plumbing). Override the defaults:

```bash
ros2 launch mobile_robot_web web_bringup.launch.py http_port:=8080 ws_port:=9091 browser:=google-chrome
```

> **Browser note.** The launch file defaults to Firefox because Chrome on llvmpipe (no-GPU VMs) refuses to enable WebGL. On Chrome, start it with `--enable-unsafe-swiftshader`, or pass `browser:=xdg-open` to use the system default.

## 9. Genesis — batched fleet simulation

The `mobile_robot_genesis` package contains standalone Genesis scripts that load the same URDF used in Gazebo (via [mobile_robot_genesis/scripts/xacro_loader.py](mobile_robot_genesis/scripts/xacro_loader.py)) and step large fleets in a single batched physics call — useful for fleet-scale RL or task-assignment experiments where launching 100 Gazebo robots is impractical.

```bash
# Sanity check — one robot, plain URDF → Genesis pipeline
python3 mobile_robot_genesis/scripts/spawn_mobile_robot.py

# 100 robots in parallel environments (one batched step covers all of them)
python3 mobile_robot_genesis/scripts/spawn_100_mobile_robots.py

# Fleet manager: 100 robots picking tasks off a shared queue,
# nearest-task assignment, kinematic motion, completion metrics
python3 mobile_robot_genesis/scripts/genesis_mobile_robot_fleet.py
```

Each script runs until the viewer window is closed. The xacro loader resolves `$(find <pkg>)` substitutions without needing the ROS environment sourced, so these scripts work from a plain Python venv as long as `genesis-world` and `xacro` are installed (both pulled in by `pip install -e .` against [pyproject.toml](pyproject.toml)).
