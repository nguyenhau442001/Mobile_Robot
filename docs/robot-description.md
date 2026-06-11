# Robot Description

## Robot structure

- Chassis (base)
- 4 Caster links (non-driven support wheels)
- Left & Right drive wheels
- IMU sensor
- Lidar sensor (180° forward Field of View, 181 samples at 1° resolution, 5 m range, 5 Hz)

To view the TF tree, run:

```bash
ros2 run tf2_tools view_frames
evince frames_*.pdf # Ubuntu
open frames_*.pdf   # macOS
```

<img width="2754" height="908" alt="image" src="https://github.com/user-attachments/assets/0bcd37cf-cb8d-4752-9c2d-6b183407cc4a" />

---

## Robot dimensions config editor

An interactive PyQt5 GUI lets you edit robot link dimensions (chassis, wheels, IMU, LiDAR) and save them back to `robot_dimensions_config.yaml` without touching YAML by hand:

```bash
ros2 run mobile_robot_description robot_dimensions_config_editor
```

The editor patches only the changed lines, preserving all comments and whitespace in the file.

---

## ROS 2 topic architecture

The firmware running on the motor controller board publishes and subscribes to these topics over a serial bridge:

**Published by the embedded controller:**

| Topic | Type | Notes |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Dead-reckoning pose + twist |
| `/imu` | `sensor_msgs/Imu` | Raw IMU readings |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint angles |
| `/tf` | `tf2_msgs/TFMessage` | `odom → base_link` transform |

**Subscribed by the embedded controller:**

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Linear `x` + angular `z` velocity command |

The velocity command from Nav2 (or from `mobile_robot_control`'s go-to-goal server) is discrete — recomputed every control cycle `Δt`. Because `Δt` is small (typically 50 ms), the piecewise-constant command approximates a continuous signal well enough for smooth motion.

---

→ For the full kinematic derivation see [docs/kinematics.md](kinematics.md).
