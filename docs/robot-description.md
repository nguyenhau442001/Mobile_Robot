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

## Robot dimensions config editor

An interactive PyQt5 GUI lets you edit robot link dimensions (chassis, wheels, IMU, LiDAR) and save them back to `robot_dimensions_config.yaml` without touching YAML by hand:

```bash
ros2 run mobile_robot_description robot_dimensions_config_editor
```

The editor patches only the changed lines, preserving all comments and whitespace in the file.

---

## Kinematics

### Differential drive model

A differential drive robot steers by varying the speed of its two wheels. At any instant the motion is locally linear, so the forward velocity of the robot's centre is simply the mean of the two wheel velocities:

```
v = (v_r + v_l) / 2
```

where `v_l` and `v_r` are the left and right wheel linear velocities.

The angular velocity about the robot's vertical axis is:

```
ω = (v_r - v_l) / L
```

where `L` is the wheel-to-wheel track width (distance between the two contact points).

Projecting `v` onto the world x–y plane and integrating gives the body velocity equations:

```
ẋ   = v · cos(θ)
ẏ   = v · sin(θ)
θ̇   = ω
```

In matrix form:

```
[ ẋ  ]   [ cos(θ)   0 ] [ v ]
[ ẏ  ] = [ sin(θ)   0 ] [ ω ]
[ θ̇  ]   [   0      1 ]
```

Inverting to find wheel speeds from a desired `(v, ω)` command (what `/cmd_vel` carries):

```
v_r = v + ω · L/2
v_l = v - ω · L/2
```

### Dead reckoning (odometry)

Dead reckoning integrates the kinematic model forward in time to estimate the robot's pose without external sensing. At each timestep `Δt`:

```
Δs    = (Δs_r + Δs_l) / 2          # distance travelled by centre
Δθ    = (Δs_r - Δs_l) / L          # heading change

x(t+1)  = x(t) + Δs · cos(θ(t) + Δθ/2)
y(t+1)  = y(t) + Δs · sin(θ(t) + Δθ/2)
θ(t+1)  = θ(t) + Δθ
```

Using the heading at the midpoint `θ + Δθ/2` (midpoint integration) reduces linearisation error compared to using the heading at the start of the step.

### Wheel encoders

Wheel displacements `Δs_l` and `Δs_r` are derived from incremental encoder ticks:

```
Δs = (ticks / ticks_per_rev) · 2π · r_wheel
```

where:
- `ticks` — encoder pulses counted since the last update
- `ticks_per_rev` — pulses per full wheel revolution (encoder resolution × gear ratio)
- `r_wheel` — wheel radius

For example, with a 4000 pulse/rev encoder and a 1:3 gear reduction the wheel completes one revolution every 4000 × 3 = 12 000 pulses, so each tick advances the wheel by `2π · r_wheel / 12000` metres.

### ROS 2 topic architecture

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

The velocity command from Nav2 (or from `mobile_robot_control`'s go-to-goal server) is discrete: it is recomputed every control cycle `Δt`. Because `Δt` is small (typically 50 ms), the piecewise-constant command approximates a continuous signal well enough for smooth motion.
