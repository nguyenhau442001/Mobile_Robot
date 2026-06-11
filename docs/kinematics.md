# Kinematics of a Differential Drive Robot

The common method for deriving the kinematics of a mobile robot is to consider the basic model of a differential drive robot, as illustrated below.

![Differential drive robot model](https://user-images.githubusercontent.com/105471622/196739775-1a44c4a5-01c2-4700-9e5d-425551a7759b.png)

| Symbol | Meaning |
|---|---|
| `v_l` | Linear velocity of the left wheel (m/s) |
| `v_r` | Linear velocity of the right wheel (m/s) |
| `v` | Linear velocity of the robot centre (m/s) |
| `ω` | Angular velocity about the vertical axis (rad/s) |
| `L` | Track width — distance between wheel contact points (m) |
| `r` | Wheel radius (m) |
| `θ` | Robot heading in the world frame (rad) |

---

## 1. Forward kinematics

### 1.1 Centre velocity from wheel speeds

We assume the robot can rotate around any point. At an instantaneous moment, the motion can be considered linear. The linear velocity of the robot's centre is:

![v = (v_r + v_l) / 2](https://user-images.githubusercontent.com/105471622/196741436-15c1fd77-d795-4a25-a4a0-d9e12acdfa1e.png)

```
v = (v_r + v_l) / 2
```

The angular velocity about the robot's vertical axis is:

![ω = (v_r - v_l) / L](https://user-images.githubusercontent.com/105471622/196741249-5b3fbe59-4a76-44c8-aabf-4c947f89cb83.png)

```
ω = (v_r - v_l) / L
```

### 1.2 Body velocity in the world frame

By projecting the velocity vector onto the flat plane, we obtain the components along the x-axis and y-axis:

![ẋ = v·cos(θ)](https://user-images.githubusercontent.com/105471622/196741322-f6b99628-0699-491f-a43b-45fbad0ea03c.png)

![ẏ = v·sin(θ)](https://user-images.githubusercontent.com/105471622/196741718-e606f7e6-6795-4b15-957f-ffa9e9ec9aac.png)

![θ̇ = ω](https://user-images.githubusercontent.com/105471622/196741850-9b6f7e35-a65c-4105-bda0-21b122a5f165.png)

```
ẋ  = v · cos(θ)
ẏ  = v · sin(θ)
θ̇  = ω
```

In matrix form:

![Matrix form](https://user-images.githubusercontent.com/105471622/196741958-2b4bbf91-5204-40cc-ba7b-d752b0125322.png)

```
[ ẋ  ]   [ cos(θ)   0 ] [ v ]
[ ẏ  ] = [ sin(θ)   0 ] [ ω ]
[ θ̇  ]   [   0      1 ]
```

---

## 2. Inverse kinematics

From equations (1) and (2), we can extract the individual wheel speeds from a desired `(v, ω)` command — what the `/cmd_vel` topic carries:

![v_r = v + ωL/2](https://user-images.githubusercontent.com/105471622/196742093-a1ed9284-a8a0-402f-8e99-ec90b3971ee9.png)

![v_l = v - ωL/2](https://user-images.githubusercontent.com/105471622/196742140-82b8dacb-5b20-4bc0-bb91-6ab5284957ba.png)

```
v_r = v + ω · L/2
v_l = v - ω · L/2
```

---

## 3. Dead reckoning (odometry)

Dead reckoning is the process of estimating the current position of a moving object from a previously known position by advancing that position using known speeds, heading, and elapsed time — without relying on external sensing.

We apply it to the kinematic model as illustrated below:

![Kinematic model — dead reckoning](https://user-images.githubusercontent.com/105471622/196742559-bfeab541-b302-47aa-b34a-11964c2cefc3.png)

*Figure 1 — Kinematic model of the robot. Source: Lee, Jung & Chung, "Accurate calibration of kinematic parameters for two-wheel differential mobile robots."*

At each timestep `Δt`, the arc lengths travelled by each wheel are:

![Δs_l, Δs_r](https://user-images.githubusercontent.com/105471622/196742675-031d5c8e-f262-4da4-b9b3-64daa166e8aa.png)

The pose update equations are:

![x, y update](https://user-images.githubusercontent.com/105471622/196742932-7de7ea88-3c02-42c7-a5be-5f41f2c4b845.png)

![θ update](https://user-images.githubusercontent.com/105471622/196742962-79cd54a2-f0b2-42af-8469-8f661ff43faf.png)

```
Δs   = (Δs_r + Δs_l) / 2     # centre arc length
Δθ   = (Δs_r - Δs_l) / L     # heading change

x(t+1) = x(t) + Δs · cos(θ(t) + Δθ/2)
y(t+1) = y(t) + Δs · sin(θ(t) + Δθ/2)
θ(t+1) = θ(t) + Δθ
```

> Using the heading at the midpoint `θ + Δθ/2` (midpoint integration) reduces linearisation error compared to using the heading at the start of the step.

---

## 4. Wheel encoders

An encoder is an electromechanical device that converts wheel rotation into digital pulses (ticks). By counting ticks, we can determine the exact angular displacement of the motor shaft and therefore compute the arc length each wheel has travelled.

To compute `Δs_l` and `Δs_r` we need the ticks counted since the last update:

![rotation per tick](https://user-images.githubusercontent.com/105471622/196743383-fd141aac-da01-450d-b8fb-a0bbd2de20ec.png)

![angular displacement](https://user-images.githubusercontent.com/105471622/196743448-f78aeeca-92b7-4f52-9693-2a5647183bc3.png)

```
Δs = (ticks / ticks_per_rev) · 2π · r
```

| Symbol | Meaning |
|---|---|
| `ticks` | Encoder pulses counted since the last update |
| `ticks_per_rev` | Pulses per full wheel revolution (encoder resolution × gear ratio) |
| `r` | Wheel radius (m) |

**Example.** With a 4 000 pulse/rev encoder and a 1:3 gear reduction, the wheel completes one revolution every 4 000 × 3 = 12 000 pulses, so each tick advances the contact point by `2π · r / 12 000` metres.

---

## 5. Control loop

![Control workflow](https://user-images.githubusercontent.com/105471622/196743528-ea7499db-eb1f-446d-af67-6e7ae8fc3f8e.png)

The `(v, ω)` command sent by the navigation stack is **discrete** — it is recomputed every `Δt` and held constant within that period. Because `Δt` is small (typically 50 ms), the piecewise-constant command approximates a continuous signal well enough for smooth motion.
