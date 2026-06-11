# Web Dashboard

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
