# Simulation

## Real-Time Factor (RTF)

The **Real-Time Factor** is the ratio between simulated time and wall-clock time. RTF = 1.0 means the simulation advances at real speed; RTF < 1.0 means the physics step is too expensive for the host to keep up, and RTF > 1.0 means it is running faster than real time. Tracking RTF is the standard way to compare the cost of different physics engines (ODE, TPE, Bullet, DART) or to detect when world complexity has outgrown the host.

The world stats are published on `/world/<world_name>/stats` (`gz.msgs.WorldStatistics`).

### Quick check — `gz topic` one-liner

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

### Reproducible benchmark — `physic_engines_rtf_measure.py`

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

---

## Genesis — batched fleet simulation

The `mobile_robot_genesis` package contains standalone Genesis scripts that load the same URDF used in Gazebo (via [mobile_robot_genesis/scripts/xacro_loader.py](../mobile_robot_genesis/scripts/xacro_loader.py)) and step large fleets in a single batched physics call — useful for fleet-scale RL or task-assignment experiments where launching 100 Gazebo robots is impractical.

```bash
# Sanity check — one robot, plain URDF → Genesis pipeline
python3 mobile_robot_genesis/scripts/spawn_mobile_robot.py

# 100 robots in parallel environments (one batched step covers all of them)
python3 mobile_robot_genesis/scripts/spawn_100_mobile_robots.py

# Fleet manager: 100 robots picking tasks off a shared queue,
# nearest-task assignment, kinematic motion, completion metrics
python3 mobile_robot_genesis/scripts/genesis_mobile_robot_fleet.py
```

Each script runs until the viewer window is closed. The xacro loader resolves `$(find <pkg>)` substitutions without needing the ROS environment sourced, so these scripts work from a plain Python venv as long as `genesis-world` and `xacro` are installed (both pulled in by `pip install -e .` against [pyproject.toml](../pyproject.toml)).
