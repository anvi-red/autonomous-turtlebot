# autonomous-turtlebot

Autonomous frontier-based exploration for TurtleBot3 using ROS2 and Nav2. The robot maps an unknown environment from scratch — no teleoperation, no pre-built map — by detecting unexplored regions, clustering them, and sending navigation goals until the space is fully mapped.

**Test results (10 runs, turtlebot3_world, randomized spawn locations):**
| Metric | Result |
|---|---|
| Success rate | 100% (10/10) |
| Average exploration time | 3.6 min |
| Map coverage | ~74.4% |
| Localization error (ATE RMSE) | 12.4 mm |

> ~74.4% is the empirical ceiling for this environment — the 9 cylindrical obstacles in `turtlebot3_world` create lidar occlusion regions that remain unmapped regardless of robot path or spawn location. Coverage was consistent across all runs (74.2–74.9%), confirming this is a hard environmental limit rather than an algorithmic one.

---

## How it works

The core node (`map_reader.py`) runs the following loop on every `/map` update:

1. **Frontier detection** — scans every free cell (value `0`) in the occupancy grid and checks its 8 neighbors. If any neighbor is unknown (value `-1`), it's a frontier cell — on the boundary between mapped and unmapped space.

2. **BFS clustering** — groups frontier cells into spatially connected clusters using breadth-first search. This avoids sending hundreds of individual goals for what is effectively one continuous unexplored region.

3. **Scoring** — scores each cluster by `size / distance` (size = number of frontier cells, distance = Euclidean distance from robot to cluster centroid in grid coordinates). This prioritises large nearby frontiers over small distant ones.

4. **Goal dispatch** — converts the best cluster's centroid from grid coordinates back to world coordinates and sends it to Nav2's `NavigateToPose` action server.

5. **Termination** — exploration ends when no cluster exceeds the minimum size threshold (`MIN_FRONTIER_SIZE = 10` cells), meaning all remaining frontiers are too small to be worth navigating to.

**Stuck handling:** two mechanisms prevent infinite loops:
- **Repeat blacklist** — if the same centroid is picked 3 times in a row without progress, it's blacklisted and excluded from future scoring.
- **Recovery limit** — if Nav2 triggers more than 5 recovery behaviors (spin, backup, wait) on a single goal, the goal is cancelled and that frontier is blacklisted.

---

## Localization error measurement

SLAM accuracy was measured by comparing the SLAM-estimated pose against Gazebo's ground-truth pose over a full exploration run (~4 min, ~2257 samples at 10 Hz).

**How the two pose sources work:**

- **Ground truth** — Gazebo publishes all entity poses on the gz-transport topic `/world/default/pose/info` as a `Pose_V` array. This is bridged to ROS2 via `ros_gz_bridge` as a `TFMessage` on `/gt_tf`. The robot (`waffle_pi`) is consistently at index 3 in this array (determined by entity creation order in the world file, not spawn position). A zero-glitch filter discards frames where the bridge briefly returns `(0, 0)` for that slot.

- **SLAM estimate** — slam_toolbox publishes the `map → base_footprint` transform via TF. This is looked up at 10 Hz using a `tf2_ros.Buffer`.

Both sources are stamped using the simulation clock (`use_sim_time=True`) so samples are time-aligned.

**Why alignment is needed before computing error:**

The SLAM `map` frame is anchored at the robot's spawn position, while ground truth is in the Gazebo `world` frame. The two trajectories are offset by the initial spawn transform — computing raw error without alignment would give a meaninglessly large number. A rigid SE(2) alignment (Umeyama method via SVD) removes this offset before computing ATE.

**Results:**
| Metric | Value |
|---|---|
| ATE RMSE | **12.4 mm** |
| ATE mean | 11.3 mm |
| ATE max | 37.0 mm |
| Final drift | 9.6 mm |

The error is low because Gazebo simulation has no sensor noise — lidar returns are perfect, wheel odometry has no slip. On a real robot, 5–20 cm RMSE is typical for a similar environment. This result is a simulation baseline, not a hardware claim.

The peak error (~37 mm at ~50s) occurred during tight turns around the cylindrical obstacles, where lidar geometry changes rapidly. Error flattened to ~10 mm once exploration completed and the robot stopped moving.

---

## Stack

| Component | Role |
|---|---|
| TurtleBot3 waffle_pi | Robot platform (simulated) |
| Gazebo Harmonic | Physics simulation |
| slam_toolbox | Real-time 2D SLAM (occupancy grid from lidar) |
| Nav2 | Path planning, local obstacle avoidance, recovery behaviors |
| `map_reader.py` | Custom frontier exploration node (this repo) |
| `gt_logger.py` | Ground-truth vs SLAM pose logger (localization error measurement) |
| `ate_analysis.py` | ATE computation and trajectory plots (run offline after data collection) |

---

## Environment setup

This project runs inside a Docker container with browser-based VNC (no WSL2 — OgreNext rendering issues on Windows).

**Prerequisites:**
- Docker Desktop
- The custom ROS2 image: `ros2-turtlebot-custom:jazzy`

```bash
docker start ros2_workspace
# open http://localhost:6080 in your browser
```

**Inside the container:**
```bash
export TURTLEBOT3_MODEL=waffle_pi   # already in ~/.bashrc
source ~/ros2_ws/install/setup.bash  # already in ~/.bashrc
```

**Dependencies (baked into the Docker image):**
```
ros-jazzy-turtlebot3
ros-jazzy-turtlebot3-gazebo
ros-jazzy-ros-gz
ros-jazzy-slam-toolbox
ros-jazzy-navigation2
ros-jazzy-nav2-bringup
```

---

## Running manually

Launch each component in a separate terminal, in order. Wait for each to finish initializing before starting the next.

```bash
# Terminal 1 — Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=0.750 y_pose:=0.3535

# Terminal 2 — SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3 — RViz (optional, for visualization)
ros2 launch nav2_bringup rviz_launch.py

# Terminal 4 — Nav2 (uses patched params — see nav2_params.yaml)
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=$HOME/ros2_ws/nav2_params.yaml

# Terminal 5 — Frontier exploration node
cd ~/ros2_ws/src/my_explorer && python3 map_reader.py
```

To also collect localization data during the run, add before Terminal 5:

```bash
# Terminal 5a — Ground-truth bridge
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  --ros-args -r /world/default/pose/info:=/gt_tf

# Terminal 5b — GT logger (Ctrl+C after exploration completes)
cd ~/ros2_ws/src/my_explorer && python3 gt_logger.py --ros-args -p use_sim_time:=true
```

Then copy the CSV out and run the analysis:

```bash
docker cp ros2_workspace:/home/ubuntu/ros2_ws/src/my_explorer/localization_log.csv .
python3 ate_analysis.py
```

---

## Automated test campaign

`run_tests.py` runs N full exploration trials unattended, launching and tearing down the complete stack between runs, and writes metrics to `results.csv`.

```bash
cd ~/ros2_ws && python3 run_tests.py
```

**Logged metrics per run:** success, time (min), map coverage (%), goals sent, stuck frontiers blacklisted, notes.

---

## Key debugging stories

### cmd_vel type mismatch

After Nav2 was confirmed running and publishing `/cmd_vel` at ~20Hz, the robot still didn't move under Nav2 control. Teleop worked fine, which initially suggested a map issue — it wasn't.

**Diagnosis:**
```bash
$ ros2 topic echo /cmd_vel
# ERROR: Cannot echo topic '/cmd_vel' — contains more than one type:
# [geometry_msgs/msg/Twist, geometry_msgs/msg/TwistStamped]

$ ros2 topic info /cmd_vel --verbose
# collision_monitor  →  publishes  geometry_msgs/msg/Twist
# ros_gz_bridge      →  subscribes geometry_msgs/msg/TwistStamped
```

`ros_gz_bridge` (the ROS2→Gazebo bridge) subscribes to `/cmd_vel` expecting `TwistStamped`. Nav2's `collision_monitor` (the final stage of the velocity pipeline) publishes plain `Twist` by default. In ROS2, publishers and subscribers only connect on exact type match — so Gazebo never received any velocity commands.

**Fix:** Nav2 Jazzy exposes `enable_stamped_cmd_vel` on `controller_server`, `velocity_smoother`, and `collision_monitor`. Copied the default `nav2_params.yaml` into the workspace and set `enable_stamped_cmd_vel: True` on all three nodes, then relaunched Nav2 with `params_file:=~/ros2_ws/nav2_params.yaml`.

### SLAM startup edge case (fixed)

Early testing revealed that when the robot spawned in an open area, the entire unexplored map formed a single large frontier cluster centered near the robot. Nav2 completed the goal instantly (robot already at the centroid), the repeat blacklist fired, and with the only viable cluster blacklisted, exploration terminated at ~12% coverage.

The fix: a 10-second startup spin in place before any frontier goals are dispatched. The lidar sweeps the full 360° environment, seeding the map asymmetrically so multiple distinct clusters form before exploration begins. All 10 runs with randomized spawn locations completed successfully after this fix.

---

## Known limitations

- **Coverage ceiling ~74.4%** — the 9 cylindrical obstacles in `turtlebot3_world` create permanent lidar occlusion regions. Consistent across all 10 runs with varied spawn locations, confirming this is an environmental limit not an algorithmic one.
- **Single environment tested** — all runs used `turtlebot3_world`. Robustness across different environments is untested.
- **Localization error is simulation-only** — the 12.4 mm ATE result reflects Gazebo's ideal sensor conditions. Real hardware would show higher error due to lidar noise, wheel slip, and IMU drift.

---

## Repository structure

```
autonomous-turtlebot/
├── map_reader.py         # Custom frontier exploration node
├── nav2_params.yaml      # Nav2 config with enable_stamped_cmd_vel fix
├── run_tests.py          # Automated test campaign script
├── results.csv           # Raw metrics from the most recent test campaign
├── gt_logger.py          # Logs ground-truth vs SLAM pose to CSV (10 Hz)
├── ate_analysis.py       # Computes ATE + generates trajectory plots
├── localization_log.csv  # Pose log from the most recent measurement run
├── trajectory_overlay.png  # Ground truth vs SLAM path (top-down)
└── error_over_time.png     # Position error (m) vs time
```
