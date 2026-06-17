# autonomous-turtlebot

Autonomous frontier-based exploration for TurtleBot3 using ROS2 and Nav2. The robot maps an unknown environment from scratch — no teleoperation, no pre-built map — by detecting unexplored regions, clustering them, and sending navigation goals until the space is fully mapped.

**Test results (10 runs, turtlebot3_world):**
| Metric | Result |
|---|---|
| Success rate | 100% (10/10) |
| Average exploration time | 4.3 min |
| Map coverage | ~74.5% |
| Localization error | Deferred (see Known Limitations) |

> ~74.5% is the empirical ceiling for this environment — lidar occlusion behind the 9 cylindrical obstacles creates permanently unobservable regions regardless of robot path.

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

## Stack

| Component | Role |
|---|---|
| TurtleBot3 waffle_pi | Robot platform (simulated) |
| Gazebo Harmonic | Physics simulation |
| slam_toolbox | Real-time 2D SLAM (occupancy grid from lidar) |
| Nav2 | Path planning, local obstacle avoidance, recovery behaviors |
| `map_reader.py` | Custom frontier exploration node (this repo) |

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
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=0.750 y_pose:=0.35350

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

---

## Automated test campaign

`run_tests.py` runs N full exploration trials unattended, launching and tearing down the complete stack between runs, and writes metrics to `results.csv`.

```bash
cd ~/ros2_ws && python3 run_tests.py
```

**Logged metrics per run:** success, time (min), map coverage (%), goals sent, stuck frontiers blacklisted, notes.

---

## Key debugging story — cmd_vel type mismatch

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

---

## Known limitations

- **Localization error not measured** — comparing SLAM-estimated pose against Gazebo ground truth requires bridging an additional Gazebo topic or using the `gz` CLI during live runs. Deferred for a future iteration.
- **Coverage ceiling ~74.5%** — the 9 cylindrical obstacles in `turtlebot3_world` create permanent lidar occlusion shadows. The theoretical maximum explorable coverage is higher than 74.5% but has not been measured via exhaustive manual teleoperation.
- **Single environment tested** — all 10 runs used `turtlebot3_world` with a fixed spawn position. Robustness across different environments and spawn locations is untested.

---

## Repository structure

```
autonomous-turtlebot/
├── map_reader.py       # Custom frontier exploration node
├── nav2_params.yaml    # Nav2 config with enable_stamped_cmd_vel fix
└── run_tests.py        # Automated test campaign script
```
