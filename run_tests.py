#!/usr/bin/env python3
"""
run_tests.py — automated test campaign for autonomous-turtlebot
Launches the full ROS2 + Gazebo stack for each run, executes the
frontier-exploration node, and logs metrics to results.csv.

Usage (from inside the Docker container):
    cd ~/ros2_ws && python3 run_tests.py
"""

import subprocess
import time
import csv
import os
import math
import random
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import OccupancyGrid, Odometry

# ── Config ─────────────────────────────────────────────────────────────────────
NUM_RUNS        = 10
RUN_TIMEOUT_SEC = 1200  # 20 minutes max per run before marking as failed
RESULTS_FILE    = os.path.expanduser('~/ros2_ws/results.csv')

# Seconds to wait after launching each component before starting the next.
# Generous to be safe; tune down once you know your machine's startup times.
GAZEBO_WAIT  = 12
SLAM_WAIT    = 6
NAV2_WAIT    = 12

# Cylinder centres from turtlebot3_world/model.sdf (3x3 grid, 1.1m spacing)
CYLINDERS = [
    (-1.1, -1.1), (-1.1, 0.0), (-1.1, 1.1),
    ( 0.0, -1.1), ( 0.0, 0.0), ( 0.0, 1.1),
    ( 1.1, -1.1), ( 1.1, 0.0), ( 1.1, 1.1),
]
CYLINDER_CLEARANCE = 0.40   # cylinder radius (0.15) + robot radius (~0.2) + margin
SPAWN_BOUNDS       = 1.6    # stay within ±1.6 m to avoid walls


def random_spawn():
    """Return a random (x, y) spawn that clears all cylinders and walls."""
    while True:
        x = random.uniform(-SPAWN_BOUNDS, SPAWN_BOUNDS)
        y = random.uniform(-SPAWN_BOUNDS, SPAWN_BOUNDS)
        if all(math.hypot(x - cx, y - cy) >= CYLINDER_CLEARANCE
               for cx, cy in CYLINDERS):
            return round(x, 3), round(y, 3)

# ── Helpers ────────────────────────────────────────────────────────────────────

def ros_env():
    """Return an environment dict with the ROS2 workspace sourced."""
    env = os.environ.copy()
    env['TURTLEBOT3_MODEL'] = 'waffle_pi'
    # Prepend the workspace's install/setup.bash paths
    ws = os.path.expanduser('~/ros2_ws')
    install_setup = f'{ws}/install/setup.bash'
    ros_setup = '/opt/ros/jazzy/setup.bash'
    # Run a bash subshell to get the sourced environment
    result = subprocess.run(
        ['bash', '-c', f'source {ros_setup} && source {install_setup} && env'],
        capture_output=True, text=True
    )
    for line in result.stdout.splitlines():
        if '=' in line:
            k, _, v = line.partition('=')
            env[k] = v
    return env


def launch(cmd_list, name, env):
    """Launch a subprocess, suppressing its output."""
    print(f"  [run_tests] Launching {name}...")
    return subprocess.Popen(
        cmd_list,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=env
    )


def kill_all(procs):
    """Terminate a list of subprocesses cleanly, then force-kill any stragglers."""
    for proc in procs:
        if proc and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=8)
            except Exception:
                proc.kill()
    # force-kill any lingering ROS2/Gazebo processes by name
    for name in ['gz', 'gzserver', 'gzclient', 'ruby', 'rviz2',
                 'robot_state_publisher', 'slam_toolbox', 'nav2', 'map_reader']:
        subprocess.run(['pkill', '-f', name],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def get_gazebo_ground_truth():
    """
    Query Gazebo Harmonic for the robot's actual world position via the gz CLI.
    Returns (x, y) in meters, or (None, None) on failure.
    """
    try:
        result = subprocess.run(
            ['gz', 'model', '-m', 'turtlebot3_waffle_pi', '-p'],
            capture_output=True, text=True, timeout=5
        )
        # Output looks like:
        #   Pose:
        #     Position [ 0.75  0.35  0.00 ]
        #     ...
        for line in result.stdout.splitlines():
            line = line.strip()
            if line.startswith('Position'):
                # extract the three numbers inside [ ... ]
                nums = line.replace('Position', '').replace('[', '').replace(']', '').split()
                if len(nums) >= 2:
                    return float(nums[0]), float(nums[1])
    except Exception:
        pass
    return None, None


# ── Metrics collector (ROS2 node) ──────────────────────────────────────────────

class MetricsCollector(Node):
    """Subscribes to /map and /odom to snapshot metrics at end of each run."""

    def __init__(self):
        super().__init__('metrics_collector')
        self.latest_map = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.create_subscription(OccupancyGrid, '/map',  self._map_cb,  10)
        self.create_subscription(Odometry,      '/odom', self._odom_cb, 10)

    def _map_cb(self, msg):
        self.latest_map = msg

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_ready(self):
        return self.latest_map is not None

    def coverage_pct(self):
        """Percentage of map cells that are known (free or occupied)."""
        if self.latest_map is None:
            return 0.0
        data = self.latest_map.data
        total = len(data)
        known = sum(1 for c in data if c != -1)
        return round(known / total * 100, 1)

    def odom_position(self):
        return self.robot_x, self.robot_y


# ── Per-run logic ──────────────────────────────────────────────────────────────

def run_exploration(env, run_timeout):
    """
    Launch map_reader.py, monitor its output, and return raw metrics.
    Returns: (success, goals_sent, stuck_count, elapsed_sec)
    """
    proc = subprocess.Popen(
        ['python3', os.path.expanduser('~/ros2_ws/src/my_explorer/map_reader.py')],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        cwd=os.path.expanduser('~/ros2_ws/src/my_explorer')
    )

    goals_sent  = 0
    stuck_count = 0
    success     = False
    start       = time.time()

    def reader():
        nonlocal goals_sent, stuck_count, success
        for line in proc.stdout:
            print(f"    {line.strip()}")
            if 'Sending goal' in line:
                goals_sent += 1
            if 'Giving up on stuck frontier' in line:
                stuck_count += 1
            if 'exploration complete' in line.lower():
                success = True
                proc.terminate()
                return

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    t.join(timeout=run_timeout)

    elapsed = time.time() - start

    if proc.poll() is None:
        proc.terminate()

    return success, goals_sent, stuck_count, elapsed


def wait_for_nav2(env, timeout=60):
    """Poll until Nav2's navigate_to_pose action server is available."""
    print("  [run_tests] Waiting for Nav2 action server...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        result = subprocess.run(
            ['ros2', 'action', 'list'],
            capture_output=True, text=True, env=env, timeout=5
        )
        if '/navigate_to_pose' in result.stdout:
            print("  [run_tests] Nav2 ready.")
            return True
        time.sleep(2)
    return False


def wait_for_map(collector, executor, timeout=60):
    """Spin the collector node until /map is publishing or timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.5)
        if collector.map_ready():
            return True
    return False


def spin_briefly(executor, seconds=2.0):
    """Spin the collector node for `seconds` to flush latest topic data."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    collector = MetricsCollector()
    executor  = SingleThreadedExecutor()
    executor.add_node(collector)

    env = ros_env()

    # CSV setup
    fieldnames = [
        'run', 'spawn_x', 'spawn_y', 'success', 'time_min', 'coverage_pct',
        'goals_sent', 'stuck_count', 'localization_error_m', 'notes'
    ]
    with open(RESULTS_FILE, 'w', newline='') as f:
        csv.DictWriter(f, fieldnames=fieldnames).writeheader()

    print(f"\n[run_tests] Starting {NUM_RUNS}-run test campaign.")
    print(f"[run_tests] Results will be saved to {RESULTS_FILE}\n")

    for run_num in range(1, NUM_RUNS + 1):
        print(f"\n{'='*55}")
        print(f"  RUN {run_num} / {NUM_RUNS}")
        print(f"{'='*55}")

        # Pick a random collision-free spawn for this run
        spawn_x, spawn_y = random_spawn()
        print(f"  [run_tests] Spawn: ({spawn_x}, {spawn_y})")

        # Launch stack
        gz_proc   = launch(
            ['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py',
             f'x_pose:={spawn_x}', f'y_pose:={spawn_y}'],
            'Gazebo', env
        )
        time.sleep(GAZEBO_WAIT)

        slam_proc = launch(
            ['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'use_sim_time:=true'],
            'slam_toolbox', env
        )
        time.sleep(SLAM_WAIT)

        rviz_proc = launch(
            ['ros2', 'launch', 'nav2_bringup', 'rviz_launch.py'],
            'RViz', env
        )

        nav2_proc = launch(
            ['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
             'use_sim_time:=true',
             f'params_file:={os.path.expanduser("~/ros2_ws/nav2_params.yaml")}'],
            'Nav2', env
        )
        wait_for_nav2(env, timeout=90)

        # Reset collector state from previous run
        collector.latest_map = None

        # Wait for /map to appear
        print("  [run_tests] Waiting for /map...")
        map_ok = wait_for_map(collector, executor, timeout=60)
        if not map_ok:
            print("  [run_tests] WARNING: /map never appeared — marking run as failed.")
            kill_all([gz_proc, slam_proc, nav2_proc, rviz_proc])
            row = dict(run=run_num, success=False, time_min=0, coverage_pct=0,
                       goals_sent=0, stuck_count=0, localization_error_m='N/A',
                       spawn_x=spawn_x, spawn_y=spawn_y,
                       notes='map never appeared')
            with open(RESULTS_FILE, 'a', newline='') as f:
                csv.DictWriter(f, fieldnames=fieldnames).writerow(row)
            time.sleep(5)
            continue

        # Run exploration
        print("  [run_tests] Starting exploration...")
        success, goals_sent, stuck_count, elapsed = run_exploration(env, RUN_TIMEOUT_SEC)

        # Snapshot final metrics
        spin_briefly(executor, seconds=2.0)
        coverage          = collector.coverage_pct()
        odom_x, odom_y    = collector.odom_position()
        gz_x,   gz_y      = get_gazebo_ground_truth()

        if gz_x is not None:
            loc_error = round(math.sqrt((odom_x - gz_x)**2 + (odom_y - gz_y)**2), 3)
        else:
            loc_error = 'N/A'

        row = {
            'run':                  run_num,
            'spawn_x':              spawn_x,
            'spawn_y':              spawn_y,
            'success':              success,
            'time_min':             round(elapsed / 60, 2),
            'coverage_pct':         coverage,
            'goals_sent':           goals_sent,
            'stuck_count':          stuck_count,
            'localization_error_m': loc_error,
            'notes':                '' if success else 'timed out or crashed'
        }
        print(f"  [run_tests] Run {run_num} complete: {row}")

        with open(RESULTS_FILE, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=fieldnames).writerow(row)

        # Tear down stack
        print("  [run_tests] Tearing down stack...")
        kill_all([gz_proc, slam_proc, nav2_proc, rviz_proc])
        time.sleep(5)   # give processes time to fully exit

    print(f"\n[run_tests] All {NUM_RUNS} runs complete.")
    print(f"[run_tests] Results saved to {RESULTS_FILE}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
