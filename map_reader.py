import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import numpy as np
from collections import deque

MIN_FRONTIER_SIZE = 10  # ignore clusters smaller than this (empirically noise in turtlebot3_world at 0.05m/cell)
STUCK_REPEAT_LIMIT = 3  # how many times in a row we'll re-pick the same goal before giving up on it
BLACKLIST_RADIUS = 3    # grid cells — how close a new centroid must be to a blacklisted one to be excluded
MAX_RECOVERIES = 5      # cancel and blacklist a goal if Nav2 triggers more than this many recoveries on it


class MapReader(Node):
    """Class called MapReader that inherits from Node. It subscribes to the '/map' topic to receive OccupancyGrid messages and processes them in the map_callback function."""
    def __init__(self):
        super().__init__('map_reader')
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # action client to talk to Nav2's NavigateToPose server
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_in_progress = False  # guard so we don't send a new goal while one is active

        self.last_centroid = None         # (row, col) of the last goal we picked
        self.repeat_count = 0             # how many times in a row we've picked the same centroid
        self.blacklist = []               # list of (row, col) centroids we've given up on
        self.current_goal_centroid = None # centroid of the goal currently being navigated to

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        # skip this whole cycle if we're still navigating to a previous goal
        if self.goal_in_progress:
            return

        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        height, width = grid.shape

        frontiers = []
        for row in range(height):
            for col in range(width):
                if grid[row, col] != 0:
                    continue
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        r, c = row + dr, col + dc
                        if 0 <= r < height and 0 <= c < width:
                            if grid[r, c] == -1:
                                frontiers.append((row, col))
                                break
                    else:
                        continue
                    break

        if not frontiers:
            self.get_logger().info("No frontiers left — exploration complete!")
            return

        clusters = self.cluster_frontiers(frontiers)

        # ignore tiny clusters (likely sensor noise / unreachable wall slivers)
        clusters = [c for c in clusters if len(c) >= MIN_FRONTIER_SIZE]

        if not clusters:
            self.get_logger().info("No frontiers left — exploration complete!")
            return

        # ignore clusters whose centroid is too close to one we've given up on
        def centroid_of(cluster):
            rows = [c[0] for c in cluster]
            cols = [c[1] for c in cluster]
            return sum(rows) / len(rows), sum(cols) / len(cols)

        def near_blacklist(centroid):
            cr, cc = centroid
            for br, bc in self.blacklist:
                if ((cr - br) ** 2 + (cc - bc) ** 2) ** 0.5 <= BLACKLIST_RADIUS:
                    return True
            return False

        clusters = [c for c in clusters if not near_blacklist(centroid_of(c))]

        if not clusters:
            self.get_logger().info("No frontiers left — exploration complete!")
            return

        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        robot_col = (self.robot_x - origin_x) / resolution
        robot_row = (self.robot_y - origin_y) / resolution

        best_cluster = None
        best_score = -1
        for cluster in clusters:
            rows = [c[0] for c in cluster]
            cols = [c[1] for c in cluster]
            centroid_row = sum(rows) / len(rows)
            centroid_col = sum(cols) / len(cols)

            distance = ((robot_row - centroid_row) ** 2 + (robot_col - centroid_col) ** 2) ** 0.5
            size = len(cluster)
            score = size / distance if distance > 0 else size

            if score > best_score:
                best_score = score
                best_cluster = (centroid_row, centroid_col, size, distance)

        self.get_logger().info(
            f"Best cluster: centroid=({best_cluster[0]:.1f},{best_cluster[1]:.1f}) "
            f"size={best_cluster[2]} dist={best_cluster[3]:.1f} score={best_score:.2f}"
        )

        # convert chosen centroid (grid row/col) back to world coordinates (meters)
        centroid_row, centroid_col = best_cluster[0], best_cluster[1]

        # check if we're picking essentially the same goal as last time (within 1 cell)
        this_centroid = (centroid_row, centroid_col)
        if self.last_centroid is not None:
            dist_from_last = ((centroid_row - self.last_centroid[0]) ** 2 +
                               (centroid_col - self.last_centroid[1]) ** 2) ** 0.5
        else:
            dist_from_last = None

        if dist_from_last is not None and dist_from_last <= 1.0:
            self.repeat_count += 1
        else:
            self.repeat_count = 0

        self.last_centroid = this_centroid

        if self.repeat_count >= STUCK_REPEAT_LIMIT:
            self.get_logger().info(
                f"Giving up on stuck frontier at ({centroid_row:.1f},{centroid_col:.1f}) "
                f"after {self.repeat_count} repeats — blacklisting it."
            )
            self.blacklist.append(this_centroid)
            self.repeat_count = 0
            self.last_centroid = None
            return  # skip this cycle; next map update will pick a different frontier

        goal_x = origin_x + centroid_col * resolution
        goal_y = origin_y + centroid_row * resolution

        self.send_nav_goal(goal_x, goal_y)

    def send_nav_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # no specific orientation, just face forward

        self.goal_in_progress = True
        self.current_goal_centroid = self.last_centroid  # track for recovery-based blacklisting
        self.get_logger().info(f"Sending goal: x={x:.2f}, y={y:.2f}")

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.goal_feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_feedback_callback(self, feedback_msg):
        recoveries = feedback_msg.feedback.number_of_recoveries
        if recoveries > MAX_RECOVERIES:
            self.get_logger().warn(
                f"Too many recoveries ({recoveries}) on current goal — cancelling and blacklisting."
            )
            # blacklist the current goal's centroid so we don't return to it
            if self.current_goal_centroid is not None:
                self.blacklist.append(self.current_goal_centroid)
                self.current_goal_centroid = None
            # cancel via the stored goal handle
            if hasattr(self, 'goal_handle') and self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.goal_in_progress = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.goal_in_progress = False
            return

        self.goal_handle = goal_handle  # store so we can cancel if needed
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Navigation finished with status: {result.status}")
        self.current_goal_centroid = None
        self.goal_in_progress = False  # ready for map_callback to pick a new goal next time

    def cluster_frontiers(self, frontiers):  # the clustering algorithm
        frontier_set = set(frontiers)  # convert the list of frontier cells to a set for O(1) lookups
        visited = set()  # set to keep track of visited frontier cells
        clusters = []  # list to hold the clusters of frontier cells

        for cell in frontiers:
            if cell in visited:
                continue  # skip this cell if it has already been visited

            cluster = []  # list to hold the current cluster of frontier cells
            queue = deque([cell])  # queue for breadth-first search, initialized with the current cell
            visited.add(cell)  # mark the current cell as visited

            while queue:
                current = queue.popleft()
                cluster.append(current)
                row, col = current  # get the row and column indices of the current cell (unpacks the tuple)

                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        neighbor = (row + dr, col + dc)
                        if neighbor in frontier_set and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)

            clusters.append(cluster)

        return clusters


def main(args=None):
    rclpy.init(args=args)  # start the ROS 2 Python communication system
    node = MapReader()    # create an instance of the MapReader class
    rclpy.spin(node)      # keep the node running and processing callbacks until it is shut down
    rclpy.shutdown()      # cleanly shut down the ROS 2 communication system when done


if __name__ == '__main__':
    main()
