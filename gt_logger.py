#!/usr/bin/env python3
"""
gt_logger.py — logs ground-truth (Gazebo) vs SLAM-estimated pose to CSV
for localization error (ATE) analysis.
"""
import csv
import math
import os

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class GTLogger(Node):
    def __init__(self):
        super().__init__('gt_logger')

        # Latest ground-truth sample (robot = index 3 in /gt_tf array)
        self.latest_gt = None  # (x, y, yaw)

        self.gt_sub = self.create_subscription(
            TFMessage, '/gt_tf', self.gt_callback, 10)

        # TF buffer/listener for the SLAM estimate (map -> base_footprint)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # CSV output. Overwrite ('w') each run rather than append — appending
        # across multiple sim restarts silently mixes runs into one file
        # (bit us once already: sim-time resets between launches made old
        # and new runs indistinguishable without manually finding the
        # boundary). One file = one run.
        self.csv_path = os.path.join(os.getcwd(), 'localization_log.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ['t', 'gt_x', 'gt_y', 'gt_yaw', 'slam_x', 'slam_y', 'slam_yaw'])

        # 10 Hz sampling timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'gt_logger started, logging to {self.csv_path}')

    def gt_callback(self, msg: TFMessage):
        if len(msg.transforms) <= 3:
            return
        t = msg.transforms[3].transform.translation
        q = msg.transforms[3].transform.rotation

        # Known bridge glitch: on some heartbeat messages the "stable index
        # 3 = robot" assumption breaks and this slot comes back as an exact
        # zero pose. The robot never legitimately sits at exact world
        # origin, so treat this as a dropped sample and keep the last good
        # value instead of overwriting it with garbage.
        if t.x == 0.0 and t.y == 0.0 and self.latest_gt is not None:
            return

        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_gt = (t.x, t.y, yaw)

    def timer_callback(self):
        if self.latest_gt is None:
            return
        slam_pose = self.get_slam_pose()
        if slam_pose is None:
            return

        t = self.get_clock().now().nanoseconds * 1e-9
        gt_x, gt_y, gt_yaw = self.latest_gt
        slam_x, slam_y, slam_yaw = slam_pose

        self.csv_writer.writerow([t, gt_x, gt_y, gt_yaw, slam_x, slam_y, slam_yaw])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

    def get_slam_pose(self):
        """Returns (x, y, yaw) for map -> base_footprint, or None if unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)


def main():
    rclpy.init()
    node = GTLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
