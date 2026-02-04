#!/usr/bin/env python3
"""
启动时自动把当前 /odom 位姿发布到 /initialpose，供 AMCL 使用，无需在 RViz 里手动点 2D Pose Estimate。
只发一次，发完等几秒再退出，便于 launch 里用 TimerAction 启动。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


def quaternion_from_yaw(yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


class PublishInitialPose(Node):
    def __init__(self, delay_sec=2.0, once=True):
        super().__init__("publish_initial_pose")
        self.declare_parameter("delay_sec", delay_sec)
        self.declare_parameter("once", once)
        self.delay_sec = self.get_parameter("delay_sec").value
        self.once = self.get_parameter("once").value
        self.last_odom = None
        self.first_odom_time = None  # clock time when we got first odom
        self.published = False
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )
        self.timer = self.create_timer(0.2, self.timer_cb)
        self.get_logger().info(
            "Will publish initial pose from /odom after %.1f s (once=%s)"
            % (self.delay_sec, self.once)
        )

    def odom_cb(self, msg):
        self.last_odom = msg
        if self.first_odom_time is None:
            self.first_odom_time = self.get_clock().now()

    def timer_cb(self):
        if self.published and not self.once:
            return
        if self.last_odom is None or self.first_odom_time is None:
            return
        now = self.get_clock().now()
        elapsed = (now - self.first_odom_time).nanoseconds / 1e9
        if elapsed < self.delay_sec:
            return
        p = self.last_odom.pose.pose.position
        q = self.last_odom.pose.pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = p.x
        msg.pose.pose.position.y = p.y
        msg.pose.pose.position.z = p.z
        q2 = quaternion_from_yaw(yaw)
        msg.pose.pose.orientation.x = q2[0]
        msg.pose.pose.orientation.y = q2[1]
        msg.pose.pose.orientation.z = q2[2]
        msg.pose.pose.orientation.w = q2[3]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891909122467
        self.initial_pose_pub.publish(msg)
        self.published = True
        self.get_logger().info(
            "Published initial pose (x=%.2f, y=%.2f, yaw=%.2f) from /odom."
            % (p.x, p.y, yaw)
        )
        if self.once:
            self.timer.cancel()
            self.get_logger().info("Initial pose sent; exiting in 1.5 s.")
            self.create_timer(1.5, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)
    node = PublishInitialPose()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
