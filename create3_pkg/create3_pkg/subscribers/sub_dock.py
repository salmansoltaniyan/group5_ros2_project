#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import DockStatus

class DockSubscriber(Node):
    def __init__(self):
        super().__init__("dock_subscriber")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(DockStatus, "/robot_9/dock_status", self.callback, qos)
        self.get_logger().info("Subscribed to /robot_9/dock_status...")

    def callback(self, msg):
        # IMPORTANT: msg.is_docked tells you if the robot has successfully connected to the charging base
        if msg.is_docked:
            self.get_logger().info("Robot is ON the charging dock.")
        else:
            self.get_logger().info("Robot is OFF the dock.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DockSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

