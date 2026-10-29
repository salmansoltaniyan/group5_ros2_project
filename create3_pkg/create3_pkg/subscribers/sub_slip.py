#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import SlipStatus

class SlipSubscriber(Node):
    def __init__(self):
        super().__init__("slip_subscriber")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(SlipStatus, "/robot_9/slip_status", self.callback, qos)
        self.get_logger().info("Subscribed to /robot_9/slip_status...")

    def callback(self, msg):
        # IMPORTANT: msg.is_slipping is True if wheels are spinning but robot isn't moving
        if msg.is_slipping:
            self.get_logger().warn("WHEELS ARE SLIPPING!")
        else:
            self.get_logger().info("Grip is good.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SlipSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

