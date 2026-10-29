#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import StopStatus

class StopSubscriber(Node):
    def __init__(self):
        super().__init__("stop_subscriber")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(StopStatus, "/robot_9/stop_status", self.callback, qos)
        self.get_logger().info("Subscribed to /robot_9/stop_status...")

    def callback(self, msg):
        # IMPORTANT: msg.is_stopped is True if E-Stop is active or robot locked
        if msg.is_stopped:
            self.get_logger().warn("EMERGENCY STOP is ACTIVE! Motors are locked.")
        else:
            self.get_logger().info("Motors enabled. E-Stop is OFF.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StopSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

