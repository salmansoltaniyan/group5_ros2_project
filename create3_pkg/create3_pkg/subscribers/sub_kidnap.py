#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import KidnapStatus

class KidnapSubscriber(Node):
    def __init__(self):
        super().__init__("kidnap_subscriber")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(KidnapStatus, "/robot_9/kidnap_status", self.callback, qos)
        self.get_logger().info("Subscribed to /robot_9/kidnap_status...")

    def callback(self, msg):
        # IMPORTANT: msg.is_kidnapped is True if someone picks up the robot
        if msg.is_kidnapped:
            self.get_logger().warn("HELP! I have been kidnapped/lifted off the ground!")
        else:
            self.get_logger().info("Safe on the ground.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(KidnapSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

