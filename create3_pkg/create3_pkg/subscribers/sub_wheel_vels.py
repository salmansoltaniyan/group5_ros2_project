#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import WheelVels

class WheelVelsSubscriber(Node):
    def __init__(self):
        super().__init__("wheel_vels_subscriber")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(WheelVels, "/robot_9/wheel_vels", self.callback, qos)
        self.get_logger().info("Subscribed to /robot_9/wheel_vels...")

    def callback(self, msg):
        # IMPORTANT: This gives the actual turning speed of the wheels in radians per second.
        vel_left = msg.velocity_left
        vel_right = msg.velocity_right
        
        self.get_logger().info(f"Wheel Speed (rad/s) -> Left: {vel_left:.2f} | Right: {vel_right:.2f}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WheelVelsSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

