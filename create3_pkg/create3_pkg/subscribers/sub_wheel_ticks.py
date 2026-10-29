#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import WheelTicks

class WheelTicksSubscriber(Node):
    def __init__(self):
        super().__init__("wheel_ticks_subscriber")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(WheelTicks, "/robot_9/wheel_ticks", self.callback, qos)
        self.get_logger().info("Subscribed to /robot_9/wheel_ticks...")

    def callback(self, msg):
        # IMPORTANT: These are the raw, unadulterated encoder ticks from the motors.
        ticks_left = msg.ticks_left
        ticks_right = msg.ticks_right
        
        self.get_logger().info(f"Raw Encoder Ticks -> Left: {ticks_left} | Right: {ticks_right}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WheelTicksSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

