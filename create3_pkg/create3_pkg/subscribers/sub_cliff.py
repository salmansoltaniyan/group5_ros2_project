#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import IrIntensityVector

class CliffSubscriber(Node):
    def __init__(self):
        super().__init__("cliff_subscriber")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to downward-facing IR sensors (Cliff sensors)
        self.sub = self.create_subscription(IrIntensityVector, "/robot_9/cliff_intensity", self.callback, qos)
        self.get_logger().info("Subscribed to /robot_9/cliff_intensity...")

    def callback(self, msg):
        # IMPORTANT: These sensors look DOWN at the floor.
        # They are used to detect stairs/cliffs OR to do line-following (detecting tape on floor).
        for reading in msg.readings:
            sensor_name = reading.header.frame_id # e.g., 'cliff_front_left'
            sensor_value = reading.value          # Raw intensity value
            
            # Example: Just print the front left sensor
            if "front_left" in sensor_name:
                self.get_logger().info(f"Cliff {sensor_name}: {sensor_value}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CliffSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

