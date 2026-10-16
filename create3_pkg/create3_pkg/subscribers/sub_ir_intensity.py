#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import IrIntensityVector

class IrIntensitySubscriber(Node):
    def __init__(self):
        super().__init__("ir_intensity_subscriber")
        
        # We use BEST_EFFORT QoS for sensor data
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribing to the IR intensity topic
        self.sub = self.create_subscription(
            IrIntensityVector, 
            "/robot_9/ir_intensity", 
            self.callback, 
            qos
        )
        self.get_logger().info("Subscribed to /robot_9/ir_intensity. Waiting for data...")

    def callback(self, msg):
        # The IrIntensityVector contains a list of readings from all 7 IR sensors on the bumper.
        # msg.readings is an array. We can iterate through it to get the value of each sensor.
        
        for reading in msg.readings:
            sensor_name = reading.header.frame_id # The name/location (e.g., 'ir_intensity_front_center')
            sensor_value = reading.value          # Raw integer value (higher = closer to an object)
            
            # =========================================================================
            # IMPORTANT EXTRACTABLE DATA:
            # - reading.value: The proximity value (usually 0 to ~4000)
            # - reading.header.frame_id: The specific sensor (left, right, front_center, etc.)
            # =========================================================================

            # Example: We only print the value of the front center sensor to keep the terminal clean
            if "front_center" in sensor_name:
                self.get_logger().info(f"Front Center IR Value: {sensor_value}")


def main(args=None):
    rclpy.init(args=args)
    node = IrIntensitySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

