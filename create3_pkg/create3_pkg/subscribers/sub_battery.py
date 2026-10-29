#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

class BatterySubscriber(Node):
    def __init__(self):
        super().__init__("battery_subscriber")
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to battery state
        self.sub = self.create_subscription(
            BatteryState, 
            "/robot_9/battery_state", 
            self.callback, 
            qos
        )
        self.get_logger().info("Subscribed to /robot_9/battery_state. Waiting for data...")

    def callback(self, msg):
        # =========================================================================
        # IMPORTANT EXTRACTABLE DATA:
        # - msg.percentage: Battery charge percentage (range: 0.0 to 1.0)
        # - msg.voltage: Battery voltage in Volts (V)
        # - msg.current: Current draw in Amperes (A). Negative means it's driving/discharging,
        #                Positive means it is currently charging on the dock.
        # - msg.power_supply_status: Constant indicating if it's charging, discharging, full, etc.
        # =========================================================================
        
        percentage = msg.percentage * 100 
        voltage = msg.voltage 
        current = msg.current 
        
        self.get_logger().info(f"Battery: {percentage:.1f}% | Voltage: {voltage:.2f}V | Current: {current:.2f}A")


def main(args=None):
    rclpy.init(args=args)
    node = BatterySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

