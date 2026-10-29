#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import InterfaceButtons

class ButtonsSubscriber(Node):
    def __init__(self):
        super().__init__("buttons_subscriber")
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to interface buttons (The physical buttons on top of the robot)
        self.sub = self.create_subscription(
            InterfaceButtons, 
            "/robot_9/interface_buttons", 
            self.callback, 
            qos
        )
        self.get_logger().info("Subscribed to /robot_9/interface_buttons. Press a button on the robot!")

    def callback(self, msg):
        # InterfaceButtons contains the state of the 3 buttons on the Create 3 robot.
        
        # =========================================================================
        # IMPORTANT EXTRACTABLE DATA:
        # - msg.button_1.is_pressed: Boolean (True/False) for the 'Dot' button
        # - msg.button_power.is_pressed: Boolean (True/False) for the middle Power button
        # - msg.button_2.is_pressed: Boolean (True/False) for the 'Double Dot' button
        # =========================================================================
        
        if msg.button_1.is_pressed:
            self.get_logger().info("Button 1 (Dot) was PRESSED!")
            
        if msg.button_power.is_pressed:
            self.get_logger().info("Power Button was PRESSED!")
            
        if msg.button_2.is_pressed:
            self.get_logger().info("Button 2 (Double Dot) was PRESSED!")


def main(args=None):
    rclpy.init(args=args)
    node = ButtonsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

