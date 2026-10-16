#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import LightringLeds, LedColor

class LightringPublisher(Node):
    def __init__(self):
        super().__init__("lightring_publisher")
        
        self.publisher_ = self.create_publisher(LightringLeds, "/robot_9/cmd_lightring", 10)
        
        # Update LEDs every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("LED Publisher started!")
        
        # A simple variable to switch colors every second
        self.toggle = True

    def timer_callback(self):
        msg = LightringLeds()
        
        # IMPORTANT: You MUST set override_system to True, otherwise the robot 
        # will ignore you and keep showing its default white/green battery lights.
        msg.override_system = True
        
        # Define some colors (Red, Green, Blue from 0 to 255)
        red_color = LedColor(red=255, green=0, blue=0)
        blue_color = LedColor(red=0, green=0, blue=255)
        
        if self.toggle:
            # Set all 6 LEDs to Red
            msg.leds = [red_color, red_color, red_color, red_color, red_color, red_color]
        else:
            # Set all 6 LEDs to Blue
            msg.leds = [blue_color, blue_color, blue_color, blue_color, blue_color, blue_color]
            
        # Flip the toggle for the next time
        self.toggle = not self.toggle
        
        self.publisher_.publish(msg)
        self.get_logger().info("Changed LED colors!")

def main(args=None):
    rclpy.init(args=args)
    node = LightringPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

