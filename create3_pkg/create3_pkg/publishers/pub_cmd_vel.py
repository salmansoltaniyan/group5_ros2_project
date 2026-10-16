#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovePublisher(Node):
    def __init__(self):
        super().__init__("move_publisher")
        
        # =========================================================================
        # STEP 1: Create the Publisher
        # =========================================================================
        # 1. Message Type: Twist (Contains linear and angular velocities)
        # 2. Topic Name: /robot_9/cmd_vel
        # 3. Queue Size: 10
        self.publisher_ = self.create_publisher(Twist, "/robot_9/cmd_vel", 10)
        
        # =========================================================================
        # STEP 2: Create a Timer
        # =========================================================================
        # Robots need to hear movement commands constantly (e.g. every 0.5 seconds).
        # If they don't hear anything, they will stop automatically for safety.
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Move Publisher started! Sending drive commands...")

    def timer_callback(self):
        # =========================================================================
        # STEP 3: Create the message and Publish
        # =========================================================================
        msg = Twist()
        
        # Linear Velocity (Moving forward or backward in m/s)
        msg.linear.x = 0.15   # Move forward at 0.15 meters per second
        msg.linear.y = 0.0    # Create 3 cannot move sideways (no mecanum wheels)
        msg.linear.z = 0.0    # Cannot fly!
        
        # Angular Velocity (Turning left or right in rad/s)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5   # Turn LEFT at 0.5 radians per second
        
        # Send the command to the robot!
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: Forward 0.15 m/s, Turn Left 0.5 rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = MovePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

