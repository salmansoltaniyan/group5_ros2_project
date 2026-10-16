#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__("odometry_subscriber")
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to odometry (Robot position and velocity)
        self.sub = self.create_subscription(
            Odometry, 
            "/robot_9/odom", 
            self.callback, 
            qos
        )
        self.get_logger().info("Subscribed to /robot_9/odom. Waiting for position data...")

    def callback(self, msg):
        # Odometry contains two main parts:
        # 1. Pose: The robot's Position (x,y,z) and Orientation (quaternions) in the world.
        # 2. Twist: The robot's Linear Velocity (speed) and Angular Velocity (turn rate).
        
        # =========================================================================
        # IMPORTANT EXTRACTABLE DATA (POSE):
        # =========================================================================
        # Position in meters (relative to starting point)
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        
        # Orientation (This is a Quaternion, usually converted to Euler angles for yaw/heading)
        # For simplicity, we just extract z and w here.
        orient_z = msg.pose.pose.orientation.z
        orient_w = msg.pose.pose.orientation.w
        
        # =========================================================================
        # IMPORTANT EXTRACTABLE DATA (TWIST - VELOCITY):
        # =========================================================================
        # Linear speed in meters per second (positive = forward)
        linear_speed = msg.twist.twist.linear.x
        
        # Angular speed in radians per second (positive = turning left)
        angular_speed = msg.twist.twist.angular.z
        
        self.get_logger().info(f"Pos: X={x_pos:.2f}, Y={y_pos:.2f} | Speed: V_x={linear_speed:.2f}, V_yaw={angular_speed:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = OdometrySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

