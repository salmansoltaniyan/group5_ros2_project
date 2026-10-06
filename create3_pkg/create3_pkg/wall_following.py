import math
from platform import node
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Subscribe to /scan topic
        self.scan_sub = self.create_subscription(
        LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info('Node started, waiting for lidar data...')

        # Create publisher for /cmd_vel topic
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.forward_speed = 0.1
        self.stop_distance = 0.5 # stop if the wall is closer than this distance
        self.turn_speed = 0.5 # positive is counter-clockwise, negative is clockwise
  
    def scan_callback(self, msg: LaserScan):
        # Get reading from 4 different directions
        front       = self.sector_distance(msg, center_deg=0.0,   half_width_deg=10.0)
        front_right = self.sector_distance(msg, center_deg=-45.0, half_width_deg=10.0)
        right       = self.sector_distance(msg, center_deg=-90.0, half_width_deg=10.0)
        front_left  = self.sector_distance(msg, center_deg=45.0,  half_width_deg=10.0)
        self.get_logger().info(
            f'front: {front:.2f}, front_right: {front_right:.2f}, '
            f'right: {right:.2f}, front_left: {front_left:.2f}')

        cmd = Twist()
        if front < self.stop_distance:
            cmd.linear.x = 0.0 # wall ahead, stop moving forward  
            cmd.angular.z = self.turn_speed # and turn left on the spot
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot by publishing zero velocities."""
        self.cmd_pub.publish(Twist()) # default Twist() has all zeros

    def sector_distance(self, msg, center_deg, half_width_deg):
        """Closest valid reading inside a slice of the scan."""

        # Convert angle from degree to radian
        center = math.radians(center_deg)
        half = math.radians(half_width_deg)

        # Formular to know the box range for the angles out of 0 - 359
        start = int((center - half - msg.angle_min) / msg.angle_increment)
        end = int((center + half - msg.angle_min) / msg.angle_increment)

        # Safety so it doesnt read anything outside of box range
        start = max(start, 0)
        end = min(end, len(msg.ranges) - 1)

        # List to get only valid readings and get the min value
        # Can later be adjusted to get mean or average value depending on behavior of the robot
        valid = [
            r for r in msg.ranges[start:end + 1]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        return min(valid) if valid else float('inf')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()