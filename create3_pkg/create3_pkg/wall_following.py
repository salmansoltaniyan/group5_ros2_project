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

        # Create publisher for /cmd_vel topic
        self.cmd_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)

        # Driving and front-obstacle settings
        self.forward_speed = 0.1
        self.slow_speed = 0.05 # slow down when right wall is lost
        self.turn_speed = 0.5 # turning in place, positive = left, negative = right

        # Turning in place: front obstacle detection thresholds
        self.stop_distance = 0.25 # stop if the wall is closer than this distance
        self.clear_distance = 0.4 # clear distance to stop turning

        # Turning in place: inside corners detection thresholds
        self.corner_distance = 0.28 # distance to detect inside corner
        self.corner_clear = 0.36 # distance to detect corner is clear

        self.turning = False # are we currently turning?

        # Wall-following settings
        self.desired_distance = 0.3 # desired distance from the wall
        self.kp = 1.5 # proportional gain for wall-following(tune this value to adjust the responsiveness of the robot)
        self.max_turn = 0.5 # maximum turning speed for steering corrections
        self.max_wall_distance = 1.0 # readinfs beyond this distance count as 1.0
        self.lost_wall_distance = 0.60 # if the wall is further than this, wall lost, hence slow down

        # Searching for a wall
        self.lost_scan = 0 # how many scans in a row have we lost the wall? (for searching)
        self.search_scans = 40 # about 5s at 8 scans/s, how many scans in a row to slow down when wall is lost
        self.get_logger().info('Node started, waiting for lidar data...')
  
    def scan_callback(self, msg: LaserScan):
        # Get reading from 4 different directions
        front       = self.sector_distance(msg, center_deg=0.0,   half_width_deg=10.0)
        front_right = self.sector_distance(msg, center_deg=-45.0, half_width_deg=10.0)
        right       = self.sector_distance(msg, center_deg=-90.0, half_width_deg=10.0)
        # front_left  = self.sector_distance(msg, center_deg=45.0,  half_width_deg=10.0)

        # Decide whether we are turning (two thresholds = hysteresis)
        if self.turning:
            if front >= self.clear_distance:
                self.turning = False    # clearly open now, stop turning
        else:
            if front < self.stop_distance:
                self.turning = True     # too close, start turning

        cmd = Twist()
        if self.turning:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
        else:
            # P controller for wall-following
            wall = min(right, self.max_wall_distance)
            error = self.desired_distance - wall
            cmd.linear.x = self.forward_speed
            cmd.angular.z = max(-self.max_turn, min(self.kp * error, self.max_turn))

            # Slow down when wall is lost so the turn around the corner is tighter
            if wall > self.lost_wall_distance:
                self.lost_scan += 1
                if self.lost_scan > self.search_scans:
                    # Lost the wall for a while, stop circular motion and drive straight to find the wall
                    self.get_logger().info('Wall lost for a while, going straight...')
                    cmd.angular.z = 0.0
                    cmd.linear.x = self.forward_speed
                else:
                    # Probsbly an outside corner, go slowly and curve right
                    cmd.linear.x = self.slow_speed
            else:
                # Wall is found, reset lost_scan counter
                self.lost_scan = 0
                cmd.linear.x = self.forward_speed
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
                    f'front: {front:.2f}, front_right: {front_right:.2f}, '
                    f'right: {right:.2f}, turning: {self.turning},' 
                    f'linear.x: {cmd.linear.x:.2f}, angular.z: {cmd.angular.z:.2f},'
                    f'lost_scan: {self.lost_scan}')

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