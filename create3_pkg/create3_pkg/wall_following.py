import math
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
        # subscribe to the lidar scan topic and publish to the robot's cmd_vel topic
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)

        # Speeds
        self.forward_speed = 0.1    # m/s
        self.lost_speed = 0.08      # m/s, forward creep to sweep around outside corners
        self.turn_speed = 0.5       # rad/s, turning in place for front obstacles, positive = left

        # Wall ahead: turn in place
        self.stop_distance = 0.35   # m, brake below this to protect the physical bumper
        self.clear_distance = 0.40  # m, stop turning above this

        # Right-wall following & PD Controller
        self.target_distance = 0.20     # m
        self.kp = 3.0                   # proportional gain
        self.kd = 0.6                   # derivative gain (damps oscillations)
        self.prev_error = 0.0           # tracked for the derivative calculation
        self.lookahead = 0.30           # m, how far ahead we predict the distance
        self.max_turn = 1             # rad/s, limit for steering
        self.max_wall_distance = 0.55   # m, outside corner threshold (farther than this = hook corner)
        self.beam_angle = math.radians(45.0)   # angle between the two beams

        self.get_logger().info('Node started, waiting for lidar data...')

    def scan_callback(self, msg: LaserScan):
        front = self.sector_distance(msg, center_deg=0.0,   half_width_deg=10.0)
        right = self.sector_distance(msg, center_deg=-90.0, half_width_deg=1.5)
        front_right = self.sector_distance(msg, center_deg=-45.0, half_width_deg=1.5)

        cmd = Twist()
        state = ""
        wall_angle = 0.0
        wall_dist = float('inf')

        # ==========================================
        # STATE 1: Obstacle Ahead (Stop & Spin Left)
        # ==========================================
        if front < self.stop_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
            self.prev_error = 0.0  # Reset error term when interrupting tracking
            state = "OBSTACLE"

        # ==========================================
        # STATE 2: Outside Corner (Creep & Tighter Arc Right)
        # ==========================================
        elif right > self.max_wall_distance:
            cmd.linear.x = self.lost_speed
            cmd.angular.z = -1.0  # Tighter turn to hook the outside corner cleanly
            self.prev_error = 0.0  # Reset error term during recovery
            state = "HOOK_CORNER"

        # ==========================================
        # STATE 3: PD Lookahead Tracking
        # ==========================================
        else:
            # Angle of the wall relative to the robot (positive = nose pointing away)
            if front_right < self.max_wall_distance:
                wall_angle = math.atan2(front_right * math.cos(self.beam_angle) - right, front_right * math.sin(self.beam_angle))
                wall_angle = max(-1.0, min(1.0, wall_angle))

            wall_dist = right * math.cos(wall_angle)
            future_dist = wall_dist + self.lookahead * math.sin(wall_angle)
            error = self.target_distance - future_dist

            # PD Controller calculation
            steering = (self.kp * error) + (self.kd * (error - self.prev_error))
            self.prev_error = error

            cmd.linear.x = self.forward_speed
            cmd.angular.z = max(-self.max_turn, min(self.max_turn, steering))
            state = "TRACKING"

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'State: {state} | front: {front:.2f}, right: {right:.2f}, '
            f'angle: {math.degrees(wall_angle):.1f}, speed: {cmd.linear.x:.2f}, steer: {cmd.angular.z:.2f}')

    def stop_robot(self):
        self.cmd_pub.publish(Twist())   # all zeros = stop
        time.sleep(0.1)

    def sector_distance(self, msg, center_deg, half_width_deg):
        """Closest valid reading inside a slice of the scan."""
        center = math.radians(center_deg)
        half = math.radians(half_width_deg)

        start = int((center - half - msg.angle_min) / msg.angle_increment)
        end = int((center + half - msg.angle_min) / msg.angle_increment)
        start = max(start, 0)
        end = min(end, len(msg.ranges) - 1)

        valid = [
            r for r in msg.ranges[start:end + 1]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        # Cap infinity at 3.0m so math never breaks on outside corners
        return min(valid) if valid else 3.0


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