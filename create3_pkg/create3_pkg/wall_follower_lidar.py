
#!/usr/bin/env python3

import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(value, low, high):
    return max(low, min(high, value))


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/robot_9/cmd_vel')

        # -1: right wall, +1: left wall
        self.declare_parameter('follow_side', -1)

        # Target distance from LiDAR to wall (meters)
        self.declare_parameter('target_distance', 0.25)

        # Motion limits
        self.declare_parameter('max_speed', 0.08)
        self.declare_parameter('min_speed', 0.025)
        self.declare_parameter('max_angular_speed', 0.65)

        # PID gains
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.12)

        # The LiDAR is 20 cm from the front of the robot.
        # Stop with a safety margin before the body reaches an obstacle.
        self.declare_parameter('front_stop_distance', 0.32)
        self.declare_parameter('front_slow_distance', 0.65)

        # Wall detection
        self.declare_parameter('wall_min_distance', 0.07)
        self.declare_parameter('wall_max_distance', 1.00)
        self.declare_parameter('wall_lost_timeout', 0.8)

        self.scan_topic = str(
            self.get_parameter('scan_topic').value
        )
        self.cmd_topic = str(
            self.get_parameter('cmd_vel_topic').value
        )

        self.side = int(self.get_parameter('follow_side').value)
        self.side = -1 if self.side < 0 else 1

        self.target = float(
            self.get_parameter('target_distance').value
        )
        self.max_speed = float(
            self.get_parameter('max_speed').value
        )
        self.min_speed = float(
            self.get_parameter('min_speed').value
        )
        self.max_omega = float(
            self.get_parameter('max_angular_speed').value
        )

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)

        self.front_stop = float(
            self.get_parameter('front_stop_distance').value
        )
        self.front_slow = float(
            self.get_parameter('front_slow_distance').value
        )

        self.wall_min = float(
            self.get_parameter('wall_min_distance').value
        )
        self.wall_max = float(
            self.get_parameter('wall_max_distance').value
        )
        self.wall_lost_timeout = float(
            self.get_parameter('wall_lost_timeout').value
        )

        # ROS interfaces
        self.cmd_pub = self.create_publisher(
            Twist, self.cmd_topic, 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Controller state
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

        self.error_history = deque(maxlen=3)

        self.last_scan_time = None
        self.last_wall_time = None
        self.last_log_time = 0.0

        # Watchdog: stop when LiDAR data becomes stale
        self.watchdog = self.create_timer(
            0.10, self.watchdog_callback
        )

        self.get_logger().info(
            f'Wall follower started | '
            f'scan: {self.scan_topic} | '
            f'cmd: {self.cmd_topic} | '
            f'side: {"right" if self.side == -1 else "left"} | '
            f'target: {self.target:.2f} m'
        )

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def angle_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def valid_range(self, value, scan):
        return (
            math.isfinite(value)
            and scan.range_min <= value <= scan.range_max
        )

    def sector_ranges(self, scan, center_deg, width_deg):
        center = math.radians(center_deg)
        half_width = math.radians(width_deg / 2.0)

        values = []

        for i, distance in enumerate(scan.ranges):
            if not self.valid_range(distance, scan):
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if abs(self.angle_diff(angle, center)) <= half_width:
                values.append(float(distance))

        return values

    def sector_min(self, scan, center_deg, width_deg):
        values = self.sector_ranges(
            scan, center_deg, width_deg
        )
        return min(values) if values else None

    def sector_median(self, scan, center_deg, width_deg):
        values = self.sector_ranges(
            scan, center_deg, width_deg
        )
        return statistics.median(values) if values else None

    def front_distance(self, scan):
        # The closest valid return in the front sector
        return self.sector_min(scan, 0.0, 30.0)

    def get_wall_distance(self, scan):
        """
        Estimate distance to the wall on the chosen side.
        Wider fallback helps when some rays are missing.
        """
        side_angle = self.side * 90.0

        distance = self.sector_median(
            scan, side_angle, 24.0
        )

        if distance is None:
            distance = self.sector_median(
                scan, side_angle, 40.0
            )

        if distance is None:
            return None

        if not self.wall_min <= distance <= self.wall_max:
            return None

        return distance

    # --------------------------------------------------
    # Main LiDAR callback
    # --------------------------------------------------

    def scan_callback(self, scan):
        now = self.now_seconds()

        valid_count = sum(
            1 for r in scan.ranges
            if self.valid_range(r, scan)
        )

        if valid_count < 10:
            self.stop_robot()
            return

        self.last_scan_time = now

        front = self.front_distance(scan)
        wall = self.get_wall_distance(scan)

        # ----------------------------------------------
        # 1. Front obstacle avoidance
        # ----------------------------------------------

        if front is not None and front <= self.front_stop:
            cmd = Twist()
            cmd.linear.x = 0.0

            # Turn away from the front obstacle.
            # Right-wall following: turn left.
            # Left-wall following: turn right.
            cmd.angular.z = 0.45 if self.side == -1 else -0.45

            self.cmd_pub.publish(cmd)

            self.integral = 0.0
            self.previous_error = None
            self.previous_time = now

            return

        # ----------------------------------------------
        # 2. Wall lost: search cautiously
        # ----------------------------------------------

        if wall is None:
            if self.last_wall_time is None:
                self.last_wall_time = now

            lost_for = now - self.last_wall_time
            cmd = Twist()

            if lost_for <= self.wall_lost_timeout:
                cmd.linear.x = 0.035

                # Search toward the expected wall.
                cmd.angular.z = 0.22 * self.side
            else:
                # Stop instead of driving blindly.
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.integral = 0.0
            self.previous_error = None
            self.previous_time = now

            return

        self.last_wall_time = now

        # ----------------------------------------------
        # 3. PID wall-distance controller
        # ----------------------------------------------

        # Positive error: wall is too far.
        # Negative error: wall is too close.
        error = wall - self.target

        self.error_history.append(error)
        filtered_error = statistics.median(
            self.error_history
        )

        if self.previous_time is None:
            dt = 0.05
        else:
            dt = clamp(now - self.previous_time, 0.01, 0.15)

        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (
                filtered_error - self.previous_error
            ) / dt

        self.integral += filtered_error * dt
        self.integral = clamp(
            self.integral, -0.20, 0.20
        )

        # ROS convention: positive angular.z turns left.
        # Right wall too far => negative angular velocity.
        # Left wall too far  => positive angular velocity.
        omega = self.side * (
            self.kp * filtered_error
            + self.ki * self.integral
            + self.kd * derivative
        )

        omega = clamp(
            omega, -self.max_omega, self.max_omega
        )

        # ----------------------------------------------
        # 4. Speed control
        # ----------------------------------------------

        speed = self.max_speed

        # Reduce speed when turning sharply.
        turn_ratio = min(
            abs(omega) / max(self.max_omega, 0.01), 1.0
        )

        speed *= 1.0 - 0.60 * turn_ratio

        # Reduce speed near a front obstacle.
        if front is not None and front < self.front_slow:
            ratio = (
                front - self.front_stop
            ) / max(
                self.front_slow - self.front_stop, 0.01
            )

            speed *= clamp(ratio, 0.0, 1.0)

        # When very close to the wall, turn away slowly.
        if wall < self.target - 0.07:
            speed = min(speed, 0.035)

        speed = clamp(speed, 0.0, self.max_speed)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = omega

        self.cmd_pub.publish(cmd)

        self.previous_error = filtered_error
        self.previous_time = now

        # Avoid flooding the terminal
        if now - self.last_log_time >= 0.5:
            self.get_logger().info(
                f'wall={wall:.2f} m | '
                f'front={front if front is not None else -1:.2f} m | '
                f'error={filtered_error:+.3f} | '
                f'v={speed:.3f} | w={omega:+.3f}'
            )
            self.last_log_time = now

    # --------------------------------------------------
    # Safety watchdog
    # --------------------------------------------------

    def watchdog_callback(self):
        if self.last_scan_time is None:
            self.stop_robot()
            return

        if self.now_seconds() - self.last_scan_time > 0.5:
            self.stop_robot()

            self.integral = 0.0
            self.previous_error = None
            self.previous_time = None


def main(args=None):
    rclpy.init(args=args)
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
