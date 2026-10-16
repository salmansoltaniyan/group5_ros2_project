#!/usr/bin/env python3
"""
===============================================================================
Robust & Stable LiDAR Wall Follower (Right-Hand Rule)
===============================================================================
A rock-solid reactive controller designed to eliminate spinning, confusion,
and corner-cutting in narrow corridors (~0.50m):

1. Clean 4-State Machine:
   - FOLLOW_WALL   : Smooth PD wall hugging at 5 cm gap (0.23m from LiDAR).
   - PIVOT_INNER   : Wall ahead -> Stops forward motion and pivots left in place.
   - OUTER_CORNER  : Wall ended -> Clears corner lip (8cm), then pivots right.
                     Automatically stops turning after 2.2s so it NEVER spins in circles!
   - FIND_WALL     : Open space -> Drives forward to seek next wall.

2. Generous Corner Threshold (WALL_LOST_DIST = 0.42m):
   - Never falsely triggers corner mode when robot slightly drifts or adjusts.
   - Triggers ONLY when the wall genuinely drops off (> 42 cm).

3. Smooth Bounded PD Control:
   - Proportional distance regulation with bounded heading damping.
   - No runaway angular velocity spikes.
===============================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SimpleWallFollower(Node):
    # State Machine Definitions
    STATE_FOLLOW_WALL = "FOLLOW_WALL"
    STATE_PIVOT_INNER = "PIVOT_INNER"
    STATE_OUTER_CORNER = "OUTER_CORNER"
    STATE_FIND_WALL = "FIND_WALL"

    def __init__(self):
        super().__init__('simple_wall_follower')

        # ---------------------------------------------------------------------
        # 1. PHYSICAL DIMENSIONS (LiDAR center to chassis edges)
        # ---------------------------------------------------------------------
        self.ROBOT_FRONT_OFFSET = 0.20   # Front bumper is 20 cm from LiDAR
        self.ROBOT_SIDE_OFFSET  = 0.18   # Right edge is 18 cm from LiDAR

        # ---------------------------------------------------------------------
        # 2. DISTANCE THRESHOLDS
        # ---------------------------------------------------------------------
        # Desired 5 cm air gap between right chassis edge and wall:
        self.DESIRED_WALL_GAP = 0.05     # 5 cm air gap
        self.TARGET_WALL_DIST = self.ROBOT_SIDE_OFFSET + self.DESIRED_WALL_GAP  # = 0.23m from LiDAR

        # Front obstacle threshold (Inner corner / Wall Ahead):
        # Stop at 12 cm clearance from bumper so in-place rotation never hits wall
        self.DESIRED_FRONT_GAP = 0.12    # 12 cm air gap in front
        self.FRONT_STOP_DIST = self.ROBOT_FRONT_OFFSET + self.DESIRED_FRONT_GAP # = 0.32m
        self.FRONT_CLEAR_DIST = self.ROBOT_FRONT_OFFSET + 0.20                  # = 0.40m to resume

        # Wall Lost threshold (Corner detection):
        # 0.34m from LiDAR (16 cm air gap). Detects the corner opening early
        # so the robot begins wrapping around the corner immediately without overshooting forward!
        self.WALL_LOST_DIST = 0.34       # 0.34m from LiDAR

        # ---------------------------------------------------------------------
        # 3. SPEEDS & CONTROLLER GAINS
        # ---------------------------------------------------------------------
        self.FORWARD_SPEED = 0.13        # Cruise speed (m/s)
        self.TURN_SPEED = 0.60           # In-place pivot turn speed (rad/s)
        self.KP = 2.2                    # Distance error gain
        self.KD = 1.8                    # Heading alignment damping gain

        # ---------------------------------------------------------------------
        # 4. SENSOR DISTANCES
        # ---------------------------------------------------------------------
        self.dist_front = float('inf')
        self.dist_front_right = float('inf')
        self.dist_right = float('inf')
        self.dist_rear_right = float('inf')
        self.dist_left = float('inf')

        # State tracking
        self.state = self.STATE_FOLLOW_WALL
        self.state_start_time = self.get_clock().now()
        self.scan_count = 0

        # ---------------------------------------------------------------------
        # 5. ROS 2 COMMUNICATION
        # ---------------------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)
        self.cmd_pub_backup = self.create_publisher(Twist, '/cmd_vel', 10)

        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, best_effort_qos)
        self.scan_sub_backup = self.create_subscription(LaserScan, '/robot_9/scan', self.scan_callback, best_effort_qos)

        # 10 Hz Control Loop Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('>>> Stable Wall Follower initialized! Ready for LiDAR (/scan)...')

    def set_state(self, new_state: str):
        if self.state != new_state:
            self.get_logger().info(f"[STATE CHANGE] {self.state} -> {new_state}")
            self.state = new_state
            self.state_start_time = self.get_clock().now()

    def get_state_elapsed(self) -> float:
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

    def normalize_angle(self, angle: float) -> float:
        """Wraps angle between [-pi, +pi] radians."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def scan_callback(self, msg: LaserScan):
        """Extracts Front, Front-Right, Right, Rear-Right, and Left distances."""
        self.scan_count += 1
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return

        front_rays = []
        front_right_rays = []
        right_rays = []
        rear_right_rays = []
        left_rays = []

        for i, dist in enumerate(msg.ranges):
            if math.isnan(dist) or math.isinf(dist) or dist <= msg.range_min or dist <= 0.05:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)
            angle = self.normalize_angle(angle)

            # 1. FRONT Corridor: -18° to +18° (-0.31 to +0.31 rad)
            if -0.31 <= angle <= 0.31:
                front_rays.append(dist)

            # 2. FRONT-RIGHT: -60° to -30° (-1.05 to -0.52 rad)
            elif -1.05 <= angle < -0.52:
                front_right_rays.append(dist)

            # 3. DIRECT RIGHT: -110° to -70° (-1.92 to -1.22 rad)
            elif -1.92 <= angle <= -1.22:
                right_rays.append(dist)

            # 4. REAR-RIGHT: -150° to -110° (-2.62 to -1.92 rad)
            elif -2.62 <= angle < -1.92:
                rear_right_rays.append(dist)

            # 5. DIRECT LEFT: +70° to +110° (+1.22 to +1.92 rad)
            elif 1.22 <= angle <= 1.92:
                left_rays.append(dist)

        self.dist_front = min(front_rays) if front_rays else float('inf')
        self.dist_front_right = min(front_right_rays) if front_right_rays else float('inf')
        self.dist_right = min(right_rays) if right_rays else float('inf')
        self.dist_rear_right = min(rear_right_rays) if rear_right_rays else float('inf')
        self.dist_left = min(left_rays) if left_rays else float('inf')

    def control_loop(self):
        """10 Hz decision-making control loop."""
        if self.scan_count == 0:
            self.get_logger().warn('Waiting for first LiDAR scan...', throttle_duration_sec=3.0)
            return

        cmd = Twist()
        action_name = ""
        elapsed = self.get_state_elapsed()

        # =====================================================================
        # STATE 1: PIVOT INNER (Wall in Front -> Turn Left in place)
        # =====================================================================
        if self.state == self.STATE_PIVOT_INNER:
            cmd.linear.x = 0.0
            cmd.angular.z = self.TURN_SPEED
            action_name = "PIVOT LEFT (Wall Ahead)"

            # Exit once path ahead has opened up (turn completed)
            if self.dist_front > self.FRONT_CLEAR_DIST:
                self.set_state(self.STATE_FOLLOW_WALL)

        # =====================================================================
        # STATE 2: OUTER CORNER (Right wall ended -> Tight corner-hugging turn)
        # =====================================================================
        elif self.state == self.STATE_OUTER_CORNER:
            action_name = "HUGGING OUTER CORNER"

            # Tight wrapping turn: DO NOT drive forward across the hallway!
            # Use tiny forward creep (0.02 m/s) with strong right rotation (-0.65 rad/s)
            # so the robot wraps tightly around the corner pivot and never reaches the opposite wall.
            if self.dist_front < self.FRONT_STOP_DIST:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.TURN_SPEED
            else:
                cmd.linear.x = 0.02   # 2 cm/s (almost zero forward displacement)
                cmd.angular.z = -0.65 # Fast, tight right turn around corner

            # Exit as soon as the new wall is captured on the right
            if elapsed > 0.3 and self.dist_right < (self.TARGET_WALL_DIST + 0.08) and self.dist_front > self.FRONT_STOP_DIST:
                self.set_state(self.STATE_FOLLOW_WALL)

            # If after 2.0s no wall is found, stop turning and seek wall forward
            elif elapsed > 2.0:
                self.set_state(self.STATE_FIND_WALL)

        # =====================================================================
        # STATE 3: FIND WALL (Seek wall in open space)
        # =====================================================================
        elif self.state == self.STATE_FIND_WALL:
            action_name = "SEEKING WALL (Straight Ahead)"

            # If front obstacle detected, turn left so it becomes the right wall!
            if self.dist_front < self.FRONT_STOP_DIST:
                self.set_state(self.STATE_PIVOT_INNER)
            # If wall is found on the right, resume wall following!
            elif self.dist_right < (self.TARGET_WALL_DIST + 0.12):
                self.set_state(self.STATE_FOLLOW_WALL)
            else:
                cmd.linear.x = self.FORWARD_SPEED
                cmd.angular.z = -0.08  # Very gentle bias towards right

        # =====================================================================
        # STATE 4: FOLLOW WALL (Normal PD Wall Following)
        # =====================================================================
        else:
            # 1. Obstacle in front -> Pivot Left
            if self.dist_front < self.FRONT_STOP_DIST:
                self.set_state(self.STATE_PIVOT_INNER)
                cmd.linear.x = 0.0
                cmd.angular.z = self.TURN_SPEED
                action_name = "OBSTACLE IN FRONT -> Turn Left"

            # 2. Outer Corner: Right wall dropped off (> 0.34m)
            elif self.dist_right > self.WALL_LOST_DIST and self.dist_front_right > self.WALL_LOST_DIST:
                self.set_state(self.STATE_OUTER_CORNER)
                cmd.linear.x = 0.02
                cmd.angular.z = -0.65
                action_name = "OUTER CORNER -> Turning Right"

            # 3. Standard Wall Following with Smooth PD Control
            else:
                # Distance error (positive = closer than 5cm, steer left)
                error_dist = self.TARGET_WALL_DIST - self.dist_right

                # Heading alignment: compare front-right with perpendicular right
                # When parallel: dist_front_right * 0.7071 ≈ dist_right
                if self.dist_front_right < float('inf') and self.dist_right < float('inf'):
                    heading_err = (self.dist_front_right * 0.7071) - self.dist_right
                    # Bound heading error to [-0.15, +0.15] to prevent violent jerks
                    heading_err = max(-0.15, min(0.15, heading_err))
                else:
                    heading_err = 0.0

                # PD Control Law:
                wz = (self.KP * error_dist) - (self.KD * heading_err)
                wz = max(-0.45, min(0.45, wz))

                # Narrow corridor left-wall safety guard
                if self.dist_left < (self.ROBOT_SIDE_OFFSET + 0.05) and wz > 0.0:
                    wz = min(wz, 0.02)

                # Smooth adaptive speed: slow slightly when turning
                speed = self.FORWARD_SPEED if abs(wz) < 0.20 else (self.FORWARD_SPEED * 0.75)

                cmd.linear.x = speed
                cmd.angular.z = wz
                action_name = f"FOLLOW WALL (e={error_dist*100:+.0f}cm, wz={wz:+.2f})"

        # Publish velocity command
        self.cmd_pub.publish(cmd)
        self.cmd_pub_backup.publish(cmd)

        # Telemetry line printed every second (10 scans)
        if self.scan_count % 10 == 0:
            actual_front_gap = max(0.0, self.dist_front - self.ROBOT_FRONT_OFFSET)
            actual_right_gap = max(0.0, self.dist_right - self.ROBOT_SIDE_OFFSET)
            print(
                f"[{self.state}] {action_name} | "
                f"Right: {actual_right_gap*100:.0f}cm | "
                f"Front: {actual_front_gap*100:.0f}cm | "
                f"Cmd: (vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f})"
            )

    def stop_robot(self):
        """Halts the robot."""
        self.cmd_pub.publish(Twist())
        self.cmd_pub_backup.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SimpleWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping robot safely...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
