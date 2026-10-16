#!/usr/bin/env python3
"""
===============================================================================
Optimized LiDAR Wall Follower with Dedicated Functional Beams
===============================================================================
Architectural Improvements:
1. Dedicated Functional Beams (Replaces uniform sectoring):
   - FRONT Corridor : -15° to +15° (collision buffer, never falsely clips side walls)
   - WALL_FRONT     : ±45° (slope / lookahead beam)
   - WALL_SIDE      : ±90° (perpendicular true wall distance)
   - WALL_REAR      : ±135° (corner clearance / wheel axle monitor)
   - OPPOSITE_SIDE  : ±90° (opposite corridor wall clearance)
2. Exact Geometric Math:
   - Calculates true perpendicular distance: D_true = D_90 * cos(tilt)
   - Calculates exact heading tilt angle: tilt = atan2(D_45 * cos(45°) - D_90, ...)
3. Reconfigurable:
   - FOLLOW_SIDE = 'RIGHT' (default right-hand rule) or 'LEFT' (left-hand rule)
4. Decisive, Crisp Cornering & High Speed:
   - Cruise speed: 0.18 m/s, pivot turn: 0.60 rad/s
   - Zero-radius in-place turns with minimum duration & cooldown
   - Auto-Recovery reverse if within 5 cm of obstacle
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
    STATE_CORNER_PASS = "CORNER_PASS"
    STATE_CORNER_PIVOT = "CORNER_PIVOT"
    STATE_RECOVERY = "RECOVERY"
    STATE_FIND_WALL = "FIND_WALL"

    def __init__(self):
        super().__init__('simple_wall_follower')

        # =====================================================================
        # 1. WALL FOLLOWING SIDE SELECTION
        # =====================================================================
        # 'RIGHT' -> Follow Right Wall (Default)
        # 'LEFT'  -> Follow Left Wall
        self.FOLLOW_SIDE = 'RIGHT'

        # =====================================================================
        # 2. PHYSICAL DIMENSIONS (LiDAR center to chassis edges)
        # =====================================================================
        self.ROBOT_FRONT_OFFSET = 0.20   # Front bumper is 20 cm from LiDAR
        self.ROBOT_BACK_OFFSET  = 0.15   # Rear bumper is 15 cm from LiDAR
        self.ROBOT_SIDE_OFFSET  = 0.18   # Left/Right edges are 18 cm from LiDAR

        # =====================================================================
        # 3. DISTANCE THRESHOLDS (Tuned for ultra-close 4cm wall hugging)
        # =====================================================================
        # 4 cm air gap between chassis edge and wall:
        self.DESIRED_WALL_GAP = 0.04     # 4 cm physical air gap
        self.TARGET_WALL_DIST = self.ROBOT_SIDE_OFFSET + self.DESIRED_WALL_GAP  # = 0.22m

        # Emergency reverse buffer (front bumper within 5 cm):
        self.COLLISION_BUFFER = self.ROBOT_FRONT_OFFSET + 0.05                  # = 0.25m

        # Inner corner / Wall ahead stop distance (15 cm clearance from front bumper):
        self.DESIRED_FRONT_GAP = 0.15    # 15 cm air gap in front
        self.FRONT_STOP_DIST = self.ROBOT_FRONT_OFFSET + self.DESIRED_FRONT_GAP # = 0.35m
        self.FRONT_CLEAR_DIST = self.ROBOT_FRONT_OFFSET + 0.22                  # = 0.42m

        # Wall lost threshold (outer corner opening):
        self.WALL_LOST_DIST = self.ROBOT_SIDE_OFFSET + 0.11                     # = 0.29m

        # =====================================================================
        # 4. SPEEDS & CONTROLLER GAINS
        # =====================================================================
        self.FORWARD_SPEED_FAST = 0.18   # Straight cruise speed (m/s)
        self.FORWARD_SPEED_SLOW = 0.11   # Turn entry / alignment speed (m/s)
        self.TURN_SPEED = 0.60           # In-place pivot turn speed (rad/s)
        self.KP = 2.5                    # Gain for perpendicular distance error
        self.KD = 3.2                    # Gain for heading alignment error

        # =====================================================================
        # 5. DEDICATED FUNCTIONAL BEAMS (Distances in meters)
        # =====================================================================
        self.dist_front = float('inf')        # FRONT (-15° to +15°)
        self.dist_wall_front = float('inf')   # ±45° (slope / lookahead)
        self.dist_wall_side = float('inf')    # ±90° (perpendicular wall distance)
        self.dist_wall_rear = float('inf')    # ±135° (corner axle clearance)
        self.dist_opposite_side = float('inf')# Opposite ±90° (corridor guard)

        # Derived real-time wall metrics
        self.true_wall_dist = float('inf')
        self.wall_tilt_deg = 0.0

        # State tracking
        self.state = self.STATE_FOLLOW_WALL
        self.state_start_time = self.get_clock().now()
        self.cooldown_end_time = self.get_clock().now()
        self.scan_count = 0

        # =====================================================================
        # 6. ROS 2 COMMUNICATION
        # =====================================================================
        self.cmd_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)
        self.cmd_pub_backup = self.create_publisher(Twist, '/cmd_vel', 10)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.scan_sub_backup = self.create_subscription(LaserScan, '/robot_9/scan', self.scan_callback, sensor_qos)

        # 10 Hz Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'>>> Optimized Wall Follower ({self.FOLLOW_SIDE} Wall) started!')

    def set_state(self, new_state: str, cooldown_sec: float = 0.0):
        if self.state != new_state:
            self.get_logger().info(f"[STATE CHANGE] {self.state} -> {new_state}")
            self.state = new_state
            self.state_start_time = self.get_clock().now()
            if cooldown_sec > 0.0:
                self.cooldown_end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=cooldown_sec)

    def get_state_elapsed(self) -> float:
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

    def is_in_cooldown(self) -> bool:
        return self.get_clock().now() < self.cooldown_end_time

    def normalize_angle(self, angle: float) -> float:
        """Wraps angle between [-pi, +pi] radians."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def scan_callback(self, msg: LaserScan):
        """
        Extracts dedicated functional beams:
        - FRONT Corridor : -15° to +15°
        - WALL_FRONT     : 45° beam on following side
        - WALL_SIDE      : 90° beam on following side
        - WALL_REAR      : 135° beam on following side
        - OPPOSITE_SIDE  : 90° beam on opposite side
        """
        self.scan_count += 1
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return

        front_rays = []
        wall_front_rays = []
        wall_side_rays = []
        wall_rear_rays = []
        opposite_side_rays = []

        is_right = (self.FOLLOW_SIDE == 'RIGHT')

        for i, dist in enumerate(msg.ranges):
            if math.isnan(dist) or math.isinf(dist) or dist <= msg.range_min or dist <= 0.05:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)
            angle = self.normalize_angle(angle)

            # 1. FRONT Corridor: -15° to +15° (-0.26 to +0.26 rad)
            if -0.26 <= angle <= 0.26:
                front_rays.append(dist)

            # Beams on Following Wall Side (Negative if RIGHT, Positive if LEFT)
            if is_right:
                # 45° Beam: -50° to -40° (-0.87 to -0.70 rad)
                if -0.87 <= angle <= -0.70:
                    wall_front_rays.append(dist)
                # 90° Beam: -95° to -85° (-1.66 to -1.48 rad)
                elif -1.66 <= angle <= -1.48:
                    wall_side_rays.append(dist)
                # 135° Beam: -140° to -125° (-2.44 to -2.18 rad)
                elif -2.44 <= angle <= -2.18:
                    wall_rear_rays.append(dist)
                # Opposite 90° Beam (Left): +85° to +95° (+1.48 to +1.66 rad)
                elif 1.48 <= angle <= 1.66:
                    opposite_side_rays.append(dist)
            else:
                # 45° Beam: +40° to +50° (+0.70 to +0.87 rad)
                if 0.70 <= angle <= 0.87:
                    wall_front_rays.append(dist)
                # 90° Beam: +85° to +95° (+1.48 to +1.66 rad)
                elif 1.48 <= angle <= 1.66:
                    wall_side_rays.append(dist)
                # 135° Beam: +125° to +140° (+2.18 to +2.44 rad)
                elif 2.18 <= angle <= 2.44:
                    wall_rear_rays.append(dist)
                # Opposite 90° Beam (Right): -95° to -85° (-1.66 to -1.48 rad)
                elif -1.66 <= angle <= -1.48:
                    opposite_side_rays.append(dist)

        # Minimum for front obstacle detection
        self.dist_front = min(front_rays) if front_rays else float('inf')

        # Averaged narrow cones for noise-free wall following
        self.dist_wall_front = (sum(wall_front_rays) / len(wall_front_rays)) if wall_front_rays else float('inf')
        self.dist_wall_side  = (sum(wall_side_rays) / len(wall_side_rays)) if wall_side_rays else float('inf')
        self.dist_wall_rear  = (sum(wall_rear_rays) / len(wall_rear_rays)) if wall_rear_rays else float('inf')
        self.dist_opposite_side = (sum(opposite_side_rays) / len(opposite_side_rays)) if opposite_side_rays else float('inf')

        # Calculate exact geometric wall metrics (Tilt & True Perpendicular Distance)
        if self.dist_wall_front < float('inf') and self.dist_wall_side < float('inf') and self.dist_wall_side < 2.0:
            diff = (self.dist_wall_front * 0.7071) - self.dist_wall_side
            tilt_rad = math.atan2(diff, self.dist_wall_front * 0.7071)
            self.wall_tilt_deg = math.degrees(tilt_rad)
            self.true_wall_dist = self.dist_wall_side * math.cos(tilt_rad)
        else:
            self.true_wall_dist = self.dist_wall_side
            self.wall_tilt_deg = 0.0

    def control_loop(self):
        """10 Hz reactive decision-making loop."""
        if self.scan_count == 0:
            self.get_logger().warn('Waiting for first LiDAR scan...', throttle_duration_sec=3.0)
            return

        cmd = Twist()
        action_name = ""
        elapsed = self.get_state_elapsed()
        is_right = (self.FOLLOW_SIDE == 'RIGHT')

        # Direction signs based on side:
        # For Right Wall: Inner corner turn = LEFT (+z), Outer corner turn = RIGHT (-z)
        # For Left Wall : Inner corner turn = RIGHT (-z), Outer corner turn = LEFT (+z)
        inner_turn_dir = +1.0 if is_right else -1.0
        outer_turn_dir = -1.0 if is_right else +1.0

        # =====================================================================
        # SAFETY OVERRIDE: Active Recovery / Unsticking
        # If front bumper is within 5 cm of wall, reverse straight back!
        # =====================================================================
        if self.dist_front < self.COLLISION_BUFFER and self.state != self.STATE_RECOVERY:
            self.set_state(self.STATE_RECOVERY)

        # =====================================================================
        # STATE 1: RECOVERY (Reverse Straight Back to Free Bumper)
        # =====================================================================
        if self.state == self.STATE_RECOVERY:
            cmd.linear.x = -0.08
            cmd.angular.z = 0.0
            action_name = "RECOVERY (Reversing to Free Bumper)"

            if self.dist_front > (self.ROBOT_FRONT_OFFSET + 0.12) or elapsed > 0.6:
                self.set_state(self.STATE_PIVOT_INNER)

        # =====================================================================
        # STATE 2: PIVOT INNER (Inner Corner / Wall Ahead)
        # =====================================================================
        elif self.state == self.STATE_PIVOT_INNER:
            cmd.linear.x = 0.0
            cmd.angular.z = inner_turn_dir * self.TURN_SPEED
            action_name = f"PIVOT INNER (Turning {'Left' if inner_turn_dir > 0 else 'Right'})"

            # Turn at least 1.2s to prevent premature exit
            turn_completed = (
                self.dist_front > self.FRONT_CLEAR_DIST and
                (self.dist_wall_side < (self.WALL_LOST_DIST + 0.06) or self.dist_front > 0.65)
            )
            if (elapsed >= 1.2 and turn_completed) or elapsed > 3.2:
                self.set_state(self.STATE_FOLLOW_WALL, cooldown_sec=0.5)

        # =====================================================================
        # STATE 3: CORNER PASS (Outer corner: wheel axle clearing corner)
        # =====================================================================
        elif self.state == self.STATE_CORNER_PASS:
            cmd.linear.x = 0.10
            cmd.angular.z = 0.0
            action_name = "CORNER PASS (Clearing Axle)"

            # If front obstacle approaches, stop forward motion and pivot immediately
            if self.dist_front < self.FRONT_STOP_DIST:
                self.set_state(self.STATE_CORNER_PIVOT)
            # Cleared corner when 135° beam clears or 1.2s passed (~12 cm forward)
            elif elapsed >= 1.2 or (elapsed > 0.5 and self.dist_wall_rear > self.WALL_LOST_DIST):
                self.set_state(self.STATE_CORNER_PIVOT)

        # =====================================================================
        # STATE 4: CORNER PIVOT (Tight In-Place Pivot around Corner)
        # =====================================================================
        elif self.state == self.STATE_CORNER_PIVOT:
            cmd.linear.x = 0.0
            cmd.angular.z = outer_turn_dir * self.TURN_SPEED
            action_name = f"CORNER PIVOT (In-Place Turn {'Right' if outer_turn_dir < 0 else 'Left'})"

            # Must turn at least 1.3s so it never aborts prematurely
            turn_completed = (
                self.dist_wall_side < (self.TARGET_WALL_DIST + 0.08) and
                self.dist_front > self.FRONT_STOP_DIST
            )
            if (elapsed >= 1.3 and turn_completed) or elapsed > 3.6:
                self.set_state(self.STATE_FOLLOW_WALL, cooldown_sec=0.5)

        # =====================================================================
        # STATE 5: FIND WALL (Seek wall if lost in open space)
        # =====================================================================
        elif self.state == self.STATE_FIND_WALL:
            action_name = f"FIND WALL (Seeking {self.FOLLOW_SIDE} Wall)"
            if self.dist_front < self.FRONT_STOP_DIST:
                self.set_state(self.STATE_PIVOT_INNER)
            elif self.dist_wall_side < self.WALL_LOST_DIST:
                self.set_state(self.STATE_FOLLOW_WALL, cooldown_sec=0.5)
            else:
                cmd.linear.x = 0.11
                cmd.angular.z = outer_turn_dir * 0.25

        # =====================================================================
        # STATE 6: FOLLOW WALL (High-Speed PD Wall Following)
        # =====================================================================
        else:
            # 1. Obstacle directly ahead -> Pivot Inner
            if self.dist_front < self.FRONT_STOP_DIST:
                self.set_state(self.STATE_PIVOT_INNER)
                cmd.linear.x = 0.0
                cmd.angular.z = inner_turn_dir * self.TURN_SPEED
                action_name = "OBSTACLE AHEAD -> Inner Pivot"

            # 2. Outer corner detected: Wall on following side dropped off
            elif not self.is_in_cooldown() and self.dist_wall_side > self.WALL_LOST_DIST and self.dist_wall_front > self.WALL_LOST_DIST:
                self.set_state(self.STATE_CORNER_PASS)
                cmd.linear.x = 0.10
                cmd.angular.z = 0.0
                action_name = f"OUTER CORNER ({self.FOLLOW_SIDE} Wall Dropped Off)"

            # 3. Standard Wall Following with Geometric PD Controller
            else:
                # Perpendicular distance error (positive = closer than 4cm)
                error_dist = self.TARGET_WALL_DIST - self.true_wall_dist

                # Heading alignment error (positive = tilted away, negative = tilted into wall)
                if self.dist_wall_front < float('inf') and self.dist_wall_side < float('inf'):
                    heading_diff = (self.dist_wall_front * 0.7071) - self.dist_wall_side
                else:
                    heading_diff = 0.0

                # PD Control Law:
                if is_right:
                    wz = (self.KP * error_dist) - (self.KD * heading_diff)
                else:
                    wz = -((self.KP * error_dist) - (self.KD * heading_diff))

                wz = max(-0.55, min(0.55, wz))

                # Opposite wall safety guard (avoid scraping opposite wall in narrow corridor)
                if self.dist_opposite_side < (self.ROBOT_SIDE_OFFSET + 0.05):
                    if is_right and wz > 0.0:
                        wz = min(wz, 0.02)
                    elif not is_right and wz < 0.0:
                        wz = max(wz, -0.02)

                # Dynamic Adaptive Speed:
                # Full 0.18 m/s on straights, eases to 0.11 m/s when adjusting angle
                if abs(wz) < 0.20 and self.dist_front > 0.55:
                    linear_x = self.FORWARD_SPEED_FAST
                else:
                    linear_x = self.FORWARD_SPEED_SLOW

                cmd.linear.x = linear_x
                cmd.angular.z = wz
                action_name = (
                    f"FOLLOW ({self.FOLLOW_SIDE}: D={self.true_wall_dist*100:.1f}cm, "
                    f"Tilt={self.wall_tilt_deg:+.1f}°, wz={wz:+.2f})"
                )

        # Publish velocity command
        self.cmd_pub.publish(cmd)
        self.cmd_pub_backup.publish(cmd)

        # Clean telemetry every second (10 scans)
        if self.scan_count % 10 == 0:
            actual_front_gap = max(0.0, self.dist_front - self.ROBOT_FRONT_OFFSET)
            actual_wall_gap = max(0.0, self.true_wall_dist - self.ROBOT_SIDE_OFFSET)
            print(
                f"[{self.state}] {action_name} | "
                f"Wall-Gap: {actual_wall_gap*100:.1f}cm | "
                f"Front-Gap: {actual_front_gap*100:.1f}cm | "
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
