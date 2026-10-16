#!/usr/bin/env python3
"""
===============================================================================
High-Precision Wall Follower with Stanley Controller & Tight Corner Hugging
===============================================================================
Key Upgrades:
1. Constant Wall Distance (Stanley Controller):
   - Computes desired heading angle based on cross-track distance error:
       desired_heading = atan2(KP_DIST * dist_error, forward_speed)
   - Eliminates steady-state drift! If the robot drifts away, it smoothly angles
     towards the wall, closes the gap quickly, and locks onto the 4cm target.
2. Tight Corner Hugging (Outer Corner):
   - Early corner detection at 0.32m (right at the corner vertex).
   - Tight wrapping turn (R ≈ 7-8 cm) wraps closely around the 11cm corner lip.
   - Never shoots forward into the corridor or sees the opposite wall!
3. Crisp Inner Corner Pivoting:
   - Zero forward motion (linear.x = 0.0), pivots in place until path opens.
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
        # Desired 4 cm air gap between right chassis edge and wall:
        self.DESIRED_WALL_GAP = 0.04     # 4 cm air gap
        self.TARGET_WALL_DIST = self.ROBOT_SIDE_OFFSET + self.DESIRED_WALL_GAP  # = 0.22m from LiDAR

        # Front obstacle threshold (Inner corner / Wall Ahead):
        # Stop at 12 cm clearance from bumper so in-place rotation never hits wall
        self.DESIRED_FRONT_GAP = 0.12    # 12 cm air gap in front
        self.FRONT_STOP_DIST = self.ROBOT_FRONT_OFFSET + self.DESIRED_FRONT_GAP # = 0.32m
        self.FRONT_CLEAR_DIST = self.ROBOT_FRONT_OFFSET + 0.18                  # = 0.38m to resume

        # Early Corner detection threshold:
        # Detects the corner opening early (0.32m = 14 cm air gap)
        # so the robot begins wrapping around the corner immediately!
        self.WALL_LOST_DIST = 0.32       # 0.32m from LiDAR

        # ---------------------------------------------------------------------
        # 3. SPEEDS & CONTROLLER GAINS (Stanley Wall Following - Smoothed)
        # ---------------------------------------------------------------------
        self.FORWARD_SPEED = 0.12        # Cruise speed (m/s) - stable cruising
        self.TURN_SPEED = 0.60           # In-place pivot turn speed (rad/s)

        # Stanley Controller Gains (Smooth & Non-Oscillating):
        self.KP_DIST = 0.75              # Cross-track error gain (1cm error -> ~3.3° heading)
        self.KP_HEAD = 1.35              # Heading alignment damping gain
        self.DEADBAND_DIST = 0.007       # 7mm deadband around 4cm target to eliminate hunting
        self.MAX_STEER_WZ = 0.22         # Gentle steering cap for straight walls (rad/s)
        self.prev_wz = 0.0               # Slew rate limiter memory
        self.MAX_WZ_ACCEL = 0.06         # Max change in wz per 0.1s tick (smooth acceleration)

        # ---------------------------------------------------------------------
        # 4. SENSOR DISTANCES & FILTERS
        # ---------------------------------------------------------------------
        self.dist_front = float('inf')
        self.dist_front_right = float('inf')
        self.dist_right = float('inf')
        self.dist_rear_right = float('inf')
        self.dist_left = float('inf')

        # Exponential Moving Average (EMA) filters for noise reduction
        self.filt_dist_right = None
        self.filt_dist_front_right = None

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

        self.get_logger().info('>>> Stanley Wall Follower initialized! Ready for LiDAR (/scan)...')

    def set_state(self, new_state: str):
        if self.state != new_state:
            self.get_logger().info(f"[STATE CHANGE] {self.state} -> {new_state}")
            self.state = new_state
            self.state_start_time = self.get_clock().now()
            self.prev_wz = 0.0

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

            # 1. FRONT Corridor: -16° to +16° (-0.28 to +0.28 rad)
            if -0.28 <= angle <= 0.28:
                front_rays.append(dist)

            # 2. FRONT-RIGHT: -55° to -35° (-0.96 to -0.61 rad)
            elif -0.96 <= angle < -0.61:
                front_right_rays.append(dist)

            # 3. DIRECT RIGHT: -105° to -75° (-1.83 to -1.31 rad)
            elif -1.83 <= angle <= -1.31:
                right_rays.append(dist)

            # 4. REAR-RIGHT: -145° to -115° (-2.53 to -2.01 rad)
            elif -2.53 <= angle < -2.01:
                rear_right_rays.append(dist)

            # 5. DIRECT LEFT: +75° to +105° (+1.31 to +1.83 rad)
            elif 1.31 <= angle <= 1.83:
                left_rays.append(dist)

        self.dist_front = min(front_rays) if front_rays else float('inf')
        self.dist_front_right = min(front_right_rays) if front_right_rays else float('inf')
        self.dist_right = min(right_rays) if right_rays else float('inf')
        self.dist_rear_right = min(rear_right_rays) if rear_right_rays else float('inf')
        self.dist_left = min(left_rays) if left_rays else float('inf')

        # Exponential Moving Average (EMA) filtering on right-side beams
        ALPHA = 0.35
        if not math.isinf(self.dist_right):
            if self.filt_dist_right is None:
                self.filt_dist_right = self.dist_right
            else:
                self.filt_dist_right = (1.0 - ALPHA) * self.filt_dist_right + ALPHA * self.dist_right
        else:
            self.filt_dist_right = float('inf')

        if not math.isinf(self.dist_front_right):
            if self.filt_dist_front_right is None:
                self.filt_dist_front_right = self.dist_front_right
            else:
                self.filt_dist_front_right = (1.0 - ALPHA) * self.filt_dist_front_right + ALPHA * self.dist_front_right
        else:
            self.filt_dist_front_right = float('inf')

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
            # Use small forward speed (0.05 m/s) with strong right rotation (-0.65 rad/s)
            # Turning radius R = v / w = 0.05 / 0.65 ≈ 7.7 cm!
            # The robot wraps tightly around the corner pivot and never reaches the opposite wall.
            if self.dist_front < self.FRONT_STOP_DIST:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.TURN_SPEED
            else:
                cmd.linear.x = 0.05   # 5 cm/s
                cmd.angular.z = -0.65 # Fast, tight right turn around corner

            # Exit condition: As soon as the new wall is captured on the right side
            if elapsed > 0.4 and self.dist_right < (self.TARGET_WALL_DIST + 0.08) and self.dist_front > self.FRONT_STOP_DIST:
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
            elif self.dist_right < (self.TARGET_WALL_DIST + 0.08):
                self.set_state(self.STATE_FOLLOW_WALL)
            else:
                cmd.linear.x = self.FORWARD_SPEED
                cmd.angular.z = -0.10  # Gentle curve towards right

        # =====================================================================
        # STATE 4: FOLLOW WALL (Stanley Cross-Track & Heading Controller)
        # =====================================================================
        else:
            # 1. Obstacle in front -> Pivot Left
            if self.dist_front < self.FRONT_STOP_DIST:
                self.set_state(self.STATE_PIVOT_INNER)
                cmd.linear.x = 0.0
                cmd.angular.z = self.TURN_SPEED
                action_name = "OBSTACLE IN FRONT -> Turn Left"

            # 2. Outer Corner: Right wall dropped off (> 0.32m)
            elif self.dist_right > self.WALL_LOST_DIST and self.dist_front_right > self.WALL_LOST_DIST:
                self.set_state(self.STATE_OUTER_CORNER)
                cmd.linear.x = 0.05
                cmd.angular.z = -0.65
                action_name = "OUTER CORNER -> Turning Right"

            # 3. Stanley Controller: Maintains CONSTANT 4 cm Distance (Smooth & Stable)
            else:
                right_d = self.filt_dist_right if self.filt_dist_right is not None else self.dist_right
                fr_d = self.filt_dist_front_right if self.filt_dist_front_right is not None else self.dist_front_right

                # 3a. Cross-track distance error (positive = closer than 4cm, negative = further)
                dist_error = self.TARGET_WALL_DIST - right_d

                # Deadband: if within deadband of target distance, suppress error to eliminate hunting
                if abs(dist_error) < self.DEADBAND_DIST:
                    dist_error = 0.0

                # 3b. Desired heading angle:
                # Soft gain: gentle slope towards wall (never aggressive)
                desired_heading = math.atan2(self.KP_DIST * dist_error, self.FORWARD_SPEED)
                desired_heading = max(-0.21, min(0.21, desired_heading))  # Clamp to ±12 degrees

                # 3c. Current heading angle relative to the wall:
                # When parallel: fr_d * cos(45°) ≈ right_d
                if fr_d < float('inf') and right_d < float('inf'):
                    diff = (fr_d * 0.7071) - right_d
                    actual_heading = math.atan2(diff, fr_d * 0.7071)
                    actual_heading = max(-0.35, min(0.35, actual_heading))
                else:
                    actual_heading = 0.0

                # 3d. Steering law: drives actual heading to desired heading
                heading_error = desired_heading - actual_heading
                target_wz = self.KP_HEAD * heading_error
                target_wz = max(-self.MAX_STEER_WZ, min(self.MAX_STEER_WZ, target_wz))

                # Narrow corridor left-wall safety guard
                if self.dist_left < (self.ROBOT_SIDE_OFFSET + 0.05) and target_wz > 0.0:
                    target_wz = min(target_wz, 0.02)

                # Slew rate limiter: smooth acceleration of angular velocity (no jerks)
                wz_diff = target_wz - self.prev_wz
                if abs(wz_diff) > self.MAX_WZ_ACCEL:
                    wz = self.prev_wz + math.copysign(self.MAX_WZ_ACCEL, wz_diff)
                else:
                    wz = target_wz
                self.prev_wz = wz

                cmd.linear.x = self.FORWARD_SPEED
                cmd.angular.z = wz
                action_name = f"STANLEY (e={dist_error*100:+.1f}cm, des={math.degrees(desired_heading):+.0f}°, act={math.degrees(actual_heading):+.0f}°, wz={wz:+.2f})"

        # Publish velocity command
        self.cmd_pub.publish(cmd)
        self.cmd_pub_backup.publish(cmd)

        # Telemetry line printed every second (10 scans)
        if self.scan_count % 10 == 0:
            actual_front_gap = max(0.0, self.dist_front - self.ROBOT_FRONT_OFFSET)
            actual_right_gap = max(0.0, self.dist_right - self.ROBOT_SIDE_OFFSET)
            print(
                f"[{self.state}] {action_name} | "
                f"Right: {actual_right_gap*100:.1f}cm | "
                f"Front: {actual_front_gap*100:.1f}cm | "
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
