#!/usr/bin/env python3
"""
===============================================================================
Robust LiDAR Wall Follower for Narrow Corridors (Right-Hand Rule)
===============================================================================
Specially tuned for narrow corridors (~0.50m) and tight corners:
- Anti-Stuck & Anti-Collision:
    * In-place pivots (linear.x = 0.0) ensure the robot never swings into opposite walls.
    * Automatic Recovery: If the front bumper gets too close (< 6 cm), the robot 
      automatically reverses straight back to clear its bumper before rotating.
- Outer Corner Handling:
    * Step 1 (CORNER_PASS): Creeps straight forward just enough to clear the 11cm lip.
    * Step 2 (CORNER_TURN_RIGHT): Pivots in place to the right until the new wall is caught.
- PD Wall Following:
    * Tight 6 cm air gap from right wall with heading angle damping (no snaking/wobbling).
    * Left-wall guard prevents scraping opposite corridor walls.
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
    STATE_PIVOT_LEFT = "PIVOT_LEFT"
    STATE_CORNER_PASS = "CORNER_PASS"
    STATE_CORNER_TURN_RIGHT = "CORNER_TURN_RIGHT"
    STATE_RECOVERY = "RECOVERY"
    STATE_FIND_WALL = "FIND_WALL"

    def __init__(self):
        super().__init__('simple_wall_follower')

        # ---------------------------------------------------------------------
        # ROBOT PHYSICAL DIMENSIONS (Measured from LiDAR center to chassis edge)
        # ---------------------------------------------------------------------
        self.ROBOT_FRONT_OFFSET = 0.20   # Front bumper is 20 cm from LiDAR
        self.ROBOT_BACK_OFFSET  = 0.15   # Rear bumper is 15 cm from LiDAR
        self.ROBOT_SIDE_OFFSET  = 0.18   # Left/Right edges are 18 cm from LiDAR

        # ---------------------------------------------------------------------
        # TUNING PARAMETERS (Optimized for narrow ~50cm corridors & tight wall hugging)
        # ---------------------------------------------------------------------
        # 1. Right wall distance (6 cm physical air gap between right edge and wall):
        self.DESIRED_RIGHT_GAP = 0.06    # 6 cm air gap (chassis hugs wall closely)
        self.TARGET_WALL_DIST = self.ROBOT_SIDE_OFFSET + self.DESIRED_RIGHT_GAP  # = 0.24m from LiDAR

        # 2. Emergency collision buffer (If front bumper is within 6 cm of wall, reverse!):
        self.COLLISION_BUFFER = self.ROBOT_FRONT_OFFSET + 0.06                   # = 0.26m from LiDAR

        # 3. Front obstacle threshold (Inner corner / Wall Ahead):
        # Stop at 16 cm clearance so the front bumper never touches the front wall when rotating
        self.DESIRED_FRONT_GAP = 0.16    # 16 cm air gap in front
        self.FRONT_STOP_DIST = self.ROBOT_FRONT_OFFSET + self.DESIRED_FRONT_GAP  # = 0.36m from LiDAR
        self.FRONT_CLEAR_DIST = self.ROBOT_FRONT_OFFSET + 0.24                   # = 0.44m to resume

        # 4. Outer corner detection (Wall drops off to the right):
        # If right side is further than chassis + 14 cm (0.32m), wall has ended
        self.WALL_LOST_DIST = self.ROBOT_SIDE_OFFSET + 0.14                      # = 0.32m from LiDAR
        
        # 5. Front-right corner safety clearance:
        self.CORNER_SAFETY_DIST = 0.28   # ~10 cm clearance from front-right bumper

        # 6. Speeds & Controller Gains:
        self.FORWARD_SPEED = 0.12        # Cruise speed during straight wall following (m/s)
        self.TURN_SPEED = 0.48           # In-place pivot speed (rad/s)
        self.KP = 2.2                    # Proportional gain for distance error
        self.KD = 3.0                    # Derivative gain for heading alignment

        # ---------------------------------------------------------------------
        # SENSOR DISTANCES & STATE TRACKING
        # ---------------------------------------------------------------------
        self.dist_front = float('inf')
        self.dist_front_right = float('inf')
        self.dist_right = float('inf')
        self.dist_rear_right = float('inf')
        self.dist_left = float('inf')
        self.dist_45 = float('inf')
        self.dist_90 = float('inf')

        self.state = self.STATE_FOLLOW_WALL
        self.state_start_time = self.get_clock().now()
        self.scan_count = 0

        # ---------------------------------------------------------------------
        # ROS 2 COMMUNICATION
        # ---------------------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.scan_sub_backup = self.create_subscription(LaserScan, '/robot_9/scan', self.scan_callback, sensor_qos)

        # 10 Hz Control Loop Timer (runs every 100 ms)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('>>> Robust Wall Follower initialized! Waiting for LiDAR (/scan)...')

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
        """Extracts directional distances and key heading angles (-45° and -90°)."""
        self.scan_count += 1
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return

        front_rays = []
        front_right_rays = []
        right_rays = []
        rear_right_rays = []
        left_rays = []
        rays_45 = []
        rays_90 = []

        for i, dist in enumerate(msg.ranges):
            if math.isnan(dist) or math.isinf(dist) or dist <= msg.range_min or dist <= 0.05:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)
            angle = self.normalize_angle(angle)

            # 1. Front Sector: -18° to +18° (-0.31 to +0.31 rad)
            if -0.31 <= angle <= 0.31:
                front_rays.append(dist)

            # 2. Front-Right Diagonal: -65° to -25° (-1.13 to -0.44 rad)
            elif -1.13 <= angle < -0.44:
                front_right_rays.append(dist)

            # 3. Direct Right: -110° to -70° (-1.92 to -1.22 rad)
            elif -1.92 <= angle <= -1.22:
                right_rays.append(dist)

            # 4. Rear-Right: -150° to -110° (-2.62 to -1.92 rad)
            elif -2.62 <= angle < -1.92:
                rear_right_rays.append(dist)

            # 5. Direct Left: +70° to +110° (+1.22 to +1.92 rad)
            elif 1.22 <= angle <= 1.92:
                left_rays.append(dist)

            # Precise angles for heading alignment (PD control):
            # Around -45° (-0.85 to -0.72 rad)
            if -0.85 <= angle <= -0.72:
                rays_45.append(dist)
            # Around -90° (-1.64 to -1.50 rad)
            if -1.64 <= angle <= -1.50:
                rays_90.append(dist)

        self.dist_front = min(front_rays) if front_rays else float('inf')
        self.dist_front_right = min(front_right_rays) if front_right_rays else float('inf')
        self.dist_right = min(right_rays) if right_rays else float('inf')
        self.dist_rear_right = min(rear_right_rays) if rear_right_rays else float('inf')
        self.dist_left = min(left_rays) if left_rays else float('inf')

        self.dist_45 = min(rays_45) if rays_45 else float('inf')
        self.dist_90 = min(rays_90) if rays_90 else self.dist_right

    def control_loop(self):
        """10 Hz reactive decision-making loop."""
        if self.scan_count == 0:
            self.get_logger().warn('Waiting for first LiDAR scan...', throttle_duration_sec=3.0)
            return

        cmd = Twist()
        action_name = ""
        elapsed = self.get_state_elapsed()

        # =====================================================================
        # SAFETY OVERRIDE: Active Recovery / Unsticking
        # If the front bumper ever gets dangerously close (< 6cm) to a wall,
        # automatically back up straight to clear space before rotating!
        # =====================================================================
        if self.dist_front < self.COLLISION_BUFFER and self.state != self.STATE_RECOVERY:
            self.set_state(self.STATE_RECOVERY)

        # =====================================================================
        # STATE 1: RECOVERY (Reverse Straight Back to Clear Trapped Bumper)
        # =====================================================================
        if self.state == self.STATE_RECOVERY:
            cmd.linear.x = -0.07  # Reverse slowly
            cmd.angular.z = 0.0
            action_name = "RECOVERY (Reversing to Free Bumper)"

            # Once front clearance is restored or 0.7s passed, pivot left safely
            if self.dist_front > (self.ROBOT_FRONT_OFFSET + 0.14) or elapsed > 0.7:
                self.set_state(self.STATE_PIVOT_LEFT)

        # =====================================================================
        # STATE 2: PIVOT LEFT (Inner Corner / Wall Ahead)
        # =====================================================================
        elif self.state == self.STATE_PIVOT_LEFT:
            # Pivot STRICTLY IN PLACE (linear.x = 0.0) so robot footprint never advances into wall
            cmd.linear.x = 0.0
            cmd.angular.z = self.TURN_SPEED
            action_name = "PIVOT LEFT (In-Place Turn)"

            # Exit condition: Front has opened up and right side begins seeing wall
            if self.dist_front > self.FRONT_CLEAR_DIST and self.dist_front_right > self.CORNER_SAFETY_DIST:
                self.set_state(self.STATE_FOLLOW_WALL)

        # =====================================================================
        # STATE 3: CORNER PASS (Outer corner: wheel axle clearing corner)
        # =====================================================================
        elif self.state == self.STATE_CORNER_PASS:
            # Creep straight forward just enough (~10-12 cm) for wheels to pass corner edge
            cmd.linear.x = 0.08
            cmd.angular.z = 0.0
            action_name = "CORNER PASS (Clearing Axle)"

            # If front obstacle approaches, stop forward motion immediately
            if self.dist_front < self.FRONT_STOP_DIST:
                self.set_state(self.STATE_CORNER_TURN_RIGHT)
            # Wheels have cleared the corner edge after 1.2s or when rear-right clears
            elif elapsed > 1.2 or (elapsed > 0.6 and self.dist_rear_right > self.WALL_LOST_DIST):
                self.set_state(self.STATE_CORNER_TURN_RIGHT)

        # =====================================================================
        # STATE 4: CORNER TURN RIGHT (Tight In-Place Pivot around Corner)
        # =====================================================================
        elif self.state == self.STATE_CORNER_TURN_RIGHT:
            # Pivot IN PLACE (linear.x = 0.0) to wrap around corner without swinging into opposite wall!
            cmd.linear.x = 0.0
            cmd.angular.z = -self.TURN_SPEED
            action_name = "CORNER TURN RIGHT (In-Place Pivot Right)"

            # Exit condition: Wall is re-acquired on the right side and front is open
            if self.dist_right < (self.TARGET_WALL_DIST + 0.08) and self.dist_front > self.FRONT_STOP_DIST:
                self.set_state(self.STATE_FOLLOW_WALL)
            # Timeout guard: If no wall found after 4.0s, seek wall
            elif elapsed > 4.0:
                self.set_state(self.STATE_FIND_WALL)

        # =====================================================================
        # STATE 5: FIND WALL (Seek right wall if lost in open area)
        # =====================================================================
        elif self.state == self.STATE_FIND_WALL:
            action_name = "FIND WALL (Seeking Right Wall)"
            if self.dist_front < self.FRONT_STOP_DIST:
                self.set_state(self.STATE_PIVOT_LEFT)
            elif self.dist_right < self.WALL_LOST_DIST:
                self.set_state(self.STATE_FOLLOW_WALL)
            else:
                cmd.linear.x = 0.09
                cmd.angular.z = -0.22

        # =====================================================================
        # STATE 6: FOLLOW WALL (Normal PD Wall Following)
        # =====================================================================
        else:
            # 1. Obstacle directly ahead -> Pivot Left
            if self.dist_front < self.FRONT_STOP_DIST:
                self.set_state(self.STATE_PIVOT_LEFT)
                cmd.linear.x = 0.0
                cmd.angular.z = self.TURN_SPEED
                action_name = "OBSTACLE AHEAD -> Pivot Left"

            # 2. Outer corner detected: Right wall dropped off
            elif self.dist_right > self.WALL_LOST_DIST and self.dist_front_right > self.WALL_LOST_DIST:
                self.set_state(self.STATE_CORNER_PASS)
                cmd.linear.x = 0.08
                cmd.angular.z = 0.0
                action_name = "OUTER CORNER -> Creeping Past Lip"

            # 3. Front-right bumper safety (gently steer away from angled walls)
            elif self.dist_front_right < self.CORNER_SAFETY_DIST:
                cmd.linear.x = self.FORWARD_SPEED * 0.5
                cmd.angular.z = self.TURN_SPEED * 0.6
                action_name = "AVOID CORNER (Steering Left)"

            # 4. Standard Wall Following with PD Controller (Distance + Heading Angle)
            else:
                # Distance error (positive = too close to right wall, steer left)
                error_dist = self.TARGET_WALL_DIST - self.dist_right

                # Heading alignment error (positive = nose tilted away from wall, steer right)
                if self.dist_45 < float('inf') and self.dist_90 < float('inf'):
                    heading_diff = (self.dist_45 * 0.7071) - self.dist_90
                else:
                    heading_diff = 0.0

                # PD Control Law:
                wz = (self.KP * error_dist) - (self.KD * heading_diff)
                wz = max(-0.50, min(0.50, wz))

                # Narrow corridor left-wall safety guard:
                if self.dist_left < (self.ROBOT_SIDE_OFFSET + 0.05) and wz > 0.0:
                    wz = min(wz, 0.02)

                # Slow down slightly if steering adjustment is large
                linear_x = self.FORWARD_SPEED if abs(wz) < 0.25 else (self.FORWARD_SPEED * 0.75)

                cmd.linear.x = linear_x
                cmd.angular.z = wz
                action_name = f"FOLLOW WALL (PD: e={error_dist*100:+.0f}cm, wz={wz:+.2f})"

        # Publish velocity command
        self.cmd_pub.publish(cmd)

        # Print telemetry every second (10 scans)
        if self.scan_count % 10 == 0:
            actual_front_gap = max(0.0, self.dist_front - self.ROBOT_FRONT_OFFSET)
            actual_right_gap = max(0.0, self.dist_right - self.ROBOT_SIDE_OFFSET)
            actual_left_gap = max(0.0, self.dist_left - self.ROBOT_SIDE_OFFSET)
            print(
                f"[{self.state}] {action_name} | "
                f"R-Gap: {actual_right_gap*100:.1f}cm | "
                f"F-Gap: {actual_front_gap*100:.1f}cm | "
                f"L-Gap: {actual_left_gap*100:.1f}cm | "
                f"Cmd: (vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f})"
            )

    def stop_robot(self):
        """Halts the robot."""
        self.cmd_pub.publish(Twist())


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
