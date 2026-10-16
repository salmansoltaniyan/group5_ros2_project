#!/usr/bin/env python3
"""
===============================================================================
Simple LiDAR-Based Wall Follower (Right-Hand Rule)
===============================================================================
A minimalist, standalone ROS 2 node to test wall following using ONLY the LiDAR.
No Odometry, no goal coordinates, no complex math.

How It Works:
-------------
1. Reads distances from the 2D LiDAR topic (/scan).
2. Looks at two key areas:
   - Front: Is there a wall directly in front of the robot?
   - Right: How far is the wall on the robot's right side?
3. Simple 3-step decision logic:
   - Case 1 (Obstacle in Front): STOP and turn LEFT in place.
   - Case 2 (Wall Lost / Outer Corner): Curve RIGHT to find/round the wall.
   - Case 3 (Following Wall): Drive forward while gently adjusting to stay 
             at the target distance (e.g., 25 cm from the right wall).
===============================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SimpleWallFollower(Node):
    def __init__(self):
        super().__init__('simple_wall_follower')

        # ---------------------------------------------------------------------
        # ROBOT PHYSICAL DIMENSIONS (Measured from LiDAR center to chassis edge)
        # ---------------------------------------------------------------------
        self.ROBOT_FRONT_OFFSET = 0.20   # Front bumper is 20 cm from LiDAR
        self.ROBOT_BACK_OFFSET  = 0.15   # Rear bumper is 15 cm from LiDAR
        self.ROBOT_SIDE_OFFSET  = 0.18   # Left/Right edges are 18 cm from LiDAR

        # ---------------------------------------------------------------------
        # TUNING PARAMETERS (Calculated with physical clearance in mind)
        # ---------------------------------------------------------------------
        # We want an actual 10 cm air gap between the robot's right edge and the wall:
        self.DESIRED_RIGHT_GAP = 0.10    # 10 cm physical air gap
        self.TARGET_WALL_DIST = self.ROBOT_SIDE_OFFSET + self.DESIRED_RIGHT_GAP  # = 0.28m from LiDAR

        # We need clearance for the 20 cm nose to pivot 90° without clipping the wall:
        self.DESIRED_FRONT_GAP = 0.20    # 20 cm physical air gap in front
        self.FRONT_STOP_DIST = self.ROBOT_FRONT_OFFSET + self.DESIRED_FRONT_GAP  # = 0.40m from LiDAR

        # If right wall is further than chassis + 27 cm, consider wall lost (corner/opening):
        self.WALL_LOST_DIST = self.ROBOT_SIDE_OFFSET + 0.27                      # = 0.45m from LiDAR
        
        # Diagonal corner threshold (Chassis corner radius is sqrt(0.20^2 + 0.18^2) ≈ 0.27m)
        self.CORNER_SAFETY_DIST = 0.32   # Prevent front-right bumper from clipping inner corners

        self.FORWARD_SPEED = 0.14        # Smooth forward speed (m/s)
        self.TURN_SPEED = 0.45           # Turning rotation speed (rad/s)
        self.KP = 2.2                    # Proportional steering gain

        # ---------------------------------------------------------------------
        # SENSOR DISTANCES (in meters)
        # ---------------------------------------------------------------------
        self.dist_front = float('inf')
        self.dist_front_right = float('inf')
        self.dist_right = float('inf')
        self.dist_rear_right = float('inf')  # Critical for tracking thin wall corners!
        self.scan_count = 0

        # ---------------------------------------------------------------------
        # ROS 2 COMMUNICATION
        # ---------------------------------------------------------------------
        # Velocity publisher for Robot 9
        self.cmd_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)

        # LiDAR subscriber (Best Effort QoS matches RPLiDAR)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        
        # Backup subscriber in case LiDAR is namespaced
        self.scan_sub_backup = self.create_subscription(LaserScan, '/robot_9/scan', self.scan_callback, sensor_qos)

        # 10 Hz Control Loop Timer (runs every 100 ms)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('>>> Simple Wall Follower initialized! Waiting for LiDAR (/scan)...')

    def normalize_angle(self, angle: float) -> float:
        """Wraps angle between [-pi, +pi] radians."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def scan_callback(self, msg: LaserScan):
        """Extracts Front, Front-Right, Direct-Right, and Rear-Right distances."""
        self.scan_count += 1
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return

        front_rays = []
        front_right_rays = []
        right_rays = []
        rear_right_rays = []

        for i, dist in enumerate(msg.ranges):
            # Discard invalid readings (inf, nan, zeros, and blind spots)
            if math.isnan(dist) or math.isinf(dist) or dist <= msg.range_min or dist <= 0.05:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)
            angle = self.normalize_angle(angle)

            # 1. Front: -25° to +25° (-0.44 to +0.44 rad)
            if -0.44 <= angle <= 0.44:
                front_rays.append(dist)

            # 2. Front-Right Diagonal: -60° to -25° (-1.05 to -0.44 rad)
            elif -1.05 <= angle < -0.44:
                front_right_rays.append(dist)

            # 3. Direct Right: -110° to -60° (-1.92 to -1.05 rad)
            elif -1.92 <= angle < -1.05:
                right_rays.append(dist)

            # 4. Rear-Right: -150° to -110° (-2.62 to -1.92 rad)
            elif -2.62 <= angle < -1.92:
                rear_right_rays.append(dist)

        # Store the closest obstacle distance per sector
        self.dist_front = min(front_rays) if front_rays else float('inf')
        self.dist_front_right = min(front_right_rays) if front_right_rays else float('inf')
        self.dist_right = min(right_rays) if right_rays else float('inf')
        self.dist_rear_right = min(rear_right_rays) if rear_right_rays else float('inf')

    def control_loop(self):
        """10 Hz reactive decision-making loop."""
        # Wait until at least one LiDAR scan is received
        if self.scan_count == 0:
            self.get_logger().warn('Waiting for first LiDAR scan...', throttle_duration_sec=3.0)
            return

        cmd = Twist()
        action_name = ""

        # ---------------------------------------------------------------------
        # BEHAVIOR 1: Obstacle directly ahead (or front-right corner close) -> Pivot LEFT
        # ---------------------------------------------------------------------
        if self.dist_front < self.FRONT_STOP_DIST:
            cmd.linear.x = 0.0
            cmd.angular.z = self.TURN_SPEED
            action_name = "PIVOT LEFT (Wall Ahead!)"

        elif self.dist_front_right < self.CORNER_SAFETY_DIST:
            # Front-right bumper corner is getting too close to a wall (< 5cm clearance)
            cmd.linear.x = self.FORWARD_SPEED * 0.5
            cmd.angular.z = self.TURN_SPEED * 0.6
            action_name = "AVOID CORNER (Steering Left)"

        # ---------------------------------------------------------------------
        # BEHAVIOR 2: Right wall ended (Thin 11cm edge / Outer corner)
        # ---------------------------------------------------------------------
        elif self.dist_right > self.WALL_LOST_DIST and self.dist_front_right > self.WALL_LOST_DIST:
            # Check if the corner lip is still alongside our rear wheels
            if self.dist_rear_right < (self.ROBOT_SIDE_OFFSET + 0.18):
                # The 11 cm corner is beside the back wheels. Creep forward slowly
                # so the wheel axle clears the corner before spinning.
                cmd.linear.x = 0.07
                cmd.angular.z = -0.25
                action_name = "CLEARING 11cm LIP (Creep Forward)"
            else:
                # The wheel axle is safely past the corner.
                # Turn with a very tight radius (R ≈ 8-10 cm) to wrap tightly around the 11cm edge!
                cmd.linear.x = 0.04
                cmd.angular.z = -0.50
                action_name = "HUGGING 11cm CORNER (Tight Right Arc)"

        # ---------------------------------------------------------------------
        # BEHAVIOR 3: Following Wall -> Proportional controller
        # ---------------------------------------------------------------------
        else:
            # error > 0: Robot is closer than 10cm air gap -> Steer left (+z)
            # error < 0: Robot is further than 10cm air gap -> Steer right (-z)
            error = self.TARGET_WALL_DIST - self.dist_right
            cmd.linear.x = self.FORWARD_SPEED
            cmd.angular.z = max(-0.5, min(0.5, self.KP * error))
            action_name = "FOLLOWING WALL (P-Control)"

        # Publish velocity command
        self.cmd_pub.publish(cmd)

        # Print clean status line with physical air gaps every second
        if self.scan_count % 10 == 0:
            actual_front_gap = max(0.0, self.dist_front - self.ROBOT_FRONT_OFFSET)
            actual_right_gap = max(0.0, self.dist_right - self.ROBOT_SIDE_OFFSET)
            print(
                f"[STATUS: {action_name}] | "
                f"Front Gap: {actual_front_gap*100:.0f}cm (LiDAR={self.dist_front:.2f}m) | "
                f"Right Gap: {actual_right_gap*100:.0f}cm (LiDAR={self.dist_right:.2f}m) | "
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

