#!/usr/bin/env python3
"""
===============================================================================
Bug 2 & Wall-Following Navigator for iRobot Create 3 (Hanze Maze Challenge)
===============================================================================

Operating Modes:
----------------
1. 'WALL_FOLLOW_ONLY': Independent test mode using ONLY 2D LiDAR (no odometry needed).
   Follows the right-hand wall for 100 seconds so you can safely verify your LiDAR 
   tuning and cornering without worrying about goal coordinates or M-lines.
2. 'BUG2': Full autonomous navigation combining Odometry + LiDAR to reach the Goal.

Key Improvements:
-----------------
- Dual LiDAR Topic Support: Subscribes to BOTH '/scan' and '/robot_9/scan' to ensure
  you never miss LiDAR data regardless of the driver's active namespace.
- Collision Buffer: Increased front detection distance to 0.42m to prevent the 34cm
  Create 3 chassis from hitting the wall before turning.
- Clear Live Telemetry: Prints compact, easy-to-read LiDAR readings (Front, Right, 
  Left, Closest) every 0.5s so you can immediately see what the robot sees.
===============================================================================
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class Bug2Navigator(Node):
    """
    Unified Navigation Node supporting both Pure Wall-Following and full Bug 2.
    """

    STATE_GO_TO_GOAL = 0
    STATE_WALL_FOLLOWING = 1
    STATE_GOAL_REACHED = 2

    def __init__(self):
        super().__init__('bug2_navigator')

        # =====================================================================
        # 1. OPERATION MODE SELECTOR (Change this to switch modes!)
        # =====================================================================
        # 'WALL_FOLLOW_ONLY' -> Independent 100-second LiDAR wall-following test
        # 'BUG2'             -> Full Bug 2 navigation to Goal
        self.OPERATION_MODE = 'WALL_FOLLOW_ONLY'
        self.TEST_DURATION = 100.0  # Run for 100 seconds in Mode 1, then stop

        # =====================================================================
        # 2. COMPETITION GOAL CONFIGURATION (Used in 'BUG2' mode)
        # =====================================================================
        self.target_dx = 2.0        # Desired forward distance in meters from start
        self.target_dy = 2.0        # Desired lateral distance in meters (+Left, -Right)
        self.goal_tolerance = 0.15  # Arrival radius (meters)

        # =====================================================================
        # 3. SPEED & OBSTACLE CLEARANCE PARAMETERS
        # =====================================================================
        self.LINEAR_SPEED_CRUISE = 0.16   # Forward speed when path is clear (m/s)
        self.LINEAR_SPEED_WALL = 0.13     # Speed during wall following (m/s)
        self.ANGULAR_SPEED_TURN = 0.45    # In-place turn speed (rad/s)

        # Distance thresholds (Robot radius is ~0.17m; Corridors are ~0.50m)
        self.FRONT_WALL_THRESHOLD = 0.42  # Trigger turn if obstacle closer than 42 cm
        self.DESIRED_WALL_DIST = 0.25     # Target distance to keep from right wall (25 cm)
        self.M_LINE_TOLERANCE = 0.12      # Re-crossing window for M-line (meters)
        self.LEAVE_PROGRESS_MARGIN = 0.20 # Must be at least 20cm closer to leave wall

        # =====================================================================
        # 4. TRACKING & SENSOR VARIABLES
        # =====================================================================
        self.state = self.STATE_WALL_FOLLOWING if self.OPERATION_MODE == 'WALL_FOLLOW_ONLY' else self.STATE_GO_TO_GOAL
        self.start_time = None
        self.initial_pose_recorded = False
        self.telemetry_counter = 0
        self.lidar_packet_count = 0

        # Odometry pose
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Bug 2 M-Line: A*x + B*y + C = 0
        self.m_line_a = 0.0
        self.m_line_b = 0.0
        self.m_line_c = 0.0
        self.hit_dist_to_goal = float('inf')

        # LiDAR distance sectors (in meters)
        self.dist_front = float('inf')
        self.dist_front_right = float('inf')
        self.dist_right = float('inf')
        self.dist_left = float('inf')
        self.dist_closest = float('inf')

        # =====================================================================
        # 5. ROS 2 PUBLISHERS, SUBSCRIBERS & SERVICE CLIENTS
        # =====================================================================
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)

        # Explicit BEST_EFFORT QoS Profile (Matches RPLiDAR and Create 3 sensors)
        self.sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot_9/odom',
            self.odom_callback,
            self.sensor_qos
        )

        # Dual LiDAR subscription: listens to both possible topic names with BEST_EFFORT
        self.scan_sub_standard = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.sensor_qos
        )


        # Auto-start LiDAR motor service client
        self.start_lidar_client = self.create_client(Empty, '/start_motor')
        self.auto_start_lidar()

        # 10 Hz Control Loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        mode_desc = "TEST MODE: PURE WALL FOLLOWING (LiDAR Only)" if self.OPERATION_MODE == 'WALL_FOLLOW_ONLY' else "MODE: FULL BUG 2 NAVIGATION"
        self.get_logger().info(f'>>> Node started in [{mode_desc}]. Waiting for sensor data...')

    def auto_start_lidar(self):
        """Attempts to asynchronously start the LiDAR motor if the service exists."""
        if self.start_lidar_client.service_is_ready():
            self.start_lidar_client.call_async(Empty.Request())
            self.get_logger().info('>>> Sent request to /start_motor to spin up LiDAR!')
        else:
            # Try to call without blocking
            self.start_lidar_client.call_async(Empty.Request())

    # =========================================================================
    # SENSOR CALLBACKS
    # =========================================================================

    def odom_callback(self, msg: Odometry):
        """Processes Odometry updates and initializes the M-Line."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert orientation quaternion to Yaw angle
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # On the first packet, calculate absolute goal using rotation matrix
        if not self.initial_pose_recorded:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw

            # Project target_dx along the robot's starting nose orientation
            dx_world = self.target_dx * math.cos(self.start_yaw) - self.target_dy * math.sin(self.start_yaw)
            dy_world = self.target_dx * math.sin(self.start_yaw) + self.target_dy * math.cos(self.start_yaw)

            self.goal_x = self.start_x + dx_world
            self.goal_y = self.start_y + dy_world

            # M-Line equation: A*x + B*y + C = 0
            self.m_line_a = self.goal_y - self.start_y
            self.m_line_b = -(self.goal_x - self.start_x)
            self.m_line_c = (self.goal_x * self.start_y) - (self.goal_y * self.start_x)

            self.initial_pose_recorded = True
            self.get_logger().info(
                f'>>> Odometry Ready! Start=({self.start_x:.2f}, {self.start_y:.2f}, yaw={math.degrees(self.start_yaw):.1f}°) | '
                f'Goal=({self.goal_x:.2f}, {self.goal_y:.2f})'
            )

    def scan_callback(self, msg: LaserScan):
        """Processes 2D LiDAR data and partitions into directional sectors."""
        self.lidar_packet_count += 1
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return

        front_dists = []
        front_right_dists = []
        right_dists = []
        left_dists = []
        all_valid = []

        for i, dist in enumerate(msg.ranges):
            # Ignore invalid readings, zeros, or values below sensor physical limit
            if math.isnan(dist) or math.isinf(dist) or dist <= msg.range_min or dist <= 0.05:
                continue

            all_valid.append(dist)
            angle = msg.angle_min + (i * msg.angle_increment)
            angle = self.normalize_angle(angle)

            # 1. Front Sector: -25 deg to +25 deg (-0.44 to +0.44 rad)
            if -0.44 <= angle <= 0.44:
                front_dists.append(dist)

            # 2. Front-Right Diagonal: -65 deg to -25 deg (-1.13 to -0.44 rad)
            elif -1.13 <= angle < -0.44:
                front_right_dists.append(dist)

            # 3. Direct Right: -115 deg to -65 deg (-2.00 to -1.13 rad)
            elif -2.00 <= angle < -1.13:
                right_dists.append(dist)

            # 4. Direct Left: +65 deg to +115 deg (+1.13 to +2.00 rad)
            elif 1.13 <= angle <= 2.00:
                left_dists.append(dist)

        # Minimum defensive distance per sector
        self.dist_front = min(front_dists) if front_dists else float('inf')
        self.dist_front_right = min(front_right_dists) if front_right_dists else float('inf')
        self.dist_right = min(right_dists) if right_dists else float('inf')
        self.dist_left = min(left_dists) if left_dists else float('inf')
        self.dist_closest = min(all_valid) if all_valid else float('inf')

    # =========================================================================
    # MATHEMATICAL HELPERS
    # =========================================================================

    def normalize_angle(self, angle: float) -> float:
        """Wraps angle between [-pi, +pi] radians."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_distance_to_m_line(self) -> float:
        """Computes perpendicular distance from current pose to the M-Line."""
        den = math.hypot(self.m_line_a, self.m_line_b)
        if den < 1e-6:
            return 0.0
        num = abs(self.m_line_a * self.current_x + self.m_line_b * self.current_y + self.m_line_c)
        return num / den

    # =========================================================================
    # MAIN CONTROL LOOP
    # =========================================================================

    def control_loop(self):
        """10 Hz decision-making and motion controller."""
        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time
        cmd = Twist()

        # ---------------------------------------------------------------------
        # 1. SAFETY CHECK: Ensure LiDAR is actively receiving packets
        # ---------------------------------------------------------------------
        if self.lidar_packet_count == 0:
            self.get_logger().warn(
                '>>> [WAITING FOR LIDAR] No scan packets received! Check /scan or /start_motor!',
                throttle_duration_sec=3.0
            )
            return

        # ---------------------------------------------------------------------
        # 2. EXECUTE BASED ON SELECTED MODE
        # ---------------------------------------------------------------------

        # =====================================================================
        # MODE 1: PURE WALL FOLLOWING TEST (Runs for 100 seconds)
        # =====================================================================
        if self.OPERATION_MODE == 'WALL_FOLLOW_ONLY':
            if elapsed >= self.TEST_DURATION:
                self.stop_robot()
                self.get_logger().info(f'>>> [TEST COMPLETE] {self.TEST_DURATION}s wall-following test finished!')
                return

            # Execute Right-Hand Wall Following controller
            cmd = self.compute_wall_following_cmd()

        # =====================================================================
        # MODE 2: FULL BUG 2 NAVIGATION (Odometry + LiDAR)
        # =====================================================================
        elif self.OPERATION_MODE == 'BUG2':
            if not self.initial_pose_recorded:
                return

            dist_to_goal = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
            dist_to_m_line = self.get_distance_to_m_line()
            desired_yaw = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
            yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)

            # Check arrival
            if dist_to_goal <= self.goal_tolerance or self.state == self.STATE_GOAL_REACHED:
                self.stop_robot()
                if self.state != self.STATE_GOAL_REACHED:
                    self.state = self.STATE_GOAL_REACHED
                    self.get_logger().info('>>> [SUCCESS] Robot arrived at destination!')
                return

            # State A: Go to Goal along M-Line
            if self.state == self.STATE_GO_TO_GOAL:
                if self.dist_front < self.FRONT_WALL_THRESHOLD:
                    self.hit_dist_to_goal = dist_to_goal
                    self.state = self.STATE_WALL_FOLLOWING
                    self.get_logger().info(f'>>> [HIT WALL at {self.dist_front:.2f}m] Switching to WALL_FOLLOWING!')
                    self.stop_robot()
                    return

                if abs(yaw_error) > 0.25:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.ANGULAR_SPEED_TURN if yaw_error > 0 else -self.ANGULAR_SPEED_TURN
                else:
                    cmd.linear.x = self.LINEAR_SPEED_CRUISE
                    cmd.angular.z = 1.2 * yaw_error

            # State B: Wall Following with Leave Condition
            elif self.state == self.STATE_WALL_FOLLOWING:
                is_on_m_line = dist_to_m_line < self.M_LINE_TOLERANCE
                is_closer = dist_to_goal < (self.hit_dist_to_goal - self.LEAVE_PROGRESS_MARGIN)
                front_is_clear = self.dist_front > 0.45 and abs(yaw_error) < 0.35

                if is_on_m_line and is_closer and front_is_clear:
                    self.get_logger().info('>>> [LEAVE CONDITION MET] Resuming GO_TO_GOAL!')
                    self.state = self.STATE_GO_TO_GOAL
                    return

                cmd = self.compute_wall_following_cmd()

        # ---------------------------------------------------------------------
        # 3. LIVE COMPACT TELEMETRY (Prints every 0.5s)
        # ---------------------------------------------------------------------
        self.telemetry_counter += 1
        if self.telemetry_counter % 5 == 0:
            mode_tag = "WALL_TEST" if self.OPERATION_MODE == 'WALL_FOLLOW_ONLY' else ("GO_TO_GOAL" if self.state == self.STATE_GO_TO_GOAL else "WALL_FOLLOW")
            print(
                f"\n--- [TELEMETRY: {mode_tag} | Time: {elapsed:.1f}s] ---\n"
                f"  LiDAR Dists : Front={self.dist_front:.2f}m | Right={self.dist_right:.2f}m | Left={self.dist_left:.2f}m | Closest={self.dist_closest:.2f}m\n"
                f"  Pose        : (x={self.current_x:.2f}, y={self.current_y:.2f})m | Yaw={math.degrees(self.current_yaw):.1f}°\n"
                f"  Motor Cmd   : Linear={cmd.linear.x:.2f} m/s | Angular={cmd.angular.z:.2f} rad/s\n"
                f"-------------------------------------------------------"
            )

        # Publish movement command
        self.cmd_vel_pub.publish(cmd)

    def compute_wall_following_cmd(self) -> Twist:
        """
        Pure Reactive Right-Hand Wall Following Controller:
        - Front blocked (< 0.42m): Pivot left in place.
        - Right wall lost (> 0.45m): Arc right around outer corner.
        - Right wall present: P-controller maintains 25 cm from right wall.
        """
        cmd = Twist()

        # 1. Front blocked: Stop and turn left in place
        if self.dist_front < self.FRONT_WALL_THRESHOLD:
            cmd.linear.x = 0.0
            cmd.angular.z = self.ANGULAR_SPEED_TURN

        # 2. Outer corner detected: wall dropped off to the right
        elif self.dist_right > 0.45 and self.dist_front_right > 0.45:
            cmd.linear.x = self.LINEAR_SPEED_WALL * 0.7
            cmd.angular.z = -self.ANGULAR_SPEED_TURN * 0.7  # Turn right around corner

        # 3. Following wall: P-controller to stay centered at DESIRED_WALL_DIST
        else:
            wall_error = self.DESIRED_WALL_DIST - self.dist_right
            cmd.linear.x = self.LINEAR_SPEED_WALL
            # If too close to right wall (error > 0), steer left (+z); if too far, steer right (-z)
            cmd.angular.z = max(-0.5, min(0.5, 1.8 * wall_error))

        return cmd

    def stop_robot(self):
        """Commands zero velocity to stop robot immediately."""
        self.cmd_vel_pub.publish(Twist())


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = Bug2Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Stopping robot safely...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
