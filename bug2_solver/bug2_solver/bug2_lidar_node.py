#!/usr/bin/env python3
"""
================================================================================
Bug2 Navigator (LiDAR variant) for iRobot Create 3  -  ROS 2 Jazzy
================================================================================

Uses 2D LiDAR (/scan) for obstacle detection and wall following.
Odometry for position tracking and Bug2 M-line logic.

ROBOT BODY & LiDAR PLACEMENT
-----------------------------
  Robot chassis: 34 cm × 34 cm (square)
  LiDAR offset from body edges:
    Front: 20 cm    Back: 14 cm    Left/Right: 16 cm each
  Diagonal LiDAR → front-right corner: √(20² + 16²) ≈ 25.6 cm

  All distance thresholds are measured FROM THE LiDAR, not from the robot edge.
  When tuning, remember to add ~20 cm for front clearance, ~16 cm for side clearance.

LiDAR SECTORS (6 regions, 0° = straight ahead, CCW positive)
─────────────────────────────────────────────────────────────

                          FRONT
                      -30° to +30°
                        ╲  ↑  ╱
            FRONT_LEFT   ╲ | ╱   FRONT_RIGHT
            +30° to +70°  ╲|╱    -70° to -30°
                ──────────[●]──────────
            LEFT         ╱|╲          RIGHT
            +70° to +120°╱ | ╲        -120° to -70°
                        ╱  |  ╲
            REAR_LEFT  ╱   |   ╲  REAR_RIGHT
            +120°-160°╱    ↓    ╲ -160° to -120°

  Each sector reports its MINIMUM valid range reading.

  Key sectors for right-hand wall following:
    FRONT        → obstacle avoidance (stop & turn left)
    FRONT_RIGHT  → CORNER PROTECTION (prevents diagonal collisions)
    RIGHT        → wall tracking (proportional distance control)
    REAR_RIGHT   → outer corner detection (wall ending behind robot)
================================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor


# ─── State constants ──────────────────────────────────────────────────────────
GO_TO_GOAL    = 0
WALL_FOLLOW   = 1
GOAL_REACHED  = 2

STATE_NAMES = {GO_TO_GOAL: 'GO_TO_GOAL', WALL_FOLLOW: 'WALL_FOLLOW', GOAL_REACHED: 'GOAL_REACHED'}


def _yaw_from_quaternion(q):
    """Extract yaw (radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a, b):
    """Shortest signed angular difference  a − b  in (−π, π]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class Bug2LidarNode(Node):
    """Single-node Bug2 navigator using LiDAR and odometry."""

    def __init__(self):
        super().__init__('bug2_lidar_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('namespace',             'robot_9')

        flex = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('goal_x',                2.0,  flex)
        self.declare_parameter('goal_y',                0.0,  flex)

        # Speeds
        self.declare_parameter('linear_speed',          0.15, flex)
        self.declare_parameter('wall_speed',            0.12, flex)
        self.declare_parameter('angular_speed',         0.50, flex)

        # LiDAR distance thresholds tuned for ~5cm body clearance from walls:
        # - Front: narrow cone straight ahead (-15° to +15°) -> 0.25m
        # - Corner: diagonal check (-45° to -15°) -> 0.25m
        # - Side: right wall detection (-110° to -70°) -> target 0.21m
        self.declare_parameter('front_dist_threshold',  0.24, flex)
        self.declare_parameter('corner_dist_threshold', 0.25, flex)
        self.declare_parameter('side_dist_threshold',   0.40, flex)
        self.declare_parameter('desired_wall_dist',     0.21, flex)

        # Navigation tolerances
        self.declare_parameter('goal_tolerance',        0.15, flex)
        self.declare_parameter('m_line_tolerance',      0.15, flex)
        self.declare_parameter('leave_progress_margin', 0.02, flex)
        self.declare_parameter('min_hit_clearance',     0.30, flex)

        # Fetch values
        ns                        = self.get_parameter('namespace').value
        self.goal_dx              = float(self.get_parameter('goal_x').value)
        self.goal_dy              = float(self.get_parameter('goal_y').value)
        self.linear_speed         = float(self.get_parameter('linear_speed').value)
        self.wall_speed           = float(self.get_parameter('wall_speed').value)
        self.angular_speed        = float(self.get_parameter('angular_speed').value)
        self.front_dist_thr       = float(self.get_parameter('front_dist_threshold').value)
        self.corner_dist_thr      = float(self.get_parameter('corner_dist_threshold').value)
        self.side_dist_thr        = float(self.get_parameter('side_dist_threshold').value)
        self.desired_wall_dist    = float(self.get_parameter('desired_wall_dist').value)
        self.goal_tolerance       = float(self.get_parameter('goal_tolerance').value)
        self.m_line_tol           = float(self.get_parameter('m_line_tolerance').value)
        self.leave_margin         = float(self.get_parameter('leave_progress_margin').value)
        self.min_hit_clearance    = float(self.get_parameter('min_hit_clearance').value)

        # ── Internal state ────────────────────────────────────────────────────
        self.state                = GO_TO_GOAL
        self.odom_ready           = False
        self.scan_ready           = False

        # Odometry
        self.start_x = self.start_y = 0.0
        self.start_yaw            = 0.0
        self.start_recorded       = False
        self.curr_x = self.curr_y = self.curr_yaw = 0.0

        # Goal in world frame (set once odom start is recorded)
        self.goal_x = self.goal_y = 0.0

        # M-line: coefficients of  A*x + B*y + C = 0
        self.ml_a = self.ml_b = self.ml_c = 0.0
        self.ml_len                = 0.0   # |SG|

        # Wall-follow bookkeeping
        self.hit_x = self.hit_y   = 0.0
        self.hit_dist_to_goal      = float('inf')
        self.hit_clearance_ok      = False

        # LiDAR 6-sector distances (meters, inf = nothing detected)
        self.lidar_front       = float('inf')   # -30° to +30°
        self.lidar_front_right = float('inf')   # -70° to -30°
        self.lidar_right       = float('inf')   # -120° to -70°
        self.lidar_rear_right  = float('inf')   # -160° to -120°
        self.lidar_front_left  = float('inf')   # +30° to +70°
        self.lidar_left        = float('inf')   # +70° to +120°

        # ── ROS 2 interfaces ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(
            Twist, f'/{ns}/cmd_vel', 10)

        self.create_subscription(
            Odometry, f'/{ns}/odom',
            self._odom_cb, sensor_qos)

        # Subscribe to both possible scan topics
        self.create_subscription(
            LaserScan, f'/{ns}/scan',
            self._scan_cb, sensor_qos)
        self.create_subscription(
            LaserScan, '/scan',
            self._scan_cb, sensor_qos)

        # Main control loop at 10 Hz
        self.create_timer(0.10, self._control_loop)

        # Telemetry at 1 Hz
        self.create_timer(1.00, self._log_status)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  Bug2 Navigator (LiDAR, 6-sector)')
        self.get_logger().info(f'  Goal offset: dx={self.goal_dx:.2f} m  dy={self.goal_dy:.2f} m')
        self.get_logger().info(f'  Front thr: {self.front_dist_thr:.2f}m  Corner thr: {self.corner_dist_thr:.2f}m')
        self.get_logger().info(f'  Side  thr: {self.side_dist_thr:.2f}m  Desired wall: {self.desired_wall_dist:.2f}m')
        self.get_logger().info('  Waiting for first odometry ...')
        self.get_logger().info('=' * 60)

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.curr_x   = pos.x
        self.curr_y   = pos.y
        self.curr_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ready = True

        if not self.start_recorded:
            self._record_start()

    def _scan_cb(self, msg: LaserScan):
        """Extract 6-sector minimum distances from LaserScan."""
        n = len(msg.ranges)
        if n == 0:
            return

        # Pre-compute angle for each ray (once per scan)
        # Sectors: accumulate min valid range per sector
        sectors = {
            'front':       float('inf'),   # -15° to +15° (Narrow corridor directly in front)
            'front_right': float('inf'),   # -50° to -15° (Diagonal corner)
            'right':       float('inf'),   # -110° to -70° (Perpendicular right wall)
            'rear_right':  float('inf'),   # -160° to -120° (Rear alignment check)
            'front_left':  float('inf'),   # +15° to +50°
            'left':        float('inf'),   # +70° to +110°
        }

        for i in range(n):
            r = msg.ranges[i]
            if not (msg.range_min <= r <= msg.range_max and math.isfinite(r)):
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)
            # Normalise to -180..+180
            while angle_deg > 180.0:
                angle_deg -= 360.0
            while angle_deg < -180.0:
                angle_deg += 360.0

            # Classify into sector
            if   -15.0 <= angle_deg <= 15.0:
                sectors['front']       = min(sectors['front'], r)
            if   -50.0 <= angle_deg < -15.0:
                sectors['front_right'] = min(sectors['front_right'], r)
            if  -110.0 <= angle_deg < -70.0:
                sectors['right']       = min(sectors['right'], r)
            if  -160.0 <= angle_deg < -120.0:
                sectors['rear_right']  = min(sectors['rear_right'], r)
            if    15.0 <  angle_deg <= 50.0:
                sectors['front_left']  = min(sectors['front_left'], r)
            if    70.0 <  angle_deg <= 110.0:
                sectors['left']        = min(sectors['left'], r)

        self.lidar_front       = sectors['front']
        self.lidar_front_right = sectors['front_right']
        self.lidar_right       = sectors['right']
        self.lidar_rear_right  = sectors['rear_right']
        self.lidar_front_left  = sectors['front_left']
        self.lidar_left        = sectors['left']
        self.scan_ready        = True

    # ── Initialisation ────────────────────────────────────────────────────────

    def _record_start(self):
        self.start_x = self.curr_x
        self.start_y = self.curr_y
        self.start_yaw = self.curr_yaw
        self.start_recorded = True

        # Rotate (goal_dx, goal_dy) by start_yaw so goal is relative to robot's initial heading!
        cos_yaw = math.cos(self.start_yaw)
        sin_yaw = math.sin(self.start_yaw)
        world_dx = self.goal_dx * cos_yaw - self.goal_dy * sin_yaw
        world_dy = self.goal_dx * sin_yaw + self.goal_dy * cos_yaw

        # World-frame goal
        self.goal_x = self.start_x + world_dx
        self.goal_y = self.start_y + world_dy

        # M-line  A*x + B*y + C = 0   (line through Start and Goal)
        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        self.ml_len = math.hypot(dx, dy)

        if self.ml_len < 1e-6:
            self.get_logger().warn('Goal is identical to start! Staying put.')
            self.state = GOAL_REACHED
            return

        # Normal form:  (-dy)*x + (dx)*y + (dy*sx - dx*sy) = 0
        self.ml_a = -dy
        self.ml_b =  dx
        self.ml_c =  dy * self.start_x - dx * self.start_y

        self.get_logger().info('Start recorded.')
        self.get_logger().info(
            f'  Start : ({self.start_x:.3f}, {self.start_y:.3f})  yaw={math.degrees(self.start_yaw):.1f}°')
        self.get_logger().info(
            f'  Goal  : ({self.goal_x:.3f}, {self.goal_y:.3f})')
        self.get_logger().info(
            f'  M-line: {self.ml_a:.3f}x + {self.ml_b:.3f}y + {self.ml_c:.3f} = 0')

    # ── Distance helpers ──────────────────────────────────────────────────────

    def _dist_to_goal(self):
        return math.hypot(self.curr_x - self.goal_x, self.curr_y - self.goal_y)

    def _dist_to_m_line(self):
        """Perpendicular distance from current position to the M-line."""
        denom = math.hypot(self.ml_a, self.ml_b)
        if denom < 1e-9:
            return float('inf')
        return abs(self.ml_a * self.curr_x + self.ml_b * self.curr_y + self.ml_c) / denom

    def _dist_from_hit(self):
        return math.hypot(self.curr_x - self.hit_x, self.curr_y - self.hit_y)

    # ── Motion primitives ─────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)

    # ── Main control loop ─────────────────────────────────────────────────────

    def _control_loop(self):
        if not self.odom_ready or not self.scan_ready or not self.start_recorded:
            return

        if self.state == GOAL_REACHED:
            self._stop()
            return

        if self.state == GO_TO_GOAL:
            self._go_to_goal()
        elif self.state == WALL_FOLLOW:
            self._wall_follow()

    # ── GO_TO_GOAL behaviour ──────────────────────────────────────────────────

    def _go_to_goal(self):
        d = self._dist_to_goal()

        # Reached?
        if d < self.goal_tolerance:
            self._stop()
            self.state = GOAL_REACHED
            self.get_logger().info('★  GOAL REACHED  ★')
            return

        # Steer toward goal
        angle_to_goal = math.atan2(
            self.goal_y - self.curr_y,
            self.goal_x - self.curr_x)
        err = _angle_diff(angle_to_goal, self.curr_yaw)

        # Obstacle check: front OR front-right diagonal (only when roughly facing goal)
        if abs(err) <= 0.60:
            if self.lidar_front <= self.front_dist_thr or self.lidar_front_right <= self.corner_dist_thr:
                self._enter_wall_follow()
                return

        if abs(err) > 0.30:          # > ~17° → turn in place
            self._publish(0.0, self.angular_speed * (1.0 if err > 0 else -1.0))
        else:                        # aligned → go forward
            kp = 1.2
            ang = max(-self.angular_speed,
                      min(self.angular_speed, kp * err))
            self._publish(self.linear_speed, ang)

    def _enter_wall_follow(self):
        self.hit_x           = self.curr_x
        self.hit_y           = self.curr_y
        self.hit_dist_to_goal = self._dist_to_goal()
        self.hit_clearance_ok = False
        self.state           = WALL_FOLLOW
        self.get_logger().info(
            f'[HIT]  ({self.hit_x:.2f}, {self.hit_y:.2f})  '
            f'dist_to_goal={self.hit_dist_to_goal:.2f} m  →  WALL_FOLLOW')

    # ── WALL_FOLLOW behaviour (right-hand rule using LiDAR, 6-sector) ────────

    def _wall_follow(self):
        d = self._dist_to_goal()

        # 1. Check goal arrival
        if d < self.goal_tolerance:
            self._stop()
            self.state = GOAL_REACHED
            self.get_logger().info('★  GOAL REACHED (during wall-follow)  ★')
            return

        # 2. Check hit-clearance (must travel min_hit_clearance before we can leave)
        if not self.hit_clearance_ok:
            if self._dist_from_hit() >= self.min_hit_clearance:
                self.hit_clearance_ok = True

        # 3. Check leave condition
        if self.hit_clearance_ok:
            on_m_line  = self._dist_to_m_line() <= self.m_line_tol
            progressed = d <= (self.hit_dist_to_goal - self.leave_margin)
            if on_m_line and progressed:
                self.state = GO_TO_GOAL
                self.get_logger().info(
                    f'[LEAVE]  ({self.curr_x:.2f}, {self.curr_y:.2f})  '
                    f'dist_to_goal={d:.2f} m  →  GO_TO_GOAL')
                return

        # 4. Smooth Right-hand wall-follow logic
        front_blocked       = self.lidar_front       <= self.front_dist_thr
        front_right_blocked = self.lidar_front_right <= self.corner_dist_thr
        right_present       = self.lidar_right        <= self.side_dist_thr

        if front_blocked:
            # Direct dead-end ahead -> turn left in place to clear path
            self._publish(0.0, self.angular_speed)

        elif front_right_blocked and self.lidar_front_right < 0.22:
            # Corner obstacle is very close -> curve left smoothly while moving forward
            self._publish(self.wall_speed * 0.5, self.angular_speed * 0.8)

        elif not right_present:
            # Lost wall on right -> curve right gently forward to re-acquire wall
            self._publish(self.wall_speed * 0.8, -self.angular_speed * 0.5)

        else:
            # Normal wall tracking: proportional steering to maintain target distance
            # Error = current distance - desired distance (0.21m)
            # If error > 0 (too far) -> steer right (negative angular)
            # If error < 0 (too close) -> steer left (positive angular)
            dist_error = self.lidar_right - self.desired_wall_dist
            kp = 2.0
            ang = -kp * dist_error
            ang = max(-self.angular_speed * 0.7, min(self.angular_speed * 0.7, ang))

            self._publish(self.wall_speed, ang)

    # ── Periodic telemetry ────────────────────────────────────────────────────

    def _log_status(self):
        if not self.start_recorded:
            return
        d     = self._dist_to_goal()
        d_ml  = self._dist_to_m_line() if self.state == WALL_FOLLOW else 0.0
        state = STATE_NAMES.get(self.state, '?')

        def _f(v):
            return f'{v:.2f}' if math.isfinite(v) else ' inf'

        self.get_logger().info(
            f'[{state:12s}]  '
            f'pos=({self.curr_x:.2f},{self.curr_y:.2f})  '
            f'yaw={math.degrees(self.curr_yaw):5.1f}°  '
            f'dist_goal={d:.2f}m  '
            f'F={_f(self.lidar_front)} FR={_f(self.lidar_front_right)} '
            f'R={_f(self.lidar_right)} RR={_f(self.lidar_rear_right)}'
            + (f'  M-line={d_ml:.3f}m' if self.state == WALL_FOLLOW else '')
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Bug2LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ...')
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
