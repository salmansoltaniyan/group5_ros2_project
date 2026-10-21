#!/usr/bin/env python3
"""
================================================================================
Bug2 Navigator for iRobot Create 3  -  ROS 2 Jazzy
================================================================================

DESCRIPTION
-----------
Solves a maze using the Bug2 algorithm.

Inputs:
  - goal_x, goal_y  : distance (meters) from the START position to the goal
                      (set as ROS 2 parameters, or via ros2 param set at runtime)
  - /robot_9/ir_intensity   : irobot_create_msgs/msg/IrIntensityVector  (wall detection)
  - /robot_9/odom           : nav_msgs/msg/Odometry                     (position)

Output:
  - /robot_9/cmd_vel        : geometry_msgs/msg/Twist

STATE MACHINE
-------------
  GO_TO_GOAL  →  (obstacle detected)  →  WALL_FOLLOW
  WALL_FOLLOW →  (back on M-line, closer to goal) → GO_TO_GOAL
  GOAL_REACHED → stop forever

Bug2 M-line:
  Straight line from Start (S) to Goal (G).
  The robot follows this line in GO_TO_GOAL mode.
  When it hits a wall it switches to WALL_FOLLOW (right-hand rule).
  It leaves WALL_FOLLOW only when it crosses the M-line again at a point
  closer to G than the hit-point H was.

SENSOR USE
----------
  IR sensors (no LiDAR):
    - Front    : sensors 3, 4         (index 3 and 4 of the vector)
    - Right    : sensors 5, 6         (index 5 and 6)
    - Left     : sensors 0, 1         (index 0 and 1)
  Thresholds are configurable via parameters.

TUNING (params you can change)
--------------------------------
  goal_x, goal_y            : target offset from start (meters)
  namespace                 : robot namespace  (default: robot_9)
  linear_speed              : forward speed    (default: 0.15 m/s)
  wall_speed                : speed along wall (default: 0.12 m/s)
  angular_speed             : turn speed       (default: 0.5  rad/s)
  front_ir_threshold        : IR count that triggers "wall in front"
  side_ir_threshold         : IR count that triggers "wall on side"
  desired_side_ir           : target IR reading for side distance control
  goal_tolerance            : arrival radius   (default: 0.15 m)
  m_line_tolerance          : band to detect M-line crossing (default: 0.10 m)
  leave_progress_margin     : must be this much closer than H to leave (default: 0.15 m)
  min_hit_clearance         : minimum distance to travel from H before checking leave
================================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector


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


class Bug2Node(Node):
    """Single-node Bug2 navigator using IR sensors and odometry."""

    def __init__(self):
        super().__init__('bug2_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('namespace',             'robot_9')
        self.declare_parameter('goal_x',                2.0)   # meters ahead (+X)
        self.declare_parameter('goal_y',                0.0)   # meters lateral (+Y = left)

        # Speeds
        self.declare_parameter('linear_speed',          0.15)
        self.declare_parameter('wall_speed',            0.12)
        self.declare_parameter('angular_speed',         0.50)

        # IR thresholds  (raw ADC counts; typical range 0–200+ in open air, 400+ near wall)
        self.declare_parameter('front_ir_threshold',    100)   # "wall in front"
        self.declare_parameter('side_ir_threshold',     80)    # "wall detected on side"
        self.declare_parameter('desired_side_ir',       120)   # target side distance

        # Navigation tolerances
        self.declare_parameter('goal_tolerance',        0.15)
        self.declare_parameter('m_line_tolerance',      0.10)
        self.declare_parameter('leave_progress_margin', 0.15)
        self.declare_parameter('min_hit_clearance',     0.30)

        # Fetch values
        ns                        = self.get_parameter('namespace').value
        self.goal_dx              = float(self.get_parameter('goal_x').value)
        self.goal_dy              = float(self.get_parameter('goal_y').value)
        self.linear_speed         = float(self.get_parameter('linear_speed').value)
        self.wall_speed           = float(self.get_parameter('wall_speed').value)
        self.angular_speed        = float(self.get_parameter('angular_speed').value)
        self.front_ir_thr         = int(self.get_parameter('front_ir_threshold').value)
        self.side_ir_thr          = int(self.get_parameter('side_ir_threshold').value)
        self.desired_side_ir      = int(self.get_parameter('desired_side_ir').value)
        self.goal_tolerance       = float(self.get_parameter('goal_tolerance').value)
        self.m_line_tol           = float(self.get_parameter('m_line_tolerance').value)
        self.leave_margin         = float(self.get_parameter('leave_progress_margin').value)
        self.min_hit_clearance    = float(self.get_parameter('min_hit_clearance').value)

        # ── Internal state ────────────────────────────────────────────────────
        self.state                = GO_TO_GOAL
        self.odom_ready           = False
        self.ir_ready             = False

        # Odometry
        self.start_x = self.start_y = 0.0
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

        # IR readings (raw counts, -1 = not yet received)
        self.ir_front   = 0   # max of sensors 3,4
        self.ir_right   = 0   # max of sensors 5,6
        self.ir_left    = 0   # max of sensors 0,1

        # ── ROS 2 interfaces ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(
            Twist, f'/{ns}/cmd_vel', 10)

        self.create_subscription(
            Odometry, f'/{ns}/odom',
            self._odom_cb, sensor_qos)

        self.create_subscription(
            IrIntensityVector, f'/{ns}/ir_intensity',
            self._ir_cb, sensor_qos)

        # Main control loop at 10 Hz
        self.create_timer(0.10, self._control_loop)

        # Telemetry at 1 Hz
        self.create_timer(1.00, self._log_status)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  Bug2 Navigator  —  Waiting for first odometry ...')
        self.get_logger().info(f'  Goal offset: dx={self.goal_dx:.2f} m  dy={self.goal_dy:.2f} m')
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

    def _ir_cb(self, msg: IrIntensityVector):
        """Extract front / right / left from the IR intensity vector."""
        readings = [r.value for r in msg.readings]
        n = len(readings)

        def _safe(idx):
            return readings[idx] if 0 <= idx < n else 0

        # iRobot Create 3 IR layout (7 sensors, 0=left-front … 6=right-front):
        #   0: far-left      1: left      2: left-front
        #   3: front-left    4: front-right
        #   5: right         6: far-right
        self.ir_left  = max(_safe(0), _safe(1))
        self.ir_front = max(_safe(3), _safe(4))
        self.ir_right = max(_safe(5), _safe(6))
        self.ir_ready = True

    # ── Initialisation ────────────────────────────────────────────────────────

    def _record_start(self):
        self.start_x = self.curr_x
        self.start_y = self.curr_y
        self.start_recorded = True

        # World-frame goal
        self.goal_x = self.start_x + self.goal_dx
        self.goal_y = self.start_y + self.goal_dy

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
            f'  Start : ({self.start_x:.3f}, {self.start_y:.3f})')
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
        if not self.odom_ready or not self.ir_ready or not self.start_recorded:
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

        # Obstacle in front?
        if self.ir_front >= self.front_ir_thr:
            self._enter_wall_follow()
            return

        # Steer toward goal
        angle_to_goal = math.atan2(
            self.goal_y - self.curr_y,
            self.goal_x - self.curr_x)
        err = _angle_diff(angle_to_goal, self.curr_yaw)

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

    # ── WALL_FOLLOW behaviour (right-hand rule using IR) ──────────────────────

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

        # 4. Right-hand wall-follow logic
        #    Goal: keep a wall on the RIGHT side.
        front_blocked = self.ir_front >= self.front_ir_thr
        right_present = self.ir_right >= self.side_ir_thr

        if front_blocked:
            # Wall ahead → turn LEFT (counter-clockwise)
            self._publish(0.0, self.angular_speed)
        elif not right_present:
            # Lost right wall → turn RIGHT to find it
            self._publish(self.wall_speed * 0.7, -self.angular_speed * 0.6)
        else:
            # Wall on right → proportional control to maintain distance
            error = self.desired_side_ir - self.ir_right   # positive → too far, need to go right
            kp    = self.angular_speed / max(self.desired_side_ir, 1)
            ang   = -kp * error   # negative error → turn right (into wall) to get closer
            ang   = max(-self.angular_speed, min(self.angular_speed, ang))
            self._publish(self.wall_speed, ang)

    # ── Periodic telemetry ────────────────────────────────────────────────────

    def _log_status(self):
        if not self.start_recorded:
            return
        d     = self._dist_to_goal()
        d_ml  = self._dist_to_m_line() if self.state == WALL_FOLLOW else 0.0
        state = STATE_NAMES.get(self.state, '?')
        self.get_logger().info(
            f'[{state:12s}]  '
            f'pos=({self.curr_x:.2f},{self.curr_y:.2f})  '
            f'yaw={math.degrees(self.curr_yaw):5.1f}°  '
            f'dist_goal={d:.2f}m  '
            f'IR F={self.ir_front:3d} R={self.ir_right:3d} L={self.ir_left:3d}'
            + (f'  M-line={d_ml:.3f}m' if self.state == WALL_FOLLOW else '')
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
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

