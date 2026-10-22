#!/usr/bin/env python3
"""
================================================================================
WallFollow Action Server for iRobot Create 3 - ROS 2 Jazzy
================================================================================
Node: wall_follow_server
Action: /wall_follow (create3_interfaces/action/Bug2WallFollow)

Role in Bug2 Architecture:
- Tracks the obstacle wall using 2D LiDAR (right-hand rule).
- Receives start_point, target_point, and hit_point.
- Computes M-line equation and monitors leave condition.
- Executes Stop-at-Corner + PD on straight walls.
- Returns result.success = True when a valid Bug2 leave point is reached.
================================================================================
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor

from create3_interfaces.action import Bug2WallFollow


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class WallFollowServer(Node):
    def __init__(self):
        super().__init__('wall_follow_server')

        flex = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('namespace',             'robot_9')
        self.declare_parameter('wall_speed',            0.12, flex)
        self.declare_parameter('angular_speed',         0.50, flex)
        self.declare_parameter('kp_wall',               1.8,  flex)
        self.declare_parameter('kd_wall',               0.6,  flex)

        self.declare_parameter('front_dist_threshold',  0.26, flex)
        self.declare_parameter('corner_dist_threshold', 0.25, flex)
        self.declare_parameter('side_dist_threshold',   0.32, flex)
        self.declare_parameter('desired_wall_dist',     0.21, flex)

        self.declare_parameter('goal_tolerance',        0.15, flex)
        self.declare_parameter('m_line_tolerance',      0.15, flex)
        self.declare_parameter('leave_progress_margin', 0.02, flex)
        self.declare_parameter('min_hit_clearance',     0.30, flex)

        ns = str(self.get_parameter('namespace').value)
        self.wall_speed           = float(self.get_parameter('wall_speed').value)
        self.angular_speed        = float(self.get_parameter('angular_speed').value)
        self.kp_wall              = float(self.get_parameter('kp_wall').value)
        self.kd_wall              = float(self.get_parameter('kd_wall').value)
        self.front_dist_thr       = float(self.get_parameter('front_dist_threshold').value)
        self.corner_dist_thr      = float(self.get_parameter('corner_dist_threshold').value)
        self.side_dist_thr        = float(self.get_parameter('side_dist_threshold').value)
        self.desired_wall_dist    = float(self.get_parameter('desired_wall_dist').value)
        self.goal_tolerance       = float(self.get_parameter('goal_tolerance').value)
        self.m_line_tol           = float(self.get_parameter('m_line_tolerance').value)
        self.leave_margin         = float(self.get_parameter('leave_progress_margin').value)
        self.min_hit_clearance    = float(self.get_parameter('min_hit_clearance').value)

        # Pose & Sensors
        self.curr_x   = 0.0
        self.curr_y   = 0.0
        self.curr_yaw = 0.0
        self.odom_ready = False

        self.lidar_front       = float('inf')
        self.lidar_front_right = float('inf')
        self.lidar_right       = float('inf')
        self.scan_ready        = False

        self.cb_group = ReentrantCallbackGroup()
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        self.create_subscription(Odometry, f'/{ns}/odom', self._odom_cb, sensor_qos, callback_group=self.cb_group)
        self.create_subscription(LaserScan, f'/{ns}/scan', self._scan_cb, sensor_qos, callback_group=self.cb_group)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos, callback_group=self.cb_group)

        # Action Server
        self._action_server = ActionServer(
            self,
            Bug2WallFollow,
            'wall_follow',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('Bug2WallFollow Action Server initialized.')

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.curr_x   = pos.x
        self.curr_y   = pos.y
        self.curr_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ready = True

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        f_min = fr_min = r_min = float('inf')
        for i in range(n):
            r = msg.ranges[i]
            if not (msg.range_min <= r <= msg.range_max and math.isfinite(r)):
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)
            while angle_deg > 180.0:
                angle_deg -= 360.0
            while angle_deg < -180.0:
                angle_deg += 360.0

            if -20.0 <= angle_deg <= 20.0:
                f_min = min(f_min, r)
            elif -50.0 <= angle_deg < -20.0:
                fr_min = min(fr_min, r)
            elif -110.0 <= angle_deg < -70.0:
                r_min = min(r_min, r)

        self.lidar_front       = f_min
        self.lidar_front_right = fr_min
        self.lidar_right       = r_min
        self.scan_ready        = True

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def goal_callback(self, goal_request):
        self.get_logger().info('Received WallFollow goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('WallFollow execution canceled by client.')
        self._stop()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        sx = goal_handle.request.start_point.x
        sy = goal_handle.request.start_point.y
        gx = goal_handle.request.target_point.x
        gy = goal_handle.request.target_point.y
        hx = goal_handle.request.hit_point.x
        hy = goal_handle.request.hit_point.y

        # M-line equation: A*x + B*y + C = 0
        dx = gx - sx
        dy = gy - sy
        ml_len = math.hypot(dx, dy)
        ml_a = -dy
        ml_b =  dx
        ml_c =  dy * sx - dx * sy

        hit_dist_to_goal = math.hypot(gx - hx, gy - hy)
        hit_clearance_ok = False

        prev_wall_error = 0.0
        prev_error_time = None

        feedback_msg = Bug2WallFollow.Feedback()
        result       = Bug2WallFollow.Result()

        self.get_logger().info(f'Starting WallFollow along M-line. Hit dist={hit_dist_to_goal:.2f}m')
        step_count = 0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop()
                result.success = False
                result.message = 'Canceled'
                return result

            d_goal = math.hypot(gx - self.curr_x, gy - self.curr_y)
            d_hit  = math.hypot(hx - self.curr_x, hy - self.curr_y)
            denom  = math.hypot(ml_a, ml_b)
            d_m_line = abs(ml_a * self.curr_x + ml_b * self.curr_y + ml_c) / (denom if denom > 1e-9 else 1.0)

            feedback_msg.distance_to_target = float(d_goal)
            feedback_msg.distance_to_m_line = float(d_m_line)
            goal_handle.publish_feedback(feedback_msg)

            # Telemetry every 1.0s (10 * 0.1s)
            step_count += 1
            if step_count % 10 == 0:
                self.get_logger().info(
                    f'[WALL] Pos: ({self.curr_x:.2f}, {self.curr_y:.2f}) | '
                    f'F: {self.lidar_front:.2f}m | FR: {self.lidar_front_right:.2f}m | '
                    f'R: {self.lidar_right:.2f}m | M-dist: {d_m_line:.2f}m | GoalDist: {d_goal:.2f}m'
                )

            # 1. Direct Goal Arrival Check
            if d_goal < self.goal_tolerance:
                self._stop()
                result.success = True
                result.message = 'Target reached directly during wall follow!'
                goal_handle.succeed()
                self.get_logger().info('★ GOAL REACHED DURING WALL FOLLOW! ★')
                return result

            # 2. Check Hit Clearance
            if not hit_clearance_ok:
                if d_hit >= self.min_hit_clearance:
                    hit_clearance_ok = True

            # 3. Bug2 Leave Point Check
            if hit_clearance_ok:
                on_m_line  = (d_m_line <= self.m_line_tol)
                progressed = (d_goal <= (hit_dist_to_goal - self.leave_margin))

                angle_to_goal = math.atan2(gy - self.curr_y, gx - self.curr_x)
                heading_err = abs(_angle_diff(angle_to_goal, self.curr_yaw))
                path_open = (self.lidar_front > 0.45) or (heading_err > 1.2)

                if on_m_line and progressed and path_open:
                    self._stop()
                    result.success = True
                    result.message = f'Valid Bug2 leave point reached at ({self.curr_x:.2f}, {self.curr_y:.2f})!'
                    goal_handle.succeed()
                    self.get_logger().info(f'[LEAVE] Reached M-line leave point at ({self.curr_x:.2f}, {self.curr_y:.2f})!')
                    return result

            # 4. Motion Control: Stop-at-Corner + PD on Straights
            # RULE 1: Inner corner / dead-end ahead
            if self.lidar_front < self.front_dist_thr or self.lidar_front_right < self.corner_dist_thr:
                self._publish(0.0, self.angular_speed)
                prev_wall_error = 0.0
                prev_error_time = None
                time.sleep(0.10)
                continue

            # RULE 2: Outer corner (wall ended on right)
            if self.lidar_right > self.side_dist_thr:
                self._publish(0.0, -self.angular_speed)
                prev_wall_error = 0.0
                prev_error_time = None
                time.sleep(0.10)
                continue

            # RULE 3: Straight wall driving with continuous PD
            error = self.lidar_right - self.desired_wall_dist
            now_time = self.get_clock().now().nanoseconds / 1e9

            if prev_error_time is not None and (now_time - prev_error_time) > 1e-4:
                dt = now_time - prev_error_time
                d_error = (error - prev_wall_error) / dt
            else:
                d_error = 0.0

            prev_wall_error = error
            prev_error_time = now_time

            u_ang = - (self.kp_wall * error + self.kd_wall * d_error)
            max_ang = 0.40
            u_ang = max(-max_ang, min(max_ang, u_ang))

            self._publish(self.wall_speed, u_ang)
            time.sleep(0.10)

        result.success = False
        result.message = 'Server shutdown'
        return result

    def _publish(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    server = WallFollowServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server._stop()
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

