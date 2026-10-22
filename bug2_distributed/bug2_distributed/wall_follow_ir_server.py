#!/usr/bin/env python3
"""
================================================================================
WallFollow IR Action Server (Adapter Node) for iRobot Create 3 - ROS 2 Jazzy
================================================================================
Node: wall_follow_ir_server
Exposes Action Server : /wall_follow (create3_interfaces/action/Bug2WallFollow)
Calls Action Client   : /{ns}/wall_follow (irobot_create_msgs/action/WallFollow)

Architecture & Role in Bug2:
- Acts as a bridge/adapter between Bug2 Coordinator and Create 3's built-in
  IR-based WallFollow action server.
- Receives start_point, target_point, and hit_point from Bug2 Coordinator.
- Calls Create 3's internal wall_follow action to track the wall using native IR sensors.
- Continuously monitors Odometry to track progress relative to the M-line.
- When a valid Bug2 leave point (or goal) is reached, cancels the internal
  Create 3 WallFollow action and returns success to Bug2 Coordinator.
================================================================================
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor

from create3_interfaces.action import Bug2WallFollow
from irobot_create_msgs.action import WallFollow


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WallFollowIRServer(Node):
    def __init__(self):
        super().__init__('wall_follow_ir_server')

        flex = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('namespace',             'robot_9')
        self.declare_parameter('goal_tolerance',        0.15, flex)
        self.declare_parameter('m_line_tolerance',      0.15, flex)
        self.declare_parameter('leave_progress_margin', 0.02, flex)
        self.declare_parameter('min_hit_clearance',     0.30, flex)
        self.declare_parameter('max_runtime_sec',       300,  flex)

        ns = str(self.get_parameter('namespace').value)
        self.goal_tolerance    = float(self.get_parameter('goal_tolerance').value)
        self.m_line_tol        = float(self.get_parameter('m_line_tolerance').value)
        self.leave_margin      = float(self.get_parameter('leave_progress_margin').value)
        self.min_hit_clearance = float(self.get_parameter('min_hit_clearance').value)
        self.max_runtime_sec   = int(self.get_parameter('max_runtime_sec').value)

        # Pose cache
        self.curr_x     = 0.0
        self.curr_y     = 0.0
        self.curr_yaw   = 0.0
        self.odom_ready = False

        self.cb_group = ReentrantCallbackGroup()
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        self.create_subscription(
            Odometry, f'/{ns}/odom', self._odom_cb, sensor_qos, callback_group=self.cb_group
        )

        # Action Client to Create 3's built-in IR WallFollow server
        c3_action_topic = f'/{ns}/wall_follow'
        self.get_logger().info(f'Connecting to Create 3 internal IR WallFollow action: {c3_action_topic} ...')
        self._c3_client = ActionClient(self, WallFollow, c3_action_topic, callback_group=self.cb_group)

        # Action Server exposed to Bug2 Coordinator
        self._action_server = ActionServer(
            self,
            Bug2WallFollow,
            'wall_follow',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('=' * 65)
        self.get_logger().info('  WallFollow IR (Create 3 Adapter) Server Initialized')
        self.get_logger().info(f'  Targeting namespace: {ns}')
        self.get_logger().info('=' * 65)

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.curr_x   = pos.x
        self.curr_y   = pos.y
        self.curr_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ready = True

    def _stop(self):
        t = Twist()
        for _ in range(3):
            self.cmd_pub.publish(t)
            time.sleep(0.02)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received Bug2WallFollow goal request for IR-based following.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Bug2WallFollow IR execution canceled by coordinator.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # 1. Wait for odometry
        while not self.odom_ready and rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Bug2WallFollow.Result()
                result.success = False
                result.message = 'Canceled while waiting for odom'
                return result
            time.sleep(0.05)

        # 2. Extract Bug2 points
        req = goal_handle.request
        sx, sy = req.start_point.x,  req.start_point.y
        gx, gy = req.target_point.x, req.target_point.y
        hx, hy = req.hit_point.x,    req.hit_point.y

        # M-line equation: A*x + B*y + C = 0
        dx = gx - sx
        dy = gy - sy
        ml_a = -dy
        ml_b =  dx
        ml_c =  dy * sx - dx * sy

        hit_dist_to_goal = math.hypot(gx - hx, gy - hy)
        hit_clearance_ok = False

        feedback_msg = Bug2WallFollow.Feedback()
        result       = Bug2WallFollow.Result()

        self.get_logger().info(
            f'[IR-WALL] Starting Create 3 IR WallFollow. Hit: ({hx:.2f}, {hy:.2f}) | '
            f'Goal: ({gx:.2f}, {gy:.2f}) | HitDist={hit_dist_to_goal:.2f}m'
        )

        # 3. Wait for Create 3 internal action server
        if not self._c3_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Create 3 internal IR WallFollow action server not available!')
            goal_handle.abort()
            result.success = False
            result.message = 'Create 3 internal WallFollow not available'
            return result

        # 4. Send Goal to Create 3 Internal WallFollow
        c3_goal = WallFollow.Goal()
        # follow_side: 1 = left, -1 = right (default to right for Bug2 if not specified)
        c3_goal.follow_side = int(req.follow_side) if req.follow_side != 0 else -1
        c3_goal.max_runtime.sec = int(self.max_runtime_sec)
        c3_goal.max_runtime.nanosec = 0

        self.get_logger().info(
            f'[IR-WALL] Calling Create 3 internal WallFollow (follow_side={c3_goal.follow_side}, '
            f'max_runtime={c3_goal.max_runtime.sec}s)...'
        )
        send_goal_future = self._c3_client.send_goal_async(c3_goal)

        while not send_goal_future.done():
            if goal_handle.is_cancel_requested or not rclpy.ok():
                goal_handle.canceled()
                self._stop()
                result.success = False
                result.message = 'Canceled before internal action accepted'
                return result
            time.sleep(0.05)

        c3_handle = send_goal_future.result()
        if not c3_handle.accepted:
            self.get_logger().error('Create 3 internal WallFollow rejected the goal!')
            goal_handle.abort()
            result.success = False
            result.message = 'Create 3 internal WallFollow rejected'
            return result

        self.get_logger().info('[IR-WALL] Internal IR WallFollow ACCEPTED. Tracking M-line and Progress...')
        c3_result_future = c3_handle.get_result_async()

        # 5. Monitoring loop while Create 3 follows wall with IR sensors
        step_count = 0
        while rclpy.ok():
            # Check coordinator cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('[IR-WALL] Coordinator canceled action. Stopping internal WallFollow...')
                c3_handle.cancel_goal_async()
                self._stop()
                goal_handle.canceled()
                result.success = False
                result.message = 'Canceled by coordinator'
                return result

            # Check if Create 3 finished by itself (e.g., timeout reached)
            if c3_result_future.done():
                self.get_logger().warn('[IR-WALL] Create 3 internal WallFollow completed unexpectedly.')
                self._stop()
                goal_handle.abort()
                result.success = False
                result.message = 'Internal Create 3 WallFollow ended without reaching M-line'
                return result

            d_goal = math.hypot(gx - self.curr_x, gy - self.curr_y)
            d_hit  = math.hypot(hx - self.curr_x, hy - self.curr_y)
            denom  = math.hypot(ml_a, ml_b)
            d_m_line = abs(ml_a * self.curr_x + ml_b * self.curr_y + ml_c) / (denom if denom > 1e-9 else 1.0)

            feedback_msg.distance_to_target = float(d_goal)
            feedback_msg.distance_to_m_line = float(d_m_line)
            goal_handle.publish_feedback(feedback_msg)

            # Telemetry every 1.0s
            step_count += 1
            if step_count % 10 == 0:
                self.get_logger().info(
                    f'[IR-WALL] Pos: ({self.curr_x:.2f}, {self.curr_y:.2f}) | '
                    f'M-dist: {d_m_line:.2f}m | GoalDist: {d_goal:.2f}m | HitClearance: {d_hit:.2f}m'
                )

            # Condition 1: Direct arrival at Goal
            if d_goal < self.goal_tolerance:
                self.get_logger().info('★ ★ ★ GOAL REACHED DIRECTLY DURING IR WALL FOLLOW! ★ ★ ★')
                c3_handle.cancel_goal_async()
                self._stop()
                result.success = True
                result.message = 'Goal reached directly during IR wall follow!'
                goal_handle.succeed()
                return result

            # Condition 2: Check Hit Clearance (prevent immediately leaving at hit point)
            if not hit_clearance_ok:
                if d_hit >= self.min_hit_clearance:
                    hit_clearance_ok = True
                    self.get_logger().info(
                        f'[IR-WALL] Hit clearance satisfied ({d_hit:.2f}m >= {self.min_hit_clearance:.2f}m). '
                        f'Now searching for M-line leave point.'
                    )

            # Condition 3: Bug2 Leave Point
            if hit_clearance_ok:
                on_m_line  = (d_m_line <= self.m_line_tol)
                progressed = (d_goal <= (hit_dist_to_goal - self.leave_margin))

                if on_m_line and progressed:
                    self.get_logger().info(
                        f'[LEAVE] Reached valid M-line leave point at ({self.curr_x:.2f}, {self.curr_y:.2f})! '
                        f'M-dist={d_m_line:.2f}m, GoalDist={d_goal:.2f}m < HitDist={hit_dist_to_goal:.2f}m. '
                        f'Canceling IR WallFollow...'
                    )
                    c3_handle.cancel_goal_async()
                    self._stop()
                    result.success = True
                    result.message = f'Valid Bug2 leave point reached at ({self.curr_x:.2f}, {self.curr_y:.2f})!'
                    goal_handle.succeed()
                    return result

            time.sleep(0.10)

        self._stop()
        result.success = False
        result.message = 'ROS shutdown'
        return result


def main(args=None):
    rclpy.init(args=args)
    server = WallFollowIRServer()
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

