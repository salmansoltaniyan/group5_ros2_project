#!/usr/bin/env python3
"""
===============================================================================
Bug 2 Wall-Following Action Server for iRobot Create 3 (ROS 2 Jazzy)
===============================================================================
Node: bug2_wall_follow_server
Action Server: Bug2WallFollow (from create3_interfaces.action)
Built-in Action Client: /robot_9/wall_follow (from irobot_create_msgs.action)
Odometry Topic: /robot_9/odom (nav_msgs/msg/Odometry)

Algorithm & Architecture:
- Receives M (Start), G (Target), H (Hit Point), and follow_side (+1 Left, -1 Right).
- Starts Create 3's native WallFollow action.
- Continuously monitors Odometry (X, Y) and calculates:
  1. Perpendicular distance d to the infinite M-line through M and G:
     d = abs((xg - x0)*(y0 - y) - (x0 - x)*(yg - y0)) / sqrt((xg - x0)^2 + (yg - y0)^2)
  2. Euclidean distance to target G.
  3. Progress along the forward portion of the M-line beyond H toward G.
- Validates leave conditions with hysteresis (consecutive valid odometry samples)
  to reject single-sample noise.
- On valid leave point:
  * Cancels the built-in Create 3 action goal.
  * Waits for cancellation confirmation.
  * Publishes zero velocity to ensure the robot halts cleanly on the M-line.
  * Returns success to the calling client.
- Handles goal cancellation from client, timeouts, and stale odometry gracefully.
===============================================================================
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.action import WallFollow
from create3_interfaces.action import Bug2WallFollow


class Bug2WallFollowServer(Node):
    def __init__(self):
        super().__init__('bug2_wall_follow_server')

        # ---------------------------------------------------------------------
        # 1. CONFIGURABLE PARAMETERS
        # ---------------------------------------------------------------------
        self.declare_parameter('odom_topic', '/robot_9/odom')
        self.declare_parameter('cmd_vel_topic', '/robot_9/cmd_vel')
        self.declare_parameter('wall_follow_action', '/robot_9/wall_follow')
        self.declare_parameter('bug2_action_name', '/robot_9/bug2_wall_follow')

        # Bug 2 Geometric Thresholds
        self.declare_parameter('line_tolerance', 0.08)          # Perpendicular band (meters)
        self.declare_parameter('hit_departure_threshold', 0.25) # Distance required away from H (meters)
        self.declare_parameter('progress_margin', 0.05)         # Distance closer to G than H (meters)
        self.declare_parameter('goal_tolerance', 0.15)          # Direct arrival radius (meters)
        self.declare_parameter('required_consecutive_samples', 3) # Hysteresis sample count
        self.declare_parameter('odom_timeout_sec', 1.0)         # Stale odometry warning threshold
        self.declare_parameter('max_wall_follow_runtime', 600)  # Max duration for built-in action (sec)

        # Retrieve parameter values
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.wall_follow_action_name = str(self.get_parameter('wall_follow_action').value)
        self.bug2_action_name = str(self.get_parameter('bug2_action_name').value)

        self.line_tolerance = float(self.get_parameter('line_tolerance').value)
        self.hit_departure_threshold = float(self.get_parameter('hit_departure_threshold').value)
        self.progress_margin = float(self.get_parameter('progress_margin').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.required_consecutive_samples = int(self.get_parameter('required_consecutive_samples').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.max_wall_follow_runtime = int(self.get_parameter('max_wall_follow_runtime').value)

        # ---------------------------------------------------------------------
        # 2. STATE & THREADING LOCKS
        # ---------------------------------------------------------------------
        self._lock = threading.Lock()
        self.latest_odom_x = 0.0
        self.latest_odom_y = 0.0
        self.latest_odom_time = None
        self.odom_received = False

        # Internal built-in action tracking
        self._builtin_goal_handle = None
        self._builtin_action_done = threading.Event()
        self._builtin_action_status = None

        # Reentrant callback group to allow concurrent service/action execution
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # 3. ROS 2 INTERFACES
        # ---------------------------------------------------------------------
        # Odometry Subscriber
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            sensor_qos,
            callback_group=self.callback_group
        )

        # Backup subscriber if namespace is omitted
        if self.odom_topic.startswith('/robot_9'):
            self.odom_sub_backup = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                sensor_qos,
                callback_group=self.callback_group
            )

        # Cmd_vel Publisher for emergency stops
        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
            callback_group=self.callback_group
        )

        # Built-in iRobot WallFollow Action Client
        self._builtin_client = ActionClient(
            self,
            WallFollow,
            self.wall_follow_action_name,
            callback_group=self.callback_group
        )

        # Custom Bug2WallFollow Action Server
        self._action_server = ActionServer(
            self,
            Bug2WallFollow,
            self.bug2_action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('=' * 70)
        self.get_logger().info('🚀 [Bug2WallFollowServer] Initialized and Ready!')
        self.get_logger().info(f'   Action Server Name:       {self.bug2_action_name}')
        self.get_logger().info(f'   Built-in WallFollow:      {self.wall_follow_action_name}')
        self.get_logger().info(f'   Odometry Topic:           {self.odom_topic}')
        self.get_logger().info(f'   Line Tolerance:           {self.line_tolerance*100:.1f} cm')
        self.get_logger().info(f'   Departure Threshold:      {self.hit_departure_threshold*100:.1f} cm')
        self.get_logger().info(f'   Progress Margin:          {self.progress_margin*100:.1f} cm')
        self.get_logger().info(f'   Hysteresis Samples:       {self.required_consecutive_samples}')
        self.get_logger().info('=' * 70)

    # -------------------------------------------------------------------------
    # ODOMETRY CALLBACK
    # -------------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        with self._lock:
            self.latest_odom_x = msg.pose.pose.position.x
            self.latest_odom_y = msg.pose.pose.position.y
            self.latest_odom_time = self.get_clock().now()
            self.odom_received = True

    # -------------------------------------------------------------------------
    # GOAL VALIDATION & ACCEPTANCE
    # -------------------------------------------------------------------------
    def goal_callback(self, goal_request):
        """Validates incoming Bug2WallFollow goal before accepting."""
        m = goal_request.start_point
        g = goal_request.target_point
        h = goal_request.hit_point
        side = goal_request.follow_side

        self.get_logger().info(
            f'📥 Received Goal Request: Start=({m.x:.2f}, {m.y:.2f}), '
            f'Target=({g.x:.2f}, {g.y:.2f}), Hit=({h.x:.2f}, {h.y:.2f}), Side={side}'
        )

        # 1. Validate follow_side (+1 or -1)
        if side not in (1, -1):
            self.get_logger().error(f'❌ Invalid follow_side {side}! Must be +1 (Left) or -1 (Right). Rejecting.')
            return GoalResponse.REJECT

        # 2. Validate distance between Start and Target
        m_line_len = math.hypot(g.x - m.x, g.y - m.y)
        if m_line_len < 1e-3:
            self.get_logger().error('❌ Start and Target points are identical! Rejecting goal.')
            return GoalResponse.REJECT

        # 3. Check odometry status
        with self._lock:
            if not self.odom_received:
                self.get_logger().warn('⚠️ Odometry not yet received. Waiting briefly before accepting...')

        self.get_logger().info('✅ Goal validated and ACCEPTED.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handles client cancellation requests."""
        self.get_logger().warn('⚠️ Client requested cancellation of Bug2WallFollow goal!')
        return CancelResponse.ACCEPT

    # -------------------------------------------------------------------------
    # BUILT-IN WALL-FOLLOW ACTION WRAPPERS
    # -------------------------------------------------------------------------
    def start_builtin_wall_follow(self, follow_side: int) -> bool:
        """Sends goal to Create 3 /robot_9/wall_follow asynchronously."""
        if not self._builtin_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'❌ Built-in action server {self.wall_follow_action_name} not available!')
            return False

        self._builtin_action_done.clear()
        self._builtin_goal_handle = None
        self._builtin_action_status = None

        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = int(follow_side)
        goal_msg.max_runtime.sec = int(self.max_wall_follow_runtime)
        goal_msg.max_runtime.nanosec = 0

        self.get_logger().info(f'🤖 Sending WallFollow goal to Create 3 (Side: {"LEFT" if follow_side == 1 else "RIGHT"})...')
        
        goal_response_event = threading.Event()
        send_future = self._builtin_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: goal_response_event.set())
        
        # Wait up to 5.0 seconds for Create 3 response
        if not goal_response_event.wait(timeout=5.0):
            self.get_logger().error('❌ Timeout waiting for Create 3 to accept WallFollow goal!')
            return False

        self._builtin_goal_handle = send_future.result()
        if not self._builtin_goal_handle.accepted:
            self.get_logger().error('❌ Create 3 REJECTED WallFollow goal!')
            return False

        self.get_logger().info('✅ Create 3 WallFollow goal ACCEPTED and active!')
        
        # Register completion callback
        result_future = self._builtin_goal_handle.get_result_async()
        result_future.add_done_callback(self._on_builtin_result)
        return True

    def _on_builtin_result(self, future):
        """Called when built-in Create 3 action finishes or is canceled."""
        try:
            res = future.result()
            self._builtin_action_status = res.status
            self.get_logger().info(f'ℹ️ Create 3 WallFollow finished with status: {self._builtin_action_status}')
        except Exception as e:
            self.get_logger().warn(f'Built-in action result exception: {e}')
        finally:
            self._builtin_action_done.set()

    def cancel_builtin_wall_follow(self):
        """Cancels the active built-in action and halts robot movement."""
        if self._builtin_goal_handle is not None:
            self.get_logger().info('🛑 Canceling built-in Create 3 WallFollow goal...')
            try:
                cancel_event = threading.Event()
                cancel_future = self._builtin_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda f: cancel_event.set())
                if not cancel_event.wait(timeout=3.0):
                    self.get_logger().warn('Timeout waiting for built-in cancel confirmation.')
                else:
                    self.get_logger().info('✅ Built-in Create 3 WallFollow canceled successfully.')
            except Exception as e:
                self.get_logger().warn(f'Exception during built-in cancel: {e}')

        # Publish zero velocity command multiple times to guarantee immediate halt
        stop_twist = Twist()
        for _ in range(3):
            self.cmd_pub.publish(stop_twist)
            time.sleep(0.02)

    # -------------------------------------------------------------------------
    # BUG 2 EXECUTE LOOP
    # -------------------------------------------------------------------------
    def execute_callback(self, goal_handle):
        """Main execution thread for the Bug2WallFollow action."""
        self.get_logger().info('▶️ Starting Bug2WallFollow execution...')

        request = goal_handle.request
        m = request.start_point
        g = request.target_point
        h = request.hit_point
        side = request.follow_side

        # M-line geometry precomputations
        # M = (x0, y0), G = (xg, yg)
        x0, y0 = m.x, m.y
        xg, yg = g.x, g.y
        xh, yh = h.x, h.y

        dx = xg - x0
        dy = yg - y0
        m_line_len = math.hypot(dx, dy)
        d_hit_to_target = math.hypot(xg - xh, yg - yh)

        # Projection of Hit point along the M-line: t_hit
        # t = ((x - x0)*dx + (y - y0)*dy) / (m_line_len^2)
        t_hit = ((xh - x0) * dx + (yh - y0) * dy) / (m_line_len ** 2)

        self.get_logger().info('=' * 70)
        self.get_logger().info('📐 [BUG 2 GEOMETRY SUMMARY]')
        self.get_logger().info(f'   Start (M):           ({x0:.2f}, {y0:.2f})')
        self.get_logger().info(f'   Target (G):          ({xg:.2f}, {yg:.2f}) | Length: {m_line_len:.2f} m')
        self.get_logger().info(f'   Hit Point (H):       ({xh:.2f}, {yh:.2f}) | Dist to Target: {d_hit_to_target:.2f} m')
        self.get_logger().info(f'   Hit Line Projection: t_hit = {t_hit:.3f}')
        self.get_logger().info('=' * 70)

        # 1. Start the built-in Create 3 WallFollow action
        if not self.start_builtin_wall_follow(side):
            goal_handle.abort()
            result = Bug2WallFollow.Result()
            result.success = False
            result.message = 'Failed to start built-in Create 3 WallFollow action.'
            return result

        # 2. Tracking loop variables
        consecutive_valid = 0
        feedback_msg = Bug2WallFollow.Feedback()
        loop_rate_sec = 0.05 # 20 Hz evaluation
        last_log_time = 0.0

        # Wait briefly for robot to start moving
        time.sleep(0.3)

        while rclpy.ok():
            # Check for cancellation from custom client
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('⚠️ Action cancellation confirmed. Stopping wall follow.')
                self.cancel_builtin_wall_follow()
                goal_handle.canceled()
                result = Bug2WallFollow.Result()
                result.success = False
                result.message = 'Bug2WallFollow action canceled by client.'
                return result

            # Check if built-in action terminated on its own (error or timeout)
            if self._builtin_action_done.is_set():
                self.get_logger().warn('⚠️ Built-in WallFollow action completed before reaching leave point!')
                self.cancel_builtin_wall_follow()
                goal_handle.abort()
                result = Bug2WallFollow.Result()
                result.success = False
                result.message = f'Built-in action ended unexpectedly with status {self._builtin_action_status}.'
                return result

            # Read thread-safe odometry
            with self._lock:
                if not self.odom_received:
                    time.sleep(loop_rate_sec)
                    continue

                curr_x = self.latest_odom_x
                curr_y = self.latest_odom_y
                odom_t = self.latest_odom_time

            # Check odometry freshness
            odom_age = (self.get_clock().now() - odom_t).nanoseconds / 1e9
            if odom_age > self.odom_timeout_sec:
                self.get_logger().warn(f'Stale odometry detected ({odom_age:.2f}s old)! Skipping evaluation.', throttle_duration_sec=2.0)
                time.sleep(loop_rate_sec)
                continue

            # -----------------------------------------------------------------
            # 3. BUG 2 CALCULATIONS
            # -----------------------------------------------------------------
            # a) Perpendicular distance from P to the infinite line through M and G:
            # d = abs((xg - x0)*(y0 - y) - (x0 - x)*(yg - y0)) / sqrt((xg - x0)^2 + (yg - y0)^2)
            cross_product = (xg - x0) * (y0 - curr_y) - (x0 - curr_x) * (yg - y0)
            d_m_line = abs(cross_product) / m_line_len

            # b) Euclidean distance to Target G
            d_curr_to_target = math.hypot(xg - curr_x, yg - curr_y)

            # c) Euclidean distance from Hit Point H
            d_from_hit = math.hypot(curr_x - xh, curr_y - yh)

            # d) Normalized scalar projection t along M -> G (0 = Start, 1 = Target)
            t_curr = ((curr_x - x0) * dx + (curr_y - y0) * dy) / (m_line_len ** 2)

            # -----------------------------------------------------------------
            # 4. PUBLISH FEEDBACK
            # -----------------------------------------------------------------
            feedback_msg.distance_to_m_line = float(d_m_line)
            feedback_msg.distance_to_target = float(d_curr_to_target)
            goal_handle.publish_feedback(feedback_msg)

            # Periodic status logging (every 0.5s)
            now_sec = time.time()
            if now_sec - last_log_time >= 0.5:
                progress = d_hit_to_target - d_curr_to_target
                self.get_logger().info(
                    f'[WALL-FOLLOW] P=({curr_x:.2f}, {curr_y:.2f}) | '
                    f'M-Line Dist: {d_m_line*100:4.1f}cm | Target Dist: {d_curr_to_target:.2f}m | '
                    f'From Hit: {d_from_hit:.2f}m | Progress: {progress:+.2f}m | '
                    f'Valid: {consecutive_valid}/{self.required_consecutive_samples}'
                )
                last_log_time = now_sec

            # -----------------------------------------------------------------
            # 5. VALID LEAVE-POINT EVALUATION
            # -----------------------------------------------------------------
            # Condition 1: Must have moved away from the Hit Point
            cleared_hit_point = d_from_hit >= self.hit_departure_threshold

            # Condition 2: Close to the M-line within tolerance
            on_m_line = d_m_line <= self.line_tolerance

            # Condition 3: Progress toward G beyond the hit point (both in projection and Euclidean distance)
            closer_than_hit = d_curr_to_target < (d_hit_to_target - self.progress_margin)
            forward_of_hit = t_curr > (t_hit + 0.01)

            # Condition 4: Forward portion within reach of target
            within_segment_bounds = (-0.05 <= t_curr <= 1.05)

            # Direct arrival condition (reached target while wall following)
            arrived_at_target = d_curr_to_target <= self.goal_tolerance

            is_valid_leave = (
                (cleared_hit_point and on_m_line and closer_than_hit and forward_of_hit and within_segment_bounds)
                or arrived_at_target
            )

            # Hysteresis counter update
            if is_valid_leave:
                consecutive_valid += 1
            else:
                consecutive_valid = 0

            # -----------------------------------------------------------------
            # 6. LEAVE POINT CONFIRMED -> STOP & RETURN SUCCESS
            # -----------------------------------------------------------------
            if consecutive_valid >= self.required_consecutive_samples:
                self.get_logger().info('=' * 70)
                if arrived_at_target:
                    msg = f"Target directly reached during wall-following at ({curr_x:.2f}, {curr_y:.2f})!"
                    self.get_logger().info(f'🏁 [TARGET REACHED] {msg}')
                else:
                    progress = d_hit_to_target - d_curr_to_target
                    msg = (
                        f"Valid Bug2 leave point reached at ({curr_x:.2f}, {curr_y:.2f})!\n"
                        f"   M-Line Offset:      {d_m_line*100:.1f} cm (tolerance: {self.line_tolerance*100:.1f} cm)\n"
                        f"   Distance to Target: {d_curr_to_target:.2f} m (Hit was {d_hit_to_target:.2f} m, Progress: +{progress:.2f} m)\n"
                        f"   M-Line Projection:  t={t_curr:.3f} (Hit was t={t_hit:.3f})"
                    )
                    self.get_logger().info(f'🎯 [BUG 2 LEAVE POINT CONFIRMED]\n{msg}')
                self.get_logger().info('=' * 70)

                # Cancel built-in action and halt
                self.cancel_builtin_wall_follow()

                goal_handle.succeed()
                result = Bug2WallFollow.Result()
                result.success = True
                result.message = msg
                return result

            time.sleep(loop_rate_sec)

        # If loop exits due to rclpy shutdown
        self.cancel_builtin_wall_follow()
        goal_handle.abort()
        result = Bug2WallFollow.Result()
        result.success = False
        result.message = 'Server shutting down.'
        return result


def main(args=None):
    rclpy.init(args=args)
    server = Bug2WallFollowServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down Bug2WallFollowServer safely...')
    finally:
        server.cancel_builtin_wall_follow()
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

