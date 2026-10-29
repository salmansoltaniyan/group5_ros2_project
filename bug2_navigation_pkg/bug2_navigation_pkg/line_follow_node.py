#!/usr/bin/env python3
"""
===============================================================================
Line Follow Node / Bug2 Client Demonstration for iRobot Create 3 (ROS 2 Jazzy)
===============================================================================
Node: line_follow_node
Action Client: Bug2WallFollow (from create3_interfaces.action)
Target Action: /robot_9/bug2_wall_follow

Role in Bug 2 Architecture:
- Represents the Line-Following / Motion-to-Goal controller.
- When an obstacle is detected on the M-line, this node records the Hit Point (H)
  and calls the Bug2WallFollow action server to navigate around the obstacle.
- Submits the goal asynchronously and listens for live feedback (distance to M-line,
  distance to target).
- Upon receiving a successful result (indicating a valid Bug2 leave point on the M-line),
  logs that wall following is complete and that it is ready to transition back to
  direct Goal Tracking.
===============================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from create3_interfaces.action import Bug2WallFollow


class LineFollowNode(Node):
    def __init__(self):
        super().__init__('line_follow_node')

        # ---------------------------------------------------------------------
        # 1. PARAMETERS
        # ---------------------------------------------------------------------
        self.declare_parameter('action_name', '/robot_9/bug2_wall_follow')
        self.declare_parameter('odom_topic', '/robot_9/odom')
        self.declare_parameter('auto_detect_start', True)   # Auto-detect pose from odometry
        self.declare_parameter('target_distance', 2.0)      # Distance (m) ahead along heading
        self.declare_parameter('start_behind_dist', 0.5)    # Distance (m) behind hit point for M
        self.declare_parameter('follow_side', -1)           # -1: Right wall, +1: Left wall

        # Manual coordinates (used when auto_detect_start is False)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 1.0)
        self.declare_parameter('hit_x', 0.8)
        self.declare_parameter('hit_y', 0.4)

        self.action_name = str(self.get_parameter('action_name').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.auto_detect_start = bool(self.get_parameter('auto_detect_start').value)
        self.target_distance = float(self.get_parameter('target_distance').value)
        self.start_behind_dist = float(self.get_parameter('start_behind_dist').value)
        self.follow_side = int(self.get_parameter('follow_side').value)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.hit_x = float(self.get_parameter('hit_x').value)
        self.hit_y = float(self.get_parameter('hit_y').value)

        # ---------------------------------------------------------------------
        # 2. ACTION CLIENT SETUP
        # ---------------------------------------------------------------------
        self._action_client = ActionClient(self, Bug2WallFollow, self.action_name)
        self._goal_handle = None
        self._goal_sent = False

        if self.auto_detect_start:
            self.get_logger().info('=' * 70)
            self.get_logger().info('🤖 [LineFollowNode] Initialized with AUTO-ODOMETRY MODE!')
            self.get_logger().info(f'   Target Action Server: {self.action_name}')
            self.get_logger().info(f'   Target Distance:      {self.target_distance:.2f} m ahead along heading')
            self.get_logger().info(f'   Follow Side:          {"LEFT" if self.follow_side == 1 else "RIGHT"}')
            self.get_logger().info(f'   Listening to:         {self.odom_topic}')
            self.get_logger().info('=' * 70)

            sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self._odom_callback,
                sensor_qos
            )
            # Backup subscriber if namespace is omitted
            if self.odom_topic.startswith('/robot_9'):
                self._odom_sub_backup = self.create_subscription(
                    Odometry,
                    '/odom',
                    self._odom_callback,
                    sensor_qos
                )
        else:
            self.get_logger().info('=' * 70)
            self.get_logger().info('🤖 [LineFollowNode] Initialized with MANUAL coordinates!')
            self.get_logger().info(f'   Target Action Server: {self.action_name}')
            self.get_logger().info(f'   Start Point (M):      ({self.start_x:.2f}, {self.start_y:.2f})')
            self.get_logger().info(f'   Target Point (G):     ({self.target_x:.2f}, {self.target_y:.2f})')
            self.get_logger().info(f'   Hit Point (H):        ({self.hit_x:.2f}, {self.hit_y:.2f})')
            self.get_logger().info(f'   Follow Side:          {"LEFT" if self.follow_side == 1 else "RIGHT"}')
            self.get_logger().info('=' * 70)
            self._timer = self.create_timer(1.0, self.send_goal_once)

    def _odom_callback(self, msg: Odometry):
        """Automatically reads the current robot pose from odometry and sets Bug2 points."""
        if self._goal_sent:
            return
        self._goal_sent = True

        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        # Compute yaw angle from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw)

        # Hit point H is current robot position at obstacle
        self.hit_x = curr_x
        self.hit_y = curr_y

        # Target point G is target_distance ahead along robot heading
        self.target_x = curr_x + self.target_distance * math.cos(yaw)
        self.target_y = curr_y + self.target_distance * math.sin(yaw)

        # Start point M is start_behind_dist behind Hit point along the line
        self.start_x = curr_x - self.start_behind_dist * math.cos(yaw)
        self.start_y = curr_y - self.start_behind_dist * math.sin(yaw)

        self.get_logger().info('=' * 70)
        self.get_logger().info('🎯 [LineFollowNode] Auto-detected Live Odometry Pose!')
        self.get_logger().info(f'   Robot Pose:           (x={curr_x:.2f}, y={curr_y:.2f}, heading={yaw_deg:.1f}°)')
        self.get_logger().info(f'   Start Point (M):      ({self.start_x:.2f}, {self.start_y:.2f}) [{self.start_behind_dist:.1f}m behind Hit]')
        self.get_logger().info(f'   Hit Point (H):        ({self.hit_x:.2f}, {self.hit_y:.2f}) [Current Robot Position]')
        self.get_logger().info(f'   Target Point (G):     ({self.target_x:.2f}, {self.target_y:.2f}) [{self.target_distance:.1f}m ahead along heading]')
        self.get_logger().info(f'   Follow Side:          {"LEFT" if self.follow_side == 1 else "RIGHT"}')
        self.get_logger().info('=' * 70)

        # Send the goal
        self.send_wall_follow_goal()

    def send_goal_once(self):
        """Timer callback that triggers goal submission once."""
        self._timer.cancel()
        self.send_wall_follow_goal()

    def send_wall_follow_goal(self):
        """Asynchronously sends the Bug2WallFollow goal to the action server."""
        self.get_logger().info(f'Waiting for action server {self.action_name}...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server {self.action_name} not available! Aborting goal submission.')
            return

        # Construct goal message
        goal_msg = Bug2WallFollow.Goal()
        goal_msg.start_point = Point(x=self.start_x, y=self.start_y, z=0.0)
        goal_msg.target_point = Point(x=self.target_x, y=self.target_y, z=0.0)
        goal_msg.hit_point = Point(x=self.hit_x, y=self.hit_y, z=0.0)
        goal_msg.follow_side = int(self.follow_side)

        self.get_logger().info('🚀 Sending Bug2WallFollow goal to Action Server...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # -------------------------------------------------------------------------
    # ACTION CALLBACKS (Non-Blocking)
    # -------------------------------------------------------------------------
    def goal_response_callback(self, future):
        """Called when server accepts or rejects the goal."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('❌ Bug2WallFollow goal was REJECTED by the server!')
            return

        self.get_logger().info('✅ Bug2WallFollow goal was ACCEPTED by the server. Wall-following active!')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Called when live feedback is received from the action server."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'📡 [Feedback] Distance to M-Line: {fb.distance_to_m_line*100:5.1f} cm | '
            f'Distance to Target: {fb.distance_to_target:.2f} m',
            throttle_duration_sec=1.0
        )

    def get_result_callback(self, future):
        """Called when the action finishes."""
        result_response = future.result()
        status = result_response.status
        result = result_response.result

        self.get_logger().info('=' * 70)
        if result.success:
            self.get_logger().info('🎉 [ACTION SUCCEEDED] Wall-following reached a valid Bug2 leave point!')
            self.get_logger().info(f'   Server Message:\n{result.message}')
            self.get_logger().info('➡️  [MODE TRANSITION] Transitioning to GOAL TRACKING (Line Follow Mode)!')
        else:
            self.get_logger().warn(f'⚠️ [ACTION FINISHED WITH STATUS {status}]')
            self.get_logger().warn(f'   Server Message: {result.message}')
        self.get_logger().info('=' * 70)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LineFollowNode shutting down safely...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

