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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from create3_interfaces.action import Bug2WallFollow


class LineFollowNode(Node):
    def __init__(self):
        super().__init__('line_follow_node')

        # ---------------------------------------------------------------------
        # 1. PARAMETERS (Example coordinates for demonstration)
        # ---------------------------------------------------------------------
        # NOTE: In a full integration, these coordinates are dynamically provided
        # by the line follower / obstacle detector when an obstacle is encountered.
        self.declare_parameter('action_name', '/robot_9/bug2_wall_follow')
        self.declare_parameter('start_x', 0.0)      # Example Start Point M (x)
        self.declare_parameter('start_y', 0.0)      # Example Start Point M (y)
        self.declare_parameter('target_x', 2.0)     # Example Target Point G (x)
        self.declare_parameter('target_y', 1.0)     # Example Target Point G (y)
        self.declare_parameter('hit_x', 0.8)        # Example Hit Point H (x)
        self.declare_parameter('hit_y', 0.4)        # Example Hit Point H (y)
        self.declare_parameter('follow_side', -1)   # -1: Right wall, +1: Left wall

        self.action_name = str(self.get_parameter('action_name').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.hit_x = float(self.get_parameter('hit_x').value)
        self.hit_y = float(self.get_parameter('hit_y').value)
        self.follow_side = int(self.get_parameter('follow_side').value)

        # ---------------------------------------------------------------------
        # 2. ACTION CLIENT SETUP
        # ---------------------------------------------------------------------
        self._action_client = ActionClient(self, Bug2WallFollow, self.action_name)
        self._goal_handle = None

        self.get_logger().info('=' * 70)
        self.get_logger().info('🤖 [LineFollowNode] Initialized!')
        self.get_logger().info(f'   Target Action Server: {self.action_name}')
        self.get_logger().info(f'   Start Point (M):      ({self.start_x:.2f}, {self.start_y:.2f})')
        self.get_logger().info(f'   Target Point (G):     ({self.target_x:.2f}, {self.target_y:.2f})')
        self.get_logger().info(f'   Hit Point (H):        ({self.hit_x:.2f}, {self.hit_y:.2f})')
        self.get_logger().info(f'   Follow Side:          {"LEFT" if self.follow_side == 1 else "RIGHT"}')
        self.get_logger().info('=' * 70)

        # Start goal submission after a brief timer to allow action server discovery
        self._timer = self.create_timer(1.0, self.send_goal_once)

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

