#!/usr/bin/env python3
"""
================================================================================
Bug2 Coordinator (Brain Node) for iRobot Create 3 - ROS 2 Jazzy
================================================================================
Node: bug2_coordinator
Clients:
  - /go_to_goal   (GoToGoal Action Client)
  - /wall_follow  (Bug2WallFollow Action Client)

Architecture:
- Acts as the central state machine / orchestrator for Bug2 navigation.
- Receives user inputs (goal_x, goal_y) relative to the robot's initial pose.
- Computes global world frame Start and Goal coordinates.
- Switches seamlessly between GoToGoal and WallFollow actions until Goal is reached.
================================================================================
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor

from create3_interfaces.action import GoToGoal, Bug2WallFollow


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Bug2Coordinator(Node):
    def __init__(self):
        super().__init__('bug2_coordinator')

        flex = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('namespace',  'robot_9')
        self.declare_parameter('goal_x',     2.0, flex)
        self.declare_parameter('goal_y',     0.0, flex)

        ns = str(self.get_parameter('namespace').value)
        self.goal_dx = float(self.get_parameter('goal_x').value)
        self.goal_dy = float(self.get_parameter('goal_y').value)

        # Action Clients
        self.goto_client = ActionClient(self, GoToGoal, 'go_to_goal')
        self.wall_client = ActionClient(self, Bug2WallFollow, 'wall_follow')

        # World coordinates
        self.start_x   = 0.0
        self.start_y   = 0.0
        self.start_yaw = 0.0
        self.goal_x    = 0.0
        self.goal_y    = 0.0

        self.curr_x    = 0.0
        self.curr_y    = 0.0
        self.curr_yaw  = 0.0

        self.odom_ready     = False
        self.start_recorded = False

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, f'/{ns}/odom', self._odom_cb, sensor_qos)

        self.get_logger().info('=' * 65)
        self.get_logger().info('  Bug2 Distributed Coordinator Initialized')
        self.get_logger().info(f'  Target offset: dx={self.goal_dx:.2f}m, dy={self.goal_dy:.2f}m')
        self.get_logger().info('  Waiting for Odometry and Action Servers...')
        self.get_logger().info('=' * 65)

        # Run coordination in a dedicated background thread
        self.coord_thread = threading.Thread(target=self._run_coordinator)
        self.coord_thread.daemon = True
        self.coord_thread.start()

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.curr_x   = pos.x
        self.curr_y   = pos.y
        self.curr_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ready = True

        if not self.start_recorded:
            self._record_start()

    def _record_start(self):
        self.start_x = self.curr_x
        self.start_y = self.curr_y
        self.start_yaw = self.curr_yaw
        self.start_recorded = True

        # Rotate goal by start_yaw
        cos_yaw = math.cos(self.start_yaw)
        sin_yaw = math.sin(self.start_yaw)
        world_dx = self.goal_dx * cos_yaw - self.goal_dy * sin_yaw
        world_dy = self.goal_dx * sin_yaw + self.goal_dy * cos_yaw

        self.goal_x = self.start_x + world_dx
        self.goal_y = self.start_y + world_dy

        self.get_logger().info('Initial Pose Recorded:')
        self.get_logger().info(f'  Start Pos: ({self.start_x:.3f}, {self.start_y:.3f}) | Yaw: {math.degrees(self.start_yaw):.1f}°')
        self.get_logger().info(f'  Goal  Pos: ({self.goal_x:.3f}, {self.goal_y:.3f})')

    def _run_coordinator(self):
        # 1. Wait for Start & Servers
        while rclpy.ok() and not self.start_recorded:
            time.sleep(0.1)

        self.get_logger().info('Waiting for GoToGoal Action Server...')
        self.goto_client.wait_for_server()
        self.get_logger().info('GoToGoal Server connected.')

        self.get_logger().info('Waiting for WallFollow Action Server...')
        self.wall_client.wait_for_server()
        self.get_logger().info('WallFollow Server connected.')

        self.get_logger().info('>>> STARTING BUG2 AUTONOMOUS NAVIGATION <<<')

        # Bug2 State Machine Loop
        while rclpy.ok():
            # ─────────────────────────────────────────────────────────────
            # STEP 1: Execute GoToGoal
            # ─────────────────────────────────────────────────────────────
            self.get_logger().info('\n[STATE: GO_TO_GOAL] Calling GoToGoal Action...')
            goto_goal = GoToGoal.Goal()
            goto_goal.target_point = Point(x=self.goal_x, y=self.goal_y, z=0.0)

            send_goal_future = self.goto_client.send_goal_async(goto_goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().error('GoToGoal goal was rejected by server!')
                break

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result

            if result.reached_goal:
                self.get_logger().info('=' * 65)
                self.get_logger().info('★ ★ ★ BUG2 NAVIGATION COMPLETE: GOAL REACHED! ★ ★ ★')
                self.get_logger().info('=' * 65)
                break

            if result.obstacle_detected:
                hit_x = result.hit_point.x
                hit_y = result.hit_point.y
                self.get_logger().info(f'[OBSTACLE HIT] Hit point recorded at ({hit_x:.2f}, {hit_y:.2f})')
            else:
                self.get_logger().warn(f'GoToGoal finished unexpectedly: {result.message}')
                break

            # ─────────────────────────────────────────────────────────────
            # STEP 2: Execute WallFollow
            # ─────────────────────────────────────────────────────────────
            self.get_logger().info('\n[STATE: WALL_FOLLOW] Calling WallFollow Action...')
            wf_goal = Bug2WallFollow.Goal()
            wf_goal.start_point  = Point(x=self.start_x, y=self.start_y, z=0.0)
            wf_goal.target_point = Point(x=self.goal_x,  y=self.goal_y,  z=0.0)
            wf_goal.hit_point    = Point(x=hit_x,        y=hit_y,        z=0.0)
            wf_goal.follow_side  = -1  # Right side

            wf_future = self.wall_client.send_goal_async(wf_goal)
            rclpy.spin_until_future_complete(self, wf_future)
            wf_handle = wf_future.result()

            if not wf_handle.accepted:
                self.get_logger().error('WallFollow goal was rejected by server!')
                break

            wf_result_future = wf_handle.get_result_async()
            rclpy.spin_until_future_complete(self, wf_result_future)
            wf_result = wf_result_future.result().result

            if wf_result.success:
                self.get_logger().info(f'[WALL FOLLOW COMPLETE] {wf_result.message}')
                self.get_logger().info('Switching back to GoToGoal on the M-line...\n')
            else:
                self.get_logger().warn(f'WallFollow aborted: {wf_result.message}')
                break


def main(args=None):
    rclpy.init(args=args)
    coordinator = Bug2Coordinator()
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.get_logger().info('Shutting down Bug2 Coordinator...')
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

