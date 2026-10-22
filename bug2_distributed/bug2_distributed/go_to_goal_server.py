#!/usr/bin/env python3
"""
================================================================================
GoToGoal Action Server for iRobot Create 3 - ROS 2 Jazzy
================================================================================
Node: go_to_goal_server
Action: /go_to_goal (create3_interfaces/action/GoToGoal)

Role in Bug2 Architecture:
- Directs the robot straight towards the goal coordinate.
- Subscribes to Odometry for current pose and LiDAR for front obstacle checks.
- If goal is reached (distance < goal_tolerance):
    Returns result.reached_goal = True.
- If an obstacle is detected in front:
    Halts robot, records hit_point, returns result.obstacle_detected = True.
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

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor

from create3_interfaces.action import GoToGoal


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


class GoToGoalServer(Node):
    def __init__(self):
        super().__init__('go_to_goal_server')

        flex = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('namespace',             'robot_9')
        self.declare_parameter('linear_speed',          0.15, flex)
        self.declare_parameter('angular_speed',         0.50, flex)
        self.declare_parameter('goal_tolerance',        0.15, flex)
        self.declare_parameter('front_dist_threshold',  0.26, flex)
        self.declare_parameter('corner_dist_threshold', 0.26, flex)

        ns = str(self.get_parameter('namespace').value)
        self.linear_speed   = float(self.get_parameter('linear_speed').value)
        self.angular_speed  = float(self.get_parameter('angular_speed').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.front_dist_thr = float(self.get_parameter('front_dist_threshold').value)
        self.corner_dist_thr= float(self.get_parameter('corner_dist_threshold').value)

        # Pose & Sensor Cache
        self.curr_x   = 0.0
        self.curr_y   = 0.0
        self.curr_yaw = 0.0
        self.odom_ready = False

        self.lidar_front       = float('inf')
        self.lidar_front_right = float('inf')
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
            GoToGoal,
            'go_to_goal',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('GoToGoal Action Server initialized.')

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

        f_min = fr_min = float('inf')
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
            elif -60.0 <= angle_deg < -20.0:
                fr_min = min(fr_min, r)

        self.lidar_front       = f_min
        self.lidar_front_right = fr_min
        self.scan_ready        = True

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received GoToGoal target: ({goal_request.target_point.x:.2f}, {goal_request.target_point.y:.2f})')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('GoToGoal execution canceled by client.')
        self._stop()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_x = goal_handle.request.target_point.x
        target_y = goal_handle.request.target_point.y

        feedback_msg = GoToGoal.Feedback()
        result       = GoToGoal.Result()

        rate = self.create_rate(10.0)

        # Wait until odometry is available
        while not self.odom_ready:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop()
                result.message = 'Canceled while waiting for odom'
                return result
            time.sleep(0.05)

        self.get_logger().info(f'Navigating towards goal ({target_x:.2f}, {target_y:.2f})...')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop()
                result.message = 'Canceled'
                return result

            dist = math.hypot(target_x - self.curr_x, target_y - self.curr_y)
            feedback_msg.distance_to_target = float(dist)
            goal_handle.publish_feedback(feedback_msg)

            # 1. Goal Reached Check
            if dist < self.goal_tolerance:
                self._stop()
                result.reached_goal = True
                result.obstacle_detected = False
                result.message = 'Target reached successfully!'
                goal_handle.succeed()
                self.get_logger().info('★ GOAL REACHED! ★')
                return result

            # 2. Heading alignment and Obstacle Check
            angle_to_goal = math.atan2(target_y - self.curr_y, target_x - self.curr_x)
            err = _angle_diff(angle_to_goal, self.curr_yaw)

            # Check obstacle when reasonably facing goal direction
            if abs(err) <= 0.60:
                if self.lidar_front <= self.front_dist_thr or self.lidar_front_right <= self.corner_dist_thr:
                    self._stop()
                    result.reached_goal = False
                    result.obstacle_detected = True
                    result.hit_point = Point(x=self.curr_x, y=self.curr_y, z=0.0)
                    result.message = f'Obstacle detected at ({self.curr_x:.2f}, {self.curr_y:.2f})'
                    goal_handle.succeed()
                    self.get_logger().info(f'[HIT] Obstacle encountered at ({self.curr_x:.2f}, {self.curr_y:.2f})!')
                    return result

            # Motion control
            t = Twist()
            if abs(err) > 0.30:
                t.linear.x  = 0.0
                t.angular.z = float(self.angular_speed * (1.0 if err > 0 else -1.0))
            else:
                kp = 1.2
                t.linear.x  = float(self.linear_speed)
                t.angular.z = float(max(-self.angular_speed, min(self.angular_speed, kp * err)))

            self.cmd_pub.publish(t)
            time.sleep(0.10)

        result.message = 'ROS shutdown'
        return result


def main(args=None):
    rclpy.init(args=args)
    server = GoToGoalServer()
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

