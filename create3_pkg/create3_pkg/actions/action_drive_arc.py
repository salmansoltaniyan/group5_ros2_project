#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveArc
import math

class DriveArcClient(Node):
    def __init__(self):
        super().__init__('drive_arc_client')
        self._action_client = ActionClient(self, DriveArc, '/robot_9/drive_arc')

    def send_goal(self):
        self._action_client.wait_for_server()

        goal_msg = DriveArc.Goal()
        # Draw a semi-circle!
        # Angle: 180 degrees (in radians)
        goal_msg.angle = math.radians(180)
        # Radius of the circle (0.5 meters)
        goal_msg.radius = 0.5
        # 1 means forward, -1 means backwards
        goal_msg.translate_direction = 1 
        goal_msg.max_translation_speed = 0.3

        self.get_logger().info('Sending Goal: Drive in a half-circle (arc) ...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: Remaining angle in arc: {math.degrees(feedback_msg.feedback.remaining_angle_travel):.1f} deg')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED.')
            return
        self.get_logger().info('Goal ACCEPTED.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('🏁 ARC COMPLETED! 🏁')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = DriveArcClient()
    client.send_goal()
    rclpy.spin(client)

if __name__ == '__main__':
    main()

