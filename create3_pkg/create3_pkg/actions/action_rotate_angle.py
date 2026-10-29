#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
import math

class RotateAngleClient(Node):
    def __init__(self):
        super().__init__('rotate_angle_client')
        self._action_client = ActionClient(self, RotateAngle, '/robot_9/rotate_angle')

    def send_goal(self, angle_degrees):
        self._action_client.wait_for_server()

        # Goal Setup
        goal_msg = RotateAngle.Goal()
        # The robot expects radians, so we convert degrees to radians
        goal_msg.angle = math.radians(angle_degrees) 
        goal_msg.max_rotation_speed = 1.0 # Max speed in radians/second

        self.get_logger().info(f'Sending Goal: Rotate {angle_degrees} degrees...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Feedback gives us remaining angle in radians, we convert back to degrees for reading
        remaining_deg = math.degrees(feedback_msg.feedback.remaining_angle_travel)
        self.get_logger().info(f'Feedback: Remaining angle: {remaining_deg:.1f} degrees')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED.')
            return
        self.get_logger().info('Goal ACCEPTED.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('🏁 Rotation FINISHED! 🏁')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = RotateAngleClient()
    # Rotate exactly 180 degrees
    client.send_goal(180.0)
    rclpy.spin(client)

if __name__ == '__main__':
    main()

