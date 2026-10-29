#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock

class UndockClient(Node):
    def __init__(self):
        super().__init__('undock_client')
        self._action_client = ActionClient(self, Undock, '/robot_9/undock')

    def send_goal(self):
        self._action_client.wait_for_server()

        # Goal Setup for Undocking (No parameters needed)
        goal_msg = Undock.Goal()

        self.get_logger().info('Sending Goal: Leave the charging dock...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Feedback: Robot is backing up from the dock...')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED.')
            return
        self.get_logger().info('Goal ACCEPTED.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('🏁 Robot has UNDOCKED and is ready to drive! 🏁')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = UndockClient()
    client.send_goal()
    rclpy.spin(client)

if __name__ == '__main__':
    main()

