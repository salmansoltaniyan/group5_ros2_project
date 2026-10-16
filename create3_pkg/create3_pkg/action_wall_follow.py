#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import WallFollow
from builtin_interfaces.msg import Duration

class WallFollowClient(Node):
    def __init__(self):
        super().__init__('wall_follow_client')
        self._action_client = ActionClient(self, WallFollow, '/robot_9/wall_follow')

    def send_goal(self):
        self._action_client.wait_for_server()

        goal_msg = WallFollow.Goal()
        # follow_side: 1 means follow wall on the LEFT, -1 means follow wall on the RIGHT
        goal_msg.follow_side = 1 
        
        # Max runtime: How long should it follow the wall? (Let's do 10 seconds)
        goal_msg.max_runtime.sec = 100
        goal_msg.max_runtime.nanosec = 0

        self.get_logger().info('Sending Goal: Follow the LEFT wall for 10 seconds...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Feedback: Still following the wall...')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED.')
            return
        self.get_logger().info('Goal ACCEPTED.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('🏁 Finished following the wall (Time is up)! 🏁')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = WallFollowClient()
    client.send_goal()
    rclpy.spin(client)

if __name__ == '__main__':
    main()

