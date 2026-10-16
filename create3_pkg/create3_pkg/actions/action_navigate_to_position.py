#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import NavigateToPosition

class NavigatePositionClient(Node):
    def __init__(self):
        super().__init__('navigate_position_client')
        self._action_client = ActionClient(self, NavigateToPosition, '/robot_9/navigate_to_position')

    def send_goal(self, x, y):
        self._action_client.wait_for_server()

        goal_msg = NavigateToPosition.Goal()
        # Set the target coordinates (X, Y) in meters based on the odometry starting point
        goal_msg.goal_pose.pose.position.x = float(x)
        goal_msg.goal_pose.pose.position.y = float(y)
        
        # We also need to specify if we care about the final angle (achieve_goal_heading)
        goal_msg.achieve_goal_heading = False 
        
        goal_msg.max_translation_speed = 0.3

        self.get_logger().info(f'Sending Goal: Navigate to X: {x}, Y: {y} ...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Feedback contains navigation states
        self.get_logger().info(f'Feedback: Navigating to position. Remaining travel: {feedback_msg.feedback.remaining_travel_distance:.2f}m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED.')
            return
        self.get_logger().info('Goal ACCEPTED.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('🏁 ARRIVED AT DESTINATION! 🏁')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = NavigatePositionClient()
    # Tell the robot to go to coordinate (1.5, 0.5)
    client.send_goal(1.5, 0.5)
    rclpy.spin(client)

if __name__ == '__main__':
    main()

