#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock

class DockClient(Node):
    def __init__(self):
        super().__init__('dock_client')
        self._action_client = ActionClient(self, Dock, '/robot_9/dock')

    def send_goal(self):
        self._action_client.wait_for_server()

        # Goal Setup for Docking (No parameters needed! The robot knows what to do)
        goal_msg = Dock.Goal()

        self.get_logger().info('Sending Goal: Go back to charging dock...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Docking feedback might be simple states (e.g. searching, aligning, parking)
        self.get_logger().info('Feedback: Robot is maneuvering to dock...')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED.')
            return
        self.get_logger().info('Goal ACCEPTED.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        
        # Now we actually check if the robot succeeded or gave up!
        if getattr(result, 'is_docked', False):
            self.get_logger().info('🏁 SUCCESS: Robot is now DOCKED and charging! 🏁')
        else:
            self.get_logger().error('❌ FAILED: Robot gave up! It could not find the IR signal of the dock. ❌')
            
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = DockClient()
    client.send_goal()
    rclpy.spin(client)

if __name__ == '__main__':
    main()

