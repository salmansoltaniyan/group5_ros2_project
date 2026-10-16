#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance

class DriveDistanceClient(Node):
    def __init__(self):
        super().__init__('drive_distance_client')
        
        # =========================================================================
        # STEP 1: CREATE ACTION CLIENT
        # We specify the Action Type (DriveDistance) and the Action Server Name.
        # =========================================================================
        self._action_client = ActionClient(self, DriveDistance, '/robot_9/drive_distance')

    def send_goal(self, dist_meters):
        self.get_logger().info('Waiting for Action Server to be online...')
        self._action_client.wait_for_server()

        # =========================================================================
        # STEP 2: CREATE THE GOAL
        # Unlike Services, Actions use a 'Goal' object. We set our target here.
        # =========================================================================
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = float(dist_meters)      # How far to drive in meters
        goal_msg.max_translation_speed = 0.3        # Max speed in meters/second

        self.get_logger().info(f'Sending Goal: Drive forward {dist_meters} meters...')
        
        # =========================================================================
        # STEP 3: SEND GOAL ASYNCHRONOUSLY
        # We send the goal and attach a 'feedback_callback' to receive live updates.
        # =========================================================================
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        # Check if the server accepted our goal
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # =========================================================================
        # STEP 4: HANDLE CONTINUOUS FEEDBACK
        # This function runs automatically every time the robot sends an update!
        # =========================================================================
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: Remaining distance: {feedback.remaining_travel_distance:.2f} m')

    def goal_response_callback(self, future):
        # =========================================================================
        # STEP 5: GOAL ACCEPTANCE
        # Check if the robot rejected the goal (e.g., if it's already doing something else)
        # =========================================================================
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was REJECTED by the robot.')
            return

        self.get_logger().info('Goal ACCEPTED! Robot is executing...')
        
        # Ask to be notified when the action is completely finished
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # =========================================================================
        # STEP 6: FINAL RESULT
        # This runs when the robot successfully reaches the target or fails.
        # =========================================================================
        result = future.result().result
        self.get_logger().info('🏁 Action COMPLETELY FINISHED! 🏁')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = DriveDistanceClient()
    
    # We want the robot to drive exactly 1.0 meter forward
    client.send_goal(1.0)
    
    # Spin keeps the program running so it can receive the callbacks
    rclpy.spin(client)

if __name__ == '__main__':
    main()

