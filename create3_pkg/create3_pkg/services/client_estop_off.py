#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import EStop

class EStopOffClient(Node):
    def __init__(self):
        super().__init__('estop_off_client')
        
        # =========================================================================
        # E-STOP OFF SERVICE CLIENT
        # This code is identical to turning the E-Stop ON, except we will change 
        # the request value to False to unlock the robot's motors.
        # =========================================================================
        
        # Create the client for the EStop service
        self.client = self.create_client(EStop, '/robot_9/e_stop')
        
        # Wait until the robot's service is online
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /robot_9/e_stop service to be ready...')
            
        self.send_request()

    def send_request(self):
        # Create the request template
        request = EStop.Request()
        
        # Set the emergency stop state to FALSE (Turn it off / Release motors)
        request.e_stop_on = False 
        
        # Send the request in the background
        self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = EStopOffClient()
    
    # Pause the code here until the robot confirms the motors are unlocked
    rclpy.spin_until_future_complete(node, node.future)
    
    node.get_logger().info('✅ EMERGENCY STOP RELEASED! Motors are free to move. ✅')
    
    # Shut down cleanly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
