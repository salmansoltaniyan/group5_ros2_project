#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import EStop

class EStopOnClient(Node):
    def __init__(self):
        super().__init__('estop_on_client')
        
        # =========================================================================
        # WHAT IS A SERVICE CLIENT?
        # Unlike Publishers (which yell data continuously) and Subscribers (which 
        # listen continuously), a Service Client makes a ONE-TIME request to a 
        # Server, waits for the server to do the job, and gets a confirmation.
        # =========================================================================
        
        # STEP 1: Create the Client
        # We specify the service type (EStop) and the exact service name ('/robot_9/e_stop')
        self.client = self.create_client(EStop, '/robot_9/e_stop')
        
        # STEP 2: Wait for the Server
        # The robot might be booting up. We must wait until the service is actually available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /robot_9/e_stop service to be ready...')
            
        # Once ready, we trigger our function to send the request
        self.send_request()

    def send_request(self):
        # STEP 3: Create the Request Object
        # Every service has a specific Request format. For EStop, it expects a boolean field.
        request = EStop.Request()
        
        # We want to turn the Emergency Stop ON (True)
        request.e_stop_on = True 
        
        # STEP 4: Send the Request Asynchronously
        # call_async sends the request without freezing our program. 
        # The result will be stored in 'self.future' when the robot replies.
        self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = EStopOnClient()
    
    # STEP 5: Wait for the Result
    # spin_until_future_complete pauses here UNTIL the robot replies "Done!"
    rclpy.spin_until_future_complete(node, node.future)
    
    # Print a success message
    node.get_logger().warn('🚨 EMERGENCY STOP ACTIVATED! Motors are now locked. 🚨')
    
    # Clean up and exit the program (Services usually run once and exit)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
