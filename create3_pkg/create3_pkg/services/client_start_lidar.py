#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class StartLidarClient(Node):
    def __init__(self):
        super().__init__('start_lidar_client')
        
        # =========================================================================
        # THE "EMPTY" SERVICE TYPE
        # Many hardware services in ROS don't need any variables (like True/False 
        # or X/Y coordinates). They just act like a simple push-button.
        # For these, ROS uses a standard service type called 'Empty' from 'std_srvs'.
        # =========================================================================
        
        # 1. Create client using the Empty service type
        self.client = self.create_client(Empty, '/start_motor')
        
        # 2. Wait for LiDAR service to be ready
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /start_motor service to be ready...')
            
        self.send_request()

    def send_request(self):
        # 3. Create the Request Object
        # Since the type is 'Empty', we don't set any variables inside 'request'!
        request = Empty.Request()
        
        # 4. Send the request
        self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = StartLidarClient()
    
    # 5. Wait for the LiDAR driver to confirm the motor has started
    rclpy.spin_until_future_complete(node, node.future)
    
    node.get_logger().info('🔄 LiDAR Motor STARTED! 🔄')
    
    # Exit program
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
