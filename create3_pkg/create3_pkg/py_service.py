#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class StopLidarClient(Node):
    def __init__(self):
        super().__init__('stop_lidar_client')
        
        # Create client to stop the LiDAR motor using the 'Empty' service type
        self.client = self.create_client(Empty, '/stop_motor')
        
        # Keep pinging until the service is found
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /stop_motor service to be ready...')
            
        self.send_request()

    def send_request(self):
        # Create an Empty request (no variables needed, just acts as a trigger)
        request = Empty.Request()
        
        # Send the trigger to the robot
        self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = StopLidarClient()
    
    # Wait for confirmation that the motor has completely stopped
    rclpy.spin_until_future_complete(node, node.future)
    
    node.get_logger().info('🛑 LiDAR Motor STOPPED! (Saves battery and motor life) 🛑')
    
    # Exit program
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
