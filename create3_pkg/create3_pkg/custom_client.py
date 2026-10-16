#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MyCustomClient(Node):
    def __init__(self):
        super().__init__('my_custom_client')
        
        # Connect to our custom server!
        self.client = self.create_client(SetBool, '/do_my_trick')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /do_my_trick server to be started...')
            
        self.send_request()

    def send_request(self):
        request = SetBool.Request()
        
        # We send True (which acts as '1')
        request.data = True 
        
        self.get_logger().info("Sending command '1' to the server...")
        self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomClient()
    
    # Wait here while the server takes 4-5 seconds to do the trick
    rclpy.spin_until_future_complete(node, node.future)
    
    # Check the response from the server
    if node.future.result() is not None:
        node.get_logger().info(f"Server replied: {node.future.result().message}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
