#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

# We import the standard SetBool (True/False or 1/0) and Empty types
from std_srvs.srv import SetBool, Empty
from geometry_msgs.msg import Twist

class MyCustomServer(Node):
    def __init__(self):
        super().__init__('my_custom_server')
        
        # 1. SERVER: Create a service called /do_my_trick
        # When someone calls this, the 'handle_request' function runs
        self.srv = self.create_service(SetBool, '/do_my_trick', self.handle_request)
        
        # 2. CLIENT: Prepare clients to control the LiDAR
        self.lidar_start = self.create_client(Empty, '/start_motor')
        self.lidar_stop = self.create_client(Empty, '/stop_motor')
        
        # 3. PUBLISHER: Prepare publisher to turn the robot
        self.cmd_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)
        
        self.get_logger().info('🎯 Custom Server is READY! Waiting for the command (True/1)...')

    def handle_request(self, request, response):
        # Check if the user sent True (which equals 1)
        if request.data == True:
            self.get_logger().info("Received command '1'! Starting the sequence...")
            
            # --- ACTION A: Start LiDAR ---
            self.get_logger().info("Step 1: Starting LiDAR...")
            self.lidar_start.call_async(Empty.Request())
            time.sleep(1.0) # Wait 1 second for LiDAR motor to spin up
            
            # --- ACTION B: Rotate 180 degrees ---
            self.get_logger().info("Step 2: Rotating 180 degrees...")
            twist = Twist()
            # Turn at 1.0 radian per second
            twist.angular.z = 1.0 
            
            # Pi radians = 180 degrees (Pi = ~3.14)
            # So if we turn at 1 rad/s, we need to wait 3.14 seconds!
            start_time = time.time()
            while time.time() - start_time < 3.14:
                self.cmd_pub.publish(twist)
                time.sleep(0.1) # Keep publishing every 0.1s so it doesn't stop
                
            # Stop the robot's rotation
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            
            # --- ACTION C: Stop LiDAR ---
            self.get_logger().info("Step 3: Stopping LiDAR...")
            self.lidar_stop.call_async(Empty.Request())
            
            # Formulate the response to send back to the user
            response.success = True
            response.message = "Trick completed successfully!"
            
        else:
            response.success = False
            response.message = "I only respond to True (1)!"
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

