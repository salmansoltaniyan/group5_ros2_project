#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import Mouse

class MouseSubscriber(Node):
    def __init__(self):
        super().__init__("mouse_subscriber")
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to the optical flow mouse sensor underneath the robot
        self.sub = self.create_subscription(
            Mouse, 
            "/robot_9/mouse", 
            self.callback, 
            qos
        )
        self.get_logger().info("Subscribed to /robot_9/mouse. Waiting for optical flow data...")

    def callback(self, msg):
        # =========================================================================
        # IMPORTANT EXTRACTABLE DATA (MOUSE):
        # - msg.integrated_x: Cumulative distance moved in the X direction (in meters).
        #                     Measured purely by looking at the floor's texture.
        # - msg.integrated_y: Cumulative distance moved in the Y direction (in meters).
        # - msg.frame_id: Usually 'mouse'.
        #
        # NOTE: Unlike Odometry (which uses wheel rotations), this sensor acts like a
        # computer mouse. It won't get tricked if the robot's wheels slip on a smooth floor!
        # =========================================================================
        
        # Extracting the accumulated distance
        dist_x = msg.integrated_x
        dist_y = msg.integrated_y
        
        self.get_logger().info(f"Actual floor movement -> X: {dist_x:.3f}m | Y: {dist_y:.3f}m")


def main(args=None):
    rclpy.init(args=args)
    node = MouseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

