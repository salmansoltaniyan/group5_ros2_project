#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection

class HazardSubscriber(Node):
    def __init__(self):
        super().__init__("hazard_subscriber")
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to hazard detection (Bumpers, Cliffs, Wheel Drops, etc.)
        self.sub = self.create_subscription(
            HazardDetectionVector, 
            "/robot_9/hazard_detection", 
            self.callback, 
            qos
        )
        self.get_logger().info("Subscribed to /robot_9/hazard_detection. Waiting for hazards...")

    def callback(self, msg):
        # msg.detections is a list of hazards currently detected.
        # If the list is empty, there are no hazards (everything is safe).
        if not msg.detections:
            return 
            
        for detection in msg.detections:
            # =========================================================================
            # IMPORTANT EXTRACTABLE DATA:
            # - detection.type: Constant indicating the type (BUMP, CLIFF, STALL, WHEEL_DROP, etc.)
            # - detection.header.frame_id: Which physical sensor triggered it (e.g., 'bump_right')
            # =========================================================================
            
            hazard_type = detection.type
            hazard_name = detection.header.frame_id 
            
            if hazard_type == HazardDetection.BUMP:
                self.get_logger().warn(f"BUMP DETECTED at {hazard_name}!")
            elif hazard_type == HazardDetection.CLIFF:
                self.get_logger().warn(f"CLIFF DETECTED at {hazard_name} (Don't fall!)")
            elif hazard_type == HazardDetection.WHEEL_DROP:
                self.get_logger().warn(f"ROBOT LIFTED! Wheel dropped at {hazard_name}")
            else:
                self.get_logger().info(f"Other hazard detected: type {hazard_type} at {hazard_name}")


def main(args=None):
    rclpy.init(args=args)
    node = HazardSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

