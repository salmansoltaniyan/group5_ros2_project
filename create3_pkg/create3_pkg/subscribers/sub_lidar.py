#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import math

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to the LiDAR scan topic.
        self.sub = self.create_subscription(
            LaserScan, 
            "/scan", 
            self.callback, 
            qos
        )
        self.get_logger().info("Subscribed to LiDAR (/robot_9/scan). Waiting for laser data...")

    def get_distance_at_angle(self, msg, target_angle_deg):
        """
        Helper function to calculate the distance at a specific angle (0, 90, 180, 270)
        """
        # 1. Convert target degree to radians
        target_rad = math.radians(target_angle_deg)
        
        # 2. Calculate the exact index in the ranges array
        # Formula: index = (Target Angle - Minimum Angle) / Angle Increment
        raw_index = int((target_rad - msg.angle_min) / msg.angle_increment)
        
        # 3. Wrap the index to handle different LiDAR formats (0 to 360 vs -180 to 180)
        index = raw_index % len(msg.ranges)
        
        dist = msg.ranges[index]
        
        # 4. Check if the reading is valid
        if math.isinf(dist) or math.isnan(dist) or dist < msg.range_min:
            return float('inf') # Return infinity if there is no obstacle
            
        return dist

    def callback(self, msg):
        # =========================================================================
        # IMPORTANT EXTRACTABLE DATA (LIDAR - LaserScan):
        # - msg.ranges: A massive list of distances (in meters) for every angle.
        # - msg.angle_min: The starting angle of the scan (in radians).
        # - msg.angle_max: The ending angle of the scan (in radians).
        # - msg.angle_increment: Angle between each measurement in the ranges list.
        # - msg.range_min / msg.range_max: The physical min/max range of the sensor.
        #
        # Note: If the laser hits nothing, the value in `ranges` is usually 'inf' (infinity).
        # =========================================================================
        
        ranges = msg.ranges
        if not ranges:
            return
            
        # 1. Filter the data to ignore 'inf' (infinity) and 'NaN' (Not a Number) values
        valid_ranges = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r >= msg.range_min]
        
        if valid_ranges:
            closest_dist = min(valid_ranges)
        else:
            closest_dist = float('inf')

        # 2. Get specific distances at 0, 90, 180, 270 degrees
        dist_0   = self.get_distance_at_angle(msg, 0)
        dist_90  = self.get_distance_at_angle(msg, 90)
        dist_180 = self.get_distance_at_angle(msg, 180)
        dist_270 = self.get_distance_at_angle(msg, 270)
        
        # 3. Print the results nicely
        self.get_logger().info(
            f"\n"
            f"=== LIDAR DISTANCES ===\n"
            f"Closest Overall: {closest_dist:.2f} m\n"
            f"Front (0°)     : {dist_0:.2f} m\n"
            f"Left  (90°)    : {dist_90:.2f} m\n"
            f"Back  (180°)   : {dist_180:.2f} m\n"
            f"Right (270°)   : {dist_270:.2f} m\n"
            f"======================="
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
