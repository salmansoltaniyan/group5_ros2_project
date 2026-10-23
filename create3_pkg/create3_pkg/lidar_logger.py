import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarLogger(Node):
    def __init__(self):
        super().__init__('lidar_logger')
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info('Logging only. The robot will NOT move.')

    def scan_callback(self, msg: LaserScan):
        front       = self.sector_distance(msg, center_deg=0.0,   half_width_deg=10.0)
        front_right = self.sector_distance(msg, center_deg=-45.0, half_width_deg=10.0)
        right       = self.sector_distance(msg, center_deg=-90.0, half_width_deg=10.0)

        # For a wall running parallel to the robot, front_right / right should be about 1.4
        if math.isfinite(front_right) and math.isfinite(right) and right > 0:
            ratio = front_right / right
        else:
            ratio = float('nan')

        self.get_logger().info(
            f'front: {front:.2f}  front_right: {front_right:.2f}  '
            f'right: {right:.2f}  ratio: {ratio:.2f}',
            throttle_duration_sec=0.5)

    def sector_distance(self, msg, center_deg, half_width_deg):
        """Closest valid reading inside a slice of the scan."""
        center = math.radians(center_deg)
        half = math.radians(half_width_deg)

        start = int((center - half - msg.angle_min) / msg.angle_increment)
        end = int((center + half - msg.angle_min) / msg.angle_increment)
        start = max(start, 0)
        end = min(end, len(msg.ranges) - 1)

        valid = [
            r for r in msg.ranges[start:end + 1]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        return min(valid) if valid else float('inf')


def main():
    rclpy.init()
    node = LidarLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()