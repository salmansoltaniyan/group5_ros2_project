import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.create_subscription(LaserScan, '/robot_9/scan', self.scan_callback,
                                 qos_profile_sensor_data)
        self.scan = None
        self.create_timer(1.0, self.print_status)

    def scan_callback(self, msg):
        self.scan = msg

    def sector_min(self, start_deg, end_deg):
        msg = self.scan
        best = float('inf')
        for deg in range(start_deg, end_deg + 1):
            angle = math.atan2(math.sin(math.radians(deg)), math.cos(math.radians(deg)))
            i = int((angle - msg.angle_min) / msg.angle_increment)
            if 0 <= i < len(msg.ranges):
                r = msg.ranges[i]
                if math.isfinite(r) and msg.range_min < r < msg.range_max:
                    best = min(best, r)
        return best

    def print_status(self):
        front = self.sector_min(-15, 15)
        left = self.sector_min(75, 105)
        right = self.sector_min(-105, -75)
        back = self.sector_min(165, 195)
        self.get_logger().info(
            f'front {front:.2f} m | left {left:.2f} m | right {right:.2f} m | back {back:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()