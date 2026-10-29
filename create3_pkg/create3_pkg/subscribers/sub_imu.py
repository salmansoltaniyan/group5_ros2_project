#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__("imu_subscriber")
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to the IMU topic (contains Gyroscope and Accelerometer)
        self.sub = self.create_subscription(
            Imu, 
            "/robot_9/imu", 
            self.callback, 
            qos
        )
        self.get_logger().info("Subscribed to /robot_9/imu. Waiting for gyroscope data...")

    def callback(self, msg):
        # =========================================================================
        # IMPORTANT EXTRACTABLE DATA (IMU):
        # 1. GYROSCOPE (msg.angular_velocity): How fast the robot is rotating (rad/s).
        #    - angular_velocity.z is the most important for 2D robots (spinning left/right).
        # 2. ACCELEROMETER (msg.linear_acceleration): The physical push/acceleration (m/s^2).
        #    - linear_acceleration.x is forward thrust/braking.
        #    - linear_acceleration.z will usually read ~9.81 (gravity) when on a flat floor.
        # 3. ORIENTATION (msg.orientation): The estimated 3D angle of the robot (Quaternions).
        # =========================================================================
        
        # Extracting Gyroscope data (Z-axis rotation speed)
        gyro_z = msg.angular_velocity.z
        
        # Extracting Accelerometer data (X-axis forward acceleration)
        accel_x = msg.linear_acceleration.x
        
        self.get_logger().info(f"Gyro (Turn speed): Z = {gyro_z:.3f} rad/s | Accel (Forward push): X = {accel_x:.3f} m/s^2")


def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

