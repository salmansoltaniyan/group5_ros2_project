#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, HazardDetection

TARGET_LEFT = 400
LOST_LEFT = 50
FRONT_BLOCKED = 400
SPEED = 0.15
KP = 0.002

class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        self.pub = self.create_publisher(Twist, "/robot_9/cmd_vel", 10)
        qos = qos_profile_sensor_data
        self.create_subscription(IrIntensityVector, "/robot_9/ir_intensity", self.ir_callback, qos)
        self.create_subscription(HazardDetectionVector, "/robot_9/hazard_detection", self.bump_callback, qos)
        self.bump_time = 0.0

    def bump_callback(self, msg):
        if any(d.type == HazardDetection.BUMP for d in msg.detections):
            self.bump_time = time.time()

    def ir_callback(self, msg):
        ir = {r.header.frame_id: r.value for r in msg.readings}
        left = ir.get("ir_intensity_side_left", 0)
        front = max(ir.get("ir_intensity_front_center_left", 0),
                    ir.get("ir_intensity_front_center_right", 0),
                    ir.get("ir_intensity_front_left", 0))
        since_bump = time.time() - self.bump_time

        cmd = Twist()
        if since_bump < 0.5:
            cmd.linear.x = -0.1
        elif since_bump < 1.3:
            cmd.angular.z = -1.0
        elif front > FRONT_BLOCKED:
            cmd.angular.z = -0.8
        elif left < LOST_LEFT:
            cmd.linear.x = 0.1
            cmd.angular.z = 0.6
        else:
            cmd.linear.x = SPEED
            cmd.angular.z = max(-1.0, min(1.0, KP * (TARGET_LEFT - left)))

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()