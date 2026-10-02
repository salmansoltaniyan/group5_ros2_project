#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection, IrIntensityVector

NS = "/robot_9"

LEFT_SIDE = (85, 95)
LEFT_WALL = (30, 150)
RIGHT_WALL = (-150, -30)

TARGET_DIST = 0.30
LOST_DIST = 0.60
FRONT_BLOCKED = 0.40
FRONT_EMERGENCY = 0.25
LANE_HALF_WIDTH = 0.19

SPEED = 0.12
TURN_SPEED = 0.8
KP = 3.0
KA = 1.5
MAX_TURN = 1.0
OPENING_CONFIRM = 2

CORNER_ANGLE = 85

IR_TOUCH = 800
BACKUP_SPEED = 0.08
BACKUP_DIST = 0.05
BACKUP_MAX_TIME = 1.5
HEADING_STEP = 10
AVOID_ANGLE = 45
DRIVE_OUT_DIST = 0.30
STUCK_TIME = 4.0
STUCK_DIST = 0.03
STUCK_ANGLE = 10
SCAN_TIMEOUT = 0.5


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class MazeSolver(Node):

    def __init__(self):
        super().__init__("maze_solver")
        self.cmd_pub = self.create_publisher(Twist, NS + "/cmd_vel", 10)

        qos = qos_profile_sensor_data
        self.create_subscription(LaserScan, NS + "/scan", self.scan_cb, qos)
        self.create_subscription(Odometry, NS + "/odom", self.odom_cb, qos)
        self.create_subscription(HazardDetectionVector, NS + "/hazard_detection", self.hazard_cb, qos)
        self.create_subscription(IrIntensityVector, NS + "/ir_intensity", self.ir_cb, qos)

        self.lidar = None
        self.points = []
        self.scan_time = 0.0
        self.opening_count = 0
        self.turn_target = 0
        self.opening_forward = TARGET_DIST

        self.last_x = self.last_y = self.last_yaw = None
        self.odometer = 0.0
        self.rotation = 0.0

        self.bumped = False
        self.ir_front = 0

        self.state = None
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.5, self.print_status)

    @staticmethod
    def closest(points, sector):
        best, best_deg = 12.0, sum(sector) / 2
        for deg, r, _, _ in points:
            if sector[0] <= deg <= sector[1] and r < best:
                best, best_deg = r, deg
        return best, best_deg

    def scan_cb(self, msg):
        points = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                a = msg.angle_min + i * msg.angle_increment
                points.append((math.degrees(a), r, r * math.cos(a), r * math.sin(a)))

        front = min([x for _, _, x, y in points if x > 0 and abs(y) < LANE_HALF_WIDTH],
                    default=12.0)
        gap = min([x for _, _, x, y in points if x > 0.05 and LANE_HALF_WIDTH < y < LOST_DIST],
                  default=12.0)
        side, _ = self.closest(points, LEFT_SIDE)
        wall, wall_angle = self.closest(points, LEFT_WALL)
        right, _ = self.closest(points, RIGHT_WALL)

        target = TARGET_DIST
        if right < LOST_DIST:
            target = min(TARGET_DIST, (wall + right) / 2)

        self.opening_count = self.opening_count + 1 if side > LOST_DIST else 0

        self.points = points
        self.lidar = {"front": front, "side": side, "wall": wall,
                      "wall_angle": wall_angle, "right": right, "target": target, "gap": gap}
        self.scan_time = self.now()

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.last_yaw is not None:
            self.odometer += math.hypot(p.x - self.last_x, p.y - self.last_y)
            self.rotation += wrap(yaw - self.last_yaw)
        self.last_x, self.last_y, self.last_yaw = p.x, p.y, yaw

    def hazard_cb(self, msg):
        for d in msg.detections:
            if d.type == HazardDetection.BUMP:
                self.bumped = True

    def ir_cb(self, msg):
        values = {r.header.frame_id: r.value for r in msg.readings}
        self.ir_front = max(values.get("ir_intensity_front_center_left", 0),
                            values.get("ir_intensity_front_center_right", 0))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_state(self, new_state, reason=""):
        print(f"--> {new_state}  {reason}")
        self.state = new_state
        self.state_time = self.now()
        self.state_odo = self.odometer
        self.state_rot = self.rotation
        self.reset_stuck_timer()

    def turned(self):
        return abs(math.degrees(self.rotation - self.state_rot))

    def driven(self):
        return self.odometer - self.state_odo

    def reset_stuck_timer(self):
        self.stuck_time = self.now()
        self.stuck_odo = self.odometer
        self.stuck_rot = self.rotation

    def is_stuck(self):
        if self.now() - self.stuck_time < STUCK_TIME:
            return False
        moved = self.odometer - self.stuck_odo
        rotated = abs(math.degrees(self.rotation - self.stuck_rot))
        self.reset_stuck_timer()
        return moved < STUCK_DIST and rotated < STUCK_ANGLE

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if self.lidar is None or self.last_yaw is None:
            return
        if self.state is None:
            self.turn_to_open("(start)")

        if self.now() - self.scan_time > SCAN_TIMEOUT:
            self.publish(0.0, 0.0)
            return

        if self.state in ("FOLLOW", "OPENING", "DRIVE_OUT"):
            if self.bumped:
                self.set_state("BACKUP", "(bumper)")
            elif self.ir_front > IR_TOUCH:
                self.set_state("BACKUP", f"(IR {self.ir_front})")
        if self.state != "BACKUP" and self.is_stuck():
            self.set_state("BACKUP", "(stuck)")
        self.bumped = False

        L = self.lidar
        if self.state == "FOLLOW":
            v, w = self.follow(L)
        elif self.state == "TURN_RIGHT":
            v, w = self.turn_right()
        elif self.state == "OPENING":
            v, w = self.opening(L)
        elif self.state == "TURN_LEFT":
            v, w = self.turn_left()
        elif self.state == "BACKUP":
            v, w = self.backup()
        elif self.state == "DRIVE_OUT":
            v, w = self.drive_out(L)
        else:
            v, w = self.turn_to()
        self.publish(v, w)

    def follow(self, L):
        if L["front"] < FRONT_EMERGENCY:
            self.set_state("TURN_RIGHT", "(too close in front)")
            return 0.0, 0.0
        if self.opening_count >= OPENING_CONFIRM:
            self.opening_forward = min(TARGET_DIST, L["gap"] / 2)
            self.set_state("OPENING", f"(drive {self.opening_forward:.2f} m, then turn left)")
            return 0.0, 0.0
        if L["front"] < FRONT_BLOCKED:
            self.set_state("TURN_RIGHT", "(wall in front)")
            return 0.0, 0.0

        dist_err = L["wall"] - L["target"]
        ang_err = math.radians(L["wall_angle"] - 90)
        w = KP * dist_err + KA * ang_err
        return SPEED, max(-MAX_TURN, min(MAX_TURN, w))

    def turn_right(self):
        if self.turned() >= CORNER_ANGLE:
            self.set_state("FOLLOW")
            return 0.0, 0.0
        return 0.0, -TURN_SPEED

    def opening(self, L):
        if self.driven() >= self.opening_forward or L["front"] < FRONT_BLOCKED:
            self.set_state("TURN_LEFT")
            return 0.0, 0.0
        return SPEED, 0.0

    def turn_left(self):
        if self.turned() >= CORNER_ANGLE:
            self.set_state("FOLLOW")
            return 0.0, 0.0
        return 0.0, TURN_SPEED

    def backup(self):
        if self.driven() >= BACKUP_DIST or self.now() - self.state_time > BACKUP_MAX_TIME:
            self.turn_to_open("(after backup)", avoid_front=True)
            return 0.0, 0.0
        return -BACKUP_SPEED, 0.0

    def free_distance(self, heading_deg):
        h = math.radians(heading_deg)
        best = 12.0
        for _, _, x, y in self.points:
            xr = x * math.cos(h) + y * math.sin(h)
            yr = -x * math.sin(h) + y * math.cos(h)
            if xr > 0 and abs(yr) < LANE_HALF_WIDTH:
                best = min(best, xr)
        return best

    def turn_to_open(self, reason, avoid_front=False):
        headings = [h for h in range(-180, 180, HEADING_STEP)
                    if not (avoid_front and abs(h) <= AVOID_ANGLE)]
        self.turn_target = max(headings, key=self.free_distance)
        self.set_state("TURN_TO", f"{reason} most open: {self.turn_target} deg")

    def turn_to(self):
        if self.turned() >= abs(self.turn_target) - HEADING_STEP / 2:
            self.set_state("DRIVE_OUT")
            return 0.0, 0.0
        return 0.0, math.copysign(TURN_SPEED, self.turn_target)

    def drive_out(self, L):
        if self.driven() >= DRIVE_OUT_DIST:
            self.set_state("FOLLOW", "(out)")
            return 0.0, 0.0
        if L["front"] < FRONT_BLOCKED:
            self.set_state("FOLLOW", "(blocked)")
            return 0.0, 0.0
        return SPEED, 0.0

    def print_status(self):
        if self.lidar is None or self.state is None:
            return
        L = self.lidar
        print(f"{self.state:10} front {L['front']:.2f}  side {L['side']:.2f}  "
              f"left {L['wall']:.2f}@{L['wall_angle']:.0f}  right {L['right']:.2f}  "
              f"target {L['target']:.2f}  turned {self.turned():.0f}  ir {self.ir_front}  "
              f"odo {self.odometer:.2f} m")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()