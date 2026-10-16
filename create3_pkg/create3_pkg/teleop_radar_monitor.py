#!/usr/bin/env python3
"""
===============================================================================
Keyboard Teleop & 8-Sector LiDAR / Odometry Monitor
===============================================================================
Features:
1. Keyboard Teleoperation:
   - [W] Forward      [S] Backward     [A] Turn Left     [D] Turn Right
   - [Q] Curve Left   [E] Curve Right  [Space / X] STOP immediately
   - [+] Speed Up     [-] Slow Down
2. Clean, Non-Flooding Dashboard (Updated at 2 Hz):
   - Odometry: Position (X, Y in meters), Heading (Yaw in degrees), Speed
   - LiDAR 8-Sector Surround Radar: Min distance for Front, FR, Right, RR,
     Back, RL, Left, FL with colored safety indicators.
   - Highlights the closest obstacle distance & direction.
===============================================================================
"""

import math
import sys
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Cross-platform keyboard input setup
IS_WINDOWS = sys.platform == 'win32'
if IS_WINDOWS:
    import msvcrt
else:
    import select
    import termios
    import tty


class TeleopRadarMonitor(Node):
    def __init__(self):
        super().__init__('teleop_radar_monitor')

        # ---------------------------------------------------------------------
        # SPEED SETTINGS
        # ---------------------------------------------------------------------
        self.speed_linear = 0.15      # m/s
        self.speed_angular = 0.50     # rad/s
        self.target_linear = 0.0
        self.target_angular = 0.0

        # ---------------------------------------------------------------------
        # ODOMETRY DATA
        # ---------------------------------------------------------------------
        self.odom_received = False
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw_deg = 0.0
        self.current_vel_x = 0.0
        self.current_vel_yaw = 0.0

        # ---------------------------------------------------------------------
        # 8-SECTOR LIDAR DATA
        # ---------------------------------------------------------------------
        self.lidar_received = False
        self.sectors = {
            'FRONT': float('inf'),
            'FR': float('inf'),
            'RIGHT': float('inf'),
            'RR': float('inf'),
            'BACK': float('inf'),
            'RL': float('inf'),
            'LEFT': float('inf'),
            'FL': float('inf'),
        }
        self.closest_dist = float('inf')
        self.closest_sector = "None"

        # ---------------------------------------------------------------------
        # ROS 2 COMMUNICATION
        # ---------------------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)
        self.cmd_pub_backup = self.create_publisher(Twist, '/cmd_vel', 10)

        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, best_effort_qos)
        self.scan_sub_backup = self.create_subscription(LaserScan, '/robot_9/scan', self.scan_callback, best_effort_qos)

        self.odom_sub = self.create_subscription(Odometry, '/robot_9/odom', self.odom_callback, best_effort_qos)
        self.odom_sub_backup = self.create_subscription(Odometry, '/odom', self.odom_callback, best_effort_qos)

        # 10 Hz Timer to continuously publish Twist command
        self.cmd_timer = self.create_timer(0.1, self.publish_velocity)

        # 2 Hz Timer (0.5s) to print the clean dashboard without flooding
        self.display_timer = self.create_timer(0.5, self.display_dashboard)

        # Running flag for keyboard listener thread
        self.running = True

        self.get_logger().info('>>> Teleop Radar Monitor initialized! Starting dashboard...')

    def normalize_angle(self, angle: float) -> float:
        """Wraps angle between [-pi, +pi] radians."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odom_callback(self, msg: Odometry):
        """Extracts X, Y, Yaw (in degrees), and current speeds from Odometry."""
        self.odom_received = True
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

        # Quaternion to Euler Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.yaw_deg = math.degrees(yaw_rad)

        self.current_vel_x = msg.twist.twist.linear.x
        self.current_vel_yaw = msg.twist.twist.angular.z

    def scan_callback(self, msg: LaserScan):
        """Partitions 360° LiDAR scan into 8 distinct 45-degree sectors."""
        self.lidar_received = True
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return

        buckets = {
            'FRONT': [],
            'FR': [],
            'RIGHT': [],
            'RR': [],
            'BACK': [],
            'RL': [],
            'LEFT': [],
            'FL': [],
        }

        for i, dist in enumerate(msg.ranges):
            # Ignore invalid readings, zeros, or values below sensor threshold
            if math.isnan(dist) or math.isinf(dist) or dist <= msg.range_min or dist <= 0.05:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)
            angle = self.normalize_angle(angle)

            # Classify into 8 sectors (45° = ~0.785 rad each)
            # 0 rad = Front, +pi/2 = Left, -pi/2 = Right, +/-pi = Back
            if -0.3927 <= angle <= 0.3927:
                buckets['FRONT'].append(dist)
            elif 0.3927 < angle <= 1.1781:
                buckets['FL'].append(dist)
            elif 1.1781 < angle <= 1.9635:
                buckets['LEFT'].append(dist)
            elif 1.9635 < angle <= 2.7489:
                buckets['RL'].append(dist)
            elif angle > 2.7489 or angle < -2.7489:
                buckets['BACK'].append(dist)
            elif -2.7489 <= angle < -1.9635:
                buckets['RR'].append(dist)
            elif -1.9635 <= angle < -1.1781:
                buckets['RIGHT'].append(dist)
            elif -1.1781 <= angle < -0.3927:
                buckets['FR'].append(dist)

        # Calculate minimum distance per sector
        closest_d = float('inf')
        closest_sec = "None"
        for sec in self.sectors.keys():
            m = min(buckets[sec]) if buckets[sec] else float('inf')
            self.sectors[sec] = m
            if m < closest_d:
                closest_d = m
                closest_sec = sec

        self.closest_dist = closest_d
        self.closest_sector = closest_sec

    def publish_velocity(self):
        """Continuously publishes current movement command at 10 Hz."""
        cmd = Twist()
        cmd.linear.x = self.target_linear
        cmd.angular.z = self.target_angular
        self.cmd_pub.publish(cmd)
        self.cmd_pub_backup.publish(cmd)

    def format_dist(self, d: float) -> str:
        """Formats distance with intuitive colored text indicators."""
        if math.isinf(d):
            return " Clear "
        # Color coding: Red if < 20cm, Yellow if < 35cm, Green otherwise
        if d < 0.20:
            return f"\033[91m{d:.2f}m!\033[0m"
        elif d < 0.35:
            return f"\033[93m{d:.2f}m \033[0m"
        else:
            return f"\033[92m{d:.2f}m \033[0m"

    def display_dashboard(self):
        """Renders an intuitive, clean telemetry dashboard every 0.5s."""
        # Clear screen ANSI escape code: \033[2J\033[H (home cursor)
        # Using clear screen ensures the display updates in-place without scrolling spam
        fl_str = self.format_dist(self.sectors['FL'])
        f_str  = self.format_dist(self.sectors['FRONT'])
        fr_str = self.format_dist(self.sectors['FR'])
        l_str  = self.format_dist(self.sectors['LEFT'])
        r_str  = self.format_dist(self.sectors['RIGHT'])
        rl_str = self.format_dist(self.sectors['RL'])
        b_str  = self.format_dist(self.sectors['BACK'])
        rr_str = self.format_dist(self.sectors['RR'])

        closest_txt = (
            f"\033[91m{self.closest_sector} ({self.closest_dist*100:.0f} cm)\033[0m"
            if self.closest_dist < 0.35
            else f"\033[92m{self.closest_sector} ({self.closest_dist:.2f} m)\033[0m"
        )

        odom_status = "READY" if self.odom_received else "Waiting (/robot_9/odom)..."
        lidar_status = "READY" if self.lidar_received else "Waiting (/scan)..."

        dashboard = (
            "\033[H\033[2J"  # Clean screen & home cursor
            "===============================================================================\n"
            "🎮 ROBOT TELEOP & 8-SECTOR RADAR MONITOR (Group 5 - Robot 9)\n"
            "===============================================================================\n"
            f"📡 Sensors: Odom [{odom_status}] | LiDAR [{lidar_status}]\n"
            "-------------------------------------------------------------------------------\n"
            "📍 ODOMETRY (Position & Orientation):\n"
            f"   Position   : X = {self.pos_x:+6.2f} m   |  Y = {self.pos_y:+6.2f} m\n"
            f"   Heading    : Yaw = {self.yaw_deg:+6.1f}°  ({self.get_heading_label(self.yaw_deg)})\n"
            f"   Live Speed : Vx = {self.current_vel_x:+5.2f} m/s | Wz = {self.current_vel_yaw:+5.2f} rad/s\n"
            "-------------------------------------------------------------------------------\n"
            "📡 8-SECTOR SURROUND RADAR (Min Distance per Sector):\n"
            f"         [FL: {fl_str}]      [FRONT: {f_str}]      [FR: {fr_str}]\n"
            f"         [ L: {l_str}]         (🤖 ROBOT)         [ R: {r_str}]\n"
            f"         [RL: {rl_str}]      [ BACK: {b_str}]      [RR: {rr_str}]\n"
            f"\n   ⚠️  Closest Obstacle: {closest_txt}\n"
            "-------------------------------------------------------------------------------\n"
            "🕹️  KEYBOARD CONTROLS:\n"
            f"   Active Command : Vx = {self.target_linear:+5.2f} m/s  |  Wz = {self.target_angular:+5.2f} rad/s\n"
            f"   Set Speeds     : Linear = {self.speed_linear:.2f} m/s | Angular = {self.speed_angular:.2f} rad/s\n"
            "   [W] Forward     [S] Backward    [A] Pivot Left    [D] Pivot Right\n"
            "   [Q] Curve Left  [E] Curve Right [Space / X] STOP  [+/-] Speed Up/Down\n"
            "   Press [Ctrl+C] to exit safely.\n"
            "==============================================================================="
        )
        sys.stdout.write(dashboard + "\n")
        sys.stdout.flush()

    def get_heading_label(self, deg: float) -> str:
        """Returns human-readable compass/relative direction."""
        if -22.5 <= deg <= 22.5:
            return "North / Forward"
        elif 22.5 < deg <= 67.5:
            return "North-West"
        elif 67.5 < deg <= 112.5:
            return "West / Left"
        elif 112.5 < deg <= 157.5:
            return "South-West"
        elif deg > 157.5 or deg < -157.5:
            return "South / Backward"
        elif -157.5 <= deg < -112.5:
            return "South-East"
        elif -112.5 <= deg < -67.5:
            return "East / Right"
        elif -67.5 <= deg < -22.5:
            return "North-East"
        return "Unknown"

    def handle_key(self, key: str):
        """Processes key presses to update target motion commands."""
        key = key.lower()
        if key == 'w':
            self.target_linear = self.speed_linear
            self.target_angular = 0.0
        elif key == 's':
            self.target_linear = -self.speed_linear
            self.target_angular = 0.0
        elif key == 'a':
            self.target_linear = 0.0
            self.target_angular = self.speed_angular
        elif key == 'd':
            self.target_linear = 0.0
            self.target_angular = -self.speed_angular
        elif key == 'q':
            self.target_linear = self.speed_linear
            self.target_angular = self.speed_angular * 0.7
        elif key == 'e':
            self.target_linear = self.speed_linear
            self.target_angular = -self.speed_angular * 0.7
        elif key == ' ' or key == 'x':
            self.target_linear = 0.0
            self.target_angular = 0.0
        elif key in ['+', '=']:
            self.speed_linear = min(0.35, self.speed_linear + 0.03)
            self.speed_angular = min(1.0, self.speed_angular + 0.10)
        elif key in ['-', '_']:
            self.speed_linear = max(0.05, self.speed_linear - 0.03)
            self.speed_angular = max(0.20, self.speed_angular - 0.10)


def keyboard_thread_fn(node: TeleopRadarMonitor):
    """Background thread to capture non-blocking keystrokes."""
    if IS_WINDOWS:
        while node.running and rclpy.ok():
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                try:
                    key = ch.decode('utf-8', errors='ignore')
                    node.handle_key(key)
                except Exception:
                    pass
            time.sleep(0.03)
    else:
        # Linux / Unix non-blocking terminal read
        orig_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while node.running and rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    key = sys.stdin.read(1)
                    if key:
                        node.handle_key(key)
                time.sleep(0.02)
        except Exception:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopRadarMonitor()

    # Start keyboard listener in background daemon thread
    kb_thread = threading.Thread(target=keyboard_thread_fn, args=(node,), daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.target_linear = 0.0
        node.target_angular = 0.0
        node.publish_velocity()
        node.destroy_node()
        rclpy.shutdown()
        # Clean terminal exit
        sys.stdout.write("\n\033[0m>>> Robot stopped. Teleop monitor exited safely.\n")
        sys.stdout.flush()


if __name__ == '__main__':
    main()
