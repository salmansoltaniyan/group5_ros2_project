#!/usr/bin/env python3
"""
===============================================================================
Dual-Sided Wall-Following Radar & Keyboard Teleop Monitor
===============================================================================
Features:
1. Keyboard Teleoperation:
   - [W] Forward      [S] Backward     [A] Turn Left     [D] Turn Right
   - [Q] Curve Left   [E] Curve Right  [Space / X] STOP immediately
   - [+] Speed Up     [-] Slow Down
2. Functional Symmetric LiDAR Radar for Wall Following (Left & Right):
   - FRONT Corridor: -15° to +15° (collision buffer, never clips side walls)
   - Left Triplet  : +45° (slope), +90° (perpendicular), +135° (corner axle clear)
   - Right Triplet : -45° (slope), -90° (perpendicular), -135° (corner axle clear)
3. Live Wall Analysis & Heading Estimation:
   - Real-time Wall Distance & Tilt/Heading Angle (in degrees) for BOTH walls
   - Corridor Width & Centering Offset estimation
4. Clean, In-Place 2 Hz Dashboard (No scrolling or terminal flooding).
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
        # SYMMETRIC FUNCTIONAL WALL-FOLLOWING BEAMS
        # ---------------------------------------------------------------------
        self.lidar_received = False
        self.beams = {
            'FRONT': float('inf'),    # -15° to +15° (Direct corridor)
            'R_FRONT': float('inf'),  # -45° (Right slope / lookahead)
            'R_SIDE': float('inf'),   # -90° (Right perpendicular distance)
            'R_REAR': float('inf'),   # -135° (Right corner axle clearance)
            'L_FRONT': float('inf'),  # +45° (Left slope / lookahead)
            'L_SIDE': float('inf'),   # +90° (Left perpendicular distance)
            'L_REAR': float('inf'),   # +135° (Left corner axle clearance)
        }

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

        # 2 Hz Timer (0.5s) to print the clean dashboard
        self.display_timer = self.create_timer(0.5, self.display_dashboard)

        # Running flag for keyboard listener thread
        self.running = True

        self.get_logger().info('>>> Dual-Sided Radar Teleop initialized! Starting dashboard...')

    def normalize_angle(self, angle: float) -> float:
        """Wraps angle between [-pi, +pi] radians."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odom_callback(self, msg: Odometry):
        """Extracts X, Y, Yaw (in degrees), and velocities from Odometry."""
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
        """Extracts the 7 functional beams for symmetric wall following."""
        self.lidar_received = True
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return

        buckets = {
            'FRONT': [],
            'R_FRONT': [],
            'R_SIDE': [],
            'R_REAR': [],
            'L_FRONT': [],
            'L_SIDE': [],
            'L_REAR': [],
        }

        for i, dist in enumerate(msg.ranges):
            # Ignore invalid readings, zeros, or values below sensor threshold
            if math.isnan(dist) or math.isinf(dist) or dist <= msg.range_min or dist <= 0.05:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)
            angle = self.normalize_angle(angle)

            # 1. FRONT Corridor: -15° to +15° (-0.26 to +0.26 rad)
            if -0.26 <= angle <= 0.26:
                buckets['FRONT'].append(dist)

            # 2. RIGHT TRIPLET:
            # R_FRONT: -50° to -40° (-0.87 to -0.70 rad, centered at -45°)
            elif -0.87 <= angle <= -0.70:
                buckets['R_FRONT'].append(dist)
            # R_SIDE: -95° to -85° (-1.66 to -1.48 rad, centered at -90°)
            elif -1.66 <= angle <= -1.48:
                buckets['R_SIDE'].append(dist)
            # R_REAR: -140° to -125° (-2.44 to -2.18 rad, centered at -135°)
            elif -2.44 <= angle <= -2.18:
                buckets['R_REAR'].append(dist)

            # 3. LEFT TRIPLET:
            # L_FRONT: +40° to +50° (+0.70 to +0.87 rad, centered at +45°)
            elif 0.70 <= angle <= 0.87:
                buckets['L_FRONT'].append(dist)
            # L_SIDE: +85° to +95° (+1.48 to +1.66 rad, centered at +90°)
            elif 1.48 <= angle <= 1.66:
                buckets['L_SIDE'].append(dist)
            # L_REAR: +125° to +140° (+2.18 to +2.44 rad, centered at +135°)
            elif 2.18 <= angle <= 2.44:
                buckets['L_REAR'].append(dist)

        # Average rays in narrow cones for noise-free stability
        for key in self.beams.keys():
            if buckets[key]:
                self.beams[key] = sum(buckets[key]) / len(buckets[key])
            else:
                self.beams[key] = float('inf')

    def publish_velocity(self):
        """Continuously publishes current movement command at 10 Hz."""
        cmd = Twist()
        cmd.linear.x = self.target_linear
        cmd.angular.z = self.target_angular
        self.cmd_pub.publish(cmd)
        self.cmd_pub_backup.publish(cmd)

    def format_dist(self, d: float) -> str:
        """Formats distance with colored indicators."""
        if math.isinf(d):
            return " Clear "
        if d < 0.18:
            return f"\033[91m{d:.2f}m!\033[0m"
        elif d < 0.35:
            return f"\033[93m{d:.2f}m \033[0m"
        else:
            return f"\033[92m{d:.2f}m \033[0m"

    def analyze_wall(self, a: float, b: float, is_right: bool) -> tuple:
        """
        Calculates true perpendicular distance and heading angle (degrees) relative to wall.
        a = 45° ray, b = 90° ray
        """
        if math.isinf(a) or math.isinf(b) or b > 2.0:
            return None, None, "No Wall Detected"

        # Difference between projected 45° ray and perpendicular 90° ray
        # When parallel: a * cos(45°) == b
        if is_right:
            diff = (a * 0.7071) - b
        else:
            diff = b - (a * 0.7071)

        # Heading angle in degrees
        tilt_rad = math.atan2(diff, a * 0.7071)
        tilt_deg = math.degrees(tilt_rad)

        # True perpendicular distance: D = b * cos(tilt)
        true_dist = b * math.cos(tilt_rad)

        if abs(tilt_deg) <= 3.0:
            status = "\033[92mParallel / Aligned\033[0m"
        elif (tilt_deg < -3.0 and is_right) or (tilt_deg > 3.0 and not is_right):
            status = "\033[91mNose Tilted TOWARDS Wall\033[0m"
        else:
            status = "\033[93mDrifting AWAY from Wall\033[0m"

        return true_dist, tilt_deg, status

    def display_dashboard(self):
        """Renders an intuitive, clean telemetry dashboard every 0.5s."""
        f_str  = self.format_dist(self.beams['FRONT'])
        rf_str = self.format_dist(self.beams['R_FRONT'])
        rs_str = self.format_dist(self.beams['R_SIDE'])
        rr_str = self.format_dist(self.beams['R_REAR'])
        lf_str = self.format_dist(self.beams['L_FRONT'])
        ls_str = self.format_dist(self.beams['L_SIDE'])
        lr_str = self.format_dist(self.beams['L_REAR'])

        # Analyze Right Wall
        r_dist, r_tilt, r_status = self.analyze_wall(self.beams['R_FRONT'], self.beams['R_SIDE'], is_right=True)
        r_line = (
            f"Dist = {r_dist*100:4.1f}cm | Tilt = {r_tilt:+5.1f}° | {r_status}"
            if r_dist is not None else "No Wall in Range (> 2m)"
        )

        # Analyze Left Wall
        l_dist, l_tilt, l_status = self.analyze_wall(self.beams['L_FRONT'], self.beams['L_SIDE'], is_right=False)
        l_line = (
            f"Dist = {l_dist*100:4.1f}cm | Tilt = {l_tilt:+5.1f}° | {l_status}"
            if l_dist is not None else "No Wall in Range (> 2m)"
        )

        # Corridor Analysis
        if r_dist is not None and l_dist is not None:
            corridor_w = r_dist + l_dist
            offset = l_dist - r_dist  # > 0: closer to right wall
            side_txt = "Closer to Right" if offset > 0 else "Closer to Left"
            corridor_txt = f"Width = {corridor_w*100:.1f}cm | Offset = {abs(offset)*100:.1f}cm ({side_txt})"
        else:
            corridor_txt = "Open Space / Single Wall"

        odom_status = "READY" if self.odom_received else "Waiting (/robot_9/odom)..."
        lidar_status = "READY" if self.lidar_received else "Waiting (/scan)..."

        dashboard = (
            "\033[H\033[2J"  # Clean screen & home cursor
            "========================================================================================\n"
            "🎮 DUAL-SIDED WALL-FOLLOWING RADAR & TELEOP MONITOR (Group 5 - Robot 9)\n"
            "========================================================================================\n"
            f"📡 Sensors: Odom [{odom_status}] | LiDAR [{lidar_status}]\n"
            "----------------------------------------------------------------------------------------\n"
            "📍 ODOMETRY:\n"
            f"   Position   : X = {self.pos_x:+6.2f} m   |  Y = {self.pos_y:+6.2f} m\n"
            f"   Heading    : Yaw = {self.yaw_deg:+6.1f}°  ({self.get_heading_label(self.yaw_deg)})\n"
            f"   Live Speed : Vx = {self.current_vel_x:+5.2f} m/s | Wz = {self.current_vel_yaw:+5.2f} rad/s\n"
            "----------------------------------------------------------------------------------------\n"
            "📡 FUNCTIONAL WALL-FOLLOWING RADAR (7 Symmetric Beams):\n"
            f"                                [ FRONT: {f_str} ]\n"
            "                                (Corridor Ahead)\n"
            "         LEFT WALL (+45°, +90°, +135°)          RIGHT WALL (-45°, -90°, -135°)\n"
            f"     [ L_FRONT (+45°) : {lf_str} ]           [ R_FRONT (-45°) : {rf_str} ]\n"
            f"     [ L_SIDE  (+90°) : {ls_str} ]  (🤖 ROBOT)  [ R_SIDE  (-90°) : {rs_str} ]\n"
            f"     [ L_REAR (+135°) : {lr_str} ]           [ R_REAR (-135°) : {rr_str} ]\n"
            "----------------------------------------------------------------------------------------\n"
            "📊 LIVE WALL & CORRIDOR ANALYSIS:\n"
            f"   ▶ RIGHT WALL : {r_line}\n"
            f"   ▶ LEFT WALL  : {l_line}\n"
            f"   ▶ CORRIDOR   : {corridor_txt}\n"
            "----------------------------------------------------------------------------------------\n"
            "🕹️  KEYBOARD CONTROLS:\n"
            f"   Active Command : Vx = {self.target_linear:+5.2f} m/s  |  Wz = {self.target_angular:+5.2f} rad/s\n"
            f"   Set Speeds     : Linear = {self.speed_linear:.2f} m/s | Angular = {self.speed_angular:.2f} rad/s\n"
            "   [W] Forward     [S] Backward    [A] Pivot Left    [D] Pivot Right\n"
            "   [Q] Curve Left  [E] Curve Right [Space / X] STOP  [+/-] Speed Up/Down\n"
            "   Press [Ctrl+C] to exit safely.\n"
            "========================================================================================"
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
        sys.stdout.write("\n\033[0m>>> Robot stopped. Teleop monitor exited safely.\n")
        sys.stdout.flush()


if __name__ == '__main__':
    main()
