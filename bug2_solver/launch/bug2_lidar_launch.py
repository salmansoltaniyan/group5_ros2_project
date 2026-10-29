#!/usr/bin/env python3
"""
Launch file for bug2_solver (LiDAR variant).

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Same Bug2 algorithm as the IR version, but uses 2D LiDAR (/scan)
  for obstacle detection and wall following.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  COORDINATE SYSTEM — same as bug2_launch.py:

                 +X  (Robot Front / Forward)
                  ↑
                  |
    -Y ←──────[ROBOT]──────→ +Y
  (Robot Right)   |         (Robot Left)
                  ↓
                 -X  (Robot Back / Backward)

  USAGE:
    ros2 launch bug2_solver bug2_lidar_launch.py goal_x:=2.0 goal_y:=0.0

Arguments:
    namespace              robot namespace              (default: robot_9)
    goal_x                 target X offset (m)          (default: 2.0)
    goal_y                 target Y offset (m)          (default: 0.0)
    linear_speed           forward speed (m/s)          (default: 0.15)
    wall_speed             wall-follow speed (m/s)      (default: 0.12)
    angular_speed          turn speed (rad/s)           (default: 0.50)
    front_dist_threshold   front obstacle distance (m)  (default: 0.35)
    side_dist_threshold    side wall detect dist (m)    (default: 0.50)
    desired_wall_dist      target wall distance (m)     (default: 0.25)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('namespace',             default_value='robot_9',
                              description='Robot ROS 2 namespace'),
        DeclareLaunchArgument('goal_x',                default_value='2.0',
                              description='Goal X offset from start (meters)'),
        DeclareLaunchArgument('goal_y',                default_value='0.0',
                              description='Goal Y offset from start (meters)'),
        DeclareLaunchArgument('linear_speed',          default_value='0.15',
                              description='Forward speed (m/s)'),
        DeclareLaunchArgument('wall_speed',            default_value='0.12',
                              description='Wall-following speed (m/s)'),
        DeclareLaunchArgument('angular_speed',         default_value='0.50',
                              description='Turn speed (rad/s)'),
        # ── LiDAR distance thresholds (~5cm body-to-wall clearance) ───────────
        DeclareLaunchArgument('front_dist_threshold',  default_value='0.24',
                              description='Front obstacle detection distance (m).'),
        DeclareLaunchArgument('corner_dist_threshold', default_value='0.25',
                              description='Front-right corner obstacle detection distance (m).'),
        DeclareLaunchArgument('side_dist_threshold',   default_value='0.40',
                              description='Max distance to consider "wall present on side" (m)'),
        DeclareLaunchArgument('desired_wall_dist',     default_value='0.21',
                              description='Target distance to maintain from right wall (m). 0.21m = ~5cm from body.'),
        # ── Bug2 leave-condition tuning ───────────────────────────────────────
        DeclareLaunchArgument('m_line_tolerance',      default_value='0.15',
                              description='M-line detection band (m)'),
        DeclareLaunchArgument('leave_progress_margin', default_value='0.02',
                              description='Min progress past hit-point required to leave wall (m)'),
        DeclareLaunchArgument('min_hit_clearance',     default_value='0.30',
                              description='Min distance from hit-point before leave is checked (m)'),
    ]

    # ── Single node ───────────────────────────────────────────────────────────
    bug2_lidar_node = Node(
        package='bug2_solver',
        executable='bug2_lidar_node',
        name='bug2_lidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'namespace':             LaunchConfiguration('namespace'),
            'goal_x':                LaunchConfiguration('goal_x'),
            'goal_y':                LaunchConfiguration('goal_y'),
            'linear_speed':          LaunchConfiguration('linear_speed'),
            'wall_speed':            LaunchConfiguration('wall_speed'),
            'angular_speed':         LaunchConfiguration('angular_speed'),
            'front_dist_threshold':  LaunchConfiguration('front_dist_threshold'),
            'corner_dist_threshold': LaunchConfiguration('corner_dist_threshold'),
            'side_dist_threshold':   LaunchConfiguration('side_dist_threshold'),
            'desired_wall_dist':     LaunchConfiguration('desired_wall_dist'),
            'm_line_tolerance':      LaunchConfiguration('m_line_tolerance'),
            'leave_progress_margin': LaunchConfiguration('leave_progress_margin'),
            'min_hit_clearance':     LaunchConfiguration('min_hit_clearance'),
        }],
    )

    return LaunchDescription(args + [bug2_lidar_node])

