#!/usr/bin/env python3
"""
Launch file for bug2_solver.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  COORDINATE SYSTEM — When placing the robot on the ground
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Starting Position (0, 0) = Where the robot is located when the node starts.
  Axis directions are relative to the robot's facing direction at startup:

                 +X  (Robot Front / Forward)
                  ↑
                  |
    -Y ←──────[ROBOT]──────→ +Y
  (Robot Right)   |         (Robot Left)
                  ↓
                 -X  (Robot Back / Backward)

  Examples:
    goal_x= 2.0  goal_y= 0.0  →  2 meters straight forward
    goal_x= 2.0  goal_y= 1.0  →  2 meters forward + 1 meter left
    goal_x= 2.0  goal_y=-1.0  →  2 meters forward + 1 meter right
    goal_x= 0.0  goal_y= 3.0  →  3 meters directly left
    goal_x=-1.0  goal_y= 0.0  →  1 meter directly backward (not recommended)

  ⚠️  IMPORTANT NOTES:
    - Robot odometry resets when the robot boots up / powers on (not when running this node).
    - When the node starts, it captures the current odometry position as the initial reference.
    - If the robot has drifted or was moved while powered on, make sure odometry is stable.
    - For best accuracy: Place the robot at the starting spot, turn it on, wait for odom
      to stabilize, and then launch this node.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  USAGE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    ros2 launch bug2_solver bug2_launch.py goal_x:=2.0 goal_y:=0.0

Arguments:
    namespace     robot namespace          (default: robot_9)
    goal_x        target X offset (m)      (default: 2.0)
    goal_y        target Y offset (m)      (default: 0.0)
    linear_speed  forward speed (m/s)      (default: 0.15)
    wall_speed    wall-follow speed (m/s)  (default: 0.12)
    angular_speed turn speed (rad/s)       (default: 0.50)
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
        DeclareLaunchArgument('front_ir_threshold',    default_value='450',
                              description='IR reading threshold to detect a front wall (higher = closer to wall)'),
        # ── Bug2 leave-condition tuning ───────────────────────────────────────
        # m_line_tolerance:      perpendicular band around M-line that counts as "on the line"
        #                        Wider → easier to detect M-line crossing after obstacle
        DeclareLaunchArgument('m_line_tolerance',      default_value='0.15',
                              description='M-line detection band (m). Default 0.15 m'),
        # leave_progress_margin: robot must be at least this much closer to goal than the
        #                        hit-point was, before it is allowed to leave WALL_FOLLOW.
        #                        Smaller → leaves sooner with less required progress.
        #                        0.02 m works well when the bypass obstacle is large and the
        #                        robot can only gain a few centimeters of progress per loop.
        DeclareLaunchArgument('leave_progress_margin', default_value='0.02',
                              description='Min progress past hit-point required to leave wall (m). Default 0.02 m'),
        # min_hit_clearance:     robot must travel at least this far (Euclidean) from the
        #                        hit-point before the leave condition is even checked.
        #                        Prevents false leave right at the hit-point.
        DeclareLaunchArgument('min_hit_clearance',     default_value='0.30',
                              description='Min Euclidean distance from hit-point before leave is checked (m)'),
    ]

    # ── Single node ───────────────────────────────────────────────────────────
    bug2_node = Node(
        package='bug2_solver',
        executable='bug2_node',
        name='bug2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'namespace':             LaunchConfiguration('namespace'),
            'goal_x':                LaunchConfiguration('goal_x'),
            'goal_y':                LaunchConfiguration('goal_y'),
            'linear_speed':          LaunchConfiguration('linear_speed'),
            'wall_speed':            LaunchConfiguration('wall_speed'),
            'angular_speed':         LaunchConfiguration('angular_speed'),
            'front_ir_threshold':    LaunchConfiguration('front_ir_threshold'),
            'm_line_tolerance':      LaunchConfiguration('m_line_tolerance'),
            'leave_progress_margin': LaunchConfiguration('leave_progress_margin'),
            'min_hit_clearance':     LaunchConfiguration('min_hit_clearance'),
        }],
    )

    return LaunchDescription(args + [bug2_node])

