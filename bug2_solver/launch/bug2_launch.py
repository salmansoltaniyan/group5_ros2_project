#!/usr/bin/env python3
"""
Launch file for bug2_solver.

Usage (simplest):
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
        DeclareLaunchArgument('namespace',     default_value='robot_9',
                              description='Robot ROS 2 namespace'),
        DeclareLaunchArgument('goal_x',        default_value='2.0',
                              description='Goal X offset from start (meters)'),
        DeclareLaunchArgument('goal_y',        default_value='0.0',
                              description='Goal Y offset from start (meters)'),
        DeclareLaunchArgument('linear_speed',  default_value='0.15',
                              description='Forward speed (m/s)'),
        DeclareLaunchArgument('wall_speed',    default_value='0.12',
                              description='Wall-following speed (m/s)'),
        DeclareLaunchArgument('angular_speed', default_value='0.50',
                              description='Turn speed (rad/s)'),
    ]

    # ── Single node ───────────────────────────────────────────────────────────
    bug2_node = Node(
        package='bug2_solver',
        executable='bug2_node',
        name='bug2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'namespace':     LaunchConfiguration('namespace'),
            'goal_x':        LaunchConfiguration('goal_x'),
            'goal_y':        LaunchConfiguration('goal_y'),
            'linear_speed':  LaunchConfiguration('linear_speed'),
            'wall_speed':    LaunchConfiguration('wall_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
        }],
    )

    return LaunchDescription(args + [bug2_node])

