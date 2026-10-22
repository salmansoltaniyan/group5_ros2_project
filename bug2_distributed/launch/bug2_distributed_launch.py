#!/usr/bin/env python3
"""
================================================================================
Launch File for Bug2 Distributed Architecture (3 Nodes)
================================================================================
Launches:
  1. go_to_goal_server   (Action Server)
  2. wall_follow_server  (Action Server)
  3. bug2_coordinator    (Action Client / State Machine)

Usage:
  ros2 launch bug2_distributed bug2_distributed_launch.py goal_x:=2.5 goal_y:=0.0
================================================================================
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace',        default_value='robot_9',
                              description='Robot namespace'),
        DeclareLaunchArgument('wall_follow_type', default_value='lidar',
                              choices=['lidar', 'ir'],
                              description='Wall follow backend: "lidar" (2D LiDAR PD) or "ir" (Create 3 built-in IR)'),
        DeclareLaunchArgument('goal_x',           default_value='2.5',
                              description='Goal X offset from robot initial pose (meters)'),
        DeclareLaunchArgument('goal_y',           default_value='0.0',
                              description='Goal Y offset from robot initial pose (meters)'),
        DeclareLaunchArgument('linear_speed',     default_value='0.15',
                              description='Forward speed (m/s)'),
        DeclareLaunchArgument('wall_speed',       default_value='0.12',
                              description='Wall-following speed for LiDAR (m/s)'),
        DeclareLaunchArgument('angular_speed',    default_value='0.50',
                              description='Turn speed (rad/s)'),
    ]

    # Node 1: GoToGoal Action Server
    goto_server_node = Node(
        package='bug2_distributed',
        executable='go_to_goal_server',
        name='go_to_goal_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'namespace':     LaunchConfiguration('namespace'),
            'linear_speed':  LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'goal_tolerance': 0.15,
            'front_dist_threshold': 0.26,
            'corner_dist_threshold': 0.26,
        }]
    )

    # Node 2A: WallFollow Action Server (LiDAR - 4-state controller)
    wall_lidar_server_node = Node(
        package='bug2_distributed',
        executable='wall_follow_server',
        name='wall_follow_server',
        output='screen',
        emulate_tty=True,
        condition=LaunchConfigurationEquals('wall_follow_type', 'lidar'),
        parameters=[{
            'namespace':             LaunchConfiguration('namespace'),
            'wall_speed':            LaunchConfiguration('wall_speed'),
            'angular_speed':         LaunchConfiguration('angular_speed'),
            'kp_wall':               1.8,
            'kd_wall':               0.6,
            'front_dist_threshold':  0.26,
            'corner_dist_threshold': 0.25,
            'side_dist_threshold':   0.32,
            'desired_wall_dist':     0.21,
            'goal_tolerance':        0.15,
            'm_line_tolerance':      0.15,
            'leave_progress_margin': 0.02,
            'min_hit_clearance':     0.30,
        }]
    )

    # Node 2B: WallFollow Action Server (Create 3 Built-in IR Wrapper)
    wall_ir_server_node = Node(
        package='bug2_distributed',
        executable='wall_follow_ir_server',
        name='wall_follow_ir_server',
        output='screen',
        emulate_tty=True,
        condition=LaunchConfigurationEquals('wall_follow_type', 'ir'),
        parameters=[{
            'namespace':             LaunchConfiguration('namespace'),
            'goal_tolerance':        0.15,
            'm_line_tolerance':      0.15,
            'leave_progress_margin': 0.02,
            'min_hit_clearance':     0.30,
            'max_runtime_sec':       300,
        }]
    )

    # Node 3: Bug2 Coordinator (Brain)
    coordinator_node = Node(
        package='bug2_distributed',
        executable='bug2_coordinator',
        name='bug2_coordinator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'namespace': LaunchConfiguration('namespace'),
            'goal_x':    LaunchConfiguration('goal_x'),
            'goal_y':    LaunchConfiguration('goal_y'),
        }]
    )

    return LaunchDescription(args + [
        goto_server_node,
        wall_lidar_server_node,
        wall_ir_server_node,
        coordinator_node
    ])

