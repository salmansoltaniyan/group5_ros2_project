#!/usr/bin/env python3
"""
===============================================================================
Bug 2 Navigation Launch File for iRobot Create 3 (ROS 2 Jazzy)
===============================================================================
Package: bug2_navigation_pkg
Launches:
1. bug2_wall_follow_server: Custom Action Server monitoring Bug2 leave conditions.
2. line_follow_node: Action Client requesting wall following around an obstacle.

Namespace: /robot_9
Does NOT launch Create 3 native action server (already running on robot hardware).
===============================================================================
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Launch Arguments
    # -------------------------------------------------------------------------
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot_9',
        description='Top-level namespace for the Create 3 robot'
    )

    start_x_arg = DeclareLaunchArgument('start_x', default_value='0.0', description='M-line Start X')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='0.0', description='M-line Start Y')
    target_x_arg = DeclareLaunchArgument('target_x', default_value='2.0', description='M-line Target X')
    target_y_arg = DeclareLaunchArgument('target_y', default_value='1.0', description='M-line Target Y')
    hit_x_arg = DeclareLaunchArgument('hit_x', default_value='0.8', description='Hit point X')
    hit_y_arg = DeclareLaunchArgument('hit_y', default_value='0.4', description='Hit point Y')
    follow_side_arg = DeclareLaunchArgument('follow_side', default_value='-1', description='Follow side: -1 Right, +1 Left')

    line_tol_arg = DeclareLaunchArgument('line_tolerance', default_value='0.08', description='M-line tolerance (m)')
    dep_thresh_arg = DeclareLaunchArgument('hit_departure_threshold', default_value='0.25', description='Hit departure (m)')
    prog_margin_arg = DeclareLaunchArgument('progress_margin', default_value='0.05', description='Progress margin (m)')

    # -------------------------------------------------------------------------
    # Node 1: Bug2 Wall Follow Action Server
    # -------------------------------------------------------------------------
    server_node = Node(
        package='bug2_navigation_pkg',
        executable='bug2_wall_follow_server',
        name='bug2_wall_follow_server',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'odom_topic': '/robot_9/odom',
            'cmd_vel_topic': '/robot_9/cmd_vel',
            'wall_follow_action': '/robot_9/wall_follow',
            'bug2_action_name': 'bug2_wall_follow', # Expands to /<namespace>/bug2_wall_follow
            'line_tolerance': LaunchConfiguration('line_tolerance'),
            'hit_departure_threshold': LaunchConfiguration('hit_departure_threshold'),
            'progress_margin': LaunchConfiguration('progress_margin'),
            'required_consecutive_samples': 3,
            'odom_timeout_sec': 1.0,
            'max_wall_follow_runtime': 600
        }]
    )

    # -------------------------------------------------------------------------
    # Node 2: Line Follow Action Client (Demonstration)
    # -------------------------------------------------------------------------
    client_node = Node(
        package='bug2_navigation_pkg',
        executable='line_follow_node',
        name='line_follow_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'action_name': 'bug2_wall_follow', # Expands to /<namespace>/bug2_wall_follow
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'hit_x': LaunchConfiguration('hit_x'),
            'hit_y': LaunchConfiguration('hit_y'),
            'follow_side': LaunchConfiguration('follow_side'),
        }]
    )

    return LaunchDescription([
        namespace_arg,
        start_x_arg,
        start_y_arg,
        target_x_arg,
        target_y_arg,
        hit_x_arg,
        hit_y_arg,
        follow_side_arg,
        line_tol_arg,
        dep_thresh_arg,
        prog_margin_arg,
        server_node,
        client_node,
    ])

