#!/usr/bin/env python3
"""启动哨兵比赛行为决策节点"""
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    sentry_behavior = Node(
        package='sentry_behavior',
        executable='sentry_behavior.py',
        name='sentry_behavior',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        LogInfo(msg='Starting sentry competition behavior...'),
        LogInfo(msg='Manual commands:'),
        LogInfo(msg='  ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: start}"'),
        LogInfo(msg='  ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: retreat}"'),
        LogInfo(msg='  ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: chase}"'),
        LogInfo(msg='  ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: hp_low}"'),
        sentry_behavior,
    ])
