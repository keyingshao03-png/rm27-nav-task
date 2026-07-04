#!/usr/bin/env python3
"""
一键启动哨兵导航系统（Nav2 正式链路）
定位 AMCL + Nav2 + 行为状态机 + 动态障碍物
注意：本 launch 不启动 simple_nav，避免多个节点同时发布 /cmd_vel。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    default_map = os.path.join(pkg_dir, 'maps', 'map.yaml')
    map_file = LaunchConfiguration('map_file')

    declare_map = DeclareLaunchArgument(
        'map_file',
        default_value=default_map,
        description='Full path to map.yaml'
    )

    loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'localization.launch.py')),
        launch_arguments={'map_file': map_file}.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'nav_bringup.launch.py')),
    )

    sentry_behavior_nav2 = Node(
        package='sentry_behavior',
        executable='sentry_behavior_nav2.py',
        name='sentry_behavior_nav2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    moving_obstacle = Node(
        package='sentry_navigation',
        executable='moving_obstacle.py',
        name='moving_obstacle',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_map,
        LogInfo(msg='=' * 50),
        LogInfo(msg='  Sentry Navigation: AMCL -> Nav2 -> Behavior'),
        LogInfo(msg='  simple_nav is NOT launched in this mode'),
        LogInfo(msg='=' * 50),
        loc_launch,
        TimerAction(period=3.0, actions=[
            LogInfo(msg='AMCL started, launching Nav2...'),
            nav_launch,
        ]),
        TimerAction(period=6.0, actions=[
            LogInfo(msg='Starting Nav2 behavior state machine...'),
            sentry_behavior_nav2,
        ]),
        TimerAction(period=8.0, actions=[
            LogInfo(msg='Spawning moving obstacle...'),
            moving_obstacle,
        ]),
        LogInfo(msg='Send: ros2 topic pub --once /sentry_command std_msgs/msg/String "{data: start}"'),
    ])
