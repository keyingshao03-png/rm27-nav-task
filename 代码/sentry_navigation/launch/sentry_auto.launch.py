#!/usr/bin/env python3
"""
一键启动完整哨兵导航系统
= 仿真检查 + 定位 + 导航 + 决策
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(__file__))

    # ---- 定位: map_server + AMCL ----
    loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'map_file': os.path.expanduser('~/map.yaml')}.items(),
    )

    # ---- Nav2 导航栈 ----
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nav_bringup.launch.py')
        ),
    )

    # ---- 哨兵行为决策 ----
    sentry_behavior = Node(
        package='sentry_behavior',
        executable='sentry_behavior.py',
        name='sentry_behavior',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ---- 动态障碍物 ----
    moving_obstacle = Node(
        package='sentry_navigation',
        executable='moving_obstacle.py',
        name='moving_obstacle',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg='='*50),
        LogInfo(msg='  哨兵导航系统 — 一键启动'),
        LogInfo(msg='  定位 → 规划 → 决策 → 避障'),
        LogInfo(msg='='*50),

        # 先启动定位
        loc_launch,

        # 等 3 秒 AMCL 收敛后再启动导航
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='AMCL initialized, starting Nav2...'),
                nav_launch,
            ],
        ),

        # 等 5 秒再启动决策模块
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='Nav2 ready, starting sentry behavior...'),
                sentry_behavior,
            ],
        ),

        # 障碍物延迟
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg='Spawning moving obstacle...'),
                moving_obstacle,
            ],
        ),

        LogInfo(msg='System ready! States: PATROL → AVOID → RETREAT'),
    ])
