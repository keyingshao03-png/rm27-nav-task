#!/usr/bin/env python3
"""
启动 Nav2 导航栈: planner + controller + behavior + bt_navigator

注意:
planner_server 和 controller_server 会分别创建自己的 global_costmap /
local_costmap，不需要额外启动独立的 nav2_costmap_2d 节点。单独启动
costmap 会让节点名和参数检查变混乱。
"""
import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    param_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # Nav2 核心节点列表
    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[param_file],
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[param_file],
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[param_file],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[param_file],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[param_file],
    )

    # lifecycle manager 自动激活所有 Nav2 节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': lifecycle_nodes,
            'bond_timeout': 5.0,
        }],
    )

    return LaunchDescription([
        LogInfo(msg='Starting Nav2 navigation stack...'),
        planner,
        controller,
        behavior,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
        LogInfo(msg='Nav2 stack started! Use RViz2 "2D Goal Pose" to send goal.'),
    ])
