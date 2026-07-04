#!/usr/bin/env python3
"""建图启动: slam_toolbox online async"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'solver_plugin': 'solver_plugins::CeresSolver',
            'max_laser_range': 12.0,
            'map_update_interval': 3.0,
            'resolution': 0.05,
            'mode': 'mapping',
        }],
        remappings=[('/scan', '/scan')],
    )

    return LaunchDescription([slam])
