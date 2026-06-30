#!/usr/bin/env python3
"""
启动定位: map_server + AMCL
替换之前简陋的 odom_tf_publisher
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 地图文件路径
    map_file = LaunchConfiguration('map_file', default=os.path.expanduser('~/map.yaml'))

    declare_map = DeclareLaunchArgument(
        'map_file', default_value=map_file,
        description='Full path to map.yaml'
    )

    # map_server: 加载地图
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_file}],
    )

    # AMCL: 蒙特卡洛定位
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(
            os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml'
        )],
        remappings=[('/scan', '/scan')],
    )

    # odom → base_footprint TF 发布器 (AMCL 需要)
    odom_tf = Node(
        package='sentry_navigation',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # lifecycle manager: 自动激活 map_server + amcl
    lifecycle_nodes = ['map_server', 'amcl']
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        declare_map,
        LogInfo(msg=['Map file: ', map_file]),
        map_server,
        amcl,
        odom_tf,
        lifecycle_manager,
    ])
