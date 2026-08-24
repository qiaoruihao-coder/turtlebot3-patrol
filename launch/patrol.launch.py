#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tb3_patrol 一键启动文件（加分项：少量命令启动）

用法:
    ros2 launch tb3_patrol patrol.launch.py

一次性启动:
    1. Gazebo 仿真环境（turtlebot3_world）
    2. Nav2 导航栈（加载地图 + 自定义参数）
    3. 巡检节点（代码自动设初始位姿 + 顺序访问目标点，全程无需 RViz 手动操作）
"""
import os

# 确保在解析被 include 的 turtlebot3 launch 文件前设置好环境变量
os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 各包 share 目录 ----
    tb3_patrol_share = get_package_share_directory('tb3_patrol')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav2_share = get_package_share_directory('turtlebot3_navigation2')

    # ---- 可配置参数 ----
    map_file = LaunchConfiguration(
        'map',
        default=os.path.expanduser('~/map.yaml'),
    )
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(tb3_patrol_share, 'config', 'nav2_params.yaml'),
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # ---- 1. Gazebo 世界（默认生成点 (-2,-0.5) = 地图原点附近, 地图即从该处构建）----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
        ),
    )

    # ---- 2. Nav2 导航栈 ----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_nav2_share, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ---- 3. 巡检节点（延迟启动，等 Nav2 就绪；内部还有 waitUntilNav2Active 兜底）----
    patrol = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='tb3_patrol',
                executable='patrol',
                name='tb3_patrol',
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        gazebo,
        nav2,
        patrol,
    ])
