"""
保存地图 Launch 文件

使用方法:
ros2 launch robot_model_pkg save_map.launch.py map_name:=my_map

地图将保存到 ~/maps/ 目录下，包括:
- my_map.yaml (地图元数据)
- my_map.pgm (地图图像)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # ============================================================================
    #                            Launch 参数
    # ============================================================================
    map_name = LaunchConfiguration('map_name', default='slam_map')
    map_dir = LaunchConfiguration('map_dir', default=os.path.expanduser('~/maps'))
    
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='slam_map',
        description='Name of the map to save (without extension)'
    )
    
    declare_map_dir = DeclareLaunchArgument(
        'map_dir',
        default_value=os.path.expanduser('~/maps'),
        description='Directory to save the map'
    )

    # ============================================================================
    #                            创建地图目录
    # ============================================================================
    create_map_dir = ExecuteProcess(
        cmd=['mkdir', '-p', map_dir],
        output='screen'
    )

    # ============================================================================
    #                            保存地图
    # ============================================================================
    save_map = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', PathJoinSubstitution([map_dir, map_name]),
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_map_name,
        declare_map_dir,
        LogInfo(msg=['Saving map to: ', map_dir, '/', map_name]),
        create_map_dir,
        save_map,
    ])
