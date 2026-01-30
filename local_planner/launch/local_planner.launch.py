#!/usr/bin/env python3
"""
局部路径规划器 Launch 文件

启动局部规划器节点，订阅全局路径和地图，发布局部轨迹。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('local_planner')
    
    # 参数文件路径
    default_params_file = os.path.join(pkg_dir, 'params', 'local_planner_config.yaml')
    
    # 声明 launch 参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the local planner params file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # 局部规划器节点
    local_planner_node = Node(
        package='local_planner',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/map', '/map'),
            ('/global_path', '/global_path'),
            ('/odom', '/odom'),
            ('/local_path', '/local_path'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        local_planner_node,
    ])
