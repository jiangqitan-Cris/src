#!/usr/bin/env python3
"""启动轨迹跟踪控制器节点（可与 navigation_demo 一起用）"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('trajectory_tracker')
    config = os.path.join(pkg_share, 'params', 'tracker_config.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    controller_type = LaunchConfiguration('controller_type', default='lqr')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('controller_type', default_value='lqr',
                              description='lqr, pure_pursuit, or mpc'),
        Node(
            package='trajectory_tracker',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time, 'controller_type': controller_type}],
            remappings=[
                ('/local_path', '/local_path'),
                ('/odom', '/odom'),
                ('/cmd_vel', '/cmd_vel'),
                ('/steering_angle', '/steering_angle'),
            ]
        ),
    ])
