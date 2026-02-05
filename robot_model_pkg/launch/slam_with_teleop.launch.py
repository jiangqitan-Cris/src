"""
SLAM 建图 + 键盘遥控 Launch 文件

【重要】必须先启动 Gazebo，再启动本 launch，否则没有 /scan 和 TF，会出现
"frame map doesn't exist" 且无法建图。

使用方法:
1. 终端 1 - 启动 Gazebo 仿真（必须保持运行）:
   ros2 launch robot_model_pkg ackermann_gazebo.launch.py

2. 终端 2 - 启动建图与 RViz:
   ros2 launch robot_model_pkg slam_with_teleop.launch.py

3. 终端 3 - 启动键盘遥控（需在该终端按键控制）:
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_raw -p use_sim_time:=true

   按键说明: i/, 前进/后退  j/l 左转/右转  k 停止  q/z w/x e/c 调速度

4. 建图完成后保存地图:
   ros2 launch robot_model_pkg save_map.launch.py map_name:=my_gazebo_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================================
    #                            路径设置
    # ============================================================================
    pkg_path = get_package_share_directory('robot_model_pkg')
    slam_params_file = os.path.join(pkg_path, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'slam_mapping.rviz')

    # ============================================================================
    #                            Launch 参数
    # ============================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # ============================================================================
    #                            SLAM Toolbox 节点
    # ============================================================================
    # 使用 slam_toolbox 自带的 online_async_launch，自动完成 lifecycle 启动与激活
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    # ============================================================================
    #                            Twist 到 Ackermann 转换节点
    # ============================================================================
    twist_to_ackermann_node = Node(
        package='robot_model_pkg',
        executable='twist_to_ackermann_node',
        name='twist_to_ackermann',
        output='screen',
        parameters=[{
            'wheelbase': 0.32,
            'max_steering_angle': 0.5236,
            'use_sim_time': use_sim_time
        }],
    )

    # 键盘遥控请单独在终端运行: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_raw -p use_sim_time:=true

    # ============================================================================
    #                            RViz 可视化
    # ============================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox_launch,
        twist_to_ackermann_node,
        rviz_node,
    ])
