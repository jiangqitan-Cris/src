"""
SLAM Toolbox 建图 Launch 文件

使用方法:
1. 首先启动 Gazebo 仿真:
   ros2 launch robot_model_pkg ackermann_gazebo.launch.py

2. 然后启动建图:
   ros2 launch robot_model_pkg slam_mapping.launch.py

3. 用键盘或其他方式控制机器人移动，探索环境

4. 建图完成后保存地图:
   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
   
   或者使用 SLAM Toolbox 的服务:
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/jiangqi/maps/my_map'}}"
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
    #                            RViz (可选，用于可视化建图过程)
    # ============================================================================
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'slam_mapping.rviz')
    
    # 如果没有专门的 SLAM rviz 配置，可以不启动或使用默认配置
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=None  # 始终启动
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox_launch,
        # rviz_node,  # 如果 Gazebo launch 已启动 RViz，这里可以注释掉
    ])
