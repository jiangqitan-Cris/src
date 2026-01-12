import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 规划器节点
    config_file = os.path.join(
        get_package_share_directory('global_planner'),
        'params',
        'planner_config.yaml'
    )
    planner_node = Node(
        package='global_planner',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[config_file]
    )

    # 2. 地图发布节点 (Map Server)
    # 注意：需要安装 nav2_map_server。如果没有，可以用下面这个简单的
    map_file = os.path.join(get_package_share_directory('global_planner'), 'maps', 'nav.yaml')
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_file}]
    )
    
    # 生命周期管理 (Nav2组件需要)
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_service',
        parameters=[{'node_names': ['map_server'], 'autostart': True}]
    )

    # 3. 静态坐标变换 (模拟机器人位置: map -> base_link)
    # 假设机器人就在原点
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # 4. Rviz2 可视化
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
        FindPackageShare('global_planner'),
        'rviz',
        'navigation.rviz'
    ])]
    )

    return LaunchDescription([
        planner_node,
        map_server,
        map_server_lifecycle,
        static_tf,
        rviz2
    ])