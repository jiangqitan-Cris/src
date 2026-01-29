import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg


def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # 配置文件路径
    config_file = os.path.join(
        get_package_share_directory('global_planner'),
        'params',
        'planner_config.yaml'
    )
    
    map_file = os.path.join(
        get_package_share_directory('global_planner'), 
        'maps', 
        'nav.yaml'
    )
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare('global_planner'),
        'rviz',
        'navigation.rviz'
    ])

    # 全局规划器节点 (Lifecycle Node)
    planner_node = LifecycleNode(
        package='global_planner',
        executable='global_planner_node',
        name='global_planner_node',
        namespace='',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz 目标点转 Action 的桥接节点
    rviz_goal_bridge_node = Node(
        package='global_planner',
        executable='rviz_goal_to_action_node',
        name='rviz_goal_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 地图服务器
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Lifecycle Manager - 管理所有需要生命周期的节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',
                'global_planner_node'
            ],
            'bond_timeout': 4.0,
        }]
    )

    # 静态 TF (map -> base_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),
        
        # 节点启动
        static_tf,
        map_server,
        planner_node,
        rviz_goal_bridge_node,
        lifecycle_manager,
        rviz2,
    ])
