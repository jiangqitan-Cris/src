#!/usr/bin/env python3
"""
综合导航演示 Launch 文件

启动完整的导航系统：
- Gazebo 仿真 + 机器人模型
- 全局路径规划器
- 局部路径规划器（含 Lattice 可视化）
- RViz 可视化
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ============================================================================
    #                            包路径
    # ============================================================================
    robot_pkg = get_package_share_directory('robot_model_pkg')
    global_planner_pkg = get_package_share_directory('global_planner')
    local_planner_pkg = get_package_share_directory('local_planner')
    
    # ============================================================================
    #                            文件路径
    # ============================================================================
    # 机器人模型
    xacro_file = os.path.join(robot_pkg, 'urdf', 'ackermann_robot.urdf.xacro')
    world_file = os.path.join(robot_pkg, 'worlds', 'test_world.sdf')
    
    # 配置文件
    global_planner_config = os.path.join(global_planner_pkg, 'params', 'planner_config.yaml')
    local_planner_config = os.path.join(local_planner_pkg, 'params', 'local_planner_config.yaml')
    
    # 地图文件
    map_file = os.path.join(global_planner_pkg, 'maps', 'nav.yaml')
    
    # RViz 配置
    rviz_config = os.path.join(global_planner_pkg, 'rviz', 'navigation.rviz')
    
    # ============================================================================
    #                            Launch 参数
    # ============================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    visualize_lattice = LaunchConfiguration('visualize_lattice', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start lifecycle nodes'
    )
    
    declare_visualize_lattice = DeclareLaunchArgument(
        'visualize_lattice', default_value='true',
        description='Visualize Lattice planner candidate trajectories'
    )
    
    # ============================================================================
    #                            Robot Description
    # ============================================================================
    robot_description_config = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_config, value_type=str)
    
    # ============================================================================
    #                            机器人相关节点
    # ============================================================================
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Gazebo 仿真
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )
    
    # Spawn 机器人
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ackermann_robot',
            '-z', '0.1'
        ]
    )
    
    # ============================================================================
    #                            Gazebo Bridges
    # ============================================================================
    node_bridge_core = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_core',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/steering_angle@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )
    
    node_bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )
    
    node_bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_lidar',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )
    
    # Joint States 桥接（用于 RobotModel 显示）
    node_bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_joint_states',
        arguments=[
            '/world/test_world/model/ackermann_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/test_world/model/ackermann_robot/joint_state', '/joint_states'),
        ],
        output='screen'
    )
    
    # ============================================================================
    #                            TF
    # ============================================================================
    # map -> odom 静态变换
    node_static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ============================================================================
    #                            地图服务器
    # ============================================================================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # ============================================================================
    #                            全局规划器
    # ============================================================================
    global_planner_node = LifecycleNode(
        package='global_planner',
        executable='global_planner_node',
        name='global_planner_node',
        namespace='',
        output='screen',
        parameters=[
            global_planner_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz 目标点桥接
    rviz_goal_bridge_node = Node(
        package='global_planner',
        executable='rviz_goal_to_action_node',
        name='rviz_goal_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Lifecycle Manager
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
    
    # ============================================================================
    #                            局部规划器
    # ============================================================================
    local_planner_node = Node(
        package='local_planner',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[
            local_planner_config,
            {
                'use_sim_time': use_sim_time,
                'visualize_lattice': visualize_lattice,
            }
        ],
        remappings=[
            ('/map', '/map'),
            ('/global_path', '/global_path'),
            ('/odom', '/odom'),
        ]
    )
    
    # ============================================================================
    #                            RViz
    # ============================================================================
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ============================================================================
    #                            Launch Description
    # ============================================================================
    return LaunchDescription([
        # 参数声明
        declare_use_sim_time,
        declare_autostart,
        declare_visualize_lattice,
        
        # 机器人和 Gazebo
        node_robot_state_publisher,
        gz_sim_launch,
        node_spawn_entity,
        node_bridge_core,
        node_bridge_odom,
        node_bridge_lidar,
        node_bridge_joint_states,
        node_static_tf_map_odom,
        
        # 地图和全局规划 (延迟启动，等待 Gazebo)
        TimerAction(
            period=3.0,
            actions=[
                map_server,
                global_planner_node,
                rviz_goal_bridge_node,
                lifecycle_manager,
            ]
        ),
        
        # 局部规划 (延迟启动，等待全局规划器)
        TimerAction(
            period=5.0,
            actions=[
                local_planner_node,
            ]
        ),
        
        # RViz
        TimerAction(
            period=2.0,
            actions=[node_rviz]
        ),
    ])
