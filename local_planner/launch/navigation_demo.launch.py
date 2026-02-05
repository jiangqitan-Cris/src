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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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
    # trajectory_tracker_pkg = get_package_share_directory('trajectory_tracker')
    
    # ============================================================================
    #                            文件路径
    # ============================================================================
    # 机器人模型
    xacro_file = os.path.join(robot_pkg, 'urdf', 'ackermann_robot.urdf.xacro')
    world_file = os.path.join(robot_pkg, 'worlds', 'test_world.sdf')
    
    # 配置文件
    global_planner_config = os.path.join(global_planner_pkg, 'params', 'planner_config.yaml')
    local_planner_config = os.path.join(local_planner_pkg, 'params', 'local_planner_config.yaml')
    amcl_config = os.path.join(local_planner_pkg, 'params', 'amcl.yaml')
    # tracker_config = os.path.join(trajectory_tracker_pkg, 'params', 'tracker_config.yaml')
    
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
    planner_type = LaunchConfiguration('planner_type', default='lattice')
    use_localization = LaunchConfiguration('use_localization', default='false')
    # controller_type = LaunchConfiguration('controller_type', default='lqr')
    
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
        description='Visualize Lattice planner candidate trajectories (only for lattice)'
    )
    
    declare_planner_type = DeclareLaunchArgument(
        'planner_type', default_value='lattice',
        description='Local planner algorithm: lattice or ilqr'
    )

    declare_use_localization = DeclareLaunchArgument(
        'use_localization', default_value='false',
        description='Use AMCL localization to publish dynamic map->odom TF (disable to use static map->odom)'
    )
    
    # 机器人生成位置参数
    spawn_x = LaunchConfiguration('spawn_x', default='-2.0')
    spawn_y = LaunchConfiguration('spawn_y', default='1.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.1')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='0.0')
    
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x', default_value='-2.0',
        description='Robot spawn X position'
    )
    
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y', default_value='1.0',
        description='Robot spawn Y position'
    )
    
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z', default_value='0.1',
        description='Robot spawn Z position'
    )
    
    declare_spawn_yaw = DeclareLaunchArgument(
        'spawn_yaw', default_value='0.0',
        description='Robot spawn yaw angle (radians)'
    )

    # declare_controller_type = DeclareLaunchArgument(
    #     'controller_type', default_value='lqr',
    #     description='Trajectory tracker: lqr, pure_pursuit, or mpc'
    # )
    
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
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,  # Yaw angle
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
    
    # Odom / TF 由 Ackermann 里程计与 robot_state_publisher 提供，此处不再从 Gazebo bridge
    node_bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=[],
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
    # map -> odom 静态变换（仅在不启用定位时使用）
    node_static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(use_localization),
    )
    
    # ============================================================================
    #                            地图服务器
    # ============================================================================
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ]
    )

    # ============================================================================
    #                            定位 (AMCL)
    # ============================================================================
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
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
                'amcl',
                'global_planner_node'
            ],
            'bond_timeout': 4.0,
        }]
    )
    
    # ============================================================================
    #                            Ackermann 里程计 (odom + TF)
    # ============================================================================
    ackermann_odom_node = Node(
        package='robot_model_pkg',
        executable='ackermann_odom_node',
        name='ackermann_odom_node',
        output='screen',
        parameters=[{'wheelbase': 0.32, 'use_sim_time': use_sim_time}],
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
                'planner_type': planner_type,
            }
        ],
        remappings=[
            ('/map', '/map'),
            ('/global_path', '/global_path'),
            ('/odom', '/odom'),
        ]
    )
    
    # ============================================================================
    #                            轨迹跟踪控制器
    # ============================================================================
    # tracker_node = Node(
    #     package='trajectory_tracker',
    #     executable='tracker_node',
    #     name='tracker_node',
    #     output='screen',
    #     parameters=[
    #         tracker_config,
    #         {'use_sim_time': use_sim_time, 'controller_type': controller_type},
    #     ],
    #     remappings=[
    #         ('/local_path', '/local_path'),
    #         ('/odom', '/odom'),
    #         ('/cmd_vel', '/cmd_vel'),
    #         ('/steering_angle', '/steering_angle'),
    #     ]
    # )

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
        declare_planner_type,
        declare_use_localization,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        # declare_controller_type,
        
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
                amcl_node,
                global_planner_node,
                rviz_goal_bridge_node,
                lifecycle_manager,
            ]
        ),
        
        # 启用定位时：延迟 5s 后自动发一次 /initialpose（从 /odom 取当前位姿），无需在 RViz 里点
        GroupAction(
            condition=IfCondition(use_localization),
            actions=[
                TimerAction(
                    period=5.0,
                    actions=[
                        Node(
                            package='local_planner',
                            executable='publish_initial_pose.py',
                            name='publish_initial_pose',
                            output='screen',
                            parameters=[
                                {'use_sim_time': use_sim_time},
                                {'delay_sec': 2.0},
                                {'once': True},
                            ],
                        ),
                    ]
                ),
            ]
        ),
        # 局部规划 (延迟启动，等待全局规划器)
        TimerAction(
            period=5.0,
            actions=[
                ackermann_odom_node,
                local_planner_node,
            ]
        ),
        # 轨迹跟踪控制 (延迟启动，等待局部规划)
        # TimerAction(
        #     period=6.0,
        #     actions=[tracker_node]
        # ),
        # RViz
        TimerAction(
            period=2.0,
            actions=[node_rviz]
        ),
    ])
