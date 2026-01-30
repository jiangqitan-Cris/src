import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ============================================================================
    #                            路径设置
    # ============================================================================
    package_name = 'robot_model_pkg'
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'ackermann_robot.urdf.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'robot_sensors.rviz')
    world_file = os.path.join(pkg_path, 'worlds', 'test_world.sdf')

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
    #                            Robot Description
    # ============================================================================
    robot_description_config = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_config, value_type=str)

    # ============================================================================
    #                            Nodes
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

    # Gazebo
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn robot
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

    # Bridge - Core
    node_bridge_core = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_core',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # 速度控制 (后轮驱动)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # 转向角控制 (前轮转向) - 直接控制转角 (rad)
            '/steering_angle@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    # Bridge - Odometry
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

    # Bridge - Joint States
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

    # Bridge - IMU
    node_bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # Bridge - LiDAR
    node_bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_lidar',
        arguments=[
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # Bridge - Images
    node_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/front_camera/image_raw',
            '/front_camera/depth',
            '/rear_camera/image_raw',
            '/left_camera/image_raw',
            '/right_camera/image_raw',
        ],
        output='screen'
    )

    # Static TF (backup)
    node_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_robot_state_publisher,
        gz_sim_launch,
        node_spawn_entity,
        node_bridge_core,
        node_bridge_odom,
        node_bridge_joint_states,
        node_bridge_imu,
        node_bridge_lidar,
        node_image_bridge,
        node_static_tf,
        node_rviz,
    ])
