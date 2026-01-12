import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 路径设置
    package_name = 'robot_model_pkg'
    xacro_file_name = 'tri_wheel_robot.urdf.xacro'
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'urdf', xacro_file_name)

    # 2. 解析 Xacro
    robot_description_config = Command(['xacro ', xacro_file])

    # 3. 发布机器人模型
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # 4. 启动 Gazebo
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 5. 在 Gazebo 中生成实体
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'tri_wheel_robot', '-z', '0.5']
    )

    # 6. 时钟桥接器
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 时钟同步
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # 速度控制话题 (cmd_vel) - ROS2发送, Gazebo接收
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # 里程计话题 (odom) - Gazebo发送, ROS2接收
            '/model/tri_wheel_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 关节状态 (Joint States) - Gazebo发送, ROS2接收
            '/world/empty/model/tri_wheel_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.ModelContactInfo',
            # 转向控制话题 - ROS2发送, Gazebo接收
            '/caster_steering/set_position@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        # 重映射可以让话题更简洁
        remappings=[
            ('/model/tri_wheel_robot/odometry', '/odom'),
            ('/world/empty/model/tri_wheel_robot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim_launch,
        node_ros_gz_bridge,
        node_spawn_entity,
    ])