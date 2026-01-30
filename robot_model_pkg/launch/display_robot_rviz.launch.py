import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. 指定包名和文件路径
    package_name = 'robot_model_pkg'
    xacro_file_name = 'tri_wheel_robot.urdf.xacro'
    
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'urdf', xacro_file_name)
    rviz_config = os.path.join(pkg_path, 'rviz', 'robot_sensors.rviz')

    # 2. 使用 xacro 命令解析文件
    robot_description_config = Command(['xacro ', xacro_file])
    
    # 使用 ParameterValue 包装，避免 YAML 解析错误
    robot_description = ParameterValue(robot_description_config, value_type=str)
    
    # 3. 定义节点
    # 节点 A: robot_state_publisher (必须)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # 节点 B: joint_state_publisher_gui (可选，用于在RViz里手动拨动关节)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 节点 C: rviz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )

    # 4. 启动所有节点
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
