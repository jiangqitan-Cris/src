import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 指定包名和文件路径
    package_name = 'robot_model_pkg' # 请确保这是你的包名
    xacro_file_name = 'tri_wheel_robot.urdf.xacro' # 你的文件名
    
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'urdf', xacro_file_name)

    # 2. 使用 xacro 命令解析文件
    # 这相当于在终端运行: xacro tri_wheel_robot.urdf.xacro
    robot_description_config = Command(['xacro ', xacro_file])
    
    # 3. 定义节点
    # 节点 A: robot_state_publisher (必须)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
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
        output='screen'
    )

    # 4. 启动所有节点
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])