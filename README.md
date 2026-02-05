# ROS2 机器人导航与控制项目

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble%20|%20Iron-blue" alt="ROS2">
  <img src="https://img.shields.io/badge/C%2B%2B-17-brightgreen" alt="C++17">
  <img src="https://img.shields.io/badge/License-Apache%202.0-orange" alt="License">
</p>

一套完整的 ROS2 机器人导航与控制模块，包含全局路径规划、局部路径规划、轨迹跟踪和仿真测试等功能。

## 项目结构

```
src/
├── global_planner/          # 全局路径规划模块
│   ├── algorithms/          # 规划算法
│   │   ├── astar.cpp       # A* 算法
│   │   └── rrt_connect.cpp # RRT-Connect 算法
│   ├── smoother/           # 路径平滑器
│   ├── launch/            # 启动文件
│   ├── params/           # 配置文件
│   ├── maps/             # 地图文件
│   └── rviz/             # RViz 配置
│
├── local_planner/           # 局部路径规划模块
│   ├── algorithms/         # 规划算法
│   │   ├── ilqr.cpp       # iLQR 轨迹优化
│   │   └── lattice_planner.cpp  # Lattice 规划器
│   ├── utils/             # 工具函数
│   ├── launch/            # 启动文件
│   └── params/           # 配置文件
│
├── trajectory_tracker/      # 轨迹跟踪控制器
│   ├── algorithms/        # 控制算法
│   │   ├── lqr_controller.cpp   # LQR 控制器
│   │   ├── mpc_controller.cpp   # MPC 控制器
│   │   └── pure_pursuit_controller.cpp  # 纯追踪控制器
│   ├── launch/            # 启动文件
│   └── params/           # 配置文件
│
└── robot_model_pkg/        # 机器人模型包
    ├── urdf/             # 机器人模型 (URDF/Xacro)
    │   ├── ackermann_robot.urdf.xacro  # 阿克曼机器人
    │   └── tri_wheel_robot.urdf.xacro  # 三轮机器人
    ├── launch/           # 启动文件
    │   ├── ackermann_gazebo.launch.py   # Gazebo 仿真
    │   ├── slam_mapping.launch.py       # SLAM 建图
    │   └── display_robot_rviz.launch.py # RViz 显示
    ├── rviz/            # RViz 配置
    ├── worlds/          # Gazebo 世界文件
    ├── config/          # 配置参数
    └── src/             # 节点源码
```

## 功能特性

### 全局路径规划 (Global Planner)
- **A* 算法**：支持八邻域搜索，带地图膨胀
- **RRT-Connect**：快速随机树算法
- **路径平滑**：
  - 贝塞尔曲线转弯圆角
  - 三次样条全局平滑
  - 曲率连续性保证 (C2 连续)

### 局部路径规划 (Local Planner)
- **Lattice Planner**：基于采样的轨迹规划
  - Frenet 坐标系规划
  - 五次多项式轨迹生成
  - 多目标代价函数优化
- **iLQR**：迭代线性二次调节器
  - 非线性轨迹优化
  - 自行车运动学模型
  - 反馈增益计算

### 轨迹跟踪 (Trajectory Tracker)
- **LQR 控制器**：线性二次调节器
- **MPC 控制器**：模型预测控制
- **Pure Pursuit**：纯追踪算法

### 机器人模型 (Robot Model)
- **阿克曼机器人**：
  - 前后轴距 32cm，左右轮距 26cm
  - 3D LiDAR (16线) + 2D LiDAR (单线)
  - IMU + 深度相机 + 多角度相机
  - Gazebo Harmonic 仿真支持
- **三轮机器人**：
  - 差速驱动配置
  - 基础传感器配置

## 快速开始

### 环境要求
- Ubuntu 22.04
- ROS2 Humble
- C++17 编译器
- Gazebo Harmonic

### 安装依赖
```bash
sudo apt install ros-humble-desktop-full \
                 ros-humble-nav2-msgs \
                 ros-humble-tf2-ros \
                 ros-humble-gazebo-ros-pkgs \
                 libeigen3-dev
```

### 编译
```bash
cd ~/your_workspace/src
colcon build
source install/setup.bash
```

### 运行

#### 1. 启动阿克曼机器人仿真
```bash
ros2 launch robot_model_pkg ackermann_gazebo.launch.py
```

#### 2. 启动 SLAM 建图
```bash
ros2 launch robot_model_pkg slam_mapping.launch.py
```

#### 3. 启动全局规划演示
```bash
ros2 launch global_planner planner_demo.launch.py
```

#### 4. 启动局部规划演示
```bash
ros2 launch local_planner navigation_demo.launch.py
```

#### 5. 启动轨迹跟踪
```bash
ros2 launch trajectory_tracker tracker.launch.py
```

## 话题和服务

### 机器人话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制 |
| `/steering_angle` | `std_msgs/Float64` | 转向角控制 |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/imu` | `sensor_msgs/Imu` | IMU 数据 |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |

### 激光雷达话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/lidar_3d/scan/points` | `sensor_msgs/PointCloud2` | 3D 点云 (16线) |
| `/lidar_2d/scan` | `sensor_msgs/LaserScan` | 2D 激光扫描 |

### 相机话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/front_camera/image_raw` | `sensor_msgs/Image` | 前置RGB图像 |
| `/front_camera/depth` | `sensor_msgs/Image` | 前置深度图像 |
| `/rear_camera/image_raw` | `sensor_msgs/Image` | 后置图像 |
| `/left_camera/image_raw` | `sensor_msgs/Image` | 左侧图像 |
| `/right_camera/image_raw` | `sensor_msgs/Image` | 右侧图像 |

### 全局规划话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 栅格地图 |
| `/global_path` | `nav_msgs/Path` | 平滑后的全局路径 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 诊断信息 |

## 参数配置

### 机器人参数 (`robot_model_pkg/params/`)

### 规划器参数 (`global_planner/params/`, `local_planner/params/`)

### 跟踪器参数 (`trajectory_tracker/params/`)

## 开发计划

| 模块 | 功能 | 状态 |
|------|------|------|
| global_planner | A* + RRT + 路径平滑 | ✅ 完成 |
| local_planner | Lattice + iLQR | ✅ 完成 |
| trajectory_tracker | LQR + MPC + Pure Pursuit | ✅ 完成 |
| robot_model_pkg | 阿克曼 + 三轮模型 | ✅ 完成 |

## 许可证

Apache 2.0 - 详见 [LICENSE](LICENSE) 文件

## 作者

**Jiangqi Tan**
