# Motion Planning Kit

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble%20|%20Iron%20|%20Jazzy-blue" alt="ROS2">
  <img src="https://img.shields.io/badge/C%2B%2B-17-brightgreen" alt="C++17">
  <img src="https://img.shields.io/badge/License-Apache%202.0-orange" alt="License">
</p>

一套完整的 ROS2 机器人运动规划与控制模块，包含全局路径规划、路径平滑、轨迹跟踪等功能。

## 功能特性

### 全局路径规划 (Global Planner)
- **A* 算法**：支持八邻域搜索，带地图膨胀
- **路径平滑**：
  - 贝塞尔曲线转弯圆角
  - 三次样条全局平滑
  - 曲率连续性保证 (C2 连续)
- **Lifecycle Node**：完整的生命周期管理
- **诊断信息**：实时发布规划状态和性能指标

### 局部路径规划 (Local Planner) - 新增
- **Lattice Planner**：基于采样的轨迹规划
  - Frenet 坐标系规划
  - 五次多项式轨迹生成
  - 多目标代价函数优化
- **iLQR**：迭代线性二次调节器
  - 非线性轨迹优化
  - 自行车运动学模型
  - 反馈增益计算

### 即将支持
- [ ] RRT-Connect 算法
- [ ] Dijkstra 算法
- [ ] MPC 轨迹跟踪控制
- [ ] 纯追踪控制器 (Pure Pursuit)

## 系统架构

```
motion_planning_kit/
├── global_planner/          # 全局路径规划模块
│   ├── algorithms/          # 规划算法
│   │   ├── astar.cpp       # A* 算法
│   │   └── rrt_connect.cpp # RRT-Connect (开发中)
│   ├── smoother/           # 路径平滑器
│   └── planner_node.cpp    # 规划节点
├── local_planner/           # 局部路径规划 (开发中)
├── trajectory_tracker/      # 轨迹跟踪控制 (开发中)
└── robot_model_pkg/         # 机器人模型
```

## 快速开始

### 环境要求
- Ubuntu 22.04 / 24.04
- ROS2 Humble / Iron / Jazzy
- C++17 编译器

### 安装依赖
```bash
sudo apt install ros-${ROS_DISTRO}-nav2-msgs \
                 ros-${ROS_DISTRO}-nav2-map-server \
                 ros-${ROS_DISTRO}-nav2-lifecycle-manager \
                 ros-${ROS_DISTRO}-tf2-ros \
                 libeigen3-dev
```

### 编译
```bash
cd ~/your_workspace
colcon build --packages-select global_planner robot_model_pkg
source install/setup.bash
```

### 运行
```bash
ros2 launch global_planner planner_demo.launch.py
```

## 话题和服务

### 订阅话题
| 话题 | 类型 | 描述 |
|------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 栅格地图 |

### 发布话题
| 话题 | 类型 | 描述 |
|------|------|------|
| `/global_path` | `nav_msgs/Path` | 平滑后的全局路径 |
| `/global_raw_path` | `nav_msgs/Path` | 原始规划路径 |
| `/inflated_map` | `nav_msgs/OccupancyGrid` | 膨胀后的地图 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 诊断信息 |

### Action 服务
| Action | 类型 | 描述 |
|--------|------|------|
| `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` | 计算到目标点的路径 |

## 参数配置

配置文件位于 `global_planner/params/planner_config.yaml`：

```yaml
global_planner_node:
  ros__parameters:
    # 规划器配置
    planner_type: "ASTAR"
    use_inflation: true
    inflation_radius: 0.30
    
    # 平滑器配置
    smoother:
      corner_radius: 0.5      # 转弯圆角半径
      sample_step: 0.05       # 采样步长
      smooth_weight: 0.5      # 平滑权重
```

## 路径平滑效果

路径平滑处理流程：

```
原始 A* 路径 → 路径裁剪 → 贝塞尔圆角 → 样条重采样 → 平滑路径
     ↓              ↓           ↓            ↓
  锯齿状路径    移除冗余点   转弯处平滑   曲率连续
```

输出路径包含：
- 位置 (x, y)
- 航向角 (theta) - 连续
- 曲率 (kappa) - 连续

## Lifecycle 节点管理

本模块使用 ROS2 Lifecycle Node，支持以下状态：

```
unconfigured → inactive → active → inactive → finalized
      ↓           ↓          ↓
  configure()  activate()  deactivate()
```

手动控制节点生命周期：
```bash
# 查看状态
ros2 lifecycle get /global_planner_node

# 激活节点
ros2 lifecycle set /global_planner_node configure
ros2 lifecycle set /global_planner_node activate
```

## 代码质量

- 内存安全：使用对象池和智能指针
- 线程安全：mutex 保护共享资源
- 无 `using namespace std`
- Doxygen 风格注释
- 单元测试覆盖 (开发中)

## 开发计划

| 版本 | 功能 | 状态 |
|------|------|------|
| v1.0 | A* 全局规划 + 路径平滑 | ✅ 完成 |
| v1.1 | RRT-Connect 算法 | 🚧 开发中 |
| v1.2 | 局部路径规划 | 📋 计划中 |
| v2.0 | MPC 轨迹跟踪 | 📋 计划中 |

## 贡献指南

欢迎提交 Issue 和 Pull Request！

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建 Pull Request

## 许可证

本项目采用 Apache 2.0 许可证 - 详见 [LICENSE](LICENSE) 文件

## 作者

- **Jiangqi Tan**

## 致谢

- [Nav2](https://navigation.ros.org/) - ROS2 导航框架
- [CppRobotics](https://github.com/AtsushiSakai/PythonRobotics) - 机器人算法参考
