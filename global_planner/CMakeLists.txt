cmake_minimum_required(VERSION 3.8)
project(global_planner)

# 1. 设置编译选项：C++17 是 ROS2 Humble/Iron/Jazzy 的标配
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 3. 定义包含目录
include_directories(
  include
  include/${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/utils
  ${EIGEN3_INCLUDE_DIRS}
)

# 4. 编译可执行文件
add_executable(global_planner_node
  src/algorithms/astar.cpp
  src/algorithms/rrt_connect.cpp
  src/planner_component.cpp
  src/smoother/smoother.cpp
  src/planner_node.cpp
)

add_executable(rviz_goal_to_action_node 
  utils/rviz_goal_to_action_node.cpp
)

# 5. 链接依赖库
ament_target_dependencies(global_planner_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  nav_msgs
  geometry_msgs
  nav2_msgs
  diagnostic_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(rviz_goal_to_action_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

# 6. 安装指令
install(TARGETS
  global_planner_node
  rviz_goal_to_action_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  params
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# 7. 测试配置（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 8. 导出环境
ament_package()
