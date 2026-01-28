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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 3. 定义包含目录（头文件路径）
include_directories(
  include
  include/${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ${catkin_INCLUDE_DIRS}
)

# 4. 编译可执行文件
# 我们把算法、组件、节点代码全部打包编译成一个可执行程序
add_executable(global_planner_node
  src/algorithms/astar.cpp
  src/planner_component.cpp
  src/smoother/smoother.cpp
  src/planner_node.cpp
)
add_executable(rviz_goal_to_action_node 
            utils/rviz_goal_to_action_node.cpp)

# 5. 链接依赖库
target_link_libraries(global_planner_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)
target_link_libraries(rviz_goal_to_action_node 
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

# 这一步非常关键：由于 ROS2 使用了大量的现代 CMake 目标
# 我们需要把所有 find_package 找到的库都挂载上去
ament_target_dependencies(global_planner_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  nav2_msgs
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

# 6. 安装指令：这决定了 colcon build 之后，二进制文件和配置文件的去向
install(TARGETS
  global_planner_node
  rviz_goal_to_action_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  params
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# 7. 导出环境（AMENT 必备）
ament_package()