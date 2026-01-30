cmake_minimum_required(VERSION 3.8)
project(local_planner)

# C++17 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 编译库
add_library(${PROJECT_NAME}_lib SHARED
  src/utils/vehicle_model.cpp
  src/algorithms/lattice_planner.cpp
  src/algorithms/ilqr.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
)

# 安装
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY params launch
  DESTINATION share/${PROJECT_NAME}
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs Eigen3)

ament_package()
