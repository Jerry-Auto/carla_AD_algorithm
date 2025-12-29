cmake_minimum_required(VERSION 3.16)
project(pnc_agent)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找ament_cmake
find_package(ament_cmake REQUIRED)

# 查找所有依赖包
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(derived_object_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pnc_msgs REQUIRED)
find_package(carla_waypoint_types REQUIRED)
find_package(general_modules REQUIRED)
find_package(emplanner REQUIRED)
find_package(lattice REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pidlqr REQUIRED)
find_package(mpc REQUIRED)

# 添加规划节点
add_executable(planning_agent src/planning_agent.cpp)
target_include_directories(planning_agent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planning_agent
  rclcpp
  carla_msgs
  derived_object_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  pnc_msgs
  carla_waypoint_types
  visualization_msgs
  general_modules
  emplanner
  lattice
)
target_link_libraries(planning_agent OsqpEigen::OsqpEigen)

# 添加控制节点
add_executable(control_agent src/control_agent.cpp)
target_include_directories(control_agent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(control_agent
  rclcpp
  carla_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  pnc_msgs
  general_modules
  pidlqr
  mpc
  ament_index_cpp
)

# 安装可执行文件
install(TARGETS planning_agent control_agent
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件（如果有）
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()

# 导出依赖
ament_export_dependencies(
  rclcpp
  carla_msgs
  derived_object_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  pnc_msgs
  carla_waypoint_types
  visualization_msgs
  general_modules
  emplanner
  lattice
  OsqpEigen
  pidlqr
  mpc
)

ament_package()