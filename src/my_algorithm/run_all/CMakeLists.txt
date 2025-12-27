cmake_minimum_required(VERSION 3.5)
project(run_all)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# run_all additional runtime nodes
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(goal_to_waypoints src/goal_to_waypoints.cpp)
ament_target_dependencies(goal_to_waypoints rclcpp nav_msgs geometry_msgs)

# 添加车辆信息检查器
add_executable(vehicle_info_checker src/vehicle_info_checker.cpp)
ament_target_dependencies(vehicle_info_checker rclcpp carla_msgs)

add_executable(vehicle_info_publisher src/vehicle_info_publisher.cpp)
ament_target_dependencies(vehicle_info_publisher rclcpp geometry_msgs carla_msgs)

install(TARGETS
  goal_to_waypoints
  vehicle_info_publisher
  vehicle_info_checker
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
