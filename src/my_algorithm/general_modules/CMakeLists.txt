cmake_minimum_required(VERSION 3.5)
project(general_modules)

# 使用 C++17（推荐）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# 查找依赖
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

# 设置库的导出名称
set(LIBRARY_EXPORT_NAME "${PROJECT_NAME}_targets")

# ========================
# 创建静态库 libgeneral_modules.a
# ========================
add_library(${PROJECT_NAME} STATIC
  src/ReferenceLine.cpp
  src/FrenetFrame.cpp
  src/Vehicle.cpp
  src/Trajectory.cpp
  src/math_tool.cpp
  src/ST_graph.cpp
  src/visualization_tool.cpp
  src/csv_logger.cpp
  src/polynomial_curve.cpp
  src/collision_detection.cpp
  src/common_types.cpp
)

# 设置头文件路径（对库本身）
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    src
)

# 链接依赖库
target_link_libraries(${PROJECT_NAME}
  OsqpEigen::OsqpEigen
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  spdlog
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  visualization_msgs
)

# ========================
# 创建测试可执行文件
# ========================
add_executable(test_frenet test/main.cpp)

# 给可执行文件包含头文件
target_include_directories(test_frenet
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接你的库 + 依赖
target_link_libraries(test_frenet
  ${PROJECT_NAME}
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
  spdlog
)

ament_target_dependencies(test_frenet
  rclcpp
  nav_msgs
  visualization_msgs
)

# ========================
# 安装规则
# ========================

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装 lut 文件夹到 share/general_modules/
install(
  DIRECTORY lut/
  DESTINATION share/${PROJECT_NAME}/lut
)

# 安装库
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${LIBRARY_EXPORT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装可执行文件（测试程序）
install(TARGETS test_frenet DESTINATION lib/${PROJECT_NAME})

# ========================
# 导出接口
# ========================
ament_export_targets(${LIBRARY_EXPORT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  OsqpEigen
  Eigen3
)

# ========================
# 测试（可选）
# ========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()