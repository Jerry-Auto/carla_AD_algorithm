cmake_minimum_required(VERSION 3.5)
project(planner)


# 设置默认构建类型为 Release（带调试信息），这是ROS2推荐的调试编译方式
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # 在单配置生成器（如Unix Makefiles）上设置默认构建类型
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "构建类型" FORCE)
  # 可选项：设置为纯Debug模式，但程序可能运行较慢
  # set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()

# 为特定编译器提供更优的调试信息（可选但推荐）
if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # 为GCC或Clang编译器生成更丰富的调试信息
    add_compile_options(-g3)
  endif()
endif()


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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(general_modules REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

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

# ========================
# 创建共享库
# ========================

set(LIBRARY_EXPORT_NAME "${PROJECT_NAME}_targets")

add_library(${PROJECT_NAME} SHARED
  src/path_planner.cpp
  src/speed_planner.cpp
  src/emplanner.cpp
  src/DP_solver.cpp
)

target_include_directories(${PROJECT_NAME}
PUBLIC 
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
PRIVATE 
 src
)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp          
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
  general_modules::general_modules 
)

# ========================
# 创建可执行文件（测试程序）
# ========================

add_executable(planner_test test/main.cpp)

target_include_directories(planner_test
  PRIVATE 
    include
)

target_link_libraries(planner_test
  ${PROJECT_NAME}
  rclcpp::rclcpp
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
  general_modules::general_modules
  Python3::Python
  Python3::NumPy
)


# 安装可执行文件
install(
  TARGETS planner_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ========================
# 安装规则
# ========================

install(
  DIRECTORY include/  
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${LIBRARY_EXPORT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(${LIBRARY_EXPORT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  Eigen3
  OsqpEigen
  general_modules
) 

ament_package()