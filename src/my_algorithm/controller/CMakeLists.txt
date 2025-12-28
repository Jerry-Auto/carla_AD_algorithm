cmake_minimum_required(VERSION 3.5)
project(controller)

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
find_package(general_modules REQUIRED)

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
  src/lateral_lqr_controller.cpp
  src/lon_cascade_pid_controller.cpp
  src/pid_controller.cpp
  src/Vehicle_model.cpp
  src/mpc_controller.cpp
  src/pid_lqr_controller.cpp
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
  general_modules::general_modules
  spdlog
)

# 添加测试可执行程序
# ========================
add_executable(test_controllers test/main.cpp)


# 将测试程序链接到 controller 库和依赖
target_link_libraries(test_controllers
  ${PROJECT_NAME}
  rclcpp::rclcpp
  Eigen3::Eigen
  general_modules::general_modules
  spdlog
)

# 设置头文件搜索路径给测试程序
target_include_directories(test_controllers
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
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

install(TARGETS test_controllers DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${LIBRARY_EXPORT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  Eigen3
  general_modules
) 



ament_package()
