cmake_minimum_required(VERSION 3.5)
project(emplanner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

# ========================
# Create Library
# ========================

add_library(${PROJECT_NAME} STATIC
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
# Install
# ========================

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
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


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs Eigen3 OsqpEigen general_modules)

ament_package()
