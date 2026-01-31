cmake_minimum_required(VERSION 3.5)
project(cilqr_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
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
find_package(casadi REQUIRED)
find_package(general_modules REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)


# Library
add_library(${PROJECT_NAME} STATIC
  src/cilqr_planner.cpp
  src/cost_define.cpp
  src/cost_factory.cpp
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
  Python3::Python
  Python3::NumPy
  casadi::casadi
  general_modules::general_modules
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

# Test executable
add_executable(${PROJECT_NAME}_test
  test/main.cpp
)

target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  rclcpp::rclcpp
  Python3::Python
  Python3::NumPy
  casadi::casadi
  general_modules::general_modules
)


ament_target_dependencies(${PROJECT_NAME}_test)


set(test_cost "test_cost_define")
# Test executable
add_executable(${test_cost}
  test/test_simple.cpp
)

target_link_libraries(${test_cost}
  ${PROJECT_NAME}
  rclcpp::rclcpp
  Python3::Python
  Python3::NumPy
  casadi::casadi
  general_modules::general_modules
)

ament_target_dependencies(${test_cost})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${test_cost}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs Eigen3 OsqpEigen general_modules casadi)

ament_package()
