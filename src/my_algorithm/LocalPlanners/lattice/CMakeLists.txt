cmake_minimum_required(VERSION 3.5)
project(lattice)

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
find_package(general_modules REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# ========================
# Create Library
# ========================

add_library(${PROJECT_NAME} STATIC
  src/lattice_planner.cpp
  src/lattice_collision_detection.cpp
  src/trajectory1d_generator.cpp
  src/trajectory_evaluator.cpp
)

add_executable(test_lattice_planner test/test_lattice_planner_main.cpp)

target_include_directories(test_lattice_planner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

target_link_libraries(test_lattice_planner
  ${PROJECT_NAME}
  rclcpp::rclcpp
  general_modules::general_modules
  Python3::Python
  Python3::NumPy
)

ament_target_dependencies(test_lattice_planner
  rclcpp
)

install(TARGETS test_lattice_planner DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_trajectory_evaluator_unit test/test_trajectory_evaluator.cpp)
  if(TARGET test_trajectory_evaluator_unit)
    target_include_directories(test_trajectory_evaluator_unit PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    target_link_libraries(test_trajectory_evaluator_unit
      ${PROJECT_NAME}
      rclcpp::rclcpp
      general_modules::general_modules
    )
    ament_target_dependencies(test_trajectory_evaluator_unit
      rclcpp
    )
  endif()
endif()


target_include_directories(${PROJECT_NAME}
PUBLIC 
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
PRIVATE 
 src
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp          
  Eigen3::Eigen
  general_modules::general_modules 
)

# ========================
# Unit tests (simple checks)
# ========================
add_executable(test_trajectory1d_generator test/trajectory1d_generator_main.cpp)

target_include_directories(test_trajectory1d_generator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

target_link_libraries(test_trajectory1d_generator
  ${PROJECT_NAME}
  rclcpp::rclcpp
  general_modules::general_modules
)

ament_target_dependencies(test_trajectory1d_generator
  rclcpp
)

install(TARGETS test_trajectory1d_generator DESTINATION lib/${PROJECT_NAME})

add_executable(test_lattice_collision_detection test/lattice_collision_detection_main.cpp)

target_include_directories(test_lattice_collision_detection PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

target_link_libraries(test_lattice_collision_detection
  ${PROJECT_NAME}
  rclcpp::rclcpp
  general_modules::general_modules
)

ament_target_dependencies(test_lattice_collision_detection
  rclcpp
)

install(TARGETS test_lattice_collision_detection DESTINATION lib/${PROJECT_NAME})

add_executable(test_trajectory_evaluator test/trajectory_evaluator_main.cpp)

target_include_directories(test_trajectory_evaluator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

target_link_libraries(test_trajectory_evaluator
  ${PROJECT_NAME}
  rclcpp::rclcpp
  general_modules::general_modules
)

ament_target_dependencies(test_trajectory_evaluator
  rclcpp
)

install(TARGETS test_trajectory_evaluator DESTINATION lib/${PROJECT_NAME})

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs Eigen3 general_modules)

ament_package()
