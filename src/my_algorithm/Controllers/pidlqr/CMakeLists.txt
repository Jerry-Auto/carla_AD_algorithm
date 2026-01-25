cmake_minimum_required(VERSION 3.5)
project(pidlqr)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(general_modules REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/lon_cascade_pid_controller.cpp
  src/lateral_lqr_controller.cpp
  src/pid_controller.cpp
  src/pid_lqr_controller.cpp
  src/Vehicle_model.cpp
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
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp Eigen3 general_modules)

ament_package()
