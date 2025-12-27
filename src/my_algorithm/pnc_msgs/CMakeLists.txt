cmake_minimum_required(VERSION 3.5)
project(pnc_msgs)

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

# 查找核心依赖（必须按此顺序）
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # 消息生成器
find_package(std_msgs REQUIRED)  # 若消息依赖 std_msgs 需添加
find_package(rosidl_typesupport_c REQUIRED)  # 显式添加 C 类型支持依赖
find_package(rosidl_typesupport_cpp REQUIRED)  # 显式添加 C++ 类型支持依赖

# 声明你的消息文件（替换为实际的 .msg 文件名）
set(msg_files
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
)

# 生成消息接口（关键：指定类型支持）
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs  # 依赖的其他消息包
  ADD_LINTER_TESTS  # 可选：添加代码检查
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_typesupport_c)
ament_export_dependencies(rosidl_typesupport_cpp)



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
