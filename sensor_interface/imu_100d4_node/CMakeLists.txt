cmake_minimum_required(VERSION 3.5)
project(imu_100d4_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# 查找必要的ROS 2包
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread atomic system regex)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含头文件
include_directories(include)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

# 添加可执行文件
add_executable(imu_100d4_node src/imu_100d4_node.cpp)

# 链接库
ament_target_dependencies(imu_100d4_node
  rclcpp
  tf2
  tf2_geometry_msgs
  sensor_msgs
  Boost
)

# 安装目标
install(TARGETS
  imu_100d4_node
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY launch
 DESTINATION share/${PROJECT_NAME}
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
