cmake_minimum_required(VERSION 3.8)
project(mickrobot_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 检测ROS版本
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  # 自动检测ROS安装
  find_package(ROS QUIET COMPONENTS roscpp)
  if(ROS_FOUND)
    set(ROS_VERSION 1)
  else()
    find_package(ament_cmake QUIET)
    if(ament_cmake_FOUND)
      set(ROS_VERSION 2)
    endif()
  endif()
endif()



if(${ROS_VERSION} EQUAL 1)
  message("MickRobot Node -- ROS1 Found. ")
  find_package(Eigen3 REQUIRED)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    serial
    tf
    nav_msgs
    rostime sensor_msgs 
  )
  
  catkin_package(
  #  INCLUDE_DIRS include
    CATKIN_DEPENDS
    serial
    std_msgs
  )

  include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )

  add_executable(mickrobot_bringup src/mickx4_bringup_v3.cpp)
  target_link_libraries(mickrobot_bringup
    ${catkin_LIBRARIES}
  )

  ## Mark executables and/or libraries for installation
  install(TARGETS mickrobot_bringup
  #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
elseif(${ROS_VERSION} EQUAL 2)
  message("MickRobot Node -- ROS2 Found. ")
  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)


  set(CMAKE_INSTALL_RPATH /usr/local/lib)
  find_package(serial REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  #add_executable(mick_node src/mickx4_bringup_v3.cpp  src/mick_chassis_protocol.cpp)
  #  
  #ament_target_dependencies(mick_node 
  #  rclcpp 
  #  std_msgs
  #  sensor_msgs
  #  tf2
  #  nav_msgs
  #  tf2_geometry_msgs
  #  serial
  #)

  add_executable(mickrobot_bringup src/mickx4_bringup_v3.1.cpp  src/mick_chassis_protocol.cpp)
  #add_executable(mickrobot_bringup src/mickx4_bringup_v3.cpp)
    
  ament_target_dependencies(mickrobot_bringup 
    rclcpp 
    std_msgs
    sensor_msgs
    tf2
    nav_msgs
    tf2_geometry_msgs
    serial
  )

  install(TARGETS
    mickrobot_bringup
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
endif()
