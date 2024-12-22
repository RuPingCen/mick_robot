cmake_minimum_required(VERSION 3.5)
project(oradar_lidar)

#=======================================
# Compile setup (CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD COLCON)

set(CMAKE_BUILD_TYPE RELEASE)

add_definitions(-O3)

#ROS#
find_package(roscpp 1.12 QUIET)
if(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Found. ROS Support is turned On!")
  message(=============================================================)
  #configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros1.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  add_definitions(-DROS_FOUND)
  #configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros1.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  include_directories(${roscpp_INCLUDE_DIRS})

else(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif(roscpp_FOUND)

#ROS2#
find_package(rclcpp QUIET)
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  message(=============================================================)
  message("-- ROS2 Found. ROS2 Support is turned On!")
  message(=============================================================)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  add_definitions(-DROS2_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)

  set(CMAKE_CXX_STANDARD 14)

  # find dependencies
  find_package(ament_cmake REQUIRED)
  #find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED) 
  include_directories(${rclcpp_INCLUDE_DIRS})

else(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  message(=============================================================)
  message("-- ROS2 Not Found. Ros2 Support is turned Off!")
  message(=============================================================)
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")


#Catkin#
if(${COMPILE_METHOD} STREQUAL "CATKIN")
  find_package(catkin REQUIRED COMPONENTS 
      rosconsole
      roscpp
      sensor_msgs
      message_generation
      std_msgs
  )

  generate_messages(
    DEPENDENCIES std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
  )

endif(${COMPILE_METHOD} STREQUAL "CATKIN")


# Bin and Install
set(ORADAR_SDK_DIR "sdk")
add_subdirectory(${ORADAR_SDK_DIR})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ORADAR_SDK_DIR}
)

add_executable(oradar_scan  ${CMAKE_CURRENT_SOURCE_DIR}/src/oradar_scan_node.cpp)

#Ros#
if(roscpp_FOUND)
  add_dependencies(oradar_scan
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
  target_link_libraries(oradar_scan
    ${catkin_LIBRARIES}
    oradar_sdk
  )

  install(TARGETS oradar_scan
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY launch rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
  )
endif(roscpp_FOUND)

#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  ament_target_dependencies(oradar_scan rclcpp sensor_msgs)
  target_link_libraries(oradar_scan oradar_sdk pthread)
  
  install(TARGETS oradar_scan
  DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch rviz2
  DESTINATION share/${PROJECT_NAME}/
  )

  ament_package()

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
