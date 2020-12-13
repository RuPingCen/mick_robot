
add_library(range_conversion ToRange.cpp)
target_link_libraries(range_conversion
                      ${catkin_LIBRARIES})

add_executable(range_node range_node.cpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(range_conversion ${catkin_EXPORTED_TARGETS})
endif()

#add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(range_node 
	range_conversion
	${catkin_LIBRARIES}
)


