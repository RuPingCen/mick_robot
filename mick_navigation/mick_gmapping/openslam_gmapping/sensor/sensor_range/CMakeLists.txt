add_library(sensor_range rangereading.cpp rangesensor.cpp)
target_link_libraries(sensor_range sensor_base)
#ament_target_dependencies(sensor_range sensor_base)

install(TARGETS sensor_range DESTINATION lib)

ament_export_libraries(sensor_range)