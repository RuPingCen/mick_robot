add_library(sensor_odometry odometryreading.cpp odometrysensor.cpp)
target_link_libraries(sensor_odometry sensor_base)

install(TARGETS sensor_odometry DESTINATION lib)

ament_export_libraries(sensor_odometry)
