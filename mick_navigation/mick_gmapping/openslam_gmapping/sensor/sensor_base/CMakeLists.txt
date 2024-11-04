add_library(sensor_base sensor.cpp sensorreading.cpp)
install(TARGETS sensor_base DESTINATION lib)

ament_export_libraries(sensor_base)