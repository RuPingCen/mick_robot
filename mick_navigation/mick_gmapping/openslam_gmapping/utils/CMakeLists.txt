include_directories(./)
add_library(utils movement.cpp stat.cpp)
install(TARGETS utils DESTINATION lib)

ament_export_libraries(utils)
