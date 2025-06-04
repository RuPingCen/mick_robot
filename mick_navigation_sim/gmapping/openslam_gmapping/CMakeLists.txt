cmake_minimum_required(VERSION 3.5)
project(openslam_gmapping)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O3 -funroll-loops)
endif()

add_compile_options(-fPIC)
find_package(ament_cmake REQUIRED)

include_directories(include)

add_subdirectory(gridfastslam)
add_subdirectory(scanmatcher)
add_subdirectory(sensor)
add_subdirectory(utils)

ament_export_libraries(gridfastslam)
ament_export_libraries(scanmatcher)
ament_export_libraries(sensor_base)
ament_export_libraries(sensor_odometry)
ament_export_libraries(sensor_range)
ament_export_libraries(utils)
ament_export_include_directories(include)

install(DIRECTORY include/
        DESTINATION include/)

ament_package()
