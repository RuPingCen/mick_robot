add_library(scanmatcher STATIC eig3.cpp scanmatcher.cpp scanmatcherprocessor.cpp smmap.cpp)
target_link_libraries(scanmatcher sensor_range utils)

install(TARGETS scanmatcher DESTINATION lib)
