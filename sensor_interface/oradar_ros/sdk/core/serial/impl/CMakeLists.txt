IF (WIN32)
add_subdirectory(windows)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/windows)
ELSE()
add_subdirectory(unix)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/unix)
ENDIF()

