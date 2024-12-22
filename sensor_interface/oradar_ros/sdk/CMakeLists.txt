cmake_minimum_required(VERSION 3.5)
project(oradar_sdk C CXX)

##########################################################
# Detect wordsize:
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	SET(CMAKE_MRPT_WORD_SIZE 64)
ELSE()
	SET(CMAKE_MRPT_WORD_SIZE 32)
ENDIF()
##################################################
#c++ 11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
##########################################################
# add -fPIC
add_compile_options(-fPIC)
#or
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#####################################################
# add cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(SDK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#############################################################################
# include cmake file
include(oradar_base)
include(install_package)

#############################################################################
# Policy CMP0023 allows to mix old and new interfaces of target_link_libraries
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW) # We don't want to mix relative and absolute paths in linker lib lists.
	cmake_policy(SET CMP0005 NEW) # Escape definitions (-D) strings
	if(POLICY CMP0053)
		cmake_policy(SET CMP0053 OLD) # Simplify variable reference and escape sequence evaluation.
	endif()
	if(POLICY CMP0037)
		cmake_policy(SET CMP0037 OLD)  # Allow defining target "test"
	endif()
	if(POLICY CMP0043)
		cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()
endif()

#############################################################################
# option
option( BUILD_SHARED_LIBS "Build shared libraries." OFF)
option( BUILD_EXAMPLES "Build Example." ON)

############################################################################
# find package
FIND_PACKAGE(SWIG)
find_package(PythonInterp)
FIND_PACKAGE(PythonLibs)

############################################################################
# include headers
include_directories(.)
include_directories(core)
include_directories(src)

#############################################################################
# addd subdirectory
add_subdirectory(core)
add_subdirectory(src)

##############################
#build examples
if(BUILD_EXAMPLES)
add_subdirectory(samples)
endif()

#############################################################################
# PARSE libraries
include(oradar_parse)
include_directories(${SDK_INCS})


#############################################################################
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

#############################################################################################
#shared library
if(BUILD_SHARED_LIBS)
oradar_add_library(${PROJECT_NAME} SHARED ${SDK_SOURCES} ${SDK_HEADERS} ${GENERATED_HEADERS})
else()
oradar_add_library(${PROJECT_NAME} STATIC ${SDK_SOURCES} ${SDK_HEADERS} ${GENERATED_HEADERS})
endif()

target_link_libraries(${PROJECT_NAME} ${SDK_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)



###############################################################################
# append path
list(APPEND SDK_INCS ${CMAKE_INSTALL_PREFIX}/include/src
    ${CMAKE_INSTALL_PREFIX}/include)


###############################################################################
# install package
string(TOUPPER ${PROJECT_NAME} PROJECT_PKG_NAME)
install_package(
    PKG_NAME ${PROJECT_PKG_NAME}
    LIB_NAME ${PROJECT_NAME}
    INSTALL_HEADERS ${SDK_HEADERS}
    INSTALL_GENERATED_HEADERS ${GENERATED_HEADERS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    INCLUDE_DIRS ${SDK_INCS}
    LINK_LIBS ${SDK_LIBS}
    )

##################################################################################
# make an uninstall target
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in)
add_custom_target(sdk_uninstall
    COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")


