
include(oradar_base)
#############################################################################
# Setup libraries

get_property( INTERNAL_INC  GLOBAL PROPERTY ORDLIDAR_INCLUDE_DIRS )
get_property( INTERNAL_LIBS GLOBAL PROPERTY ORDLIDAR_LIBRARIES )
get_property( SDK_SOURCES GLOBAL PROPERTY ORDLIDAR_SOURCES )
get_property( SDK_HEADERS GLOBAL PROPERTY ORDLIDAR_HEADERS )


# this is a horrible hack in order to set compiler flag properties to individual files
get_property( C_O_S GLOBAL PROPERTY COMPILER_OPTS_SOURCES )
get_property( C_O_F GLOBAL PROPERTY COMPILER_OPTS_FLAGS )

list(LENGTH C_O_S len_c_o_s )
math(EXPR len_c_o_s "${len_c_o_s} - 1" )

foreach(val RANGE ${len_c_o_s} )
  list(GET C_O_S ${val} source )
  list(GET C_O_F ${val} flag )
  set_source_files_properties( ${source} PROPERTIES COMPILE_FLAGS ${flag} )
endforeach()

##########################################################################

include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR} )
include_directories( ${LIB_INC_DIR} )
include_directories( ${INTERNAL_INC} )


##########################################################################
# ccache: a compiler, can speed up gcc compilation.
#
option(CCACHE "Use ccache if available" ON)
find_program(CCACHE_PROGRAM ccache)
if (CCACHE AND CCACHE_PROGRAM AND NOT DEFINED ENV{CCACHE_DISABLE})
        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
else()
endif()

##########################################################################
# generate compile command database
#
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

##########################################################################
# check required toolchain variables
#
# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


#############################################################################

set(SDK_LIBS
  ${INTERNAL_LIBS}
  ${LINK_LIBS}
  CACHE STRING "SDK required libraries"
  )

list( REMOVE_ITEM SDK_LIBS "debug" )
list( REMOVE_ITEM SDK_LIBS "optimized" )

set(SDK_INCS
    ${INTERNAL_INC}
    ${USER_INC}
    CACHE STRING "SDK required incs" )
