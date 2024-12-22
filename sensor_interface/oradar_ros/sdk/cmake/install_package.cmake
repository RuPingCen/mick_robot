################################################################################
# install_package.cmake - This function will install and "export" your library
#   or files in such a way that they can be found using either CMake's
#   "FindXXX.cmake" mechanism or with pkg-config.  This makes your code boradly
#   compatible with traditional unix best practices, and also easy to use from
#   other CMake projets.
# 
# This function will create and install a ${PACKAGE}.pc pkg-config file.
# Will also create a Find${PACKAGE}.cmake, which will in turn call.

#
# install_package - Takes a package name and the following optional named arguments:
#  PKG_NAME <name of the package for pkg-config>, usually the same as ${PROJECT_NAME}
#  LIB_NAME <name of a library to build, if any>
#  VERSION <version>
#  INSTALL_HEADERS <header files to install, if any>
#  DESTINATION <directory to install headers>
#  INCLUDE_DIRS <list of required include directories, if any>
#  LINK_LIBS <list of required link libraries, if any >
#  LINK_DIRS <list of required link directories, if any>
#  CFLAGS <optional list of REQUIRED c flags>
#  CXXFLAGS <optional list of REQUIRED c++ flags>
#
################################################################################
include(CMakeParseArguments)

get_filename_component(modules_dir ${CMAKE_CURRENT_LIST_FILE} PATH)

function(install_package)
  set(PACKAGE_OPTIONS "")
  set(PACKAGE_SINGLE_ARGS "")
  set( PACKAGE_MULTI_ARGS 
      PKG_NAME 
      LIB_NAME 
      VERSION
      DESCRIPTION
      INSTALL_HEADERS 
      INSTALL_GENERATED_HEADERS 
      INSTALL_HEADER_DIRS
      INSTALL_INCLUDE_DIR
      DESTINATION
      INCLUDE_DIRS
      LINK_LIBS
      LINK_DIRS
      CFLAGS
      CXXFLAGS
     )
  cmake_parse_arguments( PACKAGE
    "${PACKAGE_OPTIONS}"
    "${PACKAGE_SINGLE_ARGS}"
    "${PACKAGE_MULTI_ARGS}"
    "${ARGN}"
    )

  # Add package to CMake package registery for use from the build tree. RISKY.
  option( EXPORT_${PROJECT_NAME}
      "Should the ${PROJECT_NAME} package be exported for use by other software" OFF )

  mark_as_advanced( EXPORT_${PROJECT_NAME} )


  # clean things up 
  if( PACKAGE_LINK_DIRS )
    list( REMOVE_DUPLICATES PACKAGE_LINK_DIRS )
  endif()
  if(PACKAGE_LINK_LIBS)
    list( REMOVE_DUPLICATES PACKAGE_LINK_LIBS )
  endif()
  if( PACKAGE_INCLUDE_DIRS)
    list( REMOVE_DUPLICATES PACKAGE_INCLUDE_DIRS )
  endif()

  # construct Cflags arguments for pkg-config file
  set( PACKAGE_CFLAGS "${PACKAGE_CFLAGS} ${CMAKE_C_FLAGS}" )
  foreach(var IN LISTS PACKAGE_INCLUDE_DIRS )
    set( PACKAGE_CFLAGS "${PACKAGE_CFLAGS} -I${var}" )
  endforeach()

  # now construct Libs.private arguments 
  foreach(var IN LISTS PACKAGE_LINK_DIRS )
    set( PACKAGE_LIBS "${PACKAGE_LIBS} -L${var}" )
  endforeach()
  foreach(var IN LISTS PACKAGE_LINK_LIBS )
    if( EXISTS ${var} OR  ${var} MATCHES "-framework*" )
      set( PACKAGE_LIBS "${PACKAGE_LIBS} ${var}" )
    else() # assume it's just a -l call??
      set( PACKAGE_LIBS "${PACKAGE_LIBS} -l${var}" )
    endif()
  endforeach()

  # add any CXX flags user has passed in
  if( PACKAGE_CXXFLAGS )
    set( PACKAGE_CFLAGS ${PACKAGE_CXXFLAGS} )
  endif()

  # In case we want to install. 
  if( NOT EXPORT_${PROJECT_NAME} )
        # add "installed" library to list of required libraries to link against
        if( PACKAGE_LIB_NAME )
            if(POLICY CMP0026)
              cmake_policy( SET CMP0026 OLD )
            endif()
            get_target_property( _target_library ${PACKAGE_LIB_NAME} LOCATION )
            get_filename_component( _lib ${_target_library} NAME )
            list( INSERT PACKAGE_LINK_LIBS 0 ${PACKAGE_LIB_NAME} )
        endif()

        if( PACKAGE_INSTALL_HEADER_DIRS )
            foreach(dir IN LISTS PACKAGE_INSTALL_HEADER_DIRS )
            install( DIRECTORY ${dir}
                DESTINATION ${PACKAGE_DESTINATION}/include 
                FILES_MATCHING PATTERN "*.h|*.hxx|*.hpp"
                )
            endforeach()
        endif()

        # install header files
        if( PACKAGE_INSTALL_HEADERS )
          foreach(hdr IN LISTS PACKAGE_INSTALL_HEADERS )
              get_filename_component( _fp ${hdr} ABSOLUTE )
              if("${SDK_SOURCE_DIR}" STREQUAL "")
                  file( RELATIVE_PATH _rpath ${CMAKE_SOURCE_DIR} ${_fp} )
              else()
                  file( RELATIVE_PATH _rpath ${SDK_SOURCE_DIR} ${_fp} )
              endif()
              get_filename_component( _dir ${_rpath} DIRECTORY )
              install( FILES ${_fp}
                DESTINATION ${PACKAGE_DESTINATION}/${_dir} )
         endforeach()
        endif()
        if( PACKAGE_INSTALL_GENERATED_HEADERS )
          foreach(hdr IN LISTS PACKAGE_INSTALL_GENERATED_HEADERS )
             get_filename_component( _fp ${hdr} ABSOLUTE )
             file( RELATIVE_PATH _rpath ${CMAKE_BINARY_DIR} ${_fp} )
             get_filename_component( _dir ${_rpath} DIRECTORY )
             install( FILES ${_fp}
                 DESTINATION ${PACKAGE_DESTINATION}/${_dir} )
         endforeach()
        endif()

        if( PACKAGE_INSTALL_INCLUDE_DIR )
            if("${SDK_SOURCE_DIR}" STREQUAL "")
                install(DIRECTORY ${CMAKE_SOURCE_DIR}/include DESTINATION ${PACKAGE_DESTINATION} )
            else()
                install(DIRECTORY ${SDK_SOURCE_DIR}/include DESTINATION ${PACKAGE_DESTINATION} )
            endif()
        endif()

        # install library itself
        if( PACKAGE_LIB_NAME )
            install( FILES ${_target_library} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
            set( PACKAGE_LIB_LINK "-l${PACKAGE_LIB_NAME}" )
        endif()

  # In case we want to export.
  elseif( EXPORT_${PROJECT_NAME} )

      if( PACKAGE_LIB_NAME )
            if(POLICY CMP0026)
              cmake_policy( SET CMP0026 OLD )
            endif()
            get_target_property( _target_library ${PACKAGE_LIB_NAME} LOCATION )
            list( INSERT PACKAGE_LINK_LIBS 0 ${_target_library} )
        endif()

        if( PACKAGE_INSTALL_HEADER_DIRS )
            foreach(dir IN LISTS PACKAGE_INSTALL_HEADER_DIRS )
                list( APPEND PACKAGE_INCLUDE_DIRS ${dir} )
            endforeach()
        endif()


        if("${SDK_SOURCE_DIR}" STREQUAL "")
            list( APPEND PACKAGE_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}
                ${CMAKE_BINARY_DIR} )
        else()
            list( APPEND PACKAGE_INCLUDE_DIRS ${SDK_SOURCE_DIR}
                ${CMAKE_BINARY_DIR} )
        endif()

  endif()
    
endfunction()

