cmake_minimum_required(VERSION 2.8.3)

project(Pacer)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# setup include directories
include_directories("include")
set(PROJECT_SRC_DIR ./src)

#find BOOST libraries
set(Boost_ADDITIONAL_VERSIONS "1.55.0" "1.56.0")
find_package(Boost REQUIRED)
#link_directories(${BOOST_LIBRARY_DIRS})
include_directories(${BOOST_INCLUDE_DIR})

#add ravelin
find_package(Ravelin REQUIRED)
link_directories(${RAVELIN_LIBRARY_DIR})
include_directories(${RAVELIN_INCLUDE_DIR})

#add moby
find_package(Moby REQUIRED)
if (NOT MOBY_LIBRARY_DIR)
  get_filename_component(MOBY_LIBRARY_DIR ${MOBY_LIBRARY} PATH)
endif()
link_directories(${MOBY_LIBRARY_DIR})
include_directories(${MOBY_INCLUDE_DIR})

#add GLConsole
find_package(LibXml2 REQUIRED)
link_directories(${LIBXML2_LIBRARY_DIR})
include_directories(${LIBXML2_INCLUDE_DIR})

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)

# these are the setting for the in-source compiler flow control
option(LOGGING "Write logs to <working_dir>/out.log" ON)
option(BUILD_EXAMPLES "Build simulation examples" OFF)
option(BUILD_TESTS "Build Pacer Unit tests and Regression Tests for debugging?" OFF)
option(USE_THREADS "Build Pacer with multi-threading support?" OFF)

# fix the C++ linking error on 64-bit Linux
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

# These are invisible options *experimental*
IF(RANDOM_FRICTION)
  add_definitions( -DRANDOM_FRICTION)
ENDIF(RANDOM_FRICTION)

IF(BUILD_PLUGINS)
  add_definitions( -DUSE_PLUGINS)
ENDIF(BUILD_PLUGINS)

IF(CONSOLE)
  add_definitions( -DUSE_GLCONSOLE)
ENDIF(CONSOLE)

IF(SPLITTING_METHOD)
  add_definitions( -DSPLITTING_METHOD )
ENDIF(SPLITTING_METHOD)
# end 
IF(USE_THREADS)
  add_definitions(-DUSE_THREADS)
  set (REQLIBS ${REQLIBS} pthread)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
ENDIF(USE_THREADS)

IF(LOGGING)
  add_definitions( -DLOG_TO_FILE )
ENDIF(LOGGING)

include_directories(
                    include
                    /usr/include
                    /usr/local/include
                    )

link_directories(
                  /usr/lib
                  /usr/local/lib
                  )

file(GLOB LIBSOURCES
"src/*.cpp"
)

add_library(Pacer ${LIBSOURCES})

# LibXml2
IF(LIBXML2_FOUND)
  set (REQLIBS ${REQLIBS} ${LIBXML2_LIBRARIES})
ENDIF(LIBXML2_FOUND)

set(REQLIBS ${MOBY_LIBRARIES} ${RAVELIN_LIBRARIES})

target_link_libraries(Pacer ${REQLIBS})
set_target_properties(Pacer PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS Pacer DESTINATION lib)

# install include dirs
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/Pacer DESTINATION include)

# Setup configuration file
configure_file(${CMAKE_SOURCE_DIR}/CMakeModules/setup.sh.in ${CMAKE_BINARY_DIR}/setup.sh @ONLY)

install (FILES ${CMAKE_BINARY_DIR}/setup.sh DESTINATION share/Pacer)

# install plugins and models
install (DIRECTORY ${CMAKE_SOURCE_DIR}/example/models DESTINATION ${CMAKE_INSTALL_PREFIX}/share/Pacer)
if(BUILD_PLUGINS)
install (DIRECTORY ${CMAKE_BINARY_DIR}/example/plugins DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/Pacer)
endif(BUILD_PLUGINS)
if(BUILD_INTERFACES)
install (DIRECTORY ${CMAKE_BINARY_DIR}/example/interfaces DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/Pacer)
endif(BUILD_INTERFACES)

set(REQLIBS ${REQLIBS} Pacer)

# EXAMPLES
IF(BUILD_EXAMPLES)
  add_subdirectory( example )
ENDIF(BUILD_EXAMPLES)

IF(BUILD_TESTS)
  enable_testing()
  add_custom_target(unit-test ${CMAKE_CTEST_COMMAND} -V)
  add_custom_target(regression-test ${CMAKE_CTEST_COMMAND} -V)
  add_subdirectory( test )
ENDIF(BUILD_TESTS)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
