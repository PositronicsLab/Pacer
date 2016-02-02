cmake_minimum_required(VERSION 2.8.3)

project(Pacer)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# setup include directories
include_directories( src/include
                    /usr/local/include )
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
option(USE_THREADS "Build Pacer with multi-threading support?" OFF)
option(SPLITTING_METHOD "Force pacer to use a Splitting Method when running an optimization" OFF)
option(USE_GSL "Build Pacer with randomization support?" OFF)

# fix the C++ linking error on 64-bit Linux
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

# used in LCP solver
IF(SPLITTING_METHOD)
  add_definitions( -DSPLITTING_METHOD )
ENDIF(SPLITTING_METHOD)
# end 
IF(USE_THREADS)
  add_definitions(-DUSE_THREADS)
  set (REQLIBS ${REQLIBS} pthread)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
ENDIF(USE_THREADS)

IF(USE_GSL)
find_package (GSL REQUIRED)
IF(GSL_FOUND)
  add_definitions(-DUSE_GSL)
  include_directories(${GSL_INCLUDE_DIRS})
  link_directories(${GSL_LIBRARY_DIRS})
  set(REQLIBS ${REQLIBS} ${GSL_LIBRARIES})
  # handle linking error for GSL
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GSL_LIBRARY_DIRS}/libgsl.dylib")
ENDIF(GSL_FOUND)
ENDIF(USE_GSL)

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


# LibXml2
IF(LIBXML2_FOUND)
  set (REQLIBS ${REQLIBS} ${LIBXML2_LIBRARIES})
ENDIF(LIBXML2_FOUND)

set(REQLIBS ${MOBY_LIBRARIES} ${RAVELIN_LIBRARIES})

add_executable(pacer-driver ${LIBSOURCES})
target_link_libraries(pacer-driver ${REQLIBS})
set_target_properties(pacer-driver PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS pacer-driver DESTINATION bin)

add_library(Pacer ${LIBSOURCES})
target_link_libraries(Pacer ${REQLIBS})
set_target_properties(Pacer PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS Pacer DESTINATION lib)

# install include dirs
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/include/Pacer DESTINATION include)

# Setup configuration file
configure_file(${CMAKE_SOURCE_DIR}/CMakeModules/setup.sh.in ${CMAKE_BINARY_DIR}/setup.sh @ONLY)

install (FILES ${CMAKE_BINARY_DIR}/setup.sh DESTINATION share/Pacer)

set(REQLIBS ${REQLIBS} Pacer)

# EXAMPLES
option(BUILD_PLUGINS "Build Simulator and Pacer Component Plugins?" ON)
IF(BUILD_PLUGINS)
  add_definitions( -DUSE_PLUGINS)
  add_subdirectory( Plugin )
ENDIF(BUILD_PLUGINS)

option(BUILD_TESTS "Build Pacer Unit tests and Regression Tests for debugging?" OFF)
IF(BUILD_TESTS)
  enable_testing()
  add_custom_target(regression-test ${CMAKE_CTEST_COMMAND} -V)
  add_subdirectory( Test )
ENDIF(BUILD_TESTS)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
