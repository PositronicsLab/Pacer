cmake_minimum_required(VERSION 2.8.3)

project(Pacer)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)

# force Pacer to build with C++0x
SET(CMAKE_C_FLAGS_DEBUG "-g")
SET(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

###############################################################################
###############################  SETUP LIBS ###################################
###############################################################################

set (LIBS "")

###############################  REQUIRED  ####################################

# find BOOST libraries
set(Boost_ADDITIONAL_VERSIONS "1.55.0" "1.56.0")
find_package(Boost REQUIRED)
link_directories(${BOOST_LIBRARY_DIRS})
include_directories(${BOOST_INCLUDE_DIR})
set (LIBS ${LIBS} ${BOOST_LIBRARIES} boost_system)

# add Ravelin
find_package(Ravelin REQUIRED)
link_directories(${RAVELIN_LIBRARY_DIR})
include_directories(${RAVELIN_INCLUDE_DIR})
set (LIBS ${LIBS} ${RAVELIN_LIBRARIES})

# add Moby
find_package(Moby REQUIRED)
IF (NOT MOBY_LIBRARY_DIR)
  get_filename_component(MOBY_LIBRARY_DIR ${MOBY_LIBRARY} PATH)
ENDIF ()
link_directories(${MOBY_LIBRARY_DIR})
include_directories(${MOBY_INCLUDE_DIR})
set (LIBS ${LIBS} ${MOBY_LIBRARIES})

# add LibXml2
find_package(LibXml2 REQUIRED)
set (LIBS ${LIBS} ${LIBXML2_LIBRARIES})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LIBXML2_DEFINITIONS}")

# lib DL
find_package( DL REQUIRED )
set (LIBS ${LIBS} ${DL_LIBRARIES})

###############################  OPTIONAL  ####################################

option(USE_DISPLAY "Display Visualizations?" OFF)
IF(USE_DISPLAY)
  add_definitions( -DUSE_DISPLAY )
ENDIF(USE_DISPLAY)

option(USE_REALTIME "Use Realtime libraries?" OFF)
IF(USE_REALTIME)
  add_definitions( -DUSE_REALTIME )
  find_package( RT REQUIRED )
  set (LIBS ${LIBS} ${RT_LIBRARIES})
ENDIF(USE_REALTIME)

option(USE_THREADS "Build Pacer with multi-threading support?" OFF)
IF(USE_THREADS)
  add_definitions(-DUSE_THREADS)
  find_package( Pthread REQUIRED )
  set (LIBS ${LIBS} ${PTHREAD_LIBRARIES})
  #  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
ENDIF(USE_THREADS)

option(USE_GSL "Build Pacer with randomization support?" OFF)
IF(USE_GSL)
find_package (GSL REQUIRED)
IF(GSL_FOUND)
  add_definitions(-DUSE_GSL)
  include_directories(${GSL_INCLUDE_DIRS})
  link_directories(${GSL_LIBRARY_DIRS})
  set(LIBS ${LIBS} ${GSL_LIBRARIES})
  # handle linking error for GSL
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GSL_LIBRARY_DIRS}/libgsl.dylib")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_GSL_CXX_FLAGS} ${GSL_EXE_LINKER_FLAGS}")
  ENDIF(GSL_FOUND)
ENDIF(USE_GSL)

###############################################################################
############################ EXTRA BUILD OPTIONS ##############################
###############################################################################

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)

# used in LCP solver
option(USE_SPLITTING_METHOD "Force pacer to use a Splitting Method when running an optimization" OFF)
IF(USE_SPLITTING_METHOD)
  add_definitions( -DUSE_SPLITTING_METHOD )
ENDIF(USE_SPLITTING_METHOD)

option(LOGGING "Allow logging and Write logs to /your/working/directory/out-##PID##.log" OFF)
IF(LOGGING)
  add_definitions( -DLOG_TO_FILE )
ENDIF(LOGGING)


# setup include directories
include_directories( src/include
                    /usr/local/include )

file(GLOB SOURCES
  "src/*.cpp"
)

set(DRIVER_SOURCES
  src/main/driver.cpp
)

set(MAIN_SOURCES
  src/main/main.cpp
)
add_library(PacerDriver ${SOURCES}  ${DRIVER_SOURCES})
target_link_libraries(PacerDriver ${LIBS})

add_executable(pacer-main ${MAIN_SOURCES})
target_link_libraries(pacer-main PacerDriver)
# install Pacer binary
set_target_properties(PacerDriver PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
#install(TARGETS pacer-driver DESTINATION bin)

add_library(Pacer ${SOURCES})
target_link_libraries(Pacer ${LIBS})
# install Pacer library
set_target_properties(Pacer PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
#install(TARGETS Pacer DESTINATION lib)

# install include dirs
#install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/include/Pacer DESTINATION include)

# Setup configuration file
configure_file(${CMAKE_SOURCE_DIR}/CMakeModules/setup.sh.in ${CMAKE_BINARY_DIR}/setup.sh @ONLY)

# install setup file
#install (FILES ${CMAKE_BINARY_DIR}/setup.sh DESTINATION share/Pacer)

set(REQLIBS ${LIBS} Pacer)

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
#add_custom_target(uninstall
#    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
