cmake_minimum_required(VERSION 2.8.3)

project(Pacer)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)

# setup include directories
include_directories("include")
set(PROJECT_SRC_DIR ./src)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#find BOOST libraries
set(Boost_ADDITIONAL_VERSIONS "1.55.0" "1.56.0")
find_package(Boost COMPONENTS system REQUIRED)
link_directories(${BOOST_LIBRARY_DIRS})
include_directories(SYSTEM ${BOOST_INCLUDE_DIR})

#find OSG
find_package(OSG)
link_directories(${OSG_LIBRARY_DIRS})
include_directories(SYSTEM ${OSG_INCLUDE_DIR})

#add ravelin
find_package(Ravelin REQUIRED)
link_directories(${RAVELIN_LIBRARY_DIRS})
include_directories(SYSTEM ${RAVELIN_INCLUDE_DIRS})

#add moby
find_package(Moby REQUIRED)
link_directories(${MOBY_LIBRARY_DIRS})
include_directories(SYSTEM ${MOBY_INCLUDE_DIRS})

#add cvars
find_package(CVars REQUIRED)
link_directories(${CVARS_LIBRARY_DIRS})
include_directories(SYSTEM ${CVARS_INCLUDE_DIRS})

#add GLConsole
find_package(GLConsole QUIET)
link_directories(${GLConsole_LIBRARY_DIRS})
include_directories(SYSTEM ${GLConsole_INCLUDE_DIRS})

#add GLConsole
find_package(LIBXML2 REQUIRED)
link_directories(${LIBXML2_LIBRARY_DIRS})
include_directories(SYSTEM ${LIBXML2_INCLUDE_DIRS})


option(BUILD_SHARED_LIBS "Build as a shared library?" ON)
# these are the setting for the in-source compiler flow control
option(USE_OSG_DISPLAY "Display Visualizations?" OFF)
#option(SPLITTING_METHOD "Use Splitting Methods to solve QPs?" OFF)
option(LOGGING "Write logs to <working_dir>/out.log" OFF)
option(TIMING "use ctime timing functions" OFF)
option(BUILD_EXAMPLES "Build simulation examples" ON)
  option(USE_DXL "use dynamixel drivers" OFF)
  option(USE_MOBY "build Moby plugin" ON)
  option(USE_GAZEBO "build Gazebo plugin" OFF)
  option(DRIVE_ROBOT "Use drive-robot controller plugin" OFF)
  #option(CONSOLE "Display a Console window for controlling robot CVars =EXPERIMENTAL=" OFF)

# fix the C++ linking error on 64-bit Linux
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

# These are invisible options *experimental*
IF(RANDOM_FRICTION)
  add_definitions( -DRANDOM_FRICTION)
ENDIF(RANDOM_FRICTION)

IF(CONSOLE)
  add_definitions( -DUSE_GLCONSOLE)
ENDIF(CONSOLE)

IF(SPLITTING_METHOD)
  add_definitions( -DSPLITTING_METHOD )
ENDIF(SPLITTING_METHOD)
# end 
add_definitions(-DBUILD_DOUBLE)
add_definitions(-DSAFESTATIC=static)
add_definitions(-DUSE_GLPK)

IF(TIMING)
  add_definitions( -DTIMING)
ENDIF(TIMING)

IF(LOGGING)
  add_definitions( -DLOGGING )
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

set(SOURCES
            output.cpp
            robot.cpp
            kinematics.cpp
            utilities.cpp
            variables.cpp
            inverse_dynamics.cpp
            stabilization.cpp
            splines.cpp
            solve_qp.cpp
            solve_lcp_fast.cpp
            controller.cpp
            locomotion.cpp
            wcpg.cpp
           )

# prepend "src/" to each source file
FOREACH(i ${SOURCES})
  set(LIBSOURCES ${LIBSOURCES} "${CMAKE_SOURCE_DIR}/src/${i}")
ENDFOREACH(i)

# look for DXL
find_library(DXL_FOUND dxl /usr/local/lib /usr/lib)
IF(USE_DXL AND DXL_FOUND)
  set(EXTRA_LIBS ${EXTRA_LIBS} dxl)
  add_definitions (-DUSE_DXL)
ENDIF(USE_DXL AND DXL_FOUND)

add_library(Pacer ${LIBSOURCES})

# cvars
IF(LIBXML2_FOUND)
  target_link_libraries (Pacer ${LIBXML2_LIBRARIES})
ENDIF(LIBXML2_FOUND)

# cvars
IF(CVARS_FOUND)
  target_link_libraries (Pacer ${CVARS_LIBRARIES})
ENDIF(CVARS_FOUND)

# cvars
IF(GLConsole_FOUND)
  target_link_libraries (Pacer ${GLConsole_LIBRARIES})
ENDIF(GLConsole_FOUND)

set(REQLIBS Moby Ravelin)

target_link_libraries(Pacer ${REQLIBS})
set_target_properties(Pacer PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS Pacer DESTINATION lib)

# install include dirs
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/Pacer DESTINATION include)

set(REQLIBS ${REQLIBS} Pacer)

# EXAMPLES
IF(BUILD_EXAMPLES)
# Drive ROBOT
IF(DRIVE_ROBOT)
  #  add_subdirectory( example/drive_robot )
  add_definitions(-DDRIVE_ROBOT)
  add_library(PacerDriveRobotPlugin ${CMAKE_SOURCE_DIR}/example/drive-robot/control.cpp)
  target_link_libraries(PacerDriveRobotPlugin ${REQLIBS})
  set_target_properties(PacerDriveRobotPlugin PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  install(TARGETS PacerDriveRobotPlugin DESTINATION ${CMAKE_SOURCE_DIR}/example/drive-robot)
  install(TARGETS PacerDriveRobotPlugin DESTINATION lib)
  set(REQLIBS ${REQLIBS} PacerDriveRobotPlugin)
ENDIF(DRIVE_ROBOT)

# Moby Plugin
IF(USE_MOBY)
  add_library(PacerMobyPlugin MODULE ${CMAKE_SOURCE_DIR}/example/control-moby/plugin.cpp)
# osg
IF(USE_OSG_DISPLAY AND OSG_FOUND)
    target_link_libraries (PacerMobyPlugin ${OSG_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OSGVIEWER_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OSGDB_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OSGGA_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OPENTHREADS_LIBRARY})
    target_link_libraries (PacerMobyPlugin ${OSGUTIL_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OSGMANIPULATOR_LIBRARIES})
    add_definitions (-DUSE_OSG_DISPLAY)
  ENDIF(USE_OSG_DISPLAY AND OSG_FOUND)
  target_link_libraries(PacerMobyPlugin ${REQLIBS} ${EXTRA_LIBS})
  set_target_properties(PacerMobyPlugin PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  install(TARGETS PacerMobyPlugin DESTINATION ${CMAKE_SOURCE_DIR}/example/control-moby)
ENDIF(USE_MOBY)

# Gazebo Plugin
IF(USE_GAZEBO)
  add_library(PacerGazeboPlugin SHARED ${CMAKE_SOURCE_DIR}/example/control-gazebo/plugin.cpp)
# NO USE FOR OSG YET
# IF(USE_OSG_DISPLAY AND OSG_FOUND)
#   target_link_libraries (PacerGazeboPlugin ${OSG_LIBRARIES})
#   target_link_libraries (PacerGazeboPlugin ${OSGVIEWER_LIBRARIES})
#   target_link_libraries (PacerGazeboPlugin ${OSGDB_LIBRARIES})
#   target_link_libraries (PacerGazeboPlugin ${OSGGA_LIBRARIES})
#   target_link_libraries (PacerGazeboPlugin ${OPENTHREADS_LIBRARY})
#   target_link_libraries (PacerGazeboPlugin ${OSGUTIL_LIBRARIES})
#   target_link_libraries (PacerGazeboPlugin ${OSGMANIPULATOR_LIBRARIES})
#   add_definitions (-DUSE_OSG_DISPLAY)
# ENDIF(USE_OSG_DISPLAY AND OSG_FOUND)

  include (FindPkgConfig)
  IF(PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
  ENDIF(PKG_CONFIG_FOUND)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  link_directories(${GAZEBO_LIBRARY_DIRS})

  target_link_libraries(PacerGazeboPlugin ${REQLIBS} ${GAZEBO_LIBRARIES} ${BOOST_LIBRARIES} ${Boost_SYSTEM_LIBRARY} protobuf ${EXTRA_LIBS})
  install(TARGETS PacerGazeboPlugin DESTINATION ${CMAKE_SOURCE_DIR}/example/control-gazebo)
ENDIF(USE_GAZEBO)

#driver with no simulator
enable_testing()
add_executable(PacerDriver ${CMAKE_SOURCE_DIR}/example/driver.cpp)
target_link_libraries(PacerDriver ${REQLIBS} ${EXTRA_LIBS})
set_target_properties(PacerDriver PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
add_test(NAME test_pacer COMMAND PacerDriver ${CMAKE_SOURCE_DIR}/example/models/links-moby/model.xml ${CMAKE_SOURCE_DIR}/example/models/links-moby/vars.xml 1.0)

#IF(USE_MOBY)
#  add_test(moby_test  [<arg>...])
#ENDIF(USE_MOBY)
#IF(USE_GAZEBO)
#  add_test(gazebo_test gzserver ${CMAKE_SOURCE_DIR}/example/models/links/model.world)
#ENDIF(USE_GAZEBO)
ENDIF(BUILD_EXAMPLES)


