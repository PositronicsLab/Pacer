cmake_minimum_required(VERSION 2.8.3)

project(Pacer)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)
set (PROJECT_SRC_DIR .)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)

# these are the setting for the in-source compiler flow control
option(VISUALIZE_MOBY "Display Moby Visualizations?" OFF)
option(SPLITTING_METHOD "Use Splitting Methods to solve QPs?" OFF)
option(LOGGING "Logging" ON)
option(CONSOLE "GLConsole" OFF)
option(TIMING "use ctime timing functions" OFF)
option(USE_MOBY "build Moby plugin" OFF)
option(USE_GAZEBO "build Gazebo plugin" OFF)
option(RANDOM_FRICTION "Randomize friction mu = {0.1 .. 1.4}" OFF)
option(DRIVE_ROBOT "Use drive-robot controller plugin" OFF)

# fix the C++ linking error on 64-bit Linux
set (CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

if(RANDOM_FRICTION)
  add_definitions( -DRANDOM_FRICTION)
endif(RANDOM_FRICTION)


if(CONSOLE)
  add_definitions( -DUSE_GLCONSOLE)
endif(CONSOLE)

# you may have to add additional includes in here...
add_definitions(-DBUILD_DOUBLE)
add_definitions(-DUSE_OSG)
add_definitions(-DSAFESTATIC=static)
add_definitions(-DUSE_GLPK)

if(TIMING)
  add_definitions( -DTIMING)
endif(TIMING)

if(LOGGING)
  add_definitions( -DLOGGING )
endif(LOGGING)

if(SPLITTING_METHOD)
  add_definitions( -DSPLITTING_METHOD )
endif(SPLITTING_METHOD)

include_directories(
                    include
                    /usr/include
                    /usr/include/libxml2
                    /usr/local/include
                    ${Boost_INCLUDE_DIR}
                    /opt/local/include
                    /opt/X11/include
                    ../../ThirdParty/alglib/src
                    /Applications/MATLAB_R2013a.app/extern/include
                    )

link_directories(
                  /usr/local/lib
                  /opt/X11/lib
/Applications/MATLAB_R2013a.app/extern/lib/maci64
                  )

set(SOURCES-BASIC
            src/output.cpp
            src/robot.cpp
            src/kinematics.cpp
            src/utilities.cpp
            src/variables.cpp
           )

set(SOURCES ${SOURCES-BASIC}
            src/inverse_dynamics.cpp
            src/stabilization.cpp
            src/splines.cpp
            src/solve_qp.cpp
            src/solve_lcp_fast.cpp
            src/controller.cpp
            src/locomotion.cpp
            src/wcpg.cpp
           )

# install include dirs
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/Pacer DESTINATION include)

set(REQURED_LIBS Moby Ravelin)

add_library(Pacer ${SOURCES})
target_link_libraries(Pacer ${REQURED_LIBS} Opt OpenThreads cvars)

#add_library(PacerModel ${SOURCES-BASIC})
#target_link_libraries(PacerModel Moby Ravelin Opt OpenThreads cvars ${VISUALIZE_LIBS})
#install(TARGETS PacerModel DESTINATION lib)

set(REQURED_LIBS ${REQURED_LIBS} Pacer)
# EXAMPLES

# Drive ROBOT
if(DRIVE_ROBOT)
  add_definitions(-DDRIVE_ROBOT)
  add_library(PacerDriveRobotPlugin ${CMAKE_SOURCE_DIR}/example/drive-robot/control.cpp)
  target_link_libraries(PacerDriveRobotPlugin ${REQURED_LIBS})
  install(TARGETS PacerDriveRobotPlugin DESTINATION ${CMAKE_SOURCE_DIR}/example/drive-robot)
  install(TARGETS PacerDriveRobotPlugin DESTINATION lib)
  set(REQURED_LIBS ${REQURED_LIBS} PacerDriveRobotPlugin)
endif(DRIVE_ROBOT)

# Moby Plugin
if(USE_MOBY)
  if(VISUALIZE_MOBY)
    set(VISUALIZE_SRCS ${CMAKE_SOURCE_DIR}/example/control-moby/visualize.cpp)
    set(VISUALIZE_LIBS glut osg osgDB GLU ${GLUT_LIBRARIES} ${GLU_LIBRARIES} ${OPENGL_LIBRARIES})
    FIND_PACKAGE( GLUT REQUIRED )
    FIND_PACKAGE( OpenGL REQUIRED )
    add_definitions( -DVISUALIZE_MOBY )
  endif(VISUALIZE_MOBY)

  add_library(PacerMobyPlugin MODULE ${CMAKE_SOURCE_DIR}/example/control-moby/pacer-plugin.cpp ${VISUALIZE_SRCS})
  target_link_libraries(PacerMobyPlugin ${REQURED_LIBS} ${VISUALIZE_LIBS})
  install(TARGETS PacerMobyPlugin DESTINATION ${CMAKE_SOURCE_DIR}/example/control-moby)
  install(TARGETS PacerMobyPlugin DESTINATION lib)
endif(USE_MOBY)

# Gazebo Plugin
if(USE_GAZEBO)
#  if(VISUALIZE_GAZEBO)
#    set(VISUALIZE_SRCS ${CMAKE_SOURCE_DIR}/example/control-gazebo/visualize.cpp)
#    set(VISUALIZE_LIBS glut osg osgDB GLU ${GLUT_LIBRARIES} ${GLU_LIBRARIES} ${OPENGL_LIBRARIES})
#    FIND_PACKAGE( GLUT REQUIRED )
#    FIND_PACKAGE( OpenGL REQUIRED )
#    add_definitions( -DVISUALIZE_GAZEBO )
#  endif(VISUALIZE_GAZEBO)

  include (FindPkgConfig)
  if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
  endif(PKG_CONFIG_FOUND)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  link_directories(${GAZEBO_LIBRARY_DIRS})

  add_library(PacerGazeboPlugin SHARED ${CMAKE_SOURCE_DIR}/example/control-gazebo/pacer-plugin.cpp)
  target_link_libraries(PacerGazeboPlugin ${REQURED_LIBS} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
  install(TARGETS PacerGazeboPlugin DESTINATION lib)

  add_library(contact SHARED ${CMAKE_SOURCE_DIR}/example/control-gazebo/contact-plugin.cpp)
  target_link_libraries(contact Pacer ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
  install(TARGETS contact DESTINATION ${CMAKE_SOURCE_DIR}/example/control-gazebo)
endif(USE_GAZEBO)

# Test: Calling PacerModel only instead of using controller features
#  add_executable(test-pacer-model ${CMAKE_SOURCE_DIR}/example/pacer-model/test.cpp)
#  target_link_libraries(test-pacer-model PacerModel Ravelin)
#  install(TARGETS test-pacer-model DESTINATION ${CMAKE_SOURCE_DIR}/example/pacer-model)

  #  install(TARGETS PacerModel DESTINATION lib)
  # install(TARGETS Pacer DESTINATION lib)


