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
            src/visualize_moby.cpp
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

if(USE_MOBY)
  if(VISUALIZE_MOBY)
    set(SOURCES ${SOURCES} src/visualize_moby.cpp)
    set(VISUALIZE_LIBS glut osg osgDB GLU ${GLUT_LIBRARIES} ${GLU_LIBRARIES} ${OPENGL_LIBRARIES})
    FIND_PACKAGE( GLUT REQUIRED )
    FIND_PACKAGE( OpenGL REQUIRED )
    add_definitions( -DVISUALIZE_MOBY )
  endif(VISUALIZE_MOBY)
endif(USE_MOBY)

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} Moby Ravelin Opt OpenThreads cvars ${VISUALIZE_LIBS})


add_library(${PROJECT_NAME}Model ${SOURCES-BASIC})
target_link_libraries(${PROJECT_NAME}Model Moby Ravelin Opt OpenThreads cvars ${VISUALIZE_LIBS})
install(TARGETS ${PROJECT_NAME}Model DESTINATION lib)

# EXAMPLES

# Moby Plugin
if(USE_MOBY)
  add_library(${PROJECT_NAME}MobyPlugin MODULE ${CMAKE_SOURCE_DIR}/example/control-moby/pacer-plugin.cpp)
  target_link_libraries(${PROJECT_NAME}MobyPlugin ${PROJECT_NAME} Moby Ravelin)
  install(TARGETS ${PROJECT_NAME}MobyPlugin DESTINATION ${CMAKE_SOURCE_DIR}/example/control-moby)
  install(TARGETS ${PROJECT_NAME}MobyPlugin DESTINATION lib)
endif(USE_MOBY)

# Gazebo Plugin
if(USE_GAZEBO)
  set(VISUALIZE_MOBY off)

  include (FindPkgConfig)
  if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
  endif(PKG_CONFIG_FOUND)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  link_directories(${GAZEBO_LIBRARY_DIRS})

  add_library(${PROJECT_NAME}GazeboPlugin SHARED ${CMAKE_SOURCE_DIR}/example/control-gazebo/pacer-plugin.cpp)
  target_link_libraries(${PROJECT_NAME}GazeboPlugin ${PROJECT_NAME} Moby Ravelin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
  install(TARGETS ${PROJECT_NAME}GazeboPlugin DESTINATION lib)

  add_library(contact SHARED ${CMAKE_SOURCE_DIR}/example/control-gazebo/contact-plugin.cpp)
  target_link_libraries(contact ${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} protobuf)
  install(TARGETS contact DESTINATION ${CMAKE_SOURCE_DIR}/example/control-gazebo)
endif(USE_GAZEBO)

# Test: Calling PacerModel only instead of using controller features
  add_executable(test-pacer-model ${CMAKE_SOURCE_DIR}/example/pacer-model/test.cpp)
  target_link_libraries(test-pacer-model ${PROJECT_NAME}Model Ravelin)

  install(TARGETS ${PROJECT_NAME}Model DESTINATION lib)
  install(TARGETS ${PROJECT_NAME} DESTINATION lib)

  install(TARGETS test-pacer-model DESTINATION ${CMAKE_SOURCE_DIR}/example/pacer-model)

