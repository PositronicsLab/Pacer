cmake_minimum_required(VERSION 2.8.3)

project(Links)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)
set (PROJECT_SRC_DIR .)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
find_package(Boost REQUIRED)
FIND_PACKAGE( GLUT REQUIRED )
#FIND_PACKAGE( GLU REQUIRED )
FIND_PACKAGE( OpenGL REQUIRED )

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)

# these are the setting for the in-source compiler flow control
option(VISUALIZE_MOBY "Display Moby Visualizations?" OFF)
option(SPLITTING_METHOD "Use Splitting Methods to solve QPs?" OFF)
option(LOGGING "Logging" ON)
option(CONSOLE "GLConsole" OFF)

# you may have to add additional includes in here...
add_definitions(-DBUILD_DOUBLE)
add_definitions(-DUSE_OSG)
add_definitions(-DSAFESTATIC=static)
add_definitions(-DUSE_GLPK)

if(CONSOLE)
  add_definitions( -DUSE_GLCONSOL)
endif(CONSOLE)

if(LOGGING)
  add_definitions( -DLOGGING )
endif(LOGGING)

if(SPLITTING_METHOD)
  add_definitions( -DSPLITTING_METHOD )
endif(SPLITTING_METHOD)

if(VISUALIZE_MOBY)
  add_definitions( -DVISUALIZE_MOBY )
endif(VISUALIZE_MOBY)

include_directories(
                    .
                    ../../include
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

set(SOURCES ../../src/output.cpp
            ../../src/robot.cpp
            #../../src/estimate_friction.cpp
            ../../src/inverse_dynamics.cpp
            ../../src/kinematics.cpp
            ../../src/stabilization.cpp
            ../../src/utilities.cpp
            ../../src/splines.cpp
            ../../src/solve_qp.cpp
            ../../src/Solvers.cpp
           )

if(VISUALIZE_MOBY)
  set(SOURCES ${SOURCES} ../../src/visualize_moby.cpp)
  set(VISUALIZE_LIBS  glut osg osgDB GLU ${GLUT_LIBRARIES} ${GLU_LIBRARIES} ${OPENGL_LIBRARIES})
endif(VISUALIZE_MOBY)

set(SOURCES2 quadruped.cpp
             inverse_kinematics.cpp
             locomotion.cpp
             wcpg.cpp
             stabilization.cpp
             variables.cpp
            )

set(THIRDPARTY
              )

add_library(${PROJECT_NAME} ${SOURCES} ${SOURCES2} ${THIRDPARTY})
target_link_libraries(${PROJECT_NAME} Moby Ravelin Opt OpenThreads cvars ${VISUALIZE_LIBS})

add_library(${PROJECT_NAME}Plugin MODULE control_moby.cpp)
target_link_libraries(${PROJECT_NAME}Plugin ${PROJECT_NAME} Moby Ravelin)
