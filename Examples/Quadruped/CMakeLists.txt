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
option(DYNAMIXEL "Use Dynamixels?" OFF)
option(SET_KINEMATICS "Control Robot Kinematics?" OFF)
option(CONTACT_CALLBACK "Use contact parameter callback?" OFF)
option(VISUALIZE_MOBY "Display Moby Visualizations?" OFF)
option(COLLECT_DATA "Output Matlab Data?" OFF)
option(USE_TERRAIN "Add Terrain to the world?" OFF)
option(SPLITTING_METHOD "Use Splitting Methods to solve QPs?" OFF)
option(PERTURB_CONTACT_DATA "Perturb Contact Data to simulate inaccurate sensors?" OFF)

# you may have to add additional includes in here...
add_definitions(-DBUILD_DOUBLE)
add_definitions(-DUSE_OSG)
add_definitions(-DSAFESTATIC=static)
add_definitions(-DUSE_GLPK)

if(USE_TERRAIN)
add_definitions( -DUSE_TERRAIN )
endif(USE_TERRAIN)

if(PERTURB_CONTACT_DATA)
  add_definitions( -DPERTURB_CONTACT_DATA )
endif(PERTURB_CONTACT_DATA)

if(SPLITTING_METHOD)
add_definitions( -DSPLITTING_METHOD )
endif(SPLITTING_METHOD)

if(SET_KINEMATICS)
add_definitions( -DSET_KINEMATICS )
endif(SET_KINEMATICS)

if(VISUALIZE_MOBY)
add_definitions( -DVISUALIZE_MOBY )
endif(VISUALIZE_MOBY)

if(COLLECT_DATA)
add_definitions( -DCOLLECT_DATA )
endif(COLLECT_DATA)

if(CONTACT_CALLBACK)
  add_definitions( -DCONTACT_CALLBACK )
endif(CONTACT_CALLBACK)

if(DYNAMIXEL)
  add_definitions( -DUSE_ROBOT )
endif(DYNAMIXEL)

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
                    )

link_directories(
                  /usr/local/lib
                  /opt/X11/lib
                  )

set(SOURCES ../../src/output.cc
            ../../src/robot.cc
            ../../src/estimate_friction.cc
            ../../src/inverse_dynamics.cc
            ../../src/kinematics.cc
            ../../src/stabilization.cc
            ../../src/utilities.cc
            ../../src/splines.cc
            ../../src/solve_qp.cc
            ../../src/Solvers.cpp
            ../../src/moby_callback.cc
           )

if(VISUALIZE_MOBY)
  set(SOURCES ${SOURCES} ../../src/visualize_moby.cc)
  set(VISUALIZE_LIBS  glut osg osgDB GLU ${GLUT_LIBRARIES} ${GLU_LIBRARIES} ${OPENGL_LIBRARIES})
endif(VISUALIZE_MOBY)

if(USE_TERRAIN)
  set(SOURCES ${SOURCES} ../../src/terrain.cc)
endif(USE_TERRAIN)

set(SOURCES2 control_moby.cc
             quadruped.cc
             inverse_kinematics.cc
             locomotion.cc
             wcpg.cc
             stabilization.cc
             variables.cc
            )

set(THIRDPARTY
                )

add_library(${PROJECT_NAME} MODULE ${SOURCES} ${SOURCES2} ${THIRDPARTY})

target_link_libraries(${PROJECT_NAME} Moby Ravelin OpenThreads cvars ${VISUALIZE_LIBS})

if(DYNAMIXEL)
  target_link_libraries(${PROJECT_NAME} dxl)
endif(DYNAMIXEL)

