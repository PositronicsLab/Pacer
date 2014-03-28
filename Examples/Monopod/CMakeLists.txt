cmake_minimum_required(VERSION 2.8.3)

project(Hopper)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)
set (PROJECT_SRC_DIR .)

find_package(Boost REQUIRED)

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)

# these are the setting for the in-source compiler flow control
option(SET_KINEMATICS "Control Robot Kinematics?" OFF)
option(CONTACT_CALLBACK "Use contact parameter callback?" OFF)
option(VISUALIZE_MOBY "Display Moby Visualizations?" OFF)
option(COLLECT_DATA "Output Matlab Data?" OFF)
option(USE_TERRAIN "Add Terrain to the world?" OFF)
option(SPLITTING_METHOD "Use Splitting Methods to solve QPs?" OFF)

# you may have to add additional includes in here...
add_definitions(-DBUILD_DOUBLE)
add_definitions(-DUSE_OSG)
add_definitions(-DSAFESTATIC=static)
add_definitions(-DUSE_GLPK)

if(USE_TERRAIN)
add_definitions( -DUSE_TERRAIN )
endif(USE_TERRAIN)

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

include_directories(
                    .
                    ../../include
                    /usr/include
                    /usr/include/libxml2
                    /usr/local/include
                    ${Boost_INCLUDE_DIR}
                    /opt/local/include
                    /opt/X11/include
                    )

link_directories(
                  /usr/local/lib
                  /opt/local/lib
                  /opt/X11/lib
                  )

set(SOURCES ../../src/output.cc
            ../../src/robot.cc
            ../../src/estimate_friction.cc
            ../../src/inverse_dynamics.cc
            ../../src/kinematics.cc
            ../../src/stabilization.cc
            ../../src/utilities.cc
            ../../src/solve_qp.cc
            ../../src/Solvers.cpp
            ../../src/moby_callback.cc
           )

if(VISUALIZE_MOBY)
  set(SOURCES ${SOURCES} ../../src/visualize_moby.cc)
endif(VISUALIZE_MOBY)

if(USE_TERRAIN)
  set(SOURCES ${SOURCES} ../../src/terrain.cc)
endif(USE_TERRAIN)

set(SOURCES2 control_moby.cc
             monopod.cc
             inverse_kinematics.cc
             locomotion.cc
            )



          add_library(${PROJECT_NAME} MODULE ${SOURCES} ${SOURCES2})
target_link_libraries(${PROJECT_NAME} Moby Ravelin glut GLU osg osgDB OpenThreads)

