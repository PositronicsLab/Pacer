cmake_minimum_required(VERSION 2.8.3)

project(Links)

find_package(Boost REQUIRED)

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)

# you may have to add additional includes in here...
add_definitions(-DBUILD_DOUBLE)
add_definitions(-DUSE_OSG)
add_definitions(-DSAFESTATIC=static)
add_definitions(-DUSE_GLPK)

set(BUILD_DIRECTORY bin)
set(SOURCE_DIRECTORY src)

include_directories(
#/Users/samzapo/ThirdParty/Moby/include
include /usr/include /usr/include/libxml2 /usr/local/include ${Boost_INCLUDE_DIR} /opt/local/include /opt/X11/include)

link_directories(
#/Users/samzapo/ThirdParty/Moby/contact-reduce
/usr/local/lib /opt/local/lib /opt/X11/lib
)

#set(box_srcs estimate_friction.cc object_control_moby.cc output.cc)
#set(hyq_srcs estimate_friction.cc hyq_control_moby.cc output.cc)
set(links_srcs  src/control_moby.cc
                src/output.cc
                src/robot.cc
                src/quadruped.cc
                src/estimate_friction.cc
                src/inverse_dynamics.cc
                src/visualize_moby.cc
                src/bezier_solver.cc
                src/inverse_kinematics.cc
                src/kinematics.cc
                src/stabilization.cc
                src/locomotion.cc
                src/solve_qp.cc
                src/Solvers.cpp
                src/iit_cpg.cc
    )
set(coldet_srcs src/LinksColdetPlugin.cpp)

#add_library(OBJ MODULE ${box_srcs})
#add_library(HYQ MODULE ${hyq_srcs})
add_library(LINKS MODULE ${links_srcs})
add_library(LinksColdetPlugin MODULE ${coldet_srcs})

#target_link_libraries(BOX Moby Ravelin Opt)
#target_link_libraries(HYQ Moby Ravelin Opt)
target_link_libraries(LINKS Moby Ravelin  glut GLU osg osgDB OpenThreads dxl)
target_link_libraries(LinksColdetPlugin Moby Ravelin  glut GLU)
