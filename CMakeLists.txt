cmake_minimum_required(VERSION 2.8.3)

project(MobyController)

find_package(Boost REQUIRED)

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)
# you may have to add additional includes in here...
add_definitions(-DBUILD_DOUBLE)
add_definitions(-DUSE_OSG)
add_definitions(-DSAFESTATIC=static)
add_definitions(-DUSE_GLPK)

include_directories(.)
include_directories( /usr/include /usr/include/libxml2 /usr/local/include include ${Boost_INCLUDE_DIR} )

link_directories(/usr/local/lib )

# g++ -g0 -O3 -I/usr/include/libxml2 -I../../include -DBUILD_DOUBLE -DSAFESTATIC=static -DUSE_OSG -DUSE_GLPK PD-controller.cpp estimatefrict.cpp output.cpp -L../../../debug -lMoby -shared -o libcontroller.so
#set(box_srcs estimatefrict.cpp Box.cpp output.cpp)
#set(hyq_srcs estimatefrict.cpp PD-controller.cpp output.cpp)
#set(hyq_srcs simple-PD.cpp output.cpp)
set(links_srcs Controller.cpp output.cpp estimatefrict.cpp idyn.cpp Visualization.cpp StepBezier.cpp)

#add_library(BOX${PROJECT_NAME} ${box_srcs})
#add_library(HYQ${PROJECT_NAME} ${hyq_srcs})
add_library(LINKS${PROJECT_NAME} ${links_srcs})
#add_library(RCH${PROJECT_NAME} RestingContact-test.cpp)

#target_link_libraries(BOX${PROJECT_NAME} Moby Ravelin Opt)
#target_link_libraries(HYQ${PROJECT_NAME} Moby Ravelin Opt)
target_link_libraries(LINKS${PROJECT_NAME} Moby Ravelin Opt glut GLU)
#target_link_libraries(RCH${PROJECT_NAME} Moby Ravelin)
