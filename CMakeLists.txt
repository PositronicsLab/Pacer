cmake_minimum_required(VERSION 2.8.3)

project(MobyController)

find_package(Boost REQUIRED)

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)
set(THIRDPARTY /Users/samzapo/ThirdParty)
# you may have to add additional includes in here...
add_definitions(-DBUILD_DOUBLE)
add_definitions(-DUSE_OSG)
add_definitions(-DSAFESTATIC=static)
add_definitions(-DUSE_GLPK)
include_directories(/usr/include /usr/include/libxml2 /usr/local/include include ${Boost_INCLUDE_DIR})
include_directories(/usr/local/Cellar/qhull/2012.1/include/libqhull ${THIRDPARTY}/Moby/include)
link_directories(/usr/local/lib /usr/lib)
link_directories(${THIRDPARTY}/Moby/build)

# g++ -g0 -O3 -I/usr/include/libxml2 -I../../include -DBUILD_DOUBLE -DSAFESTATIC=static -DUSE_OSG -DUSE_GLPK PD-controller.cpp estimatefrict.cpp output.cpp -L../../../debug -lMoby -shared -o libcontroller.so
set(box_srcs estimatefrict.cpp Box.cpp output.cpp)
set(hyq_srcs estimatefrict.cpp PD-controller.cpp output.cpp)

add_library(BOX${PROJECT_NAME} ${box_srcs})
add_library(HYQ${PROJECT_NAME} ${hyq_srcs})
target_link_libraries(BOX${PROJECT_NAME} Moby)
target_link_libraries(HYQ${PROJECT_NAME} Moby)
