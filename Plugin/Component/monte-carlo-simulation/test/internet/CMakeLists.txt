find_package(reveal REQUIRED)
  link_directories(${REVEAL_LIBRARY_DIRS})
  include_directories(${REVEAL_INCLUDE_DIR})

  add_executable(simple-robot simple-robot.cpp)
add_executable(simple-simulation simple-simulation.cpp )
add_executable(simple-planner simple-planner.cpp )

  target_link_libraries(simple-robot ${REQLIBS} reveal_core )
target_link_libraries(simple-simulation ${REQLIBS} reveal_core )
target_link_libraries(simple-planner ${REQLIBS} reveal_core )