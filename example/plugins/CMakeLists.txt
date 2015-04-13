add_library(waypoints MODULE waypoints.cpp)
target_link_libraries(waypoints ${REQLIBS})

add_library(ik-feet MODULE ik-feet.cpp)
target_link_libraries(ik-feet ${REQLIBS})

add_library(stabilization MODULE stabilization.cpp)
target_link_libraries(stabilization ${REQLIBS})

add_library(skating-stabilization MODULE skating-stabilization.cpp)
target_link_libraries(skating-stabilization ${REQLIBS})

add_library(gait-planner MODULE gait-planner.cpp)
target_link_libraries(gait-planner ${REQLIBS})

add_library(joint-PID-controller MODULE joint-PID-controller.cpp)
target_link_libraries(joint-PID-controller ${REQLIBS})

add_library(eef-PID-controller MODULE eef-PID-controller.cpp)
target_link_libraries(eef-PID-controller ${REQLIBS})

add_library(stand-trajectory MODULE stand-trajectory.cpp)
target_link_libraries(stand-trajectory ${REQLIBS})

add_library(inverse-dynamics MODULE inverse-dynamics.cpp)
target_link_libraries(inverse-dynamics ${REQLIBS})

find_package(SDL2)
find_package(curses)

IF(SDL2_FOUND)
  IF(CURSES_FOUND AND USE_CURSES)
    add_definitions( -DUSE_CURSES )
  ENDIF()
  add_library(gamepad MODULE gamepad.cpp)
  target_link_libraries(gamepad ${REQLIBS} ${SDL2_LIBRARIES} ${CURSES_LIBRARIES})
ENDIF(SDL2_FOUND)
