add_library(drive-robot MODULE drive-robot.cpp)
target_link_libraries(drive-robot ${REQLIBS})

add_library(ik-feet MODULE ik_feet.cpp)
target_link_libraries(ik-feet ${REQLIBS})

add_library(gait-planner MODULE gait-planner.cpp)
target_link_libraries(gait-planner ${REQLIBS})

add_library(joint-PID-controller MODULE joint-PID-controller.cpp)
target_link_libraries(joint-PID-controller ${REQLIBS})

add_library(eef-PID-controller MODULE eef-PID-controller.cpp)
target_link_libraries(eef-PID-controller ${REQLIBS})

add_library(stand-trajectory MODULE stand.cpp)
target_link_libraries(stand-trajectory ${REQLIBS})

add_library(inverse-dynamics MODULE inverse_dynamics.cpp)
target_link_libraries(inverse-dynamics ${REQLIBS})
