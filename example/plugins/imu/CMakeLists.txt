add_library(imu-reader MODULE interface.cpp driver.cpp)
target_link_libraries(imu-reader ${REQLIBS} boost_system)
