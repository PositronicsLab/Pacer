

add_library(dynamixel MODULE dynamixel.cpp)
target_link_libraries(dynamixel ${REQLIBS})
option(USE_DXL "use dynamixel motor driver for motor control?" OFF)
# look for DXL
IF(USE_DXL)
find_package(DXL)
IF(DXL_FOUND)
  add_definitions(-DUSE_DXL)
	target_link_libraries(dynamixel dxl)
ENDIF(DXL_FOUND)
ENDIF(USE_DXL)

add_library(arduino MODULE arduino.cpp)
target_link_libraries(arduino ${REQLIBS})
option(USE_ARDUINO "use dynamixel motor driver for motor control?" OFF)
IF(USE_ARDUINO)
  	add_definitions(-DUSE_ARDUINO)
  	target_link_libraries(arduino ServoController)
ENDIF(USE_ARDUINO)
