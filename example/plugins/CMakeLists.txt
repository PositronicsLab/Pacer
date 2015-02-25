
     add_definitions(-DUSE_PLUGINS)
  add_library(PacerDriveRobotPlugin ${CMAKE_SOURCE_DIR}/example/drive-robot/control.cpp)
  target_link_libraries(PacerDriveRobotPlugin ${REQLIBS})
  set_target_properties(PacerDriveRobotPlugin PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  install(TARGETS PacerDriveRobotPlugin DESTINATION ${CMAKE_SOURCE_DIR}/example/plugins)
