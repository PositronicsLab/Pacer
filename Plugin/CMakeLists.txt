  option(BUILD_COMPONENT "Build Pacer's (planning/perception/control) Components?" ON)
  IF(BUILD_COMPONENT)
    add_subdirectory( Component )
  ENDIF(BUILD_COMPONENT)

  option(BUILD_SIMULATOR_PLUGINS "Build Simulator (interface/adapter) plugins for Pacer?" ON)
  IF(BUILD_SIMULATOR_PLUGINS)
    add_subdirectory( Simulator )
  ENDIF(BUILD_SIMULATOR_PLUGINS)
