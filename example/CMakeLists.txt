  option(BUILD_PLUGINS "Build Pacer plugins?" ON)
  IF(BUILD_PLUGINS)
    add_subdirectory( plugins )
  ENDIF(BUILD_PLUGINS)

  option(BUILD_INTERFACES "Build Pacer Interfaces?" ON)
  IF(BUILD_INTERFACES)
    add_subdirectory( interfaces )
  ENDIF(BUILD_INTERFACES)
