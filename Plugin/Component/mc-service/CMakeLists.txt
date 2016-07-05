option(BUILD_MONTECARLO_SERVICE "Build Monte carlo simulaton method plugin?" OFF)
if(BUILD_MONTECARLO_SERVICE)

find_package(reveal REQUIRED)
  link_directories(${REVEAL_LIBRARY_DIRS})
  include_directories(${REVEAL_INCLUDE_DIR})

  add_executable(simple-robot simple-robot.cpp)
add_executable(simple-simulation simple-simulation.cpp )
add_executable(simple-planner simple-planner.cpp )

  target_link_libraries(simple-robot ${REQLIBS} reveal_core )
target_link_libraries(simple-simulation ${REQLIBS} reveal_core )
target_link_libraries(simple-planner ${REQLIBS} reveal_core )

if(USE_PROTOBUF)
# build protobuf messages
add_subdirectory( messages )
endif(USE_PROTOBUF)


  target_link_libraries(sample.bin ${REQLIBS} boost_program_options MobyDriver reveal_core)
  IF(USE_DISPLAY)
    #find OSG
    find_package (osg)
    find_package (osgViewer)
    find_package (osgDB)
    find_package (osgGA)
    find_package (osgFX)
    find_package (osgUtil)
    find_package (OpenThreads)
    link_directories(${OSG_LIBRARY_DIRS})
    include_directories(${OSG_INCLUDE_DIR})
    IF(OSG_FOUND)
      target_link_libraries (sample.bin ${OSG_LIBRARIES})
      target_link_libraries (sample.bin ${OSGVIEWER_LIBRARIES})
      target_link_libraries (sample.bin ${OSGDB_LIBRARIES})
      target_link_libraries (sample.bin ${OSGGA_LIBRARIES})
      target_link_libraries (sample.bin ${OPENTHREADS_LIBRARY})
      target_link_libraries (sample.bin ${OSGUTIL_LIBRARIES})
      target_link_libraries (sample.bin ${OSGMANIPULATOR_LIBRARIES})
      target_link_libraries (sample.bin OpenThreads)
      add_definitions (-DUSE_OSG_DISPLAY)
    ENDIF(OSG_FOUND)
  ENDIF(USE_DISPLAY)

  add_library(test-probability MODULE pendulum/pendulum-init.cpp)
  target_link_libraries(test-probability ${REQLIBS})

endif(BUILD_MONTECARLO_SERVICE)

