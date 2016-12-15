


option(BUILD_MONTECARLO "Build Monte carlo simulaton method plugin?" OFF)
if(BUILD_MONTECARLO)
  find_package(reveal REQUIRED)
  link_directories(${REVEAL_LIBRARY_DIRS})
  include_directories(${REVEAL_INCLUDE_DIR})
  add_definitions (-DUSE_ZMQ)

foreach(copy_number RANGE 16)
  add_library(monte-carlo-simulation-${copy_number} MODULE Simulate.cpp)
  target_link_libraries(monte-carlo-simulation-${copy_number} ${REQLIBS} reveal_core)
endforeach()

#add_library(monte-carlo-print MODULE print.cpp)
# target_link_libraries(monte-carlo-print ${REQLIBS})

  option(BUILD_MC_TESTS "Build Monte carlo communication tests?" ON)
  if(BUILD_MC_TESTS)
    add_subdirectory( test )
  endif(BUILD_MC_TESTS)
endif(BUILD_MONTECARLO)

#find BOOST libraries
set(Boost_ADDITIONAL_VERSIONS "1.55.0" "1.56.0")
find_package(Boost COMPONENTS program_options REQUIRED)
link_directories(${BOOST_LIBRARY_DIRS})
include_directories(${BOOST_INCLUDE_DIR})

add_executable(sample.bin Sample.cpp)

if(BUILD_MONTECARLO)
  target_link_libraries(sample.bin reveal_core)
endif(BUILD_MONTECARLO)

target_link_libraries(sample.bin ${REQLIBS} boost_program_options MobyDriver PacerDriver)
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


