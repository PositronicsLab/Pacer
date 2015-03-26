  # Gazebo Interface
  add_library(PacerMobyPlugin MODULE ${CMAKE_SOURCE_DIR}/example/interfaces/moby.cpp)
# osg
IF(USE_OSG_DISPLAY AND OSG_FOUND)
    target_link_libraries (PacerMobyPlugin ${OSG_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OSGVIEWER_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OSGDB_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OSGGA_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OPENTHREADS_LIBRARY})
    target_link_libraries (PacerMobyPlugin ${OSGUTIL_LIBRARIES})
    target_link_libraries (PacerMobyPlugin ${OSGMANIPULATOR_LIBRARIES})
    target_link_libraries (PacerMobyPlugin OpenThreads)
   
   # target_link_libraries (PacerMobyDriver ${OSG_LIBRARIES})
   # target_link_libraries (PacerMobyDriver ${OSGVIEWER_LIBRARIES})
   # target_link_libraries (PacerMobyDriver ${OSGDB_LIBRARIES})
   # target_link_libraries (PacerMobyDriver ${OSGGA_LIBRARIES})
   # target_link_libraries (PacerMobyDriver ${OPENTHREADS_LIBRARY})
   # target_link_libraries (PacerMobyDriver ${OSGUTIL_LIBRARIES})
   # target_link_libraries (PacerMobyDriver ${OSGMANIPULATOR_LIBRARIES})
   # target_link_libraries (PacerMobyDriver OpenThreads)
    add_definitions (-DUSE_OSG_DISPLAY)
ENDIF(USE_OSG_DISPLAY AND OSG_FOUND)
target_link_libraries(PacerMobyPlugin ${REQLIBS} ${EXTRA_LIBS})

  # Gazebo Interface
IF(GAZEBO_FOUND)
  add_library(PacerGazeboPlugin SHARED ${CMAKE_SOURCE_DIR}/example/interfaces/gazebo.cpp)
  target_link_libraries(PacerGazeboPlugin ${REQLIBS} ${GAZEBO_LIBRARIES} ${BOOST_LIBRARIES} ${Boost_SYSTEM_LIBRARY} protobuf ${EXTRA_LIBS})
ENDIF(GAZEBO_FOUND)

# look for DXL
find_package(DXL)
IF(DXL_FOUND)
  add_executable(PacerMotorDriver ${CMAKE_SOURCE_DIR}/example/interfaces/motor.cpp)
  target_link_libraries(PacerMotorDriver ${REQLIBS} dxl)
ENDIF(DXL_FOUND)
