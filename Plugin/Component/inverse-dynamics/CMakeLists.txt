add_library(inverse-dynamics MODULE plugin.cpp)
target_link_libraries(inverse-dynamics ${REQLIBS})


# Moby Interface
add_library(TestPlugin MODULE test.cpp)
target_link_libraries (TestPlugin ${REQLIBS})

add_definitions (-DUSE_OSG_DISPLAY)
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
target_link_libraries (TestPlugin ${OSG_LIBRARIES})
target_link_libraries (TestPlugin ${OSGVIEWER_LIBRARIES})
target_link_libraries (TestPlugin ${OSGDB_LIBRARIES})
target_link_libraries (TestPlugin ${OSGGA_LIBRARIES})
target_link_libraries (TestPlugin ${OPENTHREADS_LIBRARY})
target_link_libraries (TestPlugin ${OSGUTIL_LIBRARIES})
target_link_libraries (TestPlugin ${OSGMANIPULATOR_LIBRARIES})
target_link_libraries (TestPlugin OpenThreads)
ENDIF(OSG_FOUND)