
option(BUILD_MONTECARLO "Build Monte carlo simulaton method plugin?" OFF)
if(BUILD_MONTECARLO)
  add_library(monte-carlo-simulation MODULE simulate.cpp)

  target_link_libraries(monte-carlo-simulation ${REQLIBS})

  find_package (gsl QUIET)
  IF(GSL_FOUND)
  add_definitions(-DUSE_GSL)
  include_directories(${GSL_INCLUDE_DIRS})
  target_link_libraries(monte-carlo-simulation ${GSL_LIBRARIES})
  ENDIF(GSL_FOUND)

#find BOOST libraries
set(Boost_ADDITIONAL_VERSIONS "1.55.0" "1.56.0")
find_package(Boost COMPONENTS program_options REQUIRED)
link_directories(${BOOST_LIBRARY_DIRS})
include_directories(${BOOST_INCLUDE_DIR})

  add_executable(sample.bin Sample.cpp)
  target_link_libraries(sample.bin ${REQLIBS} boost_program_options MobyDriver)
endif(BUILD_MONTECARLO)