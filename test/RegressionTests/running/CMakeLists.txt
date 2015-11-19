get_filename_component(TEST_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(TEST_BIN_NAME ${TEST_NAME}.test)

# this is the directory, from which cmake was started, i.e. the top level source directory 
MESSAGE( STATUS "CMAKE_BUILD_TYPE:         " ${CMAKE_BUILD_TYPE} )

add_executable(${TEST_BIN_NAME} main.cpp)
if(${USE_GTEST})
  add_definitions(-DUSE_GTEST)
  target_link_libraries(${TEST_BIN_NAME} MobyDriver Moby Pacer gtest_main gtest)
else()
  target_link_libraries(${TEST_BIN_NAME} MobyDriver Moby Pacer)
endif()

add_test(NAME ${TEST_BIN_NAME} COMMAND ${TEST_BIN_NAME} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_dependencies(regression-test ${TEST_BIN_NAME})
