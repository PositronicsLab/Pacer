get_filename_component(TEST_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(TEST_BIN_NAME ${TEST_NAME}.demo)

# this is the directory, from which cmake was started, i.e. the top level source directory 
MESSAGE( STATUS "CMAKE_BUILD_TYPE:         " ${CMAKE_BUILD_TYPE} )

add_executable(${TEST_BIN_NAME} main.cpp)
  
target_link_libraries(${TEST_BIN_NAME} MobyDriver Moby Pacer)
