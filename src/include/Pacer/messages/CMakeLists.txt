cmake_minimum_required(VERSION 2.8.7)

set( COMPONENT_NAME "messages" )
set( COMPONENT_DIR "${COMPONENT_NAME}")
project( "pacer_${COMPONENT_NAME}" )

find_package( Protobuf REQUIRED )

#add_subdirectory( Reveal/messages )
# add any messages to be compiled here
set( PROTOS
  data.proto
  sample.proto
)

#------------------------------------------------------
# add the protobuf include directories
include_directories( ${PROTOBUF_INCLUDE_DIRS} )
# generate the message classes
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS ${PROTOS} )
# Note: above generates to build folder.  protoc has --cpp_out parameter to 
# direct this location, but macro assumes build folder.  To put into a more
# controlled location, write a macro that exposes --cpp_out and set
# REVEAL_MESSAGES_INCLUDE_DIRS

#------------------------------------------------------
set( LIBS
  ${PROTOBUF_LIBRARY}
)

#------------------------------------------------------
# build the message library
add_library(${PROJECT_NAME} SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME} ${LIBS})

#------------------------------------------------------
set_target_properties( ${PROJECT_NAME} PROPERTIES SOVERSION ${REVEAL_MAJOR_VERSION} VERSION ${REVEAL_VERSION_FULL} )
install( TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_BINARY_DIR} COMPONENT shlib )
install( FILES ${PROTO_HDRS} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} COMPONENT headers )

