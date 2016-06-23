# Find REVEAL header and library.
#

# This module defines the following uncached variables:
#  REVEAL_FOUND, if false, do not try to use REVEAL.
#  REVEAL_INCLUDE_DIRS, where to find REVEAL/REVEAL_a.h.
#  REVEAL_LIBRARIES, the libraries to link against to use the REVEAL library
#  REVEAL_LIBRARY_DIRS, the directory where the REVEAL library is found.

find_path(
  REVEAL_INCLUDE_DIR
  NAMES reveal/core/system.h
  PATHS /usr/local/include /usr/include
)

if( REVEAL_INCLUDE_DIR )
  find_library(
    REVEAL_LIBRARY
    NAMES libreveal_core.so libreveal_core.dylib libreveal_core.a
    PATHS /usr/local/lib /usr/lib ../
  )
  if( REVEAL_LIBRARY )
    set(REVEAL_LIBRARY_DIR "")
    get_filename_component(REVEAL_LIBRARY_DIRS ${REVEAL_LIBRARY} PATH)
    # Set uncached variables as per standard.
    set(REVEAL_FOUND ON)
    set(REVEAL_INCLUDE_DIRS ${REVEAL_INCLUDE_DIR})
    set(REVEAL_LIBRARY_DIR ${REVEAL_LIBRARY_DIRS})
    set(REVEAL_LIBRARIES ${REVEAL_LIBRARY})
  endif(REVEAL_LIBRARY)
else(REVEAL_INCLUDE_DIR)
  message(STATUS "FindREVEAL: Could not find core/system.h")
endif(REVEAL_INCLUDE_DIR)
	    
if(REVEAL_FOUND)
  if(NOT REVEAL_FIND_QUIETLY)
    message(STATUS "FindREVEAL: Found both core/system.h and libreveal_core")
  endif(NOT REVEAL_FIND_QUIETLY)
else(REVEAL_FOUND)
  if(REVEAL_FIND_REQUIRED)
    message(FATAL_ERROR "FindREVEAL: Could not find core/system.h and/or libreveal_core")
  endif(REVEAL_FIND_REQUIRED)
endif(REVEAL_FOUND)
