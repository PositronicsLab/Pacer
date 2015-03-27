# Find curses header and library.
#

# This module defines the following uncached variables:
#  CURSES_FOUND, if false, do not try to use curses.
#  CURSES_INCLUDE_DIRS, where to find curses.h.
#  CURSES_LIBRARIES, the libraries to link against to use the curses library
#  CURSES_LIBRARY_DIRS, the directory where the curses library is found.

find_path(
  CURSES_INCLUDE_DIR
  NAMES curses.h
  PATHS /usr/local/include /usr/include
)

if( CURSES_INCLUDE_DIR )
  find_library(
    CURSES_LIBRARY
    NAMES libcurses.so libcurses.dylib libcurses.a 
    PATHS /usr/local/lib /usr/lib
  )
  if( CURSES_LIBRARY )
    set(CURSES_LIBRARY_DIR "")
    get_filename_component(CURSES_LIBRARY_DIRS ${CURSES_LIBRARY} PATH)
    # Set uncached variables as per standard.
    set(CURSES_FOUND ON)
    set(CURSES_INCLUDE_DIRS ${CURSES_INCLUDE_DIR})
    set(CURSES_LIBRARIES ${CURSES_LIBRARY})
  endif(CURSES_LIBRARY)
else(CURSES_INCLUDE_DIR)
  message(STATUS "Findcurses: Could not find curses.h")
endif(CURSES_INCLUDE_DIR)
	    
if(CURSES_FOUND)
  if(NOT CURSES_FIND_QUIETLY)
    message(STATUS "Findcurses: Found both curses.h and libcurses")
  endif(NOT CURSES_FIND_QUIETLY)
else(CURSES_FOUND)
  if(CURSES_FIND_REQUIRED)
    message(FATAL_ERROR "Findcurses: Could not find curses.h and/or libcurses")
  endif(CURSES_FIND_REQUIRED)
endif(CURSES_FOUND)
