# Find SDL2 header and library.
#

# This module defines the following uncached variables:
#  SDL2_FOUND, if false, do not try to use SDL2.
#  SDL2_INCLUDE_DIRS, where to find SDL2/SDL2_a.h.
#  SDL2_LIBRARIES, the libraries to link against to use the SDL2 library
#  SDL2_LIBRARY_DIRS, the directory where the SDL2 library is found.

find_path(
  SDL2_INCLUDE_DIR
  NAMES SDL2/SDL.h
  PATHS /usr/local/include /usr/include
)

if( SDL2_INCLUDE_DIR )
  find_library(
    SDL2_LIBRARY
    NAMES libSDL2.so libSDL2.dylib libSDL2.a 
    PATHS /usr/local/lib /usr/lib
  )
  if( SDL2_LIBRARY )
    set(SDL2_LIBRARY_DIR "")
    get_filename_component(SDL2_LIBRARY_DIRS ${SDL2_LIBRARY} PATH)
    # Set uncached variables as per standard.
    set(SDL2_FOUND ON)
    set(SDL2_INCLUDE_DIRS ${SDL2_INCLUDE_DIR})
    set(SDL2_LIBRARIES ${SDL2_LIBRARY})
  endif(SDL2_LIBRARY)
else(SDL2_INCLUDE_DIR)
  message(FATAL_ERROR "FindSDL2: Could not find SDL.h")
endif(SDL2_INCLUDE_DIR)
	    
if(SDL2_FOUND)
  if(NOT SDL2_FIND_QUIETLY)
    message(STATUS "FindSDL2: Found both SDL.h and libSDL2")
  endif(NOT SDL2_FIND_QUIETLY)
else(SDL2_FOUND)
  if(SDL2_FIND_REQUIRED)
    message(FATAL_ERROR "FindSDL2: Could not find SDL.h and/or libSDL2")
  endif(SDL2_FIND_REQUIRED)
endif(SDL2_FOUND)
