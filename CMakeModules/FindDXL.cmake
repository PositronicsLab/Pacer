# Find DXL header and library.
#

# This module defines the following uncached variables:
#  DXL_FOUND, if false, do not try to use DXL.
#  DXL_INCLUDE_DIRS, where to find DXL/DXL_a.h.
#  DXL_LIBRARIES, the libraries to link against to use the DXL library
#  DXL_LIBRARY_DIRS, the directory where the DXL library is found.

find_path(
  DXL_INCLUDE_DIR
  NAMES dxl/Dynamixel.h
  PATHS /usr/local/include /usr/include
)

if( DXL_INCLUDE_DIR )
  find_library(
    DXL_LIBRARY
    NAMES libdxl.so libdxl.dylib libdxl.a 
    PATHS /usr/local/lib /usr/lib
  )
  if( DXL_LIBRARY )
    set(DXL_LIBRARY_DIR "")
    get_filename_component(DXL_LIBRARY_DIRS ${DXL_LIBRARY} PATH)
    # Set uncached variables as per standard.
    set(DXL_FOUND ON)
    set(DXL_INCLUDE_DIRS ${DXL_INCLUDE_DIR})
    set(DXL_LIBRARIES ${DXL_LIBRARY})
  endif(DXL_LIBRARY)
else(DXL_INCLUDE_DIR)
    message(STATUS "FindDXL: Found both dxl/Dynamixel.h and libdxl")
endif(DXL_INCLUDE_DIR)
	    
if(DXL_FOUND)
  if(NOT DXL_FIND_QUIETLY)
    message(STATUS "FindDXL: Found both dxl/Dynamixel.h and libdxl")
  endif(NOT DXL_FIND_QUIETLY)
else(DXL_FOUND)
  if(DXL_FIND_REQUIRED)
    message(FATAL_ERROR "FindDXL: Could not find dxl/Dynamixel.h and/or libdxl")
  endif(DXL_FIND_REQUIRED)
endif(DXL_FOUND)
