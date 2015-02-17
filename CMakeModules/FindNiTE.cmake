# - Find NiTE
# Find the native NiTE includes and library
# This module defines
#  NITE_INCLUDE_DIR, where to find XnNiTE.h, etc.
#  NITE_LIBRARIES, the libraries needed to use NiTE.
#  NITE_FOUND, If false, do not try to use NiTE.
# also defined, but not for general use are
#  NITE_LIBRARY, where to find the NiTE library.
#  NITE_VERSION, the version of the NiTE library.

#=============================================================================
# Copyright 2012 UMONS, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distributed this file outside of CMake, substitute the full
#  License text for the above reference.)

FIND_PATH(NITE_INCLUDE_DIR NAMES NiTE.h PATH_SUFFIXES "ni" "NiTE")

FIND_LIBRARY(NITE_LIBRARY NAMES NiTE NiTE2)

# handle the QUIETLY and REQUIRED arguments and set NITE_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(NiTE DEFAULT_MSG NITE_LIBRARY NITE_INCLUDE_DIR)

IF(NITE_FOUND)
  SET(NITE_LIBRARIES ${NITE_LIBRARY})
  GET_FILENAME_COMPONENT(NITE_LINK_DIRECTORIES ${NITE_LIBRARY} PATH)
ENDIF(NITE_FOUND)

# Deprecated declarations.
SET (NATIVE_NITE_INCLUDE_PATH ${NITE_INCLUDE_DIR} )
IF(NITE_LIBRARY)
  GET_FILENAME_COMPONENT (NATIVE_NITE_LIB_PATH ${NITE_LIBRARY} PATH)
ENDIF(NITE_LIBRARY)

MARK_AS_ADVANCED(NITE_LIBRARY NITE_INCLUDE_DIR NITE_LINK_DIRECTORIES)