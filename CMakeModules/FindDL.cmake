###############################################################################
# CMake macro to find libdl library.
#
# On success, the macro sets the following variables:
# DL_FOUND       = if the library found
# DL_LIBRARY     = full path to the library
# DL_INCLUDE_DIR = where to find the library headers
#
# Author: Mateusz Loskot <mateusz@loskot.net>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###############################################################################

if(DL_INCLUDE_DIR)
  set(DL_FIND_QUIETLY TRUE)
endif()

find_path(DL_INCLUDE_DIR dlfcn.h)
find_library(DL_LIBRARY NAMES dl)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DL DEFAULT_MSG DL_LIBRARY DL_INCLUDE_DIR)

if(NOT DL_FOUND)
    # if dlopen can be found without linking in dl then,
    # dlopen is part of libc, so don't need to link extra libs.
    check_function_exists(dlopen DL_FOUND)
    set(DL_LIBRARY "")
endif()

set(DL_LIBRARIES ${DL_LIBRARY})

mark_as_advanced(DL_LIBRARY DL_INCLUDE_DIR)
