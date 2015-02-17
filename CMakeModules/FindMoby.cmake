find_library(MOBY_LIBRARY NAMES Moby $ENV{MOBYDIR}/lib)

set(MOBY_LIBRARIES ${MOBY_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MOBY DEFAULT_MSG MOBY_LIBRARY)

mark_as_advanced(MOBY_LIBRARY)
