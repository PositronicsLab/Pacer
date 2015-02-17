find_library(RAVELIN_LIBRARY NAMES Ravelin $ENV{RAVELINDIR}/lib)

set(RAVELIN_LIBRARIES ${RAVELIN_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RAVELIN DEFAULT_MSG RAVELIN_LIBRARY)

mark_as_advanced(RAVELIN_LIBRARY)