# include(FindLibraryWithDebug)

if (CLAPACK_LIBRARIES)
  set(CLAPACK_FIND_QUIETLY TRUE)
endif (CLAPACK_LIBRARIES)

find_library(CLAPACK_LIBRARIES
  lapack
  PATHS
  $ENV{CLAPACKDIR}/lib
  ${LIB_INSTALL_DIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CLAPACK DEFAULT_MSG
                                  CLAPACK_LIBRARIES)

mark_as_advanced(CLAPACK_LIBRARIES)

