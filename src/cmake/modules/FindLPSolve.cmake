# include(FindLibraryWithDebug)

if (LPSOLVE_INCLUDES AND LPSOLVE_LIBRARIES)
  set(LPSOLVE_FIND_QUIETLY TRUE)
endif (LPSOLVE_INCLUDES AND LPSOLVE_LIBRARIES)

find_path(LPSOLVE_INCLUDES
  NAMES
  lpsolve/lp_lib.h
  PATHS
  $ENV{LPSOLVEDIR}/include
  ${INCLUDE_INSTALL_DIR}
)

find_library(LPSOLVE_LIBRARIES
  lpsolve55
  PATHS
  $ENV{LPSOLVEDIR}/lib
  ${LIB_INSTALL_DIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LPSolve DEFAULT_MSG
                                  LPSOLVE_INCLUDES LPSOLVE_LIBRARIES)

mark_as_advanced(LPSOLVE_INCLUDES LPSOLVE_LIBRARIES)

