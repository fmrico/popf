# include(FindLibraryWithDebug)

if (CLP_INCLUDES AND CLP_LIBRARIES)
  set(CLP_FIND_QUIETLY TRUE)
endif (CLP_INCLUDES AND CLP_LIBRARIES)

find_path(CLP_INCLUDES
  NAMES
  coin/ClpSimplex.hpp
  PATHS
  $ENV{CLPDIR}/include
  ${INCLUDE_INSTALL_DIR}
)

find_library(CLP_LIBRARIES
  Clp
  PATHS
  $ENV{CLPDIR}/lib
  ${LIB_INSTALL_DIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Clp DEFAULT_MSG
                                  CLP_INCLUDES CLP_LIBRARIES)

mark_as_advanced(CLP_INCLUDES CLP_LIBRARIES)

