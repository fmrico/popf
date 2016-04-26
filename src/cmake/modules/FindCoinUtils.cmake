# include(FindLibraryWithDebug)

if (COINUTILS_INCLUDES AND COINUTILS_LIBRARIES)
  set(COINUTILS_FIND_QUIETLY TRUE)
endif (COINUTILS_INCLUDES AND COINUTILS_LIBRARIES)

find_path(COINUTILS_INCLUDES
  NAMES
  coin/CoinPragma.hpp
  PATHS
  $ENV{CLPDIR}/include
  ${INCLUDE_INSTALL_DIR}
)

find_library(COINUTILS_LIBRARIES
  CoinUtils
  PATHS
  $ENV{CLPDIR}/lib
  ${LIB_INSTALL_DIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CoinUtils DEFAULT_MSG
                                  COINUTILS_INCLUDES COINUTILS_LIBRARIES)

mark_as_advanced(COINUTILS_INCLUDES COINUTILS_LIBRARIES)

