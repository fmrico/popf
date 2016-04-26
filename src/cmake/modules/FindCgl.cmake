# include(FindLibraryWithDebug)

if (CGL_INCLUDES AND CGL_LIBRARIES)
  set(CGL_FIND_QUIETLY TRUE)
endif (CGL_INCLUDES AND CGL_LIBRARIES)

find_path(CGL_INCLUDES
  NAMES
  coin/CglConfig.h
  PATHS
  $ENV{CGLDIR}/include
  ${INCLUDE_INSTALL_DIR}
)

find_library(CGL_LIBRARIES
  Cgl
  PATHS
  $ENV{CGLDIR}/lib
  ${LIB_INSTALL_DIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Cgl DEFAULT_MSG
                                  CGL_INCLUDES CGL_LIBRARIES)

mark_as_advanced(CGL_INCLUDES CGL_LIBRARIES)

