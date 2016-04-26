cmake_minimum_required(VERSION 2.4)

project(val)

set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}
                          ${CMAKE_CURRENT_SOURCE_DIR}/..
                          ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules)

set(DOXYFILE_TARGET_SUFFIX val)
include(UseDoxygen OPTIONAL)

add_subdirectory(parsing)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${ParsePDDL_SOURCE_DIR})

set(libInst_SRCS
      SimpleEval.cpp
      FastEnvironment.cpp instantiation.cpp
      typecheck.cpp TypedAnalyser.cpp FuncAnalysis.cpp
      TIM.cpp TimSupport.cpp TypeStripWC.cpp
)

add_library(Inst STATIC ${libInst_SRCS})

