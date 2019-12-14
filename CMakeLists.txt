cmake_minimum_required(VERSION 3.5)
project (popf) 

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
set(dependencies
    rclcpp
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/modules)

add_subdirectory(src/VALfiles)

find_package(Clp)
find_package(CoinUtils)
find_package(Cbc)
find_package(Cgl)

set(val_SOURCE_DIR src/VALfiles)
set(popf_SOURCE_DIR src/popf)

include_directories(
  ${popf_SOURCE_DIR}
  ${val_SOURCE_DIR} 
  ${val_SOURCE_DIR}/parsing 
  ${val_BINARY_DIR} 
  ${CLP_INCLUDES}/coin 
  ${CBC_INCLUDES}/coin 
  ${CPLEX_INCLUDES} 
  ${CONCERT_INCLUDES}
)

set(popf_build_srcs
    ${popf_SOURCE_DIR}/totalordertransformer.cpp
    ${popf_SOURCE_DIR}/partialordertransformer.cpp
    ${popf_SOURCE_DIR}/FFSolver.cpp
    ${popf_SOURCE_DIR}/globals.cpp
    ${popf_SOURCE_DIR}/lpscheduler.cpp
    ${popf_SOURCE_DIR}/RPGBuilder.cpp
    ${popf_SOURCE_DIR}/RPGBuilderAnalysis.cpp
    ${popf_SOURCE_DIR}/RPGBuilderEvaluation.cpp
    ${popf_SOURCE_DIR}/RPGBuilderNumerics.cpp
    ${popf_SOURCE_DIR}/temporalanalysis.cpp
    ${popf_SOURCE_DIR}/minimalstate.cpp
    ${popf_SOURCE_DIR}/temporalconstraints.cpp
    ${popf_SOURCE_DIR}/numericanalysis.cpp
    ${popf_SOURCE_DIR}/solver.cpp
    ${popf_SOURCE_DIR}/solver-clp.cpp
    ${popf_SOURCE_DIR}/Popf.cpp)

add_library(popfCommon SHARED ${popf_build_srcs})
target_link_libraries(popfCommon ParsePDDL Inst)
install(TARGETS popfCommon ParsePDDL Inst
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ADD_EXECUTABLE(popf ${popf_SOURCE_DIR}/popfMain.cpp)
target_link_libraries(popf
  popfCommon
  ${CBC_SOLVER_LIBRARY} ${CBC_LIBRARY} ${CGL_LIBRARIES} ${CLP_LIBRARIES}  ${COINUTILS_LIBRARIES}
)
install(TARGETS popf
  ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES src/popf/Popf.hpp
  DESTINATION include/popf
)

ament_export_include_directories(include)
ament_export_libraries(popfCommon ParsePDDL Inst ${CBC_SOLVER_LIBRARY} ${CBC_LIBRARY} ${CGL_LIBRARIES} ${OSICLP_LIBRARIES} ${OSI_LIBRARIES} ${CLP_LIBRARIES}  ${COINUTILS_LIBRARIES})
ament_export_dependencies(${dependencies})
ament_package()