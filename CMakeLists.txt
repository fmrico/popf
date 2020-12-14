cmake_minimum_required(VERSION 3.5)
project (popf) 

find_package(ament_cmake REQUIRED)
set(dependencies
    rclcpp
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/modules)

find_package(Clp)
find_package(CoinUtils)
find_package(Cbc)
find_package(Cgl)

set(val_SOURCE_DIR src/VALfiles)
set(popf_SOURCE_DIR src/popf)
set(parsePDDL_SOURCE_DIR ${val_SOURCE_DIR}/parsing)

include_directories(
  ${popf_SOURCE_DIR}
  ${val_SOURCE_DIR} 
  ${parsePDDL_SOURCE_DIR}
  ${val_BINARY_DIR} 
  ${CLP_INCLUDES}/coin
  ${CBC_INCLUDES}/coin  
  ${CPLEX_INCLUDES} 
  ${CONCERT_INCLUDES}
)

## ParsePDDL

find_program(BISON_EXECUTABLE bison)

if(NOT EXISTS ${BISON_EXECUTABLE})
	message("Could not find bison executable.")
endif(NOT EXISTS ${BISON_EXECUTABLE})

FIND_PROGRAM(FLEX_EXECUTABLE flex)

if(NOT EXISTS ${FLEX_EXECUTABLE})
        message("Could not find flex executable.")
endif(NOT EXISTS ${FLEX_EXECUTABLE})

add_custom_command(
       OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc
       COMMAND
        ${FLEX_EXECUTABLE}
       ARGS
       -+  --outfile=${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/pddl+.lex
       DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/pddl+.lex
)

add_custom_command(
       OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
       COMMAND
        ${BISON_EXECUTABLE}
       ARGS
        ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/pddl+.yacc -o ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
       DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/pddl+.yacc ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc
       COMMAND
        ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/fixyywrap ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/
)

set(libParsePDDL_SRCS
      ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
      ${parsePDDL_SOURCE_DIR}/ptree.cpp
      ${parsePDDL_SOURCE_DIR}/DebugWriteController.cpp
)

add_library(ParsePDDL SHARED ${libParsePDDL_SRCS})

## Val

set(libInst_SRCS
      ${val_SOURCE_DIR}/SimpleEval.cpp
      ${val_SOURCE_DIR}/FastEnvironment.cpp
      ${val_SOURCE_DIR}/instantiation.cpp
      ${val_SOURCE_DIR}/typecheck.cpp
      ${val_SOURCE_DIR}/TypedAnalyser.cpp
      ${val_SOURCE_DIR}/FuncAnalysis.cpp
      ${val_SOURCE_DIR}/TIM.cpp 
      ${val_SOURCE_DIR}/TimSupport.cpp 
      ${val_SOURCE_DIR}/TypeStripWC.cpp
)

add_library(Inst SHARED ${libInst_SRCS})
target_link_libraries(Inst ParsePDDL)

set(validate_SRCS
     ${val_SOURCE_DIR}/Action.cpp 
     ${val_SOURCE_DIR}/Proposition.cpp 
     ${val_SOURCE_DIR}/FuncExp.cpp 
     ${val_SOURCE_DIR}/typecheck.cpp 
     ${val_SOURCE_DIR}/main.cpp
     ${val_SOURCE_DIR}/Validator.cpp 
     ${val_SOURCE_DIR}/RepairAdvice.cpp 
     ${val_SOURCE_DIR}/LaTeXSupport.cpp 
     ${val_SOURCE_DIR}/State.cpp 
     ${val_SOURCE_DIR}/Plan.cpp 
     ${val_SOURCE_DIR}/Ownership.cpp
     ${val_SOURCE_DIR}/Environment.cpp 
     ${val_SOURCE_DIR}/Polynomial.cpp 
     ${val_SOURCE_DIR}/Utils.cpp
     ${val_SOURCE_DIR}/TrajectoryConstraints.cpp 
     ${val_SOURCE_DIR}/RobustAnalyse.cpp 
     ${val_SOURCE_DIR}/random.cpp 
     ${val_SOURCE_DIR}/Events.cpp 
     ${val_SOURCE_DIR}/PrettyPrinter.cpp)
     
add_executable(validate ${validate_SRCS})

target_link_libraries(validate
  ParsePDDL
)

set(parser_SRCS
     ${val_SOURCE_DIR}/parse.cpp
)

add_executable(parser ${parser_SRCS})

target_link_libraries(parser
        ParsePDDL
)

install(TARGETS validate parser ParsePDDL
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Popf

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
  ${CBC_LIBRARY} ${CGL_LIBRARIES} ${CLP_LIBRARIES}  ${COINUTILS_LIBRARIES}
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
ament_export_libraries(popfCommon ParsePDDL Inst ${CBC_LIBRARY} ${CGL_LIBRARIES} ${OSICLP_LIBRARIES} ${OSI_LIBRARIES} ${CLP_LIBRARIES}  ${COINUTILS_LIBRARIES})
ament_export_dependencies(${dependencies})
ament_package()