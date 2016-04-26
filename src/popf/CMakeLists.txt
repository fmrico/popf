PROJECT(popf)

set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}
                          ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules)


find_package(Clp)
find_package(CoinUtils)
find_package(Cbc)
find_package(Cgl)
find_package(OSI)

set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")

if(STATIC_POPF)
set(CMAKE_CXX_LINK_FLAGS "-static ${CMAKE_CXX_LINK_FLAGS}")
endif(STATIC_POPF)

include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${val_BINARY_DIR} ${CLP_INCLUDES}/coin ${CBC_INCLUDES}/coin ${CPLEX_INCLUDES} ${CONCERT_INCLUDES})

set(popf_build_srcs
    totalordertransformer.cpp
    partialordertransformer.cpp
    FFSolver.cpp
    globals.cpp
    lpscheduler.cpp
    RPGBuilder.cpp
    RPGBuilderAnalysis.cpp
    RPGBuilderEvaluation.cpp
    RPGBuilderNumerics.cpp
    temporalanalysis.cpp
    minimalstate.cpp
    temporalconstraints.cpp
    numericanalysis.cpp
    solver.cpp)

add_library(popfCommon STATIC ${popf_build_srcs})
target_link_libraries(popfCommon ParsePDDL Inst)
    install(TARGETS popfCommon
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

#if((NOT DISABLE_CLP) AND CLP_INCLUDES)
if (CLP_INCLUDES AND CBC_INCLUDES)
    ADD_EXECUTABLE(popf popfMain.cpp solver-clp.cpp)
    
    target_link_libraries(popf
        popfCommon
        ${CBC_SOLVER_LIBRARY} ${CBC_LIBRARY} ${CGL_LIBRARIES} ${OSICLP_LIBRARIES} ${OSI_LIBRARIES} ${CLP_LIBRARIES}  ${COINUTILS_LIBRARIES}
    )
    install(TARGETS popf
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
                
#endif((NOT DISABLE_CLP) AND CLP_INCLUDES)
endif(CLP_INCLUDES AND CBC_INCLUDES)

if(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES)
    ADD_EXECUTABLE(popf-cplex popfMain.cpp solver-cpx.cpp)
    
    target_link_libraries(popf-cplex    
        popfCommon
        ${ILOCPLEX} ${CPLEX} ${CONCERT} pthread
    )
    install(TARGETS popf-cplex
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    

endif(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES)


