cmake_minimum_required(VERSION 2.4)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(colinroot)

set(CMAKE_CXX_FLAGS_PROFILE "-pg -O -g")
set(CMAKE_CXX_LINK_FLAGS_PROFILE "-pg -g")


set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

add_subdirectory(VALfiles)
add_subdirectory(popf2)
