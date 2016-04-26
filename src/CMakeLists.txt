cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(root)

add_subdirectory(VALfiles)
add_subdirectory(popf)
