cmake_minimum_required(VERSION 3.0)
project(RapidFDM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(build_dir "${CMAKE_CURRENT_SOURCE_DIR}/build/")
#set(LCM_LIBRARIES /usr/local/Cellar/lcm/1.3.0/lib/liblcm.dylib)
#include_directories(./lcm/)

add_subdirectory(modules/aerodynamics)
add_subdirectory(modules/simulation)
add_subdirectory(modules/control_system)
set(SOURCE_LISTS include/RapidFDM/utils.h ./main.cpp)

add_executable(test_main ${SOURCE_LISTS})
