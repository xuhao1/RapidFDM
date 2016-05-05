cmake_minimum_required(VERSION 3.5)
project(RapidFDM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(./modules/ ./include ./3rdparty/eigen3 ./3rdparty/rapidjson)
include(modules/aerodynamics/CMakeLists.txt)

set(SOURCE_LISTS ./include/utils.h ./main.cpp)
add_executable(test_main ${SOURCE_LISTS})
