cmake_minimum_required(VERSION 3.5)
project(RapidFDM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(./modules/ ./include ./3rdparty/eigen3)
include(./modules/airdynamics/CMakeLists.txt)

