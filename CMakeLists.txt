cmake_minimum_required(VERSION 3.0)
project(RapidFDM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED
        COMPONENTS filesystem
        )

add_subdirectory(modules/aerodynamics)
set(SOURCE_LISTS ./include/utils.h ./main.cpp)

add_executable(test_main ${SOURCE_LISTS})
