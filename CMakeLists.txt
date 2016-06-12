cmake_minimum_required(VERSION 3.0)
project(RapidFDM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(build_dir "${CMAKE_CURRENT_SOURCE_DIR}/build/")
set(PHYSX_LIBRARIES_DIR ./3rdparty/PhysX-3.3/PhysXSDK/Lib/osx64 )
#set(LCM_LIBRARIES /usr/local/Cellar/lcm/1.3.0/lib/liblcm.dylib)
#include_directories(./lcm/)
find_library(PHYSX_LIBRARIRS NAMES LowLevel PhysX3 libPhysX3Extensions libPhysX3Vehicle
        HINTS ${PHYSX_LIBRARIES_DIR} )

add_subdirectory(modules/aerodynamics)
add_subdirectory(modules/control_system)
add_subdirectory(modules/simulation)
set(SOURCE_LISTS include/RapidFDM/utils.h ./main.cpp)

add_executable(test_main ${SOURCE_LISTS})
