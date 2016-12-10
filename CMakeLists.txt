cmake_minimum_required(VERSION 3.0)
project(RapidFDM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(build_dir "${CMAKE_CURRENT_SOURCE_DIR}/build/")
if (WIN32)
set(PHYSX_LIBRARIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/PhysX-3.3/PhysXSDK/Lib/vc14win64)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif (WIN32)
if (APPLE)
set(PHYSX_LIBRARIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/PhysX-3.3/PhysXSDK/Lib/osx64 )
endif (APPLE)
LINK_DIRECTORIES(${PHYSX_LIBRARIES_DIR} )

find_package(Boost REQUIRED
        COMPONENTS filesystem system)
include_directories(${Boost_INCLUDE_DIRS})
set(PHYSX_LIBRARIES
        PhysX3
        LowLevel
        PhysX3Extensions
        PhysX3Common
		PhysXProfileSDK
        ${PHYSX_LIBRARIES_DIR}
	)

#message(${PHYSX_LIBRARIES})
link_directories(
${Boost_LIBRARY_DIRS} 
3rdparty/rapidjson/include
)
add_subdirectory(modules/network_protocol)
add_subdirectory(modules/aerodynamics)
add_subdirectory(modules/control_system)
add_subdirectory(modules/simulation)